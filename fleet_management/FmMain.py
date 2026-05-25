#!/usr/bin/env python3
import logging
logger = logging.getLogger(__name__)


import psycopg2, psycopg2.extras, psycopg2.pool
import time, yaml, re, json, os, time, threading, ast
from pathlib import Path
from FmScheduleHandler import FmScheduleHandler
from paho.mqtt import client as mqtt_client
from paho.mqtt.client import error_string
from jsonschema import validate, ValidationError
from concurrent.futures import ThreadPoolExecutor, as_completed



########################################################################
# Thread-safe connection pool proxy
#
# Wraps a ThreadedConnectionPool and routes every .cursor(), .commit()
# and .rollback() call to a per-thread connection. This lets every
# submodule use self.db_conn.cursor() / self.db_conn.commit() with no
# code changes while being fully safe under a ThreadPoolExecutor.
########################################################################
class ThreadSafeConnectionProxy:
    """Drop-in replacement for a bare psycopg2 connection.

    Wraps a ThreadedConnectionPool.  Each calling thread gets its own
    dedicated connection; that connection is returned to the pool when
    the thread is garbage-collected or the proxy is closed.
    """
    def __init__(self, pool: psycopg2.pool.ThreadedConnectionPool):
        self._pool = pool
        self._local = threading.local()

    def _get_conn(self):
        """Return (or create) the connection that belongs to this thread."""
        if not hasattr(self._local, 'conn') or self._local.conn.closed:
            self._local.conn = self._pool.getconn()
        return self._local.conn

    def cursor(self, *args, **kwargs):
        return self._get_conn().cursor(*args, **kwargs)

    def commit(self):
        return self._get_conn().commit()

    def rollback(self):
        return self._get_conn().rollback()

    def close(self):
        """Return the calling thread's connection back to the pool."""
        if hasattr(self._local, 'conn') and not self._local.conn.closed:
            self._pool.putconn(self._local.conn)
            del self._local.conn

    def shutdown(self):
        """Close the entire pool — call once on application exit."""
        self._pool.closeall()


# get config file path:
# Calculate project root (one level up from fleet_management/)
agv_dir_prefix = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Config is in config/ directory at project root
file_path = Path(os.path.join(agv_dir_prefix, 'config', 'config.yaml'))
########################################################################


class FmMain():
    """ fleet manager main """
    def __init__(self, config_path=file_path):

        # initilaize path to configuration
        mqtt_address, mqtt_port, _, \
        fleetname, fleet_version, fleet_versions, fleet_manufacturer, \
        postgres_host, postgres_port, postgres_database, postgres_user, \
        postgres_password, task_dict, maps = self.extract_yaml_info(config_path)

        self.fleetname = fleetname # "kullar"
        self.version = fleet_version # "2.0.0"
        self.versions = fleet_versions # "v2"
        self.manufacturer = fleet_manufacturer # "birfen"

        self.task_dictionary = task_dict
        self.config_path = config_path

        self.maps = maps

        # establish connection to DB via a ThreadedConnectionPool.
        # minconn=1 keeps one connection warm at idle;
        # maxconn=32 caps parallel connections (tune to Postgres max_connections).
        self.conn = None
        max_retries = 10
        for i in range(max_retries):
            try:
                pool = psycopg2.pool.ThreadedConnectionPool(
                    minconn=1,
                    maxconn=32,
                    host=postgres_host,
                    dbname=postgres_database,
                    user=postgres_user,
                    password=postgres_password,
                    port=postgres_port
                )
                if pool:
                    self.conn = ThreadSafeConnectionProxy(pool)
                    logger.debug(f"Connected to database {postgres_database} at {postgres_host}")
                    break
            except (psycopg2.OperationalError, psycopg2.ProgrammingError) as e:
                logger.debug(f"Attempt {i+1}/{max_retries}: Failed to connect to PostgreSQL. Retrying in 3s... ({e})")
                time.sleep(3)

        if not self.conn:
            logger.debug("Failed to connect to PostgreSQL database after multiple attempts.")

        # MQTT Client placeholder ('localhost', 1883)
        self.mqtt_client = self.creat_mqtt_instance() # Define the MQTT client

        # Initialize the FmTrafficHandler
        self.schedule_handler = FmScheduleHandler(
            self.fleetname,
            self.version,
            self.versions,
            self.manufacturer,
            self.conn,
            self.mqtt_client,
            self.task_dictionary)

        # connect mqtt
        self.mqtt_connect(mqtt_address, mqtt_port)

        # initialize empty fleet and robot ids.
        self.fleetnames = None
        self.serial_numbers = None
        self.job_ids = [] # initialize job ids dropdown menu

        # Thread pool for parallel per-robot management.
        # Size is capped at 32 to avoid overwhelming the DB connection;
        # re-created when serial_numbers changes (see main_loop).
        self._executor = None

        # terminal interaction: initialize fleet dropdown menu
        self.decision = ['yes', 'no']
        self.job_types =['transport','move', 'charge']
        self.job_priorites = ['low', 'high', 'medium']
        self.station_type = ['door', 'elevator', 'checkpoint', 'charge_dock', 'station_dock', 'home_dock', 'waitpoint']

        # Default interval for the timer in seconds
        self.terminal_gui_refresh_interval = 70.0
        self.timer_interval = 70.0

        # initialization for thread
        self.timer_thread = None
        self.timer_thread_running = threading.Event()
        self.lock = threading.RLock()  # Lock to ensure one process finishes before another starts

        # Do NOT auto-start any loop here.
        # When run directly (FmMain is source): __name__=="__main__" block below starts it.
        # When imported by FmInterface: FmInterface owns the single thread and calls run_cycle().

    # Define the function to start the main loop thread
    def start_main_loop_thread(self):
        """  Main loop runs wether accessed from terminal or api to manage the robot and fleet operations. """
        self.main_loop_thread = threading.Thread(target=self.main_loop)
        # self.main_loop_thread.daemon = True
        self.main_loop_thread.start()

    def start_timer_thread(self):
        """Starts the timer thread if it is not already running."""
        # If a timer thread is already running, stop it
        if self.timer_thread is not None and self.timer_thread.is_alive():
            self.timer_thread_running.clear()
            self.timer_thread.join()

        self.timer_thread_running.set()
        # self.timer_thread = threading.Thread(target=self.start_timer, args=(robot_id,), daemon=True)
        self.timer_thread = threading.Thread(target=self.start_timer)
        # self.main_loop_thread.daemon = True
        self.timer_thread.start()

    def stop_timer_thread(self):
        """Stops the timer thread if it is running."""
        if self.timer_thread is not None and self.timer_thread.is_alive():
            self.timer_thread_running.clear()
            self.timer_thread.join()
            # log viz:
            logger.critical("timer thread stopped.")

    def start_timer(self):
        """ timer logic for interactive terminal gui. """
        i = 0
        while self.timer_thread_running.is_set(): # while True:
            i += 1
            if i % self.terminal_gui_refresh_interval == 0:
                self.interactive_robot_fleet_startup()
                if i > (2 * self.terminal_gui_refresh_interval):
                    i = 0

    def run_cycle(self):
        """
        Execute one robot-management cycle for the entire fleet.

        This is the core logic that was previously embedded inside
        main_loop's while-True body.  Extracting it here allows two
        distinct callers:

          - main_loop()     → wraps it in its own while-True (FmMain standalone)
          - FmInterface     → calls it directly inside the scenario loop
                              so both robot-management AND task-dispatch
                              happen inside a single thread.
        """
        if not (self.fleetnames and self.serial_numbers and self.fleetname != ''):
            # Log viz to confirm why it's skipping
            # self.schedule_handler.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
            #     f"FmMain.run_cycle skipping: fleetnames={self.fleetnames}, serials={self.serial_numbers}, current_fleet={self.fleetname}",
            #     "FmMain", "run_cycle", "info")
            return  # not ready yet; caller's loop will retry

        with self.lock:
            time.sleep(4.0)
            for r_id in self.serial_numbers:
                self.schedule_handler.manage_robot(self.fleetname, r_id, self.manufacturer, self.version)

            # Issue terminal traffic control summary once per cycle
            traffic_dict = getattr(self.schedule_handler.traffic_handler, 'traffic_control_dict', None)
            if traffic_dict:
                colored_traffic = ", ".join([f"\033[96m{r}\033[0m: \033[93m{n}\033[0m" for r, n in traffic_dict.items()]) if isinstance(traffic_dict, dict) else traffic_dict
                logger.critical(f"Traffic Control: {{{colored_traffic}}}.")

            # Issue terminal redraw once per cycle
            self.schedule_handler.traffic_handler.task_handler.visualization_handler.terminal_graph_visualization()

    def main_loop(self):
        """
        Standalone fleet-management loop (used when FmMain is the source).
        Calls run_cycle() on every timer tick.
        """
        try:
            i = 0
            while True:
                i += 1
                if i % self.timer_interval == 0:
                    self.run_cycle()
        except KeyboardInterrupt:
            # log viz:
            logger.critical("Exiting program...")
            self.cleanup()

    def cleanup(self):
        """Clean up resources before exiting."""
        self.stop_timer_thread()
        if self._executor is not None:
            self._executor.shutdown(wait=True)
        if self.conn is not None:
            self.conn.shutdown()  # return all pool connections
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        # log viz:
        logger.critical("Cleaned up resources. Program exited.")


    def process_itinerary(self, itinerary, f_id):
        """
        Process the itinerary to update job_ids and landmark_dict based on fleet_name.
        """
        job_ids = []
        for task in itinerary:
            if task["fleet_id"] == f_id:
                description = task["description"]
                loc_id = task["loc_id"]
                if description in {'station_dock', 'charge_dock', 'elevator_dock', 'home_dock'}:
                    job_ids.append(loc_id)
        return job_ids


    def get_valid_input(self, prompt, valid_options):
        """
        Prompt user for input until a valid option is provided.
        """
        user_input = input(prompt)
        while user_input not in valid_options:
            # log viz:
            logger.critical(f'Invalid input. Please enter one of the following: {valid_options}.')
            user_input = input(prompt)
        return user_input


    def interactive_robot_fleet_startup(self):
        """
        interactive terminal user interface.
        """

        # request factsheet from all robots.
        self.schedule_handler.fm_send_factsheet_request(self.manufacturer, self.version)

        # set the fleet_id and robot_id
        self.fleetnames = self.schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_fleets()
        # log viz:
        logger.critical(f"Available Fleets: {self.fleetnames}")

        if len(self.fleetnames) == 0:
            time.sleep(5.0)
            return

        f_id_ = self.get_valid_input(
            f'Please enter fleet_name {self.fleetnames}: \n',
            self.fleetnames
        )

        self.fleetname = f_id_

        stat = self.upload_all_maps(f_id_)
        if not stat:
            return

        self.job_ids = self.process_itinerary(self.task_dictionary.get("itinerary", []), f_id_)
        # log viz:
        logger.critical(f"Valid landmark IDs that can be used for jobs are: \n[dock_ids]: {self.job_ids}. \n")

        self.serial_numbers = self.schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_serial_numbers(f_id_)

        # Define the options
        options = [
            'fm_send_task_request',
            'fm_pause_resume_task_trigger',
            'fm_cancel_task_trigger',
            'fm_ignore_robot_trigger',
            'fm_add_landmark_request',
            'fm_delete_landmark_request',
            'fm_analytics_trigger',
            'fm_add_mutex_groups',
            'fm_remove_mutex_group',
            'exit'
            ]

        # choice = 4 # example
        # show the options
        for i, option in enumerate(options, 1):
            # log viz:
            logger.critical(f'{i}. {option}')

        # Ask for and validate the user's choice
        choice = input('Please enter the number of your choice: \n')
        while not choice.isdigit() or not 1 <= int(choice) <= len(options):
            # log viz:
            logger.critical(f'Invalid input. Please enter a number between 1 and {len(options)}')
            choice = input('Please enter the number of your choice: ')
        # Get the chosen option
        chosen_option = options[int(choice) - 1]

        if chosen_option == 'fm_send_task_request':

            # show only the appropriate robots.
            r_id_ = self.get_valid_input(
                f'Please enter int serial_number: {self.serial_numbers}. \n',
                self.serial_numbers)

            # Define the valid options
            valid_loc_id_pattern = re.compile(r'^[CW]\d+$')

            # Ask for and validate from_loc_id
            # from_loc_id = "A3" # example
            from_loc_id = input('Please enter from_loc_id [options must conform to the pattern C<number> or W<number>]: \n')
            while not valid_loc_id_pattern.match(from_loc_id) or from_loc_id not in self.job_ids:
                # log viz:
                logger.critical(f'Invalid input. It does not conform to the C<number> or W<number> pattern or not in {self.job_ids}')
                from_loc_id = input('Please enter from_loc_id [options must conform to the pattern C<number> or W<number>] or exit: ')
                if from_loc_id == 'exit':
                    return

            # Ask for and validate to_loc_id
            # to_loc_id = "A5" # example
            to_loc_id = input('Please enter to_loc_id [options must conform to the pattern C<number> or W<number>]: \n')
            while not valid_loc_id_pattern.match(to_loc_id) or to_loc_id not in self.job_ids:
                # log viz:
                logger.critical(f'Invalid input. It does not conform to the C<number> or W<number> pattern or not in {self.job_ids}')
                to_loc_id = input('Please enter to_loc_id [options must conform to the pattern C<number> or W<number>] or exit: ')
                if to_loc_id == 'exit':
                    return

            # Ask for and validate task_name
            # task_name = "transport" # example
            task_name = input('Please enter task_name [move, charge, transport]: \n')
            while task_name not in self.job_types:
                # log viz:
                logger.critical('Invalid input. The task name must be one of the following: move, charge, transport.')
                task_name = input('Please enter task_name [move, charge, transport] or exit: ')
                if task_name == 'exit':
                    return

            # Ask for and validate task_priority
            # task_priority = "low" # example
            task_priority = input('Please enter task_priority [low, high, medium]: \n')
            while task_priority not in self.job_priorites:
                # log viz:
                logger.critical('Invalid input. The task priority must be one of the following: low, high, medium.')
                task_priority = input('Please enter task_priority [low, high, medium] or exit: ')
                if task_priority == 'exit':
                    return

            # Define a pattern for integer numbers (one or more digits)
            valid_payload_pattern = re.compile(r'^\d+$')  # Allows any positive integer
            payload_size = input('Please enter payload_size [must be a positive integer]: \n')
            while not valid_payload_pattern.match(payload_size):
                # log viz:
                logger.critical('Invalid input. Payload size must be a positive integer without symbols or letters.')
                payload_size = input('Please enter payload_size [must be a positive integer] or exit: ')
                if payload_size == 'exit':
                    return

            # Now you have valid inputs and you can proceed with your task
            # log viz:
            logger.critical(f'from_loc_id: {from_loc_id}, to_loc_id: {to_loc_id}, task_name: {task_name}, task_priority: {task_priority}')

            # answer = "yes"
            answer = input('WARNING! Are you sure you want to continue with robot task? [yes, no]: \n')
            while answer not in self.decision:
                # log viz:
                logger.critical('Invalid input. The decision must be one of the following: yes, no.')
                answer = input('Please enter decision [yes, no]: ')
            if answer == 'no':
                return
            else:
                task_cleared, available_robots, checkpoints, \
                    agv_itinerary, waitpoints, \
                        wait_itinerary, landmark = \
                            self.schedule_handler.traffic_handler.task_handler.fm_send_task_request(
                                f_id=f_id_,
                                r_id=r_id_,
                                from_loc_id= from_loc_id,
                                to_loc_id= to_loc_id,
                                task_name=task_name,
                                task_priority=task_priority,
                                payload_kg=int(payload_size))

        elif chosen_option == 'fm_pause_resume_task_trigger':
            # show only the appropriate robots.
            r_id_ = self.get_valid_input(
                f'Please enter int serial_number: {self.serial_numbers}. \n',
                self.serial_numbers)
            # request user input.
            answer = input('WARNING! Are you sure you want to pause task? choose no to resume or ignore. [yes, no]: \n')
            while answer not in self.decision:
                # log viz:
                logger.critical('Invalid input. The decision must be one of the following: yes to pause, no to resume or exit choice.')
                answer = input('Please enter decision [yes, no]: ')
            # Convert the pause_list to a set to ensure uniqueness
            self.schedule_handler.traffic_handler.task_handler.pause_list = set(self.schedule_handler.traffic_handler.task_handler.pause_list)
            if answer == 'yes':
                self.schedule_handler.traffic_handler.task_handler.pause_list.add(r_id_)
            elif answer == 'no':
                self.schedule_handler.traffic_handler.task_handler.pause_list.remove(r_id_)

        elif chosen_option == 'fm_cancel_task_trigger':
            # show only the appropriate robots.
            r_id_ = self.get_valid_input(
                f'Please enter int serial_number: {self.serial_numbers}. \n',
                self.serial_numbers)
            # request user input.
            answer = input('WARNING! Are you sure you want to cancel task? move robot home. [yes, no]: \n')
            while answer not in self.decision:
                # log viz:
                logger.critical('Invalid input. The decision must be one of the following: yes to cancel, no to exit choice.')
                answer = input('Please enter decision [yes, no]: ')
            if answer == 'yes':
                # cancelOrder | - | AGV is stopping or driving, until it reaches the next node.
                #             | - | AGV stands still and has canceled the order.
                self.schedule_handler.cancel_task(self, f_id_, r_id_, self.manufacturer, self.version)

        elif chosen_option == 'fm_ignore_robot_trigger':
            # show only the appropriate robots.
            r_id_ = self.get_valid_input(
                f'Please enter int serial_number: {self.serial_numbers}. \n',
                self.serial_numbers)
            # request user input.
            answer = input('WARNING! Are you sure you want the manager to ignore this robot? please ensure robot not in any landmark. [yes, no]: \n')
            while answer not in self.decision:
                # log viz:
                logger.critical('Invalid input. The decision must be one of the following: yes to add robot to the ignore list, \
                    no to remove robot from list or exit choice.')
                answer = input('Please enter decision [yes, no]: ')
            # Convert the pause_list to a set to ensure uniqueness
            self.schedule_handler.traffic_handler.task_handler.ignore_list = set(self.schedule_handler.traffic_handler.task_handler.ignore_list)
            self.schedule_handler.traffic_handler.task_handler.cp_ignore_list = set(self.schedule_handler.traffic_handler.task_handler.cp_ignore_list)
            if answer == 'yes':
                self.schedule_handler.traffic_handler.task_handler.ignore_list.add(r_id_)
                self.schedule_handler.traffic_handler.task_handler.cp_ignore_list.add(r_id_) # copy of ignore list
            elif answer == 'no':
                self.schedule_handler.traffic_handler.task_handler.ignore_list.remove(r_id_)


        elif chosen_option == 'fm_add_landmark_request':
            # Ask for and validate station_type
            station_type = input('Please enter station_type from: '+str(self.station_type)+'.')
            while station_type not in self.station_type:
                # log viz:
                logger.critical(f'Invalid input. The station_type must be one of the following: {self.station_type}.')
                station_type = input('Please enter station_type : '+str(self.station_type)+'. or exit: ')
                if station_type == 'exit':
                    return

            # this part expects that fleet, maps and their respective paths have been set in the config.yaml
            map_name = [] # ["bina_1_floor_0"]
            records = self.schedule_handler.traffic_handler.task_handler.visualization_handler.fetch_all_data(f_id_)
            if records:
                for map_ in records[0]:
                    map_name.append(map_["map_name"])

            if not map_name:
                # log viz:
                logger.critical('No map found. This part expects that fleet_id, maps and their respective .yaml and .pgm paths \
                        have been set in the config.yaml.')
                return

            map_resp = input('Which map in fleet '+str(f_id_)+' does this node belong? Please enter map_name from: '+str(map_name)+'.')
            while map_resp not in map_name:
                # log viz:
                logger.critical(f'Invalid input. The map response must be one of the following: {map_name}.')
                map_resp = input('Please enter map response : '+str(map_name)+'. or exit: ')
                if map_resp == 'exit':
                    return

            # Define the valid options
            valid_loc_id_pattern = re.compile(r'^[CW]\d+$')
            # Ask for and validate from_loc_id
            input_loc_id = input('Please enter valid location/landmark id [options must conform to the pattern C<number> or W<number>]: ')
            while not valid_loc_id_pattern.match(input_loc_id):
                # log viz:
                logger.critical('Invalid input. It does not conform to the C<number> or W<number> pattern.')
                input_loc_id = input('Please enter input_loc_id [options must conform to the pattern C<number> or W<number>] or exit: ')
                if input_loc_id == 'exit':
                    return

            # Define the valid pattern
            valid_neihbour_id_pattern = re.compile(r'^[CW]\d+(,[CW]\d+)*$')

            # Ask for and validate neighbor_loc_ids sample input_str = "C4,W2,C3,W6"
            neighbor_loc_ids = input('Neighbors cannot be left empty. Please enter valid connected location/landmark ids [options must conform to the pattern C<number> or W<number>, comma separated]: ')
            while not valid_neihbour_id_pattern.match(neighbor_loc_ids.replace(" ","")):
                # log viz:
                logger.critical('Invalid input. It does not conform to the C<number> or W<number> comma-separated pattern.')
                neighbor_loc_ids = input('Please enter neighbor_loc_ids [options must conform to the pattern C<number> or W<number>, comma separated] or exit: ')
                if neighbor_loc_ids == 'exit':
                    return

            # Ask for and validate loc_pose (list [x, y, w, z])
            while True:
                try:
                    loc_pose_input = input("Enter loc_pose as [x, y, w, z] (e.g., [1.0, 2.0, 0.0, 1.0]): ")
                    loc_pose = ast.literal_eval(loc_pose_input)  # Convert string input to a Python list
                    if (
                        isinstance(loc_pose, list) and len(loc_pose) == 4
                        and all(isinstance(coord, (int, float)) for coord in loc_pose)
                    ):
                        break  # Input is valid
                    else:
                        raise ValueError
                except Exception:
                    # log viz:
                    logger.critical("Invalid input. loc_pose must be a list of four numbers [x, y, w, z]. Try again or enter 'exit'.")
                    if loc_pose_input.lower() == 'exit':
                        return

            self.schedule_handler.traffic_handler.task_handler.visualization_handler.fm_add_landmark_request(
                f_id_, map_resp, station_type, loc_pose, input_loc_id, neighbor_loc_ids, self.config_path)

        elif chosen_option == 'fm_delete_landmark_request':
            # Define the valid pattern
            valid_pattern = re.compile(r'^[CW]\d+$')
            # Ask for and validate the input
            loc_id = input('Please enter a string of the form C<number> or W<number>: ')
            while not valid_pattern.match(loc_id):
                # log viz:
                logger.critical('Invalid input. It does not conform to the C<number> or W<number> pattern.')
                loc_id = input('Please enter a string of the form C<number> or W<number> or exit: ')
                if loc_id == 'exit':
                    return

            self.schedule_handler.traffic_handler.task_handler.visualization_handler.fm_delete_landmark_request(
                f_id_, loc_id, self.config_path)

        elif chosen_option == 'fm_analytics_trigger':
            # analytics
            self.schedule_handler.fm_analytics(f_id_, self.manufacturer, r_id=None, debug_level='info', write_to_file=True)

        elif chosen_option == 'fm_add_mutex_groups':
            while True:
                try:
                    group_input = input("Enter Mutex Group as a list of nodes (e.g., ['C1', 'C2', 'C27']): ")
                    group_list = ast.literal_eval(group_input)
                    if isinstance(group_list, list) and all(isinstance(n, str) for n in group_list):
                        self.schedule_handler.traffic_handler.fm_add_mutex_groups(group_list)
                        break
                    else:
                        raise ValueError
                except Exception:
                    logger.critical("Invalid input. Mutex Group must be a list of strings [node1, node2, ...]. Try again or enter 'exit'.")
                    if group_input.lower() == 'exit':
                        break

        elif chosen_option == 'fm_remove_mutex_group':
            current_groups = self.schedule_handler.traffic_handler.mutex_groups
            if not current_groups:
                 logger.info("No mutex groups currently defined.")
            else:
                for i, group in enumerate(current_groups, 1):
                    logger.debug(f"{i}. {group}")
                choice = input("Enter the number of the group to remove: ")
                if choice.isdigit() and 1 <= int(choice) <= len(current_groups):
                    removed_group = current_groups[int(choice) - 1]
                    self.schedule_handler.traffic_handler.fm_remove_mutex_group(removed_group)

        elif chosen_option == 'exit':
            r_id_ = 0
            # log viz:
            logger.critical('exiting menu. \nAll of this was written and designed by Hazeezadebayo. \n \
                                                                         Thank you.')
            return

    def fm_dispatch_task(self,
                        fleet_id='kullar',
                        robot_id=None,
                        from_loc=None,
                        to_loc=None,
                        task_name='transport',
                        priority='low',
                        payload=0):
            """
            SIMULATION FUNCTION:
            Call this from main to skip the interactive form filling.
            It sets up the necessary environment (maps, ids) and sends the request.
            """

            # Guard: callers must always provide from_loc and to_loc explicitly.
            # The old default 'A12' was a dev placeholder and caused spurious
            # "not found in valid job_ids" warnings on every automated dispatch call.
            if from_loc is None or to_loc is None:
                raise ValueError(
                    f"fm_dispatch_task: from_loc and to_loc are required. "
                    f"Got from_loc={from_loc!r}, to_loc={to_loc!r}."
                )

            # 1. Request Factsheets (Wait briefly for MQTT response to populate)
            self.schedule_handler.fm_send_factsheet_request(self.manufacturer, self.version)
            time.sleep(2.0) # Give MQTT time to receive factsheets

            # 2. Set Fleet Name and Fetch Fleets
            self.fleetnames = self.schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_fleets()

            # Validate Fleet
            if not self.fleetnames or fleet_id not in self.fleetnames:
                # log viz:
                logger.critical(f"Error: Fleet {fleet_id} not found in available fleets: {self.fleetnames}")

            self.fleetname = fleet_id

            # 3. Upload Maps
            stat = self.upload_all_maps(fleet_id)
            if not stat:
                # log viz:
                logger.critical("Error: Map upload failed.")
                return

            # 4. Process Job IDs (Landmarks)
            self.job_ids = self.process_itinerary(self.task_dictionary.get("itinerary", []), fleet_id)

            # 5. Fetch Serial Numbers
            self.serial_numbers = self.schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_serial_numbers(fleet_id)

            # Validate Locations
            if (from_loc not in self.job_ids) or (to_loc not in self.job_ids) or (robot_id is None):
                logger.debug(f"[FmMain] Warning: Locations {from_loc} or {to_loc} not found in valid job_ids: {self.job_ids}")
                return

            # log viz:
            logger.critical(f"Sending {robot_id} request: {from_loc} -> {to_loc} ({task_name})")

            # 6. Send the Task Request
            task_cleared, available_robots, checkpoints, \
                agv_itinerary, waitpoints, \
                    wait_itinerary, landmark = \
                self.schedule_handler.traffic_handler.task_handler.fm_send_task_request(
                    f_id=fleet_id,
                    r_id=robot_id,
                    from_loc_id=from_loc,
                    to_loc_id=to_loc,
                    task_name=task_name,
                    task_priority=priority,
                    payload_kg=int(payload)
                )


    def extract_yaml_info(self, config_path):
        """ extract yaml info """
        if config_path.is_file():
            with open(config_path, "r", encoding="utf-8") as file:
                config = yaml.safe_load(file)

            # Extract sections and assign to respective variables
            mqtt_broker_address = str(config['mqtt']['broker_address'])
            mqtt_broker_port = str(config['mqtt']['broker_port'])
            mqtt_keep_alive = str(config['mqtt']['keep_alive'])

            fleetname = str(config['fleet_info']['fleetname'])
            fleet_version = str(config['fleet_info']['version'])
            fleet_versions = str(config['fleet_info']['versions'])
            fleet_manufacturer = str(config['fleet_info']['manufacturer'])

            postgres_host = str(config['postgres']['host'])
            postgres_port = str(config['postgres']['port'])
            postgres_database = str(config['postgres']['database'])
            postgres_user = str(config['postgres']['user'])
            postgres_password = str(config['postgres']['password'])

            task_dict = {"itinerary": config["itinerary"],
                               "graph": config["graph"]}

            maps = config["maps"]

            # Return all variables
            return (mqtt_broker_address, mqtt_broker_port, mqtt_keep_alive,
                    fleetname, fleet_version, fleet_versions, fleet_manufacturer,
                    postgres_host, postgres_port, postgres_database, postgres_user,
                    postgres_password, task_dict, maps)

        else:
            # log viz:
            logger.critical('agv configuration file missing.')
            exit(0)

    def upload_all_maps(self, f_id):
        """
        Upload all maps from a YAML configuration file to the database.

        Args:
        - f_id (str): The fleet ID to associate with the maps.
        """
        if f_id in self.maps:
            for map_info in self.maps[f_id]:
                # get pgm and yaml file path
                pgm_file_path = map_info['map_pgm_path']
                yaml_file_path = map_info['map_yaml_path']
                # Check if PGM and YAML files exist
                if (not os.path.isfile(pgm_file_path)) or (not os.path.isfile(yaml_file_path)):
                    # Log viz:
                    logger.critical(f"PGM: {pgm_file_path}, or YAML: {yaml_file_path} file does not exist.")
                    return False
                # Create the message dictionary to pass to the insert function
                msg = {
                    'fleet_name': f_id,
                    'map_name': map_info['map_id'],
                    'pgm_file_path': pgm_file_path,
                    'yaml_file_path': yaml_file_path
                }
                # Call the insert function
                self.schedule_handler.traffic_handler.task_handler.visualization_handler.insert_maps_db(msg)
            return True
        else:
            # Log viz:
            logger.critical(f"No maps found for fleet_id {f_id}.")
        return False

# ----------------------------------------------------------------------- #
#               MQTT CLIENT SUBSCRIBER                                    #
# ----------------------------------------------------------------------- #

    def creat_mqtt_instance(self):
        """ create new mqtt instance """
        mqttclient = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2) # VERSION1
        return mqttclient

    def mqtt_connect(self, mqtt_address, mqtt_port):
        """ connect """
        # Set the callback function
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        # Connect to the MQTT broker
        self.mqtt_client.connect(host=str(mqtt_address), port=int(mqtt_port)) # keep alive for 60secs
        self.mqtt_client.loop_start()
        # log viz:
        logger.critical('connection done.')

    def on_mqtt_connect(self, client, userdata, flags, rc, *extra_args):
        """Callback for when the client receives a connection acknowledgment from the broker."""
        if rc == 0:
            # Subscribe to the topic once connected
            self.schedule_handler.traffic_handler.task_handler.connection_handler.subscribe(self.mqtt_client)
            self.schedule_handler.traffic_handler.task_handler.factsheet_handler.subscribe(self.mqtt_client)
            self.schedule_handler.traffic_handler.task_handler.state_handler.subscribe(self.mqtt_client)
        else:
            # log viz:
            logger.critical(f'Disconnected from MQTT Broker! with client: {client}, and userdata: {userdata}.')
        # log viz:
        logger.critical(f'Connected to MQTT broker with result code {rc}.')

    def on_mqtt_message(self, client, userdata, msg):
        """ on mqtt message """
        payload = msg.payload.decode()
        try:
            message = json.loads(payload)
            if "connection" in msg.topic:
                self.schedule_handler.traffic_handler.task_handler.connection_handler.process_message(message)
            elif "factsheet" in msg.topic:
                self.schedule_handler.traffic_handler.task_handler.factsheet_handler.process_message(message)
            elif "state" in msg.topic:
                self.schedule_handler.traffic_handler.task_handler.state_handler.process_message(message)
                r_id = message.get("serialNumber")
                if r_id:
                    self.schedule_handler.traffic_handler.online_robots.add(r_id)
        except json.JSONDecodeError as e:
            # log viz:
            logger.critical(f'Failed to decode JSON message: {e}.')
        except ValidationError as er:
            # log viz:
            logger.critical(f'validation failed: {er.message}.')

    # def on_mqtt_disconnect(self, client, userdata, rc):
    def on_mqtt_disconnect(self, client, userdata, rc, properties=None, reason_code=None):
        """MQTT client disconnect callback."""
        if rc != 0:
            # log viz:
            logger.critical(f"Disconnected (rc: {rc}, {error_string(rc)}). Trying to reconnect.")
            while not self.mqtt_client.is_connected():
                try:
                    self.mqtt_client.reconnect()
                except OSError:
                    pass
        else:
            # log viz:
            logger.critical(f'Disconnected from MQTT Broker! with client: {client}, and userdata: {userdata}.')

# ---------------------------------------------- #
#      MAIN                                      #
# ---------------------------------------------- #

if __name__ == "__main__":
    fm = FmMain()
    # FmMain is the source: start the interactive timer GUI in a background
    # thread, then run the management loop on the main thread.
    fm.start_timer_thread()
    fm.main_loop()




# Throughput in our context is essentially the rate at which the central manager processes MQTT messages coming from various robots. In other words, it’s how many messages (or tasks) the manager can handle per unit time. This is affected not only by how fast the manager’s code runs but also by communication delays (network latency), CPU/memory load, and any processing overhead introduced by the message-handling routines.

# ### Measuring Throughput & Communication Overhead

# 1. **Message Timestamps & Delays:**
#    - **Send vs. Receive Timestamps:**
#      If each MQTT message carries a timestamp indicating when it was sent (for example, a field like `"sent_time"`), you can compare that to the time when the message is processed (using, say, `time.time()`) in your `on_mqtt_message` callback. This difference gives you the communication delay.

#    - **Processing Delay:**
#      You can also record the time when the manager finishes processing a message. The difference between the arrival time and the processing-complete time gives you the processing overhead.

# 2. **Throughput Calculation:**
#    - Count the number of messages processed over a given time window (e.g., per second or per minute).
#    - Keep a log of arrival timestamps so you can compute a moving average of messages per second.

# 3. **Missing Messages / Sequence Gaps:**
#    - Include a sequence number in every message. By tracking these sequence numbers, you can detect if a message is missed (if a sequence jump occurs) or if the manager is lagging behind.

# 4. **System Load (CPU/Memory):**
#    - Use libraries such as [psutil](https://psutil.readthedocs.io/) to record the CPU and memory usage of your manager process. High CPU load or memory pressure might correlate with increased message delays.
#    - By correlating these metrics with your message delay data, you can gain insights into whether the manager is overloaded.

# ### Example Integration in Your MQTT Callbacks

# Below is a modified snippet of your MQTT callbacks that demonstrates how you might record these metrics:

# ---

# ```python
# import time
# import json
# import psutil  # For CPU/memory metrics

# class ManagerPerformanceMonitor:
#     def __init__(self):
#         # Stores delays for each message, e.g., {"connection": [delay1, delay2, ...], ...}
#         self.message_delays = {"connection": [], "factsheet": [], "state": []}
#         self.message_counts = {"connection": 0, "factsheet": 0, "state": 0}
#         self.last_sequence = {}  # To track sequence numbers per topic

#     def record_message(self, topic, sent_time, received_time, sequence_number=None):
#         delay = received_time - sent_time
#         self.message_delays[topic].append(delay)
#         self.message_counts[topic] += 1

#         # Check for missed messages using sequence numbers (if provided)
#         if sequence_number is not None:
#             last_seq = self.last_sequence.get(topic)
#             if last_seq is not None and sequence_number != last_seq + 1:
#                 print(f"[Warning] Gap detected in topic '{topic}': last_seq={last_seq}, current_seq={sequence_number}")
#             self.last_sequence[topic] = sequence_number

#     def average_delay(self, topic):
#         delays = self.message_delays.get(topic, [])
#         return sum(delays) / len(delays) if delays else 0

#     def throughput(self, topic, time_window_sec):
#         # Simple approach: Count messages received in the given time window.
#         # In a real implementation, you might want to timestamp each message and then compute dynamically.
#         return self.message_counts.get(topic, 0) / time_window_sec

#     def cpu_memory_usage(self):
#         cpu = psutil.cpu_percent(interval=0.1)
#         mem = psutil.virtual_memory().percent
#         return cpu, mem

# # In your MQTT handler class
# class YourMQTTHandler:
#     def __init__(self, schedule_handler):
#         self.schedule_handler = schedule_handler
#         self.monitor = ManagerPerformanceMonitor()
#         self.logger = schedule_handler.logger  # Assuming your handler has a logger

#     def on_mqtt_connect(self, client, userdata, flags, rc, *extra_args):
#         if rc == 0:
#             self.schedule_handler.traffic_handler.task_handler.connection_handler.subscribe(self.mqtt_client)
#             self.schedule_handler.traffic_handler.task_handler.factsheet_handler.subscribe(self.mqtt_client)
#             self.schedule_handler.traffic_handler.task_handler.state_handler.subscribe(self.mqtt_client)
#         else:
#             self.schedule_handler.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
#                 f'Disconnected from MQTT Broker! Client: {client}, userdata: {userdata}.',
#                 "FmMain", "on_mqtt_connect", "critical")
#         self.schedule_handler.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
#             f'Connected to MQTT broker with result code {rc}.', "FmMain", "on_mqtt_connect", "critical")

#     def on_mqtt_message(self, client, userdata, msg):
#         received_time = time.time()
#         payload = msg.payload.decode()
#         try:
#             message = json.loads(payload)
#             # Assume messages include a 'sent_time' (epoch seconds) and a 'sequence_number'
#             sent_time = message.get('sent_time', received_time)  # Fallback if not provided
#             sequence_number = message.get('sequence_number')

#             # Determine the topic type for our tracking (e.g., "connection", "factsheet", "state")
#             if "connection" in msg.topic:
#                 topic_key = "connection"
#                 self.schedule_handler.traffic_handler.task_handler.connection_handler.process_message(message)
#             elif "factsheet" in msg.topic:
#                 topic_key = "factsheet"
#                 self.schedule_handler.traffic_handler.task_handler.factsheet_handler.process_message(message)
#             elif "state" in msg.topic:
#                 topic_key = "state"
#                 self.schedule_handler.traffic_handler.task_handler.state_handler.process_message(message)
#             else:
#                 topic_key = "unknown"

#             # Record the message delay and check for gaps if a sequence is provided
#             if topic_key in self.monitor.message_delays:
#                 self.monitor.record_message(topic_key, sent_time, received_time, sequence_number)

#             # Optionally, check CPU/Memory usage on each message or at intervals.
#             cpu, mem = self.monitor.cpu_memory_usage()
#             self.logger.info("Current CPU: %.1f%%, Memory: %.1f%%", cpu, mem)

#         except json.JSONDecodeError as e:
#             self.schedule_handler.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
#                 f'Failed to decode JSON message: {e}.', "FmMain", "on_mqtt_message", "critical")
#         except Exception as er:
#             self.schedule_handler.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
#                 f'Error processing message: {er}.', "FmMain", "on_mqtt_message", "critical")
# ```

# ---

# ### Explanation

# - **Throughput:**
#   In this code, we count the number of messages received per topic. You can calculate throughput by dividing the total messages received over a period by that period’s duration. For example, if you count 100 messages in 10 seconds, your throughput is 10 messages per second.

# - **Communication Overhead (Latency):**
#   By having each message include a `sent_time` (ideally in a standardized format such as epoch seconds), you can compare that to the `received_time` (recorded in `on_mqtt_message`). The difference is your network plus any queuing/processing delay. Recording these values lets you compute averages and detect spikes.

# - **Detecting Missed Messages:**
#   Adding a `sequence_number` to each message lets you detect if there is a gap between consecutive messages. A gap may indicate that a message was dropped or the manager couldn’t process messages fast enough.

# - **System Load Monitoring:**
#   Using a library like `psutil`, you can log the CPU and memory usage. High CPU usage (or memory pressure) might correlate with increased message processing delays.

# - **Manager Performance:**
#   By aggregating these metrics—throughput (messages/sec), average message delay, detected missing messages, and CPU/memory usage—you gain a comprehensive view of the manager’s performance. This information can help you identify bottlenecks and decide whether to optimize code, scale horizontally, or adjust message priorities.

# In summary, throughput here is the rate of message processing. You can accurately measure it by combining timestamps, sequence tracking, and system resource monitoring. This multifaceted approach lets you quantify not only the raw processing speed but also the communication overhead and system load, giving you a better understanding of the manager's performance.
