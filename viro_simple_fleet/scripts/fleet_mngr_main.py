#!/usr/bin/env python3

import numpy as np
from collections import defaultdict
import psycopg2, psycopg2.extras, collections
from matplotlib import cm
from PIL import Image
import time, io, cv2, yaml, os, signal, math, json, heapq, re
from twilio.rest import Client
from pathlib import Path
import matplotlib.pyplot as plt
import threading

'''
Assumptions:
1. checkpoints are linked by a straight line
2. checkpoints are at least 3.0m away from one another. i.e. at least the length of agv + some_tolerance.
3. elevator -- naming convention -- E1, E2.. represent elevator so if E is in the alphabet -- we request map change.
'''





# clear task should remove agv_itinerary and wait_itinerary as well
# typing too fast shpuld not give the feedback_current_pose[0] mentioned before declaration or something
# write function/conflict scenario tests



########################################################################
# Vehicle parameters
LENGTH = 1.0# 4.5  # [m]
WIDTH = 0.5 #2.0  # [m]
BACKTOWHEEL = 0.12 #1.0  # [m] increasing shifts chasis forward.
WHEEL_LEN = 0.1 #0.3  # [m] # single tyre length
WHEEL_WIDTH = 0.05 #0.2  # [m] # single tyre width
TREAD = 0.18 #0.7  # [m] # left to right tyre distance
WB = 0.75 #2.5  # [m] front tyre to back tyre distance
MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]
robot_id, fleet_id = 0, 'unknown'
checkpoints, agv_itinerary, landmark, graph, waitpoints, wait_itinerary = [], [], [], {}, [], []
task_cleared, task_dictionary = False, {}
agv_config_dir = (os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
file_path = Path(agv_config_dir+'/viro_simple_fleet/config/fleet_mngr.yaml')
########################################################################


class Ui_MainWindow():
    def __init__(self, config_path=file_path):

        # wether or not to show matplotlib car
        self.single_robot_view = False
        # Turn on interactive mode
        plt.ion()
        # Create a new figure
        self.fig, self.ax = plt.subplots()
        # initialize helper variables
        self.o_q = []
        self.control_command = None
        self.feedback_current_pose = None
        self.feedback_notifications = None
        self.feedback_last_checkpoint = None
        self.feedback_checkpoints = None
        self.feedback_agv_status = None
        self.feedback_shutdown = None
        self.feedback_controls = None
        self.feedback_model_config = None
        self.feedback_landmarks = None
        self.feedback_base_map_img = None
        self.feedback_floor1_map_img = None
        self.feedback_agv_itinerary = None
        self.feedback_traffic = None
        self.feedback_waitpoints = None
        self.feedback_wait_traffic = None

        # temporary feedback variables
        self.temp_fb_current_pose = None
        self.temp_fb_notifications = None
        self.temp_fb_last_checkpoint = None
        self.temp_fb_checkpoints = None
        self.temp_fb_waitpoints = None
        self.temp_fb_agv_status = None
        self.temp_fb_model_config = None
        self.temp_fb_landmarks = None
        self.temp_fb_agv_itinerary = None
        self.temp_fb_traffic = None
        self.temp_fb_wait_traffic = None
        self.temp_fb_base_map_img = None

        # initialize empty fleet and robot ids.
        self.fleet_ids = []
        self.robot_ids = []
        self.job_ids = [] # initialize job ids dropdown menu
        self.send_sms = False
        # initialize fleet dropdown menu
        self.decision = ['yes', 'no'] # terminal interaction {'yes', 'no'}
        self.job_types =['transport', 'loop', 'move', 'charge', 'clean']
        self.job_priorites = ['low', 'high', 'medium']
        self.station_type = ['charge_dock', 'station_dock', 'home_dock', 'elevator_dock', 'checkpoint', 'waitpoint']
        # initialize co-ordinates! this is for plotting on matplotlib
        self.agv_track_dict = {"x":[], "y":[],}
        self.landmark_dict = {"x":[], "y":[], "t":[], "k":[],}
        self.allobs_x, self.allobs_y = [], []
        self.nav_map_obtained = False
        self.min_task_bat_level = 30
        # initilaize path to configuration
        self.file_path = config_path
        self.load_from_yaml()
        # establish connection to DB
        self.conn = psycopg2.connect(host=hostname, dbname=database, user=username, password=pwd, port=port_id)
        self.cur = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)
        # initialize
        global graph
        for node in task_dictionary['graph']:
            neighbors = set(tuple(x) for x in task_dictionary['graph'][node])
            graph[node] = neighbors
        # self.interactive_robot_fleet_startup()
        # Default interval for the timer in seconds
        self.i, self.j = 0, 0
        self.terminal_gui_refresh_interval = 50.0
        self.timer_interval = 5.0

        # initialization for thread
        self.timer_thread = None
        self.timer_thread_running = threading.Event()
        self.lock = threading.Lock()  # Lock to ensure one process finishes before another starts

        # Only start the timer thread if this script is run directly
        if __name__ == "__main__":
            # terminal based gui
            self.show_robot_animation = True
            self.start_timer_thread()
            self.main_loop()
        else:
            self.on_timer_lock = False
            # no animations as matplotlib is not threadsafe.
            self.show_robot_animation = False
            # if API call, then start main loop in its own thread
            self.start_main_loop_thread()


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
            print("Timer thread stopped.")

    def start_timer(self):
        """ timer logic for interactive terminal gui. """
        while self.timer_thread_running.is_set(): # while True:
            self.i += 1
            if self.i % self.terminal_gui_refresh_interval == 0:
                self.interactive_robot_fleet_startup()
                if self.i > 100:
                    self.i = 0

    def main_loop(self):
        """
        Main loop runs wether accessed from terminal or api to manage the robot and fleet operations.
        """
        while True:
            self.j += 1
            # Perform operations every x iterations if fleet and robot IDs are available
            if self.j % self.timer_interval == 0 and self.fleet_ids and self.robot_ids and self.on_timer_lock is False:
                # print("fleet_id: ", fleet_id, "robot_id: ", robot_id)
                # Load robot traffic manager display callback
                with self.lock:  # Ensure no other operation interferes
                    self.on_timer(fleet_id, robot_id)
                # Reset counter to avoid overflow
                if self.j > 100:
                    self.j = 0

##################################################
# ---------------------------------------------- #
#      ON START-UP: LOAD FLEET LIST AND MENU     #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ---------------------------------- update_current_agv  ---------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def load_from_yaml(self):
        """
        [output] task_dictionary, hostname, database, username, pwd, port_id
        """
        global task_dictionary, hostname, database, username, pwd, port_id
        if self.file_path.is_file():
            with open(self.file_path) as yaml_file:
                task_dictionary = yaml.safe_load(yaml_file)
                hostname = str(task_dictionary["hostname"])  #'localhost'
                database = str(task_dictionary["database"])  #'postgres'
                username = str(task_dictionary["username"])  #'postgres'
                pwd = str(task_dictionary["pwd"])            #'root'
                port_id = str(task_dictionary["port"])       #5432
        else:
            print('agv configuration file missing.')
            exit(0)

    def dump_to_yaml(self, data):
        """
        [input] graph data
        [output] new json immediately for use.
        """
        if self.file_path.is_file():
            with open(self.file_path, "w") as outfile:
                yaml.dump(data, outfile)
            self.load_from_yaml() # load new json immediately for use.
        else:
            print('agv configuration file missing.')
            exit(0)

##################################################
# ---------------------------------------------- #
#      ON START-UP: LOAD FLEET LIST AND MENU     #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ---------------------------------- update_current_agv  ---------------------------------------------
# ----------------------------------------------------------------------------------------------------

    def interactive_robot_fleet_startup(self):
        """
        [input] graph data
        [output] new json immediately for use.
        """
        # set the fleet_id and robot_id
        self.interactive_start_robot_fleet()

        if robot_id == 0:
            print("since no particular robot was seleted")
            print("exiting menu \n")
            return

        # Define the options
        options = [
            'fm_shutdown_trigger',
            'fm_robot_status_trigger',
            'fm_manual_control_trigger',
            'fm_send_task_request',
            'fm_clear_task_trigger',
            'fm_add_landmark_request',
            'fm_delete_landmark_request',
            'exit'
            ]
        # choice = 4 # example
        # # Print the options
        for i, option in enumerate(options, 1):
            print(f'{i}. {option}')
        # Ask for and validate the user's choice
        choice = input('Please enter the number of your choice: ')
        while not choice.isdigit() or not 1 <= int(choice) <= len(options):
            print('Invalid input. Please enter a number between 1 and', len(options))
            choice = input('Please enter the number of your choice: ')
        # Get the chosen option
        chosen_option = options[int(choice) - 1]

        # Call the corresponding function
        if chosen_option == 'fm_shutdown_trigger':
            answer = input('Alert! check yes or no for on or off? [yes, no]: ')
            while answer not in self.decision:
                print('Invalid input. The decision must be one of the following: yes, no.')
                answer = input('Please enter decision [yes, no]: ')
            if answer == 'yes':
                self.fm_shutdown_trigger(True)
            else:
                self.fm_shutdown_trigger(False)

        elif chosen_option == 'fm_robot_status_trigger':
            answer = input('Alert! check yes or no for active or inactive? [yes, no]: ')
            while answer not in self.decision:
                print('Invalid input. The decision must be one of the following: yes, no.')
                answer = input('Please enter decision [yes, no]: ')
            if answer == 'yes':
                self.fm_robot_status_trigger(True)
            else:
                self.fm_robot_status_trigger(False)

        elif chosen_option == 'fm_manual_control_trigger':
            answer = input('Alert! check yes or no to nav manually or not? [yes, no]: ')
            while answer not in self.decision:
                print('Invalid input. The decision must be one of the following: yes, no.')
                answer = input('Please enter decision [yes, no]: ')
            if answer == 'yes':
                self.fm_manual_control_trigger(True)
                # Define the options
                options = [
                    'left',
                    'right',
                    'forward',
                    'backward',
                    'stop',
                    'exit'
                ]
                # Print the options
                for i, option in enumerate(options, 1):
                    print(f'{i}. {option}')
                # Ask for and validate the user's choice
                choice = input('Please enter the number of your choice: ')
                while choice != 'exit':
                    while not choice.isdigit() or not 1 <= int(choice) <= len(options):
                        print('Invalid input. Please enter a number between 1 and', len(options))
                        choice = input('Please enter the number of your choice: ')
                    # Get the chosen option
                    chosen_option = options[int(choice) - 1]
                    # pass the chosen option to the db
                    if chosen_option == 'exit':
                        self.fm_manual_control_drive('stop')
                        return
                    else:
                        self.fm_manual_control_drive(chosen_option)
                    choice = input('Please enter the number of your choice: ')
            else:
                self.fm_manual_control_trigger(False)

        elif chosen_option == 'fm_send_task_request':

                # Define the valid options
                valid_loc_id_pattern = re.compile(r'^[A-Z][1-9]\d*|^[A-Z]10\d*$')

                # Ask for and validate from_loc_id
                # from_loc_id = "A3" # example
                from_loc_id = input('Please enter from_loc_id [options must conform to the pattern AnyAlphabet+<number>]: ')
                while not valid_loc_id_pattern.match(from_loc_id) or from_loc_id not in self.job_ids:
                        print('Invalid input. It does not conform to the AnyAlphabet+<number> pattern or not in : '+str( self.job_ids))
                        from_loc_id = input('Please enter from_loc_id [options must conform to the pattern AnyAlphabet+<number>] or exit: ')
                        if from_loc_id == 'exit':
                                return

                # Ask for and validate to_loc_id
                # to_loc_id = "A5" # example
                to_loc_id = input('Please enter to_loc_id [options must conform to the pattern AnyAlphabet+<number>]: ')
                while not valid_loc_id_pattern.match(to_loc_id) or to_loc_id not in self.job_ids:
                        print('Invalid input. It does not conform to the AnyAlphabet+<number> pattern or not in : '+str( self.job_ids))
                        to_loc_id = input('Please enter to_loc_id [options must conform to the pattern AnyAlphabet+<number>] or exit: ')
                        if to_loc_id == 'exit':
                                return

                # Ask for and validate task_name
                # task_name = "transport" # example
                task_name = input('Please enter task_name [move, loop, charge, clean, transport]: ')
                while task_name not in self.job_types:
                        print('Invalid input. The task name must be one of the following: move, loop, charge, clean, transport.')
                        task_name = input('Please enter task_name [move, loop, charge, clean, transport] or exit: ')
                        if task_name == 'exit':
                                return

                # Ask for and validate task_priority
                # task_priority = "low" # example
                task_priority = input('Please enter task_priority [low, high, medium]: ')
                while task_priority not in self.job_priorites:
                        print('Invalid input. The task priority must be one of the following: low, high, medium.')
                        task_priority = input('Please enter task_priority [low, high, medium] or exit: ')
                        if task_priority == 'exit':
                                return

                # Now you have valid inputs and you can proceed with your task
                print(f'from_loc_id: {from_loc_id}, to_loc_id: {to_loc_id}, task_name: {task_name}, task_priority: {task_priority}')

                # answer = "yes"
                answer = input('WARNING! Are you sure you want to continue with robot task? [yes, no]: ')
                while answer not in self.decision:
                    print('Invalid input. The decision must be one of the following: yes, no.')
                    answer = input('Please enter decision [yes, no]: ')
                if answer == 'no':
                    global checkpoints
                    if checkpoints != []:
                        checkpoints = []
                    print(" checkpoints now emptied: ", str(checkpoints))
                    return
                else:
                    self.fm_send_task_request(from_loc_id, to_loc_id, task_name, task_priority)

        elif chosen_option == 'fm_clear_task_trigger':
                answer = input('WARNING! Are you sure you want to clear task and make robot idle? [yes, no]: ')
                while answer not in self.decision:
                        print('Invalid input. The decision must be one of the following: yes, no.')
                        answer = input('Please enter decision [yes, no]: ')
                if answer == 'yes':
                        self.fm_clear_task_trigger(True)

        elif chosen_option == 'fm_add_landmark_request':
                # drive robot to the location you want to save as landmark.
                # use the interactive options to assign name and neighbour.

                # Ask for and validate station_type
                station_type = input('Please enter station_type from: '+str(self.station_type)+'.')
                while station_type not in self.station_type:
                        print('Invalid input. The station_type must be one of the following: '+str(self.station_type)+'.')
                        station_type = input('Please enter station_type : '+str(self.station_type)+'. or exit: ')
                        if station_type == 'exit':
                                return

                # Define the valid options
                valid_loc_id_pattern =  re.compile(r'^[A-Z][1-9]\d*|^[A-Z]10\d*$')
                # Ask for and validate from_loc_id
                input_loc_id = input('Please enter valid location/landmark id [options must conform to the pattern AnyAlphabet+<number>]: ')
                while not valid_loc_id_pattern.match(input_loc_id):
                        print('Invalid input. It does not conform to the AnyAlphabet+<number> pattern.')
                        input_loc_id = input('Please enter input_loc_id [options must conform to the pattern AnyAlphabet+<number>] or exit: ')
                        if input_loc_id == 'exit':
                                return

                # Ask for and validate from_loc_id
                print("Field 'Neighbors' cannot be left empty. Please choose appropriate connected checkpoints.")

                # Define the valid pattern
                valid_loc_id_pattern = re.compile(r'^A-Z(,A-Z)*$')

                # Ask for and validate neighbor_loc_ids sample input_str = "A4,A2,A3,A6"
                neighbor_loc_ids = input('Please enter valid location/landmark id [options must conform to the pattern AnyAlphabet+<number>]: ')
                while not valid_loc_id_pattern.match(neighbor_loc_ids.replace(" ","")):
                        print('Invalid input. It does not conform to the AnyAlphabet+<number> pattern.')
                        neighbor_loc_ids = input('Please enter neighbor_loc_ids [options must conform to the pattern AnyAlphabet+<number>] or exit: ')
                        if neighbor_loc_ids == 'exit':
                                return

                self.fm_add_landmark_request(station_type, input_loc_id, neighbor_loc_ids)

        elif chosen_option == 'fm_delete_landmark_request':
                # Define the valid pattern
                valid_pattern = re.compile(r'^[A-Z][1-9]\d*|^[A-Z]10\d*$')
                # Ask for and validate the input
                input_str = input('Please enter a string of the form AnyAlphabet<number>,<word>: ')
                while not valid_pattern.match(input_str.replace(" ","")):
                        print('Invalid input. It does not conform to the AnyAlphabet<number>,<word> pattern.')
                        input_str = input('Please enter a string of the form AnyAlphabet<number>,<word> or exit: ')
                        if input_str == 'exit':
                                return

                self.fm_delete_landmark_request(input_str)

        elif chosen_option == 'exit':
                print("All of this was written and designed by Hazeezadebayo.")
                print("                                           Thank you.")
                return

##################################################
# ---------------------------------------------- #
#      ON START-UP: LOAD FLEET LIST AND MENU     #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ---------------------------------- update_current_agv  ---------------------------------------------
# ----------------------------------------------------------------------------------------------------

    def fetch_fleet_ids(self):
        """
        Fetch all distinct fleet IDs from the database.
        """
        try:
            self.cur.execute("SELECT DISTINCT fleet_id FROM table_robot")
            self.fleet_ids = [str(row[0]) for row in self.cur.fetchall()]
        except psycopg2.errors.UndefinedTable:
            print('[manager] The table "table_robot" does not exist.')
            return []
        except psycopg2.Error as e:
            print(f'[manager] Database error while fetching fleet IDs: {e}')
            return []

    def fetch_robot_ids(self, f_id):
        """
        Fetch all robot IDs associated with the provided fleet_id.
        """
        try:
            if not self.validate_fleet_id(f_id):
                return []
            self.cur.execute("SELECT robot_id FROM table_robot WHERE fleet_id = %s;", (f_id,))
            self.robot_ids = [str(row[0]) for row in self.cur.fetchall()]
        except psycopg2.errors.UndefinedTable:
            print('[manager] The table "table_robot" does not exist.')
        except psycopg2.Error as e:
            print(f'[manager] Database error while fetching robot IDs: {e}')



    # --------------------------------

    def get_valid_input(self, prompt, valid_options):
        """
        Prompt user for input until a valid option is provided.
        """
        user_input = input(prompt)
        while user_input not in valid_options:
            print(f'Invalid input. Please enter one of the following: {valid_options}.')
            user_input = input(prompt)
        return user_input

    # --------------------------------

    def interactive_start_robot(self):
        """
        Prompt the user to assign a task to a robot if required.
        """
        decision = self.get_valid_input(
            '\nAlert! Do you want to assign a robot a task? [yes, no]: ',
            self.decision
        )
        if decision == 'yes':
            r_id = self.get_valid_input(
                f'Please enter int robot_id: {self.robot_ids}.',
                self.robot_ids
            )
            self.single_robot_view = True
            # Start with the chosen robot
            self.fm_start_robot(r_id)
        else:
            r_id = 0
            self.fm_start_robot(r_id)
            self.single_robot_view = False

    def fm_start_robot(self, r_id):
        """
        [API] call with robot_id.
        Initialize the robot fleet first with the given fleet_id,
        the make call to this to set robot_id.
        """
        global robot_id

        # find the r_id.
        if not self.validate_robot_id(r_id):
            return

        robot_id = r_id
        self.fm_fetch_active()
        self.busy_wait(0.05)

    # --------------------------------

    def interactive_start_robot_fleet(self):
        """
        Start interacting with the robot fleet based on user input.
        """

        self.fetch_fleet_ids()

        fleet_id = self.get_valid_input(
            f'Please enter fleet_id {self.fleet_ids}: ',
            self.fleet_ids
        )

        # Start the robot fleet with the chosen fleet_id
        self.fm_start_fleet(fleet_id)

        self.interactive_start_robot()

    def fm_start_fleet(self, f_id):
        """
        [API] call with fleet_id.
        Initialize the robot fleet with the given fleet_id by populating jobs, landmarks, and robot IDs.
        """

        global fleet_id

        self.fetch_fleet_ids()

        if not self.validate_fleet_id(f_id):
            return

        fleet_id = f_id

        self.job_ids.clear()
        self.landmark_dict = {'x': [], 'y': [], 't': [], 'k': []}

        self.process_itinerary(task_dictionary.get("itinerary", []))

        print("\nValid landmark IDs that can be used for jobs are:")
        print("[landmark_ids]: ", self.job_ids)

        self.fetch_robot_ids(fleet_id)

        print("\nValid robot IDs that can be used here are:")
        print("[robot_ids]: ", self.robot_ids)

        # Plot the world first. Then plot the car/robot in the timed function.
        self.nav_map_obtained = False

    # --------------------------------

    def validate_fleet_id(self, f_id):
        """
        Validate if the provided fleet_id exists in the fetched fleet IDs.
        """
        if f_id not in self.fleet_ids:
            print("Invalid input. The fleet id must be one of the following:", self.fleet_ids)
            return False
        return True

    def validate_robot_id(self, r_id):
        """
        Validate if the provided robot_id exists in the fetched robot IDs.
        """
        if r_id not in self.robot_ids:
            print("Invalid input. The robot id must be one of the following:", self.robot_ids)
            return False
        return True

    # --------------------------------

    def get_landmark_color(self, description):
        """
        Return color based on landmark description.
        i.e. yellow if its just checkpoint. else fetch actual
        """
        color_map = {
            'charge_dock': 'green',
            'home_dock': 'blue',
            'station_dock': 'cyan',
            'elevator_dock': 'orange',
            'waitpoint': 'springgreen'
        }
        return color_map.get(description, 'yellow')

    def process_itinerary(self, itinerary):
        """
        Process the itinerary to update job_ids and landmark_dict based on fleet_id.
        """
        for task in itinerary:
            if task["fleet_id"] == fleet_id:
                description = task["description"]
                loc_id = task["loc_id"]
                coordinate = task["coordinate"]

                if description in {'station_dock', 'charge_dock', 'elevator_dock', 'home_dock'}:
                    self.job_ids.append(loc_id)

                self.landmark_dict['x'].append(float(coordinate[0]))
                self.landmark_dict['y'].append(float(coordinate[1]))
                self.landmark_dict['t'].append(loc_id)
                self.landmark_dict['k'].append(self.get_landmark_color(description))

    def fm_fetch_active(self):
        """ Refresh active robot without waiting for timer. """
        def run_on_timer():
            self.on_timer(fleet_id, robot_id)
        # Start a new thread to run the on_timer method
        thread = threading.Thread(target=run_on_timer)
        thread.daemon = True
        thread.start()
        # Ensure the thread finishes before returning (if desired)
        thread.join()
        return fleet_id, robot_id

    def busy_wait(self, duration_in_seconds):
        """ Just loop until the time is up """
        end_time = time.time() + duration_in_seconds
        while time.time() < end_time:
            pass

##################################################
# ---------------------------------------------- #
#      on_timer callback ( AGV update)           #
# ---------------------------------------------- #
##################################################

# ----------------------------------------------------------------------------------------------------
# ------------------------------- SINGLE AGV VISUALIZATION -------------------------------------------
# ----------------------------------------------------------------------------------------------------

    def db_list_to_pylist(self, dblist):
        """Convert database list string to Python list."""
        result = []
        for sublist in dblist:
            elements = [float(x) for x in sublist.strip("{}").split(",")]
            result.append(elements)
        return result

    # --------------------------------------------------------------------------------------------

    def on_timer(self, f_id=None, r_id=0):
        """Main timer function to manage robot fleet operations."""
        if f_id is None and len(self.robot_ids) == 0:
            return

        if self.on_timer_lock is True:
            # wait enough time to finish the process.
            self.busy_wait(2.0)

        self.on_timer_lock = True
        for _r_id in self.robot_ids:
            try:
                self._fetch_robot_data(_r_id, f_id, r_id)
                traffic_control = self._update_traffic_control(f_id)
            except Exception as error:
                print(f"[traffic]: select statement {error}.")
                return

            # Plot the world and robot positions
            self.get_world(f_id)
            self.nav_plot(f_id)
            self.check_notification_msgs(f_id)

            # Process robot status and handle traffic
            self._handle_robot_traffic_status(_r_id, f_id, traffic_control)
        self.on_timer_lock = False

    # --------------------------------------------------------------------------------------------

    def _fetch_robot_data(self, _r_id, f_id, r_id):
        """Fetch and update data for a specific robot."""
        self.cur.execute('SELECT * FROM table_robot WHERE robot_id = %s and fleet_id = %s;', (_r_id, f_id,))
        records = self.cur.fetchall()

        for record in records:
            if r_id != 0 and _r_id == str(r_id):
                self._update_feedback_from_record(record)
            self.temp_fb_current_pose = str(record['current_pose']).split(',')
            self.temp_fb_notifications = str(record['notifications']).split(',')
            self.temp_fb_last_checkpoint = str(record['last_checkpoint']).split(',')
            self.temp_fb_checkpoints = str(record['checkpoints']).split(',')
            self.temp_fb_waitpoints = str(record['waitpoints']).split(',')
            self.temp_fb_agv_status = str(record['agv_status']).split(',')
            self.temp_fb_model_config = str(record['model_config']).split(',')
            self.temp_fb_landmarks = str(record['landmark']).split(',')
            self.temp_fb_agv_itinerary = self.db_list_to_pylist(str(record['agv_itinerary']).split("},{"))
            self.temp_fb_traffic = str(record['traffic'])
            self.temp_fb_wait_traffic = str(record['wait_traffic'])
            self.temp_fb_base_map_img = record['base_map_data']

    # --------------------------------------------------------------------------------------------

    def _update_feedback_from_record(self, record):
        """Update feedback data from the database record."""
        self.feedback_current_pose = str(record['current_pose']).split(',')
        self.feedback_notifications = str(record['notifications']).split(',')
        self.feedback_last_checkpoint = str(record['last_checkpoint']).split(',')
        self.feedback_checkpoints = str(record['checkpoints']).split(',')
        self.feedback_waitpoints = str(record['waitpoints']).split(',')
        self.feedback_agv_status = str(record['agv_status']).split(',')
        self.feedback_shutdown = str(record['shutdown'])
        self.feedback_controls = str(record['m_controls']).split(',')
        self.feedback_model_config = str(record['model_config']).split(',')
        self.feedback_landmarks = str(record['landmark']).split(',')
        self.feedback_base_map_img = record['base_map_data']
        self.feedback_floor1_map_img = record['floor1_map_data']
        self.feedback_agv_itinerary = self.db_list_to_pylist(str(record['agv_itinerary']).split("},{"))
        self.feedback_traffic = str(record['traffic'])
        self.feedback_wait_traffic = str(record['wait_traffic'])

    # --------------------------------------------------------------------------------------------

    def _update_traffic_control(self, f_id):
        """Fetch distinct traffic information from the database."""
        traffic_control = []
        self.cur.execute("SELECT DISTINCT traffic, wait_traffic FROM table_robot WHERE fleet_id = %s;", (f_id,))
        for row in self.cur.fetchall():
            for value in row:
                if value != 'none':
                    traffic_control.append(value)
        return traffic_control

    # --------------------------------------------------------------------------------------------

    def _handle_robot_traffic_status(self, _r_id, f_id, traffic_control):
        """Handle the status and traffic control for a specific robot."""
        # next_stop_id = 'unknown'

        next_stop_id = self.temp_fb_last_checkpoint[0]
        if next_stop_id == 'unknown':
            return

        _ = self.check_battery_status(_r_id)

        ros_pose_x, ros_pose_y = self.temp_fb_current_pose[0], self.temp_fb_current_pose[1]

        predecessor_landmark = []
        real_landmark = []
        for elements in self.temp_fb_landmarks[2:]:
            predecessor_landmark.append(elements.split('_')[0])
            real_landmark.append(elements.split('_')[1])

        next_stop_coordinate = self.temp_fb_agv_itinerary[self.temp_fb_checkpoints.index(next_stop_id)]
        dist_to_next = self._calculate_distance(ros_pose_x, ros_pose_y, next_stop_coordinate[0], next_stop_coordinate[1])

        reserved_checkpoint = self.temp_fb_traffic

        try:
            if reserved_checkpoint != next_stop_id and dist_to_next < (float(self.temp_fb_model_config[0]) + 2.7):
                # oh shit! it is occupied, we need to stop our robot.
                print("[traffic]: traffic_control " + str([traffic_control]) + ".")
                if next_stop_id in traffic_control:
                    print("[traffic]: "+str([_r_id])+" next id occupied. ")
                    if self.temp_fb_agv_status[0] == 'active':
                        self._handle_moving_robot_conflict_case(_r_id, f_id, next_stop_id, traffic_control, real_landmark)
                        self._handle_waiting_robot_conflict_case(_r_id, f_id, next_stop_id, traffic_control, reserved_checkpoint, next_stop_coordinate)
                else:
                    self._handle_no_conflict_case(_r_id, f_id, next_stop_id)
            self.conn.commit()
        except Exception as error:
            print("[manager]:-on_timer-" + str([error]) + ".")

        self.busy_wait(0.05)

    # --------------------------------------------------------------------------------------------

    def check_battery_status(self, _r_id):
        """ check if battery level is still good for task. """
        cleared = True
        battery_level = float(self.temp_fb_model_config[4])
        if battery_level < self.min_task_bat_level:
            print("[manager]: "+str([_r_id])+" battery is low. please cancel task and request a charge task immediately. ")
            cleared = False # we should use this to cancel the task on the humans behalf and request a charge task automatically.
        return cleared

    # --------------------------------------------------------------------------------------------

    def _calculate_distance(self, ros_pose_x, ros_pose_y, next_stop_coordinate_x, next_stop_coordinate_y):
        """Calculate the distance to the next stop."""
        return math.sqrt((float(next_stop_coordinate_x) - float(ros_pose_x))**2 + (float(next_stop_coordinate_y) - float(ros_pose_y))**2)

    # --------------------------------------------------------------------------------------------

    def _handle_no_conflict_case(self, _r_id, f_id, next_stop_id):
        self.temp_fb_notifications[3] = '0'
        self.temp_fb_agv_status[6] = 'green' # flag
        self.cur.execute('UPDATE table_robot SET traffic = %s, notifications = %s, agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
                        (next_stop_id, ','.join(self.temp_fb_notifications), ','.join(self.temp_fb_agv_status), _r_id, f_id,))
        print("[traffic]:-"+str([_r_id])+" agv_status-" + str([self.temp_fb_agv_status[6]]) + ".")

    # --------------------------------------------------------------------------------------------

    def _handle_moving_robot_conflict_case(self, _r_id, f_id, next_stop_id, traffic_control, real_landmark):
        """Handle conflicts when the next stop is occupied."""
        # [ROBOT ACTIVE] CASE 1:
        #################################################################################################
        if self.temp_fb_notifications[4] == "None":
            if next_stop_id in real_landmark:

                # """Handle conflict when the robot is active and the next stop is a landmark."""
                if self.temp_fb_landmarks[1] == "transport":
                    if next_stop_id in real_landmark[2:] and next_stop_id != real_landmark[-1]:
                        self.temp_fb_agv_status[6] = 'green'
                    else:
                        # """Handle conflicts specific to transport tasks."""
                        if next_stop_id == real_landmark[-1]:
                            self.temp_fb_notifications[0] = "Inactive: all home_dock occupied."
                        if self.temp_fb_notifications[3] == '0':
                            self.temp_fb_notifications[3] = time.time_ns()
                        self.temp_fb_notifications[6] = 'red'
                        self.temp_fb_notifications[3] = str(time.time_ns() - int(self.temp_fb_notifications[3]))

                elif self.temp_fb_landmarks[1] in ["loop", "move", "charge"]:
                    # """Handle conflicts specific to loop, move, or charge tasks."""
                    if (len(self.temp_fb_last_checkpoint) > 1) and (self.temp_fb_landmarks[1] == "charge"):
                        self.temp_fb_agv_status[6] = 'green'
                    else:
                        if (len(self.temp_fb_last_checkpoint) == 1) and (self.temp_fb_landmarks[1] == "charge"):
                            self.temp_fb_notifications[0] = "Inactive: all charge_dock occupied."
                        if self.temp_fb_notifications[3] == '0':
                            self.temp_fb_notifications[3] = time.time_ns()
                        self.temp_fb_agv_status[6] = 'red'
                        self.temp_fb_notifications[3] = str(time.time_ns() - int(self.temp_fb_notifications[3]))

            elif next_stop_id not in real_landmark:

                # """Handle conflicts when the robot is waiting and the next stop is not a landmark."""
                if self.temp_fb_notifications[3] == '0':
                    self.temp_fb_notifications[3] = time.time_ns()
                self.temp_fb_agv_status[6] = 'red'
                self.temp_fb_notifications[3] = str(time.time_ns() - int(self.temp_fb_notifications[3]))

            # """Update the status of the robot and handle waiting if needed."""
            if self.temp_fb_agv_status[6] == 'green':
                # we might have to skip our target node 'd' for the next 'e'
                self.temp_fb_notifications[4] = "skip"
                # over-write next_stop_id so that we have 'e' as our new target and we can proceed to check its availability
                next_stop_id = self.temp_fb_last_checkpoint[1]
                # if 'e' is not occupied and it is not a dock station, meaning if its a normal checkpoint, we move to it!
                if (next_stop_id not in traffic_control) and (next_stop_id not in real_landmark[2:]):
                    # feedback_notifications[4] = "skip"
                    # reserve 'e' in traffic for this robot and set flag to green so it goes there
                    self.cur.execute('UPDATE table_robot SET notifications = %s, traffic = %s WHERE robot_id = %s and fleet_id = %s;',
                        (','.join(self.temp_fb_notifications), next_stop_id, _r_id, f_id,))
                # if 'e' is occupied. now we must still skip but we gonna wait forever.
                # we handle this down the code! line 788 upwards or so...
                else:
                    # if 'e' which is our new target is occupied, we start waiting...
                    if self.temp_fb_notifications[3] == '0':
                        self.temp_fb_notifications[3] = time.time_ns()
                    # also flag is red
                    self.temp_fb_agv_status[6] = 'red' # flag
                    self.temp_fb_notifications[3] = str(time.time_ns() - int(self.temp_fb_notifications[3]))
                    # and we must quickly set that to the robot.
                    self.cur.execute('UPDATE table_robot SET notifications = %s, agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
                        (','.join(self.temp_fb_notifications), ','.join(self.temp_fb_agv_status), _r_id, f_id,))
            # if our decision had been directly 'red' all along. then, ...
            else: # this changes nothing if we never entered anywhere.
                self.cur.execute('UPDATE table_robot SET notifications = %s, agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
                                (','.join(self.temp_fb_notifications), ','.join(self.temp_fb_agv_status), _r_id, f_id,))

            print("[traffic]: "+str([_r_id])+" agv_status - " + str([self.temp_fb_agv_status[6]]) + ".")

    # --------------------------------------------------------------------------------------------

    def _handle_waiting_robot_conflict_case(self, r_id, f_id, next_stop_id, traffic_control, reserved_checkpoint, next_stop_coordinate):
        """
        Handle conflicts when a robot is waiting at a checkpoint and another robot is on a collision course.
        """
        if self.temp_fb_agv_status[6] == 'red':
            # Fetch data for the robot occupying the traffic node
            record_ = self.fetch_mex_data(next_stop_id, f_id)
            if record_:
                self.handle_active_mex_conflict(r_id, f_id, next_stop_id, traffic_control, reserved_checkpoint, next_stop_coordinate, record_)
        else:
            pass

    def fetch_mex_data(self, next_stop_id, f_id):
        """
        Fetch data for the robot that is currently occupying the traffic node.
        """
        self.cur.execute("SELECT * FROM table_robot WHERE traffic = %s and fleet_id = %s;", (next_stop_id, f_id,))
        record = self.cur.fetchone()
        return record

    # --------------------------------------------------------------------------------------------

    def handle_active_mex_conflict(self, r_id, f_id, next_stop_id, traffic_control, reserved_checkpoint, next_stop_coordinate, record_):
        """
        Handle the conflict with an active robot.
        """

        # Initialize variables for the conflicting robot 'unknown MEx' that might be putting us in a stand-off
        # mex_r_id = None
        # mex_last_checkpoint = ['None']
        # mex_notifications = ['None'] * 7

        mex_r_id = str(record_['robot_id'])
        mex_current_pose = str(record_['current_pose']).split(',')
        mex_last_checkpoint = str(record_['last_checkpoint']).split(',')
        mex_notifications = str(record_['notifications']).split(',')
        mex_agv_status = str(record_['agv_status']).split(',')
        mex_landmarks = str(record_['landmark']).split(',')
        mex_wait_traffic = str(record_['wait_traffic'])
        mex_waitpoints = str(record_['waitpoints']).split(',')
        mex_model_config = str(record_['model_config']).split(',')

        # Handle active robot conflict
        if mex_agv_status[0] == 'active':
            if mex_agv_status[6] == 'red' and mex_notifications[4] != "skip":
                # print("debug.------ ", "mex_last_checkpoint[0] ", mex_last_checkpoint[0]," reserved_checkpoint ", reserved_checkpoint )
                if mex_last_checkpoint[0] == reserved_checkpoint:
                    print(" --        --        --        --")
                    print("[traffic]: "+str([r_id])+" started negotiation. ")

                    priority_val = self.map_priority(self.temp_fb_landmarks[0])
                    mex_priority_val = self.map_priority(mex_landmarks[0])
                    reserved_checkpoint_coordinate = self.temp_fb_agv_itinerary[self.temp_fb_checkpoints.index(reserved_checkpoint)]

                    if priority_val > mex_priority_val:
                        mex_wait_traffic, mex_notifications = self.handle_priority_higher(
                            r_id, next_stop_id, traffic_control, mex_notifications, next_stop_coordinate,
                            mex_r_id, mex_waitpoints, reserved_checkpoint, mex_current_pose, mex_model_config,
                            reserved_checkpoint_coordinate)

                    elif priority_val < mex_priority_val:
                        _,_ = self.handle_priority_lower(r_id, reserved_checkpoint,
                                traffic_control, mex_current_pose, reserved_checkpoint_coordinate,
                                mex_model_config, next_stop_id, mex_notifications,
                                next_stop_coordinate, mex_r_id, mex_waitpoints)

                    else:
                        # """ Handle the case where both robots have equal priority. """
                        print("[traffic]: Equal task priorities detected. Time priority will be used. ")
                        if int(self.temp_fb_notifications[3]) > int(mex_notifications[3]):
                            mex_wait_traffic, mex_notifications = self.handle_priority_higher(
                                r_id, next_stop_id, traffic_control, mex_notifications, next_stop_coordinate,
                                mex_r_id, mex_waitpoints, reserved_checkpoint, mex_current_pose, mex_model_config,
                                reserved_checkpoint_coordinate)

                        elif int(self.temp_fb_notifications[3]) < int(mex_notifications[3]):
                            _,_ = self.handle_priority_lower(r_id, reserved_checkpoint,
                                    traffic_control, mex_current_pose, reserved_checkpoint_coordinate,
                                    mex_model_config, next_stop_id, mex_notifications,
                                    next_stop_coordinate, mex_r_id, mex_waitpoints)

                    if (self.temp_fb_notifications[5] != 'wt') or (mex_notifications[5] != 'wt'):
                        self.update_robot_status(r_id,
                                                 f_id,
                                                 next_stop_id,
                                                 mex_r_id,
                                                 mex_wait_traffic,
                                                 mex_last_checkpoint,
                                                 mex_notifications,
                                                 mex_agv_status)
                    else:
                        # somehow we were supposed to wait, but no waitpoint available for either robot.
                        print("[traffic]: "+str([r_id])+" and mex_r_id "+str([mex_r_id])+" stuck because \
                            waitpoint was not found in graph. Human assistance required.")

        else:
            print("[traffic]: "+str([r_id])+" stuck because mex_r_id "+str([mex_r_id])+" not responding. \
                  Human assistance required. Next node occupied by an inactive robot.")

    # --------------------------------------------------------------------------------------------

    def handle_priority_higher(self, r_id, next_stop_id, traffic_control, mex_notifications,
            next_stop_coordinate, mex_r_id, mex_waitpoints, reserved_checkpoint,
            mex_current_pose, mex_model_config, reserved_checkpoint_coordinate, from_source=True):
        """
        Handle the case where the current robot has higher priority.
        """
        # this robot should reserve the node it wants to go, and then drive directly there.
        # the MEx should reserve this robot's current reserved node, then go to the waiting area first before driving towards it.
        print("[traffic]: "+str([r_id])+" has higher priority. ")
        mex_wait_traffic = self.check_waitpoint_association(next_stop_id, mex_waitpoints)

        # check if there is waitpoint for the other robot, so that perhaps the other robot should wait instead.
        if mex_wait_traffic is None and from_source is True:
            # could be that this robot occupies a checkpoint without a waitpoint or a station dock.
            self.handle_priority_lower(r_id, reserved_checkpoint, traffic_control, mex_current_pose,
                reserved_checkpoint_coordinate, mex_model_config, next_stop_id, mex_notifications,
                next_stop_coordinate, mex_r_id, mex_waitpoints, False)
        elif mex_wait_traffic is None and from_source is False:
            print("[traffic]: "+str([mex_r_id])+" mex --> could not find waitpoint in graph. human help required.")

        if (mex_wait_traffic is not None) and (mex_wait_traffic not in traffic_control):
            print("[traffic]: "+str([mex_r_id])+" mex --> wait at "+str([mex_wait_traffic])+".")
            mex_est_time = self.estimate_time_to_node(robot_pos = self.temp_fb_current_pose,
                                                    node_pos = next_stop_coordinate,
                                                    lin_vel = float(self.temp_fb_model_config[2]))
            mex_notifications[5] = str(mex_est_time)
            print("[traffic]: "+str([mex_r_id])+" mex --> estimated wait time "+str([mex_est_time])+".")

        return mex_wait_traffic, mex_notifications

    # --------------------------------------------------------------------------------------------

    def handle_priority_lower(self, r_id, reserved_checkpoint, traffic_control, mex_current_pose,
            reserved_checkpoint_coordinate, mex_model_config, next_stop_id, mex_notifications,
            next_stop_coordinate, mex_r_id, mex_waitpoints, from_source=True):
        """
        Handle the case where the current robot has lower priority.
        """
        print("[traffic]: "+str([r_id])+" has lower priority. ")
        self.temp_fb_wait_traffic = self.check_waitpoint_association(reserved_checkpoint, self.temp_fb_waitpoints)

        # if we have not checked before,
        # check if there is waitpoint for the other robot, so that perhaps the other robot should wait instead.
        if self.temp_fb_wait_traffic is None and from_source is True:
            # could be that this robot occupies a checkpoint without a waitpoint or a station dock.
            self.handle_priority_higher(r_id, next_stop_id, traffic_control, mex_notifications,
                next_stop_coordinate, mex_r_id, mex_waitpoints, reserved_checkpoint,
                mex_current_pose, mex_model_config, reserved_checkpoint_coordinate, False)
        elif self.temp_fb_wait_traffic is None and from_source is False:
            print("[traffic]: "+str([r_id])+" --> could not find waitpoint in graph. human help required.")

        if self.temp_fb_wait_traffic and self.temp_fb_wait_traffic not in traffic_control:
            print("[traffic]: "+str([r_id])+" --> wait at "+str([self.temp_fb_wait_traffic])+".")
            est_time = self.estimate_time_to_node(robot_pos = mex_current_pose,
                                                  node_pos = reserved_checkpoint_coordinate,
                                                  lin_vel = float(mex_model_config[2]))
            self.temp_fb_notifications[5] = str(est_time)
            print("[traffic]: "+str([r_id])+" --> estimated wait time "+str([est_time])+".")

        return self.temp_fb_wait_traffic, self.temp_fb_notifications


    # --------------------------------------------------------------------------------------------

    def update_robot_status(self, r_id, f_id, next_stop_id, mex_r_id, mex_wait_traffic, mex_last_checkpoint, mex_notifications, mex_agv_status):
        """
        Update the status of both robots after negotiation.
        """
        self.temp_fb_notifications[3] = '0'
        self.temp_fb_agv_status[6] = 'green'
        self.cur.execute(
            'UPDATE table_robot SET traffic = %s, wait_traffic = %s, notifications = %s, agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
            (next_stop_id, self.temp_fb_wait_traffic, ','.join(self.temp_fb_notifications), ','.join(self.temp_fb_agv_status), r_id, f_id,))
        print("[traffic]: "+str([r_id])+" agv_status transitioning to " + str([self.temp_fb_agv_status[6]]) + ".")

        mex_notifications[3] = '0'
        mex_agv_status[6] = 'green'
        self.cur.execute(
            'UPDATE table_robot SET traffic = %s, wait_traffic = %s, notifications = %s, agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
            (mex_last_checkpoint[0], mex_wait_traffic, ','.join(mex_notifications), ','.join(mex_agv_status), mex_r_id, f_id,))
        print("[traffic]: "+str([mex_r_id])+" mex agv_status transitioning to " + str([mex_agv_status[6]]) + ".")

        print("[traffic]: "+str([r_id])+" completed negotiation.")


    # --------------------------------------------------------------------------------------------

    def map_priority(self, priority):
        """
        Function to map priority strings to integers
        """
        mapping = {'low': 1, 'medium': 2, 'high': 3}
        return mapping.get(priority, 0)  # Default to 0 if priority is not recognized

    # --------------------------------------------------------------------------------------------

    def check_waitpoint_association(self, node, mex_waitpoints):
        """
        Given a node, extract the numeric part, create a waitpoint identifier by appending 'W',
        and check if it is present in the mex_waitpoints list. for example;
        # record = {'waitpoints': 'W8,W11,W12,W1,W4,W7,W6'}
        # mex_waitpoints = str(record['waitpoints']).split(','); node = 'A5'
        Args:
        - node (str): The node identifier (e.g., 'A5').
        - mex_waitpoints (list): List of waitpoints to check against.
        Returns:
        - waitpoint or none.
        """
        # Extract the numeric part from the node
        numeric_part = ''.join(filter(str.isdigit, node))
        # Create the waitpoint identifier
        waitpoint = f'W{numeric_part}'
        # Check if the waitpoint is in the mex_waitpoints list
        if waitpoint in mex_waitpoints:
            return waitpoint
        return None

    # --------------------------------------------------------------------------------------------

    def estimate_time_to_node(self, robot_pos, node_pos, lin_vel):
        """
        Estimate the time taken for a robot to reach a given node.

        Args:
        - robot_pos (tuple): The current position of the robot (x, y, theta).
        - node_pos (tuple): The position of the node (x, y, z, w).
        - lin_vel (float): The linear velocity of the robot.

        Returns:
        - float: Estimated time to reach the node.
        """
        # Extract coordinates from robot position and node position
        robot_x, robot_y = float(robot_pos[0]), float(robot_pos[1])
        node_x, node_y = float(node_pos[0]), float(node_pos[1])
        # Calculate the Euclidean distance between the robot and the node
        distance = math.sqrt((node_x - robot_x) ** 2 + (node_y - robot_y) ** 2)
        # Estimate time to reach the node
        if lin_vel == 0.0:
            return float('inf')  # If velocity is zero, time is infinite
        time_to_reach = distance / lin_vel
        # print(f"Estimated time to reach the node: {time_to_reach} seconds")
        if time_to_reach <= 0.1:
            time_to_reach = 1.0
        return time_to_reach

# ---------------------------------------------- #
#      get_world  (WALLS AS OBSTACLES)           #
# ---------------------------------------------- #
# ----------------------------------------------------------------------------------------------------
# ----------------------------------  WORLD PLOT -----------------------------------------------------
# ----------------------------------------------------------------------------------------------------

    def get_world(self, f_id=None):
        """ Get and display world """
        if f_id is None:
            return

        # Check if the navigation map has not been obtained and the image data is valid
        if self.show_robot_animation:
            if not self.nav_map_obtained and self.temp_fb_base_map_img and self.temp_fb_base_map_img != 'None':
                self.allobs_x, self.allobs_y = [], []

                # Convert the memory object to bytes and open the image
                img_data = self.temp_fb_base_map_img.tobytes()
                img = Image.open(io.BytesIO(img_data))

                # Convert the image to a NumPy array and flip it vertically
                img_array = np.array(img)
                src = cv2.flip(img_array, 0)

                # Apply thresholding to the image
                thresh = 5
                max_value = 255
                _, dst = cv2.threshold(src, thresh, max_value, cv2.THRESH_BINARY)

                # Iterate over the image to extract obstacle coordinates
                for i in range(dst.shape[0]):  # Height
                    for j in range(dst.shape[1]):  # Width
                        if dst[i, j] == 0:  # Obstacle detected
                            # Calculate obstacle coordinates
                            self.allobs_x.append((i * 0.05) - 10.15)
                            self.allobs_y.append((j * 0.05) - 9.8)

                # Mark the navigation map as obtained
                self.nav_map_obtained = True

            elif self.nav_map_obtained:
                # Navigation map already obtained
                pass

            else:
                # Image not found
                print("[manager]: No image found. Please ensure that fleet_map_path in client is properly set in at least one robot.")

# ---------------------------------------------- #
#   nav_plot.  (SHOW CURRENT AGV TRAJECTORY)     #
# ---------------------------------------------- #
# ----------------------------------------------------------------------------------------------------
# ------------------------------------  NAV PLOT -----------------------------------------------------
# ----------------------------------------------------------------------------------------------------

    def nav_plot(self, f_id=None):
        """Plot the navigation and robot status."""

        if f_id is None:
            return

        # Check if robot animation should be displayed and feedback data is available
        if self.show_robot_animation:
            if self.feedback_base_map_img and self.feedback_base_map_img != 'None':
                self.update_agv_track()
                self.plot_navigation_data()
                plt.draw()
                plt.pause(0.001)  # Non-blocking call to update the figure
            else:
                self.plot_initial_world(f_id)

    # --------------------------------------------------------------------------------------------

    def update_agv_track(self):
        """Update AGV trajectory tracking."""
        if self.single_robot_view and self.feedback_current_pose:
            current_x, current_y = float(self.feedback_current_pose[0]), float(self.feedback_current_pose[1])
            last_x, last_y = self.agv_track_dict['x'][-1] if self.agv_track_dict['x'] else None, \
                            self.agv_track_dict['y'][-1] if self.agv_track_dict['y'] else None

            # Update track dictionary if the current pose is different from the last recorded pose
            if last_x is None or (current_x != last_x or current_y != last_y):
                self.agv_track_dict['x'].append(current_x)
                self.agv_track_dict['y'].append(current_y)

            # Keep the track list length manageable
            self.agv_track_dict['x'] = self.agv_track_dict['x'][-45:]
            self.agv_track_dict['y'] = self.agv_track_dict['y'][-45:]

    # --------------------------------------------------------------------------------------------

    def plot_navigation_data(self):
        """Plot the navigation data on the axes."""
        self.ax.cla()  # Clear the axes

        # Plot checkpoints
        self.cx, self.cy = [], [] # Clear previous data
        itinerary = self.get_agv_itinerary(self.feedback_checkpoints)
        if len(itinerary)!=0 and self.feedback_checkpoints != ['A', 'A', 'A']:
            # Extract coordinates from itinerary
            for coord in itinerary:
                if len(coord) >= 2:
                    self.cx.append(coord[0])
                    self.cy.append(coord[1])
            # Scatter plot the coordinates
            self.ax.scatter(self.cx, self.cy, marker="P", facecolor='red')

        # Plot landmarks
        if self.landmark_dict['x']:
            self.ax.scatter(self.landmark_dict['x'], self.landmark_dict['y'], marker="P", color=self.landmark_dict['k'])
            for x, y, label in zip(self.landmark_dict['x'], self.landmark_dict['y'], self.landmark_dict['t']):
                self.ax.annotate(label, (x, y))

        # Plot walls/obstacles
        self.ax.scatter(self.allobs_y, self.allobs_x, marker="X", facecolor='black')  # Obstacles

        # Plot AGV trajectory
        self.ax.plot(self.agv_track_dict['x'], self.agv_track_dict['y'], color='orange', linestyle='dashed', label="trajectory")

        # Plot the AGV itself
        if self.feedback_current_pose:
            self.plot_car(float(self.feedback_current_pose[0]), float(self.feedback_current_pose[1]), float(self.feedback_current_pose[2]), steer=0)

        # Set figure properties
        self.ax.axis("tight")
        self.ax.grid(True)
        status = self.feedback_agv_status[0]
        if status in ['idle', 'inactive', 'active']:
            self.ax.set_title(f"AGV: {robot_id}, STATUS: {status}")

    # --------------------------------------------------------------------------------------------

    def plot_initial_world(self, f_id):
        """Initialize and plot the world from database."""
        try:
            self.cur.execute('SELECT current_pose FROM table_robot WHERE fleet_id = %s;', (f_id,))
            for record in self.cur.fetchall():
                # Clear the axes
                self.ax.cla()
                feedback_all_poses = str(record['current_pose']).split(',')
                # plot walls/obstacle, basically plot the MAP
                self.ax.scatter(self.allobs_y, self.allobs_x,  marker="X",facecolor='black') # "*", "X", "s", "P" ##
                # plot the agvs, basically a beautiful vehicle
                self.plot_car(float(feedback_all_poses[0]), float(feedback_all_poses[1]), float(feedback_all_poses[2]), steer=0)
                plt.draw()
                # plt.show() # ---> blocking call. code will pass only if window is closed
                plt.pause(0.001) # Non-blocking call. pause to update the figure

        except Exception as error:
            print(f"[manager]:-nav_plot-{error}.")









# ---------------------------------------------- #
#      PLOT CAR:                                 #
# ---------------------------------------------- #
# ----------------------------------------------------------------------------------------------------
# ------------------------------------  car PLOT -----------------------------------------------------
# ----------------------------------------------------------------------------------------------------

    def plot_car(self, x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover
        # Define car outline and wheels
        outline = np.array([[-BACKTOWHEEL, LENGTH - BACKTOWHEEL, LENGTH - BACKTOWHEEL, -BACKTOWHEEL, -BACKTOWHEEL],
                            [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

        wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                        [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

        # Create copies for all wheels
        fr_wheel = np.copy(wheel)
        fl_wheel = np.copy(wheel)
        fl_wheel[1, :] *= -1
        rr_wheel = np.copy(wheel)
        rl_wheel = np.copy(wheel)
        rl_wheel[1, :] *= -1

        # Rotation matrices
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
        Rot2 = np.array([[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]])

        # Apply steering rotation to front wheels
        fr_wheel = (fr_wheel.T @ Rot2).T
        fl_wheel = (fl_wheel.T @ Rot2).T
        fr_wheel[0, :] += WB
        fl_wheel[0, :] += WB

        # Apply yaw rotation to all parts
        fr_wheel = (fr_wheel.T @ Rot1).T
        fl_wheel = (fl_wheel.T @ Rot1).T
        rr_wheel = (rr_wheel.T @ Rot1).T
        rl_wheel = (rl_wheel.T @ Rot1).T
        outline = (outline.T @ Rot1).T

        # Translate all parts to the car's position
        parts = [outline, fr_wheel, rr_wheel, fl_wheel, rl_wheel]
        for part in parts:
            part[0, :] += x
            part[1, :] += y

        # Plot the car
        self.ax.plot(outline[0, :], outline[1, :], truckcolor)
        self.ax.plot(fr_wheel[0, :], fr_wheel[1, :], truckcolor)
        self.ax.plot(rr_wheel[0, :], rr_wheel[1, :], truckcolor)
        self.ax.plot(fl_wheel[0, :], fl_wheel[1, :], truckcolor)
        self.ax.plot(rl_wheel[0, :], rl_wheel[1, :], truckcolor)
        self.ax.plot(x, y, "*")


##################################################
# ---------------------------------------------- #
#      SEND TASK REQUEST                         #
# ---------------------------------------------- #
##################################################

# ----------------------------------------------------------------------------------------------------
# ---------------------------------- fm_send_task_request  -------------------------------------------------
# ----------------------------------------------------------------------------------------------------

    def fm_send_task_request(self, from_loc_id, to_loc_id, task_name, task_priority):
        """
        Sends a task request to the selected robot and updates its status.
        """
        # global checkpoints, landmark, agv_itinerary, robot_id, fleet_id, waitpoints, wait_itinerary

        if robot_id == 0:
            print("[manager]:-Robot is not selected. Please choose robot.")
            return

        self.request_tasks(from_loc_id, to_loc_id, task_name, task_priority)

        if checkpoints and task_cleared is True:
            self.feedback_agv_status[0] = 'active'
            try:
                self.cur.execute('UPDATE table_robot SET agv_status = %s, checkpoints = %s, landmark = %s, \
                    agv_itinerary = %s, waitpoints = %s, wait_itinerary = %s WHERE robot_id = %s and fleet_id = %s',
                (','.join(self.feedback_agv_status),','.join(checkpoints),','.join(landmark),
                    agv_itinerary, ','.join(waitpoints), wait_itinerary, robot_id, fleet_id,))
                self.conn.commit()
            except Exception as error:
                print(f"[manager]:-send_task_request- Error: {error}.")
        else:
            print('[manager]: Task not cleared. Hence, not assigned.')

# ---------------------------------------------- #
#     ON TASK REQUEST                            #
# ---------------------------------------------- #

# ----------------------------------------------------------------------------------------------------
# -------------------------- TaskScheduler: request_tasks  -------------------------------------------
# ----------------------------------------------------------------------------------------------------

    def request_tasks(self, from_loc_id, to_loc_id, task_name, task_priority):
        """
        Determines the task itinerary and updates checkpoints based on the given task.
        """
        global checkpoints, task_cleared
        checkpoints = []
        task_cleared = False

        if from_loc_id == to_loc_id and task_name not in ['charge', 'move']:
            print("[manager]:-'load' location cannot be equal to 'unload' location.")
            return

        if from_loc_id not in self.job_ids or to_loc_id not in self.job_ids:
            print(f"[manager]:-only ids within this fleet [{fleet_id}] form a valid task request.")
            return

        # schedule task
        task_cleared = self.verify_robot_fitness(task_name)
        if task_cleared is False:
            # robot battery level is not enough, so suggest other robots.
            available_robots = self.suggest_robot(to_loc_id)
            if available_robots:
                print(f"[manager]: Battery percent is low. Only 'charge' task is allowed. Consider robot {available_robots[0]}.")
            else:
                print("[manager]: Battery percent is low. Only 'charge' task is allowed. cancel running tasks and reasign for other robots perhaps.")
            return

        _, loc_node_owner, home_dock_loc_ids, charge_dock_loc_ids = self.find_nearest_location()

        self.build_task_itinerary(from_loc_id,
                                  to_loc_id,
                                  task_name,
                                  task_priority,
                                  loc_node_owner,
                                  home_dock_loc_ids,
                                  charge_dock_loc_ids)

        if len(checkpoints) != 0:
            self.extract_unique_waitpoints()
            self.get_waitpoint_locations()
            self.process_landmark() # predecessor landmark-[task priority, task type or name, landmarks]

        print("---")
        print("[manager]:- checkpoints to pass: ", checkpoints)
        print("[manager]:- corresponding itinerary: ", agv_itinerary)
        print("[manager]:- landmark to visit: ", landmark)
        print("[manager]:- waitpoints: ", waitpoints)
        print("[manager]:- waitpoint's itinerary: ", wait_itinerary)
        print("---")

    # --------------------------------------------------------------------------------------------

    def process_landmark(self):
        """
        Updates the landmark information based on the checkpoints and task type.

        Args:
            checkpoints (list): List of checkpoints.
            landmark (list): List of landmark information.
        """

        global landmark

        if not landmark:
            return ['none']

        task_type = landmark[1]

        if task_type in ['loop', 'transport']:
            # Update pick location
            landmark[2] = self._update_location(landmark[2], offset=1)

            # Update drop location
            landmark[3] = self._update_location(landmark[3], offset=-1)

            if task_type == 'transport':
                # Update home locations
                landmark[4:] = [
                    self._update_location(elem, offset=-1)
                    for elem in landmark[4:]
                ]
        elif task_type == 'move':
            # Update move location
            landmark[2] = self._update_location(landmark[2], offset=-1)
        elif task_type == 'charge':
            # Update charge locations
            landmark[2:] = [
                self._update_location(elem, offset=-1)
                for elem in landmark[2:]
            ]

    # --------------------------------------------------------------------------------------------

    def _update_location(self, location, offset):
        """
        Updates the location based on its position in the checkpoints list.

        Args:
            checkpoints (list): List of checkpoints.
            location (str): The location to be updated.
            offset (int): The offset to determine the new location position.

        Returns:
            str: Updated location with the new position.
        """
        try:
            index = checkpoints.index(location)
            new_location = checkpoints[index + offset] + '_' + location
        except (ValueError, IndexError):
            # Handle case where location is not in checkpoints or index is out of range
            new_location = location

        return new_location

    # --------------------------------------------------------------------------------------------

    def find_nearest_location(self):
        """
        Finds the nearest location to the robot's current position from the list of job locations.

        Returns:
            tuple: The shortest distance and the ID of the nearest location.
        """
        shortest_distance = float('inf')
        loc_node_owner = None
        home_dock_loc_ids = []
        charge_dock_loc_ids = []

        print(f"[manager]:-robot at: {self.feedback_current_pose[0]}, {self.feedback_current_pose[1]}, {self.feedback_current_pose[2]}")

        for itinerary in task_dictionary["itinerary"]:
            if itinerary["fleet_id"] == fleet_id:
                loc_id = itinerary["loc_id"]
                description = itinerary["description"]
                coordinate = itinerary["coordinate"]
                # Build lists of home_dock and charge_dock stations
                if description == 'home_dock':
                    home_dock_loc_ids.append(loc_id)
                elif description == 'charge_dock':
                    charge_dock_loc_ids.append(loc_id)
                # Calculate distance to the current pose
                if description != 'waitpoint':
                    d = math.sqrt((float(coordinate[0]) - float(self.feedback_current_pose[0]))**2 +
                                (float(coordinate[1]) - float(self.feedback_current_pose[1]))**2)
                    # Update shortest distance found
                    if d < shortest_distance:
                        shortest_distance = d
                        loc_node_owner = loc_id
                        print(f"[manager]:-found shortest distance: {shortest_distance} with loc: {loc_node_owner}")

        return shortest_distance, loc_node_owner, home_dock_loc_ids, charge_dock_loc_ids

    # --------------------------------------------------------------------------------------------

    def build_task_itinerary(self, from_loc_id, to_loc_id, task_name, task_priority,
                             loc_node_owner, home_dock_loc_ids, charge_dock_loc_ids):
        """
        Constructs the task itinerary based on the task type and updates checkpoints and landmarks.
        """
        global checkpoints, agv_itinerary, landmark
        checkpoints, agv_itinerary, landmark = [], [], []

        if task_name == 'charge' or task_name == 'move':
            checkpoints, landmark, agv_itinerary = self.handle_charge_or_move_task(task_name, task_priority, loc_node_owner, to_loc_id, charge_dock_loc_ids)
        elif task_name in ['transport', 'loop']:
            checkpoints, landmark, agv_itinerary = self.handle_transport_or_loop_task(task_name, task_priority, loc_node_owner, from_loc_id, to_loc_id, home_dock_loc_ids)
        elif task_name == "clean":
            checkpoints, landmark, agv_itinerary = self.handle_clean_task(task_name)
        else:
            print("[manager]:-Unknown task name.")

    # --------------------------------------------------------------------------------------------

    def handle_charge_or_move_task(self, task_name, task_priority, loc_node_owner, to_loc_id, charge_dock_loc_ids):
        """
        Handles the 'charge' or 'move' tasks by updating checkpoints and landmarks.

        Args:
            task_name (str): Name of the task (e.g., 'charge', 'move').
            task_priority (int): Priority of the task.
            loc_node_owner (str): ID of the nearest location to the robot.
        """

        checkpoints = []
        landmark = []
        agv_itinerary = []

        if task_name == 'charge':
            landmark = [task_priority, task_name]
            # Adds charge dock stations to the list of landmarks and updates checkpoints.
            temp_dock_list = []
            # Iterate over charge_dock_loc_ids using enumerate to get both index and value
            for i, loc_id in enumerate(charge_dock_loc_ids):
                if i == 0:
                    # For the first dock, find the path from the current location to the first dock
                    temp_dock_list.append(self.shortest_path(loc_node_owner, loc_id))
                else:
                    # For subsequent docks, find the path from the previous dock to the current dock
                    temp_dock_list.append(self.shortest_path(charge_dock_loc_ids[i - 1], loc_id))
                landmark.append(loc_id)
            # sum all the checkpoints and make sure there is no repeated start and end checkpoints.
            checkpoints = self.merge_checkpoints(temp_dock_list)

        # if its move, check if the place the robot is closest to, which is where we have assumed is the start point/node,
        # check if it is also the target node, if it is not, then get the path that leads there.
        elif task_name == 'move':
            if loc_node_owner != to_loc_id:
                landmark = [task_priority, task_name, to_loc_id]
                checkpoints = self.shortest_path(loc_node_owner, to_loc_id)

        if len(checkpoints) != 0:
            agv_itinerary = self.get_agv_itinerary(checkpoints)

        return checkpoints, landmark, agv_itinerary

    # ----------------------------------------------------------------------------------------------------

    def handle_transport_or_loop_task(self, task_name, task_priority, loc_node_owner, from_loc_id, to_loc_id, home_dock_loc_ids):
        """
        Constructs the task itinerary based on the task type and updates checkpoints and landmarks.

        Args:
            from_loc_id (str): ID of the starting location.
            to_loc_id (str): ID of the destination location.
            task_name (str): Name of the task (e.g., 'transport', 'loop').
            task_priority (int): Priority of the task.
            loc_node_owner (str): ID of the nearest location to the robot.
        """
        checkpoints = []
        landmark = []
        agv_itinerary = []
        imaginary_checkpoints = []

        if task_name in ['transport', 'loop']:

            if loc_node_owner != from_loc_id:
                imaginary_checkpoints = self.shortest_path(loc_node_owner, from_loc_id)

            real_checkpoints = self.shortest_path(from_loc_id, to_loc_id)

            # Concatenate checkpoints
            # Check if the last element of lst_1 is the same as the first element of lst_2
            if imaginary_checkpoints and imaginary_checkpoints[-1] == real_checkpoints[0]:
                checkpoints = imaginary_checkpoints + real_checkpoints[1:]
            else:
                # If they are different, output both lists concatenated
                checkpoints = real_checkpoints

            if task_name == 'loop':
                landmark = ['high', task_name, from_loc_id, to_loc_id]
                return_checkpoints = self.shortest_path(to_loc_id, from_loc_id)
                if return_checkpoints and checkpoints[-1] == return_checkpoints[0]:
                    checkpoints += return_checkpoints[1:]

            if task_name == 'transport':
                landmark = [task_priority, task_name, from_loc_id, to_loc_id]
                if home_dock_loc_ids:

                    temp_dock_list = []
                    for i, dock_id in enumerate(home_dock_loc_ids):
                        if i == 0:
                            temp_dock_list.append(self.shortest_path(to_loc_id, dock_id))
                        else:
                            temp_dock_list.append(self.shortest_path(home_dock_loc_ids[i - 1], dock_id))
                        landmark.append(dock_id)

                    combined_checkpoints = self.merge_checkpoints(temp_dock_list)

                    if checkpoints and combined_checkpoints and checkpoints[-1] == combined_checkpoints[0]:
                        checkpoints += combined_checkpoints[1:]

            if len(checkpoints) != 0:
                agv_itinerary = self.get_agv_itinerary(checkpoints)

        return checkpoints, landmark, agv_itinerary

    # ----------------------------------------------------------------------------------------------------

    def handle_clean_task(self, task_name):
        """
        # if the task is clean, we do not have any landmark because
        # the coverage path planner will generate path on the map to be traversed.
        # and this path does not necessarily imply the normal routes traversed by other robots within same fleet.
        # truth is! if you have more than 1 cleaning robot in your fleet, then this might be a problem as probability
        # for a robot stand-off becomes high.
        # because idea is;
        #       once its a cleaning task we arent really doing anything other than monitoring the robots.
        #       and checking if its gonna collide with anything and then stop it till the obstacle leaves.
        #       we can not alter a path for which we did not create.
        """

        landmark = ['low', task_name, "unknown", "unknown"]
        checkpoints = ['A', 'B', 'A'] # default no-task structure
        agv_itinerary = [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]] # '[x,y,z,w],[x,y,z,w],[x,y,z,w]' --> 'A', 'A', 'A'

        return checkpoints, landmark, agv_itinerary

    # --------------------------------------------------------------------------------------------

    def merge_checkpoints(self, temp_dock_list):
        """
        Merges checkpoints from temp_dock_list, avoiding duplicate start and end checkpoints.
        """
        # Initialize an empty list to store the combined checkpoints
        combined_checkpoints = []
        # Iterate through temp_dock_list and process each path
        for i, path in enumerate(temp_dock_list):
            if i == 0:
                # For the first path, add all checkpoints
                combined_checkpoints.extend(path)
            else:
                # For subsequent paths, append checkpoints after the last checkpoint of the previous path
                if temp_dock_list[i - 1][-1] == path[0]:
                    combined_checkpoints.extend(path[1:])
                else:
                    combined_checkpoints.extend(path)
        return combined_checkpoints

    # --------------------------------------------------------------------------------------------

    def verify_robot_fitness(self, task_name):
        """Check if the robot is available for the task."""

        cleared = False
        is_idle = self.feedback_agv_status[0] == 'idle'
        has_no_previous_task = self.feedback_checkpoints[0] == self.feedback_checkpoints[1] == self.feedback_checkpoints[2]
        battery_level = float(self.feedback_model_config[4])
        print("For robot ", robot_id, ", is_idle: ", self.feedback_agv_status[0], ", battery level: ", battery_level, " has_no_previous_task: ", self.feedback_checkpoints)
        if has_no_previous_task or is_idle:
            # if battery level is good for task or task in itself is a charge task.
            if battery_level > self.min_task_bat_level or task_name == 'charge':
                cleared = True # if robot has a good battery level

        return cleared

    # --------------------------------------------------------------------------------------------

    def suggest_robot(self, to_loc_id):
        """Suggest a robot with the shortest distance to the target location."""

        coordinate = None

        # Retrieve the target coordinates
        for itinerary in task_dictionary["itinerary"]:
            if itinerary["fleet_id"] == fleet_id and to_loc_id == itinerary["loc_id"]:
                coordinate = itinerary["coordinate"]
                break

        if coordinate is None:
            print("[manager]:-No valid coordinate found for the location ID.")
            return []

        shortest_distance = float('inf')
        nearest_robot_id = None
        available_robots = []

        try:
            self.cur.execute("SELECT * FROM table_robot WHERE fleet_id = %s;", (fleet_id,))
            for record in self.cur.fetchall():
                r_id_ = str(record['robot_id']).split(',')
                checkp_p_ = str(record['checkpoints']).split(',')
                agv_s_ = str(record['agv_status']).split(',')
                model_conf = str(record['model_config']).split(',')
                current_pose = str(record['current_pose']).split(',')

                is_idle = agv_s_[0] == 'idle'
                has_no_previous_task = checkp_p_[0] == checkp_p_[1] == checkp_p_[2]
                battery_level = float(model_conf[4])
                current_position = (float(current_pose[0]), float(current_pose[1]))

                if (has_no_previous_task or is_idle) and (battery_level > self.min_task_bat_level):
                    # Calculate distance to the given coordinate
                    distance = math.sqrt((coordinate[0] - current_position[0])**2 +
                                        (coordinate[1] - current_position[1])**2)
                    # Update shortest distance and nearest robot ID if a shorter distance is found
                    if distance < shortest_distance:
                        shortest_distance = distance
                        nearest_robot_id = r_id_

            if nearest_robot_id:
                available_robots.append(nearest_robot_id)
                print(f"[manager]:-Found nearest robot: {nearest_robot_id} with distance: {shortest_distance}")

        except (ValueError, Exception) as error:
            print(f"[manager]:-suggest_robot- Error: {error}.")

        return available_robots

    # --------------------------------------------------------------------------------------------

    def get_agv_itinerary(self, chkpnts):
        """
        Retrieves the AGV itinerary based on the checkpoints.

        Args:
            checkpoints (list): List of location IDs or alphabets representing the path.

        Returns:
            list: AGV itinerary coordinates.
        """
        itinerary = []
        itinerary_dict = {entry["loc_id"]: entry["coordinate"] for entry in task_dictionary["itinerary"]}

        for checkpoint in chkpnts:
            if checkpoint in itinerary_dict:
                itinerary.append(itinerary_dict[checkpoint])

        return itinerary

    # ----------------------------------------------------------------------------------------------------

    def extract_unique_waitpoints(self):
        """ fetch the locations coordinates associated with the node/checkpoints path """

        print("[manager]:- fetching task associated waitpoints...")

        global waitpoints

        # Initialize a set to store unique waitpoints
        waitpoint = set()
        # Iterate through each checkpoint
        for checkpoint in checkpoints:
            # Check if the checkpoint is in the graph
            if checkpoint in graph:
                # Iterate through neighbors of the checkpoint
                for neighbor, _ in graph[checkpoint]:
                    # Check if the neighbor is a waitpoint (starts with 'W')
                    if neighbor.startswith('W'):
                        # Add the waitpoint to the set
                        waitpoint.add(neighbor)
        # Convert the set to a list
        waitpoints = list(waitpoint)

    # ----------------------------------------------------------------------------------------------------

    def get_waitpoint_locations(self):
        """ Retrieve the locations of the waitpoints from the graph. """
        print("[manager]:- generating waitpoint itinerary...")
        global wait_itinerary
        wait_itinerary = []
        # Iterate over waitpoints with index
        for _, waitpoint in enumerate(waitpoints):
            # Iterate over itinerary entries
            for task in task_dictionary["itinerary"]:
                if task["loc_id"] == waitpoint:
                    wait_itinerary.append(task["coordinate"])

    # --------------------------------------------------------------------------------------------

    def shortest_path(self, start, goal):
        """ Find the shortest path in a graph using Dijkstra's algorithm. """
        # Priority queue to store (distance, node)
        heap = [(0, start)]
        # Dictionary to store shortest distances from start to each node
        distances = {start: 0}
        # Dictionary to store the shortest path to each node
        paths = defaultdict(list)
        paths[start] = [start]
        while heap:
            current_distance, node = heapq.heappop(heap)
            # If we reach the goal, return the path to it
            if node == goal:
                return paths[node]
            # Skip nodes that have already been processed
            if current_distance > distances.get(node, float('inf')):
                continue
            # Explore neighbors
            for neighbor, weight in graph.get(node, []):
                new_distance = current_distance + weight
                # If a shorter path to the neighbor is found
                if new_distance < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_distance
                    paths[neighbor] = paths[node] + [neighbor]
                    heapq.heappush(heap, (new_distance, neighbor))
        # If goal is not reachable, return None
        return None

##################################################
# ---------------------------------------------- #
#     LANDMARKING: SAVE CURRENT LOCATION         #
# ---------------------------------------------- #
##################################################

# ----------------------------------------------------------------------------------------------------
# ------------------------------ fm_add_landmark_request  -------------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def fm_add_landmark_request(self, station_type, input_loc_id, neighbor_loc_ids):
        """ fm_add_landmark_request """

        self.fm_fetch_active()

        if robot_id == 0:
                print("[manager]: Robot is not selected. Please choose robot.")
                return

        if len(self.feedback_current_pose) < 1:
               return

        self.o_q = self.euler_to_quaternion(float(self.feedback_current_pose[2]), 0.0, 0.0)
        x = round(float(self.feedback_current_pose[0]), 2)
        y = round(float(self.feedback_current_pose[1]), 2)
        z = round(float(self.o_q[2]), 2)
        w = round(float(self.o_q[3]), 2)

        print("[manager]:-location or landmark to save: \n")
        print("[manager]:-x: "+str(x)+", y: "+str(y)+", z: "+str(z)+", w: "+str(w))
        print("[manager]:- ")

        # Define the regular expression pattern
        pattern = re.compile(r'^[A-Z][1-9]\d*|^[A-Z]10\d*$')
        # Check if input_loc_id matches the pattern
        if not pattern.match(input_loc_id):
                print('[manager]: Location Number 0 cannot be chosen. Start numbering from 1. \
                        Only single capital lettered alphabets can be used. E.g. A1, E54 etc.')
                return

        else: # check if its already in the landmarks
            # Check if the location ID already exists in the itinerary
            for entry in task_dictionary.get("itinerary", []):
                if entry.get("fleet_id") == fleet_id and entry.get("loc_id") == input_loc_id:
                    print('[manager]: Location ID already exists.')
                    return

        # choose landmark neighbours for edge list
        # input_str = "A4,C2,C3,A6, D2,A1, A4"
        # remove spaces and split the string into a list of edges
        edges = neighbor_loc_ids.split(",")
        # define the regular expression pattern
        pattern = re.compile(r'^[A-Z]\d+$')
        # check if each element matches the pattern
        for element in edges:
            if not pattern.match(element.strip()):  # strip() is used to remove leading/trailing spaces
                print(f'[manager]: Invalid element in "Neighbors" field: {element}.')
                return
            if element not in self.landmark_dict['t']:
                print(f'[manager]: Element {element} in "Neighbors" field is not a landmark.')
                return

        self.add_location_db_cmd(edges, x, y, w, z, station_type, input_loc_id)

        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------

    def add_location_db_cmd(self, edges, x, y, w, z, loc_type, node):
        """ add_location_db_cmd """

        temp_dict = {   "loc_id": node,
                        "fleet_id": fleet_id,
                        "description": loc_type,
                        "coordinate": [x, y, w, z]   }

        # more like if node == 'A1': then edge_list = [('A2', 1.0)] <-- just to initialize or prime the edge list.
        # the A2 will subsequently be updated with its cost once we actually save an A2 node.
        # Again, users need to name the nodes alphabetically and serially
        edge_list = self.update_edge_list(node, edges, x, y)

        # update the edges and cost of the listed neighbours
        self.add_node_and_edges(node, edge_list)

        # convert the graph to the desired YAML format update the 'graph' entity in the dictionary
        global task_dictionary
        task_dictionary['graph'] = {} # clear the old graph
        for node, edges in graph.items():
                yaml_edges = []
                for edge in edges:
                        yaml_edges.append([edge[0], edge[1]])
                task_dictionary['graph'][node] = yaml_edges

        # Update the 'nodes' to include the recently added node
        task_dictionary["itinerary"].append(temp_dict)

        # Write the updated YAML back to the file
        self.dump_to_yaml(task_dictionary)

        # give the computer some time to breathe
        self.busy_wait(0.05)
        print('[manager]:-Add landmark completed successfully! Modify edges in the output config file (YAML).')


    def update_edge_list(self, node, edges, x, y):
        """
        Update the edge list for a given node based on its neighbors and coordinates.
        """
        edge_list = []
        # Check if node matches the pattern of a single capital letter followed by one or more digits
        if re.match(r'^[A-Z]\d+$', node):
            letter = node[0] # Extract the letter and the number from the node
            number = int(node[1:])
            next_node = f'{letter}{number + 1}' # Create the next node by incrementing the number
            edge_list = [(next_node, 1.0)]
        else:
            for entry in task_dictionary["itinerary"]:
                if entry["fleet_id"] == fleet_id:
                    if entry["loc_id"] in edges: # we need to get the coordinates of the neighbours listed
                        coordinate = entry["coordinate"] # lets set it to coordinate yeah
                        # obtain the distance from this new landmark/location to the neighbour and use it as the cost.
                        distance = round(math.sqrt((float(coordinate[0]) - x)**2 + (float(coordinate[1]) - y)**2), 2)
                        edge_list.append((entry["loc_id"], distance)) # update the new landmarks edge list with the (neighbour,cost)
        return edge_list

        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------

    def add_node_and_edges(self, node, edges):
        """ graph work:  edit, update etc. graph content. """
        global graph
        # print("node: ", node); print("edges: ", edges); print("graph: ", graph)
        # add the node to the graph, if not already present
        if node not in graph:
            graph[node] = []
        for edge in edges:
            # split the tuple into neighbor and cost
            neighbor, cost = edge
            cost = float(cost)
            # check if edge already exists
            existing_edge = None
            for i, n in enumerate(graph[node]):
                if n[0] == neighbor:
                    existing_edge = i
                    break
            # if edge exists, update the cost
            if existing_edge is not None:
                graph[node][existing_edge][1] = cost
                # update the neighbor's edge as well
                for i, n in enumerate(graph[neighbor]):
                    if n[0] == node:
                        graph[neighbor][i][1] = cost
            # if edge does not exist, add the edge
            else:
                graph[node].append([neighbor, cost])
                # add the node to the neighbor's neighbors, if not already present
                if neighbor not in graph:
                    graph[neighbor] = set()
                if node not in [n[0] for n in graph[neighbor]]:
                    graph[neighbor].add((node, cost))

    # --------------------------------------------------------------------------------------------

    def euler_to_quaternion(self, yaw, pitch, roll):
        """ euler_to_quaternion """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

##################################################
# ---------------------------------------------- #
#     CLEAR TASK/IDLE ROBOT                      #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ---------------------------------- fm_clear_task_trigger -------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def fm_clear_task_trigger(self, trigger:bool):
        """ fm_clear_task_trigger """

        self.fm_fetch_active()

        if robot_id == 0:
            print('[manager]: Robot is not selected. Please choose robot.')
            return

        if self.feedback_checkpoints[0] == 'A' or  self.feedback_checkpoints[0] == 'A' or self.feedback_checkpoints[0] == 'A':
            print('[manager]: Robot not currently executing any task. Operation not performed.')
            return

        if trigger is True:
            self.clear_task_db_cmd('idle')
            self.busy_wait(0.05)
            self.clear_task_db_cmd('inactive')

    # active,x,y,z,w,base_floor
    def clear_task_db_cmd(self, whattodo):
        """ clear_task_db_cmd """
        try:
            floor = self.feedback_agv_status[5] # flag = self.feedback_agv_status[6]
            pseudo_itinerary = [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]

            self.cur.execute('UPDATE table_robot SET agv_status = %s, last_checkpoint = %s, \
                checkpoints = %s, waitpoints = %s, agv_itinerary = %s, wait_itinerary = %s \
                WHERE robot_id = %s and fleet_id = %s',
                (whattodo+',x,y,z,w,'+floor+',red', 'unknown', 'A,A,A', 'W,W,W',
                 pseudo_itinerary, pseudo_itinerary, robot_id, fleet_id,))
            self.conn.commit()
        except Exception as error:
            print("[manager]:-clear_task_db_cmd-"+str([error])+".")

##################################################
# ---------------------------------------------- #
#   ROBOT STATUS  (ACTIVE OR INACTIVE)           #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# --------------------------------- fm_robot_status_trigger ------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def fm_robot_status_trigger(self, trigger:bool):
        """ fm_robot_status_trigger """

        self.fm_fetch_active()

        if robot_id == 0:
            print('[manager]: Robot is not selected. Please choose robot.')
            return
        try:
            self.cur.execute('SELECT * FROM table_robot WHERE robot_id = %s and fleet_id = %s;', (robot_id, fleet_id,))
            for record in self.cur.fetchall():
                _agv_stat = str(record['agv_status']).split(',')
                if (trigger is False) and (_agv_stat[0] == 'active'):
                    self.robot_status_db_cmd('inactive')
                elif (trigger is True) and (_agv_stat[0] == 'inactive'):
                    self.robot_status_db_cmd('active')
                else:
                    pass
        except Exception as error:
                print("[manager]:-fm_robot_status_trigger-"+str([error])+".")

    def robot_status_db_cmd(self, agv_stat):
        """ robot_status_db_cmd """
        self.feedback_agv_status[0] = agv_stat
        try:
            self.cur.execute('UPDATE table_robot SET agv_status = %s WHERE robot_id = %s and fleet_id = %s', (','.join(self.feedback_agv_status), robot_id, fleet_id,))
            self.conn.commit()
            print('[manager]:-Robot Status State: ', str(agv_stat))
        except Exception as error:
            print("[manager]:-robot_status_db_cmd-"+str([error])+".")

##################################################
# ---------------------------------------------- #
#     MANUAL CONTROL                             #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ---------------------------------- send_control_nav  -----------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def fm_manual_control_trigger(self, trigger: bool):
        """ fm_manual_control_trigger """

        self.fm_fetch_active()

        if robot_id == 0:
            print('[manager]: Robot is not selected. Please choose robot.')
            return

        try:
            self.cur.execute('SELECT * FROM table_robot WHERE robot_id = %s and fleet_id = %s;',(robot_id, fleet_id))
            record = self.cur.fetchone()
            if record:
                m_controls = record['m_controls'].split(',')
                if not trigger and m_controls[0] == 'yes':
                    self.manual_cntrl_db_cmd('no')
                    print('[manager]:-Success! manual drive mode is turned off.')
                elif trigger and m_controls[0] == 'no':
                    self.feedback_agv_status[0] = 'active'
                    self.cur.execute('UPDATE table_robot SET agv_status = %s, checkpoints = %s, last_checkpoint = %s WHERE robot_id = %s and \
                        fleet_id = %s',(','.join(self.feedback_agv_status), 'A,A,A', 'unknown', robot_id, fleet_id))
                    self.conn.commit()
                    self.manual_cntrl_db_cmd('yes')

        except Exception as error:
            print(f"[manager]:-fm_manual_control_trigger-{error}.")

    def fm_manual_control_drive(self, k_pressed):
        """ fm_manual_control_drive  """

        self.fm_fetch_active()

        return self.manual_cntrl_db_cmd(k_pressed)

    def manual_cntrl_db_cmd(self, k_pressed):  # yes False, False, False, False -- on/off left forward right stop - m_controls
        """Send manual control commands to the database."""
        if k_pressed not in ['yes', 'no', 'left', 'forward', 'right', 'stop', 'backward']:
            print("[manager]:-manual_cntrl_db_cmd- Invalid command.")
            return

        try:
            if k_pressed in ['yes', 'no']:
                self.control_command = f'{k_pressed},False,False,False,False,False'
            else:
                # Retrieve the current m_controls value
                self.cur.execute('SELECT m_controls FROM table_robot WHERE robot_id = %s AND fleet_id = %s;', (robot_id, fleet_id))
                record = self.cur.fetchone()

                if record:
                    m_controls = record['m_controls'].split(',')
                    if m_controls[0] == 'yes':
                        self.control_command = self.get_control_command(k_pressed)
                    else:
                        print("[manager]:-manual_cntrl_db_cmd- Robot not in 'yes' state.")
                        return

            # Update the database with the new control command
            self.cur.execute('UPDATE table_robot SET m_controls = %s WHERE robot_id = %s AND fleet_id = %s;', (self.control_command, robot_id, fleet_id))
            self.conn.commit()

        except Exception as error:
            print(f"[manager]:-manual_cntrl_db_cmd- Error: {error}.")

    def get_control_command(self, k_pressed):
        """Generate the control command based on the key pressed."""
        commands = {
            'left': 'yes,True,False,False,False,False',
            'forward': 'yes,False,True,False,False,False',
            'right': 'yes,False,False,True,False,False',
            'stop': 'yes,False,False,False,True,False',
            'backward': 'yes,False,False,False,False,True'
        }
        return commands.get(k_pressed, 'yes,False,False,False,False,False')












##################################################
# ---------------------------------------------- #
#   SHUTDOWM  (ON OR OFF)                        #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ---------------------------------- fm_shutdown_trigger  -------------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def fm_shutdown_trigger(self, trigger:bool):
        """[API] True/False - shutdown"""

        self.fm_fetch_active()

        try:
            self.cur.execute('SELECT * FROM table_robot WHERE robot_id = %s and fleet_id = %s;', (robot_id, fleet_id,))
            for record in self.cur.fetchall():
                if (trigger is True) and (str(record['shutdown']) == 'no'):
                    # set robot to inactive
                    self.robot_status_db_cmd('inactive')
                    # wait some scnds
                    self.busy_wait(0.05)
                    # shut it down
                    self.shutdown_db_cmd('yes')
                elif (trigger is False) and (str(record['shutdown']) == 'yes'):
                    # set robot to active
                    self.robot_status_db_cmd('active')
                    # wait some scnds
                    self.busy_wait(0.05)
                    # turn it on
                    self.shutdown_db_cmd('no')
                else:
                    pass
        except Exception as error:
            print("[manager]:-fm_shutdown_trigger-"+str([error])+".")

    def shutdown_db_cmd(self, shutdown):
        """ # yes/no - shutdown """
        try:
            self.cur.execute('UPDATE table_robot SET shutdown = %s WHERE robot_id = %s and fleet_id = %s;', (shutdown, robot_id, fleet_id,))
            self.conn.commit()
            print('[manager]:-Robot Power State-', str(shutdown))
        except Exception as error:
            print("[manager]:-shutdown_db_cmd-"+str([error])+".")


##################################################
# ---------------------------------------------- #
#      SETTINGS:  (Delete landmark)              #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ----------------------------------  Delete landmark ------------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def fm_delete_landmark_request(self, str_location_id_comma_fleet):
        """ fm_delete_landmark_request """

        global task_dictionary

        self.fm_fetch_active()

        parts = str_location_id_comma_fleet.split(',')
        loc_, fleet_ = parts[0], parts[1]

        if (loc_ is None) or (fleet_ is None):
                print("[manager]: Invalid element in 'delete landmark' field.")
                return

        for i in range(0,len(task_dictionary["itinerary"])):
                if task_dictionary["itinerary"][i]["fleet_id"] == fleet_:
                        if task_dictionary["itinerary"][i]["loc_id"] == loc_:
                                del task_dictionary["itinerary"][i]
                                break

        if loc_ in task_dictionary["graph"]:
                del task_dictionary["graph"][loc_]

        for node, edges in task_dictionary['graph'].items():
                task_dictionary['graph'][node] = [edge for edge in edges if edge[0] != loc_]

        self.dump_to_yaml(task_dictionary)

        self.busy_wait(0.05)

##################################################
# ---------------------------------------------- #
#      SEND SMS NOTOFICATION                     #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ----------------------------------- check_notification_msgs  ---------------------------------------
# ----------------------------------------------------------------------------------------------------
    def check_notification_msgs(self, f_id=None):
        """ check all fleets, is there any robot requiring human intervention?
            twilio_server: -- startup msgs -- system error msg -- dock related/occupied msg -- reverse/recovery msg """

        if f_id is None or self.send_sms is False:
            return

        try:
            self.cur.execute("SELECT notifications, robot_id FROM table_robot WHERE fleet_id = %s;", (f_id,))
            for record in self.cur.fetchall():
                notifications_ = str(record['notifications']).split(',')
                r_id_ = str(record['robot_id'])
                # print("notifications: ", r_id_, notifications_[0], notifications_[1])
                if notifications_[0] != notifications_[1]:

                    account_sid = str(task_dictionary["twilio_server"]["account_sid"])
                    auth_token = str(task_dictionary["twilio_server"]["auth_token"])
                    from_number = str(task_dictionary["twilio_server"]["from_number"])
                    to_number = str(task_dictionary["twilio_server"]["to_number"])
                    client = Client(account_sid, auth_token)
                    message = client.messages.create(
                        to = to_number,
                        from_ = from_number,
                        body = "Hello! Robot "+r_id_+" in fleet "+f_id+" requires attention "+notifications_[0]+". Please inspect. ")
                    # print(message.sid, "--> account_sid: ",account_sid, "auth_token: ",auth_token, "from_number: ",from_number, "to_number: ",to_number)
                    print("[manager]:-twilio sms auth_token verified.")

                    notifications_[1] = notifications_[0]
                    self.cur.execute('UPDATE table_robot SET notifications = %s WHERE robot_id = %s;', (','.join(notifications_), r_id_,))
                    self.conn.commit()

        except (ValueError, Exception, TypeError) as error:
                print("[manager]:-check_notification_msgs-"+str([error])+".")

# ---------------------------------------------- #
#      MAIN                                      #
# ---------------------------------------------- #

if __name__ == "__main__":
        Ui_MainWindow()
