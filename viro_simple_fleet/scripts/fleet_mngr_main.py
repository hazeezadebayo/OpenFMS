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
        self.show_robot_animation = True
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
        self.nav_map_obtained = False
        # initialize empty fleet and robot ids.
        self.fleet_ids = []
        self.robot_ids = []
        self.send_sms = False
        # initialize fleet dropdown menu
        self.available_robots = []
        self.decision = ['yes', 'no'] # terminal interaction
        self.job_types =['transport', 'loop', 'move', 'charge', 'clean']
        self.job_priorites = ['low', 'high', 'medium']
        self.station_type = ['charge_dock', 'station_dock', 'home_dock', 'elevator_dock', 'checkpoint', 'waitpoint']
        # initialize co-ordinates! this is for plotting on matplotlib
        self.agv_track_dict = {"x":[], "y":[],}
        # initialize job ids dropdown menu
        self.job_ids = []
        self.landmark_dict = {"x":[], "y":[], "t":[], "k":[],}
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
        self.timer_interval = 50.0
        self.timer_thread = None
        self.timer_thread_running = threading.Event()
        # If a timer thread is already running, stop it
        if self.timer_thread is not None and self.timer_thread.is_alive():
                self.timer_thread_running.clear()
                self.timer_thread.join()
        # Update the timer interval
        self.timer_thread_running.set()
        # self.timer_thread = threading.Thread(target=self.start_timer, args=(robot_id,), daemon=True)
        self.timer_thread = threading.Thread(target=self.start_timer, daemon=True)
        self.timer_thread.start()
        # main loop
        while True:
            self.j += 1
            if self.j % 37 == 0 and \
                len(self.fleet_ids) != 0 and \
                    len(self.robot_ids) != 0:
                r_id, f_id = robot_id, fleet_id
                # load robot traffic mngr display callback
                self.on_timer(f_id, r_id)
                if self.j > 100: self.j = 0

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
    def start_timer(self):
        while True:
            self.i += 1
            if self.i % self.timer_interval == 0:
                self.interactive_robot_fleet_startup()
                if self.i > 100: self.i = 0


    def interactive_robot_fleet_startup(self):
        """
        [input] graph data
        [output] new json immediately for use.
        """
        # set the fleet_id and robot_id
        self.interactive_start_robot_fleet()
        # Create and start the timer

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
                        self.fm_manual_control_drive('stop', robot_id, fleet_id)
                        return
                    else:
                        self.fm_manual_control_drive(chosen_option, robot_id, fleet_id)
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
                print('ATTENTION! '+"Field 'Neighbors' cannot be left empty. Please choose appropriate connected checkpoints.")

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

    def interactive_start_robot_fleet(self):
                """
                from Terminal interaction,
                [input] fleet_id, robot_id
                """
                global fleet_id, robot_id

                # get all fleet ids.
                self.fleet_ids = []
                try:
                        self.cur.execute("SELECT DISTINCT fleet_id FROM table_robot") # ORDER BY something ascending descending etc
                except psycopg2.errors.UndefinedTable:
                        print('ATTENTION! The table "table_robot" does not exist.')
                        print('please startup a client robot first for a usable ui.')
                        exit(0)
                self.fleet_sql = self.cur.fetchall()
                for i in self.fleet_sql:
                        self.fleet_ids.append(i[0])

                # request for a particular fleet_id
                f_id = input('Please enter fleet_id '+str(self.fleet_ids)+': ')
                while f_id not in self.fleet_ids:
                        print('Invalid input...')
                        f_id = input('Please enter fleet id: ')

                fleet_id = f_id
                self.fm_start_robot_fleet(fleet_id)

                answer = input('\nAlert! Do you want to assign a robot a task? [yes, no]: ')
                while answer not in self.decision:
                        print('Invalid input. The decision must be one of the following: yes, no.')
                        answer = input('Please enter decision [yes, no]: ')
                if answer == 'yes':
                        # Ask for a particular robot by robot_id
                        robot_id = input('Please enter int robot_id: '+str(self.robot_ids)+'.')
                        while robot_id not in self.robot_ids:
                                print('Invalid input. The robot id must be one of the following integers: '+str(self.robot_ids)+'.')
                                robot_id = input('Please enter robot id: ')
                        self.single_robot_view = True
                        # self.on_timer(fleet_id, robot_id)
                else:
                        robot_id = 0
                        self.single_robot_view = False
                        # self.on_timer(fleet_id)

    def fm_start_robot_fleet(self, f_id):
                """
                [API] call with fleet_id.
                pass a specific fleet id, then it populates to memory all robot from that fleet.
                it also pupulates what jobs/itinerary/landmarks are available to this specific fleet
                """

                global fleet_id
                fleet_id = f_id

                # get all fleet ids.
                self.fleet_ids =[]
                try:
                        self.cur.execute("SELECT DISTINCT fleet_id FROM table_robot") # ORDER BY something ascending descending etc
                except psycopg2.errors.UndefinedTable:
                        print('ATTENTION! The table "table_robot" does not exist.')
                        return
                self.fleet_sql = self.cur.fetchall()
                for i in self.fleet_sql:
                        self.fleet_ids.append(i[0])

                if fleet_id not in self.fleet_ids:
                        print("Invalid input. The fleet id must be one of the following integers:",str(self.fleet_ids))
                        return

                temp_type = None
                for i in range(0,len(task_dictionary["itinerary"])):
                        if task_dictionary["itinerary"][i]["fleet_id"] == fleet_id:
                                if (task_dictionary["itinerary"][i]["description"] == 'station_dock' or \
                                        task_dictionary["itinerary"][i]["description"] == 'charge_dock' or \
                                        task_dictionary["itinerary"][i]["description"] == 'elevator_dock' or \
                                                task_dictionary["itinerary"][i]["description"] == 'home_dock'):
                                        self.job_ids.append(task_dictionary["itinerary"][i]["loc_id"])

                                self.landmark_dict['x'].append(float(task_dictionary["itinerary"][i]["coordinate"][0]))
                                self.landmark_dict['y'].append(float(task_dictionary["itinerary"][i]["coordinate"][1]))
                                self.landmark_dict['t'].append(task_dictionary["itinerary"][i]["loc_id"])

                                if (task_dictionary["itinerary"][i]["description"] == 'charge_dock'): temp_type = 'green'
                                elif (task_dictionary["itinerary"][i]["description"] == 'home_dock'): temp_type = 'blue'
                                elif (task_dictionary["itinerary"][i]["description"] == 'station_dock'): temp_type = 'cyan'
                                elif (task_dictionary["itinerary"][i]["description"] == 'elevator_dock'): temp_type = 'orange'
                                elif (task_dictionary["itinerary"][i]["description"] == 'waitpoint'): temp_type = 'springgreen'
                                else: temp_type = 'yellow'

                                self.landmark_dict['k'].append(temp_type)

                print("\nvalid landmark ids that can be used for jobs are:")
                print("[landmark_ids]: ", self.job_ids)

                # get all robots in this fleet_id
                self.robot_ids = []
                self.cur.execute("SELECT robot_id FROM table_robot WHERE fleet_id = %s;", (fleet_id,))
                self.robot_sql = self.cur.fetchall()
                for i in self.robot_sql:
                        self.robot_ids.append(str(i[0]))

                print("\nvalid robot ids that can be used here are:")
                print("[robot_ids]: ", self.robot_ids)

                # plot the world first. then we can plot the car/robot in the timed function.
                self.nav_map_obtained = False

##################################################
# ---------------------------------------------- #
#      on_timer callback ( AGV update)           #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ------------------------------- SINGLE AGV VISUALIZATION -------------------------------------------
# ----------------------------------------------------------------------------------------------------

    # Function to map priority strings to integers
    def map_priority(self, priority):
        mapping = {'low': 1, 'medium': 2, 'high': 3}
        return mapping.get(priority, 0)  # Default to 0 if priority is not recognized

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
        return time_to_reach

    def db_list_to_pylist(self, dblist):
        """
        Helper function to convert db string in the form of dictionary to list.
        """
        # Initialize the result list
        result = []
        # Iterate over the dblist sublists
        for sublist in dblist:
            # Strip the curly braces from the sublist
            sublist = sublist.strip("{}")
            # Split the sublist on "," to get the individual elements
            elements = sublist.split(",")
            # Convert the elements to floats
            elements = [float(x) for x in elements]
            # Append the elements to a new list
            new_list = elements
            # Append the new list to the result list
            result.append(new_list)
        # Print or return the result
        return result # print(result)


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


    def on_timer(self,  f_id=None, r_id=0):
        # print("---------u--------", f_id, "--", rbt_id)
        if (f_id is None) and (len(self.robot_ids) == 0):
            return

        # --------------

        for _r_id in self.robot_ids:

            # --------------
            # Initialize lists for storing information
            traffic_control = []
            feedback_last_checkpoint = ['unknown',]

            # --------------
            try:
                # Fetch all robots in the fleet
                self.cur.execute('SELECT * FROM table_robot WHERE robot_id = %s and fleet_id = %s;', (_r_id, f_id,))
                # self.cur.execute("SELECT * FROM table_robot WHERE fleet_id = %s;", (f_id,))
                records = self.cur.fetchall()
                # Process each robot's data
                for record in records:
                    if (r_id != 0) and (_r_id == str(r_id)):
                        # Fetch detailed information for the specific robot
                        self.feedback_current_pose = str(record['current_pose']).split(',') # print(self.feedback_current_pose[0], self.feedback_current_pose[1], self.feedback_current_pose[2])
                        self.feedback_notifications = str(record['notifications']).split(',') # print(self.feedback_notifications[0], self.feedback_notifications[1], self.feedback_notifications[2], self.feedback_notifications[3])
                        self.feedback_last_checkpoint = str(record['last_checkpoint']).split(',')
                        self.feedback_checkpoints = str(record['checkpoints']).split(',') # print(len(self.feedback_checkpoints),"-route: ", self.feedback_checkpoints) # [A,B,C,D]
                        self.feedback_waitpoints = str(record['waitpoints']).split(',')
                        self.feedback_agv_status = str(record['agv_status']).split(',') # print(self.feedback_agv_status)
                        self.feedback_shutdown = str(record['shutdown']) # print(self.feedback_shutdown)
                        self.feedback_controls = str(record['m_controls']).split(',') # print(self.feedback_controls[0], self.feedback_controls[1], self.feedback_controls[2], self.feedback_controls[3])
                        self.feedback_model_config = str(record['model_config']).split(',') # print(self.controls[0], self.controls[1], self.controls[2], self.controls[3])
                        self.feedback_landmarks = str(record['landmark']).split(',')
                        self.feedback_base_map_img = record['base_map_data']
                        self.feedback_floor1_map_img = record['floor1_map_data']
                        self.feedback_agv_itinerary = self.db_list_to_pylist(str(record['agv_itinerary']).split("},{"))
                        self.feedback_traffic = str(record['traffic'])
                        self.feedback_wait_traffic = str(record['wait_traffic'])
                        # continue  # Skip usual processing for this robot as we already fetched detailed data
                    else:
                        self.feedback_base_map_img = record['base_map_data']

                    # Fetch detailed information for the specific robot
                    feedback_current_pose = str(record['current_pose']).split(',') # print(self.feedback_current_pose[0], self.feedback_current_pose[1], self.feedback_current_pose[2])
                    feedback_notifications = str(record['notifications']).split(',') # print(self.feedback_notifications[0], self.feedback_notifications[1], self.feedback_notifications[2], self.feedback_notifications[3])
                    feedback_last_checkpoint = str(record['last_checkpoint']).split(',')
                    feedback_checkpoints = str(record['checkpoints']).split(',') # print(len(self.feedback_checkpoints),"-route: ", self.feedback_checkpoints) # [A,B,C,D]
                    feedback_waitpoints = str(record['waitpoints']).split(',')
                    feedback_agv_status = str(record['agv_status']).split(',') # print(self.feedback_agv_status)
                    # feedback_shutdown = str(record['shutdown']) # print(self.feedback_shutdown)
                    # feedback_controls = str(record['m_controls']).split(',') # print(self.feedback_controls[0], self.feedback_controls[1], self.feedback_controls[2], self.feedback_controls[3])
                    feedback_model_config = str(record['model_config']).split(',') # print(self.controls[0], self.controls[1], self.controls[2], self.controls[3])
                    feedback_landmarks = str(record['landmark']).split(',')
                    # feedback_base_map_img = record['base_map_data']
                    # feedback_floor1_map_img = record['floor1_map_data']
                    feedback_agv_itinerary = self.db_list_to_pylist(str(record['agv_itinerary']).split("},{"))
                    feedback_traffic = str(record['traffic'])
                    feedback_wait_traffic = str(record['wait_traffic'])

                # Fetch distinct traffic information
                # self.cur.execute("SELECT DISTINCT traffic FROM table_robot WHERE fleet_id = %s;", (f_id,))
                # for i in self.cur.fetchall():
                #     traffic_control.append(i[0])

                # Fetch distinct traffic information
                self.cur.execute("SELECT DISTINCT traffic, wait_traffic FROM table_robot WHERE fleet_id = %s;", (f_id,))
                for row in self.cur.fetchall():
                    for value in row:
                        if value != 'none':
                            traffic_control.append(value)

            except Exception as error:
                print("[traffic]:-select statement-" + str([error]) + ".")
            # --------------

            # --------------
            # plot the world
            self.get_world(f_id)
            # plot the car
            self.nav_plot(f_id)
            # general: notify user if any problem in any fleet's robot
            self.check_notification_msgs(f_id)
            # --------------

            # --------------
            # function for traffic management.
            # Further processing for each robot
            #for i,  in enumerate(_r_id):
            # [a,b,c,d,e,f,g,a] = chekpoints
            # [c,d,e,f,g,a] = last_checkpoint
            # if robot reaches c and wants to go to d,
            # it would have moved some 'dist_to_next' before we check its reservation.
            # in the mean time:
            # [d,e,f,g,a] = last_checkpoint
            # with d = 'next_stop_id' for analysis.
            next_stop_id = feedback_last_checkpoint[0] # the place the robot desires to go
            # if no task is assigned to this particular robot:
            if next_stop_id == 'unknown':
                continue
            ros_pose_x, ros_pose_y = feedback_current_pose[0], feedback_current_pose[1]

            # predecessor_landmark is like the gate of the dock station. so like a point just in front of the landmark/dock
            # where we have to pass first in order to get to the dock or real_landmark itself.
            # Determine the predecessor and real landmarks
            # landmark = [task_priority 0, task_name 1, pick 2, drop 3, home_docks is 4 upwards 5 6 7 etc.]
            predecessor_landmark = []
            real_landmark = []
            for elements in feedback_landmarks[2:]:
                predecessor_landmark.append(elements.split('_')[0])
                real_landmark.append(elements.split('_')[1])

            # if where agv is going does infact exist, in our memory of paths or checkpoints,
            # use it to return the real coordinates as next_stop_coordinate e.g d = [1.0, 2.0, 0.0, 0.0] == [x,y,z,w].
            next_stop_coordinate = feedback_agv_itinerary[feedback_checkpoints.index(next_stop_id)]
            # print("[traffic]:-"+str([_r_id])+" next_stop_coordinate-" + str([next_stop_coordinate]) + ".")

            # Calculate distance to next stop
            dist_to_next = math.sqrt((float(next_stop_coordinate[0]) - float(ros_pose_x))**2 + (float(next_stop_coordinate[1]) - float(ros_pose_y))**2)
            # print("[traffic]:-"+str([_r_id])+" dist_to_next-" + str([dist_to_next]) + ".")

            try:
                # since we would not be checking the robots reservation until some 'dist_to_next' away from the next target,
                # it is safe to assume that at this point, traffic or reserved checkpoint is still 'c';
                # this robot's current reserved checkpoint is ...
                reserved_checkpoint = feedback_traffic

                # print("[traffic]:-"+str([_r_id])+" reserved_checkpoint-" + str([reserved_checkpoint]) + ".")
                # if this robot is not the one currently occupying where it wants to go to.
                # That is, if 'c' != 'd' and we are now so close to reaching 'd', we must make a reservation:
                # if distance to next is less than robot length plus some tolerance value. then make reservation.
                if (reserved_checkpoint != next_stop_id) and (dist_to_next < (float(feedback_model_config[0])+2.7)):
                    # we need to check all the traffic with our own to be sure that,
                    # where we wanna are headed has not been reserved or occupied by another robot.
                    print("[traffic]:-traffic_control-" + str([traffic_control]) + ".")
                    if next_stop_id in traffic_control:
                        # oh shit! it is occupied, we need to stop our robot.
                        print("[traffic]: "+str([_r_id])+" next id occupied. ")
                        # first, is our robot moving?, pause the robot.
                        if feedback_agv_status[0] == 'active':

                            # [ROBOT ACTIVE] CASE 1:
                            #################################################################################################

                            # if 'd' is a dock station/landmark of any type and
                            # feedback notification 4 being none is essentially code for waiting area not yet utilized. then,
                            if next_stop_id in real_landmark and feedback_notifications[4] == "None":

                                # check what kind of landmark or dock station we are headed for precisely,
                                # so we can decide what to do.
                                if feedback_landmarks[1] == "transport": # three landmarks in transport task - (pick, drop, home)
                                    # if it is home from (pick, drop, home) in Transport task, and it is not the last on the list,
                                    # meaning it appears someone or some robot as docked in the home location we wanna go, but
                                    # we realize that there are more homes to go to, i.e. home is not just one id 4, 5 6 7 exists
                                    # so we try all of them in that order, till we find an un-isrealed one.
                                    # we may even re-write this statement as:
                                    # " if next_stop_id in self.landmarks[4:] and next_stop_id != self.landmarks[-1] "
                                    if next_stop_id in real_landmark[2:] and next_stop_id != real_landmark[-1]:
                                        # That is, we need to skip and go to another home dock so flag is green but skip please.
                                        feedback_agv_status[6] = 'green' # flag
                                    else:
                                        # if 'd' i.e. next_stop_id is the last of the home docks in the transport task
                                        # then we have a problem, we need to raise an alarm.
                                        if next_stop_id == real_landmark[-1]:
                                            # send text message or notification
                                            feedback_notifications[0] = "Inactive: all home_dock occupied."
                                        # if we were not already waiting before, we need to start waiting till that checkpoint is free.
                                        if feedback_notifications[3] == '0':
                                            # please let the world know what time we made the decision to wait yeah.
                                            # so we can know how long we would be here for
                                            feedback_notifications[3] = time.time_ns()
                                        # we might even raise alarm if we had one
                                        # stop the robot
                                        feedback_agv_status[6] = 'red' # flag [output]
                                        feedback_notifications[3] = str(time.time_ns() - int(feedback_notifications[3]))

                                elif feedback_landmarks[1] in ["loop", "move", "charge"]:
                                    # charge is like transport in that if it doesnt find one charge station it tries to go to the
                                    # next available charge station. however, it does not have pick, drop etc. unlike transport.
                                    # therefore, if [d,e,f,g,a] = last_checkpoint is > 1, implying that there are more locations
                                    # to visit and as such this is not the last charge dock and this is actually a charge task:
                                    if (len(feedback_last_checkpoint) > 1) and (feedback_landmarks[1] == "charge"):
                                        # again, we need to skip and set flag to green so that we drive off.
                                        feedback_agv_status[6] = 'green' # flag
                                    else:
                                        # else, if its a charge task, but we are at the last charge dock then problem...
                                        if (len(feedback_last_checkpoint) == 1) and (feedback_landmarks[1] == "charge"):
                                            # send text message or notification
                                            feedback_notifications[0] = "Inactive: all charge_dock occupied."
                                        # if we had a 'loop' or 'move' or 'charge as last dock' here, then wait...
                                        if feedback_notifications[3] == '0':
                                            feedback_notifications[3] = time.time_ns()
                                        feedback_agv_status[6] = 'red' # flag
                                        feedback_notifications[3] = str(time.time_ns() - int(feedback_notifications[3]))

                            # elif 'd' is a not dock station/landmark of any type that is, its just a normal checkpoint.
                            # this is same as saying if we are at any important place, we wait.
                            # actually we could calculate alternate route with DFS and send it to robot. but thats stressful...
                            # predecessor_landmark also fall in this category. which imply gates.
                            elif next_stop_id not in real_landmark and feedback_notifications[4] == "None":
                                if feedback_notifications[3] == '0':
                                    feedback_notifications[3] = time.time_ns()
                                feedback_agv_status[6] = 'red' # flag
                                feedback_notifications[3] = str(time.time_ns() - int(feedback_notifications[3]))

                            # Finally, so what is our concensus? whats the agreed flag?
                            # the only instances where we had 'green' was when one docking "home, charge" station is blocked,
                            # other stations exist and we can go check their availability. therefore, skip instances yeah?
                            if feedback_agv_status[6] == 'green':
                                # we might have to skip our target node 'd' for the next 'e'
                                feedback_notifications[4] = "skip"
                                # over-write next_stop_id so that we have 'e' as our new target and we can proceed to check its availability
                                next_stop_id = feedback_last_checkpoint[1]
                                # if 'e' is not occupied and it is not a dock station, meaning if its a normal checkpoint, we move to it!
                                if (next_stop_id not in traffic_control) and (next_stop_id not in real_landmark[2:]):
                                    # feedback_notifications[4] = "skip"
                                    # reserve 'e' in traffic for this robot and set flag to green so it goes there
                                    self.cur.execute('UPDATE table_robot SET notifications = %s, traffic = %s WHERE robot_id = %s and fleet_id = %s;',
                                        (','.join(feedback_notifications), next_stop_id, _r_id, f_id,))
                                # if 'e' is occupied. now we must still skip but we gonna wait forever.
                                # we handle this down the code! line 788 upwards or so...
                                else:
                                    # if 'e' which is our new target is occupied, we start waiting...
                                    if feedback_notifications[3] == '0':
                                        feedback_notifications[3] = time.time_ns()
                                    # also flag is red
                                    feedback_agv_status[6] = 'red' # flag
                                    feedback_notifications[3] = str(time.time_ns() - int(feedback_notifications[3]))
                                    # and we must quickly set that to the robot.
                                    self.cur.execute('UPDATE table_robot SET notifications = %s, agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
                                        (','.join(feedback_notifications), ','.join(feedback_agv_status), _r_id, f_id,))
                            # if our decision had been directly 'red' all along. then, ...
                            else: # this changes nothing if we never entered anywhere.
                                self.cur.execute('UPDATE table_robot SET notifications = %s, agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
                                                (','.join(feedback_notifications), ','.join(feedback_agv_status), _r_id, f_id,))

                            print("[traffic]:-"+str([_r_id])+" _agv_status-" + str([feedback_agv_status[6]]) + ".")

                            # [ROBOT INACTIVE] CASE 1:
                            #################################################################################################
                            # we talked about notifications[4] earlier where i said "none/skip: implies what stage of a
                            # negotiation a robot is with another robot (mobile executor MEx) in order to define new
                            # paths so as not to wait forever. "
                            # I mean! we are now waiting yeah, but what if the person we are waiting for is also waiting for us?
                            # recall our next_stop_id is either 'd', or 'e' if skip 'feedback_notifications[4]' node is true.
                            if (feedback_agv_status[6] == 'red'):
                                # initialize the checkpoint of the unknown MEx that might be putting us in a stand-off
                                mex_r_id = None
                                mex_last_checkpoint = ['None']
                                mex_notifications = ['None'] * 7
                                # we might have to negotiate with the other robot 'MEx', get its last_checkpoint and negotiation profile.
                                print("[traffic]:-"+str([_r_id])+" negotiation for node:"+str([next_stop_id])+" required. ")

                                # self.cur.execute("SELECT robot_id, current_pose, last_checkpoint, notifications, agv_status, \
                                #         landmark, wait_traffic, waitpoints, model_config FROM table_robot WHERE \
                                #                 traffic = %s and fleet_id = %s;", (next_stop_id, f_id,))
                                self.cur.execute("SELECT * FROM table_robot WHERE traffic = %s and fleet_id = %s;", (next_stop_id, f_id,))
                                for record in self.cur.fetchall():
                                    mex_r_id = str(record['robot_id'])
                                    mex_current_pose = str(record['current_pose']).split(',')
                                    mex_last_checkpoint = str(record['last_checkpoint']).split(',')
                                    mex_notifications = str(record['notifications']).split(',')
                                    mex_agv_status = str(record['agv_status']).split(',')
                                    mex_landmarks = str(record['landmark']).split(',')
                                    mex_wait_traffic = str(record['wait_traffic'])
                                    mex_waitpoints = str(record['waitpoints']).split(',')
                                    mex_model_config = str(record['model_config']).split(',')

                                print("[traffic]:-"+str([_r_id])+" vs occupier mex "+str([mex_r_id]))


                                if mex_r_id is None:
                                    pass
                                # if this MEx is active, that is, currently has its own task, and
                                # MEx is already aware that someone's in its path. then, ...
                                elif (mex_agv_status[0] == 'active'):
                                    if (mex_agv_status[6] == 'red') and (mex_notifications[4] != "skip"):
                                    # recall, reserved_checkpoint is still the place i am occupying in my traffic but not where i want to go (next_stop_id 'd' or 'e').
                                    # if at some point in time, the place i am is where the other robot 'MEx' wants to come.
                                    # we might just be waiting forever.
                                        if mex_last_checkpoint[0] == reserved_checkpoint:
                                            # if we have not started negotiation, we need to start
                                            print("[traffic]: "+str([_r_id])+" negotiation started. ")

                                            # if int(mex_notifications[3]) != 0: | self.notifications[4] = "None" | self.notifications[5] = "proposed_wait_time"
                                            # -------------------------------------------------
                                            priority_val = self.map_priority(feedback_landmarks[0])
                                            mex_priority_val = self.map_priority(mex_landmarks[0])
                                            reserved_checkpoint_coordinate = feedback_agv_itinerary[feedback_checkpoints.index(reserved_checkpoint)]

                                            if priority_val > mex_priority_val:
                                                # this robot should reserve the node it wants to go, and then drive directly there.
                                                # the MEx should reserve this robot's current reserved node, then go to the waiting area first before driving towards it.
                                                print("[traffic]: "+str([_r_id])+" has priority. ")
                                                mex_wait_traffic = self.check_waitpoint_association(next_stop_id, mex_waitpoints)
                                                print("[traffic]: mex "+str([mex_r_id])+" --> wait at "+str([mex_wait_traffic])+".")
                                                if (mex_wait_traffic is not None) and (mex_wait_traffic not in traffic_control):
                                                    mex_est_time = self.estimate_time_to_node(robot_pos = feedback_current_pose,
                                                                                            node_pos = next_stop_coordinate,
                                                                                            lin_vel = float(feedback_model_config[2]))
                                                    mex_notifications[5] = str(mex_est_time)
                                                    print("[traffic]: mex "+str([mex_r_id])+" --> estimated wait time "+str([mex_est_time])+".")

                                            elif (priority_val < mex_priority_val):
                                                # the MEx should reserve this robot's current reserved node, and then drive directly there.
                                                # this robot should reserve it's current next_stop_id node, then go to the waiting area first before driving towards it.
                                                print("[traffic]: "+str([mex_r_id])+" has priority. ")
                                                feedback_wait_traffic = self.check_waitpoint_association(reserved_checkpoint, feedback_waitpoints)
                                                print("[traffic]: mex "+str([_r_id])+" --> wait at "+str([feedback_wait_traffic])+".")
                                                if (feedback_wait_traffic is not None) and (feedback_wait_traffic not in traffic_control):
                                                    est_time = self.estimate_time_to_node(robot_pos = mex_current_pose,
                                                                                            node_pos = reserved_checkpoint_coordinate,
                                                                                            lin_vel = float(mex_model_config[2]))
                                                    feedback_notifications[5] = str(est_time)
                                                    print("[traffic]: mex "+str([mex_r_id])+" --> estimated wait time "+str([est_time])+".")

                                            elif (priority_val == mex_priority_val): # if equal use time, who has been waiting the longest:
                                                # if my 'how long ive been waiting' is greater, then im the one to move to the intended position while he waits at shifted midpoint
                                                if int(feedback_notifications[3]) > int(mex_notifications[3]):
                                                    print("[traffic]: "+str([_r_id])+" has waited longer. ")
                                                    mex_wait_traffic = self.check_waitpoint_association(next_stop_id, mex_waitpoints)
                                                    print("[traffic]: mex "+str([mex_r_id])+" --> wait at "+str([mex_wait_traffic])+".")
                                                    if (mex_wait_traffic is not None) and (mex_wait_traffic not in traffic_control):
                                                        mex_est_time = self.estimate_time_to_node(robot_pos = feedback_current_pose,
                                                                                                node_pos = next_stop_coordinate,
                                                                                                lin_vel = float(feedback_model_config[2]))
                                                        mex_notifications[5] = str(mex_est_time)
                                                        print("[traffic]: mex "+str([mex_r_id])+" --> estimated wait time "+str([mex_est_time])+".")

                                                elif int(feedback_notifications[3]) < int(mex_notifications[3]):
                                                    print("[traffic]: "+str([mex_r_id])+" has waited longer. ")
                                                    feedback_wait_traffic = self.check_waitpoint_association(reserved_checkpoint, feedback_waitpoints)
                                                    print("[traffic]: mex "+str([_r_id])+" --> wait at "+str([feedback_wait_traffic])+".")
                                                    if (feedback_wait_traffic is not None) and (feedback_wait_traffic not in traffic_control):
                                                        est_time = self.estimate_time_to_node(robot_pos = mex_current_pose,
                                                                                                node_pos = reserved_checkpoint_coordinate,
                                                                                                lin_vel = float(mex_model_config[2]))
                                                        feedback_notifications[5] = str(est_time)
                                                        print("[traffic]: mex "+str([mex_r_id])+" --> estimated wait time "+str([est_time])+".")

                                            # -------------------------------------------------

                                            if (feedback_notifications[5] != 'wt') or (mex_notifications[5] != 'wt'):
                                                # print("written 1: ")
                                                feedback_notifications[3] = '0'
                                                feedback_agv_status[6] = 'green'
                                                self.cur.execute('UPDATE table_robot SET traffic = %s, wait_traffic = %s, notifications = %s, agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
                                                    (next_stop_id, feedback_wait_traffic, ','.join(feedback_notifications), ','.join(feedback_agv_status), _r_id, f_id,))

                                                # print("written 2: ")
                                                mex_notifications[3] = '0' # zero the wait time
                                                mex_agv_status[6] = 'green' # move the robot
                                                self.cur.execute('UPDATE table_robot SET traffic = %s, wait_traffic = %s, notifications = %s, agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
                                                    (mex_last_checkpoint[0], mex_wait_traffic, ','.join(mex_notifications), ','.join(mex_agv_status), mex_r_id, f_id,))

                                                print("[traffic]: "+str([_r_id])+" negotiation complete. ")
                                    else:
                                        pass
                                else:
                                    print("[traffic]: "+str([_r_id])+" stuck. human assistance required. next node occupied by an inactive robot.")
                                    # raise an alarm. else we will just wait forever.
                                    pass

                    else:
                        feedback_notifications[3] = '0'
                        feedback_agv_status[6] = 'green' # flag
                        self.cur.execute('UPDATE table_robot SET traffic = %s, notifications = %s, agv_status = %s WHERE robot_id = %s and fleet_id = %s;',
                                        (next_stop_id, ','.join(feedback_notifications), ','.join(feedback_agv_status), _r_id, f_id,))
                        print("[traffic]:-"+str([_r_id])+" agv_status-" + str([feedback_agv_status[6]]) + ".")

                self.conn.commit()

            except Exception as error:
                print("[manager]:-on_timer-" + str([error]) + ".")

            time.sleep(0.03)
        # --------------

# ---------------------------------------------- #
#      get_world  (WALLS AS OBSTACLES)           #
# ---------------------------------------------- #
# ----------------------------------------------------------------------------------------------------
# ----------------------------------  WORLD PLOT -----------------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def get_world(self, f_id=None):
        if f_id is None:
            return

        # i have assumed that there is atleast 1 robot registered that suplies the "self.feedback_base_map_img" map
        if (self.nav_map_obtained == False) and (self.feedback_base_map_img is not None) and (self.feedback_base_map_img != 'None'):
                self.allobs_x,self.allobs_y = [],[]
                # img_path = "/colcon_ws/src/nav2_rosdevday_2021/nav2_rosdevday_2021/maps/my_cartographer_map.pgm"
                # src = cv2.imread(img_array, cv2.IMREAD_GRAYSCALE)
                # src = cv2.flip(src, 0)
                # Convert the memory object to bytes
                img_data = self.feedback_base_map_img.tobytes()
                img = Image.open(io.BytesIO(img_data))
                # Display the image
                # plt.imshow(img)
                # plt.show()
                # Open the image
                img_array = np.array(img)
                src = cv2.flip(img_array, 0)
                # Set threshold and maxValue
                thresh = 5 # 0
                maxValue = 255
                # Basic threshold example
                th, dst = cv2.threshold(src, thresh, maxValue, cv2.THRESH_BINARY)
                for i in range (dst.shape[0]): #traverses through height of the image  # height, width = src.shape[:2]
                        for j in range (dst.shape[1]): #traverses through width of the image
                                if dst[i][j] == 0:           # print('wall coordinates: {},{}'.format(i,j))
                                        self.allobs_x.append((i*0.05)-10.15)  # 12.3) - Kullar  # 10)-BV  # while scale fixed, map moves downward if increased.
                                        self.allobs_y.append((j*0.05)-9.8)   # 6.3)  - Kullar  # 10)-BV # while scale fixed, map moves backward if increased.
                self.nav_map_obtained = True
        elif self.nav_map_obtained is True:
                pass
        else:
                print("[manager]:-No image found. please ensure that fleet_map_path in client is properly set in atleast one robot.")
                return

# ---------------------------------------------- #
#   nav_plot.  (SHOW CURRENT AGV TRAJECTORY)     #
# ---------------------------------------------- #
# ----------------------------------------------------------------------------------------------------
# ------------------------------------  NAV PLOT -----------------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def nav_plot(self, f_id=None):

        if f_id is None:
            return

        if (self.show_robot_animation is True) and (self.feedback_base_map_img is not None) and (self.feedback_base_map_img != 'None'):
                if (self.single_robot_view is True) and (self.feedback_current_pose != None):
                        if self.agv_track_dict['x'] != [] and self.agv_track_dict['y'] != []:  # it is not empty
                                # this is so that when its not moving it stops appending same coordinates to list.
                                if (float(self.agv_track_dict['x'][-1]) != float(self.feedback_current_pose[0])) and (float(self.agv_track_dict['y'][-1]) != float(self.feedback_current_pose[1])):
                                        self.agv_track_dict['x'].append(float(self.feedback_current_pose[0]))
                                        self.agv_track_dict['y'].append(float(self.feedback_current_pose[1]))
                        else:   # initially, it is empty. hence, append first...
                                self.agv_track_dict['x'].append(float(self.feedback_current_pose[0]))
                                self.agv_track_dict['y'].append(float(self.feedback_current_pose[1]))
                        if len(self.agv_track_dict['x']) == 45:
                                self.agv_track_dict['x'].pop(0)
                        if len(self.agv_track_dict['y']) == 45:
                                self.agv_track_dict['y'].pop(0)
                        # Clear the axes
                        self.ax.cla()
                        self.cx, self.cy = [], []
                        if self.feedback_checkpoints != ['A','A','A']:
                                for i in range(len(self.feedback_checkpoints)):
                                        if self.feedback_checkpoints[i] in checkpoints: # if 7 in a
                                                self.cx.append(agv_itinerary[checkpoints.index(self.feedback_checkpoints[i])][0]) #b=a.index(7)
                                                self.cy.append(agv_itinerary[checkpoints.index(self.feedback_checkpoints[i])][1])
                                self.ax.scatter(self.cx, self.cy, marker="P", facecolor='red') # "*", "X", "s", "P" ## -- checkpoints
                        if len(self.landmark_dict['x']) != 0: # scatter([1,2,3],[4,5,6],color=['red','green','blue'])
                                self.ax.scatter(self.landmark_dict['x'], self.landmark_dict['y'], marker="P", color=self.landmark_dict['k'])
                        for i in range(0, len(self.landmark_dict['x'])):
                                self.ax.annotate(self.landmark_dict['t'][i], (self.landmark_dict['x'][i], self.landmark_dict['y'][i]))
                        # plot walls/obstacle, basically plot the MAP
                        self.ax.scatter(self.allobs_y, self.allobs_x,  marker="X",facecolor='black') # "*", "X", "s", "P" ##
                        # plot the agv trajectory in orange, basically agv footprint
                        self.ax.plot(self.agv_track_dict['x'], self.agv_track_dict['y'], color='orange',linestyle='dashed', label="trajectory")
                        # plot the agv itself, basically a beautiful vehicle
                        self.plot_car(float(self.feedback_current_pose[0]), float(self.feedback_current_pose[1]), float(self.feedback_current_pose[2]), steer=0)
                        # set some figure properties
                        self.ax.axis("tight") # equal on off tight auto square scaled
                        self.ax.grid(True)
                        if (self.feedback_agv_status[0] == 'idle') or (self.feedback_agv_status[0] == 'inactive') or (self.feedback_agv_status[0] == 'active'):
                                self.ax.set_title("AGV:" + str(robot_id)  + ", STATUS:" + str(self.feedback_agv_status[0])) # str(round(state.v * 3.6, 2))
                        plt.draw()
                        # plt.show() # ---> blocking call. code will pass only if window is closed
                        plt.pause(0.001)  # Non-blocking call. pause to update the figure
                else:
                    # initialize the world into memory
                    try:
                        self.cur.execute('SELECT current_pose FROM table_robot WHERE fleet_id = %s;', (f_id,))
                        for record in self.cur.fetchall():
                            # Clear the axes
                            self.ax.cla()
                            self.feedback_all_poses = str(record['current_pose']).split(',')
                            # plot walls/obstacle, basically plot the MAP
                            self.ax.scatter(self.allobs_y, self.allobs_x,  marker="X",facecolor='black') # "*", "X", "s", "P" ##
                            # plot the agvs, basically a beautiful vehicle
                            self.plot_car(float(self.feedback_all_poses[0]), float(self.feedback_all_poses[1]), float(self.feedback_all_poses[2]), steer=0)
                            plt.draw()
                            # plt.show() # ---> blocking call. code will pass only if window is closed
                            plt.pause(0.001) # Non-blocking call. pause to update the figure
                    except Exception as error:
                            print("[manager]:-nav_plot-"+str([error])+".")

# ---------------------------------------------- #
#      PLOT CAR:                                 #
# ---------------------------------------------- #
# ----------------------------------------------------------------------------------------------------
# ------------------------------------  car PLOT -----------------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def plot_car(self, x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

        outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                                [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

        fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                                [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

        rr_wheel = np.copy(fr_wheel)

        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                        [-math.sin(yaw), math.cos(yaw)]])
        Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                        [-math.sin(steer), math.cos(steer)]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += WB
        fl_wheel[0, :] += WB

        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T

        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T

        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        rr_wheel[0, :] += x
        rr_wheel[1, :] += y
        fl_wheel[0, :] += x
        fl_wheel[1, :] += y
        rl_wheel[0, :] += x
        rl_wheel[1, :] += y

        self.ax.plot(np.array(outline[0, :]).flatten(),np.array(outline[1, :]).flatten(), truckcolor)
        self.ax.plot(np.array(fr_wheel[0, :]).flatten(),np.array(fr_wheel[1, :]).flatten(), truckcolor)
        self.ax.plot(np.array(rr_wheel[0, :]).flatten(),np.array(rr_wheel[1, :]).flatten(), truckcolor)
        self.ax.plot(np.array(fl_wheel[0, :]).flatten(),np.array(fl_wheel[1, :]).flatten(), truckcolor)
        self.ax.plot(np.array(rl_wheel[0, :]).flatten(),np.array(rl_wheel[1, :]).flatten(), truckcolor)
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
                Args:
                        from_loc_id (_type_): _description_
                        to_loc_id (_type_): _description_
                        task_name (_type_): _description_
                        task_priority (_type_): _description_
                """
                global checkpoints, landmark, agv_itinerary, robot_id, fleet_id, waitpoints, wait_itinerary

                self.request_tasks(from_loc_id, to_loc_id, task_name, task_priority)

                if robot_id == 0:
                    print("[manager]:-ATTENTION! Robot is not selected. Please choose robot.")
                    return

                if len(checkpoints) != 0:
                        if task_cleared == True:
                                self.feedback_agv_status[0] = 'active'
                                try: # A,B,C,D,E,F - checkpoints
                                    self.cur.execute('UPDATE table_robot SET agv_status = %s, checkpoints = %s, landmark = %s, \
                                        agv_itinerary = %s, waitpoints = %s, wait_itinerary = %s WHERE robot_id = %s and fleet_id = %s',
                                    (','.join(self.feedback_agv_status),','.join(checkpoints),','.join(landmark),
                                        agv_itinerary, ','.join(waitpoints), wait_itinerary, robot_id, fleet_id,))
                                    self.conn.commit()
                                except Exception as error:
                                    print("[manager]:-fm_send_task_request-"+str([error])+".")
                        else:
                                print('[manager]:-ATTENTION! Task not cleared. Hence, not assigned.')

# ---------------------------------------------- #
#     ON TASK REQUEST                            #
# ---------------------------------------------- #
# ----------------------------------------------------------------------------------------------------
# -------------------------- TaskScheduler: request_tasks  -------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def request_tasks(self, from_loc_id, to_loc_id, task_name, task_priority):
        # from_loc_id | to_loc_id | task_name | task_priority
        """
        given a selected job id locate job information and itinerary and assign task to robot.
        """

        global checkpoints, landmark, agv_itinerary, task_dictionary, robot_id, task_cleared, waitpoints, wait_itinerary
        checkpoints, agv_itinerary, landmark, waitpoints, wait_itinerary = [], [], [], [], []
        task_cleared = False

        # ROBOT_ID and FLEET_ID have already been assigned immediately a robot is selected. Hence, the plot.

        if robot_id == 0:
              print("[manager]:-Robot is not selected. Please choose robot.")
              return

        if from_loc_id == to_loc_id and (task_name != 'charge' and task_name != 'move'):
              print("[manager]:-'load' location cannot be equal to 'unload' location.")
              return

        if (from_loc_id not in self.job_ids) or (to_loc_id not in self.job_ids):
                print("[manager]:-only ids within this fleet ["+str(fleet_id)+"] form a valid task request.")
                return

        shortest_distance_found = 10000000000000000000000
        loc_node_owner = None #  just for initialization to hold values.
        home_dock_loc_ids = []
        charge_dock_loc_ids = []

        #  1 landmark (charge) - charge | 3 landmarks (pick, drop, home > 1) - transport | 2 landmarks (pick, drop) - loop

        print("[manager]:-robot at: ", self.feedback_current_pose[0], self.feedback_current_pose[1], self.feedback_current_pose[2])
        for i in range(0,len(task_dictionary["itinerary"])):
                if task_dictionary["itinerary"][i]["fleet_id"] == fleet_id:
                        # build a list of all home_dock stations in this fleet
                        if task_dictionary["itinerary"][i]["description"] == 'home_dock':
                              home_dock_loc_ids.append(task_dictionary["itinerary"][i]["loc_id"])
                        # build a list of all charge_dock stations in this fleet
                        if task_dictionary["itinerary"][i]["description"] == 'charge_dock':
                              charge_dock_loc_ids.append(task_dictionary["itinerary"][i]["loc_id"])
                        # check if intended start location is in the fleet we belong
                        # handle case where there is a barrier for example, and shortest distance is not really the real world shortest distance.
                        coordinate = task_dictionary["itinerary"][i]["coordinate"]
                        d = math.sqrt((float(coordinate[0]) - float(self.feedback_current_pose[0]))**2 + (float(coordinate[1]) - float(self.feedback_current_pose[1]))**2)
                        if d < shortest_distance_found:
                                shortest_distance_found = d
                                loc_node_owner = task_dictionary["itinerary"][i]["loc_id"]
                                print("[manager]:-found shortestest dist: ", shortest_distance_found," with loc: ", loc_node_owner)

        imaginary_checkpoints = []
        real_checkpoints = []
        temp_dock_list = []
        landmark = []
        checkpoints = []

        if task_name == 'charge' or task_name == 'move':
                if task_name == 'charge':
                        # for the charge task, just gather all the charge stations and build a path linking all of them
                        landmark = [task_priority, task_name]
                        if len(charge_dock_loc_ids) != 0:
                                # for each charge dock station we have
                                for i in range(0,len(charge_dock_loc_ids)):
                                        if i == 0:
                                                # obtain a path from the robot to the first charge dock station
                                                temp_dock_list.append(self.shortest_path(graph, loc_node_owner, charge_dock_loc_ids[i]))
                                        else:
                                                # from i = 1,
                                                # we wanna get path from one charge dock to the next charge dock
                                                # essentially, if the first charge dock is occupied, we can move to the next
                                                temp_dock_list.append(self.shortest_path(graph, charge_dock_loc_ids[i-1], charge_dock_loc_ids[i]))
                                        # add the charge dock in the order we have analyzed.
                                        landmark.append(charge_dock_loc_ids[i])
                                # sum all the checkpoints
                                # and make sure there is no repeated start and end checkpoints.
                                for i in range(len(temp_dock_list)):
                                        if i == 0:
                                                checkpoints += temp_dock_list[i]
                                        elif temp_dock_list[i-1][-1] == temp_dock_list[i][0]:
                                                checkpoints += temp_dock_list[i][1:]
                                        else:
                                                checkpoints += temp_dock_list[i]
                # if its move, check if the place the robot is closest to,
                # which is where we have assumed is the start point/node,
                # check if it is also the target node,
                # if it is not, then get the path that leads there.
                if task_name == 'move':
                        if loc_node_owner != to_loc_id:
                                checkpoints = self.shortest_path(graph, loc_node_owner, to_loc_id)
                                landmark = [task_priority, task_name, to_loc_id]
                else:
                      pass

        elif task_name == 'transport' or task_name== 'loop':
                if loc_node_owner != from_loc_id:
                        imaginary_checkpoints = self.shortest_path(graph, loc_node_owner, from_loc_id)
                real_checkpoints = self.shortest_path(graph, from_loc_id, to_loc_id)
                # Check if the last element of lst_1 is the same as the first element of lst_2
                if len(imaginary_checkpoints) > 0:
                        if imaginary_checkpoints[-1] == real_checkpoints[0]:
                                checkpoints = imaginary_checkpoints + real_checkpoints[1:]
                else:
                        # If they are different, output both lists concatenated
                        checkpoints = real_checkpoints

                if task_name == 'loop':
                        landmark = ['high', task_name, from_loc_id, to_loc_id]
                        return_checkpoints = self.shortest_path(graph, to_loc_id, from_loc_id)
                        if len(return_checkpoints) != 0:
                                if checkpoints[-1] == return_checkpoints[0]:
                                        checkpoints = checkpoints + return_checkpoints[1:]

                if task_name == 'transport':
                        landmark = [task_priority, task_name, from_loc_id, to_loc_id]
                        if len(home_dock_loc_ids) != 0:
                                for i in range(0,len(home_dock_loc_ids)):
                                        if i == 0:
                                                temp_dock_list.append(self.shortest_path(graph, to_loc_id, home_dock_loc_ids[i]))
                                        else:
                                                temp_dock_list.append(self.shortest_path(graph, home_dock_loc_ids[i-1], home_dock_loc_ids[i]))
                                        landmark.append(home_dock_loc_ids[i])

                                temp_dock_checkpoints = []
                                for i in range(len(temp_dock_list)):
                                        if i == 0:
                                                temp_dock_checkpoints += temp_dock_list[i]
                                        elif temp_dock_list[i-1][-1] == temp_dock_list[i][0]:
                                                temp_dock_checkpoints += temp_dock_list[i][1:]
                                        else:
                                                temp_dock_checkpoints += temp_dock_list[i]

                                if len(temp_dock_checkpoints) != 0:
                                        if checkpoints[-1] == temp_dock_checkpoints[0]:
                                                checkpoints = checkpoints + temp_dock_checkpoints[1:]

        elif task_name == "clean":
        # if the task is clean, we do not have any landmark because
        # the coverage path planner will generate path on the map to be traversed.
        # and this path does not necessarily imply the normal routes traversed by other robots within same fleet.
        # truth is! if you have more than 1 cleaning robot in your fleet, then this might be a problem as probability
        # for a robot stand-off becomes high.
        # because idea is;
        #       once its a cleaning task we arent really doing anything other than monitoring the robots.
        #       and checking if its gonna collide with anything and then stop it till the obstacle leaves.
        #       we can not alter a path for which we did not create.
                landmark = ['low', task_name, "unknown", "unknown"]
                checkpoints = ['A', 'B', 'A'] # default no-task structure
                agv_itinerary = [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]] # '[x,y,z,w],[x,y,z,w],[x,y,z,w]' --> 'A', 'A', 'A'
        else:
              pass

        # fetch the locations coordinates associated with the node/checkpoints path
        # waitpoints = {}  # Initialize an empty dictionary for waitpoints
        if len(checkpoints) != 0:
            print("[manager]:- writing path into text...")
            # print("[manager]:-",str(checkpoints))
            print("[manager]:- ")
            # checkpoints -- alphabets or location ids
            # obtain agv itinerary -- coordinate
            for x in range(0,len(checkpoints)):
                for y in range(0,len(task_dictionary["itinerary"])):
                    if task_dictionary["itinerary"][y]["loc_id"] == checkpoints[x]:
                        agv_itinerary.append(task_dictionary["itinerary"][y]["coordinate"])

            waitpoints = self.extract_unique_waitpoints(checkpoints, graph)
            wait_itinerary = self.get_waitpoint_locations(waitpoints)
            # print("[manager]:- waitpoint_itinerary: ", wait_itinerary)

        # landmark -- [task priority, task type or name, landmarks]
        # do predecessor landmark stuff here;
        if landmark != []:
               if (landmark[1] == 'loop') or (landmark[1] == 'transport'):
                       # landmark[2] here is pick. recall 0 is priority, 1 is type, 2 is pick, 3 is drop,
                       # for transport, 4 is home, but for loop, it goes back to pick.
                       # so select the alphabet that represent the location after landmark pick.
                       # i am using + for the first instance because i am sure when it wants to pull out
                       # no pun intended, it must pass by the gate. however, the start point might have been
                       # precisely the pick location. i dont know. so the would have been no predecessor in that
                       # case. you know? gotta be smart!
                        landmark[2] = checkpoints[checkpoints.index(landmark[2]) + 1]+'_'+landmark[2]
                        # select the alphabet that represent the location before landmark drop.
                        landmark[3] = checkpoints[checkpoints.index(landmark[3]) - 1]+'_'+landmark[3]
                        if landmark[1] == 'transport':
                                # 4 upwards is home for transport we need to get all predecessor/gate locations too.
                                for idx, elem in enumerate(landmark[4:]): # Here, `elem` is the current element in the loop and `idx` is its index within the list # Do something with `prev_elem`
                                        landmark[4+idx] = checkpoints[checkpoints.index(elem) - 1]+'_'+landmark[4+idx]
               elif landmark[1] == 'move':
                        landmark[2] = checkpoints[checkpoints.index(landmark[2]) - 1]+'_'+landmark[2]
               elif (landmark[1] == 'charge'):
                        for idx, elem in enumerate(landmark[2:]): # Here, `elem` is the current element in the loop and `idx` is its index within the list
                                landmark[2+idx] = checkpoints[checkpoints.index(elem) - 1]+'_'+landmark[2+idx]
        else:
               landmark = ['none']

        print("[manager]:- checkpoints to pass: ", checkpoints)
        print("[manager]:- landmark to visit: ", landmark)
        print("[manager]:- waitpoints: ", waitpoints)

        # schedule task
        self.task_scheduled(task_name)

        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------

    def task_scheduled(self, task_name):
        global task_cleared
        # Find free robots or available robots not currently performing any task from table_robots
        try: # try to open a connection yeah. but if it closes abruptly, # connection will never be closed hence, move 'close()' to finally
            #select_script = '''SELECT * FROM table_robot''' # '''SELECT id,name FROM table_robot'''
            select_script = "SELECT * FROM table_robot WHERE robot_id = %s;"
            select_id = (robot_id,)
            self.cur.execute(select_script, select_id)
            # print(cur.fetchall()) # everyone gets printed on one line.
            for record in self.cur.fetchall(): #print(record) # now everyone gets its own line e.g [2, 'Robin', 22500, 'D1'] [3, 'Xavier', 19500, 'D2']
                checkp_pos_ = str(record['checkpoints']).split(',');# print(len(self.checkpoints),"-route: ", self.checkpoints) # [A,B,C,D]
                agv_stat_ = str(record['agv_status']).split(',')
                battery_stat_ = str(record['model_config']).split(',')
        except Exception as error: # show the exceptions basically as errors.
            print("[manager]:-task_scheduled-"+str([error])+".")

        try:
                if ((checkp_pos_[0] == checkp_pos_[1] == checkp_pos_[2]) or (agv_stat_[0] == 'idle')):
                        # robot is available
                        if (float(battery_stat_[-1]) > 35) or (float(battery_stat_[-1]) <= 35 and task_name == 'charge'):
                                task_cleared = True
                        elif (float(battery_stat_[-1]) <= 35) and (task_name != 'charge'):
                                task_cleared = False
                                self.available_robots = self.suggest_robot()
                                if len(self.available_robots) > 0:
                                        print("[manager]:-Robot battery percent is less than 35%. Only 'charge' task is allowed. Please consider robot "+str(self.available_robots[0])+".")
                                else:
                                        print('[manager]:-ATTENTION! '+" battery percent is less than 35%. only 'charge' task is allowed.")
                else:
                        # robot is not available
                        task_cleared = False
                        self.available_robots = self.suggest_robot()
                        if len(self.available_robots) > 0:
                                print('[manager]:-ATTENTION! '+'Robot unavailable. Robot is currently active. Please cancel current task and reassign or consider robot '+str(self.available_robots[0])+'.')
                        else:
                                print('[manager]:-ATTENTION! '+'Robot unavailable. Robot is currently active. Please cancel current task and reassign.')
        except ValueError:
                print("[manager]:-task_scheduled- could not convert battery stat to float.")

        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------

    def suggest_robot(self):
        available_robots = [] # - Assign task to any available robot. but in a while loop.
        try:
                self.cur.execute("SELECT * FROM table_robot WHERE fleet_id = %s;", (fleet_id,))
                for record in self.cur.fetchall():
                        r_id_ = str(record['robot_id']).split(',')
                        checkp_p_ = str(record['checkpoints']).split(','); # [A,B,C,D]
                        agv_s_ = str(record['agv_status']).split(',')
                        model_conf = str(record['model_config']).split(',')
                        if ((checkp_p_[0] == checkp_p_[1] == checkp_p_[2])) or ((agv_s_[0] == 'idle')):
                                # this robot is available print("battery_s_[-1]: ", float(battery_s_[-1]) )
                                if (float(model_conf[4]) > 35):
                                        available_robots.append(r_id_)
                                        break
        except (ValueError, Exception) as error:
                print("[manager]:-suggest_robot-"+str([error])+".")

        return available_robots


        # ----------------------------------------------------------------------------------------------------
        # ---------------------------- PathScheduler: dijkstra  ----------------------------------------------
        # ----------------------------------------------------------------------------------------------------

    def extract_unique_waitpoints(self, checkpoints, graph):
        # Initialize a set to store unique waitpoints
        waitpoints = set()
        # Iterate through each checkpoint
        for checkpoint in checkpoints:
            # Check if the checkpoint is in the graph
            if checkpoint in graph:
                # Iterate through neighbors of the checkpoint
                for neighbor, _ in graph[checkpoint]:
                    # Check if the neighbor is a waitpoint (starts with 'W')
                    if neighbor.startswith('W'):
                        # Add the waitpoint to the set
                        waitpoints.add(neighbor)
        # Convert the set to a list
        return list(waitpoints)


    def get_waitpoint_locations(self, waitpoints):
        """
        Retrieve the locations of the waitpoints from the graph.

        Args:
        - waitpoints (list): List of waitpoints to retrieve locations for.

        Returns:
        - dict: A list mapping waitpoints to their locations.
        """
        wait_itinerary = []
        for i in range(0,len(waitpoints)):
            for j in range(0,len(task_dictionary["itinerary"])):
                if task_dictionary["itinerary"][j]["loc_id"] == waitpoints[i]:
                    wait_itinerary.append(task_dictionary["itinerary"][j]["coordinate"])
        return wait_itinerary

        # ----------------------------------------------------------------------------------------------------
        # ---------------------------- PathScheduler: dijkstra  ----------------------------------------------
        # ----------------------------------------------------------------------------------------------------

    def dijkstra(self, graph, start):
        # print("graph: ", graph) # print("start: ", start)
        dist = {node: float('inf') for node in graph}
        dist[start] = 0
        heap = [(0, start)]
        while heap:
            (d, node) = heapq.heappop(heap)
            if d > dist[node]:
                    continue
            for neighbor, weight in graph[node]:
                new_dist = dist[node] + weight
                if new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    heapq.heappush(heap, (new_dist, neighbor))
        return dist

        # ----------------------------------------------------------------------------------------------------
        # ---------------------------- PathScheduler: shortest_path ------------------------------------------
        # ----------------------------------------------------------------------------------------------------

    def shortest_path(self, graph, start, goal):
                distances = self.dijkstra(graph, start)
                if goal not in distances:
                        return None
                paths = {start: [start]}
                heap = [(0, start)]
                while heap:
                        (d, node) = heapq.heappop(heap)
                        if node == goal:
                                return paths[node]
                        for neighbor, weight in graph[node]:
                                new_dist = distances[node] + weight
                                if neighbor not in paths or (new_dist < distances[neighbor] and len(paths[node]) + 1 <= len(paths[neighbor])):
                                        distances[neighbor] = new_dist
                                        paths[neighbor] = paths[node] + [neighbor]
                                        heapq.heappush(heap, (new_dist, neighbor))
                return None

##################################################
# ---------------------------------------------- #
#     LANDMARKING: SAVE CURRENT LOCATION         #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ------------------------------ fm_add_landmark_request  -------------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def fm_add_landmark_request(self, station_type, input_loc_id, neighbor_loc_ids): # limit by graph size, make only numeric
        if robot_id == 0:
                print('[manager]:-ATTENTION! '+"Robot is not selected. Please choose robot.")
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
                print('[manager]:-ATTENTION! ' + 'Location Number 0 cannot be chosen. Start numbering from 1.\
                        Only single capital lettered alphabets can be used. E.g. A1, E54 etc.')
                return
        else: # check if its already in the landmarks
                for i in range(0,len(task_dictionary["itinerary"])):
                        if task_dictionary["itinerary"][i]["fleet_id"] == fleet_id:
                                if task_dictionary["itinerary"][i]["loc_id"] == input_loc_id:
                                        print('[manager]:-ATTENTION! '+'Location ID already exists.')
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
                        print('[manager]:-ATTENTION! ' + "Invalid element in 'Neighbors' field: " + element + ".")
                        return
                if element not in self.landmark_dict['t']:
                        print('[manager]:-ATTENTION! ' + "Element " + element + " in 'Neighbors' field not a landmark.")
                        return

        self.add_location_db_cmd(edges, x, y, w, z, station_type, input_loc_id, fleet_id)

        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------

    def add_location_db_cmd(self, edges, x, y, w, z, loc_type, node, fleet_id):
        global task_dictionary

        temp_dict = {   "loc_id": node,
                        "fleet_id": fleet_id,
                        "description": loc_type,
                        "coordinate": [x, y, w, z]   }
        # more like if node == 'A1': then edge_list = [('A2', 1.0)] <-- just to initialize or prime the edge list.
        # the A2 will subsequently be updated with its cost once we actually save an A2 node.
        # Again, users need to name the nodes alphabetically and serially
        edge_list = []
        # Check if node matches the pattern of a single capital letter followed by one or more digits
        if re.match(r'^[A-Z]\d+$', node):
                # Extract the letter and the number from the node
                letter = node[0]
                number = int(node[1:])
                # Create the next node by incrementing the number
                next_node = f'{letter}{number + 1}'
                edge_list = [(next_node, 1.0)]
        else:
                for i in range(0,len(task_dictionary["itinerary"])):
                        if task_dictionary["itinerary"][i]["fleet_id"] == fleet_id:
                                # we need to get the coordinates of the neighbours listed
                                if task_dictionary["itinerary"][i]["loc_id"] in list(edges):
                                        # lets set it to coordinate yeah
                                        coordinate = task_dictionary["itinerary"][i]["coordinate"]
                                        # obtain the distance from this new landmark/location to the neighbour and use it as the cost.
                                        d = round(math.sqrt((float(coordinate[0]) - x)**2 + (float(coordinate[1]) - y)**2), 2)
                                        # update the new landmarks edge list with the (neighbour,cost)
                                        edge_list.append((task_dictionary["itinerary"][i]["loc_id"],d))
        # update the edges and cost of the listed neighbours
        self.add_node_and_edges(node, edge_list)
        # convert the graph to the desired YAML format update the 'graph' entity in the dictionary
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
        time.sleep(0.5)
        print('[manager]:-Add landmark completed successfully! '+'modify edges in the output config file. [yaml]!')

        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------

    def add_node_and_edges(self, node, edges):
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
        # --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------

    def euler_to_quaternion(self, yaw, pitch, roll):
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
        if robot_id == 0:
            print('[manager]:-ATTENTION! '+ "Robot is not selected. Please choose robot.")
            return

        if self.feedback_checkpoints[0] == 'A' or  self.feedback_checkpoints[0] == 'A' or self.feedback_checkpoints[0] == 'A':
            print('[manager]:-ATTENTION! '+ "Robot not currently executing any task. Operation not performed.")
            return

        if trigger is True:
            self.clear_task_db_cmd('idle')
            time.sleep(2.0)
            self.clear_task_db_cmd('inactive')

    # active,x,y,z,w,base_floor
    def clear_task_db_cmd(self, whattodo):
        try:
            floor = self.feedback_agv_status[5]
            # flag = self.feedback_agv_status[6]
            self.cur.execute('UPDATE table_robot SET agv_status = %s, last_checkpoint = %s, checkpoints = %s, waitpoints = %s WHERE robot_id = %s and \
                    fleet_id = %s', (whattodo+',x,y,z,w,'+floor+',red', 'unknown', 'A,A,A', 'W,W,W', robot_id,fleet_id,))
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
        if robot_id == 0:
                print('[manager]:-ATTENTION! Robot is not selected. Please choose robot.')
                return
        try:
                self.cur.execute('SELECT * FROM table_robot WHERE robot_id = %s and fleet_id = %s;', (robot_id, fleet_id,))
                for record in self.cur.fetchall():
                        _agv_stat = str(record['agv_status']).split(',')
                        if (trigger is False) and (_agv_stat[0] == 'active'):
                                self.robot_status_db_cmd('inactive', robot_id, fleet_id)
                        elif (trigger is True) and (_agv_stat[0] == 'inactive'):
                                self.robot_status_db_cmd('active', robot_id, fleet_id)
                        else:
                                pass
        except Exception as error:
                print("[manager]:-fm_robot_status_trigger-"+str([error])+".")

    def robot_status_db_cmd(self, agv_stat, robot_id, fleet_id):  # active/inactive - agv_stat
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
    def fm_manual_control_trigger(self, trigger:bool):
        if robot_id == 0:
                print('[manager]:-ATTENTION! Robot is not selected. Please choose robot.')
                return
        try:
                self.cur.execute('SELECT * FROM table_robot WHERE robot_id = %s and fleet_id = %s;', (robot_id, fleet_id,)) # ROBOT_ID
                for record in self.cur.fetchall():
                        txt2 = str(record['m_controls']); self.m_controls = txt2.split(',')
                        if (trigger is False) and self.m_controls[0] == 'yes':
                                self.manual_cntrl_db_cmd('no', robot_id, fleet_id)
                                print('[manager]:-Success! manual drive mode is turned off.')
                        elif (trigger is True) and self.m_controls[0] == 'no':
                                self.feedback_agv_status[0] = 'active'
                                self.cur.execute('UPDATE table_robot SET agv_status = %s, checkpoints = %s, last_checkpoint = %s WHERE robot_id = %s and \
                                        fleet_id = %s', (','.join(self.feedback_agv_status),'A,A,A','unknown',robot_id,fleet_id,))
                                self.conn.commit()
                                self.manual_cntrl_db_cmd('yes', robot_id, fleet_id)
        except Exception as error:
                print("[manager]:-fm_manual_control_trigger-"+str([error])+".")

    def fm_manual_control_drive(self, k_pressed, robot_id, fleet_id):
        return self.manual_cntrl_db_cmd(k_pressed, robot_id, fleet_id)

    def manual_cntrl_db_cmd(self, k_pressed, robot_id, fleet_id):  # yes False, False, False, False -- on/off left forward right stop - m_controls
        if k_pressed == 'yes' or k_pressed == 'no':
                if k_pressed == 'yes' : self.control_command = 'yes,False,False,False,False,False'
                elif k_pressed == 'no': self.control_command = 'no,False,False,False,False,False'
                try:
                        self.cur.execute('UPDATE table_robot SET m_controls = %s WHERE robot_id = %s and fleet_id = %s', (self.control_command, robot_id, fleet_id,))
                        self.conn.commit()
                except Exception as error:
                        print("[manager]:-manual_cntrl_db_cmd-1-"+str([error])+".")
        else: # Call the corresponding function
                try:
                        self.cur.execute('SELECT m_controls FROM table_robot WHERE robot_id = %s and fleet_id = %s;', (robot_id, fleet_id,)) # ROBOT_ID
                        for record in self.cur.fetchall():
                                txt2 = str(record['m_controls'])
                                self.m_controls = txt2.split(',')
                                if self.m_controls[0] == 'yes':
                                        if k_pressed == 'left': self.control_command       = 'yes,True,False,False,False,False'
                                        elif k_pressed == 'forward': self.control_command  = 'yes,False,True,False,False,False'
                                        elif k_pressed == 'right': self.control_command    = 'yes,False,False,True,False,False'
                                        elif k_pressed == 'stop': self.control_command     = 'yes,False,False,False,True,False'
                                        elif k_pressed == 'backward': self.control_command = 'yes,False,False,False,False,True'
                                        self.cur.execute('UPDATE table_robot SET m_controls = %s WHERE robot_id = %s and fleet_id = %s', (self.control_command, robot_id, fleet_id,))
                                        self.conn.commit()
                except Exception as error:
                        print("[manager]:-manual_cntrl_db_cmd-2-"+str([error])+".")

##################################################
# ---------------------------------------------- #
#   SHUTDOWM  (ON OR OFF)                        #
# ---------------------------------------------- #
##################################################
# ----------------------------------------------------------------------------------------------------
# ---------------------------------- fm_shutdown_trigger  -------------------------------------------------
# ----------------------------------------------------------------------------------------------------
    def fm_shutdown_trigger(self, trigger:bool):
        try:
                self.cur.execute('SELECT * FROM table_robot WHERE robot_id = %s and fleet_id = %s;', (robot_id, fleet_id,))
                for record in self.cur.fetchall():
                        if (trigger is True) and (str(record['shutdown']) == 'no'):
                                self.shutdown_db_cmd('yes', robot_id, fleet_id)
                        elif (trigger is False) and (str(record['shutdown']) == 'yes'):
                                self.shutdown_db_cmd('no', robot_id, fleet_id)
                        else:
                                pass
        except Exception as error:
                print("[manager]:-fm_shutdown_trigger-"+str([error])+".")

    def shutdown_db_cmd(self, shutdown, robot_id, fleet_id):  # yes/no - shutdown
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

        global task_dictionary

        parts = str_location_id_comma_fleet.split(',')
        loc_, fleet_ = parts[0], parts[1]

        if (loc_ == None) or (fleet_ == None):
                print("[manager]:-ATTENTION- Invalid element in 'delete landmark' field.")
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

        time.sleep(0.5)

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
