#!/usr/bin/env python3

import numpy as np
import psycopg2, psycopg2.extras, collections
import time, io, cv2, yaml, os, signal, math, json, heapq, re
from pathlib import Path

import fleet_mngr_main as fm_main

import threading
import os, sys, time
import psycopg2, psycopg2.extras
# from unittest import TestCase, main

# Define the path to the directory containing 'db_register.py'
agv_config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
# Add the directory to the Python path
sys.path.append(agv_config_dir)
# Import the class from the module
import viro_simple_fleet.viro_simple_fleet.db_register as db_register # from x import DatabaseRegistration
import viro_simple_fleet.viro_simple_fleet.db_manager as db_manager


# typing too fast shpuld not give the feedback_current_pose[0] mentioned before declaration or something
# write function/conflict scenario tests

########################################################################
# agv_config_dir = (os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
# file_path = Path(agv_config_dir+'/viro_simple_fleet/config/fleet_mngr.yaml')
########################################################################
# print("import done.")



wheel_separation = 0.1
wheel_radius = 0.04
lin_vel = 0.4
ang_vel = 0.9
bat_stat = 50.0
hostname = "192.168.1.93"
database = "postgres"
username = "postgres"
pwd = "root"
port = 5432
map1_path = "/home/hazeezadebayo/docker_ws/ros2_ws/viro_agv/src/viro_core/viro_core/maps/my_cartographer_map.pgm"
map2_path = "/home/hazeezadebayo/docker_ws/ros2_ws/viro_agv/src/viro_core/viro_core/maps/devshop.pgm"


class TestFleetManager:
    """
    for quick fleet manager and client test, without ros updates.
    fc_db_register | fc_db_manager | fm_shutdown_trigger | fm_robot_status_trigger
    fm_manual_control_trigger | fm_send_task_request | fm_clear_task_trigger
    fm_add_landmark_request | fm_delete_landmark_request
    """
    def __init__(self):

        # wether or not to show matplotlib car
        self.show_robot_animation = True
        self.single_robot_view = False

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

        # initialize fleet dropdown menu
        self.job_types =['transport', 'loop', 'move', 'charge', 'clean']
        self.job_priorites = ['low', 'high', 'medium']
        self.station_type = ['charge_dock', 'station_dock', 'home_dock', 'elevator_dock', 'checkpoint', 'waitpoint']

        # initialize
        self.fm_main = fm_main.Ui_MainWindow()
        self.fm_main.min_task_bat_level = 35


    def set_active_agv(self, fleet_id, robot_id):
        """ timer callback function that updates db and nav variables. """
        self.fm_main.fm_start_fleet(fleet_id)
        time.sleep(1.5)
        self.fm_main.fm_start_robot(fleet_id, robot_id)
        time.sleep(1.5)

    def shutdown_trigger(self, fleet_id, robot_id, val):
        """ fm_shutdown_trigger """
        # answer = input('Alert! check yes or no for on or off? [yes, no]: ')
        self.fm_main.fm_shutdown_trigger(fleet_id, robot_id, val)
        #time.sleep(1.5)

    def robot_status_trigger(self, fleet_id, robot_id, val):
        """ fm_robot_status_trigger """
        # answer = input('Alert! check yes or no for active or inactive? [yes, no]: ')
        self.fm_main.fm_robot_status_trigger(fleet_id, robot_id, val)
        #time.sleep(1.5)

    def manual_control_trigger(self, fleet_id, robot_id, val):
        """ fm_manual_control_trigger """
        # answer = input('Alert! check yes or no to nav manually or not? [yes, no]: ')
        self.fm_main.fm_manual_control_trigger(fleet_id, robot_id, val)
        #time.sleep(1.5)

    def manual_control_drive(self, fleet_id, robot_id, choice):
        """ fm_manual_control_drive """
        # choice = input('Please enter the number of your choice: ')
        # Define the options
        options = [
            'left',
            'right',
            'forward',
            'backward',
            'stop'
        ]
        if not 1 <= int(choice) <= len(options):
            # Get the chosen option
            chosen_option = options[int(choice) - 1]
            # pass the chosen option to the db
            self.fm_main.fm_manual_control_drive(fleet_id, robot_id, chosen_option)

    def send_task_request(self, fleet_id, robot_id, from_loc_id, to_loc_id, task_name, task_priority):
        """ fm_send_task_request """
        # ---
        # from_loc_id = input('Please enter from_loc_id [options must conform to the pattern AnyAlphabet+<number>]: ')
        # to_loc_id = input('Please enter to_loc_id [options must conform to the pattern AnyAlphabet+<number>]: ')
        # task_name = input('Please enter task_name [move, loop, charge, clean, transport]: ')
        # task_priority = input('Please enter task_priority [low, high, medium]: ')
        # fm_main.fm_send_task_request("A5", "A3", "transport", "low")
        # ---
        self.fm_main.fm_send_task_request(fleet_id, robot_id, from_loc_id, to_loc_id, task_name, task_priority)
        # time.sleep(2.0)

    def clear_task_trigger(self, fleet_id, robot_id, val):
        """ fm_clear_task_trigger """
        # answer = input('WARNING! Are you sure you want to clear task and make robot idle? [yes, no]: ')
        self.fm_main.fm_clear_task_trigger(fleet_id, robot_id, val) # True
        # time.sleep(2.0)

    def add_landmark_request(self, fleet_id, robot_id, station_type:str, input_loc_id:str, neighbor_loc_ids:str):
        """ fm_add_landmark_request """
        # drive robot to the location you want to save as landmark.
        # use the interactive options to assign name and neighbour.
        # ----
        # station_type = input('Please enter station_type from: '+str(self.station_type)+'.')
        # input_loc_id = input('Please enter valid location/landmark id [options must conform to the pattern AnyAlphabet+<number>]: ')
        # neighbor_loc_ids = input('Please enter valid location/landmark id [options must conform to the pattern AnyAlphabet+<number>]: ')
        # ----
        self.fm_main.fm_add_landmark_request(fleet_id, robot_id, station_type, input_loc_id, neighbor_loc_ids)
        #time.sleep(1.5)

    def delete_landmark_request(self, input_str:str):
        """ fm_delete_landmark_request """
        # input_str = input('Please enter a string of the form AnyAlphabet<number>,<word>: ')
        self.fm_main.fm_delete_landmark_request(input_str)
        #time.sleep(1.5)


    def setup_robot(self, fleet_id, robot_id, current_pose, drop_table=False):
        """ test create robot """

        db_registration = db_register.DatabaseRegistration(
            robot_id, fleet_id, map1_path,
            map2_path, hostname, database, username,
            pwd, port, ang_vel, lin_vel, wheel_separation,
            wheel_radius, current_pose, bat_stat)

        db_registration.shutdown = 'no'
        db_registration.traffic = 'none'
        db_registration.wait_traffic = 'none'
        db_registration.last_checkpoint = 'unknown,'
        db_registration.checkpoints = 'A,A,A'
        db_registration.waitpoints = 'W,W,W'
        db_registration.notifications = 'notice_msg,notice_msg,node_troubleshoot,0,None,wt'
        db_registration.agv_status = 'active,x,y,z,w,base_floor,red'
        db_registration.m_controls = 'no,False,False,False,False,False'
        db_registration.landmark = 'none'
        db_registration.wait_itinerary = '{{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0}}'
        db_registration.agv_itinerary = '{{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0}}'

        # register robot on database and check status
        if db_registration.register_robot(drop_table) is False:
            print("[main]:- registration failed. status returned False. \n")
            return

        # Create instance of DatabaseManager
        db_mngr = db_manager.DatabaseManager(
            hostname=hostname, database=database,
            username=username, pwd=pwd, port=port,
            robot_id=robot_id, fleet_id=fleet_id
        )

        return db_mngr

    def assert_equal(self, first, second):
        """ checks if two values are equal and
        raises an exception with a descriptive error message if they are not """
        if first != second:
            raise AssertionError(f'AssertionError: {first} != {second}')
        else:
            # print(f'Test passed: {first} == {second}')
            pass


    def db_mngr_read(self, db_mngr):
        """Test read from the database."""
        db_mngr.read_db()
        # Check that instance variables are updated correctly
        # self.assert_equal(db_mngr.datetime_info, ['2024-08-11', '12:00:00'])
        # self.assert_equal(db_mngr.current_pose, ['1.0', '-3.0', '0.0'])
        self.assert_equal(db_mngr.shutdown, 'no')
        self.assert_equal(db_mngr.notifications, ['notice_msg', 'notice_msg', 'node_troubleshoot', '0', 'None', 'wt'])
        self.assert_equal(db_mngr.last_checkpoint, ['unknown',""])
        self.assert_equal(db_mngr.checkpoints, ['A','A','A'])
        self.assert_equal(db_mngr.waitpoints, ['W', 'W', 'W'])
        self.assert_equal(db_mngr.traffic, 'none')
        self.assert_equal(db_mngr.wait_traffic, 'none')
        self.assert_equal(db_mngr.wait_itinerary, [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])
        self.assert_equal(db_mngr.agv_status, ['active', 'x', 'y', 'z', 'w', 'base_floor', 'red'])
        self.assert_equal(db_mngr.agv_itinerary, [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])
        self.assert_equal(db_mngr.controls, ['no', 'False', 'False', 'False', 'False', 'False'])
        self.assert_equal(db_mngr.landmarks, ['none'])
        self.assert_equal(db_mngr.model_config, ['0.1', '0.04', '0.4', '0.9', '50.0'])
        #time.sleep(1.5)


    def db_mngr_update(self, db_mngr, current_position, last_checkpoint, btry_lvl):
        """Test updating the database."""
        x = round(float(current_position[0]), 3)
        y = round(float(current_position[1]), 3)
        theta = round(float(current_position[2]), 3)
        # read from db
        db_mngr.read_db()
        # values that we wish to update
        # usually position, battery and last_checkpoints.
        db_mngr.ros_system_launch_process = True
        db_mngr.ros_dummy_checkpoints = db_mngr.checkpoints
        db_mngr.ros_last_checkpoint = last_checkpoint # ['A8','A3','A8','A11','A12','A7','A5']
        db_mngr.ros_pose_x, db_mngr.ros_pose_y, db_mngr.ros_pose_th  = x, y, theta
        db_mngr.ros_battery_percentage = btry_lvl

        # startup_failed,notice_msg,warning_stop,0,None,wt
        # db_mngr.ros_notifications = []
        # -- if dock is completed becomes true
        # db_mngr.ros_dock_status = self.dock_status
        # -- if emergency button is pushed, robot becomes inactive
        # db_mngr.ros_emergency_present = self.emergency_present

        # Invoke the method
        db_mngr.update_db()
        #time.sleep(1.5)

        # Check that the database was updated
        db_mngr.read_db()
        self.assert_equal([round(float(db_mngr.current_pose[0]), 3),
                           round(float(db_mngr.current_pose[1]), 3),
                           round(float(db_mngr.current_pose[2]), 3)],
                          [x, y, theta])
        self.assert_equal(db_mngr.last_checkpoint, last_checkpoint)
        self.assert_equal(db_mngr.model_config, ['0.1', '0.04', '0.4', '0.9', str(btry_lvl)])
        #time.sleep(1.5)


    def test_update_db_with_shutdown(self, db_mngr):
        """Test database update when shutdown is triggered."""
        # read from db
        db_mngr.read_db()
        # test what happens if we shutdown mid task,
        # we surely want to continue from where we stopped yeah.
        # so we reshuffle the checkpoints so that
        # where we stopped becomes first item in checkpoints
        db_mngr.shutdown = 'yes'
        db_mngr.ros_dummy_checkpoints = db_mngr.checkpoints
        db_mngr.last_checkpoint = ['A11','A12','A7','A5']
        db_mngr.reshuffle_checkpoints()


    def convert_to_xytheta(self, itinerary):
        """
        Converts a list of [x, y, w, z] to [x, y, theta].

        Parameters:
        itinerary (list): A list containing x, y, w, z coordinates.

        Returns:
        tuple: A tuple containing x, y, and theta (orientation in radians).
        """
        if len(itinerary) != 4:
            raise ValueError("Input list must contain exactly four elements: [x, y, w, z]")
        x, y, w, z = itinerary
        # Calculate theta from quaternion components w and z
        theta = 2 * math.atan2(z, w)
        return [x, y, theta]


if __name__ == '__main__':

# ---------------------------------------------- #
#      # initialize test class                   #
# ---------------------------------------------- #

    fm = TestFleetManager()

    print("initialized fleet mngr test class.")
    print(" --        --        --        --")

# ---------------------------------------------- #
#      register robot 1                          #
# ---------------------------------------------- #

    ROBOT1_ID = 'TB3_15'
    FLEET1_ID = 'kullar'
    R1_CURRENT_POSE = [-1.6, 0.52, 0.0] # A11
    r1_db_mngr = fm.setup_robot(FLEET1_ID, ROBOT1_ID, R1_CURRENT_POSE, True)
    # verify robot1 registeration with assert
    # fm.db_mngr_read(r1_db_mngr)
    print("Registered robot 1.")
    print(" --        --        --        --")


# ---------------------------------------------- #
#       register robot 2                         #
# ---------------------------------------------- #

    ROBOT2_ID = 'TB3_16'
    FLEET2_ID = 'kullar'
    R2_CURRENT_POSE = [-1.4, -1.3, 0.0] # A12
    r2_db_mngr =fm.setup_robot(FLEET2_ID, ROBOT2_ID, R2_CURRENT_POSE)
    # verify robot2 registeration with assert
    # fm.db_mngr_read(r2_db_mngr)
    print("Registered robot 2.")
    print(" --        --        --        --")

# ---------------------------------------------- #
#       R1 task request                          #
# ---------------------------------------------- #

    fm.set_active_agv(FLEET1_ID, ROBOT1_ID)
    print("active robot set to robot 1.")
    print(" --        --        --        --")
    # clear robot task
    # ------------
    # time.sleep(2.0)
    fm.clear_task_trigger(FLEET1_ID, ROBOT1_ID, val=True)
    print("robot 1 task cleared.")
    print(" --        --        --        --")
    # time.sleep(2.0)
    fm.send_task_request(
        fleet_id=FLEET1_ID,
        robot_id=ROBOT1_ID,
        from_loc_id="A5",
        to_loc_id="A3",
        task_name="transport",
        task_priority="high")
    print("task requested for robot 1.")
    print(" --        --        --        --")

# ---------------------------------------------- #
#      R2 task request                           #
# ---------------------------------------------- #
    ###########################################
    # For head on collision uncomment the below
    ###########################################
    # time.sleep(5.0)
    fm.set_active_agv(FLEET2_ID, ROBOT2_ID)
    print("active robot set to robot 2.")
    print(" --        --        --        --")
    # clear robot task
    # ------------
    # time.sleep(2.0)
    fm.clear_task_trigger(FLEET2_ID, ROBOT2_ID, val=True)
    print("robot 2 task cleared.")
    print(" --        --        --        --")
    # time.sleep(2.0)
    fm.send_task_request(
        fleet_id=FLEET2_ID,
        robot_id=ROBOT2_ID,
        from_loc_id="A3",
        to_loc_id="A5",
        task_name="transport",
        task_priority="low")
    print("task requested for robot 2. \n")

    print(" --        --        --        --")

    ###########################################
    # For follow for follow uncomment the below
    ###########################################
    # # time.sleep(5.0)
    # fm.set_active_agv(FLEET2_ID, ROBOT2_ID)
    # print("active robot set to robot 2.")
    # print(" --        --        --        --")
    # # clear robot task
    # # ------------
    # # time.sleep(2.0)
    # fm.clear_task_trigger(FLEET2_ID, ROBOT2_ID, val=True)
    # print("robot 2 task cleared.")
    # print(" --        --        --        --")
    # # time.sleep(2.0)
    # fm.send_task_request(
    #     fleet_id=FLEET2_ID,
    #     robot_id=ROBOT2_ID,
    #     from_loc_id="A5",
    #     to_loc_id="A3",
    #     task_name="transport",
    #     task_priority="low")
    # print("task requested for robot 2. \n")

    # print(" --        --        --        --")

# ---------------------------------------------- #
#     Handshake (R1 update)                      #
# ---------------------------------------------- #

    # time.sleep(5.0)
    # the robot needs to update last_checkpoint as an handshake.
    r1_db_mngr.read_db()
    itnrry = r1_db_mngr.agv_itinerary[1]
    R1_CURRENT_POSE = fm.convert_to_xytheta(itnrry)
    R1_LAST_CHECKPOINT = r1_db_mngr.checkpoints
    print("R1 going to [",R1_LAST_CHECKPOINT[0],"]")
    R1_BATTERY = 45.0
    fm.db_mngr_update(r1_db_mngr,
                      R1_CURRENT_POSE,
                      R1_LAST_CHECKPOINT,
                      R1_BATTERY)

    print("R1 handshake completed.")

    print(" --        --        --        --")

# ---------------------------------------------- #
#     Handshake (Rw update)                      #
# ---------------------------------------------- #

    # time.sleep(5.0)
    r2_db_mngr.read_db()
    itnrry = r2_db_mngr.agv_itinerary[1]
    R2_CURRENT_POSE = fm.convert_to_xytheta(itnrry)
    R2_LAST_CHECKPOINT = r2_db_mngr.checkpoints
    R2_BATTERY = 46.0
    fm.db_mngr_update(r2_db_mngr,
                      R2_CURRENT_POSE,
                      R2_LAST_CHECKPOINT,
                      R2_BATTERY)
    print("R2 going to [",R2_LAST_CHECKPOINT[0],"]")
    print("R2 handshake completed.")
    # assert that the respective traffics are updated
    # i.e. robots reserve the their closest nodes
    # and flag is green

# ---------------------------------------------- #
#  test for unexpected shutdown or pause state   #
# ---------------------------------------------- #

    # turn robot on or off
    # fm.shutdown_trigger(FLEET1_ID, ROBOT1_ID, val=True)

    # # set robot active or inactive
    # fm.robot_status_trigger(FLEET1_ID, ROBOT1_ID, val=False)

    print(" --        --        --        --")

# ---------------------------------------------- #
#      test conflict resolution                  #
# ---------------------------------------------- #

    # move to the next node, this will coincide with R2.
    # as R2 has already reserved the node for itself.

    print("R1 will try to change nodes now.")
    # time.sleep(5.0)
    r1_db_mngr.read_db()
    itnrry = r1_db_mngr.agv_itinerary[1]
    R1_CURRENT_POSE = fm.convert_to_xytheta(itnrry)
    # the robot needs to update last_checkpoint as an handshake.
    R1_LAST_CHECKPOINT = r1_db_mngr.checkpoints[1:]
    R1_BATTERY = 42.0
    fm.db_mngr_update(r1_db_mngr,
                      R1_CURRENT_POSE,
                      R1_LAST_CHECKPOINT,
                      R1_BATTERY)
    # print("R1 going to [",R1_LAST_CHECKPOINT[0],"]")
    # print("R1 handshake completed.")
    print(" --        --        --        --")

# ---------------------------------------------- #
#      test conflict resolution                  #
# ---------------------------------------------- #

    # robot2 will get a red flag as it has now arrived at A12
    # and desires to reserve A11, which is already occupied by R1

    print("R2 will try to change nodes now.")
    # time.sleep(5.0)
    r2_db_mngr.read_db()
    itnrry = r2_db_mngr.agv_itinerary[1]
    R2_CURRENT_POSE = fm.convert_to_xytheta(itnrry)
    R2_LAST_CHECKPOINT = r2_db_mngr.checkpoints[1:]
    R2_BATTERY = 39.0
    fm.db_mngr_update(r2_db_mngr,
                      R2_CURRENT_POSE,
                      R2_LAST_CHECKPOINT,
                      R2_BATTERY)
    print(" --        --        --        --")


# ---------------------------------------------- #
#      test manual drive                         #
# ---------------------------------------------- #

    # # manual mode on or off
    # fm.manual_control_trigger(val=True)

    # # 'left' 1, 'right' 2, 'forward' 3, 'backward' 4, 'stop' 5
    # fm.manual_control_drive(choice=1)

# ---------------------------------------------- #
#      test add and delete landmark              #
# ---------------------------------------------- #

    # # add landmark
    # fm.add_landmark_request(station_type="home_dock",
    #                             input_loc_id="A20",
    #                             neighbor_loc_ids="A4,A2,A3,A6")

    # # delete landmark
    # fm.delete_landmark_request(input_str="A20")
