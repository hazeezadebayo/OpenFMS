#! /usr/bin/env python3

# pylint: disable=multiple-imports

from pathlib import Path
from copy import deepcopy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped # ,Point ,Pose,Quaternion,Vector3,
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState # LaserScan, Image,
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import MarkerArray, Marker
from viro_simple_fleet.fleet_client_custom_plugin import FleetClientCustomPlugin
from paho.mqtt import client as mqtt_client
from paho.mqtt.client import error_string
import numpy as np
import os, rclpy, warnings, yaml, time, sys, ast, math, gc, signal, psycopg2, psycopg2.extras, collections, cv2 # , argparse, threading, csv,
import json, uuid, datetime, random, threading

warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

# important!!!
# cd colcon_ws/src/viro_core/viro_core/scripts ---$ chmod +x fleet_client_navigator.py

# USAGE:
# Terminal 1:
# cd docker_ws/env/dev1
# export DISPLAY=:0.0
# xhost +local:docker
# docker compose up --build
# or
# docker run -it dev3-tailscaled /bin/bash

# -------------- NO MQTT?
# Terminal 2:
# docker exec -it dev3-tailscaled-1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch viro_simple_fleet fleet_client.launch.py

# Terminal 3:
# docker exec -it dev3-tailscaled-1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; python3 FleetManager/viro_simple_fleet/scripts/fleet_mngr_main.py

# -------------- MQTT?
# Terminal 2:
# docker exec -it dev3-tailscaled-1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch vda5050_tb3_adapter connector_tb3.launch.py

# Terminal 3:
# docker exec -it dev3-tailscaled-1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash;

# ----------------------------------------------------------------------- #
#
#        IF, START NAVIGATION:
#                ----L----
#               0-   x   -0
#                |       |
#                |       |
#               0-       -0
#                ---------
#
# ----------------------------------------------------------------------- #
#               AGV CLIENT DatabaseRegistration                           #
# ----------------------------------------------------------------------- #

# what happens when/while i am there. this one can handle that.
class FleetClientNavigator(Node):
    """ FleetClientNavigator:  move the robot based on required task in session dictionary """
    def __init__(self):
        super().__init__('fleet_client_navigator')

        package_name = 'viro_simple_fleet'
        self.package_share_directory = get_package_share_directory(package_name)

        self.group1 = ReentrantCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        self.inspection_pose = PoseStamped()
        self.msg_lc = String()
        self.msg_notice = String()

        self.last_checkpoint_pub = self.create_publisher(String,'/'+package_name+'/last_checkpoints', 1)
        self.simple_fleet_notice_pub = self.create_publisher(String,'/'+package_name+'/external_notice', 1)
        self.marker_publisher = self.create_publisher(MarkerArray,"/cpp_markers", 1)

        self.notification_stat_sub = self.create_subscription(String,'/'+package_name+'/internal_notice', self.simple_fleet_int_cb, 1, callback_group=self.group1)

        self.fleet_base_map_yaml, self.fleet_floor1_map_yaml, self.fleet_coverage_plan_yaml = None, None, None
        self.inspection_points = []
        self.notification_ext = None
        self.last_docked_station = None
        self.count = 0
        self.negotiated_route = [] # x, y
        self.negotiation_stages = [False, False, False]
        self.checkpoints, self.path_dict, self.waitpoints, self.wait_path_dict = [],[],[],[]
        self.landmarks, self.predecessor_landmark, self.real_landmark = [],[],[]
        self.navigator_update = False
        self.interrupt = False

        self.version = None
        self.order_topic = None
        self.instant_action_topic = None
        self.mqtt_client = None
        self.order_update = 0
        self.i = 0
        self.base_nodes = []
        self.flag = "red"

        self.client_plugin = FleetClientCustomPlugin()

        # timer callback
        self.startup_init = False
        self.timer = self.create_timer(1, self.startup_cb, callback_group=self.group1)

    def startup_cb(self):
        """ read the session dictionary and start the called task """
        if not self.startup_init:
            print("This block of code will run as long as no session message has been read.")
            # Wait for the first file to be available
            info_path = Path(self.package_share_directory+"/config/session_info.yaml")
            try:
                with open(info_path, encoding='utf-8') as yaml_file:
                    try:
                        # get yaml file content safely
                        agv_dict =  yaml.safe_load(yaml_file)

                        # read session dictionary and obtain session constants.
                        init_pose = ast.literal_eval(str(agv_dict["initial_pose"]))
                        self.path_dict = ast.literal_eval(str(agv_dict["agv_itinerary"]))
                        self.checkpoints = ast.literal_eval(str(agv_dict["checkpoints"]))
                        self.landmarks = ast.literal_eval(str(agv_dict["landmarks"]))
                        self.wait_path_dict = ast.literal_eval(str(agv_dict["wait_itinerary"]))
                        self.waitpoints = ast.literal_eval(str(agv_dict["waitpoints"]))
                        update_time = float(agv_dict["update_time"])
                        curr_dock =  ast.literal_eval(str(agv_dict["curr_dock"]))
                        self.fleet_base_map_yaml = str(agv_dict["fleet_base_map_yaml"])
                        self.fleet_floor1_map_yaml = str(agv_dict["fleet_floor1_map_yaml"])
                        self.fleet_coverage_plan_yaml = str(agv_dict["fleet_coverage_plan_yaml"])
                        startup_map = str(agv_dict["startup_map"])

                        # mqtt stuff?
                        self.client_plugin.use_vda5050 = bool(str(agv_dict["use_vda5050"]))
                        self.version = str(agv_dict["version"])
                        self.client_plugin.manufacturer = str(agv_dict["manufacturer"])
                        self.client_plugin.serial_number = str(agv_dict["serial_number"])
                        self.client_plugin.connection_topic = str(agv_dict["connection_topic"])
                        self.client_plugin.state_topic = str(agv_dict["state_topic"])
                        self.order_topic = str(agv_dict["order_topic"])
                        self.instant_action_topic = str(agv_dict["instant_action_topic"])
                        mqtt_address = str(agv_dict["mqtt_address"])
                        mqtt_port = str(agv_dict["mqtt_port"])
                        # mqtt_username = str(agv_dict["mqtt_username"])
                        # mqtt_password = str(agv_dict["mqtt_password"])

                        if self.client_plugin.use_vda5050 is True:
                            # Define the MQTT client
                            self.mqtt_client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1)
                            # Set the callback function
                            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
                            # Connect to the MQTT broker
                            # self.mqtt_client.username_pw_set(mqtt_username, mqtt_password)
                            self.mqtt_client.connect(host=str(mqtt_address), port=int(mqtt_port)) # keep alive for 60secs
                            self.mqtt_client.loop_start()
                            self.get_logger().info("[client]:- mqtt connection done-. \n")

                        # what landmarks are available? what kind of task is this?
                        if (len(self.landmarks) != 0) and (self.landmarks[1] != "clean"):
                            for elements in self.landmarks[2:]:
                                self.predecessor_landmark.append(elements.split('_')[0])
                                self.real_landmark.append(elements.split('_')[1])

                        # check if the file found has the most recent update (within mins)
                        if (time.time_ns() - update_time)*1e-9 < 240:
                            if len(self.path_dict) != 0:
                                self.startup_init = True
                                self.navigator_update = True
                                self.timer.cancel()
                                self.client_plugin.on_startup(init_pose)
                                self.get_logger().info("[client]:- onstartup called-. \n")
                                # self.client_plugin.startup_config_check()
                                # self.get_logger().info("[client]:- startup config called-. \n")
                                if startup_map == 'floor1_map':
                                    self.client_plugin.load_map(self.fleet_floor1_map_yaml)
                                elif startup_map == 'base_map':
                                    self.client_plugin.load_map(self.fleet_base_map_yaml)
                                # load the path or necessary goal messages
                                self.route_pose_callback()
                                # i was never at the dock station to begin with.
                                if str(curr_dock[0]) == 'x':
                                    self.inspect_route()
                                # i was at the dock and i know my address.
                                else: # agv_status[1:] => [x,y,z,w]
                                    self.notification_ext = 'undock_required'
                                    self.simple_fleet_ext_pub(self.notification_ext)
                                    # undock [x,y,z,w]
                                    status = self.dock_undock_io(float(curr_dock[0]),
                                                                  float(curr_dock[1]),
                                                                  float(curr_dock[2]),
                                                                  float(curr_dock[3]))
                                    if status is True:
                                        self.notification_ext = 'undock_completed'
                                        self.simple_fleet_ext_pub(self.notification_ext)
                                        self.count += 1
                                        self.inspect_route()
                    except TypeError:
                        pass # print("Caught TypeError, continuing loop...")
            except FileNotFoundError as e:
                print(f"Waiting for session yaml... ({e})")
                time.sleep(0.1)

        else:
            pass

        gc.collect() # print("Garbage collector: collected %d objects." % (gc.collect()))



# ----------------------------------------------------------------------- #
#                            MAIN CALLBACK                                #
# ----------------------------------------------------------------------- #

    def route_pose_callback(self):
        """ convert path dictionary to ros message type """
        self.inspection_points = []
        self.inspection_pose.header.frame_id = "map"
        self.inspection_pose.header.stamp = self.get_clock().now().to_msg()
        # check if its a cleaning task then get all the waypoints from a different yaml.
        if self.landmarks[1] == "clean":
            coverage_path_dict = self.read_yaml_file(self.fleet_coverage_plan_yaml)
            if coverage_path_dict is None:
                print('[cov_path_plan]: Could not find cleaning coverage path yaml...')
                return
            # Use items() for iterating over dictionary key-value pairs.
            for i, pose_dict in coverage_path_dict.items():
                print(f"Processing coverage pose {i}...") # show me who you are!
                # Skip 'updatetime' or any other non-dictionary entries.
                if not isinstance(pose_dict, dict):
                    continue
                # Skip entries without 'position'.
                if 'position' not in pose_dict:
                    continue
                # Directly unpack position and orientation from pose_dict.
                self.inspection_pose.pose.position.x = pose_dict["position"]["x"]
                self.inspection_pose.pose.position.y = pose_dict["position"]["y"]
                self.inspection_pose.pose.orientation.z = pose_dict["orientation"]["z"]
                self.inspection_pose.pose.orientation.w = pose_dict["orientation"]["w"]
                # coverage_path_poses: append to goal poses.
                self.inspection_points.append(deepcopy(self.inspection_pose))
            self.publish_waypoint_markers()
            self.get_logger().info("[client]-cov_path_plan fetched path dict-:. \n")
            return

        for i, waypoint in enumerate(self.path_dict):
            if self.client_plugin.use_vda5050 is True:
                self.inspection_pose.header.frame_id = self.checkpoints[i]
            self.inspection_pose.pose.position.x =  float(waypoint[0])
            self.inspection_pose.pose.position.y = float(waypoint[1])
            self.inspection_pose.pose.orientation.z = float(waypoint[2])
            self.inspection_pose.pose.orientation.w = float(waypoint[3])
            self.inspection_points.append(deepcopy(self.inspection_pose))
        self.get_logger().info("[client]-path_created-:. \n")

    def inspect_route(self):
        """ perform actual navigation here with nav2 simple api calls """

        while True:

            rclpy.spin_once(self, timeout_sec=1.0)

            if self.navigator_update is True:
                self.navigator_update = False

                self.i = 0
                # sanity check a valid path exists
                # For when there's more than one pose:
                # path = navigator.getPathThroughPoses(initial_pose, goal_poses)
                # if only one pose:
                # path = navigator.getPath(initial_pose, goal_pose)
                # smoothed_path = navigator.smoothPath(path)
                while self.count < len(self.inspection_points):

                    rclpy.spin_once(self, timeout_sec=1.0)

                    # return    # exit the while loop & function.
                    # break     # exit the while loop but not the function.
                    # continue  # skip this iteration continue the while loop though.
                    # we wanna print after every count 10
                    self.i += 1
                    if self.i % 10 == 0:

                        # fetch the next target too
                        next_stop_coordinate = [self.inspection_points[self.count].pose.position.x,
                                                self.inspection_points[self.count].pose.position.y,
                                                self.inspection_points[self.count].pose.orientation.z,
                                                self.inspection_points[self.count].pose.orientation.w]

                        # if the task at hand is not a cleaning task:
                        if self.landmarks[1] != "clean":
                            # this publishes the remaining alphabets from where we wanna go or currently
                            # headed to the last location we will be visiting. keep track of checkpoints
                            # in these alphabets, elevators might be amongst.
                            # only thing we know about elevators is that they begin with E.
                            self.last_checkpoint_publisher(self.checkpoints[self.count:])
                            # self.get_logger().info("[client]-next_loc-:["+str(self.checkpoints[self.count:][0])+"]. \n")
                        else:
                            # just publish total visited against total required visit.
                            self.last_checkpoint_publisher(['clean',str(self.count),'/',str(len(self.inspection_points))])

                        # always check if there is a negotiated path first.
                        # a negotiated path is like an alternative route for the robot,
                        # to avoid robot stand-off, a situation where a robotA occupies a checkpoint where robotB wants to go,
                        # while robotB coincindentally occupies the checkpoint where robotA wants to go.

                        if self.flag == "red":
                            continue

                        if len(self.negotiated_route) != 0:
                            self.get_logger().info("[client]-negotiation drive-. \n")
                            self.notification_ext = 'negotiation_required'
                            self.simple_fleet_ext_pub(self.notification_ext)
                            if self.count != 0:
                                if self.client_plugin.use_vda5050 is True:
                                    if self.negotiation_stages[0] is False:
                                        # go back to previous node:
                                        # self.get_logger().info("[client]:-1-["+str(self.count)+"]. \n")
                                        # self.get_logger().info("[client]:-2-["+str(self.checkpoints)+"]. \n")
                                        prev_stop_id = self.checkpoints[self.count-1]
                                        # self.get_logger().info("[client]:-3-["+str(prev_stop_id)+"]. \n")
                                        prev_stop_coordinate = self.find_coordinates_of_point(prev_stop_id, self.checkpoints, self.path_dict)
                                        # self.get_logger().info("[client]:-4-["+str(prev_stop_coordinate)+"]. \n")
                                        prev_stop_coordinate.header.frame_id = str(prev_stop_id)
                                        result = self.vda5050_connector_drive([prev_stop_coordinate])
                                        self.get_logger().info("[client]- Negotiation stage 1! ")
                                    elif self.negotiation_stages[1] is False:
                                        # then go to wait node and sleep for wait_time seconds:
                                        wait_stop_id = self.negotiated_route[1]
                                        wait_stop_coordinate = self.find_coordinates_of_point(wait_stop_id, self.waitpoints, self.wait_path_dict)
                                        wait_stop_coordinate.header.frame_id = "wait_"+str(wait_stop_id)
                                        result = self.vda5050_connector_drive([wait_stop_coordinate])
                                        self.get_logger().info("[client]- Negotiation stage 2! ")
                                    elif self.negotiation_stages[2] is False:
                                        # then go back to previous node again before continuing to path_dict:
                                        prev_stop_id = self.checkpoints[self.count-1]
                                        prev_stop_coordinate = self.find_coordinates_of_point(prev_stop_id, self.checkpoints, self.path_dict)
                                        prev_stop_coordinate.header.frame_id = str(prev_stop_id)
                                        result = self.vda5050_connector_drive([prev_stop_coordinate])
                                        self.get_logger().info("[client]- Negotiation stage 3! ")
                                else:
                                    if self.negotiation_stages[0] is False:
                                        # go back to previous node:
                                        prev_stop_id = self.checkpoints[self.count-1]
                                        p_s_c = self.find_coordinates_of_point(prev_stop_id, self.checkpoints, self.path_dict)
                                        prev_stop_coordinate = [p_s_c.pose.position.x,p_s_c.pose.position.y,p_s_c.pose.orientation.z,p_s_c.pose.orientation.w]
                                        result = self.client_plugin.drive(prev_stop_coordinate)
                                    elif self.negotiation_stages[1] is False:
                                        # then go to wait node and sleep for wait_time seconds:
                                        wait_stop_id = self.negotiated_route[1]
                                        w_s_c = self.find_coordinates_of_point(wait_stop_id, self.waitpoints, self.wait_path_dict)
                                        wait_stop_coordinate = [w_s_c.pose.position.x,w_s_c.pose.position.y,w_s_c.pose.orientation.z,w_s_c.pose.orientation.w]
                                        result = self.client_plugin.drive(wait_stop_coordinate)
                                    elif self.negotiation_stages[2] is False:
                                        # then go back to previous node again before continuing to path_dict:
                                        prev_stop_id = self.checkpoints[self.count-1]
                                        p_s_c = self.find_coordinates_of_point(prev_stop_id, self.checkpoints, self.path_dict)
                                        prev_stop_coordinate = [p_s_c.pose.position.x,p_s_c.pose.position.y,p_s_c.pose.orientation.z,p_s_c.pose.orientation.w]
                                        result = self.client_plugin.drive(prev_stop_coordinate)

                        else:
                            # go to goal/target
                            if self.client_plugin.use_vda5050 is True:
                                result = self.vda5050_connector_drive([self.inspection_points[self.count]])
                            else:
                                result = self.client_plugin.drive(next_stop_coordinate)

                        # implement a better logic.
                        if result is None:
                            continue

                        # did the nav succeed?
                        if result is True:
                            # self.get_logger().info("[client]-self.client_plugin.recovery_retry_counter-:["+str(self.client_plugin.recovery_retry_counter)+"]. \n")
                            # move on to real business.
                            if self.client_plugin.recovery_retry_counter == 0:
                                if len(self.negotiated_route) != 0:
                                    # remind me of where i was headed before i got lost.
                                    if self.negotiation_stages[0] is False:
                                        self.negotiation_stages[0] = True
                                    elif self.negotiation_stages[1] is False:
                                        self.negotiation_stages[1] = True
                                        wait_time = self.negotiated_route[0]
                                        time.sleep(wait_time) # then sleep for wait_time seconds.
                                    elif self.negotiation_stages[2] is False:
                                        self.negotiation_stages[2] = True
                                        self.negotiated_route = []
                                        self.negotiation_stages = [False, False, False]
                                        self.notification_ext = 'negotiation_completed'
                                        self.simple_fleet_ext_pub(self.notification_ext)
                                    # Skip the rest of this iteration and proceed to the next iteration in while loop
                                    continue
                                if self.landmarks[1] != "clean":
                                    # is this place a dock station:
                                    # checkpoint has already been reserved
                                    for landmark in self.real_landmark:
                                        # if it is a landmark and its not the place we just docked.
                                        # i mean! we didnt just dock here. then, okay it must be that we need to dock.
                                        if (landmark == self.checkpoints[self.count:][0]) and \
                                            (self.last_docked_station != self.checkpoints[self.count:][0]):
                                            # print("0. self.checkpoints: ", self.checkpoints, self.count)
                                            self.notification_ext = 'dock_required'
                                            self.simple_fleet_ext_pub(self.notification_ext)
                                            status = self.dock_undock_io() # dock
                                            if status is True:
                                                self.notification_ext = 'dock_completed'
                                                self.simple_fleet_ext_pub(self.notification_ext)

                                                # 1 landmark (charge) - charge | 3 landmarks (pick, drop, home > 1) - transport | 2 landmarks (pick, drop) - loop
                                                # landmark = [task_priority 0, task_name 1, pick 2, drop 3, home_docks 4..]
                                                self.last_docked_station = self.checkpoints[self.count:][0]

                                                # if the job was a charge or move:
                                                if self.landmarks[1] == "charge" or self.landmarks[1] == "move":
                                                    # then navigation is complete, sleep/power off or cancel nav.
                                                    # db_manager will change the state to idle.
                                                    self.get_logger().info("[client]-task completed-. \n")
                                                    self.cancel_navigation()
                                                    # print('[client]: Navigation task completed successfully!')
                                                    return

                                                # however, if the task is a loop,
                                                if self.landmarks[1] == "loop":
                                                    # loop task itinerary looks like this:  [some_place, waypoints, pick, waypoints, drop, waypoints, pick]
                                                    # so, check if it is "pick" and first (i.e. theres more to the side):
                                                    #  --> we need to cut off any checkpoint before it [del some_place, waypoints]
                                                    #      and reset path_dict. then we reset count to 0 and call 'inspect_route'
                                                    # and check if it is "pick" and last:
                                                    #  --> simply reset count to 0 then call 'self.inspect_route()' again
                                                    # lastly, if it is "drop":
                                                    # most likely in the middle locations anyway so, go to the next go to the next.
                                                    #  --> self.inspect_route() without altering count.
                                                    # if self.checkpoints[self.count:][0] == self.landmarks[2] and len(self.checkpoints[self.count:]) > 1: # robot at first pickup.
                                                    if self.checkpoints[self.count:][0] == self.real_landmark[0] and len(self.checkpoints[self.count:]) > 1:
                                                        # recall that loop tasks always start with a high priority then goes on to low priority after first pickup.
                                                        # well, look here:
                                                        # if the pickup location that we just confirmed we are in, is not the first element in checkpoint. i.e. the index > 1.
                                                        # and we looked at the priority and its high. then for sure we just made our first pickup.
                                                        # if self.checkpoints.index(self.landmarks[2]) > 0 and self.landmarks[0] == "high":
                                                        if self.checkpoints.index(self.real_landmark[0]) > 0 and self.landmarks[0] == "high":
                                                            # we shall proceed to delete the entries before this point, signifying the path from wherever the robot got the task
                                                            # to the pickup location, because since its a loop we need a detour free trajectory.
                                                            print("There are elements before the first occurrence of 'self.landmarks[2]'")
                                                            # while self.checkpoints.index(self.landmarks[2]) > 0:
                                                            while self.checkpoints.index(self.real_landmark[0]) > 0:
                                                                # trim our paths:
                                                                self.checkpoints.pop(0)
                                                                self.path_dict.pop(0) # this one is loaded once anyway
                                                                self.inspection_points.pop(0)
                                                        else:
                                                            print("There are no elements before the first occurrence of 'self.landmarks[2]'")
                                                        # we need to reset count so that it aligns with our newly trimmed paths.
                                                        self.count = 0
                                                        # we cant publish undock required here because it might depend on some IO state
                                                        # to know if we picked/dropped the stuff before actually undocking
                                                    # elif self.checkpoints[self.count:][0] == self.landmarks[2] and len(self.checkpoints[self.count:]) == 1:
                                                    elif self.checkpoints[self.count:][0] == self.real_landmark[0] and len(self.checkpoints[self.count:]) == 1:
                                                        self.count = 0
                                                    # elif self.checkpoints[self.count:][0] == self.landmarks[3] and len(self.checkpoints[self.count:]) > 1:
                                                    elif self.checkpoints[self.count:][0] == self.real_landmark[1] and len(self.checkpoints[self.count:]) > 1:
                                                        # self.route_stat = 'move_base' | undock_required
                                                        pass
                                                    # recall! when you call dock
                                                    # you actually drive forward away from known coordinate i.e. 'next_stop_coordinate',
                                                    # therefore, ideally, an undock action should get you back to 'next_stop_coordinate'.
                                                    # so to avoid confusion lets say
                                                    undock_coord = next_stop_coordinate
                                                    # we call undock with 'undock target' however, we need to wait for true or false for dock status
                                                    self.notification_ext = 'undock_required'
                                                    self.simple_fleet_ext_pub(self.notification_ext)
                                                    # undock [x,y,z,w]
                                                    status = self.dock_undock_io(undock_coord[0], undock_coord[1], undock_coord[2], undock_coord[3])
                                                    if status is True: # which it should.
                                                        self.notification_ext = 'undock_completed'
                                                        self.simple_fleet_ext_pub(self.notification_ext)
                                                        # i am not sure but ideally, if we didnt +1 here, it shouldnt
                                                        # affect code behavior since we set last docked station so code would not
                                                        # enter this place, and since next_stop_coordinate would have still been our
                                                        # target. it would have not navigated and instead yield reached/success.
                                                        # hence, it would have +1 in the 'if len(self.inspection_points[self.count:]) > 1:' code block
                                                        continue # self.navigator_update = True
                                                    else:
                                                        # what could have caused this? call for help
                                                        print("undock failed.")

                                                # also, if the job was a transport:
                                                if self.landmarks[1] == "transport":
                                                    # check if it is "pick" or "drop":
                                                    # --> go to the next -- # self.inspect_route()
                                                    # check if it is "home":
                                                    # sleep/power off or cancel nav
                                                    self.get_logger().info("[client]:-transport-. \n")
                                                    # if (self.checkpoints[self.count:][0] == self.landmarks[2] or self.checkpoints[self.count:][0] == self.landmarks[3]) and len(self.checkpoints[self.count:]) > 1: # robot at pickup or drop loc
                                                    if (self.checkpoints[self.count:][0] == self.real_landmark[0] or self.checkpoints[self.count:][0] == self.real_landmark[1]) and len(self.checkpoints[self.count:]) > 1:
                                                        # similarly,
                                                        undock_coord = next_stop_coordinate
                                                        # we call undock with 'undock target' however, we need to wait for true or false for dock status
                                                        self.notification_ext = 'undock_required'
                                                        self.simple_fleet_ext_pub(self.notification_ext)
                                                        # undock [x,y,z,w]
                                                        status = self.dock_undock_io(undock_coord[0], undock_coord[1], undock_coord[2], undock_coord[3])
                                                        self.get_logger().info("[client]:-undock_successful-["+str(status)+"]. \n")
                                                        if status is True: # which it should.
                                                            self.notification_ext = 'undock_completed'
                                                            self.simple_fleet_ext_pub(self.notification_ext)
                                                            continue # self.navigator_update = True
                                                    # we are at the home dock station.
                                                    # elif self.checkpoints[self.count:][0] in self.landmarks[4:]:
                                                    elif self.checkpoints[self.count:][0] in self.real_landmark[2:]:
                                                        # db_manager will change the state to idle.
                                                        self.cancel_navigation() # you may break the code/end here
                                                        print('[client]: Navigation task completed successfully!')
                                                        return
                                        else:
                                            if self.notification_ext == 'undock_required':
                                                undock_coord = next_stop_coordinate
                                                status = self.dock_undock_io(undock_coord[0], undock_coord[1], undock_coord[2], undock_coord[3])
                                                if status is True: # which it should.
                                                    self.notification_ext = 'undock_completed'
                                                    self.simple_fleet_ext_pub(self.notification_ext)
                                                    self.count += 1 # not sure about this!!
                                                    continue # self.navigator_update = True

                                # if we are not at a dock station,
                                # then check if theres more point to be visited
                                if len(self.inspection_points[self.count:]) > 1:
                                    # before we leave here, we need to check, if this current checkpoint,
                                    # or whatever location this is, is an elevator.
                                    # because if it is, we need to change map. the only person who has information about
                                    # elevators is the "self.checkpoints[self.count:][0]"
                                    # if the first letter in this checkpoint is E. then we need to switch map. and then
                                    # publish a notification that we changed map. so that we can modify 'agv_status[5]' to
                                    could_be_an_elevator = self.checkpoints[self.count:][0]
                                    if could_be_an_elevator[0] == 'E':
                                        # 92percent sure its an elevator.
                                        print("E. for Elevator encountered request map switch.")
                                        # we need to request a switch and get confirmation.
                                        # this process also waits a while because we dont know how long the ride
                                        # in the elevator is:
                                        self.notification_ext = 'elevator_required'
                                        self.simple_fleet_ext_pub(self.notification_ext)
                                        while not self.client_plugin.state_io(self.notification_ext) :
                                            # waste time here until the state_io(type) function returns true.
                                            time.sleep(0.05)
                                        self.cancel_navigation()
                                        return
                                    # go to the next target
                                    print('[client]: Goal succeeded! Moving to the next goal.')
                                    self.notification_ext = 'waypoint'
                                    while not self.client_plugin.state_io(self.notification_ext) :
                                        # waste time here until the state_io(type) function returns true.
                                        time.sleep(0.05)
                                    # move on to the next +1:
                                    self.count += 1
                                    continue
                                else:
                                    # kill the navigator and return
                                    self.cancel_navigation()
                                    print('[client]: Navigation task completed successfully!')
                                    return
                            elif self.client_plugin.recovery_retry_counter != 0 and self.client_plugin.recovery_retry_counter <= self.client_plugin.recovery_retry_max:
                                # we failed the actual task but succeeded in the recovery.
                                # we need to go to same goal again.
                                continue
                            else:
                                if self.client_plugin.use_vda5050 is True:
                                    # Disconnect from the MQTT broker
                                    self.mqtt_client.disconnect()
                                self.client_plugin.on_exit()
                                self.notification_ext = 'recovery_required'
                                self.simple_fleet_ext_pub(self.notification_ext)
                                raise SystemExit
                        else:
                            # did we interrupt the process?
                            # we need TODO a logic for if it was interrupted. like check if it was so we can dock
                            # or for some serious reason.
                            # if self.client_plugin.interrupt == True:
                            self.get_logger().info("[client]:-what_happened?-. \n")
                            return

    # ................................
    # ............ MQTT ..............
    # ................................

    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT client disconnect callback."""
        if rc != 0:
            self.get_logger().info(f"MQTT client disconnected (rc: {rc}, {error_string(rc)}). Trying to reconnect.")
            while not self.mqtt_client.is_connected():
                try:
                    self.mqtt_client.reconnect()
                except OSError:
                    pass
        else:
            self.get_logger().info("[vda5050]-Disconnected from MQTT Broker! with client, userdata-: ["+str([client, userdata])+"]. \n")

    def vda5050_send_order(self, itinerary):
        """ mqtt pub send_order """
        nodes = []
        edges = []
        for i, point in enumerate(itinerary):
            node_id, x, y, z, w = point
            theta = self.client_plugin.quaternion_to_euler(0, 0, z, w)[-1]
            node = {
                "nodeId": node_id, # f"node{i+1}",
                "released": True,
                "sequenceId": i*2,
                "nodePosition": {
                    "x": x,
                    "y": y,
                    "theta": theta,
                    "mapId": "map"
                },
                "actions": [
                    {
                        "actionType": "beep",
                        "actionId": str(uuid.uuid4()),
                        "actionDescription": "reached_action",
                        "blockingType": "NONE",
                        "actionParameters": []
                    }
                ]
            }
            nodes.append(node)
            if i < len(itinerary) - 1:
                next_node_id = itinerary[i+1][0]  # Get the node_id of the next point in the itinerary
                edge = {
                    "edgeId": f"edge_{node_id}", # f"edge{i+1}",
                    "released": True,
                    "sequenceId": i*2+1,
                    "startNodeId": node_id,  # f"node{i+1}",
                    "endNodeId": next_node_id,  #  f"node{i+2}", Set endNodeId to the node_id of the next point
                    "actions": []
                }
                edges.append(edge)
        # rand = random.randint(1, 100)
        message = {
            "header_id": 1, # rand, # 1,
            "timestamp": datetime.datetime.now().isoformat(),
            "version": self.version,
            "manufacturer": self.client_plugin.manufacturer,
            "serial_number": self.client_plugin.serial_number,
            "order_id": str(uuid.uuid4()),
            "order_update_id": self.order_update, # rand - 1, # 0
            "zone_set_id": "",
            "nodes": nodes,
            "edges": edges
        }
        message_json = json.dumps(message)
        self.mqtt_client.publish(self.order_topic, message_json)
        self.get_logger().info("[vda5050]:- order published-. \n")

    def vda5050_cancel_order(self):
        """ mqtt pub cancel_order """
        message = {
            "version": self.version,
            "manufacturer": self.client_plugin.manufacturer,
            "serial_number": self.client_plugin.serial_number,
            "actions": [
                {
                    "actionType": "cancelOrder",
                    "actionId": str(uuid.uuid4()),
                    "blockingType": "NONE",
                    "actionParameters": []
                }
            ]
        }
        message_json = json.dumps(message)
        self.mqtt_client.publish(self.instant_action_topic, message_json)
        self.get_logger().info("[vda5050]:- cancel order published-. \n")

# ----------------------------------------------------------------------- #
#                            VDA5050 MSG                                  #
# ----------------------------------------------------------------------- #

    def vda5050_connector_drive(self, base): #, horizon
        """ vda5050 connector drive """

        # Add the last element of base and the first two elements of horizon to target_list
        target_list = []
        target_list.append(base[-1])
        target_list.append(base[-1])

        base_nodes = [] # convert to list of lists for order message
        for _, inspection_point in enumerate(target_list):
            base_nodes.append([inspection_point.header.frame_id,
                                inspection_point.pose.position.x,
                                inspection_point.pose.position.y,
                                inspection_point.pose.orientation.z,
                                inspection_point.pose.orientation.w])
            # self.get_logger().info("[vda5050]-curr base_node-:["+str(base_nodes[-1])+"]. \n")

        # self.get_logger().info("[vda5050]-::: connection state ::::-["+str(self.client_plugin.vda5050_connection_state)+"]. \n")
        # If this is the first call or every fourth call, update base_list

        if (self.base_nodes != base_nodes) and (self.client_plugin.vda5050_connection_state == "ONLINE"):
            self.base_nodes = base_nodes
            self.client_plugin.driving = None
            # Send the order --> itinerary [[node_id,x1,y1,z1,w1], ...]
            self.vda5050_send_order(self.base_nodes)
            self.order_update += 1
            time.sleep(0.07)

        # is task finished
        if (self.base_nodes == base_nodes) and \
            (self.client_plugin.driving is False) and \
                (self.client_plugin.completed_tracker == "COMPLETED"):
            self.client_plugin.completed_tracker = None
            self.get_logger().info("[vda5050]:-.-.-.-.-.-.-.-.hopefully reached.-.-.-.-.-.-.-. \n")
            time.sleep(0.07)
            return True
        else:
            return None

# ----------------------------------------------------------------------- #
#                FLEET CLIENT INT MSG CALLBACK | TRAFFIC                  #
# ----------------------------------------------------------------------- #

    def simple_fleet_int_cb(self, msg):
        """ simple_fleet_int_cb """
        if ',' not in msg.data:
            notification = msg.data
            self.flag = notification
            # pause.
            if notification == 'red':
                # interrupt whatever process is ongoing. stop the robot.
                self.cancel_navigation()
                self.get_logger().info("[vda5050]-::: red received ::::-. \n")
            # play.
            elif notification == 'green':
                self.base_nodes = []
                self.navigator_update = True
                self.client_plugin.interrupt = False
                self.get_logger().info("[vda5050]-::: green received ::::-. \n")
        else:
            self.get_logger().info("[vda5050]-::: special cases ::::- ["+str([msg.data])+"]. \n")
            # special cases:
            notification = msg.data.split(',')
            self.flag = notification[0]
            # interrupt whatever process is ongoing. stop the robot.
            self.cancel_navigation()
            # if its skip and red, wouldnt have mattered cos cancel nav is already done.
            # skip this current waypoint or go to next.
            # used in Transport task & charge task.
            if notification[1] == 'skip':
                self.count += 1 # go to next
                self.get_logger().info("[vda5050]-::: skip received ::::-. \n")
                # update navigator cb
                self.navigator_update = True
            if notification[0] == 'green':
                # [green, elevator_map_response]: for elevator cases where we have to switch map.
                if notification[1] == 'floor1_map':
                    self.client_plugin.load_map(self.fleet_floor1_map_yaml)
                    self.count += 1 # go to next
                    self.get_logger().info("[vda5050]-::: map change received ::::-. \n")
                elif notification[1] == 'base_map':
                    self.client_plugin.load_map(self.fleet_base_map_yaml)
                    self.count += 1 # go to next
                    self.get_logger().info("[vda5050]-::: map change received ::::-. \n")
                # [green, wait_time, waitpoint]
                elif len(notification) > 2:
                    # negotiation. go to waitpoint
                    self.negotiated_route = [float(notification[1]),str(notification[2])]
                    self.get_logger().info("[vda5050]-::: negotiation received ::::-. \n")
                # update navigator cb
                self.navigator_update = True

        self.get_logger().info("[vda5050]-::: obo baddest ::::- ["+str([msg.data])+"]. \n")

# ----------------------------------------------------------------------- #
#                          HELPER FUNCTIONS                               #
# ----------------------------------------------------------------------- #

    def find_coordinates_of_point(self, ids, points, itinerary):
        """ find coordinates of point """
        try:
            index = points.index(ids)
            # self.get_logger().info("[vda5050]-::: index ::::- ["+str([index])+"]. \n")
            coordinates = itinerary[index]
            # self.get_logger().info("[vda5050]-::: coord ::::- ["+str([coordinates])+"]. \n")
            alternate_route = PoseStamped()
            alternate_route.header.stamp = self.get_clock().now().to_msg()
            alternate_route.pose.position.x = float(coordinates[0])
            alternate_route.pose.position.y = float(coordinates[1])
            alternate_route.pose.orientation.z = float(coordinates[2])
            alternate_route.pose.orientation.w = float(coordinates[3])
            return alternate_route
        except ValueError:
            self.get_logger().info("[client]:- find_coordinates_of_point value error. \n")
            return None  # If the waitpoint is not found

    def cancel_navigation(self):
        """ cancel navigation """
        self.vda5050_cancel_order()
        self.client_plugin.stop()
        time.sleep(1.2)

    def dock_undock_io(self, x=0.0, y=0.0, z=0.0, w=0.0):
        """
            waits for io state for dock/undock actions
            returns true after action is completed
        """
        self.get_logger().info("[client]:-choice-["+str(self.notification_ext)+"]. \n")
        if self.notification_ext == 'undock_required':
            # call io_state first
            while not self.client_plugin.state_io(self.notification_ext) :
                # waste time here until the state_io(type) function returns true.
                time.sleep(0.05)
            # must always return true though. else just exit(1).
            status = self.client_plugin.undock(x, y, z, w)
        elif self.notification_ext == 'dock_required':
            # call io_state first
            while not self.client_plugin.state_io(self.notification_ext) :
                # waste time here until the state_io(type) function returns true.
                time.sleep(0.05)
            # perform the dock action
            status = self.client_plugin.dock()
        return status

    def read_yaml_file(self, file_path):
        " [coverage path] read cleaning itinerary from yaml "
        if not os.path.exists(file_path):
            print(f"File {file_path} does not exist. Exiting.")
            return None
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                return yaml.load(f, Loader=yaml.FullLoader)
        except FileNotFoundError:
            print(f"File {file_path} not found. Exiting.")
            return None

    def simple_fleet_ext_pub(self, notification):
        """ simple_fleet_ext_pub """
        self.msg_notice.data = notification
        self.simple_fleet_notice_pub.publish(self.msg_notice)
        # self.get_logger().info("[client] -extpub- : ["+str(self.msg_notice.data)+"]. \n")

    def last_checkpoint_publisher(self, last_checkpoints):
        """ publish the remaining yet to be visited targets or checkpoints """
        self.msg_lc.data = ','.join(last_checkpoints)
        self.last_checkpoint_pub.publish(self.msg_lc)
        # self.get_logger().info("[client] -lastpub- : ["+str(self.msg_lc.data)+"]. \n")

    def publish_waypoint_markers(self):
        """ show paths to be visited for the cleaning task cpp """
        marker_array = MarkerArray()
        for idx, goal_pose in enumerate(self.inspection_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = goal_pose.pose
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        # Hedefleri yayınla
        self.marker_publisher.publish(marker_array)

# ----------------------------------------------------------------------- #
#                                    MAIN                                 #
# ----------------------------------------------------------------------- #

def main(args=None):
    """ fleet client navigator node """
    rclpy.init(args=args)
    fleet_client_navigator = FleetClientNavigator()
    try:
        # executor = MultiThreadedExecutor()
        # executor.add_node(fleet_client_navigator)
        # executor.spin()

        # executor_thread = threading.Thread(target=executor.spin) # Start spinning the node
        # executor_thread.start()

        rclpy.spin(fleet_client_navigator)

    except SystemExit: # <- process the exception
        rclpy.logging.get_logger("fleet_client_navigator").info('Exited')
    fleet_client_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
