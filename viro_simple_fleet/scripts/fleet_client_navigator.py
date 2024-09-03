#! /usr/bin/env python3

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
from typing import List, Dict, Optional

# warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

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

class FleetClientNavigator(Node):
    """FleetClientNavigator handles robot navigation based on session information."""

    def __init__(self):
        super().__init__('fleet_client_navigator')

        self.package_name = 'viro_simple_fleet'
        self.package_share_directory = get_package_share_directory(self.package_name)

        self.group1 = ReentrantCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        self.inspection_pose = PoseStamped()
        self.msg_lc = String()
        self.msg_notice = String()

        self.last_checkpoint_pub = self.create_publisher(String, f'/{self.package_name}/last_checkpoints', 1)
        self.simple_fleet_notice_pub = self.create_publisher(String, f'/{self.package_name}/external_notice', 1)
        self.marker_publisher = self.create_publisher(MarkerArray, "/cpp_markers", 1)

        self.notification_stat_sub = self.create_subscription(
            String, f'/{self.package_name}/internal_notice', self.simple_fleet_int_cb,
            1, callback_group=self.group1)

        """ initialize_data """
        self.fleet_base_map_yaml = None
        self.fleet_floor1_map_yaml = None
        self.fleet_coverage_plan_yaml = None
        self.inspection_points: List[PoseStamped] = []
        self.notification_ext = None
        self.last_docked_station = None
        self.count = 0
        self.negotiated_route: List = []
        self.negotiation_stages: List[bool] = [False, False, False]
        self.checkpoints: List = []
        self.path_dict: List = []
        self.waitpoints: List = []
        self.wait_path_dict: List = []
        self.landmarks: List = []
        self.predecessor_landmark: List = []
        self.real_landmark: List = []
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

        self.startup_init = False
        self.timer = self.create_timer(1, self.startup_cb, callback_group=self.group1)

# ----------------------------------------------------------------------- #
#                FLEET CLIENT INT MSG CALLBACK | TRAFFIC                  #
# ----------------------------------------------------------------------- #

    def startup_cb(self):
        """Callback for startup timer."""
        if not self.startup_init:
            self.load_session_info(Path(self.package_share_directory, "config/session_info.yaml"))
        gc.collect()

    def load_session_info(self, yaml_file_path: str):
        """Load configuration and session information from a YAML file."""
        try:
            with open(yaml_file_path, encoding='utf-8') as yaml_file:
                agv_dict = yaml.safe_load(yaml_file)

                self.update_data_from_dict(agv_dict)
                self.setup_mqtt_client(agv_dict)
                self.setup_landmarks()
                self.startup_navigation(agv_dict)

        except FileNotFoundError as e:
            print(f"Waiting for session yaml... ({e})")
            time.sleep(0.07)

    def setup_mqtt_client(self, agv_dict: Dict):
        """ setup_mqtt_client """
        if self.client_plugin.use_vda5050:
            self.mqtt_connect(str(agv_dict.get("mqtt_address", "")),
                              int(agv_dict.get("mqtt_port", 1883)))

    def update_data_from_dict(self, agv_dict: Dict):
        """Update internal data from the configuration dictionary."""
        # read session dictionary and obtain session constants.
        self.path_dict = ast.literal_eval(str(agv_dict.get("agv_itinerary", [])))
        self.checkpoints = ast.literal_eval(str(agv_dict.get("checkpoints", [])))
        self.landmarks = ast.literal_eval(str(agv_dict.get("landmarks", [])))
        self.wait_path_dict = ast.literal_eval(str(agv_dict.get("wait_itinerary", [])))
        self.waitpoints = ast.literal_eval(str(agv_dict.get("waitpoints", [])))
        self.fleet_base_map_yaml = str(agv_dict.get("fleet_base_map_yaml", ""))
        self.fleet_floor1_map_yaml = str(agv_dict.get("fleet_floor1_map_yaml", ""))
        self.fleet_coverage_plan_yaml = str(agv_dict.get("fleet_coverage_plan_yaml", ""))
        self.client_plugin.use_vda5050 = bool(str(agv_dict.get("use_vda5050", "False")))
        # mqtt stuff?
        self.version = str(agv_dict["version"])
        self.client_plugin.manufacturer = str(agv_dict["manufacturer"])
        self.client_plugin.serial_number = str(agv_dict["serial_number"])
        self.client_plugin.connection_topic = str(agv_dict["connection_topic"])
        self.client_plugin.state_topic = str(agv_dict["state_topic"])
        self.order_topic = str(agv_dict["order_topic"])
        self.instant_action_topic = str(agv_dict["instant_action_topic"])

    def setup_landmarks(self):
        """Initialize landmark information. what landmarks are available? what kind of task is this? """
        if self.landmarks and self.landmarks[1] != "clean":
            self.predecessor_landmark = [element.split('_')[0] for element in self.landmarks[2:]]
            self.real_landmark = [element.split('_')[1] for element in self.landmarks[2:]]

    def startup_navigation(self, agv_dict: Dict):
        """Check file timestamp and perform startup actions."""

        update_time = float(agv_dict.get("update_time", 0))
        startup_map = str(agv_dict.get("startup_map", ""))
        curr_dock = ast.literal_eval(str(agv_dict.get("curr_dock", [])))
        initial_pose = ast.literal_eval(str(agv_dict.get("initial_pose", {})))

        if (time.time_ns() - update_time) * 1e-9 < 240 and self.path_dict:
            self.startup_init = True
            self.navigator_update = True
            self.timer.cancel()

            self.client_plugin.on_startup(initial_pose)
            self.get_logger().info("[client] onstartup called. \n")

            if startup_map == 'floor1_map':
                self.client_plugin.load_map(self.fleet_floor1_map_yaml)
            elif startup_map == 'base_map':
                self.client_plugin.load_map(self.fleet_base_map_yaml)

            self.route_pose_callback() # load the path or necessary goal messages

            if str(curr_dock[0]) == 'x':
                self.inspect_route()
            else: # i was at the dock and i know my address. agv_status[1:] => [x,y,z,w]
                self.notification_ext = 'undock_required'
                self.simple_fleet_ext_pub(self.notification_ext)
                # undock [x,y,z,w]
                status = self.dock_undock_io(*map(float, curr_dock))
                if status:
                    self.notification_ext = 'undock_completed'
                    self.simple_fleet_ext_pub(self.notification_ext)
                    self.count += 1
                    self.inspect_route()

# ----------------------------------------------------------------------- #
#                            MAIN CALLBACK                                #
# ----------------------------------------------------------------------- #

    def route_pose_callback(self):
        """Convert path dictionary to ROS message type."""
        self.inspection_points = []
        self.inspection_pose.header.frame_id = "map"
        self.inspection_pose.header.stamp = self.get_clock().now().to_msg()

        if self.landmarks[1] == "clean":
            self.process_cleaning_task()
        else:
            self.process_navigation_task()

    def process_cleaning_task(self):
        """Process cleaning task waypoints."""
        coverage_path_dict = self.read_yaml_file(self.fleet_coverage_plan_yaml)
        if coverage_path_dict is None:
            self.get_logger().error('[client-cov_path_plan] could not find cleaning coverage path yaml.')
            return

        for i, pose_dict in coverage_path_dict.items():
            print(f"Processing coverage pose {i}...")
            if isinstance(pose_dict, dict) and 'position' in pose_dict:
                # """Update the inspection pose from a pose dictionary."""
                self.inspection_pose.pose.position.x = pose_dict["position"]["x"]
                self.inspection_pose.pose.position.y = pose_dict["position"]["y"]
                self.inspection_pose.pose.orientation.z = pose_dict["orientation"]["z"]
                self.inspection_pose.pose.orientation.w = pose_dict["orientation"]["w"]
                # coverage_path_poses: append to goal poses.
                self.inspection_points.append(deepcopy(self.inspection_pose))
        self.publish_waypoint_markers()
        self.get_logger().info("[client-cov_path_plan] fetched path dict. \n")

    def read_yaml_file(self, file_path):
        " [coverage path] read cleaning itinerary from yaml "
        if not os.path.exists(file_path):
            print(f"[client-cov_path_plan] File {file_path} does not exist. Exiting.")
            return None
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                return yaml.load(f, Loader=yaml.FullLoader)
        except FileNotFoundError:
            print(f"[client-cov_path_plan] File {file_path} not found. Exiting.")
            return None

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
        self.marker_publisher.publish(marker_array)

    def process_navigation_task(self):
        """Process navigation task waypoints."""
        for i, waypoint in enumerate(self.path_dict):
            if self.client_plugin.use_vda5050:
                self.inspection_pose.header.frame_id = self.checkpoints[i]
            # """Update the inspection pose from a waypoint."""
            self.inspection_pose.pose.position.x = float(waypoint[0])
            self.inspection_pose.pose.position.y = float(waypoint[1])
            self.inspection_pose.pose.orientation.z = float(waypoint[2])
            self.inspection_pose.pose.orientation.w = float(waypoint[3])
            self.inspection_points.append(deepcopy(self.inspection_pose))
        self.get_logger().info("[client-navigator] path_created. \n")

# ----------------------------------------------------------------------- #
#                FLEET CLIENT INT MSG CALLBACK | TRAFFIC                  #
# ----------------------------------------------------------------------- #

    def inspect_route(self):
        """Perform actual navigation here with Nav2 simple API calls."""
        while True:
            rclpy.spin_once(self, timeout_sec=1.07)

            if self.navigator_update:
                self.navigator_update = False
                self.i = 0

                while self.count < len(self.inspection_points):

                    rclpy.spin_once(self, timeout_sec=1.07)

                    self.i += 1

                    if self.i % 10 == 0:

                        next_stop_coordinate = [self.inspection_points[self.count].pose.position.x,
                                                self.inspection_points[self.count].pose.position.y,
                                                self.inspection_points[self.count].pose.orientation.z,
                                                self.inspection_points[self.count].pose.orientation.w]

                        self.handle_publish_checkpoint()

                        if self.flag == "red":
                            continue

                        if self.negotiated_route:
                            result = self.handle_negotiation()
                        else:
                            result = self.handle_navigation(next_stop_coordinate)

                        if result is None:
                            continue

                        if result:
                            if self.client_plugin.recovery_retry_counter == 0:

                                if self.negotiated_route:
                                    self.complete_negotiation()
                                    continue

                                # """Process successful navigation, handling tasks such as docking."""
                                if self.landmarks[1] != "clean":

                                    for landmark in self.real_landmark:

                                        if self.should_dock(landmark):

                                            self.notification_ext = 'dock_required'
                                            self.simple_fleet_ext_pub(self.notification_ext)

                                            if self.dock_undock_io():

                                                self.notification_ext = 'dock_completed'
                                                self.simple_fleet_ext_pub(self.notification_ext)

                                                self.last_docked_station = self.checkpoints[self.count:][0]

                                                if self.landmarks[1] in ["charge", "move"]:
                                                    self.cancel_navigation()
                                                    return

                                                if self.landmarks[1] == "loop":

                                                    if self.is_first_pickup():
                                                        self.trim_paths_before_pickup()
                                                        self.count = 0
                                                    elif self.is_last_pickup():
                                                        self.count = 0
                                                    elif self.is_drop_location():
                                                        pass

                                                    self.perform_undock(next_stop_coordinate)
                                                    continue

                                                if self.landmarks[1] == "transport":

                                                    if self.is_pickup_or_drop_location():
                                                        self.perform_undock(next_stop_coordinate)
                                                        continue

                                                    elif self.is_home_dock_station():
                                                        self.cancel_navigation()
                                                        return

                                        else:
                                            if self.notification_ext == 'undock_required':
                                                self.perform_undock(next_stop_coordinate)
                                                self.count += 1 # not sure about this!!
                                                continue

                                elif self.has_more_inspection_points():
                                    if self.is_elevator():
                                        self.request_elevator_switch()
                                        return
                                    else: # go to the next target
                                        self.notification_ext = 'waypoint'
                                        while not self.client_plugin.state_io(self.notification_ext):
                                            time.sleep(0.07)
                                        self.count += 1 # move on to the next +1:
                                        continue

                                else:
                                    self.cancel_navigation()
                                    return

                            elif self.client_plugin.recovery_retry_counter != 0 and \
                                self.client_plugin.recovery_retry_counter <= self.client_plugin.recovery_retry_max:
                                # we failed the actual task but succeeded in the recovery.
                                # we need to go to same goal again.
                                continue
                            else:
                                if self.client_plugin.use_vda5050:
                                    self.mqtt_client.disconnect()
                                self.client_plugin.on_exit()
                                self.notification_ext = 'recovery_required'
                                self.simple_fleet_ext_pub(self.notification_ext)
                                raise SystemExit

                        else:
                            self.get_logger().info("[client]: what_happened? \n") # self.client_plugin.interrupt is True
                            return

    def has_more_inspection_points(self):
        """ has_more_inspection_points """
        return len(self.inspection_points[self.count:]) > 1

    def is_elevator(self):
        """ is_elevator """
        could_be_an_elevator = self.checkpoints[self.count:][0]
        return could_be_an_elevator[0] == 'E'

    def request_elevator_switch(self):
        """ request_elevator_switch """
        self.notification_ext = 'elevator_required'
        self.simple_fleet_ext_pub(self.notification_ext)
        while not self.client_plugin.state_io(self.notification_ext):
            time.sleep(0.07)
        self.cancel_navigation()

    def cancel_navigation(self):
        """ cancel navigation """
        self.mqtt_cancel_vda5050_order()
        self.client_plugin.stop()
        time.sleep(1.07)
        self.get_logger().info("[client] cancelling completed. \n")

    def is_pickup_or_drop_location(self):
        """ is_pickup_or_drop_location """
        return (self.checkpoints[self.count:][0] == self.real_landmark[0] or \
                self.checkpoints[self.count:][0] == self.real_landmark[1]) and \
               len(self.checkpoints[self.count:]) > 1

    def is_home_dock_station(self):
        """ is_home_dock_station """
        return self.checkpoints[self.count:][0] in self.real_landmark[2:]

    def perform_undock(self, undock_coord):
        """ perform_undock """
        self.notification_ext = 'undock_required'
        self.simple_fleet_ext_pub(self.notification_ext)
        if self.dock_undock_io(undock_coord[0],
                               undock_coord[1],
                               undock_coord[2],
                               undock_coord[3]):
            self.notification_ext = 'undock_completed'
            self.simple_fleet_ext_pub(self.notification_ext)
        else:
            print("undock failed.")

    def should_dock(self, landmark):
        """ should_dock """
        return (landmark == self.checkpoints[self.count:][0]) and \
               (self.last_docked_station != self.checkpoints[self.count:][0])

    def is_first_pickup(self):
        """ is_first_pickup """
        return self.checkpoints[self.count:][0] == self.real_landmark[0] and len(self.checkpoints[self.count:]) > 1

    def is_last_pickup(self):
        """ is_last_pickup """
        return self.checkpoints[self.count:][0] == self.real_landmark[0] and len(self.checkpoints[self.count:]) == 1

    def is_drop_location(self):
        """ is_drop_location """
        return self.checkpoints[self.count:][0] == self.real_landmark[1] and len(self.checkpoints[self.count:]) > 1

    def trim_paths_before_pickup(self):
        """ trim_paths_before_pickup """
        if self.checkpoints.index(self.real_landmark[0]) > 0 and self.landmarks[0] == "high":
            while self.checkpoints.index(self.real_landmark[0]) > 0:
                self.checkpoints.pop(0)
                self.path_dict.pop(0)
                self.inspection_points.pop(0)
        else:
            print("There are no elements before the first occurrence of 'self.landmarks[2]'")

    def handle_publish_checkpoint(self):
        """Handle actions related to checkpoints."""
        if self.landmarks[1] != "clean":
            self.last_checkpoint_publisher(self.checkpoints[self.count:])
        else:
            self.last_checkpoint_publisher(['clean', str(self.count), '/', str(len(self.inspection_points))])

    def last_checkpoint_publisher(self, last_checkpoints):
        """ publish the remaining yet to be visited targets or checkpoints """
        self.msg_lc.data = ','.join(last_checkpoints)
        self.last_checkpoint_pub.publish(self.msg_lc)

# ----------------------------------------------------------------------- #
#                          HELPER FUNCTIONS                               #
# ----------------------------------------------------------------------- #

    def simple_fleet_ext_pub(self, notification):
        """ simple_fleet_ext_pub """
        self.msg_notice.data = notification
        self.simple_fleet_notice_pub.publish(self.msg_notice)

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x ** 2 + y ** 2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y ** 2 + z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
# ----------------------------------------------------------------------- #
#                MQTT VDA5050                                             #
# ----------------------------------------------------------------------- #

    def mqtt_connect(self, mqtt_address, mqtt_port):
        """ connect """
        # Define the MQTT client
        self.mqtt_client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1)
        # Set the callback function
        self.mqtt_client.on_disconnect = self.mqtt_disconnect
        # Connect to the MQTT broker
        self.mqtt_client.connect(host=str(mqtt_address), port=int(mqtt_port)) # keep alive for 60secs
        self.mqtt_client.loop_start()
        print("[client-mqtt] connection done. \n")

    def mqtt_disconnect(self, client, userdata, rc):
        """MQTT client disconnect callback."""
        if rc != 0:
            print(f"[client-mqtt] disconnected (rc: {rc}, {error_string(rc)}). Trying to reconnect.")
            while not self.mqtt_client.is_connected():
                try:
                    self.mqtt_client.reconnect()
                except OSError:
                    pass
        else:
            print("[client-mqtt] Disconnected from MQTT Broker! with client, userdata: ["+str([client, userdata])+"]. \n")

    def mqtt_send_vda5050_order(self, itinerary):
        """ mqtt pub send_order """
        nodes = []
        edges = []
        for i, point in enumerate(itinerary):
            node_id, x, y, z, w = point
            theta = self.quaternion_to_euler(0, 0, z, w)[-1]
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

        message = {
            "header_id": 1, # rand = random.randint(1, 100)
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
        print("[client-mqtt] order published. \n")

    def mqtt_cancel_vda5050_order(self):
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
        print("[client-mqtt] cancel order published. \n")


# ----------------------------------------------------------------------- #
#                FLEET CLIENT INT MSG CALLBACK | TRAFFIC                  #
# ----------------------------------------------------------------------- #

    def handle_negotiation(self):
        """Handle negotiation stages."""
        self.get_logger().info("[client] negotiation drive... \n")
        self.notification_ext = 'negotiation_required'
        self.simple_fleet_ext_pub(self.notification_ext)
        if self.count != 0:
            if self.client_plugin.use_vda5050 is True:
                if self.negotiation_stages[0] is False:
                    # go back to previous node:
                    prev_stop_id = self.checkpoints[self.count-1]
                    prev_stop_coordinate = self.find_coordinates_of_point(prev_stop_id, self.checkpoints, self.path_dict)
                    prev_stop_coordinate.header.frame_id = str(prev_stop_id)
                    result = self.vda5050_connector_drive([prev_stop_coordinate])
                    self.get_logger().info("[client] Negotiation stage 1. ")
                elif self.negotiation_stages[1] is False:
                    # then go to wait node and sleep for wait_time seconds:
                    wait_stop_id = self.negotiated_route[1]
                    wait_stop_coordinate = self.find_coordinates_of_point(wait_stop_id, self.waitpoints, self.wait_path_dict)
                    wait_stop_coordinate.header.frame_id = "wait_"+str(wait_stop_id)
                    result = self.vda5050_connector_drive([wait_stop_coordinate])
                    self.get_logger().info("[client] Negotiation stage 2. ")
                elif self.negotiation_stages[2] is False:
                    # then go back to previous node again before continuing to path_dict:
                    prev_stop_id = self.checkpoints[self.count-1]
                    prev_stop_coordinate = self.find_coordinates_of_point(prev_stop_id, self.checkpoints, self.path_dict)
                    prev_stop_coordinate.header.frame_id = str(prev_stop_id)
                    result = self.vda5050_connector_drive([prev_stop_coordinate])
                    self.get_logger().info("[client] Negotiation stage 3. ")
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
        return result

    def complete_negotiation(self):
        """ Handles the completion of negotiation stages """
        if len(self.negotiated_route) != 0:
            if not self.negotiation_stages[0]:
                self.negotiation_stages[0] = True
            elif not self.negotiation_stages[1]:
                self.negotiation_stages[1] = True
                wait_time = self.negotiated_route[0]
                time.sleep(wait_time)
            elif not self.negotiation_stages[2]:
                self.negotiation_stages[2] = True
                self.negotiated_route = []
                self.negotiation_stages = [False, False, False]
                self.notification_ext = 'negotiation_completed'
                self.simple_fleet_ext_pub(self.notification_ext)

    def find_coordinates_of_point(self, ids, points, itinerary):
        """ find coordinates of point """
        try:
            index = points.index(ids)
            coordinates = itinerary[index]
            alternate_route = PoseStamped()
            alternate_route.header.stamp = self.get_clock().now().to_msg()
            alternate_route.pose.position.x = float(coordinates[0])
            alternate_route.pose.position.y = float(coordinates[1])
            alternate_route.pose.orientation.z = float(coordinates[2])
            alternate_route.pose.orientation.w = float(coordinates[3])
            return alternate_route
        except ValueError:
            self.get_logger().info("[client]:- find_coordinates_of_point value error. \n")
            return None

# ----------------------------------------------------------------------- #
#                FLEET CLIENT INT MSG CALLBACK | TRAFFIC                  #
# ----------------------------------------------------------------------- #

    def dock_undock_io(self, x:float=0.0, y:float=0.0, z:float=0.0, w:float=0.0) -> bool:
        """
            waits for io state for dock/undock actions
            returns true after action is completed
        """
        self.get_logger().info("[client]: ["+str(self.notification_ext)+"]. \n")
        if self.notification_ext == 'undock_required':
            # call io_state first
            while not self.client_plugin.state_io(self.notification_ext) :
                # waste time here until the state_io(type) function returns true.
                time.sleep(0.07)
            # must always return true though. else just exit(1).
            status = self.client_plugin.undock(x, y, z, w)
        elif self.notification_ext == 'dock_required':
            # call io_state first
            while not self.client_plugin.state_io(self.notification_ext) :
                # waste time here until the state_io(type) function returns true.
                time.sleep(0.07)
            # perform the dock action
            status = self.client_plugin.dock()
        return status

# ----------------------------------------------------------------------- #
#                      VDA5050 NAV DRIVE                                  #
# ----------------------------------------------------------------------- #

    def handle_navigation(self, next_stop_coordinate):
        """Drive the robot to the goal/target."""
        if self.client_plugin.use_vda5050:
            result = self.vda5050_connector_drive([self.inspection_points[self.count]])
        else:
            result = self.client_plugin.drive(next_stop_coordinate)
        return result

    def vda5050_connector_drive(self, base):
        """Handle the driving connection for VDA5050 protocol."""
        target_list = self._generate_target_list(base)
        base_nodes = self._convert_to_base_nodes(target_list)

        if self._should_update_base_nodes(base_nodes):
            self._update_base_nodes(base_nodes)

        if self._is_goal_reached(base_nodes):
            self.get_logger().info("[vda5050] -.-.-.-.reached.-.-.-. \n")
            return True

        return None

    def _generate_target_list(self, base):
        """Generate a target list from base."""
        target_list = [base[-1], base[-1]]
        return target_list

    def _convert_to_base_nodes(self, target_list):
        """Convert target list to base nodes."""
        return [[point.header.frame_id,
                 point.pose.position.x,
                 point.pose.position.y,
                 point.pose.orientation.z,
                 point.pose.orientation.w] for point in target_list]

    def _should_update_base_nodes(self, base_nodes):
        """Determine whether base nodes should be updated."""
        return self.base_nodes != base_nodes and self.client_plugin.vda5050_connection_state == "ONLINE"

    def _update_base_nodes(self, base_nodes):
        """Update base nodes and send order."""
        self.base_nodes = base_nodes
        self.client_plugin.driving = None
        self.mqtt_send_vda5050_order(self.base_nodes) # Send the order --> itinerary [[node_id,x1,y1,z1,w1], ...]
        self.order_update += 1
        time.sleep(0.07)

    def _is_goal_reached(self, base_nodes):
        """Check if the task is finished."""
        return (self.base_nodes == base_nodes and
                self.client_plugin.driving is False and
                self.client_plugin.completed_tracker == "COMPLETED")

# ----------------------------------------------------------------------- #
#                FLEET CLIENT INT MSG CALLBACK | TRAFFIC                  #
# ----------------------------------------------------------------------- #

    def simple_fleet_int_cb(self, msg):
        """Internal callback for fleet notifications."""
        if ',' not in msg.data:
            self._handle_simple_notification(msg.data)
        else:
            self._handle_special_cases(msg.data)
        self.get_logger().info(f"[vda5050] [{msg.data}].")

    def _handle_simple_notification(self, notification):
        """Handle simple notifications."""
        self.flag = notification
        if notification == 'red':
            self._pause_robot()
        elif notification == 'green':
            self._resume_robot()

    def _pause_robot(self):
        """Pause the robot."""
        self.cancel_navigation()
        self.get_logger().info("[vda5050] red received. ")

    def _resume_robot(self):
        """Resume robot operations."""
        self.base_nodes = []
        self.navigator_update = True
        self.client_plugin.interrupt = False
        self.get_logger().info("[vda5050] green received. ")

    def _handle_special_cases(self, notification):
        """Handle special cases in fleet notifications."""
        notification_list = notification.split(',')
        self.flag = notification_list[0]
        self._pause_robot()

        if notification_list[1] == 'skip':
            self._skip_current_waypoint()
        elif notification_list[0] == 'green':
            self._handle_green_special_cases(notification_list)

    def _skip_current_waypoint(self):
        """Skip the current waypoint."""
        self.count += 1
        self.navigator_update = True
        self.get_logger().info("[vda5050] skip received. ")

    def _handle_green_special_cases(self, notification_list):
        """Handle special cases when the notification is 'green'."""
        if notification_list[1] in ['floor1_map', 'base_map']:
            self._switch_map(notification_list[1])
        elif len(notification_list) > 2:
            self._handle_negotiation(notification_list[1], notification_list[2])

    def _switch_map(self, map_type):
        """Switch to a different map."""
        if map_type == 'floor1_map':
            self.client_plugin.load_map(self.fleet_floor1_map_yaml)
        elif map_type == 'base_map':
            self.client_plugin.load_map(self.fleet_base_map_yaml)
        self.count += 1
        self.navigator_update = True
        self.get_logger().info("[vda5050] map change received. ")

    def _handle_negotiation(self, wait_time, waitpoint):
        """Handle negotiation scenarios."""
        self.negotiated_route = [float(wait_time), str(waitpoint)]
        self.navigator_update = True
        self.get_logger().info("[vda5050] negotiation received. ")

# ----------------------------------------------------------------------- #
#                                    MAIN                                 #
# ----------------------------------------------------------------------- #

def main(args=None):
    """ fleet client navigator node """
    rclpy.init(args=args)
    fleet_client_navigator = FleetClientNavigator()
    try:
        rclpy.spin(fleet_client_navigator)
    except SystemExit:
        rclpy.logging.get_logger("fleet_client_navigator").info('Exited')
    fleet_client_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
