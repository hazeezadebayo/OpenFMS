#!/usr/bin/env python3

# important!!!
# cd colcon_ws/src/viro_core/viro_core/scripts -----$ chmod +x fleet_client_monitor.py

import os, xml, xml.dom.minidom, rclpy, warnings
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, TransformStamped, PoseStamped # ,Point ,Pose,Quaternion,Vector3,
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState, LaserScan
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.msg import ParticleCloud, Costmap
from tf2_msgs.msg import TFMessage
from collections import deque
import numpy as np
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

class FleetClientMonitor(Node):
    """ FleetClientMonitor """
    def __init__(self):
        super().__init__('fleet_client_monitor')

        package_name = 'viro_simple_fleet'

        self.motion_return = "no_motion"
        self.scan_return = "no_scan"
        self.odom_return = "no_odom"
        self.map_return = "no_map"
        self.robot_description_return = "no_robot_description"
        self.amcl_return = "no_amcl"

        # overwrite related shi!
        # this should be ontop of your hardware interface though!
        self.warning_stop = 0
        self.warning_slow = 0
        self.no_warning = 0

        self.declare_parameter("laser_topic", '/scan')
        self.declare_parameter("robot_description_topic", '/robot_description')
        self.declare_parameter("map_topic", '/map')
        self.declare_parameter("odom_topic", '/odom')
        self.declare_parameter("amcl_topic", '/amcl_pose')
        self.declare_parameter("base_frame_name", 'base_link')
        self.declare_parameter("similarity_threshold", 0.1) # Threshold for determining if the robot is moving 0.005
        self.declare_parameter("startup_status_wait_sec", 60.0)

        laser_topic = self.get_parameter("laser_topic").get_parameter_value().string_value
        robot_description_topic = self.get_parameter("robot_description_topic").get_parameter_value().string_value
        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        amcl_topic = self.get_parameter("amcl_topic").get_parameter_value().string_value
        base_frame_name = self.get_parameter("base_frame_name").get_parameter_value().string_value
        self.similarity_threshold = self.get_parameter("similarity_threshold").get_parameter_value().double_value
        self.startup_status_wait_sec = self.get_parameter("startup_status_wait_sec").get_parameter_value().double_value

        self.robot_description_sub = self.create_subscription(String, robot_description_topic, self.rd_callback, 1)
        self.map_sub = self.create_subscription(OccupancyGrid,map_topic,self.map_callback,1)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, amcl_topic, self.amcl_callback, 1)
        self.scan_sub = self.create_subscription(LaserScan, laser_topic, self.scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 1)

        self.transforms_return = ['failed', 'failed', 'failed']
        self.target_frame = [base_frame_name, 'base_footprint', 'odom']

        # Store up to 10 recent scans
        self.scan_history = deque(maxlen=10)

        if 'ROS_DOMAIN_ID' in os.environ:
            print('ROS_DOMAIN_ID', os.environ['ROS_DOMAIN_ID'])

        self.start_time = self.get_clock().now()

        self.msg_notice = String()
        self.simple_fleet_notice_pub = self.create_publisher(String, '/'+package_name+'/external_notice', 1)

        self.target = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Create a timer to check motion every 4.5 seconds
        self.timer = self.create_timer(4.5, self.check_robot_status)

    def simple_fleet_ext_pub(self, notification):
        """ no motion | motion  """
        self.msg_notice.data = notification
        self.simple_fleet_notice_pub.publish(self.msg_notice)

    def check_robot_status(self):
        """ robot tf listener """
        if self.target < len(self.target_frame):
            if self.transforms_return[self.target] == 'failed':
                trans = None
                from_frame_rel = self.target_frame[self.target]
                to_frame_rel = 'map'
                try:
                    now = rclpy.time.Time()
                    trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
                    self.transforms_return[self.target] = "success" if isinstance(trans.transform.translation.x, float) and isinstance(trans.transform.translation.y, float) else "failed";
                except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as ex:
                    self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')

            if self.transforms_return[self.target] == "success" :
                print("Transformation map - ",self.target_frame[self.target], ": ",self.transforms_return[self.target])
                self.target += 1

        if self.target == len(self.target_frame):
            # robot_description_ = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
            # self.path is None or self.global_plan is None or self.local_plan is None or self.costmap is None or
            if self.get_clock().now().to_msg().sec - self.start_time.to_msg().sec > self.startup_status_wait_sec:
                msg_string = ""
                if self.scan_return == "no_scan":
                    msg_string += self.scan_return
                if self.odom_return == "no_odom":
                    msg_string += self.odom_return
                # if self.map_return == "no_map":
                #     msg_string += self.map_return
                if self.amcl_return == "no_amcl":
                    msg_string += self.amcl_return
                # if self.robot_description_return == "no_robot_description":
                #     msg_string += self.robot_description_return
                if "failed" in self.transforms_return:
                    msg_string += "no_tf"

                if msg_string == "":
                    msg_string = "startup_success"
                else:
                    self.get_logger().info(" fleet monitor node report: "+msg_string+". \n")
                    msg_string = "startup_failed"

                self.simple_fleet_ext_pub(msg_string)
                # there is no point killing this node since we are now checking for motion or no motion
                # raise SystemExit

        if self.get_clock().now().to_msg().sec - self.start_time.to_msg().sec > self.startup_status_wait_sec:
            self.check_motion()
            self.start_time = self.get_clock().now()

    def amcl_callback(self, msg):
        """ amcl listener """
        if self.amcl_return != "amcl_success":
            temp_1 = "True" if isinstance(msg.pose.pose.position.x, float) and isinstance(msg.pose.pose.position.y, float) else "False"
            temp_2 = "True" if isinstance(msg.pose.pose.orientation.z, float) and isinstance(msg.pose.pose.orientation.w, float) else "False"
            if (temp_1 == "True") and (temp_2 == "True"):
                self.amcl_return = "amcl_success"
                self.destroy_subscription(self.amcl_sub)
            else:
                self.amcl_return = "amcl_failed"
            print("self.amcl_return: ", self.amcl_return)

    def odom_callback(self, msg):
        """ odom listener """
        if self.odom_return != "odom_success":
            temp_1 = "True" if isinstance(msg.pose.pose.position.x, float) and isinstance(msg.pose.pose.position.y, float) else "False"
            temp_2 = "True" if isinstance(msg.pose.pose.orientation.z, float) and isinstance(msg.pose.pose.orientation.w, float) else "False"
            if temp_1 == "True" and temp_2 == "True":
                self.odom_return = "odom_success"
                self.destroy_subscription(self.odom_sub)
            else:
                self.odom_return = "odom_failed"
            print("self.odom_return: ", self.odom_return)

    def scan_callback(self, msg):
        """ laser scan listener """
        # Append the latest scan to the history
        self.scan_history.append(msg.ranges)
        # msgs_rx = []
        # lambda msg: msgs_rx.append(msg)
        # for msg in msgs_rx:
        if self.scan_return != "scan_success":
            temp_1 = "True" if len(msg.ranges) > 0 else "False"
            temp_2 = "True" if len(msg.intensities) > 0 else "False"
            if temp_1 == "True" and temp_2 == "True":
                self.scan_return = "scan_success"
                # self.destroy_subscription(self.scan_sub)
            else:
                self.scan_return = "scan_failed"
            print("self.scan_return: ", self.scan_return)

    def check_warning(self, msg):
        """ [overwrite!]
            lidar fields warning
            this should overwrite the decisions of the fleet manager
            # reduce robot speed
            # stop the motion entirely
            # special alarms/sirene/buzzer trigger.
        """
        # you have to play around with the range_of_interest though
        # it is relative to the placement of your lidar on the robot and
        # the measurement units etc.
        range_of_interest =  list(msg)[int(len(msg)*0.15):int(len(msg)*0.88)]
        if (min(range_of_interest) < 0.75):
            if self.warning_stop == 0:
                # print("warning: stop | sirene")
                self.warning_stop = 1
                self.warning_slow = 0
                self.no_warning = 0
                self.simple_fleet_ext_pub('warning_stop')
        # here you must force the hardware interface to reduce the speed of the agv
        # like publish a signal or write to serial in a way that halves the speed or something.
        elif min(range_of_interest) < 1.5:
            if self.warning_slow == 0:
                # print("warning: slow | sirene")
                self.warning_stop = 0
                self.warning_slow = 1
                self.no_warning = 0
                self.simple_fleet_ext_pub('warning_slow')
        # this should take it back to its normal speed etc.
        elif min(range_of_interest) >= 1.5:
            if self.no_warning == 0:
                # self.pub_field_stat("no_warning")
                self.warning_stop = 0
                self.warning_slow = 0
                self.no_warning = 1
                self.simple_fleet_ext_pub('no_warning')

    def check_motion(self):
        """ is the robot in motion? """
        # check for warning messages:
        self.check_warning(self.scan_history[-1])
        # Check if there are at least two scans in the history
        if len(self.scan_history) >= 2:

            # Calculate the average scan
            averaged_scan = np.mean(self.scan_history, axis=0)
            # print("averaged_scan: ", averaged_scan)

            # Calculate the absolute differences between the current scan and the average scan
            differences = np.abs(averaged_scan - self.scan_history[-1])
            # print("differences: ", differences)

            # Calculate the mean difference
            similarity = np.mean(differences)
            print("similarity: ", similarity)

            # Motion or no motion?
            if similarity < self.similarity_threshold:
                if self.motion_return != "no_motion":
                    self.motion_return = "no_motion"
                    self.simple_fleet_ext_pub(self.motion_return)
                # self.get_logger().info("Robot is not moving! \n")
                # it depends, but you can triggery recovery actions here. not sure what you want though eylul.
            else:
                if self.motion_return != "motion":
                    self.motion_return = "motion"
                    self.simple_fleet_ext_pub(self.motion_return)
                # self.get_logger().info("Robot seems to be moving. \n")

    def map_callback(self, msg):
        """ map listener"""
        if self.map_return != "map_success":
            temp_1 = "True" if msg.info.width > 1 else "False"
            temp_2 = "True" if msg.info.height > 1 else "False"
            if temp_1 == "True" and temp_2 == "True":
                self.map_return = "map_success"
                self.destroy_subscription(self.map_sub)
            else:
                self.map_return = "no_map"
            print("self.map_callback: ", self.map_return)

    def rd_callback(self, msg):
        """ robot descriptiion listener """
        if self.robot_description_return != "robot_description_success":
            # print("robot_description")
            try:
                robot = xml.dom.minidom.parseString(msg.data)
                self.robot_description_return = "robot_description_success"
                self.destroy_subscription(self.robot_description_sub)
            except xml.parsers.expat.ExpatError:
                print('Invalid robot_description given, ignoring')
                self.robot_description_return = "no_robot_description"
            print("self.robot_description_return: ", self.robot_description_return)

# ----------------------------------------------------------------------- #
#                                    MAIN                                 #
# ----------------------------------------------------------------------- #

def main(args=None):
    """ health_checker node """
    rclpy.init(args=args)
    fleet_client_monitor = FleetClientMonitor()
    try:
        rclpy.spin(fleet_client_monitor)
    except SystemExit: # <- process the exception
        rclpy.logging.get_logger("fleet_client_monitor").info('Exited')
    fleet_client_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
