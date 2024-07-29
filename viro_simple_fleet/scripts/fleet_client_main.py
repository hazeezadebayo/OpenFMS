#!/usr/bin/env python3

import os, rclpy, warnings, yaml, time, sys, ast, math, gc, signal
from subprocess import Popen
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)
import threading
import viro_simple_fleet.db_register as db_register
import viro_simple_fleet.db_manager as db_manager

# profinet_connection_path = (os.path.abspath(os.path.join(os.path.dirname(__file__), '../'+ '/viro_core/')))
# sys.path.append(profinet_connection_path)
# from profinet_connection import profinet_connection

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
#                        AGV CLIENT FleetClient                           #
# ----------------------------------------------------------------------- #

# AGV CLIENT NODE
class FleetClient(Node):
    """
    Args:
        Node (_type_): _description_
    """
    def __init__(self):
        super().__init__('FleetClient')

        # Create a lock object
        self.lock = threading.Lock()

        # re-initialize messages
        self.group = ReentrantCallbackGroup()
        self.twist = Twist()
        self.msg_notice = String()

        package_name = 'viro_simple_fleet'
        self.package_share_directory = get_package_share_directory(package_name)

        self.declare_parameter("robot_id", '2')
        self.declare_parameter("fleet_id", 'kullar')
        self.declare_parameter("hostname", 'localhost')
        self.declare_parameter("database", 'postgres')
        self.declare_parameter("username", 'postgres')
        self.declare_parameter("pwd", 'root')
        self.declare_parameter("port", 5432)
        self.declare_parameter("fleet_base_map_pgm", self.package_share_directory+'/maps/my_cartographer_map.pgm')
        self.declare_parameter("fleet_base_map_yaml", self.package_share_directory+'/maps/my_cartographer_map.yaml')
        self.declare_parameter("fleet_floor1_map_pgm", self.package_share_directory+'/maps/devshop.pgm')
        self.declare_parameter("fleet_floor1_map_yaml", self.package_share_directory+'/maps/devshop.yaml')
        self.declare_parameter("fleet_coverage_plan_yaml", '')
        self.declare_parameter("wheel_seperation", 7.0)
        self.declare_parameter("wheel_radius", 3.0)
        self.declare_parameter('current_pose', ['x','y','z'])
        self.declare_parameter("use_vda5050", True)
        self.declare_parameter("angular_vel", 0.9)
        self.declare_parameter("linear_vel", 0.4)
        self.declare_parameter("db_readwrite_freq", 10.0)
        self.declare_parameter("bat_stat", 50.0)
        # MQTT bridge parameters
        self.declare_parameter("mqtt_address", 'localhost')
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_username", '')
        self.declare_parameter("mqtt_password", '')
        self.declare_parameter("vda5050_version", '2.0.0')
        self.declare_parameter("manufacturer", 'birfen')
        self.declare_parameter("system_version", 'v1')

        self.mqtt_address = self.get_parameter("mqtt_address").get_parameter_value().string_value
        self.mqtt_port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        self.mqtt_username = self.get_parameter("mqtt_username").get_parameter_value().string_value
        self.mqtt_password = self.get_parameter("mqtt_password").get_parameter_value().string_value
        self.version = self.get_parameter("vda5050_version").get_parameter_value().string_value
        self.manufacturer = self.get_parameter("manufacturer").get_parameter_value().string_value
        major_version = self.get_parameter("system_version").get_parameter_value().string_value
        self.use_vda5050 = self.get_parameter("use_vda5050").get_parameter_value().bool_value
        robot_id = self.get_parameter("robot_id").get_parameter_value().string_value # .integer_value
        fleet_id = self.get_parameter("fleet_id").get_parameter_value().string_value
        hostname = self.get_parameter('hostname').get_parameter_value().string_value
        database = self.get_parameter("database").get_parameter_value().string_value
        username = self.get_parameter("username").get_parameter_value().string_value
        pwd = self.get_parameter('pwd').get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        fleet_base_map_pgm = self.get_parameter("fleet_base_map_pgm").get_parameter_value().string_value
        self.fleet_base_map_yaml = self.get_parameter("fleet_base_map_yaml").get_parameter_value().string_value
        fleet_floor1_map_pgm = self.get_parameter("fleet_floor1_map_pgm").get_parameter_value().string_value
        self.fleet_floor1_map_yaml = self.get_parameter("fleet_floor1_map_yaml").get_parameter_value().string_value
        self.fleet_coverage_plan_yaml = self.get_parameter("fleet_coverage_plan_yaml").get_parameter_value().string_value
        wheel_seperation = self.get_parameter("wheel_seperation").get_parameter_value().double_value
        wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        ang_vel = self.get_parameter("angular_vel").get_parameter_value().double_value
        lin_vel = self.get_parameter("linear_vel").get_parameter_value().double_value
        db_readwrite_freq = self.get_parameter("db_readwrite_freq").get_parameter_value().double_value
        bat_stat = self.get_parameter("bat_stat").get_parameter_value().double_value
        curr_pose = self.get_parameter('current_pose').get_parameter_value().string_array_value
        # declare robot ros topic [fleet_id ---> interface_name | robot_id ---> serial_number]
        self.instant_action_topic = fleet_id+"/"+major_version+"/"+self.manufacturer+"/"+str(robot_id)+"/instantActions"
        self.order_topic = fleet_id+"/"+major_version+"/"+self.manufacturer+"/"+str(robot_id)+"/order"
        self.connection_topic = "/"+fleet_id+"/"+major_version+"/"+self.manufacturer+"/"+str(robot_id)+"/connection"
        self.state_topic = "/"+fleet_id+"/"+major_version+"/"+self.manufacturer+"/"+str(robot_id)+"/state"

        # os.environ["ROS_DOMAIN_ID"] = str(robot_id)
        # os.environ["ROS_LOCALHOST_ONLY"] = str(0) # str(1)   # 0 - works without internet | 1 - should work as purely localhost
        # self.cli_cmd = "ros2 run viro_simple_fleet fleet_client_navigator.py"
        self.cli_cmd = "ros2 launch viro_simple_fleet connector_tb3.launch.py"

        # register robot on database
        self.db_registration = db_register.DatabaseRegistration(
            robot_id, fleet_id, fleet_base_map_pgm, fleet_floor1_map_pgm,
            hostname, database, username, pwd, port, ang_vel, lin_vel,
            wheel_seperation, wheel_radius, curr_pose, bat_stat)

        # connect to database
        self.db_manager = db_manager.DatabaseManager(hostname, database, username, pwd, port, robot_id, fleet_id)
        self.dbm_flag = self.db_manager.flag
        self.db_manager.ros_state_clearance_intervel_sec = 250 # seconds

        # subscribers
        self.battery_sub = self.create_subscription(BatteryState, 'battery_status', self.battery_cb, 10)
        self.simple_fleet_notice_sub = self.create_subscription(String,'/'+package_name+'/external_notice', self.simple_fleet_ext_cb, 10)
        self.odom_pose_sub = self.create_subscription(Odometry,'/odom', self.position_cb, 1, callback_group=self.group)
        self.last_checkpoints_sub = self.create_subscription(String,'/'+package_name+'/last_checkpoints', self.last_checkpoint_cb, 10)

        # publishers
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel', 1)
        self.simple_fleet_notice_pub = self.create_publisher(String,'/'+package_name+'/internal_notice', 1)

        # timer callback
        self.update_frequency = db_readwrite_freq # 6.0
        self.odom_timer = self.create_timer(self.update_frequency, self.agv_cb)

        # declare and initialize variables
        if curr_pose[0]!='x' and curr_pose[1]!='y' and curr_pose[2]!='z':
            self.pose_x, self.pose_y, self.pose_th = float(curr_pose[0]), float(curr_pose[1]), float(curr_pose[2])
        else:
            print("Oops! enter a valid current robot pose")
            return
        self.system_launch_process = None
        # 'notice_msg,notice_msg,node_troubleshoot,0,None,x,y',  # error_msg,error_msg,node_troubleshoot,wait_time,post/pre/none,
        self.notifications = ['None'] * 3
        self.dummy_checkpoints = ['A','A','A']
        self.landmarks = None
        self.agv_status = None
        self.agv_itinerary = []
        self.last_checkpoint = ['unknown']
        self.wait_time_switch = None
        self.manual_control = False
        # initialize sample message:
        self.battery_percentage = None # percent
        self.emergency_present = None # bool
        # self.map_saver_counter = 0
        self.linear_vel = lin_vel # 0.4
        self.angular_vel = ang_vel # 0.9
        self.previous_notice = None
        self.elevator_map_request = None
        self.dock_status = False
        self.print_count = 0

    # ----------------------------------------------------------------------- #
    #                           define HELPER fnc.                            #
    # ----------------------------------------------------------------------- #

    def euler_to_quaternion(self, yaw, pitch, roll):
        """
            yaw, pitch, roll ---> x y z w
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    # x y z w --> yaw or theta
    def euler_from_quaternion(self, x, y, z, w):
        """
            x y z w --> yaw or theta
        """
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z # in radians

    # odom pose sub
    def position_cb(self, msg):
        """
            obtain robot's current pose from odom
        """
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.pose_th = self.euler_from_quaternion(0, 0, msg.pose.pose.orientation.z , msg.pose.pose.orientation.w) # x y z w -orientation
        # self.get_logger().info("[main]:-pose-["+str([self.pose_x, self.pose_y, self.pose_th])+"]. \n")

    def battery_cb(self, msg):
        """ battery_state subscription
        #    battery percentage [float], emergency_present [bool]
        #    should mean battery present or not but we are using it for emergency state placeholder
        """
        self.battery_percentage = msg.percentage
        # should mean battery present or not
        # but I am using it for emergency state placeholder
        self.emergency_present = msg.present

    # last checkpoint listener
    def last_checkpoint_cb(self, msg):
        """
            state monitoring. obtain last checkpoints to be uploaded to db.
        """
        delim = ','
        self.last_checkpoint = msg.data.split(delim)

    def simple_fleet_int_pub(self, notification):
        """
            publish internal notification states essentially traffic management.
            red | green
        """
        # pause / stop the robot
        if notification == 'red' or \
                notification == 'inactive' or \
                notification == 'idle':
            self.msg_notice.data = 'red'
        # play / continue moving the robot
        elif notification == 'active' or \
                notification == 'green':
            self.msg_notice.data = 'green'
        # special cases:
        elif ',' in notification:
            # publish [green, 'skip'-nextnext_stop_id]: we wanna skip our current target for next.
            # publish [green, 'x,y'-alternate_route]: we want to move to a shifted midpoint or alternate goal.
            # publish [green, elevator_map_response]: for elevator cases where we have to switch map.
            self.msg_notice.data = notification
        # ensure we publish only when we have new information
        # if self.previous_notice != self.msg_notice.data:
        self.simple_fleet_notice_pub.publish(self.msg_notice)
        #    self.previous_notice = self.msg_notice.data
        self.get_logger().info("[main]:-int_notice-["+str([self.msg_notice.data])+"]. \n")

    def simple_fleet_ext_cb(self, msg):
        """ handle external related messages intended to be forwarded to the db. """
        if ',' not in msg.data:
            # we need to handle other notifications and send to db.
            # [None ext, None ext, None node_troubleshoot, None waittime, None post/pre, None x, None y]
            # ------------------------------------------------------------------------
            # dock_required | dock_completed | undock_required | motion
            # no_motion | negotiation_required | negotiation_completed
            # warning_stop | warning_slow | no_warning | undock_completed
            # ------------------------------------------------------------------------
            # elevator_required | startup_failed | startup_success | recovery_required
            # ------------------------------------------------------------------------
            # this requires human attention!!
            if (msg.data == 'elevator_required') or (msg.data == 'startup_failed') \
                or (msg.data == 'recovery_required'):
                if msg.data == 'elevator_required':
                    # we know we found E in the alphabet for the place we just reached
                    # we have guessed its an elevator, we want to get some form of confirmation as
                    # to what map we are changing to or from.
                    # if we find in agv_status[5] = base_map
                    #       we switch to floor1_map
                    # elif we find in agv_status[5] = E_ doesnt matter
                    #       we switch back to base_map
                    # therefore, by requesting we are waiting for a confirmation of what
                    # is currently contained in agv_status[5]
                    self.elevator_map_request = True
                self.notifications[0] = msg.data
            # this does not. only reports robot's status.
            else:
                if msg.data == 'dock_completed':
                    self.dock_status = True

                self.notifications[2] = msg.data
        # else:
        #    self.x_stat = msg.data.split(',')
        self.get_logger().info("[main]:-ext_notice-["+str([msg.data])+"]. \n")

    # ----------------------------------------------------------------------- #
    #                        define system launch fnc.                        #
    # ----------------------------------------------------------------------- #

    def start_system_launch(self):
        """
        Starts the system launch process.
        """
        time.sleep(1.0)
        if not self.system_launch_process:
            try:
                self.system_launch_process = Popen(self.cli_cmd, shell=True, preexec_fn=os.setpgrp, executable="/bin/bash")
            except Exception as err:
                raise RuntimeError("Failed to start system launch process") from err

    def stop_system_launch(self):
        """
        Stops the system launch process.
        """
        if self.system_launch_process:
            os.killpg(os.getpgid(self.system_launch_process.pid), signal.SIGTERM)
            self.system_launch_process = None

    # ----------------------------------------------------------------------- #
    #                       define MANUAL CONTROL fnc.                        #
    # ----------------------------------------------------------------------- #

    def cmd_vel_db_control(self, move_left, move_forward, move_right, stop, move_backward):
        """_
        publish manual cmd_vel commands from ui/db
        """
        # execute drive logic:
        if move_right == 'True':
            vx = 0.01
            vth = -1 * self.angular_vel
        elif move_forward == 'True':
            vx = self.linear_vel
            vth = 0.0
        elif move_left == 'True':
            vx = 0.01
            vth = self.angular_vel
        elif move_backward == 'True':
            vx = -1 * self.linear_vel
            vth = 0.0
        elif stop == 'True':
            vx = 0.0
            vth = 0.0
        else:
            vx = 0.0
            vth = 0.0
        # publish ros twist message
        self.twist.linear.x = vx
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = vth
        self.cmd_vel_pub.publish(self.twist)

    # ----------------------------------------------------------------------- #
    #                           define MAIN loop fnc.                         #
    # ----------------------------------------------------------------------- #

    def agv_cb(self):
        """ timer callback function that updates db and nav variables. """

        # Acquire the lock
        with self.lock:
            # Perform operations that need synchronization
            # Call script B functions here

            # self.get_logger().info(" \n")
            if self.system_launch_process is not None:
                self.print_count += 1
                if self.print_count % 10 == 0:
                    self.print_count = 0
                    self.get_logger().info("[main]:-last_checkpoint-["+str([self.last_checkpoint])+"]. \n")
                # self.db_manager.db_count = self.count

            # read from db
            self.db_manager.read_db()
            self.get_logger().info("[main]:-db_manager.flag-["+str([self.db_manager.flag])+"]. \n")
            # self.get_logger().info("\n checkpoints : ["+str(self.db_manager.checkpoints)+"]. \n")

            # only if we have a launch process, we need to put them in the db yeah!
            self.db_manager.ros_system_launch_process = self.system_launch_process
            self.db_manager.ros_pose_x, self.db_manager.ros_pose_y, self.db_manager.ros_pose_th = self.pose_x, self.pose_y, self.pose_th
            self.db_manager.ros_notifications = self.notifications
            self.db_manager.ros_last_checkpoint = self.last_checkpoint
            self.db_manager.ros_dummy_checkpoints = self.dummy_checkpoints
            self.db_manager.ros_dock_status = self.dock_status
            # get battery and emergency state to be used by the database
            self.db_manager.ros_battery_percentage = self.battery_percentage
            self.db_manager.ros_emergency_present = self.emergency_present

            # we have just encountered an elevator and we desire to switch the map
            # we need to notify the db as well.
            if self.elevator_map_request is not None:
                # one of this cases would always be satisfied by default.
                # but it will be a special case if a switch map is requested.
                if self.db_manager.agv_status[5] == 'base_map':
                    self.db_manager.ros_elevator_map_request = 'floor1_map'
                elif self.db_manager.agv_status[5] == 'floor1_map':
                    self.db_manager.ros_elevator_map_request = 'base_map'
                self.elevator_map_request = None

            # self.get_logger().info("[main]:-b4 update last_checkpoint: ["+str(self.last_checkpoint)+"]. \n")
            # update db with new information
            self.db_manager.update_db()
            time.sleep(0.1)
            # self.get_logger().info("[main]:-5ta update ros_last_checkpoint: ["+str(self.db_manager.ros_last_checkpoint)+"]. \n")
            # self.get_logger().info(" [main] : -----update done----- ["+str([8])+"]. \n")

            # after they were uploaded, some had their internal states changed.
            self.notifications = self.db_manager.ros_notifications
            self.last_checkpoint = self.db_manager.ros_last_checkpoint
            self.dummy_checkpoints = self.db_manager.ros_dummy_checkpoints
            self.dock_status = self.db_manager.ros_dock_status

            # now we write new and updated states to agv
            # -----------------------------------------------------SINGLE OR MULTI DESTINATION  ---------------------------------------------
            if (self.dummy_checkpoints != self.db_manager.checkpoints) and \
                (self.db_manager.checkpoints[0] != 'A' and self.db_manager.checkpoints[1] != 'A') and \
                    (self.db_manager.shutdown == 'no'):
                self.get_logger().info("[main]:-Multi destination: checkpoints or agv itinerary started-["+str([1])+"]. \n")
                self.dummy_checkpoints = self.db_manager.checkpoints
                self.stop_system_launch()
                dictionary = {
                    "initial_pose": self.db_manager.current_pose,
                    "agv_itinerary": self.db_manager.agv_itinerary,
                    "checkpoints": self.db_manager.checkpoints,
                    "landmarks": self.db_manager.landmarks,
                    "waitpoints": self.db_manager.waitpoints,
                    "wait_itinerary": self.db_manager.wait_itinerary,
                    "update_time": time.time_ns(),
                    "curr_dock": self.db_manager.agv_status[1:], # recall! [0 active/inactive/idle, 1 x, 2 y, 3 z, 4 w]
                    "fleet_base_map_yaml": self.fleet_base_map_yaml,
                    "fleet_floor1_map_yaml": self.fleet_floor1_map_yaml,
                    "fleet_coverage_plan_yaml": self.fleet_coverage_plan_yaml,
                    "startup_map": self.db_manager.agv_status[5],
                    "use_vda5050": self.use_vda5050,
                    "version": self.version,
                    "manufacturer": self.manufacturer,
                    "serial_number": self.db_manager.robot_id,
                    "connection_topic": self.connection_topic,
                    "state_topic": self.state_topic,
                    "order_topic": self.order_topic,
                    "instant_action_topic": self.instant_action_topic,
                    "mqtt_address": self.mqtt_address,
                    "mqtt_port": self.mqtt_port,
                    "mqtt_username": self.mqtt_username,
                    "mqtt_password": self.mqtt_password}
                with open(self.package_share_directory + "/config/session_info.yaml", "w", encoding="utf-8") as outfile:
                    yaml.dump(dictionary, outfile)
                time.sleep(0.5)
                self.start_system_launch()
                self.manual_control = False

            # ---------------------------------------------------- MANUAL CONTROL --------------------------------------------------
            elif (self.db_manager.controls[0] == 'yes') and \
                (self.db_manager.checkpoints[0] == self.db_manager.checkpoints[1] == self.db_manager.checkpoints[2]) and \
                    (self.db_manager.shutdown == 'no'):
                self.get_logger().info("[main]:-manual control started-["+str([2])+"]. \n")
                self.last_checkpoint = ['manual']
                self.dummy_checkpoints = ['A','A','A']
                if self.manual_control is False:
                    self.stop_system_launch()
                    time.sleep(0.5)
                    self.manual_control = True # move_left, move_forward, move_right, stop, move_backward
                self.cmd_vel_db_control(self.db_manager.controls[1], self.db_manager.controls[2], self.db_manager.controls[3], self.db_manager.controls[4], self.db_manager.controls[5])

            # --------------------------------------------------- SHUTDOWN | AGV STATUS -------------------------------------------------------
            if self.db_manager.shutdown == 'no':
                # self.get_logger().info(" [main] : no shutdown ["+str([3])+"]. \n")
                # print("3. agv_status confirmed.")
                if self.dbm_flag != self.db_manager.flag:
                    self.dbm_flag = self.db_manager.flag # red / green
                    self.simple_fleet_int_pub(self.dbm_flag)
                    self.get_logger().info("[main]:-dbm_flag-["+str([self.dbm_flag])+"]. \n")

                if self.agv_status != self.db_manager.agv_status[0]:
                    self.agv_status = self.db_manager.agv_status[0] # active / inactive / idle

                    if self.agv_status == 'idle':
                        self.stop_system_launch()
                        self.dummy_checkpoints = ['A','A','A']
                        self.last_checkpoint = ['unknown']
                        time.sleep(0.1)
                    # if agv status is manipulated from the fleet_manager [active/inactive] gui then please publish
                    elif (self.agv_status == 'active') or (self.agv_status == 'inactive'): #  or (self.agv_status == 'idle')
                        self.simple_fleet_int_pub(self.agv_status)
                        self.get_logger().info("[main]:-agv_status-["+str([self.agv_status])+"]. \n")

            # if shutdown is yes you should take the state back to inactive or does it matter?
            elif self.db_manager.shutdown == 'yes':
                self.get_logger().info("[main]:-shutdown-["+str([4])+"]. \n")
                # print("4. agv system shutdown.")
                self.dummy_checkpoints = ['A','A','A']
                self.stop_system_launch()
                time.sleep(0.1)

# ----------------------------------------------------------------------- #
#                                    MAIN                                 #
# ----------------------------------------------------------------------- #

def main(args=None):
    """
        main node starter.
    """
    rclpy.init(args=args)
    try:
        fleet_client = FleetClient()
        rclpy.spin(fleet_client)
    except (TypeError, ValueError) as e:
        print(f"An error occurred: {e}")
        fleet_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
