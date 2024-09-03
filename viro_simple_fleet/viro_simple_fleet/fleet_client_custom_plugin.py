
#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan

from viro_simple_fleet.robot_navigator import BasicNavigator, TaskResult

from vda5050_connector.action import NavigateToNode
from vda5050_connector.action import ProcessVDAAction

from vda5050_connector.srv import GetState
from vda5050_connector.srv import SupportedActions

from vda5050_msgs.msg import Connection as VDAConnection
from vda5050_msgs.msg import AGVPosition as VDAAGVPosition
from vda5050_msgs.msg import Velocity as VDAVelocity
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import CurrentAction as VDACurrentAction


import json, uuid, datetime, math, time, random, threading



## TODO
# function at the manager side that constantly checks table_robot
# to see if two robots have the same traffic and the quickly stops one robot
# changes its reservation to previous node id if no one has requested for
# it.

# first instance appear to be none and it moves. why?

# USAGE:
# Terminal 1:
# cd docker_ws/env/dev1
# export DISPLAY=:0.0
# xhost +local:docker
# docker-compose up --build
# or
# docker run -it dev3_tailscaled /bin/bash

# -------------- NO MQTT?
# Terminal 2:
# docker exec -it dev3-tailscaled-1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch viro_simple_fleet fleet_client.launch.py

# Terminal 3:
# docker exec -it dev3-tailscaled-1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; python3 FleetManager/viro_simple_fleet/scripts/fleet_mngr_main.py

# -------------- MQTT?
# Terminal 2:
# docker exec -it dev3_tailscaled_1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch vda5050_tb3_adapter connector_tb3.launch.py
#

# Terminal 3:
# docker exec -it dev3_tailscaled_1 bash
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

class FleetClientCustomPlugin:
    """
    FleetClientCustomPlugin
    """
    def __init__(self):
        self.navigator = None
        self.nav_start = None
        self.cancel_goal = False
        self.skip_goal = False
        self.continue_goal = False
        self.io_state = True
        self.interrupt = False
        self.pre_recovery_timeout = 200.0
        self.recovery_retry_counter = 0
        self.recovery_retry_max = 3
        self.goal_angle_tol = 47 # [degrees]
        self.goal_distance_tol = 0.75 # [m]

        # Pose attributes
        self.quat_z, self.quat_w, self.quat_x, self.quat_y = 0.0, 0.0, 0.0, 0.0
        self._agv_position = VDAAGVPosition()
        self._velocity = VDAVelocity()

        # VDA5050 MQTT attributes
        self.manufacturer = None
        self.serial_number = None
        self.connection_topic = None
        self.state_topic = None
        self.use_vda5050 = None
        self.order_id = None
        self.driving = False
        self.last_node_id = None
        self.goal_node_id = None
        self.error_type = None # orderUpdateError
        self.error_description = None # string
        self.map_id = None
        self.vda5050_connection_state = None
        self.vda5050_task_state = None
        self.executor = None
        self.executor_thread = None
        self.completed_tracker = None

    # ................................
    # ...... HELPER FUNCTIONS ........
    # ................................

    def get_distance_to_goal(self, goal_id):
        """
        Get the distance between the current x,y coordinate and the desired x,y coordinate. The unit is meters.
        """
        distance_to_goal = math.sqrt((goal_id[0] - self._agv_position.x) ** 2 + (goal_id[1] - self._agv_position.y) ** 2)
        return distance_to_goal


    def get_heading_error(self, goal_id, docked=True):
        """
        Get the heading error in radians.
        """
        delta_x = goal_id[0] - self._agv_position.x
        delta_y = goal_id[1] - self._agv_position.y

        if docked:
            delta_x = self._agv_position.x - goal_id[0]
            delta_y = self._agv_position.y - goal_id[1]

        desired_heading = math.atan2(delta_y, delta_x)
        heading_error = desired_heading - self._agv_position.theta

        # Normalize the heading error to be within -PI to PI range
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        return heading_error


    def get_radians_to_goal(self, goal_id):
        """
        Get the yaw goal angle error in radians.
        """
        yaw_goal_angle_error = goal_id[2] - self._agv_position.theta
        return yaw_goal_angle_error


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


    def euler_to_quaternion(self, yaw, pitch, roll):
        """
        Convert Euler angles (yaw, pitch, roll) to quaternion (qx, qy, qz, qw).
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    # ................................
    # .......... CALLBACKS ...........
    # ................................

    def position_cb(self, msg):
        """
        Track robot pose with odom position callback.
        """
        self.quat_z = msg.pose.pose.orientation.z
        self.quat_w = msg.pose.pose.orientation.w
        self.quat_x = msg.pose.pose.orientation.x
        self.quat_y = msg.pose.pose.orientation.y

        # Feedback pose/twist
        self._agv_position.map_id = "map"
        self._agv_position.theta = self.quaternion_to_euler(self.quat_x, self.quat_y, self.quat_z, self.quat_w)[-1]
        self._agv_position.x = msg.pose.pose.position.x
        self._agv_position.y = msg.pose.pose.position.y
        self._velocity.vx = msg.twist.twist.linear.x
        self._velocity.vy = msg.twist.twist.linear.y
        self._velocity.omega = msg.twist.twist.angular.z


    def connection_cb(self, msg):
        """
        VDA5050 connection callback.
        """
        self.vda5050_connection_state = msg.connection_state
        self.vda5050_task_state = msg.task_state
        if self.vda5050_task_state == "COMPLETED":
            self.completed_tracker = self.vda5050_task_state
        self.navigator.get_logger().info(f"[plugin] state [{[self.vda5050_connection_state, self.vda5050_task_state]}]. \n")


    def state_cb(self, msg):
        """
        VDA5050 state callback.
        """
        self.order_id = msg.order_id
        self.last_node_id = msg.last_node_id
        for node_state in msg.node_states:
            self.goal_node_id = node_state.node_id
            self.navigator.get_logger().info(f"[plugin] goal_node_id [{[self.goal_node_id]}]. \n")
        if msg.errors:
            self.error_type = msg.errors[-1].error_type
            self.error_description = msg.errors[-1].error_description
            self.navigator.get_logger().info(f"[plugin] error_type [{[self.error_type]}]. \n")
        # Count the number of "beep" actions with a status of "FINISHED" in the action_states list
        # self.beep_count = sum(action.action_type == "beep" and action.action_status == "FINISHED" for action in msg.action_states)
        # self.navigator.get_logger().info("[plugin]:-action_states- ["+str([msg.action_states])+"]. \n")


    def get_state_callback(self, request, response):
        """
        Set the state (position, velocity) of the robot.
        """
        order_state = VDAOrderState()
        order_state.agv_position = self._agv_position
        order_state.velocity = self._velocity
        order_state.driving = self.driving if isinstance(self.driving, bool) else order_state.driving
        response.state = order_state
        return response


    def process_vda_action_callback(self, goal_handle):
        """
        Process VDA action callback when a task is completed.
        """
        action = goal_handle.request.action  # vda5050_msgs>msg>Action.msg
        result = ProcessVDAAction.Result()
        action_parameters = {param.key: param.value for param in action.action_parameters}

        self.navigator.get_logger().info(f"Parsed action parameters: {action_parameters}")

        if action.action_type == "initPosition":
            init_pos_x_y_th = [
                float(action_parameters["x"]),
                float(action_parameters["y"]),
                float(action_parameters["theta"]),
                action_parameters["mapId"]
            ]
            self.pub_init_pose(init_pos_x_y_th)
            self.navigator.get_logger().info(f"Instant action '{action.action_id}' finished")
            result.result = VDACurrentAction(
                action_id=action.action_id,
                action_description=action.action_description,
                action_status=VDACurrentAction.FINISHED,
            )
        else:
            self.navigator.get_logger().info(
                f"Received unsupported action: '{action.action_type}'. "
                "Beep Boop Bop ... other actions not implemented yet."
            )
            result.result = VDACurrentAction(
                action_id=action.action_id,
                action_description=action.action_description,
                action_status=VDACurrentAction.FINISHED,
            )

        goal_handle.succeed()
        return result


    def navigate_to_node_callback(self, goal_handle):
        """
        Process VDA node callback for robot motion.
        """
        node = goal_handle.request.node  # vda5050_msgs>msg>Node.msg

        self.navigator.get_logger().info(f"Navigating to node '{node.node_id}', edge '{goal_handle.request.edge}'")
        self.navigator.get_logger().info(f"The actions to do are: {node.actions}")

        q = self.euler_to_quaternion(float(node.node_position.theta), 0.0, 0.0)
        goal_x_y_z_w = [node.node_position.x, node.node_position.y, q[2], q[3], node.node_position.map_id]
        self.drive(goal_x_y_z_w)

        goal_handle.succeed()
        result = NavigateToNode.Result()
        return result

    # ................................
    # ......... publisher ............
    # ................................

    def pub_init_pose(self, init_pos_x_y_th=None):
        """
        Publish initial pose.
        """
        if init_pos_x_y_th is None:
            init_pos_x_y_th = [self._agv_position.x, self._agv_position.y, self._agv_position.theta]

        q = self.euler_to_quaternion(float(init_pos_x_y_th[2]), 0.0, 0.0)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = float(init_pos_x_y_th[0])
        initial_pose.pose.position.y = float(init_pos_x_y_th[1])
        initial_pose.pose.orientation.z = q[2]
        initial_pose.pose.orientation.w = q[3]
        # self.navigator.setInitialPose(initial_pose)

    # ................................
    # ........ ON STARTUP ............
    # ................................

    def on_startup(self, init_pos_x_y_th=None):
        """
        Initialize node capabilities and publish initial pose.
        """
        self.navigator = BasicNavigator()
        self.nav_start = self.navigator.get_clock().now()

        # Add subscriptions and publishers to the existing node
        self.navigator.odom_sub = self.navigator.create_subscription(Odometry, "odom", self.position_cb, 1)
        self.navigator.cmd_vel_pub = self.navigator.create_publisher(Twist, '/cmd_vel', 1)
        self.navigator.battery_pub = self.navigator.create_publisher(BatteryState, 'battery_status', 1)
        self.navigator.pose_pub = self.navigator.create_publisher(PoseStamped, '/goal_pose', 1)

        # Publish initial pose if provided
        if init_pos_x_y_th is not None:
            self.pub_init_pose(init_pos_x_y_th)

        # Ensure AMCL is up and running
        # self.navigator.waitUntilNav2Active()
        time.sleep(0.1)

        # Initialize VDA5050 MQTT communication if enabled
        self.navigator.get_logger().info(f"[plugin] mqtt+vda5050: [{self.use_vda5050}]. \n")
        if self.use_vda5050:
            self.initialize_vda5050_communication()

        # Start the executor thread
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.navigator)
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.start()


    def initialize_vda5050_communication(self):
        """
        Initialize VDA5050 MQTT communication.
        """
        self.navigator.connection_sub = self.navigator.create_subscription(VDAConnection, self.connection_topic, self.connection_cb, 1)
        self.navigator.state_sub = self.navigator.create_subscription(VDAOrderState, self.state_topic, self.state_cb, 1)

        base_interface_name = f"vda5050_connector/{self.manufacturer}/{self.serial_number}/"

        # Hosted services
        self.navigator.get_adapter_state_srv = self.navigator.create_service(
            srv_type=GetState,
            srv_name=base_interface_name + "adapter/get_state",
            callback=self.get_state_callback,
        )
        self.navigator.supported_actions_srv = self.navigator.create_service(
            SupportedActions,
            base_interface_name + "adapter/supported_actions",
            lambda _: self.navigator.get_logger().info("[plugin] supported actions request not implemented. \n"),
        )

        # Hosted actions
        self.navigator.process_vda_action_ac_srv = ActionServer(
            node=self.navigator,
            action_type=ProcessVDAAction,
            action_name=base_interface_name + "adapter/vda_action",
            execute_callback=self.process_vda_action_callback,
        )
        self.navigator.nav_to_node_ac_srv = ActionServer(
            node=self.navigator,
            action_type=NavigateToNode,
            action_name=base_interface_name + "adapter/nav_to_node",
            execute_callback=self.navigate_to_node_callback,
        )
        self.navigator.get_logger().info("[plugin] initialization done. \n")


    def startup_config_check(self):
        """
        Action client for sending NavigateToNode goals to adapter.
        """
        base_interface_name = f"vda5050_connector/{self.manufacturer}/{self.serial_number}/"
        _nav_to_node_act_srv = "adapter/nav_to_node"
        navigate_to_node_act_cli = ActionClient(
            node=self.navigator,
            action_type=NavigateToNode,
            action_name=base_interface_name + _nav_to_node_act_srv,
        )

        while not navigate_to_node_act_cli.wait_for_server(timeout_sec=1.0):
            self.navigator.get_logger().error("[plugin] NavigateToNode adapter action server not available, restarting...")
            time.sleep(1)
            self.on_startup()

        if navigate_to_node_act_cli:
            navigate_to_node_act_cli.destroy()
            navigate_to_node_act_cli = None
            self.navigator.get_logger().info("[plugin] Test action client destroyed.")

    # ................................
    # .......... STOP ................
    # ................................

    def stop(self):
        """
        Cancel or stop the navigation task.
        """
        self.interrupt = True
        self.nav_start = self.navigator.get_clock().now()
        self.navigator.cancelTask()

    # ................................
    # ........ CHANGE MAP ............
    # ................................

    def load_map(self, map_yaml):
        """
        Change the current map, e.g., elevator states.
        """
        self.navigator.changeMap(map_yaml)
        self.navigator.clearAllCostmaps()

    # ................................
    # ........ RECOVERY ..............
    # ................................

    def recovery(self):
        """
        Perform recovery actions if the robot is stuck.
        """
        time_allowed = 5  # [secs]
        self.stop()
        self.navigator.clearAllCostmaps()
        self.navigator.backup(backup_dist=0.5, backup_speed=0.1, time_allowance=time_allowed)
        time.sleep(time_allowed + 1)

        result = self.navigator.getResult()
        return result == TaskResult.SUCCEEDED

    # ................................
    # ........... DRIVE ..............
    # ................................

    def drive(self, goal_x_y_z_w):
        """
        Go to our destination/goal pose.
        Return success or failure.
        If failure, increment number of retries.
        """
        self.navigator.get_logger().info("[plugin] drive call received. \n")
        self.nav_start = self.navigator.get_clock().now()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav_start.to_msg()
        goal_pose.pose.position.x = float(goal_x_y_z_w[0])
        goal_pose.pose.position.y = float(goal_x_y_z_w[1])
        goal_pose.pose.orientation.z = float(goal_x_y_z_w[2])
        goal_pose.pose.orientation.w = float(goal_x_y_z_w[3])

        self.driving = True
        while self.driving:
            self.navigator.pose_pub.publish(goal_pose)
            time.sleep(0.1)

            if self.interrupt:
                self._handle_interrupt(goal_pose)
                return self.driving

            if (self.navigator.get_clock().now() - self.nav_start) > Duration(seconds=self.pre_recovery_timeout):
                self.recovery_retry_counter += 1
                if not self.recovery():
                    self.driving = False
                    return self.driving

            if self._has_reached_goal(goal_x_y_z_w):
                self.navigator.get_logger().info("[plugin] .-.-reached-.-. \n")
                self.recovery_retry_counter = 0
                self.driving = False
                return True

    def _handle_interrupt(self, goal_pose):
        """
        Handle navigation interruption.
        """
        goal_pose.pose.position.x = float(self._agv_position.x)
        goal_pose.pose.position.y = float(self._agv_position.y)
        goal_pose.pose.orientation.z = float(self.quat_z)
        goal_pose.pose.orientation.w = float(self.quat_w)
        self.navigator.pose_pub.publish(goal_pose)
        self.navigator.get_logger().info("[plugin] interrupted. \n")
        self.driving = False

    def _has_reached_goal(self, goal_x_y_z_w):
        """
        Check if the robot has reached the goal.
        """
        d = math.sqrt((float(goal_x_y_z_w[0]) - float(self._agv_position.x))**2 + (float(goal_x_y_z_w[1]) - float(self._agv_position.y))**2)
        if d < self.goal_distance_tol:
            current_yaw = abs(self.quaternion_to_euler(self.quat_x, self.quat_y, self.quat_z, self.quat_w)[-1])
            goal_yaw = abs(self.quaternion_to_euler(0.0, 0.0, goal_x_y_z_w[2], goal_x_y_z_w[3])[-1])
            if goal_yaw - math.radians(self.goal_angle_tol) <= current_yaw <= goal_yaw + math.radians(self.goal_angle_tol):
                return True
        return False

    # ................................
    # ............ DOCK ..............
    # ................................

    def dock(self):
        """
        Simple controller to drive the robot 0.5m forward to dock.
        Assumes every dock location is 0.5m from the landmarks.
        """
        self.navigator.get_logger().info("[plugin] dock call received. \n")
        d = 0.5  # [m]
        goal_th = self.quaternion_to_euler(0.0, 0.0, self.quat_z, self.quat_w)[-1]
        goal_x = self._agv_position.x + d * math.cos(goal_th)
        goal_y = self._agv_position.y + d * math.sin(goal_th)
        goal_idx = [goal_x, goal_y, goal_th]
        cmd_vel_msg = Twist()
        heading_tolerance = 0.04
        yaw_goal_tolerance = 0.07
        goal_distance_tol = self.goal_distance_tol
        goal_achieved = False

        while not goal_achieved:
            if self.interrupt:
                return False

            distance_to_goal = self.get_distance_to_goal(goal_idx)
            heading_error = self.get_heading_error(goal_idx)
            yaw_goal_error = self.get_radians_to_goal(goal_idx)

            if math.fabs(distance_to_goal) > goal_distance_tol:
                if math.fabs(heading_error) > heading_tolerance:
                    cmd_vel_msg.linear.x = 0.01
                    cmd_vel_msg.angular.z = 0.9 if heading_error > 0 else -0.9
                else:
                    cmd_vel_msg.linear.x = 0.35
                    cmd_vel_msg.angular.z = 0.0
            elif math.fabs(yaw_goal_error) > yaw_goal_tolerance:
                cmd_vel_msg.linear.x = 0.01
                cmd_vel_msg.angular.z = 0.9 if yaw_goal_error > 0 else -0.9
            else:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.navigator.cmd_vel_pub.publish(cmd_vel_msg)
                goal_achieved = True
                self.navigator.get_logger().info("[plugin] dock completed. \n")
                return goal_achieved

            cmd_vel_msg.angular.z /= 9
            self.navigator.cmd_vel_pub.publish(cmd_vel_msg)

    # ................................
    # ............ UNDOCK ............
    # ................................

    def undock(self, reverse_goal_x, reverse_goal_y, reverse_goal_z, reverse_goal_w):
        """
        Undock the robot.
        """
        self.navigator.get_logger().info("[plugin] undock called. \n")
        reverse_goal_th = self.quaternion_to_euler(0.0, 0.0, reverse_goal_z, reverse_goal_w)[-1]

        goal_idx = [reverse_goal_x, reverse_goal_y, reverse_goal_th]
        cmd_vel_msg = Twist()
        heading_tolerance = 0.04
        yaw_goal_tolerance = 0.07
        goal_distance_tol = self.goal_distance_tol
        goal_achieved = False

        while not goal_achieved:
            if self.interrupt:
                return False

            distance_to_goal = self.get_distance_to_goal(goal_idx)
            heading_error = self.get_heading_error(goal_idx, docked=True)
            yaw_goal_error = self.get_radians_to_goal(goal_idx)

            if math.fabs(distance_to_goal) > goal_distance_tol:
                if math.fabs(heading_error) > heading_tolerance:
                    cmd_vel_msg.linear.x = -0.01
                    cmd_vel_msg.angular.z = 0.9 if heading_error > 0 else -0.9
                else:
                    cmd_vel_msg.linear.x = -0.35
                    cmd_vel_msg.angular.z = 0.0
            elif math.fabs(yaw_goal_error) > yaw_goal_tolerance:
                cmd_vel_msg.linear.x = -0.01
                cmd_vel_msg.angular.z = 0.9 if yaw_goal_error > 0 else -0.9
            else:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.navigator.cmd_vel_pub.publish(cmd_vel_msg)
                goal_achieved = True
                self.navigator.get_logger().info("[plugin] undock completed. \n")
                return goal_achieved

            cmd_vel_msg.angular.z /= 9
            self.navigator.cmd_vel_pub.publish(cmd_vel_msg)

    # ................................
    # .......... IO STATE ............
    # ................................

    def state_io(self, type_):
        """
        State check on success of reaching a destination or checkpoint.
        type: dock_required | undock_required | elevator_required | waypoint
        """
        self.navigator.get_logger().info("[plugin] state_io. \n")
        time.sleep(0.1)

        if type_ == 'dock_required':
            # Check if lift is up
            if self.check_lift_status():
                return True
            else:
                return False

        elif type_ == 'undock_required':
            # Check if undocking conditions are met
            if self.check_undock_status():
                return True
            else:
                return False

        elif type_ == 'elevator_required':
            # Check if elevator door is open
            if self.check_elevator_door_status():
                return True
            else:
                return False

        elif type_ == 'waypoint':
            # Check if waypoint conditions are met
            if self.check_waypoint_status():
                return True
            else:
                return False

        # Default case if type is not recognized
        return False

    def check_lift_status(self):
        """ Check if the lift is up """
        # Implement your logic to check lift status here
        # For example, using a sensor reading or a message from another system
        return self.io_state

    def check_undock_status(self):
        """ Check if undocking conditions are met """
        # Implement your logic to check undock status here
        return self.io_state

    def check_elevator_door_status(self):
        """ Check if the elevator door is open """
        # Implement your logic to check elevator door status here
        return self.io_state

    def check_waypoint_status(self):
        """ Check if waypoint conditions are met """
        # Implement your logic to check waypoint status here
        return self.io_state


    # ................................
    # ............ EXIT ..............
    # ................................

    def on_exit(self):
        """
        Exit or kill the plugin node.
        """
        if self.navigator is not None:
            self.navigator.destroy_node()

























































































































































# #### Topic
# /kullar/v1/birfen/TB3_1/connection
# /kullar/v1/birfen/TB3_1/factsheet
# /kullar/v1/birfen/TB3_1/instantActions
# /kullar/v1/birfen/TB3_1/order
# /kullar/v1/birfen/TB3_1/state
# /kullar/v1/birfen/TB3_1/visualization

#### Action
# /vda5050_connector/birfen/TB3_1/adapter/nav_to_node
# /vda5050_connector/birfen/TB3_1/adapter/vda_action

#### Service
# /vda5050_connector/birfen/TB3_1/adapter/get_state
# /vda5050_connector/birfen/TB3_1/adapter/supported_actions

# from vda5050_msgs.msg import Connection, Visualization, OrderState
# from vda5050_msgs.msg import Order, Node, Edge, NodePosition, Action

    # def vda5050_send_order(self, itinerary):
    #     """  pub send_order """
    #     msg = Order()
    #     msg.header_id = 1
    #     msg.timestamp = datetime.datetime.now().isoformat()
    #     msg.version = "2.0.0"
    #     msg.manufacturer = "birfen"
    #     msg.serial_number = "TB3_1"
    #     msg.order_id = str(uuid.uuid4())
    #     msg.order_update_id = self.test_count - 1
    #     msg.zone_set_id = ""

    #     for i, point in enumerate(itinerary):
    #         node_id, x, y, z, w = point
    #         theta = self.quaternion_to_euler(0, 0, z, w)[-1]
    #         node = Node()
    #         node.node_id = node_id
    #         node.released = True
    #         node.sequence_id = i*2
    #         node.node_position = NodePosition()
    #         node.node_position.x = x
    #         node.node_position.y = y
    #         node.node_position.theta = theta
    #         node.node_position.map_id = "map"
    #         node.actions = [Action(action_type="beep", action_id=str(uuid.uuid4()), action_description="reached", blocking_type="NONE")]
    #         msg.nodes.append(node)

    #         if i < len(itinerary) - 1:
    #             edge = Edge()
    #             edge.edge_id = f"edge_{node_id}" # f"edge{i+1}"
    #             edge.released = True
    #             edge.sequence_id = i*2+1
    #             edge.start_node_id = node_id
    #             edge.end_node_id = itinerary[i+1][0]
    #             msg.edges.append(edge)

    #     self.navigator.order_pub.publish(msg)
    #     self.test_count += 1
    #     self.navigator.get_logger().info("[vda5050]:- order published-. \n")

# ros2 topic pub /kullar/v1/birfen/TB3_1/order vda5050_msgs/msg/Order '{
#     "header_id": 1,
#     "timestamp": "2024-04-27T12:49:37.268Z",
#     "version": "2.0.0",
#     "manufacturer": "birfen",
#     "serial_number": "TB3_1",
#     "order_id": "'$(cat /proc/sys/kernel/random/uuid)'",
#     "order_update_id": 1,
#     "zone_set_id": "",
#     "nodes": [
#         {
#             "node_id": "node1",
#             "released": true,
#             "sequence_id": 0,
#             "node_position": {
#                 "x": 2.0,
#                 "y": 0.95,
#                 "theta": -0.66,
#                 "map_id": "map"
#             },
#             "actions": []
#         },
#         {
#             "node_id": "node2",
#             "released": true,
#             "sequence_id": 2,
#             "node_position": {
#                 "x": 1.18,
#                 "y": -1.76,
#                 "theta": 0.0,
#                 "map_id": "map"
#             },
#             "actions": [
#                 {
#                     "action_type": "beep",
#                     "action_id": "'$(cat /proc/sys/kernel/random/uuid)'",
#                     "action_description": "Make a beep noise on node",
#                     "blocking_type": "NONE",
#                     "action_parameters": []
#                 }
#             ]
#         },
#         {
#             "node_id": "node3",
#             "released": true,
#             "sequence_id": 4,
#             "node_position": {
#                 "x": -0.38,
#                 "y": 1.89,
#                 "theta": 0.0,
#                 "map_id": "map"
#             },
#             "actions": [
#                 {
#                     "action_type": "beep",
#                     "action_id": "'$(cat /proc/sys/kernel/random/uuid)'",
#                     "action_description": "Make a beep noise on node",
#                     "blocking_type": "NONE",
#                     "action_parameters": []
#                 }
#             ]
#         },
#         {
#             "node_id": "node4",
#             "released": true,
#             "sequence_id": 6,
#             "node_position": {
#                 "x": -0.17,
#                 "y": 1.74,
#                 "theta": -2.6,
#                 "map_id": "map"
#             },
#             "actions": [
#                 {
#                     "action_type": "beep",
#                     "action_id": "'$(cat /proc/sys/kernel/random/uuid)'",
#                     "action_description": "Make a beep noise on node",
#                     "blocking_type": "NONE",
#                     "action_parameters": []
#                 }
#             ]
#         },
#         {
#             "node_id": "node1",
#             "released": true,
#             "sequence_id": 8,
#             "node_position": {
#                 "x": 2.0,
#                 "y": 0.95,
#                 "theta": -0.66,
#                 "map_id": "map"
#             },
#             "actions": [
#                 {
#                     "action_type": "beep",
#                     "action_id": "'$(cat /proc/sys/kernel/random/uuid)'",
#                     "action_description": "Make a beep noise on node",
#                     "blocking_type": "NONE",
#                     "action_parameters": []
#                 }
#             ]
#         }
#     ],
#     "edges": [
#         {
#             "edge_id": "edge1",
#             "released": true,
#             "sequence_id": 1,
#             "start_node_id": "node1",
#             "end_node_id": "node2",
#             "actions": []
#         },
#         {
#             "edge_id": "edge2",
#             "released": true,
#             "sequence_id": 3,
#             "start_node_id": "node2",
#             "end_node_id": "node3",
#             "actions": []
#         },
#         {
#             "edge_id": "edge3",
#             "released": true,
#             "sequence_id": 5,
#             "start_node_id": "node3",
#             "end_node_id": "node4",
#             "actions": []
#         },
#         {
#             "edge_id": "edge4",
#             "released": true,
#             "sequence_id": 7,
#             "start_node_id": "node4",
#             "end_node_id": "node1",
#             "actions": []
#         }
#     ]
# }'
