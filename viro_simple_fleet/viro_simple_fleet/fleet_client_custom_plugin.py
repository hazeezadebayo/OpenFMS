
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

class FleetClientCustomPlugin():
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
        # pose stuff
        self.quat_z, self.quat_w, self.quat_x, self.quat_y = 0.0, 0.0, 0.0, 0.0
        self._agv_position = VDAAGVPosition()
        self._velocity = VDAVelocity()
        self._agv_position.x, self._agv_position.y, self._agv_position.theta = 0.0, 0.0, 0.0
        self._velocity.vx, self._velocity.vy, self._velocity.omega = 0.0, 0.0, 0.0
        # vda5050 mqtt stuff
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
        distance_to_goal = math.sqrt(math.pow(goal_id[0] - self._agv_position.x, 2) + math.pow(goal_id[1] - self._agv_position.y, 2))
        return distance_to_goal

    def get_heading_error(self, goal_id, docked=True):
        """
        Get the heading error in radians
        """
        # assume this is a [dock] action.
        delta_x = goal_id[0] - self._agv_position.x
        delta_y = goal_id[1] - self._agv_position.y
        # if docked is true, then we have docked already and...
        # we wanna drive backward i.e. [undock] reverse.
        if docked is True:
            delta_x = self._agv_position.x - goal_id[0]
            delta_y = self._agv_position.y - goal_id[1]

        desired_heading = math.atan2(delta_y, delta_x)
        heading_error = desired_heading - self._agv_position.theta
        # Make sure the heading error falls within -PI to PI range
        if heading_error > math.pi:
            heading_error = heading_error - (2 * math.pi)
        if heading_error < -math.pi:
            heading_error = heading_error + (2 * math.pi)
        return heading_error

    def get_radians_to_goal(self, goal_id):
        """
        Get the yaw goal angle error in radians
        """
        yaw_goal_angle_error = goal_id[2] - self._agv_position.theta
        return yaw_goal_angle_error

    def quaternion_to_euler(self, x, y, z, w):
        """ x y z w --> roll, pitch, yaw """
        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def euler_to_quaternion(self, yaw, pitch, roll):
        """ special: radians (yaw pitch roll) -->  [qx, qy, qz, qw]
            abs because I want to use it to define tolerance hence + and - ignored. the mag will be used.
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw] # print([qx, qy, qz, qw])

    # ................................
    # .......... CALLBACKS ...........
    # ................................

    def position_cb(self, msg):
        """ track robot pose with odom position callback """
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
        """ vba5050 connection_cb """
        self.vda5050_connection_state = msg.connection_state
        # if self.vda5050_task_state is None:
        self.vda5050_task_state = msg.task_state
        if self.vda5050_task_state == "COMPLETED":
            self.completed_tracker = self.vda5050_task_state
        self.navigator.get_logger().info("[plugin]:-state-["+str([self.vda5050_connection_state, self.vda5050_task_state ])+"]. \n")

    def state_cb(self, msg):
        """ vda5050 state_cb """
        self.order_id = msg.order_id
        self.last_node_id = msg.last_node_id
        for node_state in msg.node_states:
            self.goal_node_id = node_state.node_id
            self.navigator.get_logger().info("[plugin]:-goal_node_id-["+str([self.goal_node_id])+"]. \n")
        if len(msg.errors) != 0:
            self.error_type = msg.errors[-1].error_type
            self.error_description = msg.errors[-1].error_description
            self.navigator.get_logger().info("[plugin]:-error_type-["+str([self.error_type])+"]. \n")
        # Count the number of "beep" actions with a status of "FINISHED" in the action_states list
        # self.beep_count = sum(action.action_type == "beep" and action.action_status == "FINISHED" for action in msg.action_states)
        # self.navigator.get_logger().info("[plugin]:-action_states- ["+str([msg.action_states])+"]. \n")

    def get_state_callback(self, request, response):
        """ set the state (pos, vel) of the robot """
        order_state = VDAOrderState()
        order_state.agv_position = self._agv_position
        order_state.velocity = self._velocity
        order_state.driving = self.driving if isinstance(self.driving, bool) else order_state.driving
        response.state = order_state
        return response

    def process_vda_action_callback(self, goal_handle):
        """ action 'vda5050_msgs.msg.Action.msg' to process when a task is completed """
        action = goal_handle.request.action # vda5050_msgs>msg>Action.msg
        result = ProcessVDAAction.Result()
        action_parameters = {}
        for action_parameter in action.action_parameters:
            action_parameters[action_parameter.key] = action_parameter.value
        self.navigator.get_logger().info(f"Parsed action parameters: {action_parameters}")
        if action.action_type == "initPosition":
            # q = tf_transformations.quaternion_from_euler( 0, 0, float(action_parameters["theta"])) # --> q[0], q[1]..
            init_pos_x_y_th = [float(action_parameters["x"]),
                               float(action_parameters["y"]),
                               float(action_parameters["theta"]),
                               action_parameters["mapId"]]
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
                action_status=VDACurrentAction.FINISHED,  # VDACurrentAction.INITIALIZING, # RUNNING, # PAUSED, # RUNNING, # FINISHED,
            )
        # vda action related stuff
        goal_handle.succeed()
        return result

    def navigate_to_node_callback(self, goal_handle):
        """ action 'vda5050_msgs.msg.Node.msg' to process for robot motion """
        node = goal_handle.request.node # vda5050_msgs>msg>Node.msg

        self.navigator.get_logger().info(f"Navigating to node '{goal_handle.request.node}', edge '{goal_handle.request.edge}'")
        # if you pass an action when the goal was published, say you want something to happen
        # when the robot reaches a node. like beep beep bop stuff or publish init pose etc
        # that action is shown here and executed at the process_vda_action_callback()
        self.navigator.get_logger().info(f"The actions to do are: {node.actions}")

        # q = tf_transformations.quaternion_from_euler(0, 0, node.node_position.theta)
        # goal_x_y_z_w = [x, y, z, w, frame_id]
        q = self.euler_to_quaternion(float(node.node_position.theta), 0.0, 0.0)
        goal_x_y_z_w = [node.node_position.x, node.node_position.y, q[2], q[3], node.node_position.map_id]
        self.drive(goal_x_y_z_w)

        # nav to node related
        goal_handle.succeed()
        result = NavigateToNode.Result()
        return result

    # ................................
    # ......... publisher ............
    # ................................

    def pub_init_pose(self, init_pos_x_y_th=None):
        """ publish initial pose """
        if init_pos_x_y_th is None:
            init_pos_x_y_th = [self._agv_position.x, self._agv_position.y, self._agv_position.theta]
        # publish init pose
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
        """ publish initial pose and  """
        # initialize node capabilities.
        self.navigator = BasicNavigator()
        self.nav_start = self.navigator.get_clock().now()
        # self.navigator.sub_cb_group = ReentrantCallbackGroup() # MutuallyExclusiveCallbackGroup()

        # Add subscription and publishers to the existing node
        # self.amcl_sub = self.node_.create_subscription(PoseWithCovarianceStamped,"/amcl_pose", self.position_cb, 1)
        self.navigator.odom_sub = self.navigator.create_subscription(Odometry, "odom", self.position_cb, 1)
        self.navigator.cmd_vel_pub = self.navigator.create_publisher(Twist, '/cmd_vel', 1)
        self.navigator.battery_pub = self.navigator.create_publisher(BatteryState, 'battery_status', 1)
        self.navigator.pose_pub = self.navigator.create_publisher(PoseStamped,'/goal_pose', 1) # single destination

        # publish init pose
        if init_pos_x_y_th is not None:
            self.pub_init_pose(init_pos_x_y_th)

        # ensure amcl is up and running
        # self.navigator.waitUntilNav2Active()
        time.sleep(0.1)

        # vda5050_mqtt: initialize the subscriptions and vda5050 mqtt comm.
        self.navigator.get_logger().info("[plugin]- mqtt + vda5050 - : ["+str(self.use_vda5050)+"]. \n")
        if self.use_vda5050 is True:

            # mqtt vda5050 subscriptions
            self.navigator.connection_sub = self.navigator.create_subscription(VDAConnection, self.connection_topic, self.connection_cb, 1)
            self.navigator.state_sub = self.navigator.create_subscription(VDAOrderState, self.state_topic, self.state_cb, 1)

            ##### ADAPTER
            _vda_action_act_srv = "adapter/vda_action"
            _nav_to_node_act_srv = "adapter/nav_to_node"
            _supported_actions_svc_srv = "adapter/supported_actions"
            _get_state_svc_srv = "adapter/get_state"

            # base_interface_name = (f"{self.get_namespace()}/{self.manufacturer}/{self.serial_number}/")
            base_interface_name = (f"vda5050_connector/{self.manufacturer}/{self.serial_number}/")

            # hosted services:
            self.navigator.get_adapter_state_srv = self.navigator.create_service(
                srv_type=GetState,
                srv_name=base_interface_name + _get_state_svc_srv,
                callback=self.get_state_callback,
            )
            self.navigator.supported_actions_srv = self.navigator.create_service(
                SupportedActions,
                base_interface_name + _supported_actions_svc_srv,
                lambda _: self.navigator.get_logger().info("[plugin]:- supported actions request not implemented."),
            )

            # hosted actions:
            self.navigator.process_vda_action_ac_srv = ActionServer(
                node = self.navigator,
                action_type = ProcessVDAAction,
                action_name = base_interface_name + _vda_action_act_srv,
                execute_callback = self.process_vda_action_callback,
            )
            self.navigator.nav_to_node_ac_srv = ActionServer(
                node = self.navigator,
                action_type = NavigateToNode,
                action_name = base_interface_name + _nav_to_node_act_srv,
                execute_callback = self.navigate_to_node_callback,
                # callback_group = self.navigator.sub_cb_group,
            )
            self.navigator.get_logger().info("[plugin]:- initialization done-. \n")

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.navigator)
        self.executor_thread = threading.Thread(target=self.executor.spin) # Start spinning the node
        self.executor_thread.start()

    def startup_config_check(self):
        """ Action client for sending NavigateToNode goals to adapter """
        base_interface_name = (f"vda5050_connector/{self.manufacturer}/{self.serial_number}/")
        _nav_to_node_act_srv = "adapter/nav_to_node"
        navigate_to_node_act_cli = ActionClient(
            node = self.navigator,
            action_type = NavigateToNode,
            action_name = base_interface_name + _nav_to_node_act_srv,
            )
        while not navigate_to_node_act_cli.wait_for_server(timeout_sec=1.0):
            self.navigator.get_logger().error("NavigateToNode adapter action server not available, restarting...")
            time.sleep(1)
            self.on_startup()

        if navigate_to_node_act_cli:
            navigate_to_node_act_cli.destroy()
            navigate_to_node_act_cli = None
            self.navigator.get_logger().info("test action client destroyed.")


    # ................................
    # .......... STOP ................
    # ................................

    def stop(self):
        """ cancel or stop the nav task """
        # check interrupt because if for any reason, negotiate path etc. respawns the main function
        # we dont want multiple while loops running. so we want to hopefully terminate one first
        # before calling another. so update interrupt
        self.interrupt = True
        # reset my timer
        self.nav_start = self.navigator.get_clock().now()
        # cancel or stop the nav task
        self.navigator.cancelTask()


    # ................................
    # ........ CHANGE MAP ............
    # ................................

    def load_map(self, map_yaml):
        """ change the current map e.g. elevator states etc. """
        self.navigator.changeMap(map_yaml)
        self.navigator.clearAllCostmaps()

    # ................................
    # ........ RECOVERY ..............
    # ................................

    def recovery(self):
        """
            i have assumed that X amount of time is good enough to go from point A to point B.
            if X amount of time is exceeded and we are still on the road to point B.
            Then robot is stuck therefore reverse.
        """
        # initialize allowed time
        time_allowed = 5 # [secs]
        # cancel the navigation first
        self.stop()
        # Robot is clearing costmaps.
        self.navigator.clearAllCostmaps()
        # Robot hit a dead end, back it up
        self.navigator.backup(backup_dist=0.5, backup_speed=0.1, time_allowance=time_allowed)
        # sleep for the same amount of time + some tolerance.
        time.sleep(time_allowed+1)
        # handle success or failure
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            return True
        else:
            return False

    # ................................
    # ........... DRIVE ..............
    # ................................

    def drive(self, goal_x_y_z_w):
        """
            Go to our destination/goal pose
            return success or failure
            if failure, increment number of retries
        """
        self.navigator.get_logger().info("[plugin]:-drive call recieved-. \n")
        # when did we start navigating?
        self.nav_start = self.navigator.get_clock().now()
        # convert goal to posestamped message for nav2
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav_start.to_msg()
        goal_pose.pose.position.x = float(goal_x_y_z_w[0])
        goal_pose.pose.position.y = float(goal_x_y_z_w[1])
        goal_pose.pose.orientation.z = float(goal_x_y_z_w[2])
        goal_pose.pose.orientation.w = float(goal_x_y_z_w[3])
        # like publish target as single goal.
        # self.navigator.goToPose(goal_pose)
        # self.navigator.followWaypoints([goal_pose])
        # self.navigator.goThroughPoses([goal_pose])
        # while we are enroute
        self.driving = True
        while self.driving:
        # while not self.navigator.isTaskComplete():
            # where is the robot?
            self.navigator.pose_pub.publish(goal_pose)
            time.sleep(0.1)
            # rclpy.spin_once(self.navigator, timeout_sec=10)
            # check interrupt:
            if self.interrupt is True:
                # since we have published a goal earlier and the goal is somewhat
                # unrelated from the instance of navigator. then we need to find a
                # way to cancel the goal. so, publish robot's current pose as target.
                goal_pose.pose.position.x = float(self._agv_position.x)
                goal_pose.pose.position.y = float(self._agv_position.y)
                goal_pose.pose.orientation.z = float(self.quat_z)
                goal_pose.pose.orientation.w = float(self.quat_w)
                self.navigator.pose_pub.publish(goal_pose)
                # the easiest way, would have been to just use navigator instance like
                # self.navigator.followWaypoints([goal_pose]) then cancel/stop would work.
                self.navigator.get_logger().info("[plugin]:-interrupted-. \n")
                self.driving = False
                return self.driving
            # because i used followwaypoints, somehow it percists even after
            # essentially, if we have taken too long and still havent reached our goal...
            if (self.navigator.get_clock().now() - self.nav_start) > Duration(seconds=self.pre_recovery_timeout):
                # increment 'self.num_recovery_retry_counter' since we already failed the actual goal.
                self.recovery_retry_counter += 1
                # in case of a problem, we wanna reverse yeah. or we could spin. doesnt matter.
                result = self.recovery()
                if result is False:
                    # we need not try anymore.
                    self.driving = False
                    return self.driving
            # # if robot is within our softer tolerance limit specified below, go to the next goal.
            d = math.sqrt((float(goal_x_y_z_w[0]) - float(self._agv_position.x))**2 + (float(goal_x_y_z_w[1]) - float(self._agv_position.y))**2)
            # print("[samwise-gamgee]: ----------------------------------------------- ")
            # print("[samwise-gamgee]: champ!! ---------2: ", d)
            if d < self.goal_distance_tol:
                # self.navigator.get_logger().info("[plugin]:-almost home-. \n")
                if (abs(self.quaternion_to_euler(self.quat_x,self.quat_y,self.quat_z,self.quat_w)[-1]) <= \
                    abs(self.quaternion_to_euler(0.0,0.0,goal_x_y_z_w[2],goal_x_y_z_w[3])[-1]) + math.radians(self.goal_angle_tol) and \
                        abs(self.quaternion_to_euler(self.quat_x,self.quat_y,self.quat_z,self.quat_w)[-1]) >= \
                            abs(self.quaternion_to_euler(0.0,0.0,goal_x_y_z_w[2],goal_x_y_z_w[3])[-1]) - math.radians(self.goal_angle_tol)):
                    # please dont forget to reset the retry counter as it signals total task success.
                    self.navigator.get_logger().info("[plugin]:-reached-. \n")
                    self.recovery_retry_counter = 0
                    self.driving = False
                    return True

    # ................................
    # ............ DOCK ..............
    # ................................

    def dock(self):
        """
            Usually there would be some detection here.
            so like [lidar] detects a v-shape and then a controller moves the robot to align with that dock station.
            or like [camera] detects an ar-tag and then a controller moves the robot to the tag and then stops
            here is a simple controller that drives the robot 3m to the front.
            Assumming that every dock location is 3m from the landmarks.
        """
        self.navigator.get_logger().info("[plugin]:-dock call recieved-. \n")
        # distance to move forward
        d = 0.5  # meters
        # calculate new position
        goal_th = self.quaternion_to_euler(0.0, 0.0, self.quat_z, self.quat_w)[-1]
        goal_x = self._agv_position.x + d * math.cos(goal_th)
        goal_y = self._agv_position.y + d * math.sin(goal_th)
        # orientation remains the same
        goal_idx = [goal_x, goal_y, goal_th]
        # initialize cmd_vel msg:
        cmd_vel_msg = Twist()
        # some function variables:
        heading_tolerance = 0.04
        yaw_goal_tolerance = 0.07
        goal_distance_tol = self.goal_distance_tol
        # initialize some temp variable
        goal_achieved = False
        while not goal_achieved:
            # rclpy.spin_once(self.navigator, timeout_sec=10)
            # check interrupt
            if self.interrupt is True:
                # reset interrupt
                return False
            # -------------------------------------
            distance_to_goal = self.get_distance_to_goal(goal_idx)
            heading_error    = self.get_heading_error(goal_idx)
            yaw_goal_error   = self.get_radians_to_goal(goal_idx)
            # -------------------------------------
            # If we are not yet at the position goal
            if (math.fabs(distance_to_goal) > goal_distance_tol):
            # -------------------------------------
            # If the robot's heading is off, fix it
                if (math.fabs(heading_error) > heading_tolerance):
                    if heading_error > 0:
                        cmd_vel_msg.linear.x  = 0.01
                        cmd_vel_msg.angular.z = 0.9
                        # print("forward_left::")
                    else:
                        cmd_vel_msg.linear.x  = 0.01
                        cmd_vel_msg.angular.z = -0.9
                        # print("forward_right::")
                else:
                    cmd_vel_msg.linear.x = 0.35
                    cmd_vel_msg.angular.z = 0.0
                    # print("forward")
            # -------------------------------------
            # Orient towards the yaw goal angle
            elif (math.fabs(yaw_goal_error) > yaw_goal_tolerance):
                if yaw_goal_error > 0:
                    cmd_vel_msg.linear.x  = 0.01
                    cmd_vel_msg.angular.z = 0.9
                    # print("::forward_left")
                else:
                    cmd_vel_msg.linear.x  = 0.01
                    cmd_vel_msg.angular.z = -0.9
                    # print("::forward_right")
            # -------------------------------------
            # Goal achieved, go to the next goal
            else:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.navigator.cmd_vel_pub.publish(cmd_vel_msg)
                goal_achieved = True
                self.navigator.get_logger().info("[plugin]:----------dock completed---------. \n")
                return goal_achieved

            cmd_vel_msg.angular.z = cmd_vel_msg.angular.z / 9 # turn speed reduction
            self.navigator.cmd_vel_pub.publish(cmd_vel_msg)  # Publish the velocity message | vx: meters per second | w: radians per second
            # self.navigator.get_logger().info(" [plugin] goal_dist, heading_err, yaw_err : ["+str([distance_to_goal, heading_error, yaw_goal_error])+"]. \n")

    # ................................
    # ............ UNDOCK ............
    # ................................

    def undock(self, reverse_goal_x, reverse_goal_y, reverse_goal_z, reverse_goal_w):
        """ undock robot """
        self.navigator.get_logger().info("[plugin]:-undock called-. \n")
        reverse_goal_th = self.quaternion_to_euler(0.0,0.0,reverse_goal_z,reverse_goal_w)[-1]
        # set the undock or reverse target
        goal_idx = [reverse_goal_x, reverse_goal_y, reverse_goal_th]
        # initialize cmd_vel msg:
        cmd_vel_msg = Twist()
        # some function variables:
        heading_tolerance = 0.04
        yaw_goal_tolerance = 0.07
        goal_distance_tol = self.goal_distance_tol
        # initialize some temp variable
        goal_achieved = False
        while not goal_achieved:
            # rclpy.spin_once(self.navigator, timeout_sec=10)
            # check interrupt
            if self.interrupt is True:
                # reset interrupt
                return False
            # -------------------------------------
            distance_to_goal = self.get_distance_to_goal(goal_idx)
            heading_error    = self.get_heading_error(goal_idx, docked=True)
            yaw_goal_error   = self.get_radians_to_goal(goal_idx)
            # -------------------------------------
            # If we are not yet at the position goal
            if (math.fabs(distance_to_goal) > goal_distance_tol):
            # -------------------------------------
            # If the robot's heading is off, fix it
                if (math.fabs(heading_error) > heading_tolerance):
                    if heading_error > 0:
                        cmd_vel_msg.linear.x  = -0.01
                        cmd_vel_msg.angular.z = 0.9
                        # print("backward_right::")
                    else:
                        cmd_vel_msg.linear.x  = -0.01
                        cmd_vel_msg.angular.z = -0.9
                        # print("backward_left::")
                else:
                    cmd_vel_msg.linear.x = -0.35
                    cmd_vel_msg.angular.z = 0.0
                    # print("backward")
            # -------------------------------------
            # Orient towards the yaw goal angle
            elif (math.fabs(yaw_goal_error) > yaw_goal_tolerance):
                if yaw_goal_error > 0:
                    cmd_vel_msg.linear.x  = -0.01
                    cmd_vel_msg.angular.z = 0.9
                    # print("::backward_right")
                else:
                    cmd_vel_msg.linear.x  = -0.01
                    cmd_vel_msg.angular.z = -0.9
                    # print("::backward_left")
            # -------------------------------------
            # Goal achieved, go to the next goal
            else:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.navigator.cmd_vel_pub.publish(cmd_vel_msg)
                goal_achieved = True
                self.navigator.get_logger().info("[plugin]:-undock completed-. \n")
                return goal_achieved

            cmd_vel_msg.angular.z = cmd_vel_msg.angular.z / 9 # turn speed reduction
            self.navigator.cmd_vel_pub.publish(cmd_vel_msg)  # Publish the velocity message | vx: meters per second | w: radians per second
            # self.navigator.get_logger().info(" [plugin] goal_dist, heading_err, yaw_err : ["+str([distance_to_goal, heading_error, yaw_goal_error])+"]. \n")

    # ................................
    # .......... IO STATE ............
    # ................................

    def state_io(self, type):
        """ state check on success of reaching a destination or checkpoint
            type: dock_required | undock_required | elevator_required | waypoint
        """
        # wait for some stuff to trigger
        # could be a door etc.
        # implement your lidar field messages here or something, anything.
        # whatever it is IO
        # like if type == 'dock_required':
        #       check if lift is up:
        #       if lift is up
        #           return True

        # or like if type == 'elevator_required'
        #       check if door is open
        #       if door is open
        #           return true

        # etc.

        # Please NOTE that the code will not move forward unless this io_state
        # returns true. because it is a while io_state blah blah blah.
        print("[state_io]:::::::::::::::::::")
        time.sleep(0.1)
        # assume it has triggered
        if self.io_state is True:
            # do some stuff
            return True
        elif self.io_state is False:
            # do some more stuff
            # see if those return true
            return False

    # ................................
    # ............ EXIT ..............
    # ................................

    def on_exit(self):
        """ exit or kill plugin node """
        # self.node_.destroy_node()
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
