#!/usr/bin/env python3

import psycopg2, psycopg2.extras
import time, yaml, os, math, re, sys, datetime
from pathlib import Path

from FmTaskHandler import FmTaskHandler

from dataclasses import dataclass
from typing import Any

@dataclass
class RobotContext:
    agv_position: Any = None
    agv_status: Any = None
    errors: Any = None
    horizon: Any = None
    horizon_release: Any = None
    base: Any = None
    checkpoints: Any = None
    waitpoints: Any = None
    landmarks: Any = None
    checkp_itinerary: Any = None
    waitp_itinerary: Any = None
    active_map_name: Any = None
    order_id: Any = None
    header_id: Any = None
    order_timestamp: Any = None
    merged_nodes: Any = None
    dock_action_done: Any = None
    pending_dock_action: Any = None
    halt: Any = None
    speed_min: Any = None
    agv_size: Any = None
    vel_lin_ang: Any = None
    waitpoints_release: Any = None



class FmTrafficHandler():
    """ task traffic handler """
    def __init__(self, fleetname, version, versions, manufacturer,
                dbconn, mqttclient=None, task_dict=None):

        self.version = version # '1.0.0'
        self.manufacturer = manufacturer # 'birfen'
        self.fleetname = fleetname # 'kullar'
        self.versions = versions # 'v1'

        # Initialize the FmTaskHandler
        self.task_handler = FmTaskHandler(
            self.fleetname,
            self.version,
            self.versions,
            self.manufacturer,
            dbconn,
            mqttclient,
            task_dict)

        # initialize the task network dictionary
        self.task_dictionary = task_dict if task_dict else {}

        self.temp_robot_delay_time = {}
        self.wait_time_default = 10.5 # [secs]
        self.mutex_groups = [] #  ['C10', 'C11', 'C3'] Store list of lists e.g. [['C1', 'C2'], ['C10', 'C11']]
        self.collision_tracker = 0
        self.robots_in_collision = set()

        # online_robots lives here (traffic management layer) not in StateSubscriber (data layer).
        # Semantics: "robot is a live participant in the current fleet session".
        # Populated by FmMain.on_mqtt_message whenever a state message arrives.
        self.online_robots: set = set()  # r_ids that have published at least one state

    # --------------------------------------------------------------------------------------------

    def fm_add_mutex_groups(self, group_list):
        """ Add a new mutex group (list of nodes) to the handler. """
        if isinstance(group_list, list) and group_list not in self.mutex_groups:
            self.mutex_groups.append(group_list)
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"Added Mutex Group: {group_list}",
                "FmTrafficHandler",
                "fm_add_mutex_groups",
                "info")

    def fm_remove_mutex_group(self, group_list):
        """ Remove an existing mutex group from the handler. """
        if group_list in self.mutex_groups:
            self.mutex_groups.remove(group_list)
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"Removed Mutex Group: {group_list}",
                "FmTrafficHandler",
                "fm_remove_mutex_group",
                "info")

    # --------------------------------------------------------------------------------------------

    def euler_to_quaternion(self, yaw, pitch, roll):
        """
        Convert Euler angles (yaw, pitch, roll) to quaternion (qx, qy, qz, qw).
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    # --------------------------------------------------------------------------------------------

    def process_maps(self, maps):
        """ Process maps to get active map and extract map IDs """
        active_map = None
        for i, map_item in enumerate(maps):
            if map_item["mapStatus"] == "ENABLED":
                active_map = map_item["mapId"]
        return active_map

    # --------------------------------------------------------------------------------------------

    def get_map(self, f_id_, b_node, h_node):
        """ fetch new active map from base node or reserved node """
        # we know with some certainty that elevator nodes dont have associated wait nodes.
        # so C or W, we use map_name directly.
        temp_map_name = self.task_handler.get_node_map(f_id_, b_node)
        # check if i am in an elevator node, then switch map or get next node's map.
        temp_node_description = self.task_handler.get_node_description(f_id_, b_node)
        if temp_node_description == 'elevator':
            temp_map_name = self.task_handler.get_node_map(f_id_, h_node)
        return temp_map_name

    # --------------------------------------------------------------------------------------------

    def extract_wait_time(self, node_description):
        """ custom function to extract wait time information if it exists. """
        # Check if 'Wait Time' is in the node_description
        if 'Wait Time' in node_description:
            # Use regular expression to extract the value of Wait Time
            match = re.search(r"Wait Time: ([\d\.]+|None)", node_description)
            if match:
                wait_time_value = match.group(1)
                # Check if the extracted value is 'None' or a float
                if wait_time_value == 'None':
                    return None
                else:
                    try:
                        # Try converting the value to a float
                        return float(wait_time_value)
                    except ValueError:
                        # In case the value cannot be converted to float
                        return None
            else:
                # If regex didn't match the pattern, return None
                return None
        else:
            # If 'Wait Time' is not in node_description
            return None

    # --------------------------------------------------------------------------------------------

    def map_priority(self, priority):
        """
        Parameters
            priority (str): The priority string (e.g., 'low', 'medium', 'high').
        Logic
            Uses a dictionary to map priority strings to corresponding integer values.
            Returns (int): Integer representation of the priority.
        """
        mapping = {'low': 1, 'medium': 2, 'high': 3}
        return mapping.get(priority, 0)  # Default to 0 if priority is not recognized

    # --------------------------------------------------------------------------------------------

    def check_waitpoint_association(self, node, mex_waitpoints):
        """
        Given a node, extract the numeric part, create a waitpoint identifier by appending 'W',
        and check if it is present in the mex_waitpoints list. for example;
        record = {'waitpoints': 'W8,W11,W12,W1,W4,W7,W6'}
        mex_waitpoints = str(record['waitpoints']).split(','); node = 'A5'
        Args:
        - node (str): The node identifier (e.g., 'A5').
        - mex_waitpoints (list): List of waitpoints to check against.
        - returns: waitpoint or none.
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

    def estimate_time_to_node(self, robot_pos, node_pos, min_vel):
        """
        Estimate the time taken for a robot to reach a given node.
        Args:
        - robot_pos (tuple): The current position of the robot (x, y, theta).
        - node_pos (tuple): The position of the node (x, y, z, w).
        - min_vel (float): The linear velocity of the robot.
        Logic
            Calculates the Euclidean distance between the robot and the node.
            Estimates the time to reach the node based on the robot's velocity.
            Returns a default value if the estimated time is too short.
            That is, (float) Estimated time to reach the node.
        """
        # Extract coordinates from robot position and node position
        robot_x, robot_y = float(robot_pos[0]), float(robot_pos[1])
        node_x, node_y = float(node_pos[0]), float(node_pos[1])
        # Calculate the Euclidean distance between the robot and the node
        distance = math.sqrt((node_x - robot_x) ** 2 + (node_y - robot_y) ** 2)
        # Estimate time to reach the node
        if min_vel == 0.0:
            return float('inf')  # If velocity is zero, time is infinite
        time_to_reach = distance / min_vel
        if time_to_reach <= 0.1:
            time_to_reach = self.wait_time_default
        return time_to_reach

    # --------------------------------------------------------------------------------------------

    def check_robot_arrival(self, eta_timestamp):
        """
        Check if the robot has arrived at the node on time.
        Args:
        - est_arrival_time (datetime): The estimated arrival time to the node.
        """
        # Parse the order timestamp into a datetime object
        est_arrival_time = datetime.datetime.fromisoformat(str(eta_timestamp))
        current_time = datetime.datetime.now()
        return current_time > est_arrival_time

    # --------------------------------------------------------------------------------------------

    def calculate_estimated_arrival_time(self, estimated_seconds):
        """
        Calculate the estimated arrival time based on the current timestamp and the estimated travel time.
        Args:
        - estimated_seconds (float): Estimated travel time in seconds.
        """
        return datetime.datetime.now() + datetime.timedelta(seconds=float(estimated_seconds))

    # --------------------------------------------------------------------------------------------

    def _extract_eta_from_description(self, description):
        """
        Extracts the ETA from the node description.
        Args:
        - description (str): The node description containing the ETA.
        """
        try:
            # Assuming ETA is always the last part of the description after "Node ETA: "
            eta_part = description.split("Node ETA: ")[-1]
            if eta_part and eta_part != 'None':
                return eta_part
        except (IndexError, ValueError):
            pass
        return None

    # --------------------------------------------------------------------------------------------

    def check_minute_passed(self, order_timestamp, minuite):
        """
        This function checks if one minute has passed since the order was issued.

        Args:
            order_timestamp (str): The timestamp of the order in ISO format (YYYY-MM-DDTHH:MM:SS.ssssss+ZZZZ).

        Returns:
            bool: True if one minute has passed, False otherwise.
        """
        # check if input format is correct
        if not order_timestamp:
            return False  # Handle None or empty string

        try:
            # Parse the order timestamp into a datetime object
            order_datetime = datetime.datetime.fromisoformat(str(order_timestamp))

            # Get the current datetime
            current_datetime = datetime.datetime.now()

            # Calculate the time difference in seconds
            time_delta = current_datetime - order_datetime
            seconds_passed = time_delta.total_seconds()

            # Check if x minute (x * 60 seconds) has passed
            return seconds_passed >= (minuite * 60)
        except ValueError:
            # log viz: Handle invalid timestamp format
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"Invalid timestamp format: {order_timestamp}.",
                "FmTrafficHandler",
                "check_minute_passed",
                "error")
            return False

    # --------------------------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------



    def manage_traffic(self, f_id=None, r_id=None, m_id=None, v_id=None):
        """
        Inputs:
            f_id: Fleet ID for operations.
        Details:
            should be called in a loop.
            Checks if f_id is provided and if there are any serial numbers to process.
            Locks the timer to prevent concurrent execution.
            Iterates over serial numbers to fetch and process current robot data, update traffic control, and handle robot traffic status.
        """
        traffic_control = None
        unassigned = None
        ctx = None

        f_id = f_id or self.fleetname
        m_id = m_id or self.manufacturer
        v_id = v_id or self.version

        if r_id is None:
            return traffic_control, unassigned, ctx

        try:
            print(" ")
            # _fetch_current_robot_data creates and returns ctx.
            # Only call _handle_robot_traffic_status if the fetch succeeded.
            # If ctx.agv_position is still None, the DB fetch failed transiently
            # and we must skip — otherwise we'd act on the previous robot's stale state.
            traffic_control, unassigned, mex_record, ctx = self._fetch_current_robot_data(f_id, r_id, m_id)
            if ctx is not None and ctx.agv_position is not None:
                self._handle_robot_traffic_status(f_id, r_id, m_id, v_id, traffic_control, mex_record, ctx)
        except (ValueError, TypeError) as error:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"{error}..",
                "FmTrafficHandler",
                "manage_traffic",
                "info")
            return traffic_control, unassigned, ctx

        return traffic_control, unassigned, ctx

    # --------------------------------------------------------------------------------------------

    def _fetch_current_robot_data(self, f_id, _r_id, m_id):
        """
        Inputs:
            f_id: Fleet ID to query.
            r_id: Robot ID to query.
        Logic:
            Fetch and update data for a specific robot.
            Executes a SQL query to fetch robot data from the database based on the fleet and robot IDs.
            Maps the fetched data to corresponding class variables, including:
                Serial number, maps, order ID, last node ID, driving and paused states, node and edge states, position, velocity, battery state, errors, and additional information.
            Processes special AGV information such as active map, waypoints, status, and configuration.
            Handles exceptions if there is a database error or no data is found.
        """
        # Always reset first — prevents stale data from a prior robot (or prior
        # successful cycle) from surviving into this robot's failed-fetch path.


        fb_rec = self.fetch_mex_data(f_id, r_id=_r_id, m_id=m_id)

        try:
            # write to class variables for active robot params
            # ctx.version = fb_rec["version"]
            # ctx.manufacturer = fb_rec["manufacturer"]
            # ctx.serial_number = serial_number
            # ctx.fleet_name = information.get('fleet_name', 'none')
            ctx = RobotContext(
                agv_position=fb_rec["robot_data"]["position"],
                base=fb_rec["robot_data"]["base"],
                horizon=fb_rec["robot_data"]["horizon"],
                horizon_release=fb_rec["robot_data"]["horizon_release"],
                checkpoints=fb_rec["robot_data"]["c_pnts"],
                waitpoints=fb_rec["robot_data"]["w_pnts"],
                checkp_itinerary=fb_rec["robot_data"]["c_pnts_pos"],
                waitp_itinerary=fb_rec["robot_data"]["w_pnts_pos"],
                waitpoints_release=fb_rec["robot_data"].get("w_pnts_release", []),
                agv_status=fb_rec["robot_data"]["agv_status"],
                order_timestamp=fb_rec["robot_data"]["order_timestamp"],
                merged_nodes=fb_rec["robot_data"]["merged_nodes"],
                dock_action_done=fb_rec["robot_data"]["dock_action_done"],
                pending_dock_action=fb_rec["robot_data"].get("pending_dock_action", None),
                halt=fb_rec["robot_data"]["halt"],
                errors=fb_rec["robot_data"]["errors"],
                active_map_name=fb_rec["robot_data"]["active_map"],
                landmarks=fb_rec["robot_data"]["dock"],
                order_id=fb_rec["robot_data"]["order_id"],
                header_id=fb_rec["robot_data"]["header_id"],
                agv_size=[fb_rec["robot_data"]["agv_size"][0], fb_rec["robot_data"]["agv_size"][1]],
                speed_min=fb_rec["robot_data"]["speed_min"],
                vel_lin_ang=[fb_rec["robot_data"]["lin_vel"], fb_rec["robot_data"]["ang_vel"]]
            )

            mex_record = fb_rec["mex_data"]

            self.last_traffic_dict = fb_rec["traffic_control"]
            traffic_ = set([n for nodes in self.last_traffic_dict.values() for n in nodes]) if isinstance(self.last_traffic_dict, dict) else set(self.last_traffic_dict)
            unassigned = fb_rec["unassigned"]
        except KeyError as key_err:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"Missing key {key_err} in feedback record.",
                "FmTrafficHandler",
                "_fetch_current_robot_data",
                "warn")
            return [], [], {}, None
        except (TypeError, ValueError) as specific_err:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"Type or Value Error: {specific_err}.",
                "FmTrafficHandler",
                "_fetch_current_robot_data",
                "warn")
            return [], [], {}, None
        return traffic_, unassigned, mex_record, ctx

    # --------------------------------------------------------------------------------------------

    def _handle_robot_traffic_status(self, f_id, _r_id, m_id, v_id, traffic_control, mex_record, ctx):
        """
        Manage robot traffic status by verifying next_stop occupancy and handling conflicts.

        Parameters:
            f_id (str): Fleet ID.
            _r_id (str): Robot ID.
            m_id (str): Map ID.
            v_id (str): Vehicle ID.
            traffic_control (list): List of currently reserved nodes.

        Process:
            1. Logs the current traffic control state.
            2. If no horizon is available, returns immediately.
            3. Determines the base coordinate using the reserved checkpoint,
            selecting from either the waitpoint or checkpoint itinerary.
            4. Extracts the next stop ID, its release status, and its coordinate.
            5. Calculates the distance to the base coordinate.
            6. Checks if sufficient time has passed since order issuance.
            7. Depending on robot status and proximity:
                a. If the robot is in conflict with traffic and conditions are met,
                it handles the conflict either as a last-mile or waiting conflict.
                b. If there is no conflict, it proceeds with a no-conflict action.
        """
        if not ctx.horizon:
            return

        # The current robot's own base node — needed before mutex expansion.
        reserved_checkpoint = ctx.base

        # -----------------------------------------------------------------------
        # MUTEX GROUP EXPANSION
        # A mutex group means: if robot X occupies any node in the group, all
        # other nodes in that group are blocked for OTHER robots.
        # Bug fix: the expansion must NOT trigger on the current robot's own base.
        # Without this guard, the robot would add its own sibling nodes to
        # traffic_control, then reject its own next_stop as "occupied".
        #
        # Rule: expand a group's siblings into traffic_control only if at least one
        # group node is occupied by a robot OTHER than the current one.
        # "Other robot" = node is in traffic_control AND it is not reserved_checkpoint.
        # -----------------------------------------------------------------------

        # traffic_control is now an O(1) set of all occupied nodes
        active_traffic_nodes = traffic_control
        
        expanded_traffic = list(active_traffic_nodes)
        for group in self.mutex_groups:
            occupied_by_other = any(
                node in active_traffic_nodes and node != reserved_checkpoint
                for node in group
            )
            if occupied_by_other:
                for node in group:
                    if node not in expanded_traffic:
                        expanded_traffic.append(node)
        
        # We pass expanded_traffic down for simple node occupancy checks
        checking_traffic_control = expanded_traffic

        # -----------------------------------------------------------------------
        
        # Determine base coordinate based on reserved checkpoint type
        base_coordinate = self._get_base_coordinate(reserved_checkpoint, ctx)

        # Retrieve next stop details
        next_stop_id = ctx.horizon[0]
        next_stop_id_release = ctx.horizon_release[0]

        idx = ctx.checkpoints.index(next_stop_id)
        next_stop_coordinate = ctx.checkp_itinerary[idx]

        # Calculate distance from current position to base coordinate
        dist_to_base = self._calculate_distance(
            ctx.agv_position[0], ctx.agv_position[1],
            base_coordinate[0], base_coordinate[1]
        )

        has_order_minute_passed = self.check_minute_passed(ctx.order_timestamp, 0.25)

        # Log current status (critical information)
        # log viz:
        self.task_handler.visualization_handler.terminal_log_visualization(
            f"\nR_id: {_r_id}. \n"
            f"Reserved_checkpoint: {reserved_checkpoint}, "
            f"Next_stop_id: {next_stop_id}, "
            f"Moving: {ctx.agv_status}  \n"
            f"Horizon: {ctx.horizon}, \n"
            f"Dist_to_base: {dist_to_base}, "
            f"has_order_minute_passed: {has_order_minute_passed}, "
            f"temp_fb_dock_action_done: {ctx.dock_action_done}, "
            f"isCheckpoint: {(ctx.agv_status == 'red' and reserved_checkpoint not in ctx.landmarks)}, "
            f"isStationandDocked: {(ctx.agv_status == 'red' and reserved_checkpoint in ctx.landmarks and ctx.dock_action_done)}.",
            "FmTrafficHandler",
            "_handle_robot_traffic_status",
            "info")

        try:
            # Define primary conditions for node reservations and conflict resolutions.
            is_ready = ctx.agv_status == 'red'
            is_safe = (reserved_checkpoint not in ctx.landmarks) or \
                (reserved_checkpoint in ctx.landmarks and ctx.dock_action_done)
            is_within_range = dist_to_base <= (0.8 * float(ctx.agv_size[1]))
            is_different_stop = reserved_checkpoint != next_stop_id
            conditions_met = is_ready and is_safe and is_different_stop and \
                            (not next_stop_id_release) and has_order_minute_passed and \
                            (not ctx.halt) and is_within_range

            if conditions_met:
                # Handle special waitpoint case
                # check if we are having a waitpoint case. a special case wx cx cy.
                if ctx.merged_nodes[0].startswith('W'):
                    self._handle_waitpoint_case(f_id, _r_id, m_id, v_id, ctx)
                    return

                # Check if next stop is occupied in traffic control
                if next_stop_id in checking_traffic_control:
                    # log viz:
                    self.task_handler.visualization_handler.terminal_log_visualization(
                        f"Robot {_r_id}: next_stop_id {next_stop_id} occupied.",
                        "FmTrafficHandler",
                        "_handle_robot_traffic_status",
                        "info"
                    )

                    # normal plan: last mile dock is either single home dock if transport
                    # or single charge or home dock directly if move or charge.
                    # Determine conflict resolution method based on last-mile conditions
                    if (next_stop_id in ctx.landmarks and
                        len(ctx.horizon) == 1 and
                        ctx.landmarks[2] != 'loop'):
                        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                        # LOGIC FOR SKIPPING LAST MILE NODES!!!
                        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                        self._handle_last_mile_conflict_case(f_id, _r_id, m_id, v_id,
                                                            reserved_checkpoint, next_stop_id,
                                                            checking_traffic_control, ctx)
                    else:
                        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                        # _handle_waiting_robot_conflict_case
                        # Only negotiate if we know who the occupant is.
                        # mex_record is {} when the occupying robot's state message hasn't
                        # arrived yet — it's in traffic_control (position known from a
                        # previous cycle) but not yet in robot_states (cache entry absent).
                        # Calling _handle_active_mex_conflict with an empty dict causes
                        # KeyError: 'version'. Correct behaviour: hold position and wait
                        # for the mex robot's next state message before negotiating.
                        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                        if not mex_record.get("serial_number"):
                            self.task_handler.visualization_handler.terminal_log_visualization(
                                f"{_r_id}: occupant of {next_stop_id} is not yet in cache. Holding position.",
                                "FmTrafficHandler",
                                "_handle_robot_traffic_status",
                                "info")
                        else:
                            self._handle_active_mex_conflict(f_id, _r_id, m_id, v_id,
                                                            next_stop_id, checking_traffic_control,
                                                            reserved_checkpoint,
                                                            next_stop_coordinate,
                                                            mex_record, ctx)

                else:
                    self._handle_no_conflict_case(f_id, _r_id, m_id, v_id, ctx)

            elif (is_ready and not is_different_stop and
                (not next_stop_id_release) and has_order_minute_passed and
                (not ctx.halt) and is_within_range):
                # log viz:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"Robot {_r_id} occupies its target node. Converting to base.",
                    "FmTrafficHandler",
                    "_handle_robot_traffic_status",
                    "info"
                )
                self._handle_no_conflict_case(f_id, _r_id, m_id, v_id, ctx)

            elif (is_ready and (not next_stop_id_release) and has_order_minute_passed and
                  (not ctx.halt) and is_within_range and
                  (reserved_checkpoint in ctx.landmarks) and
                  not ctx.dock_action_done):
                # log viz:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"Robot {_r_id} at destination but yet to dock. Pinging...",
                    "FmTrafficHandler",
                    "_handle_robot_traffic_status",
                    "info"
                )
                self._handle_dock_reminder(f_id, _r_id, m_id, v_id, ctx)

        except (ValueError, TypeError) as error:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"Error: {error}",
                "FmTrafficHandler",
                "_handle_robot_traffic_status",
                "warn"
            )


    # --------------------------------------------------------------------------------------------

    def _get_base_coordinate(self, reserved_checkpoint, ctx):
        """
        Determine base coordinate based on the reserved checkpoint.

        If reserved_checkpoint is a waitpoint (starts with 'W'), select the corresponding coordinate
        from the waitpoint itinerary; otherwise, use the checkpoint itinerary.
        """
        if reserved_checkpoint:
            if reserved_checkpoint.startswith('W'):
                try:
                    idx = ctx.waitpoints.index(reserved_checkpoint)
                    return ctx.waitp_itinerary[idx]
                except ValueError:
                    pass  # Fallback to default if not found
            else:
                try:
                    idx = ctx.checkpoints.index(reserved_checkpoint)
                    return ctx.checkp_itinerary[idx]
                except ValueError:
                    pass
        # Default coordinate if reserved_checkpoint is None or not found
        return [ctx.agv_position[0], ctx.agv_position[1]]

    # --------------------------------------------------------------------------------------------

    def _handle_dock_reminder(self, f_id, _r_id, m_id, v_id, ctx):
        """
        Pings a pending instant action (e.g., 'dock') to a robot that is at a node 
        but hasn't completed the action yet, maintaining VDA 5050 idempotency by
        using the EXACT original order action dictionary.
        """
        if not ctx.pending_dock_action:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"Robot {_r_id} at destination but no pending action found to ping.",
                "FmTrafficHandler",
                "_handle_dock_reminder",
                "warn"
            )
            return

        action_list = [ctx.pending_dock_action]
        # Use a new header id for the new message wrapper, but keep the actionId identical
        header_id = ctx.header_id + 1
        self.task_handler.instant_actions_handler.build_instant_action_msg(f_id, _r_id, header_id, v_id, m_id, action_list)
        print("Dock action pinged to robot: {} with original actionId: {}".format(_r_id, ctx.pending_dock_action.get("actionId")))

    # --------------------------------------------------------------------------------------------

    def _handle_node_action(self, f_id, _r_id, m_id, v_id, header_id, base, dock_action_done, landmark):
        """
        this actions are called after docking action is completed. like pick place etc.
        Logic:
            action states | :---: | 'INITIALIZING', 'RUNNING', 'PAUSED', 'FINISHED', 'FAILED'
            ------------------
            startPause | - | Activation of the mode is in preparation.<br>If the AGV supports an instant transition, this state can be omitted.
                        | - | Vehicle stands still. <br>All actions will be paused. <br>The pause mode is activated. <br>The AGV reports .paused: "true".
                        | The pause mode cannot be activated for some reason (e.g., overridden by hardware switch).
            stopPause | - | Deactivation of the mode is in preparation. <br>If the AGV supports an instant transition, this state can be omitted.
                        | - | The pause mode is deactivated. <br>All paused actions will be resumed. <br>The AGV reports .paused: "false".
                        | The pause mode cannot be deactivated for some reason (e.g., overwritten by hardware switch).
            startCharging | - | Activation of the charging process is in progress (communication with charger is running). <br>If the AGV supports an instant transition, this state can be omitted.
                        | - | The charging process is started. <br>The AGV reports .batteryState.charging: "true".
                        | The charging process could not be started for some reason (e.g., not aligned to charger). Charging problems should correspond with an error.
            pick | Initializing of the pick process, e.g., outstanding lift operations.
                | The pick process is running (AGV is moving into station, load handling device is busy, communication with station is running, etc.).
                | The pick process is being paused, e.g., if a safety field is violated. <br>After removing the violation, the pick process continues.
                | Pick is done. <br>Load has entered the AGV and AGV reports new load state.
                | Pick failed, e.g., station is unexpected empty. <br> Failed pick operations should correspond with an error.
            drop | Initializing of the drop process, e.g., outstanding lift operations.
                | The drop process is running (AGV is moving into station, load handling device is busy, communication with station is running, etc.).
                | The drop process is being paused, e.g., if a safety field is violated. <br>After removing the violation the drop process continues.
                | Drop is done. <br>Load has left the AGV and AGV reports new load state.
                | Drop failed, e.g., station is unexpected occupied. <br>Failed drop operations should correspond with an error.
            waitForTrigger | - | AGV has to wait for a trigger on the AGV (e.g., button press, manual loading).
              <br>Master control is responsible to handle the timeout and has to cancel the order if necessary.
              | yes | triggerType(string) | - | no | yes | no
        """
        # Determine if the node is a dock node (e.g., C5, C4, etc.)
        is_dock_node = any(base == dock for dock in landmark)
        # Ensure we have enough landmark info
        if is_dock_node and dock_action_done:
            # Add pick or drop action based on task_name and node_id
            task_name = landmark[2]
            if task_name in ["transport", "loop"]:
                # get landmark constituents = [payload_kg, task_priority, task_name, from_loc_id, to_loc_id] + [home_dock_loc_ids]
                from_loc_id = landmark[3]
                to_loc_id = landmark[4]
                if base == from_loc_id:  # Pick action
                    # "approx. time for picking up the load"
                    pick_action_params = {
                        "key": "duration",
                        "value": "2.0"
                    }
                    pick_action = self.task_handler.instant_actions_handler.create_action("pick", pick_action_params)
                    action_list = [pick_action]
                    self.task_handler.instant_actions_handler.build_instant_action_msg(f_id, _r_id, header_id, v_id, m_id, action_list)
                elif base == to_loc_id:  # drop action
                    # "approx. time for dropping up the load"
                    drop_action_params = {
                        "key": "duration",
                        "value": "2.0"
                    }
                    drop_action = self.task_handler.instant_actions_handler.create_action("drop", drop_action_params)
                    action_list = [drop_action]
                    self.task_handler.instant_actions_handler.build_instant_action_msg(f_id, _r_id, header_id, v_id, m_id, action_list)
            elif task_name in ["charge"]:
                # get landmark constituents = [payload_kg, task_priority, task_name, to_loc_id] + [charge_dock_loc_ids_temp]
                if base in landmark[3:]:  # charge action
                    # "approx. time for startCharging"
                    charge_action_params = {
                        "key": "duration",
                        "value": "2200.0"
                    }
                    charge_action = self.task_handler.instant_actions_handler.create_action("startCharging", charge_action_params)
                    action_list = [charge_action]
                    self.task_handler.instant_actions_handler.build_instant_action_msg(f_id, _r_id, header_id, v_id, m_id, action_list)
        else:
            node_description = self.task_handler.get_node_description(f_id, base)
            if node_description == 'door':
                trigger_action_params = {
                    "key": "duration",
                    "value": "2.0"
                }
                trigger_action = self.task_handler.instant_actions_handler.create_action("waitForTrigger", trigger_action_params)
                action_list = [trigger_action]
                self.task_handler.instant_actions_handler.build_instant_action_msg(f_id, _r_id, header_id, v_id, m_id, action_list)

    # --------------------------------------------------------------------------------------------

    def _handle_waitpoint_case(self, f_id, _r_id, m_id=None, v_id=None, ctx=None):
        """
        Parameters:
            _r_id (str): Robot ID.
            f_id (str): Fleet ID.
            next_stop_id (str): Identifier for the next stop node to be reserved.
        Logic:
            Since no traffic conflicts, reset the robot's delay time to '0' and sets its status flag to 'green'.
            Creates action objects for the next stop ID to be reserved and traffic signal.
            Inserts the decision to the instant action database.
        """
        self.temp_robot_delay_time[_r_id] = (0,0)  # Reset delay time | Update flag green

        # log viz:
        self.task_handler.visualization_handler.terminal_log_visualization(
            f"{_r_id} wait case completed scenario. green.",
            "FmTrafficHandler",
            "_handle_waitpoint_case",
            "info")

        try:
            # get new active map
            temp_map_name = self.get_map(f_id, ctx.checkpoints[0], ctx.horizon[0])
            # ensure that map name is different from current map
            # Only compare maps if we already know the active map.
            # On startup, temp_fb_active_map_name is None because the robot
            # hasn't received a downloadMap yet — comparing None to a map_id
            # would always trigger a false 'elevator' warning every cycle.
            if ctx.active_map_name is not None and ctx.active_map_name != temp_map_name:
                # then we have a problem
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"r_id: {_r_id} is in an elevator but can not fetch new map.",
                    "FmTrafficHandler",
                    "_handle_waitpoint_case",
                    "warn")
            # estimate goal nodes time of arrival
            temp_fb_node_eta = self.calculate_estimated_arrival_time(self.wait_time_default)
            # prepare the order update message
            header_id = ctx.header_id + 1
            order_id, update_id = self.task_handler.generate_new_order_id(ctx.order_id)
            b_node, h_nodes, h_edges = self.task_handler.order_handler.create_order(ctx.checkpoints[1:], # = [ctx.base]+ctx.horizon, # ctx.checkpoints, # checkpoints
                                                            ctx.waitpoints, # waitpoints,
                                                            ctx.checkp_itinerary[1:], # agv_itinerary,
                                                            ctx.waitp_itinerary, # wait_itinerary,
                                                            ctx.landmarks, # landmark)
                                                            temp_map_name, # active map for which this nav will be performed
                                                            [ctx.base]+ctx.horizon, # release (next_stop_id)
                                                            temp_fb_node_eta) # time of arrival at the newly released node
            self.task_handler.order_handler.build_order_msg(f_id,
                                                _r_id,
                                                header_id,
                                                v_id,
                                                m_id,
                                                b_node,
                                                h_nodes,
                                                h_edges,
                                                order_id,
                                                update_id)
        except (ValueError, TypeError) as error:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"{error}.",
                "FmTrafficHandler",
                "_handle_waitpoint_case",
                "error")

    # --------------------------------------------------------------------------------------------

    def check_available_last_mile_dock(self, reserved_checkpoint, traffic_control,
                                       task_dict, start_idx):
        """Handle dock availability and path planning."""
        available_last_mile_dock = False

        graph = self.task_handler.build_graph(task_dict)

        for dock_node in ctx.landmarks[start_idx:]:
            # skip it, since its occupied | 'green'
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"checking docks {ctx.landmarks[start_idx:]} for availability.",
                "FmTrafficHandler",
                "check_available_last_mile_dock",
                "info")

            # check which dock is available amongst the last mile docks.
            if dock_node not in traffic_control:
                # plan to it to obtain checkpoints, checkp_itinerary, waitpoints and its itinerary as well.
                paths = self.task_handler.fm_shortest_paths(reserved_checkpoint, dock_node, graph)
                # log viz:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"start {reserved_checkpoint}, target {dock_node}, paths --> {paths}.",
                    "FmTrafficHandler",
                    "check_available_last_mile_dock",
                    "info")
                if paths:
                    if len(paths) != 0 and paths[0]:
                        ctx.checkpoints = paths[0]
                        ctx.checkp_itinerary = self.task_handler.fm_get_itinerary(ctx.checkpoints, task_dict)
                        ctx.waitpoints = self.task_handler.fm_extract_unique_waitpoints(ctx.checkpoints, task_dict)
                        ctx.waitp_itinerary = self.task_handler.fm_get_itinerary(ctx.waitpoints, task_dict)
                        ctx.horizon = ctx.checkpoints # [1:]
                        available_last_mile_dock = True
        return available_last_mile_dock

    # --------------------------------------------------------------------------------------------

    def _handle_last_mile_conflict_case(self, f_id, _r_id, m_id, v_id, reserved_checkpoint,
                                        next_stop_id, traffic_control, task_dict=None, ctx=None):
        """
        # 1. is this next node id occupied?
        # 2. is next node id a home or charge dock?
        # 3. is there more nodes after this particular node?
        # 4. which one of them is not occupied?
        # 5. can you find a path from where you are now to there?
        # 6. send route to robot
        """

        task_dict = task_dict or self.task_dictionary

        available_last_mile_dock = False

        # normal plan: last mile dock is either single home dock if transport
        # or single charge or home dock directly if move or charge.
        if len(ctx.horizon) != 1:
            return

        # landmark = [payload_kg 0, task_priority 1, task_name 2, from_loc_id 3, to_loc_id 4, home_docks 5:]

        # if next stop id is in the last mile dock. we need to suggest alternative if they exist
        if (next_stop_id in ctx.landmarks[5:]) and (ctx.landmarks[2] == "transport"):
            # but not the last dock in the list i.e. we have more dock stations that may be used.
            if len(ctx.landmarks[5:]) > 1:
                available_last_mile_dock = self.check_available_last_mile_dock(
                    reserved_checkpoint,
                    traffic_control,
                    task_dict,
                    4)
        elif (next_stop_id in ctx.landmarks[3:]) and (ctx.landmarks[2] in ["move", "charge"]):
            if len(ctx.landmarks[3:]) > 1:
                available_last_mile_dock = self.check_available_last_mile_dock(
                    reserved_checkpoint,
                    traffic_control,
                    task_dict,
                    2)

        if available_last_mile_dock:
            self._handle_no_conflict_case(f_id, _r_id, m_id, v_id, ctx) # publish it.
        else:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"robot {_r_id} Inactive: all docks occupied.",
                "FmTrafficHandler",
                "_handle_last_mile_conflict_case",
                "warn")

    # --------------------------------------------------------------------------------------------

    def _calculate_distance(self, ros_pose_x, ros_pose_y, stop_coordinate_x, stop_coordinate_y):
        """
        Parameters:
            ros_pose_x (float): Robot's current x-coordinate.
            ros_pose_y (float): Robot's current y-coordinate.
            stop_coordinate_x (float): x-coordinate of the stop.
            stop_coordinate_y (float): y-coordinate of the stop.
        Logic:
            Computes the distance to the next stop using the Euclidean distance formula.
            Returns the calculated distance.
        """
        return math.sqrt((float(stop_coordinate_x) - float(ros_pose_x))**2 + (float(stop_coordinate_y) - float(ros_pose_y))**2)

    # --------------------------------------------------------------------------------------------

    def update_robot_elapsed_time(self, _r_id):
        """
        Parameters:
            _r_id (str): Robot ID.
        Logic:
            this represents the elapsed time since robot has been waiting to use or reserve a node.
            Checks if an entry exists for the robot in temp_robot_delay_time.
                If it exists, updates the total elapsed time by calculating the difference from the current time.
                If it does not exist, initializes the entry with the current time and zero total elapsed time.
        """
        # If _r_id exists and its value is '0' and If _r_id doesn't exist, set it directly
        if isinstance(self.temp_robot_delay_time.get(_r_id, (0,0)), tuple):
            start_time, total_elapsed = self.temp_robot_delay_time.get(_r_id, (0, 0))
            if start_time == 0:
                # First time this robot enters a conflict — initialize entry.
                self.temp_robot_delay_time[_r_id] = (time.time(), 0.1)
            else:
                current_time = time.time()
                elapsed_time = current_time - start_time
                total_elapsed += elapsed_time
                self.temp_robot_delay_time[_r_id] = (current_time, total_elapsed)
        else:
            self.temp_robot_delay_time[_r_id] = (time.time(), 0.1)

    # --------------------------------------------------------------------------------------------

    def _handle_no_conflict_case(self, f_id, _r_id, m_id=None, v_id=None, ctx=None):
        """
        Parameters:
            _r_id (str): Robot ID.
            f_id (str): Fleet ID.
            next_stop_id (str): Identifier for the next stop node to be reserved.
        Logic:
            Since no traffic conflicts, reset the robot's delay time to '0' and sets its status flag to 'green'.
            Creates action objects for the next stop ID to be reserved and traffic signal.
            Inserts the decision to the instant action database.
        """
        self.temp_robot_delay_time[_r_id] = (0,0)  # Reset delay time | Update flag green

        # log viz:
        self.task_handler.visualization_handler.terminal_log_visualization(
            f"{_r_id} no conflict reservation. green.",
            "FmTrafficHandler",
            "_handle_no_conflict_case",
            "info")

        try:
            # get new active map
            temp_map_name = self.get_map(f_id, ctx.checkpoints[0], ctx.horizon[0])
            # ensure that map name is different from current map
            # Only compare maps if we already know the active map.
            # On startup, temp_fb_active_map_name is None because the robot
            # hasn't received a downloadMap yet — comparing None to a map_id
            # would always trigger a false 'elevator' warning every cycle.
            if ctx.active_map_name is not None and ctx.active_map_name != temp_map_name:
                # then we have a problem
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"r_id: {_r_id} is in an elevator but can not fetch new map.",
                    "FmTrafficHandler",
                    "_handle_no_conflict_case",
                    "warn")
            # estimate goal nodes time of arrival
            eta = self.estimate_time_to_node(robot_pos = ctx.agv_position,
                node_pos = ctx.checkp_itinerary[ctx.checkpoints.index(ctx.horizon[0])],
                min_vel = float(ctx.speed_min))
            temp_fb_node_eta = self.calculate_estimated_arrival_time(eta)
            # check if node actions exist
            self._handle_node_action(f_id, _r_id, m_id, v_id,
                                     ctx.header_id,
                                     ctx.base,
                                     ctx.dock_action_done,
                                     ctx.landmarks)
            # prepare order update message
            header_id = ctx.header_id + 1
            order_id, update_id = self.task_handler.generate_new_order_id(ctx.order_id)
            b_node, h_nodes, h_edges = self.task_handler.order_handler.create_order(ctx.checkpoints, # checkpoints
                                                            ctx.waitpoints, # waitpoints,
                                                            ctx.checkp_itinerary, # agv_itinerary,
                                                            ctx.waitp_itinerary, # wait_itinerary,
                                                            ctx.landmarks, # landmark
                                                            temp_map_name, # active map for which this nav will be performed
                                                            ctx.horizon[0:], # release (next_stop_id)
                                                            temp_fb_node_eta) # time of arrival at the newly released node
            self.task_handler.order_handler.build_order_msg(f_id,
                                                _r_id,
                                                header_id,
                                                v_id,
                                                m_id,
                                                b_node,
                                                h_nodes,
                                                h_edges,
                                                order_id,
                                                update_id)
        except (ValueError, TypeError) as error:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"{error}.",
                "FmTrafficHandler",
                "_handle_no_conflict_case",
                "error")

    # --------------------------------------------------------------------------------------------

    def fetch_mex_data(self, f_id, r_id=None, m_id=None):
        """
        Unified fleet and robot data fetcher.
        This version replaces the 3 prior call styles and returns everything at once:
            - Data for the current robot (r_id)
            - Data for its contending robot (mex) occupying next_stop_id
            - Fleet-wide traffic_control list
            - Unassigned tasks list

        Returns:
            {
                "robot_data": {...},
                "mex_data": {...},
                "traffic_control": [...],
                "unassigned": [...]
            }
        """

        mex_record = {}
        traffic_control = {}
        unassigned = []

        # -----------------------------------------------
        # Build robot_states from in-memory cache.
        # robot_state_cache is populated by on_mqtt_message every time a robot
        # publishes a state message — zero DB round-trips needed for state reads.
        # This makes state management cost O(1) per robot regardless of fleet size,
        # instead of the previous O(N) DB table scan.
        # -----------------------------------------------
        raw_cache = self.task_handler.state_handler.cache   # { r_id: raw_mqtt_state_msg }

        # Fall back to DB scan only if the cache is completely empty (cold start / first boot).
        if not raw_cache:
            state_recs = self.task_handler.state_handler.fetch_all_data(f_id, m_id)
            if state_recs:
                # Populate the cache from DB rows so subsequent calls never hit DB again.
                col = self.task_handler.state_handler.table_col
                s_idx   = col.index('serial_number')
                pos_idx = col.index('agv_position')
                base_idx = col.index('last_node_id')
                ns_idx  = col.index('node_states')
                as_idx  = col.index('action_states')
                v_idx   = col.index('velocity')
                err_idx = col.index('errors')
                sf_idx  = col.index('safety_state')
                maps_idx = col.index('maps')
                ts_idx  = col.index('timestamp')
                ver_idx = col.index('version')
                man_idx = col.index('manufacturer')
                for row in state_recs:
                    sn = row[s_idx + 1]
                    # Reconstruct a minimal raw-msg-like dict from DB columns.
                    raw_cache[sn] = {
                        "serialNumber": sn,
                        "version": row[ver_idx + 1],
                        "manufacturer": row[man_idx + 1],
                        "lastNodeId": row[base_idx + 1],
                        "agvPosition": row[pos_idx + 1],
                        "nodeStates": row[ns_idx + 1],
                        "actionStates": row[as_idx + 1],
                        "velocity": row[v_idx + 1],
                        "errors": row[err_idx + 1] or [],
                        "safetyState": row[sf_idx + 1],
                        "maps": row[maps_idx + 1],
                        "timestamp": str(row[ts_idx + 1]),
                    }
                    self.online_robots.add(sn)

        # Also get order records from DB (orders are only written by the FM, not pushed via MQTT).
        order_recs = self.task_handler.order_handler.fetch_all_data(f_id, m_id)
        if not raw_cache or not order_recs:
            self.task_handler.visualization_handler.terminal_log_visualization(
                "No state or order records to dissect.",
                "FmTrafficHandler",
                "fetch_mex_data",
                "warn")
            return {
                "robot_data": {},
                "mex_data": {},
                "traffic_control": {},
                "unassigned": []
            }

        # Dict to store traffic control layout {robot_serial: node_id}
        traffic_control_dict = {}

        # First pass: map nodes to occupier for quick conflict lookups.
        node_occupancy = {}
        for row in order_recs:
            o_id = row[5] # serial_number
            o_base_node = None
            try:
                # nodes field is at index 9 for order
                o_nodes = row[9] if isinstance(row[9], list) else json.loads(row[9])
                if o_nodes:
                    # Target node is nodes[1] if present, else nodes[0]
                    if len(o_nodes) > 1:
                        o_base_node = o_nodes[1].get("nodeId")
                    else:
                        o_base_node = o_nodes[0].get("nodeId")
            except Exception:
                continue

            if o_base_node:
                traffic_control_dict[o_id] = o_base_node
                
                # Check for completed or cancelled tasks
                order_uuid = row[6] # order_id
                parts = order_uuid.rsplit('_', 1)
                suffix = parts[-1] if len(parts) > 1 else ""
                
                # We only populate node_occupancy if the order is still "live"
                if suffix not in ["completed", "cancelled"]:
                    node_occupancy[o_base_node] = row



        # -----------------------------------------------
        # Pass 1: Parse STATE from in-memory cache
        # Each raw_cache entry is the raw MQTT state payload (dict with camelCase keys).
        # -----------------------------------------------
        robot_states = {}
        for serial_number, raw_msg in raw_cache.items():
            try:
                agv_position  = raw_msg.get("agvPosition") or {}
                errors        = raw_msg.get("errors") or []
                action_states = raw_msg.get("actionStates") or []
                velocity      = raw_msg.get("velocity") or {}
                maps          = raw_msg.get("maps") or []
                node_states   = raw_msg.get("nodeStates") or []
                safety_state  = raw_msg.get("safetyState") or {}
                state_timestamp = raw_msg.get("timestamp", "")
                base          = raw_msg.get("lastNodeId", "")

                record = {}
                record["version"]      = raw_msg.get("version", "")
                record["manufacturer"] = raw_msg.get("manufacturer", "")
                record["serial_number"] = serial_number
                record["base"]         = base.split(',')[0] if base else ''
                record["position"]     = [agv_position.get('x', 0.0),
                                          agv_position.get('y', 0.0),
                                          agv_position.get('theta', 0.0)]
                record["errors"]       = [err.get("errorType", "") for err in errors]

                lin_vel = velocity.get("vx", 0.0)
                ang_vel = velocity.get("omega", 0.0)
                record["lin_vel"] = float(lin_vel) if str(lin_vel).replace('.', '', 1).lstrip('-').isdigit() else 0.0
                record["ang_vel"] = float(ang_vel) if str(ang_vel).replace('.', '', 1).lstrip('-').isdigit() else 0.0

                record["active_map"] = self.process_maps(maps)

                e_stop = safety_state.get("eStop", "NONE")
                record["halt"] = False
                if e_stop == "MANUAL" or (serial_number in self.task_handler.pause_list) \
                        or (serial_number in self.task_handler.ignore_list):
                    record["halt"] = True

                has_order_minute_passed = self.check_minute_passed(state_timestamp, 3.0)
                if has_order_minute_passed:
                    self.task_handler.visualization_handler.terminal_log_visualization(
                        f"Message too old for {serial_number}: {state_timestamp}.",
                        "FmTrafficHandler",
                        "fetch_mex_data",
                        "warn")
                    record["halt"] = True

                record["node_states"]   = node_states
                record["action_states"] = action_states

                # default no-task fields
                record["agv_status"]        = 'red'
                record["horizon"]           = []
                record["horizon_release"]   = []
                record["header_id"]         = ''
                record["order_id"]          = ''
                record["c_pnts"]            = []
                record["c_pnts_pos"]        = []
                record["w_pnts"]            = []
                record["w_pnts_pos"]        = []
                record["dock"]              = ['low']
                record["unassigned"]        = []
                record["order_timestamp"]   = ''
                record["merged_nodes"]      = []
                record["dock_action_done"]  = False

                robot_states[serial_number] = record

            except Exception as err:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"State parsing error for record: {err}",
                    "FmTrafficHandler",
                    "fetch_mex_data",
                    "warn")
                continue

        # -----------------------------------------------
        # Pass 2: Get AGV dimensions and speed from factsheet cache
        # factsheet_cache holds the raw MQTT factsheet payload, populated on first
        # factsheet message arrival — no DB read needed after that.
        # -----------------------------------------------
        for serial_number, rec in robot_states.items():
            fs = self.task_handler.factsheet_handler.cache.get(serial_number)
            if fs:
                phys = fs.get("physicalParameters", {})
                agv_width  = phys.get("width")
                agv_length = phys.get("length")
                speed_min  = phys.get("speedMin")
            else:
                # Cold-start fallback: hit DB once, then don't do it again.
                _, _, agv_width, agv_length, speed_min, _, _ = \
                    self.task_handler.factsheet_handler.fetch_data(f_id, serial_number, m_id)

            if not agv_length:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"No factsheet for {serial_number}.",
                    "FmTrafficHandler",
                    "fetch_mex_data",
                    "warn")
                continue
            rec["speed_min"] = float(speed_min)
            rec["agv_size"]  = [float(agv_width), float(agv_length)]

        # -----------------------------------------------
        # Pass 3: Parse ORDER table
        # -----------------------------------------------
        # Use dynamic index lookup instead of hardcoding column indices
        timestamp_index = self.task_handler.order_handler.table_col.index('timestamp')
        serial_number_index = self.task_handler.order_handler.table_col.index('serial_number')
        # zone_set_id_index = self.task_handler.order_handler.table_col.index('zone_set_id')
        nodes_index = self.task_handler.order_handler.table_col.index('nodes')
        # edges_index = self.task_handler.order_handler.table_col.index('edges')
        order_id_index = self.task_handler.order_handler.table_col.index('order_id')
        # order_update_id_index = self.task_handler.order_handler.table_col.index('order_update_id')
        header_id_index = self.task_handler.order_handler.table_col.index('header_id')

        # Loop through the recs to find rows where 'base' and 'fleet_name' match
        for order_rec in order_recs:
            try:
                timestamp = order_rec[timestamp_index+1]
                serial_number = order_rec[serial_number_index+1]
                # zone_set_id = order_rec[zone_set_id_index+1]
                nodes = order_rec[nodes_index+1]
                # edges = order_rec[edges_index+1]
                header_id = order_rec[header_id_index+1]
                order_id = order_rec[order_id_index+1]
                # order_update_id = order_rec[order_update_id_index+1]

                if serial_number not in robot_states:
                    continue

                rec = robot_states[serial_number]
                rec["order_timestamp"] = timestamp
                rec["header_id"] = header_id
                rec["order_id"] = order_id.split(',')[0]

                waypoints = [
                    (node['nodeId'], node['nodePosition']['x'], node['nodePosition']['y'],
                    node['nodePosition']['theta'], node['released'], node.get('actions', []),
                    node['nodeDescription'])
                    for node in sorted(nodes, key=lambda x: x['sequenceId'])
                ]

                node_ids = []
                checkps, c_pnts_pos, checkps_release = [], [], []
                waitps, w_pnts_pos, waitps_release = [], [], []
                order_actions_list = []

                # Single loop to populate node_ids, node_loc, released, actions, checkps, checkps_release, and traffic_control
                for point in waypoints:

                    node_id, x, y, theta, release_state, actions, description = point
                    q = self.euler_to_quaternion(float(theta), 0.0, 0.0)

                    # Populate node-related lists
                    node_ids.append(node_id)
                    node_loc = [x, y, q[2], q[3]]
                    order_actions_list.append(actions)

                    # Simultaneously check if node_id starts with 'C' and populate checkps and checkps_release
                    if node_id.startswith('C'):
                        checkps.append(node_id)
                        c_pnts_pos.append(node_loc)
                        checkps_release.append(release_state) # Append corresponding release state
                    elif node_id.startswith('W'):
                        waitps.append(node_id)
                        w_pnts_pos.append(node_loc)
                        waitps_release.append(release_state)

                    # Populate traffic control if node is released
                    # we also want to skip an order that doesnt have a serial number because it implies unassigned task.
                    if release_state and \
                        (not serial_number.startswith("unassigned_")) and (serial_number not in self.task_handler.ignore_list):
                        
                        robot_nodes = traffic_control.setdefault(serial_number, [])
                        if node_id not in robot_nodes:
                            robot_nodes.append(node_id)
                        
                        # [Spoofing] Also protect the corresponding physical checkpoint if a Waitpoint is occupied
                        if node_id.startswith('W'):
                            numeric_part = ''.join(filter(str.isdigit, node_id))
                            corresponding_c = f'C{numeric_part}'
                            if corresponding_c not in robot_nodes:
                                robot_nodes.append(corresponding_c)

                # Extracts the last released node from the order to use as the logical 'base' (granted target)
                last_released_node = rec["base"]
                for point in waypoints:
                    node_id, _, _, _, release_state, _, description = point
                    if release_state:
                         last_released_node = node_id
                
                rec["base"] = last_released_node

                # Extract ETA from node description and check if robot is late
                description = next((p[6] for p in waypoints if p[0] == last_released_node), "")
                eta_str = self._extract_eta_from_description(description)

                # get merged nodes
                rec["merged_nodes"] = node_ids
                rec["dock"] = ['low']
                rec["dock_action_done"] = False
                rec["horizon"] = []
                rec["horizon_release"] = []
                rec["agv_status"] = 'red'

                # Keep orders that have not been assigned for later.
                if serial_number.startswith("unassigned_"):
                    # Ensure correct dictionary access based on provided keys.
                    # print("1: ", order_actions_list[0][0])
                    # print("2: ", order_actions_list[0][0]['actionParameters'][0]['value'])

                    # Fetch the desired 'landmark' value.
                    landmark_value = order_actions_list[0][0]['actionParameters'][0]['value']
                    unassigned.append((serial_number, timestamp, landmark_value))

                node_states = rec.get("node_states", [])
                base = rec.get("base", '')

                if order_id.endswith("_cancelled") or order_id.endswith("_completed"):
                    # since we dont want to issue more node targets, if true.
                    rec["halt"] = True

                if node_states:
                    # ensure that this is only checked if robot has active order.
                    # check if the robot is getting late i.e. current_time > est_arrival_time:
                    if eta_str and self.check_robot_arrival(eta_str):
                        self.task_handler.visualization_handler.terminal_log_visualization(
                            f"Robot {serial_number} is running late.",
                            "FmTrafficHandler",
                            "fetch_mex_data",
                            "warn")
                    # set agv status
                    rec["agv_status"] = 'green'
                else:
                    rec["agv_status"] = 'red'

                if base.split(',')[0] != '':
                    rec["base"] = base.split(',')[0]
                    # is the first element of the node_ids approved for me or not. did i reserve it?
                    # if yes, then it is my base:
                    if node_ids[0].startswith('C'):
                        if checkps_release[0]:
                            # implying a continuation of order "update etc."
                            rec["horizon"] = checkps[1:]
                            rec["horizon_release"] = checkps_release[1:]
                        # if it is not, its an horizon:
                        else:
                            # implying this is a new order
                            rec["horizon"] = checkps
                            rec["horizon_release"] = checkps_release
                    else: # we have a waitpoint wx cx cy. e.g. w17, c17 and c18
                        if len(node_ids) >= 3:
                            rec["horizon"] = checkps[2:] # [0] c17, [1] c18, then.. horizon[2:]
                            rec["horizon_release"] = checkps_release[2:]
                        else:
                            # houstin we have a problem. why we in a waitpoint with less than 3 nodes? wx cx cy.
                            raise ValueError("[FmTrafficHandler] Not enough nodes.")
                else:
                    # agv had no previous task before this. hence, everything is an horizon
                    rec["horizon"] = checkps
                    rec["horizon_release"] = checkps_release
                    rec["base"] = '' # base is empty

                rec["c_pnts"] = checkps
                rec["c_pnts_pos"] = c_pnts_pos if checkps else []
                rec["w_pnts"] = waitps
                rec["w_pnts_pos"] = w_pnts_pos if waitps else []
                rec["w_pnts_release"] = waitps_release

                # Process docking actions
                if order_actions_list:
                    for order_act_ in order_actions_list:
                        if order_act_:
                            order_act = order_act_[0]
                            # Process actions for docking
                            if order_act.get('actionType') == 'dock':
                                rec["dock"] = order_act.get('actionParameters', [{}])[0].get('value', [])
                                rec["pending_dock_action"] = order_act  # STORE ORIGINAL FOR IDEMPOTENT PINGS

                                if rec.get("action_states"):
                                    for state_act in rec["action_states"]:
                                        # STRICT VDA 5050 IDEMPOTENCY: Match original order actionId
                                        if state_act.get("actionId") == order_act.get('actionId'):
                                            action_status = state_act.get("actionStatus", "")
                                            if action_status == "FINISHED":
                                                rec["dock_action_done"] = True
                                            elif action_status == "FAILED":
                                                rec["halt"] = True
                                                self.task_handler.visualization_handler.terminal_log_visualization(
                                                    f"Robot {serial_number} dock failed.",
                                                    "FmTrafficHandler",
                                                    "fetch_mex_data",
                                                    "error")
                                            break

            except Exception as err:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"Order parsing error: {err}",
                    "FmTrafficHandler",
                    "fetch_mex_data",
                    "warn")
                continue

        # -----------------------------------------------
        # Resolve MEx (next stop occupant)
        # -----------------------------------------------
        next_stop_id = None
        if r_id in robot_states:
            robot_data = robot_states[r_id]
            if robot_data.get("horizon") and len(robot_data["horizon"]) > 0:
                next_stop_id = robot_data["horizon"][0]

        if next_stop_id:
            for serial_number, rec in robot_states.items():
                if rec.get("base") == next_stop_id:
                    mex_record = rec
                    break

        # --- ANALYTICS: COLLISION TRACKING ---
        # "what matters is that one reserves and the other waits or yield. 
        # problem or collision arrises if both are granted the node and the distance between them is less than 1.5m."

        granted_nodes_map = {}
        for rid, rec in robot_states.items():
            agv_pos = rec.get("agv_position", [])
            if len(agv_pos) >= 2:
                rx, ry = float(agv_pos[0]), float(agv_pos[1])
                granted = set()
                is_yielding = False
                
                if rec.get("base"):
                    granted.add(rec["base"])
                    if rec["base"].startswith('W'): is_yielding = True
                if rec.get("horizon"):
                    for n in rec["horizon"]:
                        if n: 
                            granted.add(n)
                            if n.startswith('W'): is_yielding = True
                
                for node in granted:
                    if node not in granted_nodes_map:
                        granted_nodes_map[node] = []
                    granted_nodes_map[node].append({ "rid": rid, "x": rx, "y": ry, "yielding": is_yielding })
        
        current_collision_robots = set()
        for node, robots_info in granted_nodes_map.items():
            if len(robots_info) > 1:
                for i in range(len(robots_info)):
                    for j in range(i+1, len(robots_info)):
                        r1 = robots_info[i]
                        r2 = robots_info[j]
                        
                        if r1["yielding"] or r2["yielding"]:
                            continue
                            
                        dist = math.sqrt((r1["x"] - r2["x"])**2 + (r1["y"] - r2["y"])**2)
                        if dist < 1.5:
                            current_collision_robots.add(r1["rid"])
                            current_collision_robots.add(r2["rid"])
        
        # Increment tracker for NEW collisions only
        for rid in current_collision_robots:
            if rid not in self.robots_in_collision:
                self.collision_tracker += 1
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"COLLISION: Robot {rid} in collision zone (< 1.5m) sharing granted node.",
                    "FmTrafficHandler", "fetch_mex_data", "critical"
                )
        
        # Update stateful set
        self.robots_in_collision = current_collision_robots

        # -----------------------------------------------
        # Clean up and return
        # -----------------------------------------------
        # Remove empty strings and 'none' (case-insensitive) from traffic_control
        traffic_control_clean = {}
        if isinstance(traffic_control, dict):
            for rid, nodes in traffic_control.items():
                clean_nodes = [tc for tc in nodes if tc != '' and tc.lower() != 'none']
                if clean_nodes:
                    traffic_control_clean[rid] = clean_nodes
        else:
            traffic_control_clean = traffic_control

        return {
            "robot_data": robot_states.get(r_id, {}),
            "mex_data": mex_record,
            "traffic_control": traffic_control_clean,
            "unassigned": unassigned
        }

    # --------------------------------------------------------------------------------------------

    def _handle_active_mex_conflict(self, f_id, r_id, m_id, v_id, next_stop_id, traffic_control, reserved_checkpoint, next_stop_coordinate, fb_rec, ctx):
        """
        Parameters
            r_id (str): The ID of the robot currently being processed.
            f_id (str): The fleet ID of the robot currently being processed.
            next_stop_id (str): The ID of the next stop.
            traffic_control (list): List of traffic control states.
            reserved_checkpoint (str): The checkpoint currently reserved by the robot.
            next_stop_coordinate (tuple): The coordinates of the next stop.
            record_ (dict): The data record of the robot currently occupying the checkpoint.
        Logic
            Initializes variables from the fetched data of the conflicting robot.
            Checks if the conflicting robot is active.
            If active and the status light is red:
                Compares priorities of the current and conflicting robots.
                Handles the conflict based on priority (higher, lower, or equal) or delay time.
                Updates the status of both robots after negotiation.
            If the robot is inactive or not responding, shows a message indicating human assistance is required.
        """

        self.update_robot_elapsed_time(r_id)

        # Initialize variables for the conflicting robot 'unknown MEx' that might be putting us in a stand-off
        try:
            # Initialize variables for the conflicting robot 'unknown MEx' that might be putting us in a stand-off
            mex_version = fb_rec["version"]
            mex_manufacturer = fb_rec["manufacturer"]
            mex_r_id = fb_rec["serial_number"]
            mex_agv_position = fb_rec["position"]
            mex_base = fb_rec["base"]
            mex_horizon = fb_rec["horizon"]
            mex_checkpoints = fb_rec["c_pnts"]
            mex_waitpoints = fb_rec["w_pnts"] # waitpoints,
            mex_agv_itinerary = fb_rec["c_pnts_pos"] # agv_itinerary,
            mex_wait_itinerary = fb_rec["w_pnts_pos"] # wait_itinerary,
            mex_agv_status = fb_rec["agv_status"]
            # notifications: int_msg:string, ext_msg:string, skip:bool, wait_time:float
            # mex_notifications = fb_rec["errors"]
            mex_dock_action_done = fb_rec["dock_action_done"]
            mex_halt = fb_rec["halt"]
            mex_active_map_name = fb_rec["active_map"]
            mex_landmarks = fb_rec["dock"] # landmark
            mex_order_id = fb_rec["order_id"]
            mex_header_id = fb_rec["header_id"]
            # mex_agv_size = [fb_rec["agv_size"][0], fb_rec["agv_size"][1]]
            mex_speed_min = fb_rec["speed_min"]
            # mex_vel_lin_ang = [fb_rec["lin_vel"], fb_rec["ang_vel"]]
            mex_merged_nodes = fb_rec["merged_nodes"]
        except KeyError as key_err:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"mex_r_id {key_err} could not be verified. handshake incomplete.",
                "FmTrafficHandler",
                "_handle_active_mex_conflict",
                "error")
            return
        except (TypeError, ValueError) as specific_err:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"Type or Value Error: {specific_err}.",
                "FmTrafficHandler",
                "_handle_active_mex_conflict",
                "error")
            return

        temp_fb_wait_time = None
        mex_wait_time = None

        # Handle active robot conflict

        # Handle active robot conflict:
        if ((mex_agv_status == 'red') and
            ((mex_base not in mex_landmarks) or ((mex_base in mex_landmarks) and mex_dock_action_done)) and
            (not mex_halt) and
            (len(mex_horizon) > 0) and
            (not mex_merged_nodes[0].startswith('W')) and
            (not ctx.merged_nodes[0].startswith('W'))):

            # Ensure once again that mex_horizon is not empty before accessing its elements
            if mex_horizon and mex_horizon[0] == reserved_checkpoint:
                # log viz:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"{r_id} started negotiation.",
                    "FmTrafficHandler",
                    "_handle_active_mex_conflict",
                    "info")

                priority_val = self.map_priority(ctx.landmarks[1])
                mex_priority_val = self.map_priority(mex_landmarks[1])
                reserved_checkpoint_coordinate = ctx.checkp_itinerary[ctx.checkpoints.index(reserved_checkpoint)]

                if priority_val > mex_priority_val:
                    temp_fb_wait_time, mex_wait_time = self.handle_priority_higher(
                        r_id, next_stop_id, next_stop_coordinate,
                        mex_r_id, mex_waitpoints, mex_wait_itinerary, mex_horizon, mex_checkpoints, mex_agv_itinerary, mex_base, 
                        mex_agv_position, mex_speed_min,
                        traffic_control, reserved_checkpoint, reserved_checkpoint_coordinate, ctx=ctx)

                elif priority_val < mex_priority_val:
                    temp_fb_wait_time, mex_wait_time = self.handle_priority_lower(
                        r_id, next_stop_id, next_stop_coordinate,
                        mex_r_id, mex_waitpoints, mex_wait_itinerary, mex_horizon, mex_checkpoints, mex_agv_itinerary, mex_base, 
                        mex_agv_position, mex_speed_min,
                        traffic_control, reserved_checkpoint, reserved_checkpoint_coordinate, ctx=ctx)

                else:
                    self.temp_robot_delay_time[mex_r_id] = self.temp_robot_delay_time.get(mex_r_id, (0,0))

                    # log viz: Handle the case where both robots have equal priority
                    self.task_handler.visualization_handler.terminal_log_visualization(
                        "Equal task priorities detected. Time priority will be used.",
                        "FmTrafficHandler",
                        "_handle_active_mex_conflict",
                        "info")

                    if float(self.temp_robot_delay_time[r_id][1]) > float(self.temp_robot_delay_time[mex_r_id][1]):
                        temp_fb_wait_time, mex_wait_time = self.handle_priority_higher(
                            r_id, next_stop_id, next_stop_coordinate,
                            mex_r_id, mex_waitpoints, mex_wait_itinerary, mex_horizon, mex_checkpoints, mex_agv_itinerary, mex_base, 
                            mex_agv_position, mex_speed_min,
                            traffic_control, reserved_checkpoint, reserved_checkpoint_coordinate, ctx=ctx)

                    # if their curr robot wait time is lower or wait times are also equal then just treat as low priority
                    elif float(self.temp_robot_delay_time[r_id][1]) < float(self.temp_robot_delay_time[mex_r_id][1]) or \
                        float(self.temp_robot_delay_time[r_id][1]) == float(self.temp_robot_delay_time[mex_r_id][1]):
                        temp_fb_wait_time, mex_wait_time = self.handle_priority_lower(
                            r_id, next_stop_id, next_stop_coordinate,
                            mex_r_id, mex_waitpoints, mex_wait_itinerary, mex_horizon, mex_checkpoints, mex_agv_itinerary, mex_base, 
                            mex_agv_position, mex_speed_min,
                            traffic_control, reserved_checkpoint, reserved_checkpoint_coordinate, ctx=ctx)

                if (temp_fb_wait_time is not None) or (mex_wait_time is not None):
                    self.update_robot_status(f_id,
                                            r_id,
                                            m_id,
                                            v_id,
                                            temp_fb_wait_time,
                                            mex_version,
                                            mex_manufacturer,
                                            mex_r_id,
                                            mex_agv_position,
                                            mex_speed_min,
                                            mex_wait_time,
                                            mex_base,
                                            mex_horizon,
                                            mex_checkpoints,
                                            mex_agv_itinerary,
                                            mex_waitpoints,
                                            mex_wait_itinerary,
                                            mex_landmarks,
                                            mex_dock_action_done,
                                            mex_active_map_name,
                                            mex_order_id,
                                            mex_header_id,
                                            ctx=ctx)
                else:
                    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                    #  we can try rerouting (✔️)
                    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                    reroute_status = self.reroute_robot(f_id,
                                                        r_id,
                                                        reserved_checkpoint,
                                                        next_stop_id,
                                                        mex_horizon, 
                                                        traffic_control,
                                                        ctx)
                    if reroute_status:
                        ctx.horizon = ctx.checkpoints
                        # publish the targets to the robots.
                        self.update_robot_status(f_id,
                                                r_id,
                                                m_id,
                                                v_id,
                                                temp_fb_wait_time,
                                                mex_version,
                                                mex_manufacturer,
                                                mex_r_id,
                                                mex_agv_position,
                                                mex_speed_min,
                                                mex_wait_time,
                                                mex_base,
                                                mex_horizon,
                                                mex_checkpoints,
                                                mex_agv_itinerary,
                                                mex_waitpoints,
                                                mex_wait_itinerary,
                                                mex_landmarks,
                                                mex_dock_action_done,
                                                mex_active_map_name,
                                                mex_order_id,
                                                mex_header_id,
                                                ctx=ctx)
                    else:
                        # log viz:
                        self.task_handler.visualization_handler.terminal_log_visualization(
                            f"{r_id} and mex_r_id {mex_r_id} stuck because waitpoint was not found in graph and no re-route/alternative path. Human assistance required.",
                            "FmTrafficHandler",
                            "_handle_active_mex_conflict",
                            "info")
            else:
                # log viz:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"{r_id} waiting for mex_r_id {mex_r_id} to pass. no possible conflict expected.",
                    "FmTrafficHandler",
                    "_handle_active_mex_conflict",
                    "info")

        else:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"{r_id}  waiting for mex_r_id {mex_r_id} to reach decision node.",
                "FmTrafficHandler",
                "_handle_active_mex_conflict",
                "info")

            if mex_halt:
                # log viz:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"however, mex_r_id {mex_r_id} is in halt mode. check pause, ignore, emergency cases.",
                    "FmTrafficHandler",
                    "_handle_active_mex_conflict",
                    "info")


    # --------------------------------------------------------------------------------------------

    def reroute_robot(self, f_id, r_id, reserved_checkpoint, next_stop_id, mex_horizon, traffic_control, ctx, task_dict=None):
        """
        Reroutes a robot from its current checkpoint to a new destination, handling traffic and waitpoints.
        1. fetch current robots landmark: task_type = transport, charge or move
        2. from 1, disect the docks. c10 c3 c5 etc.
        3. match current horizon with the task itinerary to figure where "node" robot is.
        4. plan from the current node to next target. at least 2 path suggestions and pick second.
        6. publish new path request.

        Args:
            f_id (str): Fleet ID.
            r_id (str): Robot ID.
            next_stop_id (int): The ID of the next stop.
            reserved_checkpoint (int): The ID of the current reserved checkpoint.
            next_stop_coordinate (tuple): The coordinates of the next stop.
            fb_checkpoints (list): List of checkpoints in the current route.
            fb_checkp_itinerary (list): Itinerary corresponding to checkpoints.
            landmarks (list): List of landmark stop IDs that cannot be skipped.
            task_dictionary (dict, optional): Dictionary mapping tasks to checkpoints. Defaults to None.
        """

        # Set task dictionary if none provided
        task_dict = task_dict or self.task_dictionary

        reroute_status = False
        if next_stop_id in ctx.landmarks:
            # then we can not skip it, we need to wait for a resolution.
            reroute_status = self._handle_no_skip_case(r_id, reserved_checkpoint, next_stop_id, task_dict, ctx)
        else:
            # we can skip this nex_stop_id node and try to plan a path to its upper one.
            # however, check if there is an alternative path to it first
            reroute_status = self._handle_skip_case(r_id, reserved_checkpoint, next_stop_id, task_dict, ctx)
        
        if reroute_status is False:
            # check if we do not have to skip, but perhaps we can go to a temporary wait.
            # we should call find temporary wait node. but this time its a waitcheckpoint yeah
            # we are not gonna spoof a c19 into a w19. instead we would simply insert a c19 into the horizon and path
            # so that the robot can go there and wait for the next stop to be available.
            # so we call find temporary wait node here
            yielding_goal_node = ctx.horizon[-1] if ctx.horizon else reserved_checkpoint
            cy_id, _, cy_pos = self._find_temporary_waitpoint(
                yielding_base_node=reserved_checkpoint,
                yielding_goal_node=yielding_goal_node,
                other_base_node=next_stop_id,
                other_horizon_path=mex_horizon,
                traffic_control=traffic_control
            )

            if cy_id:
                # Need to add yield point to checkpoints/horizon for it to be actually targeted
                ctx.checkpoints.insert(0, cy_id)
                ctx.checkpoints.insert(1, reserved_checkpoint)
                
                # Insert corresponding coordinates
                ctx.checkp_itinerary.insert(0, cy_pos)
                # For returning to the reserved checkpoint, we use the original reserved_checkpoint_coordinate
                # Here we fetch it from the itinerary (it should be at index 0 currently)
                reserved_checkpoint_coordinate = ctx.checkp_itinerary[0]
                ctx.checkp_itinerary.insert(1, reserved_checkpoint_coordinate)

                # Update horizon to force detour
                ctx.horizon[0:0] = [cy_id, reserved_checkpoint] 

                reroute_status = True

        # log viz:
        self.task_handler.visualization_handler.terminal_log_visualization(
            f"Fleet {f_id} <-> {r_id} Reroute_status --> {reroute_status}.",
            "FmTrafficHandler",
            "reroute_robot",
            "info")
        return reroute_status


    def _handle_no_skip_case(self, r_id, reserved_checkpoint, next_stop_id, task_dict, ctx):
        """Handles rerouting logic when the next stop is a landmark."""
        reroute_status = False
        paths = self.task_handler.fm_shortest_paths(reserved_checkpoint, next_stop_id, task_dict)
        # log viz:
        self.task_handler.visualization_handler.terminal_log_visualization(
            f"Checking no_skip paths for {r_id} --> {paths}.",
            "FmTrafficHandler",
            "_handle_no_skip_case",
            "info")

        if len(paths) > 1:
            sub_path_checkps = paths[1]
            if sub_path_checkps:
                sub_path_itinerary = self.task_handler.fm_get_itinerary(sub_path_checkps, task_dict)
                sub_path_waitpoints = self.task_handler.fm_extract_unique_waitpoints(sub_path_checkps, task_dict)
                sub_path_wait_itinerary = self.task_handler.fm_get_itinerary(sub_path_waitpoints, task_dict)
                # Replace first two elements in checkpoints and itinerary
                self._replace_path_elements(2, sub_path_checkps, sub_path_itinerary, ctx, reserved_checkpoint, next_stop_id)
                # Handle waitpoints and their itinerary
                self._update_waitpoints(next_stop_id, sub_path_waitpoints, sub_path_wait_itinerary, ctx)
                reroute_status = True
        return reroute_status


    def _handle_skip_case(self, r_id, reserved_checkpoint, next_stop_id, task_dict, ctx):
        """Handles rerouting logic when the next stop is not a landmark."""
        reroute_status = False
        # first handle no skip next_stop_id case, as it might not be necessary to skip it if this works out.
        reroute_status = self._handle_no_skip_case(r_id, reserved_checkpoint, next_stop_id, task_dict, ctx)
        # if false, then try to go for the upper node from the horizon.
        if reroute_status is False:
            if len(ctx.horizon) > 1:
                upper_stop_id = ctx.horizon[1]
                paths = self.task_handler.fm_shortest_paths(reserved_checkpoint, upper_stop_id, task_dict)
                # log viz:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"Checking skip paths for {r_id} --> {paths}.",
                    "FmTrafficHandler",
                    "_handle_skip_case",
                    "info")
                if len(paths) > 1:
                    sub_path_checkps = paths[1]
                    # we want to fuse this path into our real checkpoints. such that we dont include 'next_stop_id'
                    if sub_path_checkps and sub_path_checkps != [reserved_checkpoint, next_stop_id, upper_stop_id]:
                        sub_path_itinerary = self.task_handler.fm_get_itinerary(sub_path_checkps, task_dict)
                        sub_path_waitpoints = self.task_handler.fm_extract_unique_waitpoints(sub_path_checkps, task_dict)
                        sub_path_wait_itinerary = self.task_handler.fm_get_itinerary(sub_path_waitpoints, task_dict)
                        # Replace first three elements in checkpoints and itinerary
                        self._replace_path_elements(3, sub_path_checkps, sub_path_itinerary, ctx, reserved_checkpoint, next_stop_id, upper_stop_id)
                        # Handle waitpoints and their itinerary
                        self._update_waitpoints(next_stop_id, sub_path_waitpoints, sub_path_wait_itinerary, ctx)
                        reroute_status = True
        return reroute_status

    def _replace_path_elements(self, num_elements_to_replace, sub_path_checkps, sub_path_itinerary, ctx, *checkpoints_to_remove):
        """Replaces the first few elements of checkpoints and itinerary with a new sub-path."""
        # replace the first three elements of the original path with the sub_path_replacement path
        # do these for checkpoints and checkp_itinerary
        assert ctx.checkpoints[:num_elements_to_replace] == list(checkpoints_to_remove)
        # delete the first three elements from the temp_fb_checkpoints and from the checkpoint itinerary list as well
        # then add the new sub path to the checkpoint front
        ctx.checkpoints = sub_path_checkps + ctx.checkpoints[num_elements_to_replace:]
        ctx.checkp_itinerary = sub_path_itinerary + ctx.checkp_itinerary[num_elements_to_replace:]

    def _update_waitpoints(self, next_stop_id, sub_path_waitpoints, sub_path_wait_itinerary, ctx):
        """Updates the waitpoints and their itinerary based on the next stop."""
        # then do for waitpoints and waitp_itinerary
        # recall that there was no waitpoint associated with the reserved node. hence why we are here:
        # so, check if the first element of the waitpoint is for the next_stop_id. if it is we replace it.
        # with our new wait itinerary.
        # extract the number from 'next_stop_id' and extract the number of the first element of the waitpoint list.
        temp_fb_wait_traffic = self.check_waitpoint_association(next_stop_id, ctx.waitpoints)
        if temp_fb_wait_traffic:
            ctx.waitpoints = sub_path_waitpoints + ctx.waitpoints[1:]
            ctx.waitp_itinerary = sub_path_wait_itinerary + ctx.waitp_itinerary[1:]
        else:
            # if its not we just add/append the new waitpoint list to the front of the list.
            ctx.waitpoints = sub_path_waitpoints + ctx.waitpoints
            ctx.waitp_itinerary = sub_path_wait_itinerary + ctx.waitp_itinerary

    def _find_temporary_waitpoint(self, yielding_base_node, yielding_goal_node,
                                  other_base_node, other_horizon_path, traffic_control):
        """
        Finds a temporary waitpoint (a free neighbouring checkpoint) for the yielding robot.
        Rules:
        1. Node must be a 'checkpoint' (not dock).
        2. Must not block the other robot's horizon.
        3. Closer to the yielding robot's goal than other valid neighbours.
        4. Neither robot can be at a dock (both must be on a 'checkpoint').
        """
        def is_checkpoint(node_id):
            node_desc = next((item['description'] for item in self.task_dictionary.get('itinerary', []) if item['loc_id'] == node_id), None)
            return node_desc == 'checkpoint'

        if not is_checkpoint(yielding_base_node): 
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"YBN --> {yielding_base_node}.",
                "FmTrafficHandler",
                "_find_temporary_waitpoint",
                "info")
            return None, None, None

        graph = self.task_handler.build_graph(self.task_dictionary)
        neighbors = graph.get(yielding_base_node, set())

        best_cy_id = None
        best_distance = float('inf')
        best_cy_itinerary = None
        best_spoofed_wy = None

        for cy in neighbors:
            cy_id = cy[0]

            if cy_id == yielding_base_node or cy_id == other_base_node or cy_id in traffic_control:
                continue
                
            if not is_checkpoint(cy_id):
                continue

            if cy_id in other_horizon_path:
                continue

            distance = self.task_handler.fm_get_path_distance(cy_id, yielding_goal_node, graph)
            if distance is None:
                continue

            cy_itinerary = self.task_handler.fm_get_itinerary([cy_id], self.task_dictionary)
            if cy_itinerary and distance < best_distance:
                numeric_part = ''.join(filter(str.isdigit, cy_id))
                spoofed_wy = f'W{numeric_part}'

                best_distance = distance
                best_cy_id = cy_id
                best_cy_itinerary = cy_itinerary
                best_spoofed_wy = spoofed_wy

        # log viz:
        self.task_handler.visualization_handler.terminal_log_visualization(
            f"NEIGHBOR YIELD: Viable node for temporary wait, cy id --> {best_cy_id}.",
            "FmTrafficHandler",
            "_find_temporary_waitpoint",
            "info")
            
        if best_cy_id is not None:
            return best_cy_id, best_spoofed_wy, best_cy_itinerary[0]

        return None, None, None



    # --------------------------------------------------------------------------------------------

    def handle_priority_higher(self, r_id, next_stop_id, next_stop_coordinate,mex_r_id, mex_waitpoints, mex_wait_itinerary, mex_horizon,
                               mex_checkpoints, mex_agv_itinerary, mex_base, mex_agv_position, mex_speed_min, traffic_control, reserved_checkpoint,
                               reserved_checkpoint_coordinate, from_source=True, ctx=None):
        """
        Parameters
            r_id (str): The ID of the current robot.
            next_stop_id (str): The ID of the next stop.
            traffic_control (list): List of traffic control states.
            mex_notifications (list): List of notifications for the conflicting robot.
            next_stop_coordinate (tuple): The coordinates of the next stop.
            mex_r_id (str): The ID of the conflicting robot.
            mex_waitpoints (list): List of waitpoints for the conflicting robot.
            reserved_checkpoint (str): The checkpoint currently reserved by the robot.
            mex_agv_position (tuple): The position of the conflicting robot.
            mex_speed_min (list): The model speed configuration for the conflicting robot.
            reserved_checkpoint_coordinate (tuple): The coordinates of the reserved checkpoint.
            from_source (bool, optional): Whether this function is called from the source.
        Logic
            Determines if the robot should reserve the node and handle the conflict based on whether the current robot has higher priority.
            If a waitpoint is not found, handles the conflict as if the priority is lower.
            Estimates wait time (how long to wait at waitpoint) for the conflicting robot and updates notifications.
            Returns:mex_wait_traffic (str or None): The wait traffic associated with the conflicting robot or None.
                    mex_notifications (list): Updated notifications for the conflicting robot.
        """
        # this robot should reserve the node it wants to go, and then drive directly there.
        # the MEx should reserve this robot's current reserved node, then go to the waiting area first before driving towards it.
        # log viz:
        self.task_handler.visualization_handler.terminal_log_visualization(
            f"{r_id}: high priority case.",
            "FmTrafficHandler",
            "handle_priority_higher",
            "info")
        temp_fb_wait_time = None
        mex_wait_time = None
        mex_wait_traffic = self.check_waitpoint_association(next_stop_id, mex_waitpoints)

        # check if there is waitpoint for the other robot, so that perhaps the other robot should wait instead.
        if mex_wait_traffic is None:
            if from_source is True:
                # could be that this robot occupies a checkpoint without a waitpoint or a station dock.
                temp_fb_wait_time, _ = self.handle_priority_lower(r_id, next_stop_id, next_stop_coordinate,mex_r_id, mex_waitpoints, mex_wait_itinerary, mex_horizon,
                                    mex_checkpoints, mex_agv_itinerary, mex_base, mex_agv_position, mex_speed_min, traffic_control, reserved_checkpoint,
                                    reserved_checkpoint_coordinate, False, ctx=ctx)
            else:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"{mex_r_id} mex --> could not find waitpoint in graph. human help required.",
                    "FmTrafficHandler",
                    "handle_priority_higher",
                    "error")

        if (mex_wait_traffic is not None) and (mex_wait_traffic not in traffic_control):
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"{mex_r_id} mex --> wait at {mex_wait_traffic}.",
                "FmTrafficHandler",
                "handle_priority_higher",
                "info")
            mex_est_time = self.estimate_time_to_node(robot_pos = ctx.agv_position,
                                                    node_pos = next_stop_coordinate,
                                                    min_vel = float(ctx.speed_min))
            mex_wait_time = str(mex_est_time)
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"{mex_r_id} mex --> estimated wait time {mex_est_time}.",
                "FmTrafficHandler",
                "handle_priority_higher",
                "info")

        return temp_fb_wait_time, mex_wait_time

    # --------------------------------------------------------------------------------------------

    def handle_priority_lower(self, r_id, next_stop_id, next_stop_coordinate,mex_r_id, mex_waitpoints, mex_wait_itinerary, mex_horizon,
                               mex_checkpoints, mex_agv_itinerary, mex_base, mex_agv_position, mex_speed_min, traffic_control, reserved_checkpoint,
                               reserved_checkpoint_coordinate, from_source=True, ctx=None):
        """
        Parameters
            same as handle priority higher
        Logic
            Determines if the robot should wait at the reserved checkpoint and
            handles the conflict based on whether the current robot has lower priority.
            If a waitpoint is not found, handles the conflict as if the priority is higher.
            Estimates wait time for the current robot and updates notifications.
            Returns: temp_fb_wait_traffic (str or None), temp_fb_wait_time
        """
        # log viz:
        self.task_handler.visualization_handler.terminal_log_visualization(
            f"{r_id}: lower priority case.",
            "FmTrafficHandler",
            "handle_priority_lower",
            "info")
        temp_fb_wait_time = None
        mex_wait_time = None
        temp_fb_wait_traffic = self.check_waitpoint_association(reserved_checkpoint, ctx.waitpoints)

        # log viz:
        self.task_handler.visualization_handler.terminal_log_visualization(
            f"currently reserved: {reserved_checkpoint}. temp_fb_wait_traffic: {temp_fb_wait_traffic}.",
            "FmTrafficHandler",
            "handle_priority_lower",
            "info")
        # if we have not checked before,
        # check if there is waitpoint for the other robot, so that perhaps the other robot should wait instead.
        if temp_fb_wait_traffic is None:
            if from_source is True:
                # log viz:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"{r_id}: could not find a node associated waitpoint. will try for high priority.",
                    "FmTrafficHandler",
                    "handle_priority_lower",
                    "info")
                # could be that this robot occupies a checkpoint without a waitpoint or a station dock.
                _, mex_wait_time = self.handle_priority_higher(r_id, next_stop_id, next_stop_coordinate,mex_r_id, mex_waitpoints, mex_wait_itinerary, mex_horizon,
                                    mex_checkpoints, mex_agv_itinerary, mex_base, mex_agv_position, mex_speed_min, traffic_control, reserved_checkpoint,
                                    reserved_checkpoint_coordinate, False, ctx=ctx)
            else:
                # log viz:
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"{r_id} --> could not find waitpoint in graph. human help required.",
                    "FmTrafficHandler",
                    "handle_priority_lower",
                    "error")

        if temp_fb_wait_traffic and temp_fb_wait_traffic not in traffic_control:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"{r_id} --> wait at {temp_fb_wait_traffic}.",
                "FmTrafficHandler",
                "handle_priority_lower",
                "info")
            est_time = self.estimate_time_to_node(robot_pos = mex_agv_position,
                                                  node_pos = reserved_checkpoint_coordinate,
                                                  min_vel = float(mex_speed_min))
            temp_fb_wait_time = str(est_time)
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"{r_id} --> estimated wait time {est_time}.",
                "FmTrafficHandler",
                "handle_priority_lower",
                "info")
        return temp_fb_wait_time, mex_wait_time


    # --------------------------------------------------------------------------------------------

    def update_robot_status(self, f_id, r_id, m_id, v_id, temp_fb_wait_time,
                            mex_version, mex_manufacturer, mex_r_id, mex_agv_position, mex_speed_min,
                            mex_wait_time, mex_base, mex_horizon, mex_checkpoints, mex_agv_itinerary,
                            mex_waitpoints, mex_wait_itinerary, mex_landmarks, mex_dock_action_done,
                            mex_active_map_name, mex_order_id, mex_header_id, ctx=None):
        """
        Parameters
            r_id (str): The ID of the current robot.
            f_id (str): The fleet ID of the current robot.
            next_stop_id (str): The ID of the next stop.
            mex_r_id (str): The ID of the conflicting robot.
            mex_wait_traffic (str or None): The wait traffic associated with the conflicting robot or None.
            mex_horizon (str): The last checkpoint of the conflicting robot.
            mex_notifications (list): List of notifications for the conflicting robot.
        Logic
            Updates the status of both robots in the system and shows status transition messages.
            Creates and inserts actions for both robots to effect these changes.
        """

        # robot in consideration
        self.temp_robot_delay_time[r_id] = (0,0) # 'green'
        try:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"r_id: {r_id} --> estimated wait time {temp_fb_wait_time}.",
                "FmTrafficHandler",
                "update_robot_status",
                "info")
            # get new active map
            temp_map_name = self.get_map(f_id, ctx.checkpoints[0], ctx.horizon[0])
            # ensure that map name is different from current map
            if ctx.active_map_name != temp_map_name:
                # then we have a problem
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"r_id: {r_id} is in an elevator but can not fetch new map.",
                    "FmTrafficHandler",
                    "update_robot_status",
                    "warn")
            # estimate goal nodes time of arrival
            eta = self.estimate_time_to_node(robot_pos = ctx.agv_position,
                node_pos = ctx.checkp_itinerary[ctx.checkpoints.index(ctx.horizon[0])],
                min_vel = float(ctx.speed_min))
            if temp_fb_wait_time:
                # here horizon is the target node that is currently occupied,
                # since we would go to a wait point, wait some seconds at the waitpoint,
                # and then come back to our current base node, we need to add this up:
                # wait_time_default should be 2 ideally but we cant wait more for testing.
                eta += ((self.wait_time_default * eta) + float(temp_fb_wait_time))

            temp_fb_node_eta = self.calculate_estimated_arrival_time(eta)
            # check if node actions exist
            self._handle_node_action(f_id, r_id, m_id, v_id,
                                     ctx.header_id,
                                     ctx.base,
                                     ctx.dock_action_done,
                                     ctx.landmarks)
            # publish a wait order update message.
            header_id = ctx.header_id + 1
            order_id, update_id = self.task_handler.generate_new_order_id(ctx.order_id)
            b_node, h_nodes, h_edges = self.task_handler.order_handler.create_order(
                                                            ctx.checkpoints, # checkpoints
                                                            ctx.waitpoints, # waitpoints,
                                                            ctx.checkp_itinerary, # agv_itinerary,
                                                            ctx.waitp_itinerary, # wait_itinerary,
                                                            ctx.landmarks, # landmark)
                                                            temp_map_name, # active map for which this nav will be performed
                                                            ctx.horizon[0:], # release (next_stop_id)
                                                            temp_fb_node_eta, # time of arrival at the newly released node
                                                            temp_fb_wait_time) # wait_time
            self.task_handler.order_handler.build_order_msg(f_id,
                                                r_id,
                                                header_id,
                                                v_id,
                                                m_id,
                                                b_node,
                                                h_nodes,
                                                h_edges,
                                                order_id,
                                                update_id)

        except (ValueError, TypeError) as error:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"r_id: {r_id}, {error}.",
                "FmTrafficHandler",
                "update_robot_status",
                "info")

        # vs mobile executor * traffic or next_stop_id?
        self.temp_robot_delay_time[mex_r_id] = (0, 0) # mex_status[6] = 'green'
        try:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"mex_r_id: {mex_r_id} --> mex estimated wait time {mex_wait_time}.",
                "FmTrafficHandler",
                "update_robot_status",
                "info")
            # get new active map
            mex_map_name = self.get_map(f_id, mex_checkpoints[0], mex_horizon[0])
            # ensure that map name is different from current map
            if mex_active_map_name == mex_map_name:
                # then we have a problem
                self.task_handler.visualization_handler.terminal_log_visualization(
                    f"mex_r_id: {mex_r_id} is in an elevator but can not fetch new map.",
                    "FmTrafficHandler",
                    "update_robot_status",
                    "error")
            # similarly, estimate goal nodes time of arrival
            mex_eta = self.estimate_time_to_node(robot_pos = mex_agv_position,
                node_pos = mex_agv_itinerary[mex_checkpoints.index(mex_horizon[0])],
                min_vel = float(mex_speed_min))
            if mex_wait_time:
                mex_eta += ((self.wait_time_default * mex_eta) + float(mex_wait_time))
            mex_node_eta = self.calculate_estimated_arrival_time(mex_eta)
            # check if node actions exist
            self._handle_node_action(f_id, mex_r_id, mex_manufacturer, mex_version,
                                     mex_header_id,
                                     mex_base,
                                     mex_dock_action_done,
                                     mex_landmarks)
            # publish a wait order update message.
            header_id = mex_header_id + 1
            order_id, update_id = self.task_handler.generate_new_order_id(mex_order_id)
            b_node, h_nodes, h_edges = self.task_handler.order_handler.create_order(
                                                            mex_checkpoints, # checkpoints
                                                            mex_waitpoints, # waitpoints,
                                                            mex_agv_itinerary, # agv_itinerary,
                                                            mex_wait_itinerary, # wait_itinerary,
                                                            mex_landmarks, # landmark
                                                            mex_map_name, # active map for which this nav will be performed
                                                            mex_horizon[0:], # release (next_stop_id)
                                                            mex_node_eta, # time of arrival at the newly released node
                                                            mex_wait_time) # wait_time
            self.task_handler.order_handler.build_order_msg(f_id,
                                                mex_r_id,
                                                header_id,
                                                mex_version,
                                                mex_manufacturer,
                                                b_node,
                                                h_nodes,
                                                h_edges,
                                                order_id,
                                                update_id)
        except (ValueError, TypeError) as error:
            # log viz:
            self.task_handler.visualization_handler.terminal_log_visualization(
                f"mex_r_id: {mex_r_id}, {error}.",
                "FmTrafficHandler",
                "update_robot_status",
                "info")

        # log viz:
        self.task_handler.visualization_handler.terminal_log_visualization(
            f"r_id: {r_id} completed negotiation.",
            "FmTrafficHandler",
            "update_robot_status",
            "info")

    # --------------------------------------------------------------------------------------------

# ---------------------------------------------- #
#      MAIN                                      #
# ---------------------------------------------- #

if __name__ == "__main__":

   # establish connection to DB
    conn = None
    try: # Sample database connection setup (assuming the database is already created)
        conn = psycopg2.connect(host='localhost', dbname='postgres', user='postgres', password='root', port='5432')
    except (psycopg2.OperationalError, psycopg2.ProgrammingError) as e:
        print("Failed to connect to PostgreSQL database: %s", e)

    # initialize the task network dictionary
    task_dictionary = None
    # get config file path:
    agv_dir_prefix = (os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
    print("Config directory path: ", agv_dir_prefix)
    # file_path = Path(agv_dir_prefix+'/viro_simple_fleet/config/fleet_mngr.yaml')
    file_path = Path(os.path.join(agv_dir_prefix, 'fleet_management', 'config', 'config.yaml'))
    if file_path.is_file():
        with open(file_path, 'r', encoding='utf-8') as yaml_file:
            task_dictionary = yaml.safe_load(yaml_file)
    else:
        print("file path not set.")

    # Initialize the StateSubscriber: Generate order message
    fleetname = "kullar"
    robot_serial_number = "AGV-001" # "SN12345678"
    robot_serial_number2 = "AGV-002"
    version = "1.0.0"
    versions = "v1"
    manufacturer = "birfen"
    connectionState = "ONLINE"
    robot1_reserved_node = "C6" # "C10"
    robot2_reserved_node = "C6" # "C6" or "C11"

    # Step 2: Insert a pseudo robot data into the database
    sample_conn_message = {
        "headerId": 1,
        "timestamp": datetime.datetime.now().isoformat(),
        "version": version,
        "manufacturer": manufacturer,
        "serialNumber": robot_serial_number,
        "connectionState": connectionState # ['ONLINE', 'OFFLINE', 'CONNECTIONBROKEN']
    }

    # Sample message structure that will be validated, inserted into the database, and fetched
    sample_state_message = {
        "headerId": 2,
        "version": version,
        "manufacturer": manufacturer,
        "serialNumber": robot_serial_number, # "SN12345678",
        "timestamp": datetime.datetime.now().isoformat(), # "2024-09-17T14:30:00Z",
        "maps": [
            {
                "mapId": "map1",
                "mapVersion": "1.0",
                "mapDescription": "Main warehouse map",
                "mapStatus": "ENABLED"
            }
        ],
        # "orderId": "order_1234",
        "orderId": "",
        "orderUpdateId": 1,
        "zoneSetId": fleetname,
        # if robot has a last successful node, then 'robot1_reserved_node'. and it automatically implies that, its released or reserved "true" state.
        # if robot has no last visited node, then ''.
        "lastNodeId": '', # robot1_reserved_node,
        "lastNodeSequenceId": 1,
        "driving": False,
        "paused": True,
        "newBaseRequest": False,
        "distanceSinceLastNode": 1.0,
        "operatingMode": "AUTOMATIC",
        "nodeStates": [
            # {
            #     "nodeId": "C10", # node_1
            #     "sequenceId": 1,
            #     "nodeDescription": "Start Node",
            #     "nodePosition": {
            #         "x": 1.8,
            #         "y": 0.49,
            #         "theta": 0.0,
            #         "mapId": "map1"
            #     },
            #     "released": False
            # }
        ],
        "edgeStates": [
            # {
            #     "edgeId": "edge_1",
            #     "sequenceId": 1,
            #     "edgeDescription": "Edge between C1 and C2",
            #     "released": False,
            #     "trajectory": {
            #         "degree": 3,
            #         "knotVector": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            #         "controlPoints": [
            #             {"x": 0.0, "y": 0.0, "weight": 1.0},
            #             {"x": 2.5, "y": 2.5, "weight": 1.0},
            #             {"x": 5.0, "y": 5.0, "weight": 1.0}
            #         ]
            #     }
            # }
        ],
        "agvPosition": {
            # "x": 1.0,
            # "y": 1.0,
            # "theta": 1.57,
            "x": -1.7,
            "y": 0.63,
            "theta": 1.57,
            "mapId": "map1",
            "positionInitialized": True
        },
        "velocity": {
            "vx": 1.0,
            "omega": 0.3
        },
        "loads": [
            {
                "loadId": "some_id",
                "loadType": "6kg",
                "loadPosition": "front",
                "boundingBoxReference": {
                    "x": 1.0,
                    "y": 1.0,
                    "z": 0.5,
                    "theta": 0.0
                },
                "loadDimensions": {
                    "length": 1.0,
                    "width": 0.5,
                    "height": 0.5
                },
                "weight": 5.0
            }
        ],
        "actionStates": [
            {
                "actionId": "action_0",
                "actionType": "load",
                "actionDescription": "Loading item",
                "actionStatus": "WAITING",
                "resultDescription": "Waiting to load item"
            }
        ],
        "batteryState": {
            "batteryCharge": 85.0,
            "charging": False
        },
        "errors": [
            {
                "errorType": "SensorFailure",
                "errorLevel": "FATAL",
                "errorReferences": [
                    {
                        "referenceKey": "sensorId",
                        "referenceValue": "sensor_123"
                    }
                ],
                "errorDescription": "The sensor has failed and is not responding.",
                "errorHint": "Check the sensor connections and replace the sensor if necessary."
            },
            {
                "errorType": "BatteryLow",
                "errorLevel": "WARNING",
                "errorReferences": [
                    {
                        "referenceKey": "batteryId",
                        "referenceValue": "battery_456"
                    }
                ],
                "errorDescription": "The battery level is critically low.",
                "errorHint": "Charge the battery as soon as possible."
            }
        ],
        "information": [],
        "safetyState": {
            "eStop": "NONE",
            "fieldViolation": True
        }
    }

    sample_factsheet_message = {
        "headerId": 1,
        "timestamp": datetime.datetime.now().isoformat(), # "2024-09-15T12:00:00Z",
        "version": version,
        "manufacturer": manufacturer,
        "serialNumber": robot_serial_number,
        "typeSpecification": {
            "seriesName": "Series A",
            "agvKinematic": "DifferentialDrive",
            "agvClass": "Class 1",
            "maxLoadMass": 69,
            "localizationTypes": ["GPS", "LIDAR"],
            "navigationTypes": ["SLAM", "ODOMETRY"]
        },
        "physicalParameters": {
            "speedMin": 0.1,
            "speedMax": 2.5,
            "accelerationMax": 1.0,
            "decelerationMax": 1.0,
            "heightMin": 0.5,
            "heightMax": 1.2,
            "width": 0.8,
            "length": 1.5
        },
        "protocolLimits": {
            "maxStringLens": {
                "msgLen": 256,
                "topicSerialLen": 50,
                "topicElemLen": 30,
                "idLen": 10,
                "idNumericalOnly": True,
                "enumLen": 20,
                "loadIdLen": 15
            },
            "maxArrayLens": {
                "order.nodes": 100,
                "order.edges": 100,
                "node.actions": 50,
                "edge.actions": 50,
                "actions.actionsParameters": 50,
                "instantActions": 10,
                "trajectory.knotVector": 20,
                "trajectory.controlPoints": 30,
                "state.nodeStates": 50,
                "state.edgeStates": 50,
                "state.loads": 10,
                "state.actionStates": 50,
                "state.errors": 10,
                "state.information": 50,
                "error.errorReferences": 10,
                "information.infoReferences": 10
            },
            "timing": {
                "minOrderInterval": 0.5,
                "minStateInterval": 0.5,
                "defaultStateInterval": 1.0,
                "visualizationInterval": 2.0
            }
        },
        "protocolFeatures": {
            "optionalParameters": {
                "param1": "value1",
                "param2": "value2"
            },
            "agvActions": {
                "action1": "description1",
                "action2": "description2"
            }
        },
        "agvGeometry": {
            "wheelDefinitions": {
                "wheel1": {"radius": 0.2, "width": 0.05},
                "wheel2": {"radius": 0.2, "width": 0.05}
            },
            "envelopes2d": [
                {"shape": "rectangle", "dimensions": {"width": 0.8, "length": 1.5}}
            ]
        },
        "loadSpecification": {
            "loadPositions": ["front", "rear"],
            "loadSets": [
                {"set1": ["loadA", "loadB"]},
                {"set2": ["loadC"]}
            ]
        },
        "vehicleConfig": {
            "versions": [{"fleetName" : fleetname}],
            "config1": "value1",
            "config2": "value2"
        }
    }


    traffic_handler = FmTrafficHandler(
        fleetname = fleetname,
        version = version,
        versions = versions,
        manufacturer = manufacturer,
        dbconn = conn,
        mqttclient=None,
        task_dict=task_dictionary)

    # robot 1
    # ---- time.sleep(1.5)
    # Step 3: Insert sample message into database
    traffic_handler.task_handler.factsheet_handler.process_message(sample_factsheet_message)
    traffic_handler.task_handler.connection_handler.process_message(sample_conn_message)
    traffic_handler.task_handler.state_handler.process_message(sample_state_message)
    # Test fm_send_task_request method
    task_cleared, available_robots, checkpoints, \
        agv_itinerary, waitpoints, \
            wait_itinerary, landmark = traffic_handler.task_handler.fm_send_task_request(f_id=fleetname,
                                                                         r_id=robot_serial_number,
                                                                         from_loc_id='C5',
                                                                         to_loc_id='C3',
                                                                         task_name='transport',
                                                                         task_priority='high',
                                                                         payload_kg=35.0)

    # --- time.sleep(1.5)
    sample_conn_message["serialNumber"] = robot_serial_number2
    sample_state_message["serialNumber"] = robot_serial_number2
    sample_factsheet_message["serialNumber"] = robot_serial_number2
    sample_state_message["lastNodeId"] = '' # robot2_reserved_node
    sample_state_message["nodeStates"] = [
            # {
            #     "nodeId": "C6", # node_1
            #     "sequenceId": 1,
            #     "nodeDescription": "Start Node",
            #     "nodePosition": {
            #         "x": 0.53,
            #         "y": 0.53,
            #         "theta": 0.0,
            #         "mapId": "map1"
            #     },
            #     "released": False
            # }
        ]
    sample_state_message["agvPosition"] = {
            # "x": -1.0,
            # "y": 1.0,
            # "theta": 1.57,
            "x": 1.8,
            "y": 0.49,
            "theta": -0.695,
            # "x": 0.5,
            # "y": 1.7,
            # "theta": 1.57,
            "mapId": "map1",
            "positionInitialized": True
        }
    traffic_handler.task_handler.factsheet_handler.process_message(sample_factsheet_message)
    traffic_handler.task_handler.connection_handler.process_message(sample_conn_message)
    traffic_handler.task_handler.state_handler.process_message(sample_state_message)
    task_cleared, available_robots, checkpoints, \
        agv_itinerary, waitpoints, \
            wait_itinerary, landmark = traffic_handler.task_handler.fm_send_task_request(f_id=fleetname,
                                                                         r_id=robot_serial_number2,
                                                                         from_loc_id= 'C3', # 'C5',
                                                                         to_loc_id= 'C11', # 'C3',
                                                                         task_name='transport',
                                                                         task_priority='low',
                                                                         payload_kg=50.0)

    # simulate motion
    sample_state_message["orderId"] = "order_0"
    sample_state_message["serialNumber"] = robot_serial_number
    sample_state_message["lastNodeId"] = 'C11' # 'C11' # robot1_reserved_node # '' # robot1_reserved_node
    traffic_handler.task_handler.state_handler.process_message(sample_state_message)

    sample_state_message["orderId"] = "order_0"
    sample_state_message["serialNumber"] = robot_serial_number2
    sample_state_message["lastNodeId"] = 'C3' # 'C4' # robot2_reserved_node
    traffic_handler.task_handler.state_handler.process_message(sample_state_message)


    r_ids = traffic_handler.task_handler.factsheet_handler.fetch_serial_numbers(fleetname)
    print("r_ids: ", r_ids)
    # If serial numbers exist, fetch detailed data for the first serial number

    # Initialize the FmTrafficHandler too
    COUNT = 0
    while COUNT <= 10: # 3: #
        COUNT += 1
        print("count: ", COUNT)
        if r_ids:
            for _r_id in r_ids:
                traffic_handler.manage_traffic(f_id = fleetname, r_id = _r_id)
                if COUNT == 1:
                    sample_state_message["orderId"] = "order_0"
                    sample_state_message["serialNumber"] = robot_serial_number
                    sample_state_message["lastNodeId"] = 'C8' # 'C4' # robot2_reserved_node
                    traffic_handler.task_handler.state_handler.process_message(sample_state_message)

    # Close the database connection
    conn.close()










# # 6.8.2 States of predefined actions
# detectObject | - | Object detection is running. | - | Object has been detected. | AGV could not detect the object.
# finePositioning | - | AGV positions itself exactly on a target. | The fine positioning process is being paused, e.g., if a safety field is violated. <br>After removing the violation, the fine positioning continues. | Goal position in reference to the station is reached. | Goal position in reference to the station could not be reached.
# waitForTrigger | - | AGV is waiting for the trigger | - | Trigger has been triggered. | waitForTrigger fails, if order has been canceled.
# initPosition | - | Initializing of the new pose in progress (confidence checks, etc.). <br>If the AGV supports an instant transition, this state can be omitted. | - | The pose is reset. <br>The AGV reports <br>.agvPosition.x = x, <br>.agvPosition.y = y, <br>.agvPosition.theta = theta <br>.agvPosition.mapId = mapId <br>.agvPosition.lastNodeId = lastNodeId | The pose is not valid or cannot be reset. <br>General localization problems should correspond with an error.
# | downloadMap | Initialize the connection to the map server. | AGV is downloading the map, until download is finished. | - | AGV updates its state by setting the mapId/mapVersion and the corresponding mapStatus to 'DISABLED'. | The download failed, updated in vehicle state (e.g., connection lost, Map server unreachable, mapId/mapVersion not existing on map server). |
# | enableMap | - | AGV enables the map with the requested mapId and mapVersion while disabling other versions with the same mapId. | - | The AGV updates the corresponding mapStatus of the requested map to 'ENABLED' and the other versions with same mapId to 'DISABLED'. | The requested mapId/mapVersion combination does not exist.|
# | deleteMap | - | AGV deletes map with requested mapId and mapVersion from its internal memory. | - | AGV removes mapId/mapVersion from its state. | Can not delete map, if map is currently in use. The requested mapId/mapVersion combination was already deleted before. |
# stateRequest | - | - | - | The state has been communicated | -
# logReport | - | The report is in generating. <br>If the AGV supports an instant generation, this state can be omitted. | - | The report is stored. <br>The name of the log will be reported in status. | The report can not be stored (e.g., no space).
# factsheetRequest | - | - | - | The factsheet has been communicated | -

        ######
#        C17 ---> W17 C17 C18
#        C18 ---> C17
#        traffic ---> W17, C17, ---> W17, C17, C18

# use time of last received order to know if its feasible to send a new target
#  diffamr goes to c17 but
#  while yet to get there, a new order is given for it to go to C13
#  this order is rejected but the manager does not know
#  so while the robot goes to C17, manager thinks its headed to C13
#  when robot gets to C17 successfully, manager assumes it has reached C13 and gives it a goal to C10.
#  essentially the actual C13 is thus skipped.

# todo
# if checkps[0] startswit w
# take real horizon, we never gonna be read until we reach last node anyway
# fb_rec[waitpointcase] = bool
# if red
# reserved_checkpoint == next_stop_id and not waitpointcase:

#  For the other robot,
#  while it goes to all these "3" goals. since node_states exists, its base goes from W17
#  to C17 and to C18 however, its horizon is C17 and C18 because, manager does not give it a new goal.
#  this implies that, its horizon is always occupied since its in the traffic.
#  and as such it waits forever without receiving any new orders

# we only have red, if no node_states implying a want for a new node.