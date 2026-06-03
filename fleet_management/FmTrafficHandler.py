#!/usr/bin/env python3
import logging
logger = logging.getLogger(__name__)

import psycopg2, psycopg2.extras
import time, yaml, os, math, re, sys, datetime
from pathlib import Path

from FmTaskHandler import FmTaskHandler
from order import RoutePlan

from dataclasses import dataclass
from collections import defaultdict
from typing import DefaultDict, Any, Optional, List, Dict, Tuple, Union, cast




@dataclass
class RobotContext:
    robot_id: Any = None  
    version: Any = None  
    manufacturer: Any = None  
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
    merged_poses: Any = None
    dock_action_done: Any = None
    pending_dock_action: Any = None
    halt: Any = None
    speed_min: Any = None
    agv_size: Any = None
    vel_lin_ang: Any = None
    num_released: Any = None










class FmTrafficHandler():
    """
    Traffic negotiation core.

    This rewrite keeps the logic explicit:
        - scenario 1: no conflict
        - scenario 2: standard conflict with waitpoint + escape
        - scenario 3: proactive yield with CY node
    """
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

        self.temp_robot_delay_time: Dict[str, Tuple[float, float]] = {}
        self.wait_time_default = 10.5 # [secs]
        self.mutex_groups = [] # Store list of lists e.g. [['C1', 'C2'], ['C10', 'C11']]
        self.collision_tracker = 0
        self.robots_in_collision = set()

        # online_robots lives here (traffic management layer) not in StateSubscriber (data layer).
        # Semantics: "robot is a live participant in the current fleet session".
        # Populated by FmMain.on_mqtt_message whenever a state message arrives.
        self.online_robots: set = set()  # r_ids that have published at least one state
        self.traffic_control_dict: dict = {} # Shadow variable for visualization

    # --------------------------------------------------------------

    def fetch_mex_data(self, f_id: str, r_id: Optional[str] = None, m_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Unified fleet and robot data fetcher.
        This version replaces the 3 prior call styles and returns everything at once:
            - Data for the current robot (r_id)
            - Data for its contending robot (mex) occupying next_stop_id
            - Fleet-wide traffic_control list
            - Unassigned tasks list
        """

        mex_record = {}
        unassigned = []
        traffic_control_dict: dict[str, list[str]] = {}

        # -----------------------------------------------
        # Build robot_states from in-memory cache.
        # robot_state_cache is populated by on_mqtt_message every time a robot
        # publishes a state message — zero DB round-trips needed for state reads.
        # This makes state management cost O(1) per robot regardless of fleet size,
        # instead of the previous O(N) DB table scan.
        # -----------------------------------------------
        state_cache = self.task_handler.state_handler.cache # { r_id: raw_mqtt_state_msg }

        # Fall back to DB scan only if the cache is completely empty (cold start / first boot).
        if not state_cache:
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
                    # Reconstruct a minimal raw-state_cache-like dict from DB columns.
                    state_cache[sn] = {
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
        # logger.debug("ORDER: ",order_recs)
        if not state_cache or not order_recs:
            logger.warning("No state or order records to dissect.")
            return {
                "robot_data": {},
                "mex_data": {},
                "traffic_control_dict": {},
                "unassigned": []
            }

        # -----------------------------------------------
        # Pass 1: Parse STATE from in-memory cache
        # Each state_cache entry is the raw MQTT state payload (dict with camelCase keys).
        # -----------------------------------------------
        
        robot_states: Dict[str, Dict[str, Any]] = {}
        for serial_number, raw_msg in state_cache.items():
            try:
                agv_position  = raw_msg.get("agvPosition") or {}
                errors        = raw_msg.get("errors") or []
                action_states = raw_msg.get("actionStates") or []
                velocity      = raw_msg.get("velocity") or {}
                maps          = raw_msg.get("maps") or []
                node_states   = raw_msg.get("nodeStates") or []
                safety_state  = raw_msg.get("safetyState") or {}
                state_timestamp = raw_msg.get("timestamp", "")
                state_base          = raw_msg.get("lastNodeId", "")
                state_update_id     = raw_msg.get("orderUpdateId", 0)

                record = {}
                record["version"]       = raw_msg.get("version", "")
                record["manufacturer"]  = raw_msg.get("manufacturer", "")
                record["serial_number"] = serial_number
                record["order_update_id"] = state_update_id
                record["base"]    = state_base.split(',')[0] if state_base else ''
                record["position"]      = [agv_position.get('x', 0.0),
                                          agv_position.get('y', 0.0),
                                          agv_position.get('theta', 0.0)]
                record["errors"]        = [err.get("errorType", "") for err in errors]

                lin_vel = velocity.get("vx", 0.0)
                ang_vel = velocity.get("omega", 0.0)
                record["lin_vel"] = float(lin_vel) if str(lin_vel).replace('.', '', 1).lstrip('-').isdigit() else 0.0
                record["ang_vel"] = float(ang_vel) if str(ang_vel).replace('.', '', 1).lstrip('-').isdigit() else 0.0

                record["active_map"] = self.process_maps(maps)
                # logger.warning(#     f"{serial_number}: active map {record['active_map']}, and uploaded maps {maps}.",
                #     "FmTrafficHandler",
                #     "fetch_mex_data",
                #     "warn")

                e_stop = safety_state.get("eStop", "NONE")
                record["halt"] = False
                if e_stop == "MANUAL" or (serial_number in self.task_handler.pause_list) \
                        or (serial_number in self.task_handler.ignore_list):
                    record["halt"] = True

                # has_order_minute_passed = self.check_minute_passed(state_timestamp, 3.0)
                # if has_order_minute_passed:
                #     logger.warning(f"Message too old for {serial_number}: {state_timestamp}.")
                #     record["halt"] = True 

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
                record["merged_poses"]      = []
                record["dock_action_done"]  = False

                robot_states[serial_number] = record

            except Exception as err:
                logger.warning(f"State parsing error for record: {err}")
                continue

        # -----------------------------------------------
        # Pass 2: Get AGV dimensions and speed from FACTSHEET cache
        # factsheet_cache holds the raw MQTT factsheet payload, populated on first
        # factsheet message arrival — no DB read needed after that.
        # -----------------------------------------------
        for serial_number, state_rec in robot_states.items():
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
                logger.warning(f"No factsheet for {serial_number}.")
                continue
            state_rec["speed_min"] = float(speed_min)
            state_rec["agv_size"]  = [float(agv_width), float(agv_length)]

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
        order_update_id_index = self.task_handler.order_handler.table_col.index('order_update_id')
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
                order_update_id = order_rec[order_update_id_index+1]

                if serial_number not in robot_states:
                    continue


                # logger.info(f"\033[95m[DEBUG] R01 raw nodes from order_rec: {[n['nodeId'] for n in sorted(nodes, key=lambda x: x['sequenceId'])]}\033[0m")

                state_rec = robot_states[serial_number]
                state_rec["order_timestamp"] = timestamp
                state_rec["header_id"] = header_id
                state_rec["order_id"] = order_id.split(',')[0]
                state_rec["master_order_update_id"] = order_update_id

                waypoints = [
                    (node['nodeId'], node['nodePosition']['x'], node['nodePosition']['y'],
                    node['nodePosition']['theta'], node['released'], node.get('actions', []),
                    node['nodeDescription'])
                    for node in sorted(nodes, key=lambda x: x['sequenceId'])
                ]

                node_ids = []
                node_poses = []
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
                    node_poses.append(node_loc)
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
                    if release_state \
                        and (not serial_number.startswith("unassigned_")) \
                            and (serial_number not in self.task_handler.ignore_list):

                        # Auto-create the list for the robot if it's the first time we see it
                        traffic_control_dict.setdefault(serial_number, [])

                        # Now you don’t need setdefault
                        if node_id not in cast(List[str], traffic_control_dict[serial_number]):
                            cast(List[str], traffic_control_dict[serial_number]).append(node_id)

                        # [Spoofing] Also protect the corresponding physical checkpoint if a Waitpoint is occupied
                        if node_id.startswith('W'):
                            numeric_part = ''.join(filter(str.isdigit, node_id))
                            corresponding_c = f'C{numeric_part}'
                            if corresponding_c not in cast(List[str], traffic_control_dict[serial_number]):
                                cast(List[str], traffic_control_dict[serial_number]).append(corresponding_c)

                # if serial_number == "R01":
                # logger.info(f"\033[95m[DEBUG] R01 checkps: {checkps} | checkps_release: {checkps_release}\033[0m")

                # # check if the robot is getting late i.e. current_time > est_arrival_time:
                # # Extract ETA from node description and check if robot is late
                # eta_str = self._extract_eta_from_description(description)
                # if eta_str and self.check_robot_arrival(eta_str):
                #     logger.error(#         f"Robot {serial_number} is running late.",
                #         "FmTrafficHandler",
                #         "fetch_mex_data",
                #         "warn")

                # get merged nodes
                state_rec["merged_nodes"] = node_ids
                state_rec["merged_poses"] = node_poses
                state_rec["dock"] = ['low']
                state_rec["dock_action_done"] = False
                state_rec["c_pnts"] = checkps
                state_rec["c_pnts_pos"] = c_pnts_pos
                state_rec["w_pnts"] = waitps
                state_rec["w_pnts_pos"] = w_pnts_pos
                
                # Keep orders that have not been assigned for later.
                if serial_number.startswith("unassigned_"):
                    # Ensure correct dictionary access based on provided keys.
                    # logger.debug("1:",order_actions_list[0][0],"2:",order_actions_list[0][0]['actionParameters'][0]['value'])
                    # Fetch the desired 'landmark' value.
                    landmark_value = order_actions_list[0][0]['actionParameters'][0]['value']
                    unassigned.append((serial_number, timestamp, landmark_value))

                # since we dont want to issue more node targets, if true.
                state_rec["halt"] = order_id.endswith("_cancelled") or order_id.endswith("_completed")

                # agv_status: 'red'   = robot has BOTH reported AND physically reached its last released node.
                #             'green' = still travelling (either state is stale or GPS not settled yet).
                # Two conditions are required to prevent false positives:
                #   1. state_base == last_released : robot's lastNodeId matches what master released.
                #   2. dist <= threshold           : GPS confirms robot is physically at that node.
                # Condition 1 alone fires while robot is still travelling away from the node.
                # Condition 2 alone can't distinguish which node the robot should be at.
                state_base = state_rec["base"]
                agv_pos    = state_rec["position"] # [0.0, 0.0, 0.0, 0.0]
                agv_sz     = state_rec["agv_size"] # [width, length]

                last_released = next(
                    (n for n, r in zip(reversed(checkps), reversed(checkps_release)) if r),
                    None
                )

                released_coord = None
                if last_released in waitps:
                    released_coord = w_pnts_pos[waitps.index(last_released)]
                elif last_released in checkps:
                    released_coord = c_pnts_pos[checkps.index(last_released)]

                within_range = False
                if released_coord:
                    dist_to_released = math.sqrt((float(released_coord[0]) - float(agv_pos[0]))**2 + \
                            (float(released_coord[1]) - float(agv_pos[1]))**2)
                    within_range = dist_to_released <= (1.5 * float(agv_sz[1]))

                state_update_id = state_rec.get("order_update_id", 0)
                master_update_id = state_rec.get("master_order_update_id", 0)

                # logic: robot is 'red' (ready for more) ONLY if it has acknowledged the current grant,
                # is physically at the last released node, and reports that node as its base.
                # OR it's a new order start that hasn't acknowledged yet.
                is_acknowledged = (state_update_id == master_update_id)
                order_base = checkps[0] if checkps else None
                
                # logic: robot is 'red' if:
                # 1. It is at the last released node AND acknowledged update is pending (not is_acknowledged)
                # 2. It is a new order (last_released is None) and it's not yet at the starting node.
                is_new_order_start = (last_released is None and order_base is not None and state_base != order_base)
                is_at_last_released = (last_released is not None and state_base == last_released and within_range)

                state_rec["agv_status"] = "red" if ((is_at_last_released or is_new_order_start)) else "green" # and not is_acknowledged

                if not state_base and checkps: state_rec["base"] = checkps[0]
                
                state_rec["num_released"] = sum(1 for r in checkps_release if r)

                num_released = state_rec["num_released"] 
                start_idx = num_released-1 if num_released > 1 else num_released
                state_rec["horizon"] = checkps[start_idx:] 
                state_rec["horizon_release"] = checkps_release[start_idx:] 

                # logger.debug(f"1. Rbt : {serial_number} : -> state_base : {state_base} & last_released : {last_released} & within_range : {within_range}.") # & acknowledged : {is_acknowledged}.
                # logger.debug(f"       : -> horizon : {state_rec['horizon'] }. --> n_r: {num_released}") 

                # Process docking actions
                dock_action = next((a[0] for a in order_actions_list if a and a[0].get("actionType") == "dock"),None)
                if dock_action:
                    state_rec["dock"] = dock_action.get("actionParameters", [{}])[0].get("value", [])
                    state_rec["pending_dock_action"] = dock_action
                    for s in state_rec.get("action_states", []):
                        if s.get("actionId") == dock_action.get("actionId"):
                            status = s.get("actionStatus", "")
                            if status == "FINISHED":
                                state_rec["dock_action_done"] = True
                            elif status == "FAILED":
                                state_rec["halt"] = True
                                logger.warning(f"Robot {serial_number} dock failed.")
                            break

            except Exception as err:
                logger.warning(f"Order parsing error: {err}")
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
            for serial_number, state_rec in robot_states.items():
                if state_rec.get("base") == next_stop_id:
                    mex_record = state_rec
                    break

        # -----------------------------------------------
        # --- ANALYTICS: COLLISION TRACKING ---
        # ----------------------------------------------- 
        # "what matters is that one reserves and the other waits or yield. 
        # problem or collision arrises if both are granted the node and the distance between them is less than 1.5m."
        node_to_robots = {}
        for rid, nodes in traffic_control_dict.items():
            for n in nodes:
                node_to_robots.setdefault(n, []).append(rid)
        self.robots_in_collision = set()
        for node, rids in node_to_robots.items():
            if node.startswith('C') and len(rids) > 1:
                w = f"W{''.join(filter(str.isdigit, node))}"
                if not any(w in traffic_control_dict[r] for r in rids):
                    self.robots_in_collision.update(rids)

        # -----------------------------------------------
        # Clean up and return
        # -----------------------------------------------
        # Standardize traffic_control_dict: remove empty nodes/records and filter out empty robot entries
        traffic_control_dict = {
            rid: [node for node in nodes if node and node.lower() != 'none']
            for rid, nodes in traffic_control_dict.items()}

        traffic_control_dict = {rid: nodes for rid, nodes in traffic_control_dict.items() if nodes}

        return {
            "robot_data": robot_states.get(r_id, {}),
            "mex_data": mex_record,
            "traffic_control_dict": traffic_control_dict,
            "unassigned": unassigned
        }

    # --------------------------------------------------------------    

    def _fetch_current_robot_data(self, f_id: str, r_id: str, m_id: str) -> Tuple[Dict[str, List[str]], Any, Any, Optional[RobotContext]]:
        """
            Fetch and update data for a specific robot.
            Executes a SQL query to fetch robot data from the database based on the fleet and robot IDs.
            Maps the fetched data to corresponding class variables, including:
                Serial number, maps, order ID, last node ID, driving and paused states, node and edge states, position, velocity, battery state, errors, and additional information.
            Processes special AGV information such as active map, waypoints, status, and configuration.
            Handles exceptions if there is a database error or no data is found.
        """
        # Always reset first — prevents stale data from a prior robot (or prior
        # successful cycle) from surviving into this robot's failed-fetch path.
        fb_rec = self.fetch_mex_data(f_id, r_id=r_id, m_id=m_id)

        if not fb_rec or not fb_rec.get("robot_data"):
            return [], [], {}, None

        try:
            # write to class variables for active robot params
            r_ctx = RobotContext(
                robot_id                =   fb_rec["robot_data"]["serial_number"],
                version                 =   fb_rec["robot_data"]["version"],
                manufacturer            =   fb_rec["robot_data"]["manufacturer"],
                agv_position            =   fb_rec["robot_data"]["position"],
                base                    =   fb_rec["robot_data"]["base"],
                horizon                 =   fb_rec["robot_data"]["horizon"],
                horizon_release         =   fb_rec["robot_data"]["horizon_release"],
                checkpoints             =   fb_rec["robot_data"]["c_pnts"],
                waitpoints              =   fb_rec["robot_data"]["w_pnts"],
                checkp_itinerary        =   fb_rec["robot_data"]["c_pnts_pos"],
                waitp_itinerary         =   fb_rec["robot_data"]["w_pnts_pos"],
                num_released            =   fb_rec["robot_data"].get("num_released", []),
                agv_status              =   fb_rec["robot_data"]["agv_status"],
                order_timestamp         =   fb_rec["robot_data"]["order_timestamp"],
                merged_nodes            =   fb_rec["robot_data"]["merged_nodes"],
                merged_poses            =   fb_rec["robot_data"]["merged_poses"],
                dock_action_done        =   fb_rec["robot_data"]["dock_action_done"],
                pending_dock_action     =   fb_rec["robot_data"].get("pending_dock_action", None),
                halt                    =   fb_rec["robot_data"]["halt"],
                errors                  =   fb_rec["robot_data"]["errors"],
                active_map_name         =   fb_rec["robot_data"]["active_map"],
                landmarks               =   fb_rec["robot_data"]["dock"],
                order_id                =   fb_rec["robot_data"]["order_id"],
                header_id               =   fb_rec["robot_data"]["header_id"],
                agv_size                =   [fb_rec["robot_data"]["agv_size"][0], fb_rec["robot_data"]["agv_size"][1]],
                speed_min               =   fb_rec["robot_data"]["speed_min"],
                vel_lin_ang             =   [fb_rec["robot_data"]["lin_vel"], fb_rec["robot_data"]["ang_vel"]]
            )
            # others
            mex_data = fb_rec.get("mex_data", {})
            mex_ctx = RobotContext(
                robot_id                =   mex_data.get("serial_number"),
                version                 =   mex_data.get("version"),
                manufacturer            =   mex_data.get("manufacturer"), 
                agv_position            =   mex_data.get("position"),
                base                    =   mex_data.get("base"),
                horizon                 =   mex_data.get("horizon", []),
                horizon_release         =   mex_data.get("horizon_release", []),
                checkpoints             =   mex_data.get("c_pnts", []),
                waitpoints              =   mex_data.get("w_pnts", []),
                checkp_itinerary        =   mex_data.get("c_pnts_pos", []),
                waitp_itinerary         =   mex_data.get("w_pnts_pos", []),
                num_released            =   mex_data.get("num_released", []),
                agv_status              =   mex_data.get("agv_status"),
                order_timestamp         =   mex_data.get("order_timestamp"),
                merged_nodes            =   mex_data.get("merged_nodes", []),
                merged_poses            =   mex_data.get("merged_poses", []),
                dock_action_done        =   mex_data.get("dock_action_done"),
                pending_dock_action     =   mex_data.get("pending_dock_action", None),
                halt                    =   mex_data.get("halt"),
                errors                  =   mex_data.get("errors", []),
                active_map_name         =   mex_data.get("active_map"),
                landmarks               =   mex_data.get("dock", []),
                order_id                =   mex_data.get("order_id"),
                header_id               =   mex_data.get("header_id"),
                agv_size                =   mex_data.get("agv_size", [0.0, 0.0]) if "agv_size" in mex_data else [0.0, 0.0],
                speed_min               =   mex_data.get("speed_min", 0.0),
                vel_lin_ang             =   [mex_data.get("lin_vel", 0.0), mex_data.get("ang_vel", 0.0)]
            )
            unassigned = fb_rec["unassigned"]
            # convert traffic_control_dict to traffic_control list.
            self.traffic_control_dict = fb_rec["traffic_control_dict"]
            traffic_control = list(set(node for nodes in self.traffic_control_dict.values() for node in nodes))
        except KeyError as key_err:
            logger.warning(f"Missing key {key_err} in record.")
            return [], [], {}, None
        except (TypeError, ValueError) as specific_err:
            logger.warning(f"Type or Value Error: {specific_err}.")
            return [], [], {}, None
        return traffic_control, unassigned, mex_ctx, r_ctx

    # --------------------------------------------------------------

    def manage_traffic(self, f_id=None, r_id=None, m_id=None, v_id=None):
        """
        Entry point called in a loop. Iterates over serial numbers to fetch 
        and process current robot data, update traffic control, and handle 
        robot traffic status.
        """
        traffic_control: List[str] = []
        unassigned = None
        r_ctx = None

        f_id = f_id or self.fleetname
        m_id = m_id or self.manufacturer
        v_id = v_id or self.version

        if r_id is None:
            return traffic_control, unassigned, r_ctx

        try:
            traffic_control, unassigned, mex_ctx, r_ctx = self._fetch_current_robot_data(f_id, r_id, m_id)
            if (r_ctx is not None) and r_ctx.horizon:
                self._handle_robot_traffic_status(f_id, r_id, m_id, v_id, traffic_control, mex_ctx, r_ctx)
        except (ValueError, TypeError) as error:
            logger.info(f"{error}..")
            return traffic_control, unassigned, r_ctx

        return traffic_control, unassigned, r_ctx

    # --------------------------------------------------------------

    def _handle_robot_traffic_status(self, f_id, _r_id, m_id, v_id,
        traffic_control: list[str], mex_ctx, r_ctx):
        """
        Resolve one robot's traffic status with a small, explicit decision tree.  
        """
        # --------------------------------------------------
        # 0. Define key nodes
        # --------------------------------------------------
        reserved = r_ctx.base
        next_stop = r_ctx.horizon[0]

        # --------------------------------------------------
        # 1. MUTEX EXPANSION
        # --------------------------------------------------
        expanded = set(traffic_control)
        for group in self.mutex_groups:
            if any(n in traffic_control and n != reserved for n in group):
                expanded.update(group)

        # --------------------------------------------------
        # 2. STATE FLAGS (single source of truth)
        # --------------------------------------------------
        # Count how many upcoming path positions are currently authorized
        is_ready = r_ctx.agv_status == "red"
        is_notdocked = reserved in r_ctx.landmarks and not r_ctx.dock_action_done
        is_released = not r_ctx.horizon_release[0] or \
            (r_ctx.horizon_release[0] and (reserved == next_stop))
        is_fresh = self.check_minute_passed(r_ctx.order_timestamp, 0.15)
        is_active = not r_ctx.halt
        reserved_type = "Dock" if reserved in r_ctx.landmarks else "Checkpoint"

        can_progress = all([is_ready, is_released, is_active, is_fresh])

        # --------------------------------------------------
        # 3. DEBUG LOG
        # --------------------------------------------------
        # if _r_id == "R02":
        msg = (
            f"\nR_id: {_r_id}\n"
            f"[{reserved_type}] Reserved: {reserved} → Next: {next_stop}\n"
            f"Status: {r_ctx.agv_status} | Traffic: {next_stop in expanded} | Docked: {not is_notdocked}\n"
            f"Horizon: {r_ctx.horizon} | Can_progress: {can_progress} | n_r: {r_ctx.num_released}\n"
        )
        logger.info(msg)

        # --------------------------------------------------
        # 4. MAIN DECISION TREE (Original flow preserved)
        # --------------------------------------------------
        try:
            # ------------------------------
            # CASE 1: NOT READY TO MOVE
            # ------------------------------
            if not can_progress:
                logger.debug("\033[93m DEBUG ----------------- 🛸 8 🛸 ----------------- \033[0m")
                return

            # ------------------------------
            # CASE 2: DOCK NOT COMPLETE
            # ------------------------------
            if is_notdocked:
                self._handle_dock_reminder(f_id, _r_id, m_id, v_id, r_ctx)
                return

            # ------------------------------------------------------------------
            # CASE 3: CONFLICT (next stop occupied and no ongoing negotiation)
            # ------------------------------------------------------------------
            if (next_stop in expanded) and not (r_ctx.num_released > 1):
                logger.info(f"Robot {_r_id}: {next_stop} occupied.")

                # --- LAST MILE SPECIAL CASE ---
                if (next_stop in r_ctx.landmarks and 
                    len(r_ctx.horizon) == 1 and 
                    r_ctx.landmarks[2] != "loop"):    
                    logger.debug("\033[93m DEBUG ----------------- 🛸 1 🛸 ----------------- \033[0m")
                    r_plan = self._handle_last_mile_conflict_case(f_id, _r_id, m_id, v_id, expanded, r_ctx)
                    if r_plan is not None:
                        self.publish_route_plan(r_plan)
                    return

                # --- DETERMINE MEX AND THREAT LEVEL---
                if (not mex_ctx.robot_id) or (not self._handle_conflict_threat_level(mex_ctx, r_ctx)):
                    logger.info(f"{_r_id}: occupant unknown → holding.")
                    return

                # Try upgraded conflict strategies in order of preference
                relocate = self._handle_conflict_relocate_plans(f_id, _r_id, m_id, v_id, 
                                                               expanded, mex_ctx, r_ctx)
                logger.debug("\033[93m DEBUG ----------------- 🛸 2 🛸 ----------------- \033[0m")
                if relocate and any(relocate):
                    r_plan, mex_plan = relocate
                    self.publish_route_plan(r_plan)
                    self.publish_route_plan(mex_plan)
                    return

                reroute = self._handle_conflict_reroute_plans(f_id, _r_id, m_id, v_id, 
                                                             expanded, mex_ctx, r_ctx)
                logger.debug("\033[93m DEBUG ----------------- 🛸 3 🛸 ----------------- \033[0m")
                if reroute and any(reroute):
                    r_plan, mex_plan = reroute
                    self.publish_route_plan(r_plan)
                    self.publish_route_plan(mex_plan)
                    return

                redirect = self._handle_conflict_redirect_plans(f_id, _r_id, m_id, v_id, 
                                                               expanded, mex_ctx, r_ctx)
                logger.debug("\033[93m DEBUG ----------------- 🛸 4 🛸 ----------------- \033[0m")  
                if redirect and any(redirect):
                    r_plan, mex_plan = redirect
                    self.publish_route_plan(r_plan)
                    self.publish_route_plan(mex_plan)
                    return

                # Fallback if nothing worked
                logger.warning(f"Robot {_r_id}: Conflict at {next_stop} but no resolution strategy succeeded.")
                return

            # ---------------------------------------------------------
            # CASE 4: NO CONFLICT → Normal plan
            # ---------------------------------------------------------
            logger.debug("\033[93m DEBUG ----------------- 🛸 5 🛸 ----------------- \033[0m")
            plan = self._handle_regular_plan(f_id, _r_id, m_id, v_id, r_ctx)
            self.publish_route_plan(plan)

        except (ValueError, TypeError) as err:
            logger.warning(f"Error in traffic logic: {err}")

    # --------------------------------------------------------------

    # ======================================================================== #
    # ACTION & DOCK ROUTING DISPATCHERS                                        #
    # ======================================================================== #

    # --------------------------------------------------------------

    def _handle_node_action(self, f_id, _r_id, m_id, v_id, header_id, base, dock_action_done, landmark):
        """Dispatches operational immediate fleet instant action messages post-dock verification."""
        action_type, duration = None, "2.0"
        
        if any(base == d for d in landmark) and dock_action_done:
            task = landmark[2]
            if task in ["transport", "loop"] and base in (landmark[3], landmark[4]):
                action_type = "pick" if base == landmark[3] else "drop"
            elif task == "charge" and base in landmark[3:]:
                action_type, duration = "startCharging", "2200.0"
        elif self.task_handler.get_node_description(f_id, base) == 'door':
            action_type = "waitForTrigger"

        if action_type:
            action = self.task_handler.instant_actions_handler.create_action(action_type, {"key": "duration", "value": duration})
            self.task_handler.instant_actions_handler.build_instant_action_msg(f_id, _r_id, header_id, v_id, m_id, [action])

    # --------------------------------------------------------------

    def check_available_last_mile_dock(self, traffic_control: list[str], task_dict, start_idx, r_ctx) -> bool:
        """Evaluates remaining unallocated destination infrastructure for pathing optimization."""
        graph = self.task_handler.build_graph(task_dict)
        logger.info(f"checking docks {r_ctx.landmarks[start_idx:]} for availability.")

        for dock in r_ctx.landmarks[start_idx:]:
            if dock not in traffic_control:
                paths = self.task_handler.fm_shortest_paths(r_ctx.base, dock, graph)
                logger.info(f"start {r_ctx.base}, target {dock}, paths --> {paths}.")
                if paths and paths[0]:
                    r_ctx.checkpoints = r_ctx.horizon = paths[0]
                    r_ctx.checkp_itinerary = self.task_handler.fm_get_itinerary(r_ctx.checkpoints, task_dict)
                    r_ctx.waitpoints = self.task_handler.fm_extract_unique_waitpoints(r_ctx.checkpoints, task_dict)
                    r_ctx.waitp_itinerary = self.task_handler.fm_get_itinerary(r_ctx.waitpoints, task_dict)
                    return True
        return False

    # --------------------------------------------------------------

    def _handle_last_mile_conflict_case(self, f_id, _r_id, m_id, v_id, traffic_control, r_ctx=None, task_dict=None):
        """Resolves edge infrastructure contention via programmatic target fallback redirection."""

        task_dict = task_dict or self.task_dictionary
        next_stop, task = r_ctx.horizon[0], r_ctx.landmarks[2]
        
        idx = 5 if task == "transport" else 3 if task in ["move", "charge"] else None
        available = False
        
        if idx and next_stop in r_ctx.landmarks[idx:] and len(r_ctx.landmarks[idx:]) > 1:
            available = self.check_available_last_mile_dock(traffic_control, task_dict, 4 if task == "transport" else 2, r_ctx)

        if available: # return the plan instead of calling _handle_regular_plan
            return self._handle_regular_plan(f_id, _r_id, m_id, v_id, r_ctx)
        else: # ... log ...
            logger.warning(f"robot {_r_id} Inactive: all docks occupied.")
            return None

    # --------------------------------------------------------------

    # ======================================================================== #
    # MUTEX MANAGEMENT ENGINE                                                  #
    # ======================================================================== #

    # --------------------------------------------------------------

    def _log_mutex(self, action, group):
        logger.info(f"{action} Mutex Group: {group}")

    # --------------------------------------------------------------

    def fm_add_mutex_groups(self, group_list):
        """Registers a strict network topology mutex group collection."""
        if isinstance(group_list, list) and group_list not in self.mutex_groups:
            self.mutex_groups.append(group_list)
            self._log_mutex("Added", group_list)

    # --------------------------------------------------------------

    def fm_remove_mutex_group(self, group_list):
        """Deregistrates a structural network topology mutex layer constraint."""
        if group_list in self.mutex_groups:
            self.mutex_groups.remove(group_list)
            self._log_mutex("Removed", group_list)

    # --------------------------------------------------------------

    # ======================================================================== #
    # OPERATIONAL STRIPS & MATHEMATICAL CONVERSIONS                            #
    # ======================================================================== #

    # --------------------------------------------------------------

    def euler_to_quaternion(self, yaw, pitch, roll) -> list[float]:
        """Maps standard kinematic coordinate spatial configurations efficiently."""
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return [sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy]

    # --------------------------------------------------------------

    def process_maps(self, maps) -> str:
        """Determines active context map configurations using standard evaluation lookup."""
        return next((m["mapId"] for m in maps if m["mapStatus"] == "ENABLED"), None)

    # --------------------------------------------------------------

    def get_map(self, f_id_, b_node, h_node) -> str:
        """Fetches dynamic situational active identifier topologies matching vertical travel."""
        is_elevator = self.task_handler.get_node_description(f_id_, b_node) == 'elevator'
        return self.task_handler.get_node_map(f_id_, h_node if is_elevator else b_node)

    # --------------------------------------------------------------

    def map_priority(self, priority) -> int:
        """Transforms structural prioritization semantics to sequential ordinal logic weights."""
        return {'low': 1, 'medium': 2, 'high': 3}.get(priority, 0)

    # --------------------------------------------------------------

    def check_waitpoint_association(self, node, mex_waitpoints) -> str:
        """Tracks operational numeric patterns validation for node alignment markers."""
        wp = f"W{''.join(filter(str.isdigit, node))}"
        return wp if wp in mex_waitpoints else None

    # --------------------------------------------------------------

    # ======================================================================== #
    # TEMPORAL METRICS                                                         #
    # ======================================================================== #

    # --------------------------------------------------------------

    def estimate_time_to_node(self, robot_pos, node_pos, min_vel) -> float:
        """Computes Euclidean target distance mapping metrics with specialized boundaries."""
        if float(min_vel) == 0.0: 
            return float('inf')
        t = math.hypot(float(node_pos[0]) - float(robot_pos[0]), \
            float(node_pos[1]) - float(robot_pos[1])) / float(min_vel)
        return t if t > 0.1 else getattr(self, 'wait_time_default', 0.1)

    # --------------------------------------------------------------

    def check_minute_passed(self, order_timestamp, minute) -> bool:
        """Tracks task execution lifespans relative to explicit baseline thresholds."""
        if not order_timestamp: 
            return False
        try:
            delta = datetime.datetime.now() - datetime.datetime.fromisoformat(str(order_timestamp))
            return delta.total_seconds() >= (minute * 60)
        except ValueError:
            logger.warning(f"Invalid timestamp format: {order_timestamp}.")
            return False

    # --------------------------------------------------------------

    # ======================================================================== #
    # PRIVATE UTILITY HELPERS (DRY PRINCIPLE)                                  #
    # ======================================================================== #

    # --------------------------------------------------------------

    def update_robot_elapsed_time(self, _r_id):
        """Tracks dynamic runtime cumulative execution tracking buffers safely inside maps."""
        now = time.time()
        curr_state = self.temp_robot_delay_time.get(_r_id)
        start, total = curr_state if isinstance(curr_state, tuple) else (0, 0)
        self.temp_robot_delay_time[_r_id] = (now, 0.1 if start == 0 else total + (now - start))

    # --------------------------------------------------------------

    def _sync_map_and_actions(self, f_id, r_id, m_id, v_id, ctx, target_node, log_level="warn") -> str:
        """Executes node actions and updates active map with variance tracking."""
        self.update_robot_elapsed_time(r_id)
        self._handle_node_action(f_id, r_id, m_id, v_id, ctx.header_id, ctx.base, ctx.dock_action_done, ctx.landmarks)
        map_name = self.get_map(f_id, ctx.base, target_node)
        
        if ctx.active_map_name is not None and ctx.active_map_name != map_name:
            logger.info(f"r_id: {r_id} elevator flag {map_name} and active map {ctx.active_map_name} problem.")
        return map_name

    # --------------------------------------------------------------

    def _build_route_plan(self, f_id, r_id, m_id, v_id, ctx, map_name, release_nodes, release_type=None, checkpoints=None, checkp_poses=None) -> RoutePlan:
        """Constructs a standard fleet RoutePlan structure uniformly."""
        order_id, update_id = self.task_handler.generate_new_order_id(ctx.order_id)
        return RoutePlan(
            fleet_id=f_id, robot_id=r_id, version=v_id, manufacturer=m_id,
            header_id=ctx.header_id + 1, order_id=order_id, update_id=update_id, map_name=map_name,
            checkp_nodes=checkpoints if checkpoints is not None else ctx.checkpoints,
            checkp_poses=checkp_poses if checkp_poses is not None else getattr(ctx, 'checkp_poses', getattr(ctx, 'checkp_itinerary', None)),
            waitp_nodes=ctx.waitpoints,
            waitp_poses=getattr(ctx, 'waitp_poses', getattr(ctx, 'waitp_itinerary', None)),
            landmarks=ctx.landmarks, release_nodes=release_nodes, release_type=release_type
        )

    # --------------------------------------------------------------

    def publish_route_plan(self, plan: RoutePlan):
        """Core public entrypoint. Processes and dispatches a complete RoutePlan."""

        logger.debug("\033[93m DEBUG ----------------- 🛸 6 🛸 ----------------- \033[0m")
        # print(f"\033[93m DEBUG ------- 🛸 {plan} 🛸 ------- \033[0m")
        logger.debug(f"\033[93m DEBUG ------- 🛸 {plan.checkp_nodes} 🛸 ------- \033[0m")
        logger.debug(f"\033[93m DEBUG ------- 🛸 {plan.release_nodes} 🛸 ------- \033[0m")
        

        # 1. Structural Trajectory Matrix Union
        merged_nodes, merged_poses = self.task_handler.order_handler._merge_trajectory_itinerary(plan)

        # 2. Reconstructive Trajectory Stitching (Handles Swaps & Loopbacks)
        active_nodes, active_poses = [], []
        consumed_indices = []

        # Extract explicit release cluster entries sequentially from first-seen pool
        for target in plan.release_nodes:
            try:
                idx = merged_nodes.index(target)
                consumed_indices.append(idx)
                active_nodes.append(merged_nodes[idx])
                active_poses.append(merged_poses[idx])
            except ValueError:
                raise ValueError(f"Required release node '{target}' missing from graph trajectory path.")

        # Determine where the untouched remainder of the path begins
        # Any node appearing prior to max_idx that wasn't consumed is cleanly discarded (e.g., C1)
        max_idx = max(consumed_indices)
        
        # Stitch the untouched remaining path tail to the reordered cluster
        active_nodes.extend(merged_nodes[max_idx + 1:])
        active_poses.extend(merged_poses[max_idx + 1:])

        # 3. Build VDA-5050 Structural Geometry Elements
        base_node, base_edge, horizon_nodes, horizon_edges = self.task_handler.order_handler.create_order(
            plan, active_nodes, active_poses)

        # 4. Package and Dispatch Message
        self.task_handler.order_handler.build_order_msg(plan, base_node, base_edge, horizon_nodes, horizon_edges)
        logger.debug("\033[93m DEBUG ----------------- 🛸 7 🛸 ----------------- \033[0m")

    # --------------------------------------------------------------

    def _generate_dual_plans(self, f_id, r_id, m_id, v_id, r_ctx, mex_ctx, r_goal, mex_goal):
        """Utility wrapper to synchronize maps and dispatch concurrent route plans."""
        r_map = self._sync_map_and_actions(f_id, r_id, m_id, v_id, r_ctx, r_ctx.horizon[0], "warn")
        mex_map = self._sync_map_and_actions(f_id, mex_ctx.robot_id, mex_ctx.manufacturer, mex_ctx.version, mex_ctx, mex_ctx.horizon[0], "error")
        
        r_plan = self._build_route_plan(f_id, r_id, m_id, v_id, r_ctx, r_map, r_goal)
        mex_plan = self._build_route_plan(f_id, mex_ctx.robot_id, mex_ctx.manufacturer, mex_ctx.version, mex_ctx, mex_map, mex_goal)
        return r_plan, mex_plan

    # --------------------------------------------------------------

    # ======================================================================== #
    # CORE ROUTING HANDLERS                                                    #
    # ======================================================================== #

    # --------------------------------------------------------------

    def _handle_regular_plan(self, f_id, r_id, m_id, v_id, r_ctx) -> RoutePlan:
        """Build the direct route for the no-conflict case."""
        map_name = self._sync_map_and_actions(f_id, r_id, m_id, v_id, r_ctx, r_ctx.horizon[0], "warn")
        idx = r_ctx.num_released - 1 if r_ctx.num_released > 1 else 0
        return self._build_route_plan(
            f_id, r_id, m_id, v_id, r_ctx, map_name,
            checkpoints=r_ctx.checkpoints[idx:],
            checkp_poses=r_ctx.checkp_itinerary[idx:],
            release_nodes=[r_ctx.base] if idx > 0 else [r_ctx.horizon[0]])

    # --------------------------------------------------------------

    def _handle_conflict_threat_level(self, mex_ctx, r_ctx):
        """Answers the question, is mex_ctx a threat to r_ctx or not?"""
        
        
        # Flattened validation guard block for conflict threat checks
        is_mex_threatening = (
            mex_ctx.agv_status == 'red' and
            (mex_ctx.base not in mex_ctx.landmarks or mex_ctx.dock_action_done) and
            not mex_ctx.halt and mex_ctx.horizon and
            not mex_ctx.num_released > 1 and 
            mex_ctx.horizon[0] == r_ctx.base)

        if not is_mex_threatening:
            logger.info(f"{r_ctx.robot_id} waiting for mex_r_id {mex_ctx.robot_id}. No immediate threat or active decision node reached.")
            if mex_ctx.halt:
                logger.warning(f"however, mex_r_id {mex_ctx.robot_id} is halted. Verify emergency protocols.")

        return is_mex_threatening

    # --------------------------------------------------------------

    def _handle_conflict_relocate_plans(self, f_id, r_id, m_id, v_id, traffic_control, mex_ctx, r_ctx):
        """Build the two routes used in the normal conflict case.
            r_id:  base -> CY -> target
            mex:   base -> waitpoint -> base/next
        """
        
        
        # 1. Setup & Logging
        logger.info(f"{r_id} started negotiation.")
        p1, p2 = self.map_priority(r_ctx.landmarks[1]), self.map_priority(mex_ctx.landmarks[1])

        mex_waitp = self.check_waitpoint_association(r_ctx.horizon[0], mex_ctx.waitpoints)
        r_waitp = self.check_waitpoint_association(r_ctx.base, r_ctx.waitpoints)

        # 2. Determine Outcome (Boolean Logic)
        if p1 == p2:
            logger.info("Equal task priorities. Evaluating operational delay time metric.")
            # Ensure keys exist, then compare times
            t1 = float(self.temp_robot_delay_time.get(r_id, (0, 0))[1])
            t2 = float(self.temp_robot_delay_time.setdefault(mex_ctx.robot_id, (0, 0))[1])
            is_higher = t1 > t2
        else:
            is_higher = p1 > p2

        # 3. Unified Dispatch (The "Superpower" Move)
        handler = self.handle_priority_higher if is_higher else self.handle_priority_lower
        temp_fb_wait_time, mex_wait_time = handler(r_id, traffic_control, mex_waitp, mex_ctx, r_waitp, r_ctx)

        if temp_fb_wait_time is None and mex_wait_time is None:
            logger.info(f"{r_id} & {mex_ctx.robot_id} stuck. Waitpoint unavailable. Human assistance required.")
            return None, None

        # 4. Build Dual Plans --> r_plan, mex_plan
        plans = []
        for ctx, w_time, waitp, rob_id, mfr, ver in [
            (r_ctx, temp_fb_wait_time, r_waitp, r_id, m_id, v_id),
            (mex_ctx, mex_wait_time, mex_waitp, mex_ctx.robot_id, mex_ctx.manufacturer, mex_ctx.version)]:
            logger.info(f"{rob_id} --> estimated wait time {w_time}.")
            c_map = self._sync_map_and_actions(f_id, rob_id, mfr, ver, ctx, ctx.horizon[0], "error")
            if w_time is not None and float(w_time) >= 0:
                w_time = float(w_time) + self.estimate_time_to_node(
                    ctx.agv_position, ctx.checkp_itinerary[ctx.checkpoints.index(ctx.horizon[0])], float(ctx.speed_min))
                goal = [waitp, ctx.base, ctx.horizon[0]]
            else:
                goal = [ctx.horizon[0], ctx.horizon[1]]         
            plans.append(self._build_route_plan(f_id, rob_id, mfr, ver, ctx, c_map, goal, w_time))

        return plans[0], plans[1] 

    # --------------------------------------------------------------

    def _handle_conflict_reroute_plans(self, f_id, r_id, m_id, v_id, traffic_control, mex_ctx, r_ctx, task_dict=None):
        """Build proactive routing utilizing graph alternate paths.
            r_id:  base -> CY -> target
            mex:   base -> waitpoint -> base/next
        """
        task_dict = task_dict or self.task_dictionary
        
        for active_ctx, other_ctx in [(r_ctx, mex_ctx), (mex_ctx, r_ctx)]:
            if not active_ctx or not active_ctx.horizon:
                continue

            # no-skip alternate: base -> horizon[0]
            plan = self._try_alternate_path(active_ctx, task_dict, traffic_control, target_index=0, replace_count=2)
            if plan is None and len(active_ctx.horizon) > 1 and active_ctx.horizon[0] not in active_ctx.landmarks:
                # skip alternate: base -> horizon[1] only if horizon[0] is NOT landmark       
                plan = self._try_alternate_path(active_ctx, task_dict, traffic_control, target_index=1, replace_count=3)

            if plan is not None:
                robot_name = r_id if active_ctx == r_ctx else f"{mex_ctx.robot_id} (mex)"
                logger.info(f"Fleet {f_id} <-> {robot_name} Reroute_status --> True (alternate path discovered).")

                # Apply structural modifications mapping out the alternate path sequence
                active_ctx.checkpoints, active_ctx.horizon = plan["checkpoints"], plan["horizon"]
                active_ctx.checkp_itinerary = plan["checkp_itinerary"]
                active_ctx.waitpoints, active_ctx.waitp_itinerary = plan["waitpoints"], plan["waitp_itinerary"]

                return self._generate_dual_plans(f_id, r_id, m_id, v_id, r_ctx, mex_ctx, [r_ctx.base, r_ctx.checkpoints[1]], [mex_ctx.base, mex_ctx.checkpoints[1]])

        logger.info(f"Fleet {f_id} <-> {r_id} Reroute_status --> False.")
        return None, None

    # -------------------------------------------------------------- 

    def _handle_conflict_redirect_plans(self, f_id, r_id, m_id, v_id, traffic_control, mex_ctx, r_ctx, task_dict=None):
        """Build proactive strategy inserting instant temporary waits and validating escape paths.
            r_id:  base -> CY -> target
            mex:   base -> waitpoint -> base/next
        """
        task_dict = task_dict or self.task_dictionary
        
        for active_ctx, other_ctx in [(r_ctx, mex_ctx), (mex_ctx, r_ctx)]:
            if not active_ctx or not active_ctx.horizon:
                continue

            cy_id, _, cy_pos = self._try_temp_wait(active_ctx, other_ctx, traffic_control, task_dict)
            if cy_id and self._verify_escape_chain(active_ctx, other_ctx, cy_id, traffic_control):
                robot_name = r_id if active_ctx == r_ctx else f"{mex_ctx.robot_id} (mex)"
                logger.info(f"Fleet {f_id} <-> {robot_name} Reroute_status --> True (temp wait + escape verified).")

                # State mutation injection for safe wait zones
                active_ctx.checkpoints.insert(1, cy_id)
                active_ctx.checkpoints.insert(2, active_ctx.base)
                active_ctx.checkp_itinerary.insert(1, cy_pos)
                active_ctx.checkp_itinerary.insert(2, active_ctx.checkp_itinerary[0])
                active_ctx.horizon[0:0] = [cy_id, active_ctx.base]

                # [CB, CY, CB, CX] | [CB CZ WZ CZ] or [CB CZ CV] depending on target active role contexts
                active_goal = [cy_id, active_ctx.base, active_ctx.horizon[2]] # 2 because 2 new nodes were inserted in idx [0, 1 -- >2]
                
                other_w_id = self.check_waitpoint_association(other_ctx.horizon[0], other_ctx.waitpoints)
                if len(other_ctx.horizon) > 1 and other_ctx.horizon[1] not in traffic_control:
                    other_goal = [other_ctx.horizon[0], other_ctx.horizon[1]]
                elif other_w_id:
                    other_goal = [other_ctx.horizon[0], other_w_id, other_ctx.horizon[0]]
                else:
                    continue

                if active_ctx == r_ctx:
                    r_goal, mex_goal = active_goal, other_goal
                else:
                    r_goal, mex_goal = other_goal, active_goal

                return self._generate_dual_plans(f_id, r_id, m_id, v_id, r_ctx, mex_ctx, r_goal, mex_goal)

        logger.info(f"Fleet {f_id} <-> {r_id} Reroute_status --> False.")
        return None, None

    # --------------------------------------------------------------

    def _try_alternate_path(self,r_ctx,task_dict,
        traffic_control,target_index: int,replace_count: int,):
        """
        Build an alternate path plan using the second shortest path.
        target_index:
            0 -> horizon[0]
            1 -> horizon[1]
        replace_count:
            2 -> replace [base, horizon[0]]
            3 -> replace [base, horizon[0], horizon[1]]
        """
        if len(r_ctx.horizon) <= target_index:
            return None

        start_node = r_ctx.base
        target_node = r_ctx.horizon[target_index]
        occupied = traffic_control

        paths = self.task_handler.fm_shortest_paths(start_node, target_node, task_dict)
        logger.info(f"Checking alternate paths for {r_ctx.robot_id} --> {paths}.")

        if len(paths) < 2:
            return None

        chosen_path = paths[1]
        if not chosen_path:
            return None

        # Only the first node is allowed to already be occupied by the robot itself.
        for node in chosen_path[1:]:
            if node != start_node and node in occupied:
                return None

        waitpoints = self.task_handler.fm_extract_unique_waitpoints(chosen_path, task_dict)
        wait_itinerary = self.task_handler.fm_get_itinerary(waitpoints, task_dict)

        merged_waitpoints, merged_wait_itinerary = self._merge_waitpoints(
            r_ctx,waitpoints,wait_itinerary,)

        return {
            "checkpoints": chosen_path + r_ctx.checkpoints[replace_count:],
            "checkp_itinerary": self.task_handler.fm_get_itinerary(chosen_path, task_dict)
            + r_ctx.checkp_itinerary[replace_count:],
            "waitpoints": merged_waitpoints,
            "waitp_itinerary": merged_wait_itinerary,
            "horizon": chosen_path + r_ctx.horizon[replace_count:],
        }

    # --------------------------------------------------------------

    def _try_temp_wait(self, r_ctx, mex_ctx, traffic_control, task_dict):
        """
        Finds a temporary waitpoint (a free neighbouring checkpoint) for the yielding robot.
        Rules:
        1. Node must be a 'checkpoint' (not dock).
        2. Must not block the other robot's horizon.
        3. Closer to the yielding robot's goal than other valid neighbours.
        4. Neither robot can be at a dock (both must be on a 'checkpoint').
        """

        occupied = traffic_control
        yielding_base_node = r_ctx.base
        yielding_goal_node = r_ctx.horizon[1] if len(r_ctx.horizon) > 1 else r_ctx.horizon[0]
        other_base_node = r_ctx.horizon[0]
        other_horizon_path = mex_ctx.horizon

        def is_checkpoint(node_id):
            node_desc = next((item['description'] for item in self.task_dictionary.get('itinerary', []) if item['loc_id'] == node_id), None)
            return node_desc == 'checkpoint'

        if not is_checkpoint(yielding_base_node): 
            logger.warning(f"YBN --> {yielding_base_node}.")
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

        logger.info(f"NEIGHBOR YIELD: Viable node for temporary wait, cy id --> {best_cy_id}.")
            
        if best_cy_id is not None:
            return best_cy_id, best_spoofed_wy, best_cy_itinerary[0]

        return None, None, None

    # --------------------------------------------------------------

    def _merge_waitpoints(self,r_ctx,new_waitpoints: List[str],
        new_wait_itinerary: List[Tuple[float, float, float]],):
        """
        Keep the old waitpoint behavior, but do it in one place.
        """
        if not new_waitpoints:
            return r_ctx.waitpoints, r_ctx.waitp_itinerary

        if r_ctx.horizon and self.check_waitpoint_association(r_ctx.horizon[0], r_ctx.waitpoints):
            return (new_waitpoints + r_ctx.waitpoints[1:],
                    new_wait_itinerary + r_ctx.waitp_itinerary[1:],)

        return (new_waitpoints + r_ctx.waitpoints,
                new_wait_itinerary + r_ctx.waitp_itinerary,)

    # --------------------------------------------------------------

    def _verify_escape_chain(self, yielding_ctx, other_ctx, cy_id, traffic_control):
        """
        Validate the full reservation chain before committing a temp-wait maneuver.

        When the yielding robot (R1) moves from its base (e.g. C2) to cy_id (e.g. C6),
        four conditions must all hold for the move to be safe:

          Check 1: cy_id is free — not reserved by any robot other than R1.
          Check 2: yielding_ctx.base (C2) is held only by R1 — it becomes free the
                   moment R1 departs, allowing other_ctx (R3) to occupy it next.
          Check 3: other_ctx (R3) has a clear escape beyond yielding_ctx.base —
                   i.e. other_ctx.horizon[1] (e.g. C50 / W2) is not reserved by
                   a third robot.
          Check 4: other_ctx.base (R3's current node, e.g. C1) is free of third
                   robots — R1 must be able to return through it eventually.

        Returns True only when all four conditions hold; False otherwise.
        """
        tc_dict = self.traffic_control_dict

        # Check 1: cy_id not already taken by any other robot
        for rid, nodes in tc_dict.items():
            if rid != yielding_ctx.robot_id and cy_id in nodes:
                return False

        # Check 2: yielding base not shared with any other robot
        for rid, nodes in tc_dict.items():
            if rid != yielding_ctx.robot_id and yielding_ctx.base in nodes:
                return False

        # Check 3: other robot has a clear escape beyond yielding_ctx.base
        if other_ctx and other_ctx.horizon:
            other_escape = other_ctx.horizon[1] if len(other_ctx.horizon) > 1 else None
            if other_escape:
                for rid, nodes in tc_dict.items():
                    if rid != other_ctx.robot_id and other_escape in nodes:
                        return False

        # Check 4: other robot's current base not taken by a third robot
        #           (R1 must be able to route back through it eventually)
        if other_ctx and other_ctx.base:
            for rid, nodes in tc_dict.items():
                if rid not in (yielding_ctx.robot_id, other_ctx.robot_id) and other_ctx.base in nodes:
                    return False

        return True

    # --------------------------------------------------------------

    def _has_escape(self, horizon, own_id):
        """Validates if the immediate next node (escape route) is available."""
        if not horizon or len(horizon) <= 1:
            return True
        escape_node = horizon[1]
        for occupant, occupied_nodes in getattr(self, 'traffic_control_dict', {}).items():
            if occupant != own_id and escape_node in occupied_nodes:
                return False
        return True


    # --------------------------------------------------------------

    def handle_priority_higher(self, r_id, traffic_control, mex_wp=None, mex_ctx=None, 
        r_wp=None, r_ctx=None, from_source=True):
        """
            Determines if the robot should reserve the node and handle the conflict based on whether the current robot has higher priority.
            If a waitpoint is not found or escape route is blocked, handles the conflict as if the priority is lower.
            Estimates wait time (how long to wait at waitpoint) for the conflicting robot and updates notifications.
            Returns: temp_fb_wait_time, mex_wait_time
        """
        # this robot should reserve the node it wants to go, and then drive directly there.
        # the MEx should reserve this robot's current reserved node, then go to the waiting area first before driving towards it.

        logger.info(f"{r_id}: high priority case.")

        next_stop_id = r_ctx.horizon[0]
        next_stop_coordinate = r_ctx.checkp_itinerary[r_ctx.checkpoints.index(next_stop_id)]
        r_escape = self._has_escape(r_ctx.horizon, r_id)

        # Valid if mex can wait AND current robot can escape
        is_valid = (mex_wp is not None) and (mex_wp not in traffic_control) and r_escape

        if not is_valid:
            if from_source is True:
                logger.info(f"{r_id}: high priority dispatch blocked (no waitpoint or no escape). Falling back to lower.")
                return self.handle_priority_lower(r_id, traffic_control, mex_wp, mex_ctx, r_wp, r_ctx, False)
            else:
                logger.error(f"{mex_ctx.robot_id} mex --> dispatch blocked. Neither robot guarantees exit.")
                return None, None

        mex_est_time = self.estimate_time_to_node(robot_pos = r_ctx.agv_position,
                                                node_pos = next_stop_coordinate,
                                                min_vel = float(r_ctx.speed_min))

        logger.info(f"{mex_ctx.robot_id} mex - reserved: {next_stop_id} --> wait at {mex_wp} for estimated {mex_est_time}.")

        return str(-1.0), str(mex_est_time)

    # --------------------------------------------------------------

    def handle_priority_lower(self, r_id, traffic_control, mex_wp=None, mex_ctx=None, 
        r_wp=None, r_ctx=None, from_source=True):
        """
            Determines if the robot should wait at the reserved checkpoint and
            handles the conflict based on whether the current robot has lower priority.
            If a waitpoint is not found or escape route is blocked, handles the conflict as if the priority is higher.
            Estimates wait time for the current robot and updates notifications.
            Returns: temp_fb_wait_time, mex_wait_time
        """

        logger.info(f"{r_id}: lower priority case.")

        reserved_checkpoint = r_ctx.base
        reserved_checkpoint_coordinate = r_ctx.checkp_itinerary[r_ctx.checkpoints.index(reserved_checkpoint)]
        mex_escape = self._has_escape(mex_ctx.horizon, mex_ctx.robot_id)

        # Valid if current robot can wait AND mex can escape
        is_valid = (r_wp is not None) and (r_wp not in traffic_control) and mex_escape

        if not is_valid:
            if from_source is True:
                logger.info(f"{r_id}: lower priority dispatch blocked (no waitpoint or no escape). Falling back to higher.")
                return self.handle_priority_higher(r_id, traffic_control, mex_wp, mex_ctx, r_wp, r_ctx, False)
            else:
                logger.error(f"{r_id} --> dispatch blocked. Neither robot guarantees exit.")
                return None, None

        r_est_time = self.estimate_time_to_node(robot_pos = mex_ctx.agv_position,
                                              node_pos = reserved_checkpoint_coordinate,
                                              min_vel = float(mex_ctx.speed_min))

        logger.info(f"{r_id} - reserved: {reserved_checkpoint} --> wait at {r_wp} for estimated {r_est_time}.")

        return str(r_est_time), str(-1.0)

    # --------------------------------------------------------------

    def _handle_dock_reminder(self, f_id, _r_id, m_id, v_id, r_ctx):
        """
        Keep the existing dock reminder behavior.
        """
        if not r_ctx.pending_dock_action:
            logger.warning(f"Robot {_r_id} at destination but no pending action found to ping.")
            return

        action_list = [r_ctx.pending_dock_action]
        header_id = r_ctx.header_id + 1
        self.task_handler.instant_actions_handler.build_instant_action_msg(
            f_id, _r_id, header_id, v_id, m_id, action_list)

        print("Dock action pinged to robot: {} with original actionId: {}".format(
                _r_id, r_ctx.pending_dock_action.get("actionId")))

    

