#!/usr/bin/env python3

import psycopg2
import time
import yaml
import os
import math
import sys
import datetime
import uuid
import random
import heapq
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple

# === NO logging.basicConfig() → ONLY USE VISUALIZATION HANDLER ===
# Submodules
sys.path.append('/home/hazeezadebayo/Desktop/fleet_management_v5/fleet_management/submodules')
from connection import ConnectionSubscriber
from factsheet import FactsheetSubscriber
from state import StateSubscriber
from visualization import VisualizationSubscriber
from instant_actions import InstantActionsPublisher
from order import OrderPublisher

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


# ================================================================
# 1. FUZZY DECISION SYSTEM
# ================================================================
class FuzzyTaskDispatcher:
    def __init__(self, max_idle_time=300, max_travel_time=600):
        self.idle_time = ctrl.Antecedent(np.arange(0, max_idle_time + 0.1, 0.1), 'idle_time')
        self.battery = ctrl.Antecedent(np.arange(0, 100.1, 0.1), 'battery')
        self.travel_time = ctrl.Antecedent(np.arange(0, max_travel_time + 0.1, 0.1), 'travel_time')
        self.payload_efficiency = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'payload_efficiency')
        self.charging_opportunity_cost = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'charging_opportunity_cost')
        self.task_type = ctrl.Antecedent(np.arange(0, 2.1, 0.1), 'task_type')
        self.fitness = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'fitness')

        self._define_membership_functions(max_idle_time, max_travel_time)
        self._define_rules()
        self.system = ctrl.ControlSystem(self.rules)
        self.sim = ctrl.ControlSystemSimulation(self.system)

    def _define_membership_functions(self, max_idle_time, max_travel_time):
        self.idle_time['short'] = fuzz.trimf(self.idle_time.universe, [0, 0, 100])
        self.idle_time['long'] = fuzz.trimf(self.idle_time.universe, [50, max_idle_time, max_idle_time])

        self.battery['low'] = fuzz.trimf(self.battery.universe, [0, 0, 40])
        self.battery['medium'] = fuzz.trimf(self.battery.universe, [30, 50, 70])
        self.battery['high'] = fuzz.trimf(self.battery.universe, [60, 100, 100])

        self.travel_time['short'] = fuzz.trimf(self.travel_time.universe, [0, 0, 200])
        self.travel_time['long'] = fuzz.trimf(self.travel_time.universe, [150, max_travel_time, max_travel_time])

        self.payload_efficiency['underutilized'] = fuzz.trimf(self.payload_efficiency.universe, [0, 0, 0.4])
        self.payload_efficiency['balanced'] = fuzz.trimf(self.payload_efficiency.universe, [0.3, 0.6, 0.9])
        self.payload_efficiency['overloaded'] = fuzz.trimf(self.payload_efficiency.universe, [0.8, 1, 1])

        self.charging_opportunity_cost['low'] = fuzz.trimf(self.charging_opportunity_cost.universe, [0, 0, 0.3])
        self.charging_opportunity_cost['high'] = fuzz.trimf(self.charging_opportunity_cost.universe, [0.2, 1, 1])

        self.task_type['delivery'] = fuzz.trimf(self.task_type.universe, [0, 0, 1])
        self.task_type['charge'] = fuzz.trimf(self.task_type.universe, [1, 1, 2])
        self.task_type['move'] = fuzz.trimf(self.task_type.universe, [2, 2, 2])

        self.fitness['poor'] = fuzz.trimf(self.fitness.universe, [0, 0, 0.4])
        self.fitness['good'] = fuzz.trimf(self.fitness.universe, [0.3, 1, 1])

    def _define_rules(self):
        self.rules = [
            ctrl.Rule(self.task_type['delivery'] & self.idle_time['long'] &
                      (self.battery['medium'] | self.battery['high']) &
                      self.travel_time['short'] & self.payload_efficiency['balanced'],
                      self.fitness['good']),
            ctrl.Rule(self.task_type['delivery'] & (self.battery['low'] | self.payload_efficiency['overloaded']),
                      self.fitness['poor']),
            ctrl.Rule(self.task_type['delivery'] & self.payload_efficiency['underutilized'],
                      self.fitness['poor']),
            ctrl.Rule(self.task_type['delivery'] & self.travel_time['short'],
                      self.fitness['good']),
            ctrl.Rule(self.task_type['charge'] & self.battery['low'] & self.charging_opportunity_cost['low'],
                      self.fitness['good']),
            ctrl.Rule(self.task_type['charge'] & (self.charging_opportunity_cost['high'] | self.battery['high']),
                      self.fitness['poor']),
            ctrl.Rule(self.task_type['move'] & self.battery['low'] & self.idle_time['long'],
                      self.fitness['good']),
            ctrl.Rule(self.task_type['move'] & (self.battery['high'] | self.travel_time['short']),
                      self.fitness['poor']),
        ]

    def manhattan_distance(self, pos1, pos2):
        return abs(pos2['x'] - pos1['x']) + abs(pos2['y'] - pos1['y']) + abs(pos2['z'] - pos1['z'])

    def compute_completion_time(self, task_type, rbt_curr_pos, pickup_pos, dropoff_pos=None,
                               velocity=1.0, current_battery=50.0, payload=0.0, max_payload=100.0,
                               drain_rate=0.1, min_battery=10.0, task_energy_cost=2.0):
        payload_factor = min(payload / max_payload, 1.0) if max_payload > 0 else 0
        adjusted_velocity = velocity * (1.0 - 0.2 * payload_factor)

        dist_to_pickup = self.manhattan_distance(rbt_curr_pos, pickup_pos)
        time_to_pickup = dist_to_pickup / adjusted_velocity
        battery_depletion = drain_rate * time_to_pickup
        remaining_battery = current_battery - battery_depletion
        total_time = time_to_pickup

        if task_type == 0 and dropoff_pos:
            dist_to_dropoff = self.manhattan_distance(pickup_pos, dropoff_pos)
            time_to_dropoff = dist_to_dropoff / adjusted_velocity
            battery_depletion += drain_rate * time_to_dropoff
            total_time += time_to_dropoff
            remaining_battery -= drain_rate * time_to_dropoff

        remaining_battery -= task_energy_cost
        if remaining_battery < min_battery:
            return None, None
        return total_time, remaining_battery

    def evaluate_robot_for_task(self, task, robot_state):
        task_type = task['type']
        pickup_pos = task.get('pickup_pos')
        dropoff_pos = task.get('dropoff_pos')
        payload = task.get('payload', 0.0)

        curr_pos = robot_state['current_pos']
        battery = robot_state['current_battery']
        idle = robot_state['idle_time']
        max_payload = robot_state['max_payload']
        velocity = robot_state['velocity']
        drain_rate = robot_state.get('drain_rate', 0.1)

        travel_time = 0.0
        effective_battery = battery

        if task_type in [0, 2] and pickup_pos:
            travel_time, remaining = self.compute_completion_time(
                task_type=task_type,
                rbt_curr_pos=curr_pos,
                pickup_pos=pickup_pos,
                dropoff_pos=dropoff_pos,
                velocity=velocity,
                current_battery=battery,
                payload=payload,
                max_payload=max_payload,
                drain_rate=drain_rate
            )
            if travel_time is None:
                return {'fitness': 0.0, 'travel_time': None}
            effective_battery = remaining

        payload_eff = min(payload / max_payload, 1.0) if max_payload > 0 else 1.0

        self.sim.input['task_type'] = task_type
        self.sim.input['idle_time'] = min(idle, 300)
        self.sim.input['battery'] = effective_battery
        self.sim.input['payload_efficiency'] = payload_eff
        self.sim.input['charging_opportunity_cost'] = 0.0
        self.sim.input['travel_time'] = travel_time

        try:
            self.sim.compute()
            fitness = self.sim.output['fitness']
        except:
            fitness = 0.0

        return {'fitness': fitness, 'travel_time': travel_time}


# ================================================================
# 2. FUZZY TASK HANDLER — SINGLE PASS, NO REDUNDANCY
# ================================================================
class FmTaskHandler:
    def __init__(self, fleetname, version, versions, manufacturer,
                 dbconn, mqttclient=None, task_dict=None):
        self.header_id_count = 0
        self.min_charge_level = 33
        self.max_charge_level = 95
        self.max_task_delay = 2
        self.version = version
        self.manufacturer = manufacturer
        self.fleetname = fleetname
        self.versions = versions
        self.db_conn = dbconn
        self.mqttclient = mqttclient
        self.task_dictionary = task_dict or {}
        self.pause_list = []
        self.ignore_list = []
        self.cp_ignore_list = []

        self.create_fleet_tables()
        self.fuzzy_dispatcher = FuzzyTaskDispatcher()

    # --------------------------------------------------------------------------------

    def create_fleet_tables(self):
        """ create_fleet_tables """
        self.connection_handler = ConnectionSubscriber(self.fleetname, self.versions, self.db_conn, self.mqttclient, True)
        self.factsheet_handler = FactsheetSubscriber(self.fleetname, self.versions, self.db_conn, self.mqttclient, True)
        self.state_handler = StateSubscriber(self.fleetname, self.versions, self.db_conn, self.mqttclient, True)
        self.visualization_handler = VisualizationSubscriber(self.fleetname, self.versions, self.db_conn, self.mqttclient, self.task_dictionary)
        self.instant_actions_handler = InstantActionsPublisher(self.fleetname, self.versions, self.db_conn, self.mqttclient, True)
        self.order_handler = OrderPublisher(self.fleetname, self.versions, self.db_conn, self.mqttclient, True)

    # --------------------------------------------------------------------------------

    def generate_new_order_id(self, order_id=None):
        """ generate a new one with _0. If the order ID already contains an index, increment the index."""
        res = str(uuid.uuid4()) + "_0", 0
        if order_id is not None and '_' in order_id:
            uuid_part, index_part = order_id.rsplit('_', 1)
            new_index = int(index_part) + 1
            res = f"{uuid_part}_{new_index}", new_index
        return res

    # --------------------------------------------------------------------------------

    def generate_unassigned_robot_id(self, r_id):
        uuid_, _ = self.generate_new_order_id()
        return f"unassigned_{r_id}_{uuid_}"

    # --------------------------------------------------------------------------------

    def extract_r_id(self, unassigned_id):
        return unassigned_id.split('_')[1]

    # ----------------------------------------------------------------------------

    def fm_send_task_request(
        self, f_id, r_id, from_loc_id, to_loc_id, task_name, task_priority,
        task_dictionary=None, unassigned=False, override_cleared=False, payload_kg=-1.0
    ):
        """
        Build task, select best robot (or user-specified), and file it.

        Returns:
            (task_cleared, chosen_r_id, checkpoints, agv_itinerary,
            waitpoints, wait_itinerary, landmark)
        """
        task_dictionary = task_dictionary or self.task_dictionary

        # --- Step 1: Build task ---
        task = self._build_task(
            f_id, from_loc_id, to_loc_id, task_name,
            task_priority, payload_kg, task_dictionary
        )
        if not task:
            return False, None, [], [], [], [], []

        # --- Step 2: Get candidate robots (plus all IDs) ---
        candidates, all_r_ids = self._get_candidate_robots_with_state(
            f_id, task, payload_kg)

        # --- Step 3: Handle if we have candidates ---
        if candidates:
            # Sort by fitness descending (so we can easily get best and second best)
            candidates.sort(key=lambda x: x['fitness'], reverse=True)
            best = candidates[0]
            chosen_r_id = best['r_id']
            robot_state = best['robot_state']

            # --- Handle user-specified robot ---
            if r_id:
                forced = next((c for c in candidates if c['r_id'] == r_id), None)
                if forced:
                    # ✅ User's chosen robot is available
                    chosen_r_id = r_id
                    robot_state = forced['robot_state']
                else:
                    # ❌ User robot unavailable
                    if task_name in ('transport', 'loop'):
                        # ✅ Use next best (second-best) candidate if available
                        alt = next((c for c in candidates if c['r_id'] != r_id), None)
                        if alt:
                            chosen_r_id = alt['r_id']
                            robot_state = alt['robot_state']

                            self.visualization_handler.terminal_log_visualization(
                                f"User robot {r_id} busy. Reassigning to next best: {chosen_r_id} "
                                f"(fitness: {alt['fitness']:.3f})",
                                "FmTaskHandler", "fm_send_task_request", "info")
                        else:
                            # No alternative available (could happen if we only had one robot)
                            return self._queue_for_robot(
                                f_id, r_id, from_loc_id, to_loc_id,
                                task_name, task_priority, payload_kg)
                    else:
                        # move/charge → queue for the user robot
                        return self._queue_for_robot(
                            f_id, r_id, from_loc_id, to_loc_id,
                            task_name, task_priority, payload_kg)
            else:
                # No user-specified robot
                self.visualization_handler.terminal_log_visualization(
                    f"Assigning to best fit: {chosen_r_id} "
                    f"(fitness: {best['fitness']:.3f})",
                    "FmTaskHandler", "fm_send_task_request", "info")
        else:
            # --- Step 4: Handle no available robots ---
            # Use first available known robot id, if any
            fallback_r_id = next((rid for rid in all_r_ids if rid not in self.ignore_list), None)
            if not fallback_r_id:
                self.visualization_handler.terminal_log_visualization(
                    "No valid robots found in fleet. Task cannot be queued.",
                    "FmTaskHandler", "fm_send_task_request", "critical")
                return
            return self._queue_for_robot(
                f_id, fallback_r_id, from_loc_id, to_loc_id,
                task_name, task_priority, payload_kg)

        # --- Step 5: Log assignment ---
        msg = (f"[FUZZY ASSIGN] → {chosen_r_id} | Fitness: {best['fitness']:.3f} "
            f"| Travel: {best['travel_time']:.1f}s")
        self.visualization_handler.terminal_log_visualization(
            msg, "FmTaskHandler", "fm_send_task_request", "critical"
        )

        # --- Step 6: Finalize and file ---
        cleared, checkpoints, agv_itinerary, waitpoints, wait_itinerary, landmark = \
            self._finalize_task(
                f_id, chosen_r_id, from_loc_id, to_loc_id,
                task_name, task_priority, task_dictionary,
                override_cleared, payload_kg, robot_state
            )

        if checkpoints and cleared:
            self.fm_file_task(
                f_id, chosen_r_id, checkpoints, waitpoints,
                agv_itinerary, wait_itinerary, landmark
            )

        return (cleared, chosen_r_id, checkpoints, agv_itinerary,
                waitpoints, wait_itinerary, landmark)



    # ----------------------------------------------------------------------------

    def _get_candidate_robots_with_state(self, f_id, task, payload_kg):
        """Return list of candidate robots with full state & fitness score."""
        candidates = []
        r_ids = self.factsheet_handler.fetch_serial_numbers(f_id)

        for r_id in r_ids:
            if r_id in self.ignore_list or r_id in self.cp_ignore_list:
                continue

            # ---- single fitness check (now returns idle_time & max_payload) ----
            (cleared, pos, battery, last_node, _, _,
            home_docks, charge_docks, station_docks,
            idle_time, max_payload) = self.verify_robot_fitness(
                f_id, r_id, self.manufacturer, task['type'], payload_kg)

            if not cleared:
                continue

            if task['type'] == 0 and payload_kg > max_payload:
                print("6. ")
                continue

            r_state = {
                'current_pos': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
                'current_battery': battery,
                'idle_time': idle_time,
                'max_payload': max_payload,
                'velocity': 1.0,
                'drain_rate': 0.1
            }

            result = self.fuzzy_dispatcher.evaluate_robot_for_task(task, r_state)
            print("result: ", result)

            if result['fitness'] > 0.0:
                candidates.append({
                    'r_id': r_id,
                    'fitness': result['fitness'],
                    'travel_time': result['travel_time'],
                    'robot_state': {
                        'pos': pos,
                        'battery': battery,
                        'last_node': last_node,
                        'home_docks': home_docks,
                        'charge_docks': charge_docks,
                        'station_docks': station_docks
                    }
                })

        return candidates, r_ids

    # ----------------------------------------------------------------------------

    # Helper: No candidates at all or Queue task for specific robot (move/charge)
    def _queue_for_robot(self, f_id, r_id, from_loc_id, to_loc_id,
                        task_name, task_priority, payload_kg):
        generic_unassigned_robot_id = self.generate_unassigned_robot_id(r_id)
        self.fm_file_task(f_id,
            generic_unassigned_robot_id,
            checkpoints=[from_loc_id, to_loc_id],
            waitpoints=[],
            agv_itinerary=[[0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]],
            wait_itinerary=[],
            landmark=[payload_kg, task_priority, task_name, from_loc_id, to_loc_id])
        return False, None, [], [], [], [], []

    # ----------------------------------------------------------------------------

    def _finalize_task(self, f_id, r_id, from_loc_id, to_loc_id, task_name, task_priority,
                       task_dictionary, override_cleared, payload_kg, robot_state):
        task_clear = True  # Already cleared in candidate phase

        checkpoints, landmark, agv_itinerary = self.build_task_itinerary(
            from_loc_id, to_loc_id, task_name, task_priority,
            robot_state['last_node'], robot_state['home_docks'],
            robot_state['charge_docks'], task_dictionary, payload_kg)

        waitpoints = self.fm_extract_unique_waitpoints(checkpoints, task_dictionary) if checkpoints else []
        wait_itinerary = self.fm_get_itinerary(waitpoints, task_dictionary)

        log_msg = (
            f"R_id: {r_id}. \n --- \n"
            f"task clear status: {task_clear} \n"
            f"checkpoints to pass: {checkpoints} \n"
            f"corresponding itinerary: {agv_itinerary} \n"
            f"landmark to visit: {landmark} \n"
            f"waitpoints: {waitpoints} \n"
            f"waitpoint's itinerary: {wait_itinerary}. \n --- "
        )
        self.visualization_handler.terminal_log_visualization(log_msg, "FmTaskHandler", "request_tasks", "critical")

        return task_clear, checkpoints, agv_itinerary, waitpoints, wait_itinerary, landmark

    # ----------------------------------------------------------------------------------------------------

    def _build_task(self, f_id, from_loc_id, to_loc_id, task_name, task_priority, payload_kg, task_dictionary):
        if from_loc_id == to_loc_id and task_name not in ['charge', 'move']:
            msg = "'load' location cannot be equal to 'unload' location."
            self.visualization_handler.terminal_log_visualization(msg, "FmTaskHandler", "_build_task", "error")
            return None

        if not self.check_node_in_fleet(f_id, from_loc_id, task_dictionary) or \
           not self.check_node_in_fleet(f_id, to_loc_id, task_dictionary):
            msg = f"Invalid node in fleet {f_id}."
            self.visualization_handler.terminal_log_visualization(msg, "FmTaskHandler", "_build_task", "error")
            return None

        task_type_map = {'transport': 0, 'loop': 0, 'charge': 1, 'move': 2}
        task_type = task_type_map.get(task_name, 0)

        pickup = self._get_node_coords(f_id, from_loc_id, task_dictionary)
        dropoff = self._get_node_coords(f_id, to_loc_id, task_dictionary) if task_type == 0 else None

        return {
            'type': task_type,
            'pickup_pos': pickup,
            'dropoff_pos': dropoff,
            'payload': float(payload_kg),
            'task_priority': {'low': 1, 'medium': 2, 'high': 3}.get(task_priority.lower(), 2)
        }

    # ----------------------------------------------------------------------------------------------------

    def _get_node_coords(self, f_id, loc_id, task_dictionary=None):
        """Return {'x':, 'y':, 'z':} of node, or None."""
        td = task_dictionary or self.task_dictionary
        return next(
            ({"x": i["coordinate"][0], "y": i["coordinate"][1], "z": i["coordinate"][2]}
            for i in td["itinerary"]
            if i["fleet_id"] == f_id and i["loc_id"] == loc_id),
            None
        )

    # ----------------------------------------------------------------------------------------------------

    def fm_file_task(self, f_id, r_id, checkpoints, waitpoints, agv_itinerary, wait_itinerary, landmark):
        """
        Create and send order for robot (if not ignored).

        Preserves:
            - Order creation
            - Error logging
            - Ignore list check
        """
        if r_id in self.ignore_list:
            self.visualization_handler.terminal_log_visualization(
                f"Robot {r_id}: ignored. Remove from ignore list.",
                "FmTaskHandler", "fm_file_task", "error")
            return

        try:
            header_id = self.header_id_count + 1
            order_id, update_id = self.generate_new_order_id()

            b_node, h_nodes, h_edges = self.order_handler.create_order(
                checkpoints, waitpoints, agv_itinerary, wait_itinerary, landmark)

            self.order_handler.build_order_msg(
                f_id, r_id, header_id, self.version, self.manufacturer,
                b_node, h_nodes, h_edges, order_id, update_id)

        except (ValueError, TypeError) as e:
            self.visualization_handler.terminal_log_visualization(
                f"Robot {r_id}: {e}", "FmTaskHandler", "fm_file_task", "warn")

    # ----------------------------------------------------------------------------

    def verify_robot_fitness(self, f_id, r_id, m_id, task_name, payload_kg):
        """
        Functionality:
            Check if the robot is available for the task: that is,
            it checks if the robot is idle, has no previous tasks,
            and if the battery level is sufficient for the task.
        """
        cleared = False
        current_position = []
        battery_charge = 0.0
        last_node_id = ''
        order_id_ = ''
        loc_node_owner = None
        home_dock_loc_ids = []
        charge_dock_loc_ids = []
        station_dk_loc_ids = []
        order_nodes = []
        order_timestamp = None
        can_carry = True
        idle_time = 300.0

        _, max_payload, _, _, _, _, _ = self.factsheet_handler.fetch_data(f_id, r_id, m_id)
        try:
            # Attempt to convert payload_kg and max_payload to float
            if (task_name in ['transport', 'loop'] and (float(payload_kg) > float(max_payload))):
                can_carry = False
                # log viz:
                self.visualization_handler.terminal_log_visualization(
                    f"{r_id}: required payload size '{payload_kg}' is greater than the robot's max payload size '{max_payload}'.",
                    "FmTaskHandler",
                    "verify_robot_fitness",
                    "info")
        except ValueError as e:
            # log viz:
            self.visualization_handler.terminal_log_visualization(
                f"{r_id}: Error converting payload values to float: {e}",
                "FmTaskHandler",
                "verify_robot_fitness",
                "error")
            # Optionally, set a default behavior or re-raise the exception:
            can_carry = False

        connection_state, _ = self.connection_handler.fetch_data(m_id, r_id)
        # log viz:
        self.visualization_handler.terminal_log_visualization(
            f"{r_id}: connection state is {connection_state}.",
            "FmTaskHandler",
            "verify_robot_fitness",
            "info")

        if connection_state:

            if (connection_state == "ONLINE") and can_carry:

                # get robot last received state message
                latest_state = self.state_handler.fetch_data(f_id, r_id, m_id)

                # -----------------
                # parse fetched data
                # -----------------
                _, _, order_id_, \
                    last_node_id, _, _, node_states, \
                        agv_position, _, battery_state, \
                            _, _ = latest_state
                battery_charge = battery_state['batteryCharge']
                current_position = [float(agv_position['x']),
                                    float(agv_position['y']),
                                    float(agv_position['theta'])]

                # -----------------
                # identify related landmarks
                # -----------------
                _, loc_node_owner, home_dock_loc_ids, charge_dock_loc_ids, station_dk_loc_ids = \
                    self.find_nearest_node(f_id, current_position)

                # -----------------
                # check at_home = True or False --> in the robot state, is the last_node_id a home dock?
                # -----------------
                if last_node_id != '':
                    if node_states: # green
                        at_home = False
                        # log viz:
                        self.visualization_handler.terminal_log_visualization(
                            f"Robot {r_id}: appears to already be on a task.",
                            "FmTaskHandler",
                            "verify_robot_fitness",
                            "info")
                    else: # red
                        at_home = self.get_if_home(f_id, last_node_id)
                        if not at_home:
                            # log viz:
                            self.visualization_handler.terminal_log_visualization(
                                f"Robot {r_id}: last node id not a home dock.",
                                "FmTaskHandler",
                                "verify_robot_fitness",
                                "info")
                else:
                    last_node_id = loc_node_owner
                    at_home = self.get_if_home(f_id, last_node_id)
                    if not at_home:
                        # log viz:
                        self.visualization_handler.terminal_log_visualization(
                            f"Robot {r_id}: no last node id found in state msg and robot location not close to a home dock.",
                            "FmTaskHandler",
                            "verify_robot_fitness",
                            "info")
                # -----------------
                # Fetch and handle last received order
                # -----------------
                latest_order = self.order_handler.fetch_data(f_id, r_id, m_id)
                if latest_order:
                    order_timestamp = latest_order[2]
                    order_nodes = latest_order[9]
                    # --------------------------------------------------------------------- active order
                    # idle-time calculation (mirrors _build_robot_state)
                    now = datetime.datetime.now(datetime.timezone.utc)
                    try:
                        ts = datetime.datetime.fromisoformat(order_timestamp.replace('Z', '+00:00'))
                        idle_time = min((now - ts).total_seconds(), 300.0)
                    except Exception:
                        idle_time = 300.0

                # -----------------
                # check battery level
                # -----------------
                if float(battery_charge) > self.min_charge_level:
                    if at_home and not len(order_nodes) > 1:
                        cleared = True
                    elif (task_name == 'loop') and (last_node_id in station_dk_loc_ids) and len(order_nodes) == 1:
                        # robot at drop off location
                        cleared = True
                    else:
                        # if it was not at home, check if last task/order was cancelled.
                        last_cancel_action, _ = self.instant_actions_handler.fetch_data(f_id,
                                                                                        r_id,
                                                                                        m_id,
                                                                                        "cancelOrder")
                        if last_cancel_action:
                            if last_cancel_action['actionParameters'][0]['orderID'] == order_id_:
                                cleared = True
                        else:
                            # log viz:
                            self.visualization_handler.terminal_log_visualization(
                                f"Robot {r_id}: previous task was not cancelled and robot is not at a home dock.",
                                "FmTaskHandler",
                                "verify_robot_fitness",
                                "info")
                # or task in itself is a charge task.
                else:
                    # and the elements in the current order or last recieved order is not more than 1 - >1 implies that there is
                    # an active task-,
                    if task_name == 'charge' and not len(order_nodes) > 1\
                        and (last_node_id not in charge_dock_loc_ids): # i.e. we are not already at a charge station
                        cleared = True
                    else:
                        # log viz:
                        self.visualization_handler.terminal_log_visualization(
                            f"Robot {r_id}: battery level lower than minimum required.",
                            "FmTaskHandler",
                            "verify_robot_fitness",
                            "info")

        return (cleared, current_position, battery_charge, last_node_id, order_id_,
                order_timestamp, home_dock_loc_ids, charge_dock_loc_ids, station_dk_loc_ids,
                idle_time, max_payload)

    # ----------------------------------------------------------------------------------------------------

    def check_node_in_fleet(self, f_id, loc_id, task_dictionary=None):
        """Return True if loc_id exists in fleet's itinerary, else False."""
        items = (task_dictionary or self.task_dictionary)["itinerary"]
        return any(i["fleet_id"] == f_id and i["loc_id"] == loc_id for i in items)

    # ----------------------------------------------------------------------------------------------------

    def find_nearest_node(self, f_id, r_position, task_dictionary=None):
        """
        Returns:
            tuple: The shortest distance and the node ID of the nearest location.
        Functionality:
            Finds the nearest node location to the robot's current position from the list of graph nodes.
            Iterates over job locations to find the shortest distance from the robot's current position.
            Updates the lists of home dock and charge dock locations based on job descriptions.
        """

        # log viz:
        self.visualization_handler.terminal_log_visualization(
            f"robot position: {r_position[0]}, {r_position[1]}, {r_position[2]}.",
            "FmTaskHandler",
            "find_nearest_node",
            "info")

        shortest_dist = float('inf')
        loc_node_ownr = None
        home_dk_loc_ids = []
        charge_dk_loc_ids = []
        station_dk_loc_ids = []

        if task_dictionary is None:
            task_dictionary = self.task_dictionary

        for item in task_dictionary["itinerary"]:
            if item["fleet_id"] == f_id:
                loc_id = item["loc_id"]
                description = item["description"]
                coordinate = item["coordinate"]
                # Build lists of home_dock and charge_dock stations
                if description == 'home_dock':
                    home_dk_loc_ids.append(loc_id)
                elif description == 'charge_dock':
                    charge_dk_loc_ids.append(loc_id)
                elif description == 'station_dock':
                    station_dk_loc_ids.append(loc_id)
                # Calculate distance to the current pose
                if description != 'waitpoint':
                    d = math.sqrt((float(coordinate[0]) - float(r_position[0]))**2 +
                                (float(coordinate[1]) - float(r_position[1]))**2)
                    # Update shortest distance found
                    if d < shortest_dist:
                        shortest_dist = d
                        loc_node_ownr = loc_id
                        # log viz:
                        self.visualization_handler.terminal_log_visualization(
                            f"Found shortest distance: {shortest_dist} with loc: {loc_node_ownr}.",
                            "FmTaskHandler",
                            "find_nearest_node",
                            "info")

        return shortest_dist, loc_node_ownr, home_dk_loc_ids, charge_dk_loc_ids, station_dk_loc_ids

    # ----------------------------------------------------------------------------------------------------

    def get_if_home(self, f_id, last_node_id, task_dictionary=None):
        """Return True if robot is at a home dock node, else False."""
        items = (task_dictionary or self.task_dictionary)["itinerary"]
        return any(
            i["fleet_id"] == f_id and i["description"] == "home_dock" and i["loc_id"] == last_node_id
            for i in items
        )

    # --------------------------------------------------------------------------------------------

    def get_node_map(self, f_id, node_id, task_dictionary=None):
        """Return map ID of node in fleet, or '' if not found."""
        items = (task_dictionary or self.task_dictionary)["itinerary"]
        return next(
            (i["map_id"] for i in items if i["fleet_id"] == f_id and i["loc_id"] == node_id),
            ""
        )

    # --------------------------------------------------------------------------------------------

    def get_node_description(self, f_id, node_id, task_dictionary=None):
        """Return description of node in fleet, or '' if not found."""
        items = (task_dictionary or self.task_dictionary)["itinerary"]
        return next(
            (i["description"] for i in items if i["fleet_id"] == f_id and i["loc_id"] == node_id),
            ""
        )

    # --------------------------------------------------------------------------------------------

    def build_graph(self, task_dictionary):
        """Convert task dictionary graph to adjacency set format."""
        return {
            node: {tuple(neigh) for neigh in neighbors}
            for node, neighbors in task_dictionary["graph"].items()
        }

    # ----------------------------------------------------------------------------------------------------

    def build_task_itinerary(self, from_loc_id, to_loc_id, task_name, task_priority,
                            loc_node_owner, home_dock_loc_ids, charge_dock_loc_ids,
                            task_dictionary=None, payload_kg=-1.0):
        """
        Build full task itinerary based on task type.

        Returns:
            tuple: (checkpoints, landmark, agv_itinerary)
        """
        td = task_dictionary or self.task_dictionary

        if task_name in ('charge', 'move'):
            return self.handle_charge_or_move_task(
                task_name, task_priority, loc_node_owner, to_loc_id,
                home_dock_loc_ids, charge_dock_loc_ids, td, payload_kg
            )
        if task_name in ('transport', 'loop'):
            return self.handle_transport_or_loop_task(
                task_name, task_priority, loc_node_owner, from_loc_id, to_loc_id,
                home_dock_loc_ids, td, payload_kg
            )

        self.visualization_handler.terminal_log_visualization(
            "Unknown task name. Must be: charge, move, transport, loop",
            "FmTaskHandler", "build_task_itinerary", "error"
        )
        return [], [], []

    # ----------------------------------------------------------------------------------------------------

    def handle_charge_or_move_task(self, task_name, task_priority, loc_node_owner,
                                to_loc_id, home_dock_loc_ids, charge_dock_loc_ids,
                                task_dictionary=None, payload_kg=-1.0):
        """
        - 'charge': go to a valid charge dock (pick random if given ID is invalid);
        list all docks in landmark for fallback.
        - 'move': only to home docks; return empty if robot is already there.

        Returns:
            tuple: (checkpoints, landmark, agv_itinerary)
        """
        td = task_dictionary or self.task_dictionary
        graph = self.build_graph(td)
        checkpoints = []
        landmark = []

        if task_name == 'charge':
            landmark = [payload_kg, task_priority, task_name]

            if to_loc_id in charge_dock_loc_ids:
                target = to_loc_id
                others = [d for d in charge_dock_loc_ids if d != target]
            else:
                self.visualization_handler.terminal_log_visualization(
                    "Invalid charge dock. Selecting random valid one.",
                    "FmTaskHandler", "handle_charge_or_move_task", "info")
                target = random.choice(charge_dock_loc_ids)
                others = [d for d in charge_dock_loc_ids if d != target]

            checkpoints = self.fm_shortest_paths(loc_node_owner, target, graph)[0]
            landmark.extend([target] + others)

        elif task_name == 'move':
            if loc_node_owner == to_loc_id:
                return [], [], []

            if to_loc_id not in home_dock_loc_ids:
                self.visualization_handler.terminal_log_visualization(
                    "Move task only to home docks.", "FmTaskHandler",
                    "handle_charge_or_move_task", "error")
                return [], [], []

            landmark = [payload_kg, task_priority, task_name]
            target = to_loc_id
            others = [d for d in home_dock_loc_ids if d != target]
            checkpoints = self.fm_shortest_paths(loc_node_owner, target, graph)[0]
            landmark.extend([target] + others)

        itinerary = self.fm_get_itinerary(checkpoints, td) if checkpoints else []
        return checkpoints, landmark, itinerary

    # ----------------------------------------------------------------------------------------------------

    def handle_transport_or_loop_task(self, task_name, task_priority, loc_node_owner,
                                    from_loc_id, to_loc_id, home_dock_loc_ids,
                                    task_dictionary=None, payload_kg=-1.0):
        """
        - Always: current → from_loc_id → to_loc_id
        - 'transport' only: append return to a random home dock **only if the
        last node of the current path equals the first node of the home path**.
        - Avoid duplicate nodes when concatenating.

        Returns:
            tuple: (checkpoints, landmark, agv_itinerary)
        """
        td = task_dictionary or self.task_dictionary
        graph = self.build_graph(td)
        checkpoints = []
        landmark = []

        if task_name not in ('transport', 'loop'):
            return [], [], []

        # current → pickup
        to_pickup = []
        if loc_node_owner != from_loc_id:
            to_pickup = self.fm_shortest_paths(loc_node_owner, from_loc_id, graph)[0]

        # pickup → delivery
        pickup_to_delivery = self.fm_shortest_paths(from_loc_id, to_loc_id, graph)[0]

        # join without duplication
        if to_pickup and to_pickup[-1] == pickup_to_delivery[0]:
            checkpoints = to_pickup + pickup_to_delivery[1:]
        else:
            checkpoints = (to_pickup or []) + pickup_to_delivery

        landmark = [payload_kg, task_priority, task_name, from_loc_id, to_loc_id]

        # optional return home (transport only)
        if task_name == 'transport' and home_dock_loc_ids:
            primary = random.choice(home_dock_loc_ids)
            others = [d for d in home_dock_loc_ids if d != primary]
            landmark.extend([primary] + others)

            home_path = self.fm_shortest_paths(to_loc_id, primary, graph)[0]
            if checkpoints and home_path and checkpoints[-1] == home_path[0]:
                checkpoints += home_path[1:]

        itinerary = self.fm_get_itinerary(checkpoints, td) if checkpoints else []
        return checkpoints, landmark, itinerary

    # ----------------------------------------------------------------------------------------------------

    def fm_get_itinerary(self, nodes, task_dictionary):
        """ General function to retrieve coordinates for given location IDs. """
        d = {e["loc_id"]: e["coordinate"] for e in task_dictionary["itinerary"]}
        return [d[n] for n in nodes if n in d]

    # ----------------------------------------------------------------------------------------------------

    def fm_extract_unique_waitpoints(self, checkpoints, task_dictionary):
        """ fetch the locations coordinates associated with the node/checkpoints path """
        self.visualization_handler.terminal_log_visualization(
            "fetching task checkpoint's associated waitpoints...", "FmTaskHandler", "fm_extract_unique_waitpoints", "info")
        graph = self.build_graph(task_dictionary)
        waitpoints = set()
        for cp in checkpoints:
            for n, _ in graph.get(cp, []):
                if n.startswith('W'):
                    waitpoints.add(n)
        return list(waitpoints)

    # ----------------------------------------------------------------------------------------------------

    def fm_shortest_paths(self, start, goal, graph, max_alternatives=2):
        """ Find the shortest path and additional alternatives using Dijkstra's algorithm. """
        heap = [(0, [start])]
        visited = set()
        paths = []
        while heap and len(paths) < max_alternatives:
            dist, path = heapq.heappop(heap)
            node = path[-1]
            if node == goal:
                paths.append(path)
                continue
            if node in visited:
                continue
            visited.add(node)
            for neigh, w in graph.get(node, []):
                if neigh not in visited:
                    heapq.heappush(heap, (dist + w, path + [neigh]))
        return paths if paths else [[]]

    # ----------------------------------------------------------------------------------------------------

    def cancel_task_db_cmd(self, f_id, r_id, m_id, v_id, o_id):
        """ cancel_task_db_cmd """
        cancel_order_action = self.instant_actions_handler.create_action("cancelOrder",
                                                                         {"orderID":o_id,
                                                                          "fleetName":f_id})
        h_id = self.header_id_count + 1
        action_list = [cancel_order_action]
        self.instant_actions_handler.build_instant_action_msg(f_id, r_id, h_id, v_id, m_id, action_list)



# ================================================================
# MAIN — FULLY TESTABLE WITH REAL OUTPUT
# ================================================================


if __name__ == "__main__":
    conn = None
    try:
        conn = psycopg2.connect(host='localhost', dbname='postgres', user='postgres', password='root', port='5432')
    except Exception as e:
        print("DB connect failed:", e)

    agv_dir_prefix = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
    file_path = Path(os.path.join(agv_dir_prefix, 'fleet_management', 'config', 'config.yaml'))
    task_dict_ = yaml.safe_load(open(file_path, 'r', encoding='utf-8')) if file_path.is_file() else {}


    # Initialize the StateSubscriber: Generate order message
    fleetname = "kullar"
    robot_serial_number = "AGV-001" # "SN12345678"
    version = "1.0.0"
    versions = "v1"
    manufacturer = "birfen"
    connectionState = "ONLINE"

    # Step 2: Insert a pseudo robot data into the database
    sample_conn_message = {
        "headerId": 1,
        "timestamp": datetime.datetime.now().isoformat(),
        "version": version,
        "manufacturer": manufacturer,
        "serialNumber": robot_serial_number,
        "connectionState": connectionState # ['ONLINE', 'OFFLINE', 'CONNECTIONBROKEN']
    }

    # sample factsheet message:
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
            "maxLoadMass": 40,
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
            "versions": [{"fleetName": fleetname}],
            "config2": "value2"
        }
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
        "orderId": "",
        "orderUpdateId": 1,
        "zoneSetId": fleetname,
        "lastNodeId": "", # "C10",
        "lastNodeSequenceId": 1,
        "driving": False,
        "paused": True,
        "newBaseRequest": False,
        "distanceSinceLastNode": 1.0,
        "operatingMode": "AUTOMATIC",
        "nodeStates": [
            # {
            #     "nodeId": "node_0",
            #     "sequenceId": 0,
            #     "nodeDescription": "start and end Node",
            #     "nodePosition": {
            #         "x": 0.0,
            #         "y": 0.0,
            #         "theta": 0.0,
            #         "mapId": "map1"
            #     },
            #     "released": True
            # }
        ],
        "edgeStates": [
            # {
            #     "edgeId": "",
            #     "sequenceId": 0,
            #     "edgeDescription": "Edge between node_x and node_y",
            #     "released": True,
            #     "trajectory": {
            #         "degree": 0,
            #         "knotVector": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            #         "controlPoints": [
            #             {"x": 1.0, "y": 5.0, "weight": 1.0},
            #             {"x": 1.5, "y": 6.5, "weight": 1.0},
            #             {"x": 2.0, "y": 7.0, "weight": 1.0}
            #         ]
            #     }
            # }
        ],
        "agvPosition": {
            # c10
            # "x": 1.8,
            # "y": 0.49,
            # "theta": -0.695,
            # c11
            "x": -1.7,
            "y": 0.52,
            "theta": -0.95,
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
        "information": [
            {
                "infoType": "fleet_name",
                "infoLevel": "INFO",
                "infoReferences": [
                    {
                        "referenceKey": "fleet_name",
                        "referenceValue": fleetname
                    }
                ],
                "infoDescription": "The battery is in good condition and has a sufficient charge."
            },
            {
                "infoType": "wheelinfo",
                "infoLevel": "INFO",
                "infoReferences": [
                    {
                        "referenceKey": "wheel_sep_rad",
                        "referenceValue": "0.55,0.15"
                    }
                ],
                "infoDescription": "The battery is in good condition and has a sufficient charge."
            }
        ],
        "safetyState": {
            "eStop": "NONE",
            "fieldViolation": True
        }
    }

    task_handler = FmTaskHandler(
        fleetname=fleetname, version=version, versions=versions,
        manufacturer=manufacturer, dbconn=conn, task_dict=task_dict_
    )

    task_handler.connection_handler.process_message(sample_conn_message)
    task_handler.factsheet_handler.process_message(sample_factsheet_message)
    task_handler.state_handler.process_message(sample_state_message)

    # AUTO-ASSIGN
    # test payload kg related tasks
    task_c, robot_serial_number, checkps, \
        checkp_itinrry, waitps, \
            wait_itinrry, landm_ = task_handler.fm_send_task_request(
                f_id=fleetname,
                r_id=0,  # ignored — auto-assign | robot_serial_number,
                from_loc_id='C3',
                to_loc_id='C5',
                task_name='transport',
                task_priority='high',
                payload_kg=35.0)

    _, _, order_id, \
        _, _, _, _, \
            _, _, _, \
                _, _ = task_handler.state_handler.fetch_data(f_id=fleetname, r_id=robot_serial_number, m_id=manufacturer)
    task_handler.cancel_task_db_cmd(f_id=fleetname, r_id=robot_serial_number, m_id=manufacturer, v_id=version, o_id=order_id)

    time.sleep(1.0)

    # Close the database connection
    conn.close()


# #######################################################################
# hazeezadebayo@hazeezadebayo:~/Desktop/fleet_management_v5/fleet_management$ /usr/bin/python3 /home/hazeezadebayo/Desktop/fleet_management_v5/fleet_management/fleet_management/Fuzzy_FmTaskHandler.py
# [INFO] [2025-10-29 14:34:12,976] VisualizationSubscriber: table maps dropped successfully.
# [INFO] [2025-10-29 14:34:12,982] VisualizationSubscriber: Maps table created successfully.
# [INFO] [2025-10-29 14:34:12,992] OrderPublisher: table_order table dropped successfully.
# [INFO] [2025-10-29 14:34:12,997] OrderPublisher: Order table created successfully..
# [INFO] [2025-10-29 14:34:13,034] VisualizationSubscriber: [INFO ] [2025-10-29 14:34:13] FmTaskHandler.fm_send_task_request - Auto-selecting best robot via fuzzy logic...
# [INFO] [2025-10-29 14:34:13,037] VisualizationSubscriber: [INFO ] [2025-10-29 14:34:13] FmTaskHandler.verify_robot_fitness - AGV-001: connection state is ONLINE.
# [INFO] [2025-10-29 14:34:13,037] VisualizationSubscriber: [INFO ] [2025-10-29 14:34:13] FmTaskHandler.find_nearest_node - robot position: -1.7, 0.52, -0.95.
# [INFO] [2025-10-29 14:34:13,037] VisualizationSubscriber: [INFO ] [2025-10-29 14:34:13] FmTaskHandler.find_nearest_node - Found shortest distance: 3.909015221254581 with loc: C9.
# [INFO] [2025-10-29 14:34:13,038] VisualizationSubscriber: [INFO ] [2025-10-29 14:34:13] FmTaskHandler.find_nearest_node - Found shortest distance: 1.777751388693031 with loc: C1.
# [INFO] [2025-10-29 14:34:13,038] VisualizationSubscriber: [INFO ] [2025-10-29 14:34:13] FmTaskHandler.find_nearest_node - Found shortest distance: 1.400142849854971 with loc: C25.
# [INFO] [2025-10-29 14:34:13,045] VisualizationSubscriber: [CRITICAL] [2025-10-29 14:34:13] FmTaskHandler.fm_send_task_request - [FUZZY ASSIGN] → AGV-001 | Fitness: 0.321 | Travel: 20.8s
# [INFO] [2025-10-29 14:34:13,046] VisualizationSubscriber: [INFO ] [2025-10-29 14:34:13] FmTaskHandler.fm_extract_unique_waitpoints - fetching task checkpoint's associated waitpoints...
# [INFO] [2025-10-29 14:34:13,046] VisualizationSubscriber: [CRITICAL] [2025-10-29 14:34:13] FmTaskHandler.request_tasks - R_id: AGV-001.
#  ---
# task clear status: True
# checkpoints to pass: ['C3', 'C2', 'C1', 'C5', 'C1', 'C2', 'C3', 'C4', 'C28']
# corresponding itinerary: [[0.0, 6.0, -0.097, 0.995], [0.0, 3.0, -0.097, 0.995], [0.0, 0.0, -0.194, 0.98], [3.0, 0.0, -0.194, 0.98], [0.0, 0.0, -0.194, 0.98], [0.0, 3.0, -0.097, 0.995], [0.0, 6.0, -0.097, 0.995], [0.0, 9.0, -0.097, 0.995], [-3.0, 9.0, -0.097, 0.995]]
# landmark to visit: [35.0, 'high', 'transport', 'C3', 'C5', 'C28', 'C25', 'C26', 'C27']
# waitpoints: ['W2', 'W4', 'W5', 'W1', 'W3']
# waitpoint's itinerary: [[0.3, 3.3, -0.097, 0.995], [0.3, 9.3, -0.695, 0.718], [3.3, 0.3, -0.194, 0.98], [0.3, 0.3, -0.194, 0.98], [0.3, 6.3, -0.097, 0.995]].
#  ---
# # #######################################################################
