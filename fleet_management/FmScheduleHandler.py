#!/usr/bin/env python3

import psycopg2, psycopg2.extras
import time, yaml, os, datetime, time, random, glob
from twilio.rest import Client
from pathlib import Path

from FmTrafficHandler import FmTrafficHandler


class FmScheduleHandler():
    """ fleet manager scheduler """
    def __init__(self, fleetname, version, versions, manufacturer,
                dbconn, mqttclient=None, task_dict=None):

        self.version = version # '1.0.0'
        self.manufacturer = manufacturer # 'birfen'
        self.fleetname = fleetname # 'kullar'
        self.versions = versions # 'v1'

        # Initialize the FmTrafficHandler
        self.traffic_handler = FmTrafficHandler(
            self.fleetname,
            self.version,
            self.versions,
            self.manufacturer,
            dbconn,
            mqttclient,
            task_dict)

        # initialize the task network dictionary
        self.task_dictionary = task_dict if task_dict else {}

        self.iteration_count = 0
        self.send_sms = False
        self.start_robot_id = None
        self.iteration_interval = 40 # 300
        self.last_node_id = ''
        self.state_order_id = ''
        self.home_dock_loc_ids = []
        self.charge_dock_loc_ids = []
        self.station_dk_loc_ids = []
        self.header_id = 0

        # ----------------------
        # Idle Time Tracking
        # ----------------------
        # This dictionary holds idle time info for each robot.
        # Format:
        # { robot_id: { "idle_time": total_idle_time_in_seconds,
        #               "start_time": timestamp_when_robot_became_idle or None } }
        self.idle_tracker = {}


    # ===================== idle Analytics =====================
    def track_idle_time(self, r_id):
        """
        Track and update idle time for the given robot.
        Update the idle time for a robot.
        - If the robot's last_node_id is in home_dock_ids, the robot is idle.
          * If this is the first detection of idleness, record the start time.
          * If already idle, accumulate the difference (current_time - last recorded start_time)
            into the cumulative idle_time, then update start_time.
        - If the robot's last_node_id is not in home_dock_ids, then reset start_time (stop tracking).

        """
        current_time = time.time()

        # Check if robot is idle (last_node_id is in home dock)
        is_idle = self.last_node_id in self.home_dock_loc_ids

        if r_id not in self.idle_tracker:
            self.idle_tracker[r_id] = {"idle_time": 0, "start_time": 0}

        if is_idle:
            if self.idle_tracker[r_id]["start_time"] == 0:
                # Start tracking idle time
                self.idle_tracker[r_id]["start_time"] = current_time
            else:
                # Accumulate idle time
                elapsed_time = current_time - self.idle_tracker[r_id]["start_time"]
                self.idle_tracker[r_id]["idle_time"] += elapsed_time
                self.idle_tracker[r_id]["start_time"] = current_time  # Reset start time for continuous tracking
        else:
            # Robot is no longer idle
            self.idle_tracker[r_id]["start_time"] = 0

    def compute_average_idle_time(self):
        """
        Compute the average idle time per robot.
        Returns a dictionary: { robot_id: cumulative_idle_time_in_seconds }
        """
        avg_idle = {}
        for robot, record in self.idle_tracker.items():
            avg_idle[robot] = record["idle_time"]
        return avg_idle

    def compute_overall_idle_metrics(self, show_plot=True):
        """
        Compute overall idle metrics across the fleet.
        For example, aggregate by computing:
          - The number of robots (N)
          - The overall average idle time = sum(idle_time for each robot)/N
        Then, plot a bar chart where the x-axis is the number of robots and the y-axis is the overall average idle time.
        """
        avg_idle = self.compute_average_idle_time()
        num_robots = len(avg_idle)
        if num_robots > 0:
            overall_avg = sum(avg_idle.values()) / num_robots
        else:
            overall_avg = 0

        overall_data = {num_robots: overall_avg}
        if show_plot:
            self.traffic_handler.task_handler.order_handler.terminal_bar_chart(
                overall_data, xlabel="Number of Robots", title="Overall Average Idle Time vs. Robot Count")
        return num_robots, overall_avg, avg_idle

    # -------------------------------------------------------------------

    def manage_robot(self, f_id, r_id, m_id=None, v_id=None):
        """ Main loop runs wether accessed from terminal or api to manage the robot and fleet operations. """

        f_id = f_id or self.fleetname
        m_id = m_id or self.manufacturer
        v_id = v_id or self.version
        if r_id is None:
            return

        self.iteration_count += 1

        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # check if robot is task free
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        # check if robot is online and at home or free:
        cleared, current_position, bat_level, self.last_node_id, self.state_order_id, \
        order_timestamp, self.home_dock_loc_ids, self.charge_dock_loc_ids, self.station_dk_loc_ids, \
        idle_time, max_payload = self.traffic_handler.task_handler.verify_robot_fitness(f_id, r_id, m_id, 'charge', 0.0)

        # ------------------------------------------------------------------
        # Here we update idle time tracking:
        # If the robot's last_node_id is in the home dock IDs, then the robot is idle.
        # ------------------------------------------------------------------
        self.track_idle_time(r_id)

        # visualize graph and nav
        self.traffic_handler.task_handler.visualization_handler.robot_positions[r_id] = \
            {'x': current_position[0], 'y': current_position[1]}
        self.traffic_handler.task_handler.visualization_handler.terminal_graph_visualization()

        # Fetch orders for a specific fleet and serial number
        latest_order = self.traffic_handler.task_handler.order_handler.fetch_data(f_id, r_id, m_id)

        # init robot: if robot has no order history, for traffic update, insert its current location as a default order target.
        if not latest_order or (r_id in self.traffic_handler.task_handler.cp_ignore_list and
                                r_id not in self.traffic_handler.task_handler.ignore_list):
            # 6.7.3 Map download: The map download is triggered by the `downloadMap` instant action from the master control.
            # This command contains the mandatory parameters `mapId` and `mapDownloadLink` under which the map is stored on the map server and which can be accessed by the vehicle.

            records = self.traffic_handler.task_handler.visualization_handler.fetch_all_data(f_id)
            if records:
                for map_ in records:
                    self.header_id += 1
                    downloadmap_action_params = {
                        "mapId": map_["map_name"], # the identification is the map name.
                        "mapDownloadLink": map_["map_id"] # the map id here is the serial number on the database. so that the robot may fetch using name and s/n for download.
                    }
                    # get the robot to download the map of its location. untop which we would init the robot.
                    downloadmap_action = self.traffic_handler.task_handler.instant_actions_handler.create_action("downloadMap", downloadmap_action_params)
                    action_list = [downloadmap_action]
                    self.traffic_handler.task_handler.instant_actions_handler.build_instant_action_msg(f_id, r_id, self.header_id, v_id, m_id, action_list)

                # register or init robot on the map.
                self._handle_no_latest_order(f_id, r_id)


        # manage current robot
        traffic_control, unassigned_tasks = self.traffic_handler.manage_traffic(
            f_id=f_id, r_id=r_id, m_id=m_id, v_id=v_id)

        if self.iteration_count >= self.iteration_interval:
            self._handle_iteration_interval(
                f_id, r_id, m_id, v_id, cleared, bat_level, traffic_control, unassigned_tasks)

        # TODO!
        # monitor fleet wide error and send notification to users if necessary
        # check all fleets for any emergency to notify user.
        # errors_ = self.traffic_handler.temp_fb_errors # fb_rec["errors"]
        # self.check_notification_msgs(f_id, r_id, errors_)

    def _handle_no_latest_order(self, f_id, r_id):
        """ handle no latest order. """
        # log viz:
        self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
            "robot has no active order. manager will register robot's current node.",
            "FmScheduleHandler",
            "handle_no_latest_order",
            "info")
        if r_id in self.traffic_handler.task_handler.cp_ignore_list and \
            r_id not in self.traffic_handler.task_handler.ignore_list:
            # remove r_id from self.traffic_handler.cp_ignore_list as well.
            self.traffic_handler.task_handler.cp_ignore_list.remove(r_id)

        # check if its not at home, then send it home.
        if (self.last_node_id not in self.home_dock_loc_ids) and \
            (self.last_node_id not in self.charge_dock_loc_ids):
            # go to any random home dock for now
            random_dock_id = random.choice(self.home_dock_loc_ids)
            # request the robot task to go to one of the free home dock station directly.
            self._move_to_dock(f_id, r_id, random_dock_id, 'move')

        # if we are at a charge dock by default or home dock, then we dont need to move away.
        # we only need register the robot to the traffic control list.
        elif (self.last_node_id in self.home_dock_loc_ids) or \
            (self.last_node_id in self.charge_dock_loc_ids):
            # this does not plan for path!
            # filing a task does not publish any target, merely writes the robots occupied node into traffic.
            checkpoints=[self.last_node_id]
            self.traffic_handler.task_handler.fm_file_task(
                f_id = f_id,
                r_id = r_id,
                checkpoints=checkpoints,
                waitpoints=[],
                agv_itinerary=self.traffic_handler.task_handler.fm_get_itinerary(checkpoints, self.task_dictionary),
                wait_itinerary=[],
                landmark=[0.0, 'low', 'move' if self.last_node_id in self.home_dock_loc_ids else 'charge', self.last_node_id])


    def _handle_iteration_interval(self, f_id, r_id, m_id, v_id, cleared,
                                   bat_level, traffic_control, unassigned_tasks):
        # Reset counter to avoid overflow
        if self.start_robot_id == r_id:
            self.start_robot_id = None
            self.iteration_count = 0
            return
        # keep first robot temporarily so we know when to stop
        if self.iteration_count == self.iteration_interval:
            self.start_robot_id = r_id

        # this function belongs in traffic handler.
        # Verify robot's readiness for task assignment
        task_cleared = False
        robot_unassigned_tasks = []

        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # we can try auto charge task request (✔️)
        # we can try to re-assign queued task (✔️)
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        # if we:
        # 1. are at a home dock.
        # 2. not at a home dock but task was cancelled.
        if cleared:

            # task action is completed.
            _ = self.traffic_handler.task_handler.order_handler.cleanup_orders(
                f_id=f_id, r_id=r_id, m_id=m_id, cleared=cleared)

            # autocahrge: check battery level of each robot in fleet.
            if bat_level <= self.traffic_handler.task_handler.min_charge_level:
                task_cleared = self._handle_low_battery(f_id, r_id, traffic_control)

            # If unassigned tasks exist, sort by timestamp (asc) and priority (desc)
            sorted_unassigned_tasks = []
            if unassigned_tasks:

                # log viz: # Debug Print: Print the unassigned tasks before sorting
                self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
                    f"unassigned tasks: {unassigned_tasks}",
                    "FmScheduleHandler",
                    "handle_iteration_interval",
                    "info")

                # Debug Print: Print individual elements of unassigned_tasks
                # for idx, task in enumerate(unassigned_tasks):
                    # Debug: unassigned_task[0] - ('unassigned_AGV-001_923f1323-2a1b-4f88-8a67-da834ce43c2a_0', datetime.datetime(2025, 2, 22, 10, 10, 20, 677451), [0,0, 'high', 'transport', 'C3', 'C5'])
                    # print(f"Debug: unassigned_task[{idx}] - {task}")

                # Define task outside the loop to avoid undefined variable error
                task = None

                try:
                    # Sorting the unassigned tasks: (r_id, timestamp, dock_params)
                    sorted_unassigned_tasks = sorted(
                        unassigned_tasks,
                        key=lambda task: (task[1], -self.traffic_handler.map_priority(task[2][1]))  # timestamp & priority
                    )
                except IndexError as e:
                    # Print the error with context
                    print(f"IndexError: {e} - Task causing error: {task}")

                # The rest of your code...
                # Gather only the unassigned tasks for this specific robot
                robot_unassigned_tasks = [
                    task for task in sorted_unassigned_tasks
                    if r_id == self.traffic_handler.task_handler.extract_r_id(task[0])]

                # Check if there’s a "charge" task to delete, given that a charge task was successfully assigned
                if any(task[2][2] == 'charge' for task in robot_unassigned_tasks) and task_cleared:
                    self._delete_charge_tasks(f_id, m_id, robot_unassigned_tasks)

                # if we never issued a charge task to this robot. but it has unassigned tasks then,
                elif not task_cleared:
                    # Assign the first task in the sorted unassigned list to the robot
                    self._assign_unassigned_tasks(f_id, r_id, m_id, robot_unassigned_tasks, sorted_unassigned_tasks)

        # we probably:
        # 1. currently on a task.
        # 2. finished a task but it is a loop task.
        else:
            # if task was to charge, we must undock if done charging? go back home.
            # are we at a charge dock?
            # -- check if robot occupies a charge dock.
            if self.last_node_id in self.charge_dock_loc_ids:
                # - if battery level is high enough...
                if bat_level >= self.traffic_handler.task_handler.max_charge_level:
                    # -- if it occupies a charge dock, check if any home dock is free.
                    # -- get all charge docks and fetch the ones not in traffic -unoccupied-
                    self._request_home_dock(f_id, r_id, m_id, v_id, traffic_control, 'charge')

            # we are not at a charge dock, so we must be on a task or something.
            elif self.last_node_id in self.station_dk_loc_ids:
                # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                # check if its loop and as such we must re-enter task
                # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                if self._is_loop_task():
                    self._reassign_loop_task(f_id, r_id, m_id, v_id, traffic_control)


    def _is_loop_task(self):
        """ check if robot currently on a loop task. """
        # check if last node in checkpoints is last_node_id, then we can check if is a loop
        # if loop task no home targets by default only to and from. hence if 1 is left it must be a station id.
        return (len(self.traffic_handler.temp_fb_horizon) == 1 and
                self.traffic_handler.temp_fb_landmarks[2] == 'loop' and
                self.traffic_handler.temp_fb_dock_action_done and
                self.traffic_handler.temp_fb_agv_status == 'red')


    def _reassign_loop_task(self, f_id, r_id, m_id, v_id, traffic_control):
        """ manager will reassign loop task. """
        # place action is completed.
        _ = self.traffic_handler.task_handler.order_handler.cleanup_orders(
            f_id=f_id, r_id=r_id, m_id=m_id, cleared=True)

        # wait some seconds:
        time.sleep(0.5)

        # Attempt to reassign loop task to the current robot
        task_cleared, available_rbts, _, _, _, _, _ = self.traffic_handler.task_handler.fm_send_task_request(
            f_id=f_id,
            r_id=r_id,  # Assign to the current robot instead
            from_loc_id=self.traffic_handler.temp_fb_landmarks[3],
            to_loc_id=self.traffic_handler.temp_fb_landmarks[4],
            task_name=self.traffic_handler.temp_fb_landmarks[2],
            task_priority=self.traffic_handler.temp_fb_landmarks[1],
            task_dictionary=None,
            unassigned=False, # this is not an unassigned task
            override_cleared=False, # no dont override clear status
            payload_kg=self.traffic_handler.temp_fb_landmarks[0]
        )
        # if the task was not cleared, cool!
        # or if task was cleared but for an alternative robot. how will you know?
        if (task_cleared is False) or (available_rbts and task_cleared):
            # request the robot task to go to one of the free home dock station directly.
            self._request_home_dock(f_id, r_id, m_id, v_id, traffic_control, 'loop')


    def _request_home_dock(self, f_id, r_id, m_id, v_id, traffic_control, src_task):
        """ go to home dock. """
        # check all home docks for the free or unoccupied one.
        for node in self.home_dock_loc_ids:
            # -- if an home dock is free,
            if traffic_control is not None and node not in traffic_control:
                # If stopCharging | - | Deactivation of the charging process is in progress | - | The charging process is stopped.
                if src_task == 'charge':
                    # charge action is completed.
                    _ = self.traffic_handler.task_handler.order_handler.cleanup_orders(
                        f_id=f_id, r_id=r_id, m_id=m_id, cleared=True)
                    # "approx. time for stopcharge_action"
                    stopcharge_action_params = {"key": "duration",
                                                "value": "2.0"}
                    self.header_id += 1
                    stopcharge_action = self.traffic_handler.task_handler.instant_actions_handler.create_action(
                        "stopCharging", stopcharge_action_params)
                    action_list = [stopcharge_action]
                    self.traffic_handler.task_handler.instant_actions_handler.build_instant_action_msg(
                        f_id, r_id, self.header_id, v_id, m_id, action_list)
                # autocharge will handle the rest for when there is a free charge station.
                # because if all charge station is occupied or robot stays at drop node, then robot blocks reserves node forever.
                task_cleared = self._move_to_dock(f_id, r_id, node, 'move')
                if task_cleared:
                    return


    def _move_to_dock(self, f_id, r_id, target_dock, task_name):
        """ go to home dock. """
        task_cleared, _, _, _, _, _, _ \
            = self.traffic_handler.task_handler.fm_send_task_request(
                f_id=f_id,
                r_id=r_id,
                from_loc_id=self.last_node_id, # from_loc_id,
                to_loc_id=target_dock, # to_loc_id,
                task_name=task_name, # task_name,
                task_priority='low', # task_priority
                task_dictionary=None, # use default graph in class.
                unassigned=False, # this is not an unassigned task
                override_cleared=True, # since we already checked previously
                # payload_kg=0.0 # [default] since its a move/charge task.
                )
        return task_cleared

    def _handle_low_battery(self, f_id, r_id, traffic_control):
        """ _handle_low_battery. """
        # -- get all charge docks and fetch the ones not in traffic -unoccupied-
        for node in self.charge_dock_loc_ids:
            if traffic_control is not None and \
                node not in traffic_control:
                # -- request the robot task to go to the free charge stations to charge.
                task_cleared = self._move_to_dock(f_id, r_id, node, 'charge')
                # 4. break, for we will delete the queued task now.
                if task_cleared:
                    return task_cleared
        return False

    def _delete_charge_tasks(self, f_id, m_id, robot_unassigned_tasks):
        """ delete_charge_tasks. """
        for task in robot_unassigned_tasks:
            # If a charge task was assigned, delete all existing "charge" tasks for this robot
            if task[2][2] == 'charge':
                self.traffic_handler.task_handler.order_handler.delete_data(
                    f_id, task[0], m_id)


    def _assign_unassigned_tasks(self, f_id, r_id, m_id, robot_unassigned_tasks, sorted_unassigned_tasks):
        """ assign unassigned tasks. """
        for task in robot_unassigned_tasks:
            # since robot is cleared for any task and we didnt just issue a charge task to it,
            # here we need to take the first transport or loop or charge and assign it to the robot.
            # landmark=[payload, task_priority, task_name, from_loc_id, to_loc_id, payload_kg]
            task_cleared, _, _, _, _, _, _ = self.traffic_handler.task_handler.fm_send_task_request(
                f_id=f_id,
                r_id=r_id,
                from_loc_id=task[2][3],   # from_loc_id
                to_loc_id=task[2][4],     # to_loc_id
                task_name=task[2][2],     # task_name
                task_priority=task[2][1], # task_priority
                task_dictionary=None,
                unassigned=True,
                override_cleared=True,     # Since it had been payload validated earlier.
                payload_kg=task[2][0]      # payload_kg
            )
            # Delete the assigned task from the unassigned tasks if successfully assigned
            if task_cleared:
                self.traffic_handler.task_handler.order_handler.delete_data(
                    f_id, task[0], m_id)
                return

        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # Can these tasks be given to other free robots?
        # Which order should be handled next? Order time and priority shuffle?
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        # however, if we never issued any charge task to this robot, and it has no unnassigned task, then
        # check if any task assigned to a currently working robot as passed the delay tolerance.
        # then consider assigning it to this robot to handle.
        # Loop through the sorted unassigned tasks
        for unassigned_id, timestamp, landmark in sorted_unassigned_tasks:
            # Calculate the time difference between the current time and the task timestamp
            time_elapsed_minutes = (datetime.datetime.now() - timestamp).total_seconds() / 60 # Convert to minutes
            if time_elapsed_minutes > self.traffic_handler.task_handler.max_task_delay and landmark[2] in ['transport', 'loop']:
                # Attempt to reassign task to the current robot
                task_cleared, _, _, _, _, _, _ = self.traffic_handler.task_handler.fm_send_task_request(
                    f_id=f_id,
                    r_id=r_id,  # Assign to the current robot instead
                    from_loc_id=landmark[3],   # from_loc_id
                    to_loc_id=landmark[4],     # to_loc_id
                    task_name=landmark[2],     # task_name
                    task_priority=landmark[1], # task_priority
                    task_dictionary=None,
                    unassigned=True,
                    override_cleared=False, # True
                    payload_kg=landmark[0]     # payload_kg
                )
                # Delete the assigned task from the unassigned tasks if reassignment is successful
                if task_cleared:
                    self.traffic_handler.task_handler.order_handler.delete_data(
                        f_id, unassigned_id, m_id)
                return


    def cancel_task(self, f_id, r_id, m_id, v_id):
        """ cancel task. """
        # publish cancel task action
        self.traffic_handler.task_handler.cancel_task_db_cmd(f_id,
                                                             r_id,
                                                             m_id,
                                                             v_id,
                                                             self.state_order_id)
        # clear the several updates from this specific order_id
        success = self.traffic_handler.task_handler.order_handler.cleanup_orders(
            f_id=f_id, r_id=r_id, m_id=m_id, cleared=False)
        if success:
            # go to any random home dock for now
            random_dock_id = random.choice(self.home_dock_loc_ids)
            task_cleared = self._move_to_dock(f_id, r_id, random_dock_id, 'move')
            # if success, then show that task was cancelled succesfully
            if task_cleared:
                # log viz:
                self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
                    "task cancelled succesfully.",
                    "FmScheduleHandler",
                    "cancel_task",
                    "info")
                return
        # log viz:
        self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
            "task cancellation encountered a db related problem please try again.",
            "FmScheduleHandler",
            "cancel_task",
            "error")

    def check_notification_msgs(self, f_id, r_id, errors_):
        """ external notification: send message to user via text. """
        if self.send_sms is False:
            return
        try:
            external_msg = errors_[0] # elevator_required | startup_failed | startup_success | recovery_required
            _ = errors_[1] # internal_msg --> dock_required | warning_stop | warning_slow | no_warning | undock_completed etc.
            # SMS: we need to determine wether or not a message should be sent or not
            if (external_msg == 'elevator_required') \
                or (external_msg == 'recovery_required'): # (external_msg == 'startup_failed')
                self.fm_send_notification_msg(f_id, r_id, external_msg)
        except (ValueError, TypeError) as error:
            # log viz:
            self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
                f"{error}.",
                "FmScheduleHandler",
                "check_notification_msgs",
                "warn")
            return


    def fm_send_notification_msg(self, f_id, r_id, external_msg, task_dictionary=None):
        """ external notification: send message to user via text. """

        task_dictionary = task_dictionary or self.task_dictionary

        if self.send_sms is False:
            return
        try:
            account_sid = str(task_dictionary["twilio_server"]["account_sid"])
            auth_token = str(task_dictionary["twilio_server"]["auth_token"])
            from_number = str(task_dictionary["twilio_server"]["from_number"])
            to_number = str(task_dictionary["twilio_server"]["to_number"])
            client = Client(account_sid, auth_token)
            _ = client.messages.create(
                to = to_number,
                from_ = from_number,
                body = "Hello! Robot "+r_id+" in fleet "+f_id+" requires attention "+str(external_msg)+". Please inspect. ")
        except (ValueError, TypeError) as error:
            # log viz:
            self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
                f"{error}.",
                "FmScheduleHandler",
                "fm_send_notification_msg",
                "warn")

    def fm_send_factsheet_request(self, m_id, v_id):
        """ request factsheet """
        conn_recs = self.traffic_handler.task_handler.connection_handler.fetch_all_data(m_id)

        # Use dynamic index lookup instead of hardcoding column indices
        timestamp_index = self.traffic_handler.task_handler.connection_handler.table_col.index('timestamp')
        serial_number_index = self.traffic_handler.task_handler.connection_handler.table_col.index('serial_number')
        connection_state_index = self.traffic_handler.task_handler.connection_handler.table_col.index('connection_state')

        # Loop through the recs to find rows where 'r_id' and 'conn_state'.
        for conn_rec in conn_recs:
            _ = conn_rec[timestamp_index+1]
            serial_number = conn_rec[serial_number_index+1]
            _ = conn_rec[connection_state_index+1]

            # request their respective factsheet so we can ascertain the f_id.
            factsheet_req_action = self.traffic_handler.task_handler.instant_actions_handler.create_action("factsheetRequest",{})
            self.header_id += 1
            action_list = [factsheet_req_action]
            self.traffic_handler.task_handler.instant_actions_handler.build_instant_action_msg(
                None, serial_number, self.header_id, v_id, m_id, action_list = action_list)

    # def fm_analytics(self, f_id, m_id, r_id=None, debug_level='info'):
    #     """ analytics """
    #     completed_orders = self.traffic_handler.task_handler.order_handler.fetch_completed_tasks(
    #         f_id, m_id, cleared=True, task_type=None, r_id=None)
    #     # log viz:
    #     self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #         f"number of total completed Orders: {len(completed_orders)}.",
    #         "FmScheduleHandler",
    #         "fm_analytics",
    #         debug_level)

    #     cancelled_orders = self.traffic_handler.task_handler.order_handler.fetch_completed_tasks(
    #         f_id, m_id, cleared=False, task_type=None, r_id=None)
    #     # log viz:
    #     self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #         f"number of total cancelled Orders: {len(cancelled_orders)}.",
    #         "FmScheduleHandler",
    #         "fm_analytics",
    #         debug_level)

    #     charge_orders = self.traffic_handler.task_handler.order_handler.fetch_completed_tasks(
    #         f_id, m_id, cleared=True, task_type='charge', r_id=None)
    #     # log viz:
    #     self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #         f"number of charge Orders fullfilled: {len(charge_orders)}.",
    #         "FmScheduleHandler",
    #         "fm_analytics",
    #         debug_level)

    #     active_orders, unassigned_orders = self.traffic_handler.task_handler.order_handler.fetch_active_and_unassigned_tasks(
    #         f_id, m_id)
    #     # log viz:
    #     self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #         f"number of currently active Orders: {len(active_orders)}.",
    #         "FmScheduleHandler",
    #         "fm_analytics",
    #         debug_level)
    #     # log viz:
    #     self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #         f"number of currently unassigned Orders: {len(unassigned_orders)}.",
    #         "FmScheduleHandler",
    #         "fm_analytics",
    #         debug_level)

    #     # ---------- .
    #     num_robots, total_cum_delay = self.traffic_handler.task_handler.order_handler.calculate_completed_delays(
    #         duration_window_sec=7200)
    #     # Log viz:
    #     self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #         f"number of robots: {num_robots}: cummulative delays: {total_cum_delay}.",
    #         "FmScheduleHandler",
    #         "fm_analytics",
    #         'critical')

    #     avg_times = self.traffic_handler.task_handler.order_handler.compute_average_execution_duration(
    #         show_plot=True)
    #     # Print the average execution duration for each robot
    #     for robot, avg_time in avg_times.items():
    #         # Log viz:
    #         self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #             f"Robot ID: {robot}, Average Execution Duration: {avg_time:.2f} seconds",
    #             "FmScheduleHandler",
    #             "fm_analytics",
    #             debug_level)

    #     timestamps, throughput_values = self.traffic_handler.task_handler.order_handler.compute_overall_throughput(
    #         duration_minutes=120, show_plot=True)
    #     # Iterate through the timestamps and throughput values
    #     for timestamp, throughput in zip(timestamps, throughput_values):
    #         # Log viz:
    #         self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #             f"timestamp [min]: {timestamp}, Throughput: {throughput:.2f}.",
    #             "FmScheduleHandler",
    #             "fm_analytics",
    #             debug_level)

    #     avg_per_robot_latencies = self.traffic_handler.task_handler.state_handler.compute_robot_avg_latency(
    #         show_plot=True)
    #     # Log viz:
    #     self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #         f"avg per robot latency [sec]: {avg_per_robot_latencies}.",
    #         "FmScheduleHandler",
    #         "fm_analytics",
    #         debug_level)

    #     num_robots, overall_avg = self.traffic_handler.task_handler.state_handler.compute_system_avg_latency(
    #         show_plot=True)
    #     # Log viz:
    #     self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #         f"num robots: {num_robots}, overall avg [sec]: {overall_avg:.2f}.",
    #         "FmScheduleHandler",
    #         "fm_analytics",
    #         debug_level)

    #     num_robots, overall_avg, per_robot_avrge = self.compute_overall_idle_metrics(show_plot=True)
    #     # Log viz:
    #     self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #         f"avg per robot Idle Time [sec]: {per_robot_avrge}.",
    #         "FmScheduleHandler",
    #         "fm_analytics",
    #         'critical')

    #     # ---------- .

    #     if r_id:
    #         robot_orders = self.traffic_handler.task_handler.order_handler.fetch_completed_tasks(
    #             f_id, m_id, cleared=True, task_type='transport', r_id=r_id)
    #         # log viz:
    #         self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
    #             f"number of transport Orders fullfilledby robot {r_id}: {len(robot_orders)}.",
    #             "FmScheduleHandler",
    #             "fm_analytics",
    #             debug_level)

    def fm_analytics(self, f_id, m_id, r_id=None, debug_level='info', write_to_file=False):
        """ analytics """

        # Collect all log messages in a list
        log_messages = []

        def log_and_store(message, level=debug_level):
            # Terminal logging
            self.traffic_handler.task_handler.visualization_handler.terminal_log_visualization(
                message,
                "FmScheduleHandler",
                "fm_analytics",
                level
            )
            # Store for file output
            log_messages.append(message)

        # --- Use log_and_store instead of direct logging ---
        completed_orders = self.traffic_handler.task_handler.order_handler.fetch_completed_tasks(
            f_id, m_id, cleared=True, task_type=None, r_id=None)
        log_and_store(f"number of total completed Orders: {len(completed_orders)}.")

        cancelled_orders = self.traffic_handler.task_handler.order_handler.fetch_completed_tasks(
            f_id, m_id, cleared=False, task_type=None, r_id=None)
        log_and_store(f"number of total cancelled Orders: {len(cancelled_orders)}.")

        charge_orders = self.traffic_handler.task_handler.order_handler.fetch_completed_tasks(
            f_id, m_id, cleared=True, task_type='charge', r_id=None)
        log_and_store(f"number of charge Orders fullfilled: {len(charge_orders)}.")

        active_orders, unassigned_orders = self.traffic_handler.task_handler.order_handler.fetch_active_and_unassigned_tasks(
            f_id, m_id)
        log_and_store(f"number of currently active Orders: {len(active_orders)}.")
        log_and_store(f"number of currently unassigned Orders: {len(unassigned_orders)}.")

        num_robots, total_cum_delay = self.traffic_handler.task_handler.order_handler.calculate_completed_delays(
            duration_window_sec=7200)
        log_and_store(f"number of robots: {num_robots}: cummulative delays: {total_cum_delay}.", level='critical')

        avg_times = self.traffic_handler.task_handler.order_handler.compute_average_execution_duration(show_plot=True)
        for robot, avg_time in avg_times.items():
            log_and_store(f"Robot ID: {robot}, Average Execution Duration: {avg_time:.2f} seconds")

        timestamps, throughput_values = self.traffic_handler.task_handler.order_handler.compute_overall_throughput(
            duration_minutes=120, show_plot=True)
        for timestamp, throughput in zip(timestamps, throughput_values):
            log_and_store(f"timestamp [min]: {timestamp}, Throughput: {throughput:.2f}.")

        avg_per_robot_latencies = self.traffic_handler.task_handler.state_handler.compute_robot_avg_latency(show_plot=True)
        log_and_store(f"avg per robot latency [sec]: {avg_per_robot_latencies}.")

        num_robots, overall_avg = self.traffic_handler.task_handler.state_handler.compute_system_avg_latency(show_plot=True)
        log_and_store(f"num robots: {num_robots}, overall avg [sec]: {overall_avg:.2f}.")

        num_robots, overall_avg, per_robot_avrge = self.compute_overall_idle_metrics(show_plot=True)
        log_and_store(f"avg per robot Idle Time [sec]: {per_robot_avrge}.", level='critical')

        if r_id:
            robot_orders = self.traffic_handler.task_handler.order_handler.fetch_completed_tasks(
                f_id, m_id, cleared=True, task_type='transport', r_id=r_id)
            log_and_store(f"number of transport Orders fullfilled by robot {r_id}: {len(robot_orders)}.")

        # ---------- FILE OUTPUT ----------
        if write_to_file:
            # logs_dir = os.path.abspath("logs")

            # Get the directory of the current script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            # Go one folder backwards
            parent_dir = os.path.dirname(script_dir)
            # Create "logs" folder there
            logs_dir = os.path.join(parent_dir, "logs")
            os.makedirs(logs_dir, exist_ok=True)

            base_name = "result_snapshot"
            # find all existing snapshot files
            existing = glob.glob(os.path.join(logs_dir, f"{base_name}_*.txt"))

            if existing:
                # extract numeric suffixes and compute next index
                indices = [
                    int(os.path.splitext(f)[0].split("_")[-1])
                    for f in existing
                    if os.path.splitext(f)[0].split("_")[-1].isdigit()
                ]
                next_index = max(indices) + 1 if indices else 1
            else:
                next_index = 1

            filename = os.path.join(logs_dir, f"{base_name}_{next_index}.txt")

            with open(filename, "w") as f:
                for msg in log_messages:
                    f.write(msg + "\n")

            print(f"✅ Results written to {filename}")


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
    task_dict_ = None
    # get config file path:
    agv_dir_prefix = (os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
    print("Config directory path: ", agv_dir_prefix)
    file_path = Path(os.path.join(agv_dir_prefix, 'fleet_management', 'config', 'config.yaml'))
    if file_path.is_file():
        with open(file_path, 'r', encoding='utf-8') as yaml_file:
            task_dict_ = yaml.safe_load(yaml_file)
    else:
        print("file path not set.")

    # Initialize the StateSubscriber: Generate order message
    fleetname = "kullar"
    robot_serial_number = "R1" # "AGV-001" # "SN12345678"
    robot_serial_number2 = "R2" # "AGV-002"
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
            "batteryCharge": 81.0, # 85.0,
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
            "maxLoadMass": 57,
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

    # Initialize the FmScheduleHandler
    schedule_handler = FmScheduleHandler(
        fleetname = fleetname,
        version = version,
        versions = versions,
        manufacturer = manufacturer,
        dbconn = conn,
        mqttclient=None,
        task_dict=task_dict_)

    # Pause to ensure the message is inserted
    time.sleep(0.5)

    # Step 3: Insert sample message into database
    schedule_handler.traffic_handler.task_handler.factsheet_handler.process_message(sample_factsheet_message)
    schedule_handler.traffic_handler.task_handler.connection_handler.process_message(sample_conn_message)
    schedule_handler.traffic_handler.task_handler.state_handler.process_message(sample_state_message)

    # Test fm send_task_request method
    # for move task only to_loc_id is considered.
    task_c, available_r, checkps, \
        checkp_itinrry, waitps, \
            wait_itinrry, landm_ = schedule_handler.traffic_handler.task_handler.fm_send_task_request(
                f_id=fleetname,
                r_id=robot_serial_number,
                from_loc_id='C3',
                # to_loc_id='C11', # C11, C12 - home docks
                # task_name='move',
                to_loc_id='C5', # C3, C5 - station docks
                task_name='transport',
                task_priority='high',
                payload_kg=45)

    # request factsheet from all robots.
    schedule_handler.fm_send_factsheet_request(manufacturer, version)
    # assuming the factsheets were sent, fetch the fleet that the robots belong to
    f_ids = schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_fleets()

    while len(f_ids) == 0:
        # keep requesting for factsheet
        f_ids = schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_fleets()
        time.sleep(1.0)

    print("f_ids: ", f_ids)
    for f_id_ in f_ids:
        # assuming the fleets were fetched successfully, return the robots in each fleet.
        r_ids = schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_serial_numbers(f_id_)
        print("r_ids: ", r_ids)
        if r_ids:
            for _r_id in r_ids:
                # Initialize the FmTrafficHandler too
                COUNT = 0
                while COUNT <= 3: # 3:
                    COUNT += 1
                    print("\ncount: ", COUNT)
                    # manage the robot
                    schedule_handler.manage_robot(f_id=fleetname, r_id=_r_id, m_id=None, v_id=None)

    schedule_handler.fm_analytics(fleetname, manufacturer, r_id=None, debug_level='info')
