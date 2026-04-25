import json, os, logging, datetime, time, uuid, math, re, sys, threading
from psycopg2 import sql
import psycopg2, psycopg2.extras

import numpy as np


    # Rank	Robot ID	Avg Time (sec)	Interpretation
    # 🥇	R04	79.14s	The MVP. Fastest cycles. Likely assigned close targets.
    # 🥈	R03	82.70s	Very efficient.
    # ...	...	...	...
    # 🐢	R08	204.03s	Bottleneck. Takes 2.5x longer than R04.

    # Log Index:      1      10 ...     115      116      117 ... 120 (Now)
    # Real Time:    -2hrs   -1.8hrs ... -5min    -4min    -3min   0min
    #                |____________________|________|________|______|
    # Data:             [EMPTY SPACE]           [SIMULATION RUNS HERE]
    # Throughput:          0.00                    1.00 ...

    # Time (s):   0s (Start)       73s (Min 116)                   313s (End/Min 120)
    #             |                   |                                 |
    # Timeline:   [====== Task 1 =====] [----------- IDLE ------------] [== Task 2 ==]
    #             ^                   ^                                 ^
    #         Sim Starts         Task 1 Done                       Task 2 Done


class OrderPublisher:
    """ OrderPublisher """
    # Class-level shared registries to bridge multiple instances
    _shared_orders_issued = {}
    _shared_analytics_data = {}
    _shared_wait_analytics = {}
    _registry_lock = threading.RLock()

    def __init__(self, fleetname, versions, dbconn, mqttclient=None,
                 drop_table=False, dbname = 'postgres', tname = 'orders',
                 output_log=True):

        self.fleetname = fleetname
        self.versions = versions
        self.db_conn = dbconn
        self.mqtt_client = mqttclient
        self.table_order = tname

        self.drop_table = drop_table # True # False
        self.max_entry_per_robot = 250

        # Corrected list of column names based on the order table schema
        self.table_col = ['header_id', 'timestamp', 'version', 'manufacturer', 'serial_number', 'order_id',
                'order_update_id', 'zone_set_id', 'nodes', 'edges']

        self.output_to_screen = output_log
        self.output_to_file = output_log

        # logger; here we use a simple print-based one for clarity.
        self.logger = self._get_logger("OrderPublisher", output_log)

        self.create_database(dbname)

        # Reference the shared registries
        self.orders_issued = OrderPublisher._shared_orders_issued
        self.analytics_data = OrderPublisher._shared_analytics_data
        self.wait_analytics = OrderPublisher._shared_wait_analytics

        # ------------------- Wait Analytics -------------------
        # Format: { robot_id: { "wait_events": [ { "wait_start": <start>, "wait_end": <end>, "duration": <sec> } ],
        #               "cumulative_wait": <total_wait_seconds> } }
        self.current_order_wait = None

        # In-memory cache — keyed by serial_number, holds the tuple from db
        self.cache: dict = {}

        # We can implement a background loop to verify these if needed:--------------------------------------------------------------------------------------------

    def _get_logger(self, logger_name, output_log):

        logger = logging.getLogger(logger_name)
        logger.setLevel(logging.INFO)

        # Ensure handlers are not duplicated
        if not logger.hasHandlers() and output_log:
            # Set up logging to log file
            log_file_path = os.path.abspath("FmLogHandler.log")
            file_mode = 'a' if os.path.exists(log_file_path) else 'w'

            file_handler = logging.FileHandler(log_file_path, mode=file_mode)
            file_handler.setFormatter(
                logging.Formatter("[%(levelname)s] [%(asctime)s] %(name)s: %(message)s")
            )
            logger.addHandler(file_handler)

            if self.output_to_screen:
                # Output to terminal (stdout)
                stream_handler = logging.StreamHandler(sys.stdout)
                stream_handler.setFormatter(
                    logging.Formatter("[%(levelname)s] [%(asctime)s] %(name)s: %(message)s")
                )
                logger.addHandler(stream_handler)

            # Show log file path
            # logger.info(f"Logs are written to: {log_file_path}")
            pass

        return logger

    # --------------------------------------------------------------------------------------------

    # ===================== Throughput Analytics Methods =====================
    def compute_overall_throughput(self, show_plot=True):
        """
        Compute overall task throughput (tasks per minute) across all robots for each minute
        within the actual duration of the simulation.
        """
        timestamps = []
        throughput_values = []

        with OrderPublisher._registry_lock:
            all_tasks = [task for tasks in self.analytics_data.values() for task in tasks]
        
        if not all_tasks:
            return timestamps, throughput_values
            
        # Synchronize: use "issued_timestamp" which is recorded in record_order_issuance
        dispatch_times = [
            task["issued_timestamp"] if task.get("issued_timestamp") is not None 
            else task["completion_timestamp"] - task.get("execution_duration", 0)
            for task in all_tasks
        ]
        first_dispatch = min(dispatch_times)
        last_completion = max(task.get("completion_timestamp", time.time()) for task in all_tasks)
        
        # Calculate span in minutes
        actual_duration_seconds = max(last_completion - first_dispatch, 1.0)
        actual_duration_minutes = max(int(actual_duration_seconds / 60) + 1, 1)

        # Initialize bins for each minute
        bins = [0] * actual_duration_minutes
        for task in all_tasks:
            # Determine which minute bucket this completion falls into
            minute_idx = int((task["completion_timestamp"] - first_dispatch) / 60)
            if 0 <= minute_idx < actual_duration_minutes:
                bins[minute_idx] += 1
        
        timestamps = list(range(1, actual_duration_minutes + 1))
        throughput_values = bins

        if show_plot:
            self.terminal_bar_chart(
                data=dict(zip(timestamps, throughput_values)),
                xlabel="Minute",
                title=f"Overall Throughput Over {actual_duration_minutes} Minutes"
            )

        return timestamps, throughput_values


    # ===================== Wait Analytics Methods =====================
    def record_wait_event(self, robot_id, order_id, duration, is_completed=False):
        """
        Record a wait duration for a robot associated with a particular order.
        """
        with OrderPublisher._registry_lock:
            if robot_id not in self.wait_analytics:
                self.wait_analytics[robot_id] = {"orders": {}}

            # Initialize order tracking if not already present
            if order_id not in self.wait_analytics[robot_id]["orders"]:
                self.wait_analytics[robot_id]["orders"][order_id] = {
                    "cumulative_wait": 0.0,
                    "is_completed": False,
                    "completion_timestamp": None
                }

            # Update cumulative wait
            self.wait_analytics[robot_id]["orders"][order_id]["cumulative_wait"] += duration

            if is_completed:
                self.wait_analytics[robot_id]["orders"][order_id]["is_completed"] = True
                self.wait_analytics[robot_id]["orders"][order_id]["completion_timestamp"] = time.time()

            self.logger.info("Robot %s: Logged duration %.2fs for order %s. (Cumulative: %.2fs, Completed: %s)", 
                             robot_id, duration, order_id, 
                             self.wait_analytics[robot_id]["orders"][order_id]["cumulative_wait"],
                             is_completed)

    def process_wait_event(self, robot_id, order_id, wait_time):
        """
        Process a wait event for a robot using the provided wait_time duration.
        """
        if wait_time is None:
            return

        base_uuid = order_id.rsplit('_', 1)[0]  # Extract base UUID safely

        # Track processed order_ids to avoid double-counting re-transmissions
        try:
            duration = float(wait_time)
            self.record_wait_event(
                            robot_id, base_uuid, duration)
            self.logger.info("🛸 [WAIT RECORDED] Robot %s: Order %s wait duration %s", 
                         robot_id, order_id, duration)
        except (ValueError, TypeError):
            self.logger.warning("🛸 [WAIT SKIPPED] Robot %s: Order %s had invalid wait_time: %s", 
                             robot_id, order_id, wait_time)
            return

    def calculate_completed_delays(self, duration_window_sec=3600, **kwargs):
        """
        Compute:
        - The number of robots that have at least one completed order within the duration window (in seconds)
        - The total cumulative delay (wait time) for all completed orders within the window.
        """
        # current time in seconds since the epoch
        current_time = time.time()
        threshold_time = current_time - duration_window_sec

        num_robots = 0
        total_cum_delay = 0.0
        robots_with_data = set()

        # Optional robot filter from kwargs
        r_id = kwargs.get('r_id')

        with OrderPublisher._registry_lock:
            # Iterate over each robot's wait analytics data.
            for robot_id, robot_data in self.wait_analytics.items():
                # If r_id is specified in kwargs, skip other robots
                if r_id and robot_id != r_id:
                    continue
                    
                for order_id, order_data in robot_data.get("orders", {}).items():
                    # Check if the order is marked as completed and its completion timestamp is within the window.
                    if order_data.get("is_completed", False) and order_data.get("completion_timestamp", 0) >= threshold_time:
                        total_cum_delay += order_data["cumulative_wait"]
                        robots_with_data.add(robot_id)
        
            num_robots = len(robots_with_data)
                
        print(f"\033[91m 🛸 🛸 🛸 🛸 Fleet-wide Cumulative Delay: {total_cum_delay:.2f}s | Robots: {num_robots} (Window: {duration_window_sec}s) 🛸 🛸 🛸 🛸 \033[0m")
        return num_robots, total_cum_delay

    # ===================== Task Completion Analytics Methods =====================
    def record_order_issuance(self, robot_id, order_id):
        """
        Record that an order was issued for a given robot.
        Only the base UUID (everything before the first underscore) is stored.
        """
        base_uuid = order_id.rsplit('_', 1)[0]
        with OrderPublisher._registry_lock:
            if robot_id not in self.orders_issued:
                self.orders_issued[robot_id] = []

            # Avoid storing multiple timestamps for the same order
            for record in self.orders_issued[robot_id]:
                if record["order_id"] == base_uuid:
                    return  # Order already recorded

            issued_ts = time.time()
            self.orders_issued[robot_id].append({
                "order_id": base_uuid,
                "issued_timestamp": issued_ts
            })
        self.logger.info("🛸 [ISSUANCE] (Instance: %s) Robot %s: Recorded order %s at %s", id(self), robot_id, base_uuid, issued_ts)


    def record_order_completion(self, robot_id, order_id):
        """
        Record the task completion for a given robot by calculating the execution duration.
        """
        base_uuid = order_id.rsplit('_', 1)[0]
        
        issued_timestamp = None
        completion_timestamp = time.time()

        with OrderPublisher._registry_lock:
            # Idempotency check: don't record the same full order_id (with suffix) twice
            if robot_id in self.analytics_data:
                for task in self.analytics_data[robot_id]:
                    if task.get("full_order_id") == order_id:
                        return

            # Attempt to find the issuance timestamp for this order (by base UUID)
            if robot_id in self.orders_issued:
                for record in self.orders_issued[robot_id]:
                    if record["order_id"] == base_uuid:
                        issued_timestamp = record["issued_timestamp"]
                        break

            # --- DB FALLBACK ---
            # If issuance record is missing in memory (e.g. restart or multiple instances), 
            # query the database for the original '_0' or '_1' order timestamp.
            if issued_timestamp is None:
                try:
                    cursor = self.db_conn.cursor()
                    query = f"""
                        SELECT timestamp FROM {self.table_order}
                        WHERE serial_number = %s AND order_id LIKE %s
                        ORDER BY timestamp ASC LIMIT 1
                    """
                    cursor.execute(query, (robot_id, f"{base_uuid}%"))
                    row = cursor.fetchone()
                    if row:
                        db_ts = row[0]
                        # Handle both datetime objects and ISO strings
                        if isinstance(db_ts, str):
                            issued_timestamp = datetime.datetime.fromisoformat(db_ts).timestamp()
                        elif isinstance(db_ts, datetime.datetime):
                            issued_timestamp = db_ts.timestamp()
                    cursor.close()
                except Exception as db_err:
                    self.logger.error("🛸 [DB ERROR] Failed to fetch issuance timestamp from DB: %s", db_err)

            if issued_timestamp is None:
                self.logger.warning("🛸 [MISSING] No issuance record found for robot %s, order %s. Duration will be 0.", robot_id, base_uuid)
                execution_duration = 0
            else:
                execution_duration = completion_timestamp - issued_timestamp
                if execution_duration < 0:
                    self.logger.warning("Negative execution time detected for robot %s, order %s", robot_id, base_uuid)

            if robot_id not in self.analytics_data:
                self.analytics_data[robot_id] = []

            # update execution analytics
            self.analytics_data[robot_id].append({
                "order_id": base_uuid,
                "full_order_id": order_id,
                "execution_duration": execution_duration,
                "completion_timestamp": completion_timestamp,
                "issued_timestamp": issued_timestamp
            })

            # update wait analytics
            self.record_wait_event(robot_id, base_uuid, 0, is_completed=True)

        # log or print event
        self.logger.info("🛸 [SUCCESS] Robot %s: order %s, full_id %s, duration: %.2f sec (Issued: %s, Completed: %s)",
                        robot_id, base_uuid, order_id, execution_duration, issued_timestamp, completion_timestamp)

    def compute_average_execution_duration(self, show_plot=True):
        """_summary_

        Args:
            show_plot (bool, optional): _description_. Defaults to True.

        Returns:
            _type_: _description_
        """
        avg_times = {}
        with OrderPublisher._registry_lock:
            for robot, tasks in self.analytics_data.items():
                durations = [task["execution_duration"] for task in tasks]
                if durations:
                    avg_times[robot] = np.mean(durations) # Normal average
                    median_time = np.median(durations)    # Median for detecting outliers
                    self.logger.info("Robot %s -> Avg: %.2f sec, Median: %.2f sec", robot, avg_times[robot], median_time)

        if not avg_times:
            self.logger.info("No task completion data available to plot.")
            return avg_times

        if show_plot:
            # plt.xlabel("Robot ID")
            # plt.ylabel("Average Completion Time (seconds)")
            self.terminal_bar_chart(avg_times, xlabel="Robot ID", title="Average Task Completion Time per Robot")

        return avg_times

    def terminal_bar_chart(self, data, xlabel="Category", title="Bar Chart"):
        """
        Draws a simple ASCII bar chart.

        data: a dictionary mapping labels to numeric values.
        xlabel: label for the x-axis (displayed next to each bar)
        title: title of the chart.
        """
        if not data:
            print("No data to plot.")
            return

        # Find maximum value for scaling:
        max_val = max(data.values())
        scale = 50 / max_val if max_val != 0 else 1

        print(f"\n{title}\n")
        for key, val in data.items():
            bar = "#" * int(val * scale)
            print(f"{key}: {bar} ({val:.2f} sec)")


    # --------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------

    def close(self):
        """Ensure the log file is properly closed."""
        for handler in self.logger.handlers[:]:
            handler.close()
            self.logger.removeHandler(handler)

    # --------------------------------------------------------------------------------------------


    def create_database(self, dbname):
        """ create_database """
        # self.db_conn.autocommit = True
        cursor = self.db_conn.cursor()
        cursor.execute(f"SELECT 1 FROM pg_database WHERE datname = '{dbname}';")
        if not cursor.fetchone():
            cursor.execute(sql.SQL("CREATE DATABASE {}").format(sql.Identifier(dbname)))
            self.db_conn.commit()
            self.logger.info("Veritabani '%s' başariyla oluşturuldu.", dbname)
        if self.drop_table:
            self.drop_order_table()
        self.create_order_table()


    def drop_order_table(self):
        """
        Drops the table_order table from the database.
        """
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("DROP TABLE IF EXISTS "+self.table_order)
            self.db_conn.commit()
            self.logger.info("table_order table dropped successfully.")
        except Exception as e:
            self.logger.error("Error dropping table '%s': '%s'", self.table_order, e)


    def create_order_table(self):
        """Creates the 'orders' table if it doesn't exist.
        Args:
            conn: A psycopg2 connection object.
        Returns:
            None
        """
        cursor = self.db_conn.cursor()
        cursor.execute("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = 'orders'
            );
        """)
        if not cursor.fetchone()[0]:
            cursor.execute("""
                CREATE TABLE orders (
                    id SERIAL PRIMARY KEY,
                    header_id INTEGER,
                    timestamp TIMESTAMP,
                    version VARCHAR(50),
                    manufacturer VARCHAR(100),
                    serial_number VARCHAR(100),
                    order_id VARCHAR(100),
                    order_update_id INTEGER,
                    zone_set_id VARCHAR(100),
                    nodes JSONB,
                    edges JSONB
                );
            """)
            self.db_conn.commit()
            self.logger.info("Order table created successfully..")


    def quaternion_to_euler(self, x, y, z, w):
        """ convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw). """
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


    def merge_itineraries(self, checkpoints, waitpoints, agv_itinerary, wait_itinerary):
        """
        Merges checkpoints with their corresponding waitpoints, ensuring that the waitpoint for each node is
        inserted after the node, and updates the agv_itinerary accordingly.

        Requires global scope:
            checkpoints (list): List of nodes the AGV needs to visit (e.g. ['A11', 'A8', 'A3']).
            waitpoints (list): List of waitpoints (e.g. ['W11', 'W8']).
            agv_itinerary (list): Corresponding coordinates for the checkpoints (e.g. [[x1, y1, z1, w1], ...]).
            wait_itinerary (list): Corresponding coordinates for the waitpoints (e.g. [[wx1, wy1, wz1, ww1], ...]).

        Returns:
            merged_nodes (list): Combined list of checkpoints and waitpoints.
            merged_itinerary (list): Combined list of coordinates for checkpoints and waitpoints.
        """
        merged_nodes = []
        merged_itinerary = []

        # Create a mapping from waitpoints to their coordinates
        waitpoint_dict = {wp: wi for wp, wi in zip(waitpoints, wait_itinerary)}

        for idx, node in enumerate(checkpoints):
            # Add the checkpoint and its itinerary
            merged_nodes.append(node)
            merged_itinerary.append(agv_itinerary[idx])

            # Generate the waitpoint corresponding to the current checkpoint
            waitpoint = f'W{node[1:]}'

            # If the generated waitpoint exists in the waitpoints list, insert it
            if waitpoint in waitpoint_dict:
                merged_nodes.append(waitpoint)
                merged_itinerary.append(waitpoint_dict[waitpoint])

        return merged_nodes, merged_itinerary


    # Function to extract the number from a node
    def extract_number(self, node):
        """ extract node number or id after C E or W etc. """
        match = re.search(r'\d+', node)
        return int(match.group()) if match else None


    def create_order(self, checkpoints, waitpoints, agv_itinerary, wait_itinerary, landmark, map_name='map', release_node=None, h_node_eta=None, wait_time=None):
        """
        Builds and inserts an order message based on merged checkpoints and waitpoints.
        requires:
            checkpoints: A list of checkpoints. (global scope)
            waitpoints: A list of waitpoints. (global scope)
            agv_itinerary: The AGV's itinerary for the checkpoints. (global scope)
            wait_itinerary: The AGV's itinerary for the waitpoints. (global scope)
            landmark: List containing payload_kg, task priority, task type, and dock information.
        """
        # Merge checkpoints and waitpoints along with their corresponding itineraries
        merged_nodes, merged_itinerary = self.merge_itineraries(checkpoints, waitpoints, agv_itinerary, wait_itinerary)
        horizon_nodes = []
        horizon_edges = []
        # Determine the node to be released
        release_index = None
        # Find the intended node in the filtered merged list or Find the waitpoint next to the intended node
        if release_node is not None:
            intended_node = release_node[0]
            for i, node in enumerate(merged_nodes):
                if node == intended_node and [n for n in merged_nodes[i:] if not n.startswith('W')] == release_node:
                    release_index = i
                    # if we wanna wait!
                    if wait_time is not None:
                        wait_time = float(wait_time)
                        # Check if the first merged_node has a corresponding waitpoint
                        if merged_nodes[0].startswith('C') and merged_nodes[1].startswith('W'):
                            # assert that the number beside the W is same as the checkpoint number
                            if self.extract_number(merged_nodes[0]) == self.extract_number(merged_nodes[1]):
                                # Swap the checkpoint with the waitpoint
                                self.logger.info("Swapping waitpoint with the first checkpoint.")
                                merged_nodes[0], merged_nodes[1] = merged_nodes[1], merged_nodes[0]
                                merged_itinerary[0], merged_itinerary[1] = merged_itinerary[1], merged_itinerary[0]
                                release_index = 0  # Set release index to zero
                    break
        else:
            # If release_node is None, we don't want any node to be released
            release_index = 0  # Start from the beginning
            wait_time = None  # Override wait_time to ensure no nodes are released

        # base is assumed the current node always granted to the robot.
        base_node = {
            "nodeId": checkpoints[0],
            "released": True,
            "sequenceId": 1, # Odd sequenceId for nodes,
            "nodeDescription": "current base node.",
            "nodePosition": {
                "x": agv_itinerary[0][0],
                "y": agv_itinerary[0][1],
                "theta": self.quaternion_to_euler(0, 0, agv_itinerary[0][2], agv_itinerary[0][3])[-1],
                "mapId": map_name,
                "allowedDeviationXY": 0.5,
                "allowedDeviationTheta": 3.1
            },
            "actions": []
        }

        # Create the order message using the merged data
        for i, (node_id, location) in enumerate(zip(merged_nodes[release_index:], merged_itinerary[release_index:])):
            x, y, z, w = location
            theta = self.quaternion_to_euler(0, 0, z, w)[-1]
            # Determine if the node is a dock node (e.g., C5, C4, etc.)
            is_dock_node = any(node_id == dock for dock in landmark)
            # Build action parameters for dock nodes
            action_parameters = []
            if is_dock_node:
                # Include landmark information for dock nodes
                action_parameters = [{"key": "landmark", "value": landmark}]
            # Create a node description based on task priority and type
            # and Determine the node type based on the prefix of node_id
            if node_id.startswith('C'):
                node_type = "Checkpoint"
            elif node_id.startswith('W'):
                node_type = "Waitpoint"
            else:
                node_type = "Unknown"  # Optional: Handle other cases
            # Create the node description
            node_description = (
                f"Task Priority: {landmark[1] if landmark else 'None'}, "
                f"Task Type: {landmark[2] if len(landmark) > 2 else 'None'}, "
                f"Node Type: {node_type}, "
                f"Wait Time: {wait_time if wait_time is not None else 'None'}, "
                f"Node ETA: {h_node_eta if h_node_eta is not None else 'None'}"
            )

            # Create a node entry
            node = {
                "nodeId": node_id,
                "released": (i < 3 and wait_time is not None) or (i == 0 and wait_time is None and release_node is not None),
                "sequenceId": i * 2 + 1, # Odd sequenceId for nodes,
                "nodeDescription": node_description,
                "nodePosition": {
                    "x": x,
                    "y": y,
                    "theta": theta,
                    "mapId": map_name,
                    "allowedDeviationXY": 0.5,
                    "allowedDeviationTheta": 3.1
                },
                "actions": [
                    {
                        "actionType": "dock",
                        "actionId": str(uuid.uuid4()),
                        "actionDescription": "task priority pick_place dock_location_info.",
                        "blockingType": "NONE",
                        "actionParameters": action_parameters # Inserted landmark parameters
                    }
                ] if is_dock_node else []  # Only add actions for dock nodes
            }

            # append to nodes
            horizon_nodes.append(node)

            # Create an edge entry if it's not the last node
            if i < len(merged_nodes[release_index:]) - 1:
                next_node_id = merged_nodes[release_index + i + 1]
                edge = {
                    "edgeId": f"edge_{node_id}",
                    "released": (i < 3 and wait_time is not None) or (i == 0 and wait_time is None and release_node is not None),
                    "sequenceId": i * 2, # Even sequenceId for edges
                    "startNodeId": node_id,
                    "endNodeId": next_node_id,
                    "actions": []
                }
                horizon_edges.append(edge)

        # ---------------------
        # wait/delay analytics:
        # ---------------------
        # If a wait_time is provided, store it for later association with the order.
        self.current_order_wait = wait_time
        # --------------------

        return base_node, horizon_nodes, horizon_edges


    def build_order_msg(self, f_id, _r_id, header_id, version, manufacturer, b_node, h_nodes, h_edges, order_id, order_update_id):
        """
            Create the order message, insert it into the database, and publish it over MQTT.
            Only record the order issuance if the order_id ends with '_1', which indicates the first (and true)
            issuance instance. Subsequent updates (with _2, _3, etc.) are not recorded.
        """
        order_message = {
            "headerId": header_id,  # Consider using a sequence generator for better tracking
            "timestamp": datetime.datetime.now().isoformat(),
            "version": version,  # Adjust version as needed
            "manufacturer": manufacturer,
            "serialNumber": _r_id,
            "orderId": order_id,
            "orderUpdateId": order_update_id,  # Can be incremented for updates
            "zoneSetId": f_id,  # "",
            "nodes": h_nodes,
            "edges": h_edges
        }

        # Insert new record into instant_actions for robot with the serial number.
        self.insert_order_db(order_message)  # Save into the database

        # Record issuance timestamp for analytics (independent of MQTT publishing).
        # This must happen here so completion timing is always available regardless of MQTT state.
        if order_id.endswith("_0"):
            self.record_order_issuance(_r_id, order_id)

        # MQTT publishing logic
        if self.mqtt_client is not None:
            # For non-waitpoint case, publish custom node and edge structure. only send the first node and edge as base.
            first_node = h_nodes[0]  # Release the base
            # Check if the first node is released
            if not first_node.get('released', False):  # Default to False if 'released' key is not present
                self.logger.info("First node is not released. Skipping publication.")
            else:

                # Prepare the message for MQTT publication.
                # Duplicate the first node with modified sequence IDs 1 first.
                # b_node functions as a patch
                duplicated_node_1 = b_node # first_node.copy()
                # duplicated_node_1["sequenceId"] = 1
                # Duplicate node with seq 3
                duplicated_node_2 = first_node.copy()
                duplicated_node_2["sequenceId"] = 3
                # Create a custom self-looping edge connecting the duplicated nodes
                custom_edge = {
                    "edgeId": f"edge_{duplicated_node_1['nodeId']}", #"edge0",  # You can modify the edge ID as needed
                    "released": True,
                    "sequenceId": 2,  # Sequence ID between the two duplicated nodes
                    "startNodeId": duplicated_node_1["nodeId"], # first_node["nodeId"],
                    "endNodeId": duplicated_node_2["nodeId"],
                    "actions": []
                }

                # If nodes exist and the first node is a waitpoint
                if h_nodes and h_nodes[0]['nodeId'].startswith('W'):

                    # Here, we record the wait start event; later when the wait ends, process wait event will be called.
                    self.process_wait_event(_r_id, order_id, self.current_order_wait)
                    print(f"\033[93m 🛸 Order Wait Time Triggered: {self.current_order_wait} 🛸 \033[0m")

                    # Publish the first three nodes (node, waitpoint, and next checkpoint) and their respective edges
                    if len(h_nodes) >= 3 and len(h_edges) >= 2:
                        # Adjust the nodes and edges to send first three nodes if waitpoint is found
                        # Step 3: Construct the rest of the order (include the next two nodes and edges)
                        order_message["nodes"] = [duplicated_node_1, duplicated_node_2] + h_nodes[1:3]
                        order_message["edges"] = [custom_edge] + h_edges[:2]
                    else:
                        raise ValueError("Not enough nodes or edges to publish the required three nodes and edges.")
                else:
                    # Step 3: Update the order message with the duplicated nodes and custom edge
                    order_message["nodes"] = [duplicated_node_1, duplicated_node_2]  # Two nodes with different sequence IDs
                    order_message["edges"] = [custom_edge]  # One self-looping edge

                # show the order message in a fancy way for debugging
                self.logger.info("Publishing Order Message...")
                # self.logger.info(json.dumps(order_message, indent=4)) # pretty_message = json.dumps(order_message, indent=4, sort_keys=True)

                self.pub_order_mqtt(self.mqtt_client, order_message)


    def insert_order_db(self, order_message):
        """
        Inserts the order message into the database. If the number of entries for the given robot exceeds a maximum,
        it deletes the oldest record.
        """
        try:
            # Insert the order message into the database
            cursor = self.db_conn.cursor()
            cursor.execute("""
                INSERT INTO """+self.table_order+""" (header_id, timestamp, version, manufacturer, serial_number, order_id, order_update_id, zone_set_id, nodes, edges)
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s) RETURNING *;
            """, (order_message["headerId"],
                order_message["timestamp"],
                order_message["version"],
                order_message["manufacturer"],
                order_message["serialNumber"],
                order_message["orderId"],
                order_message["orderUpdateId"],
                order_message["zoneSetId"],
                json.dumps(order_message["nodes"]),
                json.dumps(order_message["edges"])))
            inserted_row = cursor.fetchone()
            self.db_conn.commit()

            # Update cache with inserted row
            if inserted_row:
                self.cache[order_message["serialNumber"]] = inserted_row
            
            # Tracking has been transitioned to fetch_active_and_unassigned_tasks dynamic querying
            # This logic can involve query for existing entries and deleting the oldest one
            # before inserting the new record if the limit is reached. example approach:

            # Check the number of entries for this robot without ORDER BY and LIMIT.
            cursor.execute("SELECT COUNT(*) FROM " + self.table_order + " WHERE serial_number = %s AND manufacturer = %s;",
                        (order_message["serialNumber"], order_message["manufacturer"]))
            entry_count = cursor.fetchone()[0]

            if entry_count >= self.max_entry_per_robot:
                # Delete the oldest entry using a subquery.
                cursor.execute("""
                    DELETE FROM """ + self.table_order + """
                    WHERE id IN (
                        SELECT id FROM """ + self.table_order + """
                        WHERE serial_number = %s AND manufacturer = %s
                        ORDER BY id ASC
                        LIMIT 1
                    );
                """, (order_message["serialNumber"], order_message["manufacturer"]))
                self.db_conn.commit()
        except Exception as er:
            self.logger.error("insert_order_db Database Error: order failed to save data to database: %s", er)
            self.db_conn.rollback()


    def pub_order_mqtt(self, mqtt_client, order_message):
        """ pub_order_mqtt """
        message = json.dumps(order_message)
        topic = f"{self.fleetname}/{self.versions}/{order_message['manufacturer']}/{order_message['serialNumber']}/order"
        mqtt_client.publish(topic, message, qos=0, retain=False)
        # self.logger.info("Order message published.")
        
        # Log colored output as requested for easier debugging
        nodes_info = [n.get("nodeId", "") for n in order_message.get("nodes", [])]
        print(f"\033[92m[OrderPublisher] NEW ORDER PUBLISHED -> Robot: {order_message['serialNumber']} | Nodes: {nodes_info}\033[0m")


    def delete_data(self, f_id, r_id, m_id):
        """
        Delete a row from the database where serial_number, fleet_id, and manufacturer match the given inputs.
        """
        try:
            cursor = self.db_conn.cursor()
            query = """
                DELETE FROM """ + self.table_order + """
                WHERE zone_set_id = %s
                AND serial_number = %s
                AND manufacturer = %s;
            """
            cursor.execute(query, (f_id, r_id, m_id,))
            self.db_conn.commit()  # Make sure to commit the changes to the database
            # Remove from cache if exists
            if r_id in self.cache:
                del self.cache[r_id]
            self.logger.info("Order deleted for fleet %s, robot %s, manufacturer %s.", f_id,r_id,m_id)
        except Exception as er:
            self.logger.error("delete_order Database Error: %s", er)


    def fetch_data(self, f_id, r_id, m_id):
        """ Fetch the most recent order for a specific robot by fleet name, serial number, and manufacturer using cache """
        # Try fetching from cache first
        if r_id in self.cache:
            return self.cache[r_id]
            
        order_ = []
        try:
            cursor = self.db_conn.cursor()
            query = """
                SELECT *
                FROM """ + self.table_order + """
                WHERE zone_set_id = %s
                AND serial_number = %s
                AND manufacturer = %s
                ORDER BY timestamp DESC
                LIMIT 1;
            """
            cursor.execute(query, (f_id, r_id, m_id,))
            order_ = cursor.fetchone()  # Fetch the most recent entry (LIMIT 1)
            # Update cache
            if order_:
                self.cache[r_id] = order_
            # if not order_:
            #     self.logger.info(f"No order data found for fleet {f_id}, robot {r_id}, manufacturer {m_id}")
        except Exception as er:
            self.logger.error("fetch_data order Database Error: %s", er)
        return order_


    def fetch_all_data(self, f_id, m_id):
        """ Fetch the latest order for each robot in the fleet from the database and populate cache. """
        # Try returning filtered cache if populated
        if self.cache:
            filtered_orders = [
                order for order in self.cache.values()
                if (order[7] == f_id and order[4] == m_id)  # zone_set_id and manufacturer
            ]
            if filtered_orders:
                return filtered_orders

        orders = []
        try:
            cursor = self.db_conn.cursor()
            query = f"""
                SELECT DISTINCT ON (serial_number) *
                FROM {self.table_order}
                WHERE zone_set_id = %s
                AND manufacturer = %s
                ORDER BY serial_number, timestamp DESC;
            """
            cursor.execute(query, (f_id, m_id))
            rows = cursor.fetchall()
            for row in rows:
                self.cache[row[5]] = row # serial_number is at index 5
                orders.append(row)
        except Exception as er:
            self.logger.error("fetch_all_data order Database Error: %s", er)
        return orders


    def cleanup_orders(self, f_id, r_id, m_id, cleared):
        """
        Fetch the last order using fetch_data, identify its order ID, and remove all previous orders
        with suffixes (_0, _1, _2, etc.), leaving only the most recent order.
        """
        try:
            # Fetch the latest order
            latest_order = self.fetch_data(f_id, r_id, m_id)
            if not latest_order:
                self.logger.info("Robot %s -> No orders found for cleanup.",r_id)
                return False

            # Assuming `fetch_data` returns a tuple and the `order_id` is at index 6
            latest_order_id = latest_order[6]  # Replace 6 with the correct index for order_id
            # If the order is already marked completed or cancelled, skip.
            if latest_order_id.endswith("_completed") or latest_order_id.endswith("_cancelled"):
                # self.logger.info("Robot %s -> order %s already finalized. Skipping cleanup.", r_id, latest_order_id)
                return False

            # Extract the base UUID part of the order_id (everything before the underscore)
            base_uuid = latest_order_id.rsplit('_', 1)[0]

            # ---------------------
            # analytics:
            # ---------------------
            # Record analytics BEFORE deleting redundant orders so the DB fallback can find the original timestamp.
            if cleared is True:
                nodes = latest_order[9]
                if self._validate_task_signature(nodes, 'transport') or self._validate_task_signature(nodes, 'loop'):
                    self.record_order_completion(r_id, latest_order_id)
            # --------------------

            # Check if there are other orders with the same base UUID
            cursor = self.db_conn.cursor()
            query_check = f"""
                SELECT COUNT(*)
                FROM {self.table_order}
                WHERE serial_number = %s
                AND manufacturer = %s
                AND zone_set_id = %s
                AND order_id LIKE %s
                AND order_id != %s;
            """
            cursor.execute(query_check, (
                r_id,
                m_id,
                f_id,
                f"{base_uuid}_%",
                latest_order_id
            ))
            other_orders_count = cursor.fetchone()[0]

            # Delete redundant orders if they exist
            if other_orders_count > 0:
                print("------------------x-------------------------------x-------------------")
                query_delete = f"""
                    DELETE FROM {self.table_order}
                    WHERE serial_number = %s
                    AND manufacturer = %s
                    AND zone_set_id = %s
                    AND order_id LIKE %s
                    AND order_id != %s;
                """
                cursor.execute(query_delete, (
                    r_id,
                    m_id,
                    f_id,
                    f"{base_uuid}_%",
                    latest_order_id
                ))
                self.db_conn.commit()
                deleted_count = cursor.rowcount
                self.logger.info("Deleted %s redundant orders for robot %s.", deleted_count, r_id)
            

            # Append "_completed" or "_cancelled" to the latest order_id
            # Determine final state suffix
            if cleared is True:
                new_order_id = f"{latest_order_id}_completed"
            elif cleared is False:
                new_order_id = f"{latest_order_id}_cancelled"
            else:
                self.logger.error("Robot %s -> invalid choice: %s. Expected bool for 'completed' or 'cancelled'.", r_id, cleared)
                return False

            # Update the order_id in the database
            query_update = f"""
                UPDATE {self.table_order}
                SET order_id = %s
                WHERE serial_number = %s
                AND manufacturer = %s
                AND zone_set_id = %s
                AND order_id = %s
                RETURNING *;
            """
            cursor.execute(query_update, (
                new_order_id,
                r_id,
                m_id,
                f_id,
                latest_order_id
            ))
            updated_row = cursor.fetchone()
            self.db_conn.commit()

            # Update cache with updated row
            if updated_row:
                self.cache[r_id] = updated_row
                return True

            return False
        except Exception as er:
            self.logger.error("Robot %s -> cleanup_orders Database Error: %s", r_id, er)
            self.db_conn.rollback()
            return False
        finally:
            if 'cursor' in locals() and cursor:
                cursor.close()


    def fetch_active_and_unassigned_tasks(self, f_id, m_id, task_type=None):
        """
        Fetch active tasks (ongoing orders) and unassigned tasks.
        Queries the database for the latest order per robot to determine active status.
        If task_type is provided, filters the returned tasks.
        """
        try:
            cursor = self.db_conn.cursor()
            
            # Fetch latest order for all robots matching f_id and m_id
            query_latest = f"""
                SELECT DISTINCT ON (serial_number) serial_number, order_id, nodes
                FROM {self.table_order}
                WHERE zone_set_id = %s AND manufacturer = %s AND order_id NOT LIKE 'unassigned_%%'
                ORDER BY serial_number, timestamp DESC
            """
            cursor.execute(query_latest, (f_id, m_id))
            latest_orders = cursor.fetchall()
            
            active_robots = []
            for r_id_db, o_id, nodes in latest_orders:
                # Active orders are those that have NOT been completed or cancelled
                if o_id.endswith("_completed") or o_id.endswith("_cancelled"):
                    continue
                
                if not task_type:
                    active_robots.append(r_id_db)
                else:
                    nodes_list = nodes if isinstance(nodes, list) else json.loads(str(nodes))
                    match_found = False
                    for node in nodes_list:
                        if isinstance(node, dict):
                            for action in node.get("actions", []):
                                if action.get("actionType") == "dock":
                                    parameters = action.get("actionParameters", [{}])[0].get("value", [])
                                    if len(parameters) > 2 and parameters[2] == task_type:
                                        active_robots.append(r_id_db)
                                        match_found = True
                                        break
                            if match_found: break

            # Fetch unassigned tasks separately (as they use a different naming convention)
            unassigned_tasks = []
            if not task_type:
                query_unassigned = f"""
                    SELECT order_id FROM {self.table_order}
                    WHERE zone_set_id = %s AND manufacturer = %s AND order_id LIKE 'unassigned_%%'
                """
                cursor.execute(query_unassigned, (f_id, m_id))
                unassigned_tasks = [row[0] for row in cursor.fetchall()]
            else:
                query_unassigned = f"""
                    SELECT order_id, nodes FROM {self.table_order}
                    WHERE zone_set_id = %s AND manufacturer = %s AND order_id LIKE 'unassigned_%%'
                """
                cursor.execute(query_unassigned, (f_id, m_id))
                for u_id, nodes in cursor.fetchall():
                    nodes_list = nodes if isinstance(nodes, list) else json.loads(str(nodes))
                    match_found = False
                    for node in nodes_list:
                        if isinstance(node, dict):
                            for action in node.get("actions", []):
                                if action.get("actionType") == "dock":
                                    parameters = action.get("actionParameters", [{}])[0].get("value", [])
                                    if len(parameters) > 2 and parameters[2] == task_type:
                                        unassigned_tasks.append(u_id)
                                        match_found = True
                                        break
                            if match_found: break

            return active_robots, unassigned_tasks

        except Exception as er:
            self.logger.error("fetch_active_and_unassigned_tasks Database Error: %s", er)
            return [], []


    def _validate_task_signature(self, nodes, expected_type):
        """
        Structural signature check:
        Verifies that the order contains a 'dock' action matching the expected_type
        and adheres to the required parameter structure.
        """
        if not expected_type:
            return True

        if isinstance(nodes, str):
            import json
            try:
                nodes = json.loads(nodes)
            except (json.JSONDecodeError, TypeError):
                return False
        
        if not isinstance(nodes, list):
            return False

        for node in nodes:
            for action in node.get("actions", []):
                if action.get("actionType") == "dock":
                    params = action.get("actionParameters", [{}])[0].get("value", [])
                    
                    # Basic type match (Index 2 MUST be the expected type)
                    if len(params) > 2 and params[2] == expected_type:
                        # Transport/Loop tasks MUST have from/to locations (length >= 5)
                        if expected_type in ('transport', 'loop'):
                            if len(params) >= 5:
                                from_loc = params[3]
                                to_loc = params[4]
                                # EXCLUSION: Ignore "Self-Loop" tasks (Registration/Snapping)
                                if from_loc == to_loc:
                                    self.logger.info("🛸 [REJECTED] Self-loop task: %s == %s", from_loc, to_loc)
                                    return False
                                return True
                            self.logger.info("🛸 [REJECTED] Task %s structural length too short: %s", expected_type, len(params))
                            return False
                        # Move/Charge tasks are registration/maintenance (length >= 3)
                        elif expected_type in ('move', 'charge'):
                            return len(params) >= 3
                        return True
                    else:
                        if len(params) > 2:
                             self.logger.debug("Task type mismatch: found %s, expected %s", params[2], expected_type)
        return False


    def fetch_completed_tasks_count(self, f_id, m_id, cleared, task_type=None):
        """
        Lightweight database query to get the count of completed or cancelled tasks.
        Replaces the heavy fetch_completed_tasks to prevent manager lag.
        """
        try:
            cursor = self.db_conn.cursor()
            suffix = "_completed" if cleared else "_cancelled"
            
            # Simple count query is much faster than fetching all rows
            query = f"SELECT COUNT(*) FROM {self.table_order} WHERE zone_set_id = %s AND manufacturer = %s AND order_id LIKE %s"
            cursor.execute(query, (f_id, m_id, f"%{suffix}"))
            return cursor.fetchone()[0]
        except Exception:
            return 0






if __name__ == "__main__":

   # establish connection to DB
    conn = None
    try:
        conn = psycopg2.connect(host='localhost', dbname='postgres', user='postgres', password='root', port='5432')
        # cur = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)
    except (psycopg2.OperationalError, psycopg2.ProgrammingError) as e:
        print("Failed to connect to PostgreSQL database: %s", e)

    # Initialize OrderPublisher
    order_publisher = OrderPublisher(fleetname="kullar", versions="v1", dbconn=conn, mqttclient=None,
                                     dbname = 'postgres', tname = 'orders')

    # Define the required inputs for the create_order method
    checkpoints = ['C1', 'C2', 'C3']  # Sample checkpoint nodes
    waitpoints = ['W1', 'W2']         # Corresponding waitpoints
    agv_itinerary = [[1, 2, 0, 1], [2, 3, 0, 1], [3, 4, 0, 1]]  # Checkpoint positions (x, y, z, w)
    wait_itinerary = [[1.5, 2.5, 0, 1], [2.5, 3.5, 0, 1]]       # Waitpoint positions (x, y, z, w)
    landmark = [20.0, 'high', 'transport', 'C2', 'C3']  # Sample landmarks

    # Create nodes and edges
    b_node, h_nodes, h_edges = order_publisher.create_order(checkpoints, waitpoints, agv_itinerary, wait_itinerary, landmark)

    # Generate order message
    fleet_id = "kullar"
    robot_serial_number = "AGV-001"
    header_id = 1
    version = "v2"
    manufacturer = "birfen"

    # Build the order message and save to the database
    order_publisher.build_order_msg(
        f_id=fleet_id,
        _r_id=robot_serial_number,
        header_id=header_id,
        version=version,
        manufacturer=manufacturer,
        b_node=b_node,
        h_nodes=h_nodes,
        h_edges=h_edges,
        order_id=str(uuid.uuid4()),
        order_update_id=0
    )

    # Fetch orders for a specific fleet and serial number
    orders = order_publisher.fetch_data(fleet_id,robot_serial_number,manufacturer)

    time.sleep(1.5)
    print(" ------------- ")
    print(f"timestamp: {orders[2]}")
    print(f"serial_number: {orders[5]}")
    print(f"Order ID: {orders[6]}")
    # Since order[9] is already a list, no need for json.loads()
    print(f"Nodes: {orders[9]}")
    print("-- x -- ", len(orders[9]))
    # Similarly, for order[10], if it's also a list
    print(f"Edges: {orders[10]}")
    print(" ------------- ")

    # analytics
    cleared = True
    print("Analytics updated.")

    nodes = orders[9]
    # Fetch landmarks and specific nodes
    # for node in nodes:
    for node in sorted(nodes, key=lambda x: x.get('sequenceId')):
        print(f"\nNode ID: {node['nodeId']}")
        print(f"Node Position: {node['nodePosition']}")
        print(f"Sequence ID: {node['sequenceId']}")
        # Fetching actions
        for action in node['actions']:
            print('actions', action)
            if action['actionType'] == 'landmarks':
                print("  Landmark action found:")
                for param in action['actionParameters']:
                    if param['key'] == 'landmark':
                        print(f"    Landmarks: {param['value']}")

    # edges = orders[10]
    # # for edge in edges:
    # for edge in sorted(edges, key=lambda x: x.get('sequenceId')):
    #     print(f"  Edge ID: {edge.get('edgeId')}")
    #     print(f"    Start Node ID: {edge.get('startNodeId')}")
    #     print(f"    End Node ID: {edge.get('endNodeId')}")

    # # Fetching a specific node by nodeId
    # node_id_to_fetch = "C2"
    # specific_node = next((node for node in nodes if node['nodeId'] == node_id_to_fetch), None)
    # if specific_node:
    #     print(f"\nDetails for node {node_id_to_fetch}:")
    #     print(f"Position: {specific_node['nodePosition']}")
    #     print(f"Actions: {specific_node['actions']}")
    # else:
    #     print(f"Node {node_id_to_fetch} not found.")

    # orders = order_publisher.fetch_all_data(fleet_id,manufacturer)

    num_robots, total_cum_delay = order_publisher.calculate_completed_delays(duration_window=6000)
    print(f"number of robots: {num_robots}: cummulative delays: {total_cum_delay}.")

    avg_times = order_publisher.compute_average_execution_duration(show_plot=True)
    # Print the average execution duration for each robot
    # for robot, avg_time in avg_times.items():
    #    print(f"Robot ID: {robot}, Average Execution Duration: {avg_time:.2f} seconds")

    timestamps, throughput_values = order_publisher.compute_overall_throughput(show_plot=True)
    # Iterate through the timestamps and throughput values
    # for timestamp, throughput in zip(timestamps, throughput_values):
    #    print(f"timestamp [min]: {timestamp}, Throughput: {throughput:.2f}.")

    # Close the database connection after use
    order_publisher.db_conn.close()
