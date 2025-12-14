import json, os, logging, datetime, sys
from jsonschema import validate, ValidationError
from psycopg2 import sql
import psycopg2, psycopg2.extras

import time, psutil  # For CPU usage monitoring


class StateSubscriber:
    """ StateSubscriber """
    def __init__(self, fleetname, versions, dbconn, mqttclient=None,
                 drop_table=False, dbname = 'postgres', tname = 'state',
                 output_log=False):

        self.fleetname = fleetname
        self.versions = versions
        self.db_conn = dbconn
        self.mqtt_client = mqttclient
        self.table_state = tname

        self.drop_table = drop_table # True # False

        # Corrected list of column names based on the state table schema
        self.table_col = ['header_id', 'timestamp', 'version', 'manufacturer', 'serial_number', 'maps', 'order_id',
                'order_update_id', 'zone_set_id', 'last_node_id', 'last_node_sequence_id', 'driving', 'paused',
                'new_base_request', 'distance_since_last_node', 'operating_mode', 'node_states', 'edge_states',
                'agv_position', 'velocity', 'loads', 'action_states', 'battery_state', 'errors', 'information',
                'safety_state']

        schema_path = os.path.join(os.path.dirname(__file__), 'schemas', 'state.schema')
        with open(schema_path, 'r', encoding="utf-8") as schema_file:
            self.state_schema = json.load(schema_file)

        self.output_to_screen = output_log
        self.output_to_file = output_log

        # logger; here we use a simple print-based one for clarity.
        self.logger = self._get_logger("StateSubscriber", output_log)

        self.create_database(dbname)

        # New: Dictionary to store latency data by robot (serialNumber)
        # Format: { robot_id: [latency1, latency2, ...] }
        self.latency_record_len_min = 5 # Initialize the latency analytics with a fixed recording window length (in minutes).
        self.latency_data = {}  # { robot_id: { "start_time": float, "buckets": [ {"sum": float, "count": int}, ... ] } }


    # --------------------------------------------------------------------------------------------

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

        return logger

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
            self.drop_state_table()
        self.create_state_table()

    def drop_state_table(self):
        """
        Drops the table_state table from the database.
        """
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("DROP TABLE IF EXISTS "+self.table_state)
            self.db_conn.commit()
            self.logger.info("table_state table dropped successfully.")
        except Exception as e:
            self.logger.error("Error dropping table '%s': '%s'", self.table_state, e)

    def create_state_table(self):
        """ create_state_table """
        cursor = self.db_conn.cursor()
        cursor.execute("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = 'state'
            );
        """)
        if not cursor.fetchone()[0]:
            cursor.execute("""
                CREATE TABLE state (
                    id SERIAL PRIMARY KEY,
                    header_id INTEGER,
                    timestamp TIMESTAMP,
                    version VARCHAR(50),
                    manufacturer VARCHAR(100),
                    serial_number VARCHAR(100),
                    maps JSONB,
                    order_id VARCHAR(100),
                    order_update_id INTEGER,
                    zone_set_id VARCHAR(100),
                    last_node_id VARCHAR(100),
                    last_node_sequence_id INTEGER,
                    driving BOOLEAN,
                    paused BOOLEAN,
                    new_base_request BOOLEAN,
                    distance_since_last_node REAL,
                    operating_mode VARCHAR(50),
                    node_states JSONB,
                    edge_states JSONB,
                    agv_position JSONB,
                    velocity JSONB,
                    loads JSONB,
                    action_states JSONB,
                    battery_state JSONB,
                    errors JSONB,
                    information JSONB,
                    safety_state JSONB
                );
            """)
            self.db_conn.commit()
            self.logger.info("table created successfully.")

    def subscribe(self, mqtt_client):
        """ subscribe """
        topic = f"{self.fleetname}/{self.versions}/+/+/state"
        mqtt_client.subscribe(topic, qos=0)
        # self.logger.info("Subscribed to topic: %s", topic)

    def validate_message(self, msg):
        """ validate_message """
        try:
            validate(instance=msg, schema=self.state_schema)
        except ValidationError as er:
            self.logger.error("schema validation failed: %s", er.message)
            raise

    def insert_state_db(self, msg):
        """ insert_state_db """
        try:
            cursor = self.db_conn.cursor()
            insert_query = """
                INSERT INTO state (
                    header_id, timestamp, version, manufacturer, serial_number, maps, order_id, order_update_id, zone_set_id,
                    last_node_id, last_node_sequence_id, driving, paused, new_base_request, distance_since_last_node,
                    operating_mode, node_states, edge_states, agv_position, velocity, loads, action_states, battery_state,
                    errors, information, safety_state
                ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
            """
            values = (
                msg.get("headerId"),
                # Use the timestamp contained in the message if available.
                datetime.datetime.fromisoformat(msg.get("timestamp")) if msg.get("timestamp") else datetime.datetime.now(),
                msg.get("version"),
                msg.get("manufacturer"),
                msg.get("serialNumber"),
                json.dumps(msg.get("maps")),
                msg.get("orderId"),
                msg.get("orderUpdateId"),
                msg.get("zoneSetId"),
                msg.get("lastNodeId"),
                msg.get("lastNodeSequenceId"),
                msg.get("driving"),
                msg.get("paused"),
                msg.get("newBaseRequest"),
                msg.get("distanceSinceLastNode"),
                msg.get("operatingMode"),
                json.dumps(msg.get("nodeStates")),
                json.dumps(msg.get("edgeStates")),
                json.dumps(msg.get("agvPosition")),
                json.dumps(msg.get("velocity")),
                json.dumps(msg.get("loads")),
                json.dumps(msg.get("actionStates")),
                json.dumps(msg.get("batteryState")),
                json.dumps(msg.get("errors")),
                json.dumps(msg.get("information")),
                json.dumps(msg.get("safetyState"))
            )

            # Check the number of placeholders and values
            num_placeholders = insert_query.count('%s')
            num_values = len(values)

            if num_placeholders != num_values:
                self.logger.error("Error: Number of placeholders (%s) doesn't match number of values (%s)",num_placeholders, num_values)
                return

            cursor.execute(insert_query, values)
            self.db_conn.commit()

        except Exception as er:
            self.logger.error("Failed to save state data to database: %s", er, exc_info=True)
            # self.logger.error("Failed to save state data to database: %s", er)
            self.db_conn.rollback()

        finally:
            cursor.close()


    def process_message(self, msg):
        """ process_message with latency, CPU usage, and per-robot aggregation """
        try:
            # validate message schema
            self.validate_message(msg)
            # track message latencies
            self.record_msg_latency(msg)
            # Insert the state message into the database
            self.insert_state_db(msg)
        except ValidationError:
            self.logger.error("State message validation failed. Skipping database save. %s", msg)


    # ===================== Latency Analytics =====================
    def record_msg_latency(self, msg):
        """
        Record the latency from an incoming state message.
        Instead of storing every individual latency, we accumulate them into one-minute buckets.
        """
        receive_time = time.time()
        msg_timestamp_str = msg.get("timestamp")
        try:
            # Convert the message timestamp from ISO format to epoch seconds.
            msg_timestamp = datetime.datetime.fromisoformat(msg_timestamp_str).timestamp()
        except Exception as e:
            self.logger.error("Failed to parse message timestamp: %s", e)
            msg_timestamp = receive_time  # Fall back to current time

        msg_latency = receive_time - msg_timestamp
        robot_id = msg.get("serialNumber", "unknown")

        # Initialize record for the robot if not already present.
        if robot_id not in self.latency_data:
            self.latency_data[robot_id] = {
                "start_time": receive_time,
                "buckets": [{"sum": 0.0, "count": 0} for _ in range(self.latency_record_len_min)]
            }

        data = self.latency_data[robot_id]
        # Calculate how many seconds have elapsed since we started recording.
        elapsed = receive_time - data["start_time"]
        # Determine the bucket index (each bucket represents one minute).
        bucket_index = int(elapsed // 60)

        # If the elapsed time exceeds our fixed window, shift the buckets:
        if bucket_index >= self.latency_record_len_min:
            # Compute how many full minutes (buckets) to shift.
            shift_count = bucket_index - self.latency_record_len_min + 1
            # Remove the oldest shift_count buckets and append that many new empty buckets.
            data["buckets"] = data["buckets"][shift_count:] + [{"sum": 0.0, "count": 0} for _ in range(shift_count)]
            # Shift the start_time forward by shift_count minutes.
            data["start_time"] += shift_count * 60
            # Recalculate elapsed time and bucket index after shifting.
            elapsed = receive_time - data["start_time"]
            bucket_index = int(elapsed // 60)

        # Add the current message's latency to the appropriate bucket.
        data["buckets"][bucket_index]["sum"] += msg_latency
        data["buckets"][bucket_index]["count"] += 1

        self.logger.info("Recorded latency for robot %s in bucket %d: %.2f sec", robot_id, bucket_index, msg_latency)

    def compute_robot_avg_latency(self, show_plot=True):
        """
        For each robot, we maintain:
          - "start_time": the time when we started recording latencies.
          - "buckets": a fixed-length list (of length = latency_record_len_min) where each bucket
                       stores a dict with:
                         - "sum": cumulative latency for that minute,
                         - "count": number of messages in that minute.

        Compute the average latency for each robot over the fixed window.
        Returns a dictionary: { robot_id: average_latency_in_seconds }.
        """
        avg_latency = {}
        for robot, data in self.latency_data.items():
            total_sum = 0.0
            total_count = 0
            for bucket in data["buckets"]:
                total_sum += bucket["sum"]
                total_count += bucket["count"]
            avg_latency[robot] = total_sum / total_count if total_count > 0 else 0

        if show_plot:
            # plt.xlabel("Robot ID")
            # plt.ylabel("Average Latency (sec)")
            self.terminal_bar_chart(avg_latency, xlabel="Robot ID",
                                    title=f"Average State Message Latency per Robot (Last {self.latency_record_len_min} min)")
        return avg_latency


    def compute_system_avg_latency(self, show_plot=True):
        """
        Compute the overall average latency across all robots.

        This function takes the per-robot average latencies computed from the fixed window and then
        aggregates them by taking the mean. It then plots a bar chart where the x-axis is the number
        of robots (i.e. the count of distinct robot IDs) and the y-axis is the overall average latency.

        Returns a tuple: (number_of_robots, overall_average_latency)
        """
        # Compute per-robot averages using your existing method.
        avg_latency = self.compute_robot_avg_latency(show_plot=False)
        num_robots = len(avg_latency)
        if num_robots > 0:
            overall_avg = sum(avg_latency.values()) / num_robots
        else:
            overall_avg = 0

        if show_plot:
            overall_data = {num_robots: overall_avg}
            # Use terminal_bar_chart to print a bar chart.
            self.terminal_bar_chart(
                data=overall_data,
                xlabel="Number of Robots",
                title="Overall Average Latency vs. Robot Count")

        return num_robots, overall_avg


    def terminal_bar_chart(self, data, xlabel="Category", title="Bar Chart"):
        """
        Draw a simple ASCII bar chart in the terminal.
        data: dictionary mapping labels (e.g., robot IDs) to numeric values.
        """
        if not data:
            print("No data to plot.")
            return
        max_val = max(data.values())
        scale = 50 / max_val if max_val != 0 else 1
        print(f"\n{title}\n")
        for key, val in data.items():
            bar = "#" * int(val * scale)
            print(f"{key}: {bar} ({val:.2f} sec)")
    # -------------------------------------------------------------------


    def fetch_data(self, f_id, r_id, m_id):
        """Update feedback data from the database record."""
        serial_number = None
        maps = None
        order_id = None
        last_node_id = None
        driving = None
        paused = None
        node_states = None
        agv_position = None
        velocity = None
        battery_state = None
        errors = None
        information = None

        try:
            cursor = self.db_conn.cursor()
            query = """
                SELECT *
                FROM """ + self.table_state + """
                WHERE zone_set_id = %s
                AND serial_number = %s
                AND manufacturer = %s
                ORDER BY timestamp DESC
                LIMIT 1;
            """
            cursor.execute(query, (f_id, r_id, m_id))
            result = cursor.fetchone()
            if result:
                # Extract values from result tuple
                serial_number = result[5]
                maps = json.loads(result[6]) if isinstance(result[6], str) else result[6] # safe_json_loads(result[6])
                order_id = result[7] if isinstance(result[7], (int, str)) else None
                last_node_id = result[10] if isinstance(result[10], (str)) else None
                driving = result[12] if isinstance(result[12], bool) else None
                paused = result[13] if isinstance(result[13], bool) else None
                node_states =  result[17] # json.loads(result[17]) # if isinstance(result[17], str) else result[17]
                agv_position = result[19]
                velocity = result[20]
                battery_state = result[23]
                errors = result[24]
                information = result[25]
            else:
                self.logger.warning("No state data found for serial_number %s and fleet_name %s.", r_id, f_id)
        except Exception as er:
            self.logger.error("fetch_data state Database Error: %s", er)
        return serial_number, maps, order_id, last_node_id, driving, paused, node_states, agv_position, velocity, battery_state, errors, information


    def fetch_all_data(self, f_id, m_id):
        """Fetch all rows and columns of data based on fleet_id from the database."""
        records = None
        try:
            cursor = self.db_conn.cursor()
            query = """
                SELECT DISTINCT ON (serial_number) *
                FROM """ + self.table_state + """
                WHERE zone_set_id = %s
                AND manufacturer = %s
                ORDER BY serial_number, timestamp DESC;
            """
            cursor.execute(query, (f_id, m_id))
            records = cursor.fetchall()
            if not records:
                self.logger.warning("No state data found for fleet_name %s", f_id)
        except Exception as er:
            self.logger.error("fetch_all_data state Database Error: %s", er)

        return records



if __name__ == "__main__":

    # MQTT Client placeholder (if needed for testing)
    # mqtt_client = None  # Initialize with an actual MQTT client if needed

   # establish connection to DB
    conn = None
    try: # Sample database connection setup (assuming the database is already created)
        conn = psycopg2.connect(host='localhost', dbname='postgres', user='postgres', password='root', port='5432')
        # cur = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)
    except (psycopg2.OperationalError, psycopg2.ProgrammingError) as e:
        print("Failed to connect to PostgreSQL database: %s", e)

    # Initialize OrdStateSubscribererPublisher
    state_subscriber = StateSubscriber(fleetname="Fleet1", versions="v2", dbconn=conn, mqttclient=None,
                                     dbname = 'postgres', tname = 'orders')

    # Initialize the StateSubscriber: Generate order message
    fleetname = "kullar"
    robot_serial_number = "AGV-001" # "SN12345678"
    versions = "v1"
    manufacturer = "birfen"
    state_subscriber = StateSubscriber(fleetname, versions, conn)

    # Sample message structure that will be validated, inserted into the database, and fetched
    message = {
        "headerId": 1,
        "version": "1.0.0",
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
        "orderId": "order_1234",
        "orderUpdateId": 56,
        "zoneSetId": fleetname,
        "lastNodeId": "node_3",
        "lastNodeSequenceId": 3,
        "driving": True,
        "paused": False,
        "newBaseRequest": False,
        "distanceSinceLastNode": 20.0,
        "operatingMode": "AUTOMATIC",
        "nodeStates": [
            {
                "nodeId": "C1", # node_1
                "sequenceId": 1,
                "nodeDescription": "Start Node",
                "nodePosition": {
                    "x": 0.0,
                    "y": 0.0,
                    "theta": 0.0,
                    "mapId": "map1"
                },
                "released": True
            },
            {
                "nodeId": "C2", # node_2
                "sequenceId": 2,
                "nodeDescription": "Intermediate Node",
                "nodePosition": {
                    "x": 5.0,
                    "y": 5.0,
                    "theta": 1.57,
                    "mapId": "map1"
                },
                "released": False
            },
            {
                "nodeId": "C3", # node_3
                "sequenceId": 3,
                "nodeDescription": "Another Intermediate Node",
                "nodePosition": {
                    "x": 10.0,
                    "y": 10.0,
                    "theta": 3.14,
                    "mapId": "map1"
                },
                "released": True
            },
            {
                "nodeId": "C4", # node_4
                "sequenceId": 4,
                "nodeDescription": "Penultimate Node",
                "nodePosition": {
                    "x": 15.0,
                    "y": 15.0,
                    "theta": 4.71,
                    "mapId": "map1"
                },
                "released": True
            },
            {
                "nodeId": "C5", # node_5
                "sequenceId": 5,
                "nodeDescription": "Final Node",
                "nodePosition": {
                    "x": 20.0,
                    "y": 20.0,
                    "theta": 6.28,
                    "mapId": "map1"
                },
                "released": True
            }
        ],
        "edgeStates": [
            {
                "edgeId": "edge_1",
                "sequenceId": 1,
                "edgeDescription": "Edge between C1 and C2",
                "released": True,
                "trajectory": {
                    "degree": 3,
                    "knotVector": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    "controlPoints": [
                        {"x": 0.0, "y": 0.0, "weight": 1.0},
                        {"x": 2.5, "y": 2.5, "weight": 1.0},
                        {"x": 5.0, "y": 5.0, "weight": 1.0}
                    ]
                }
            },
            {
                "edgeId": "edge_2",
                "sequenceId": 2,
                "edgeDescription": "Edge between C2 and C3",
                "released": True,
                "trajectory": {
                    "degree": 3,
                    "knotVector": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    "controlPoints": [
                        {"x": 5.0, "y": 5.0, "weight": 1.0},
                        {"x": 7.5, "y": 7.5, "weight": 1.0},
                        {"x": 10.0, "y": 10.0, "weight": 1.0}
                    ]
                }
            },
            {
                "edgeId": "edge_3",
                "sequenceId": 3,
                "edgeDescription": "Edge between C3 and C4",
                "released": False,
                "trajectory": {
                    "degree": 3,
                    "knotVector": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    "controlPoints": [
                        {"x": 10.0, "y": 10.0, "weight": 1.0},
                        {"x": 12.5, "y": 12.5, "weight": 1.0},
                        {"x": 15.0, "y": 15.0, "weight": 1.0}
                    ]
                }
            },
            {
                "edgeId": "edge_4",
                "sequenceId": 4,
                "edgeDescription": "Edge between C4 and C5",
                "released": True,
                "trajectory": {
                    "degree": 3,
                    "knotVector": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    "controlPoints": [
                        {"x": 15.0, "y": 15.0, "weight": 1.0},
                        {"x": 17.5, "y": 17.5, "weight": 1.0},
                        {"x": 20.0, "y": 20.0, "weight": 1.0}
                    ]
                }
            }
        ],
        "agvPosition": {
            "x": 18.0,
            "y": 18.0,
            "theta": 1.57,
            "mapId": "map1",
            "positionInitialized": True
        },
        "velocity": {
            "vx": 2.0,
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
                "actionId": "action_123",
                "actionType": "move",
                "actionDescription": "Move to position A",
                "actionStatus": "RUNNING",
                "resultDescription": "Moving towards position A"
            },
            {
                "actionId": "action_124",
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
                "infoDescription": "The robot's wheel seperation and radius. same as in factsheet."
            }
        ],
        "safetyState": {
            "eStop": "NONE",
            "fieldViolation": True
        }
    }

    # Simulate receiving and processing the message
    try:
        # This method will validate and then insert the message into the database
        state_subscriber.process_message(message)
        print("State message processed and inserted successfully.")
    except Exception as e:
        print(f"Error processing state message: {e}")

   # Fetching the data back using fleetname and serial number
    try:
        fetched_data = state_subscriber.fetch_data(fleetname, robot_serial_number, manufacturer)
        # Display the fetched data
        print("Fetched State Data:")
        for key, value in zip(
            ['serial_number', 'maps', 'order_id', 'last_node_id', 'driving', 'paused', 'node_states', 'agv_position', 'velocity', 'battery_state', 'errors', 'information'],
            fetched_data):
            # print(f"{key}: {value}")
            pass
    except Exception as e:
        print(f"Error fetching state data: {e}")

    rec_ = state_subscriber.fetch_all_data(fleetname, manufacturer)
    # print("\n item: ", rec_)
    node_states_index = state_subscriber.table_col.index('node_states')
    for state_rec in rec_:
        node_states = state_rec[node_states_index+1]
        if node_states:
            print("\n node_states: ", node_states)
            # for node in node_states:
            waypoints = [(node['nodeId'], node['nodePosition']['x'], node['nodePosition']['y'], \
                node['nodePosition']['theta'], node['released'], node.get('actions', [])) \
                    for node in sorted(node_states, key=lambda x: x['sequenceId'])]
            print("first node id: ", waypoints[0][0])
            for point in waypoints:
                node_id = point[0]
                print("node_id: ", node_id)

    # Display the latency analytics table.

    avg_per_robot_latencies = state_subscriber.compute_robot_avg_latency(show_plot=True)
    num_robots, overall_avg = state_subscriber.compute_system_avg_latency(show_plot=True)
    print("Average latencies per robot:", avg_per_robot_latencies)










# Modified query to properly handle JSONB fields
# query = """
#     SELECT *
#     FROM """ + self.table_state + """
#     WHERE serial_number = %s
#     AND EXISTS (
#         SELECT 1
#         FROM jsonb_array_elements(information) AS info_element
#         WHERE info_element->>'infoType' = 'fleet_name'
#             AND EXISTS (
#                 SELECT 1
#                 FROM jsonb_array_elements(info_element->'infoReferences') AS ref_element
#                 WHERE ref_element->>'referenceKey' = 'fleet_name'
#                 AND ref_element->>'referenceValue' = %s
#             )
#     );
# """


# # General query to fetch all rows based on fleet_id (from the information JSONB field)
# query = """
#     SELECT *
#     FROM """ + self.table_state + """
#     WHERE EXISTS (
#         SELECT 1
#         FROM jsonb_array_elements(information) AS info_element
#         WHERE info_element->>'infoType' = 'fleet_name'
#             AND EXISTS (
#                 SELECT 1
#                 FROM jsonb_array_elements(info_element->'infoReferences') AS ref_element
#                 WHERE ref_element->>'referenceKey' = 'fleet_name'
#                 AND ref_element->>'referenceValue' = %s
#             )
#     );
# """