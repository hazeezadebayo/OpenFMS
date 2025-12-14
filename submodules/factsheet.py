import json, os, logging, datetime, time, sys
from jsonschema import validate, ValidationError
from psycopg2 import sql
import psycopg2, psycopg2.extras

class FactsheetSubscriber:
    """ FactsheetSubscriber """
    def __init__(self, fleetname, versions, dbconn, mqttclient=None,
                 drop_table=False, dbname = 'postgres', tname = 'factsheet',
                 output_log=False):

        self.fleetname = fleetname
        self.versions = versions
        self.db_conn = dbconn
        self.mqtt_client = mqttclient
        self.table_factsheet = tname

        self.drop_table = drop_table # True # False

        schema_path = os.path.join(os.path.dirname(__file__), "schemas", "factsheet.schema")
        with open(schema_path, "r", encoding="utf-8") as schema_file:
            self.factsheet_schema = json.load(schema_file)

        self.output_to_screen = output_log
        self.output_to_file = output_log

        # logger; here we use a simple print-based one for clarity.
        self.logger = self._get_logger("FactsheetSubscriber", output_log)

        self.create_database(dbname)

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
                logging.Formatter("[%(levelname)s] [%(asctime)s] %(name)s - %(message)s")
            )
            logger.addHandler(file_handler)

            if self.output_to_screen:
                # Output to terminal (stdout)
                stream_handler = logging.StreamHandler(sys.stdout)
                stream_handler.setFormatter(
                    logging.Formatter("[%(levelname)s] [%(asctime)s] %(name)s - %(message)s")
                )
                logger.addHandler(stream_handler)

            # Show log file path
            # logger.info(f"Logs are written to: {log_file_path}")

        return logger

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
            self.drop_factsheet_table()
        self.create_factsheet_table()

    def drop_factsheet_table(self):
        """
        Drops the table_factsheet table from the database.
        """
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("DROP TABLE IF EXISTS "+self.table_factsheet)
            self.db_conn.commit()
            self.logger.info("table_factsheet table dropped successfully.")
        except Exception as er:
            self.logger.error("Error dropping table '%s': '%s'", self.table_factsheet, er)

    def create_factsheet_table(self):
        """Factsheet tablosunu oluşturma fonksiyonu."""
        cursor = self.db_conn.cursor()
        cursor.execute("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = 'factsheet'
            );
        """)
        if not cursor.fetchone()[0]:
            cursor.execute("""
                CREATE TABLE factsheet (
                    id SERIAL PRIMARY KEY,
                    header_id INTEGER,
                    timestamp TIMESTAMP,
                    version VARCHAR(50),
                    manufacturer VARCHAR(100),
                    serial_number VARCHAR(100),
                    series_name VARCHAR(100),
                    agv_kinematic VARCHAR(50),
                    agv_class VARCHAR(50),
                    max_load_mass INTEGER,
                    localization_types TEXT[],
                    navigation_types TEXT[],
                    speed_min REAL,
                    speed_max REAL,
                    acceleration_max REAL,
                    deceleration_max REAL,
                    height_min REAL,
                    height_max REAL,
                    width REAL,
                    length REAL,
                    -- Protocol Limits
                    msg_len INTEGER,
                    topic_serial_len INTEGER,
                    topic_elem_len INTEGER,
                    id_len INTEGER,
                    id_numerical_only BOOLEAN,
                    enum_len INTEGER,
                    load_id_len INTEGER,
                    order_nodes_max INTEGER,
                    order_edges_max INTEGER,
                    node_actions_max INTEGER,
                    edge_actions_max INTEGER,
                    actions_parameters_max INTEGER,
                    instant_actions_max INTEGER,
                    trajectory_knot_vector_max INTEGER,
                    trajectory_control_points_max INTEGER,
                    state_node_states_max INTEGER,
                    state_edge_states_max INTEGER,
                    state_loads_max INTEGER,
                    state_action_states_max INTEGER,
                    state_errors_max INTEGER,
                    state_information_max INTEGER,
                    error_references_max INTEGER,
                    information_references_max INTEGER,
                    min_order_interval REAL,
                    min_state_interval REAL,
                    default_state_interval REAL,
                    visualization_interval REAL,
                    -- Protocol Features
                    optional_parameters JSONB,
                    agv_actions JSONB,
                    -- AGV Geometry
                    wheel_definitions JSONB,
                    envelopes_2d JSONB,
                    -- Load Specification
                    loadSpecification JSONB,
                    load_positions TEXT[],
                    load_sets JSONB,
                    -- Add vehicleConfig column
                    vehicleConfig JSONB
                );
            """)
            self.db_conn.commit()
            self.logger.info("Factsheet table created successfully..")


    def insert_factsheet_db(self, data):
        """ insert_factsheet_db """
        try:
            # Establish a database connection
            cursor = self.db_conn.cursor()

            # Prepare data with correct formatting
            formatted_data = {
                "header_id": data["headerId"],
                "timestamp": data["timestamp"],
                "version": data["version"],
                "manufacturer": data["manufacturer"],
                "serial_number": data["serialNumber"],
                "series_name": data["typeSpecification"]["seriesName"],
                "agv_kinematic": data["typeSpecification"]["agvKinematic"],
                "agv_class": data["typeSpecification"]["agvClass"],
                "max_load_mass": data["typeSpecification"]["maxLoadMass"],
                "localization_types": data["typeSpecification"]["localizationTypes"],
                "navigation_types": data["typeSpecification"]["navigationTypes"],
                "speed_min": data["physicalParameters"]["speedMin"],
                "speed_max": data["physicalParameters"]["speedMax"],
                "acceleration_max": data["physicalParameters"]["accelerationMax"],
                "deceleration_max": data["physicalParameters"]["decelerationMax"],
                "height_min": data["physicalParameters"]["heightMin"],
                "height_max": data["physicalParameters"]["heightMax"],
                "width": data["physicalParameters"]["width"],
                "length": data["physicalParameters"]["length"],
                # Protocol Limits
                "msg_len": data["protocolLimits"]["maxStringLens"]["msgLen"],
                "topic_serial_len": data["protocolLimits"]["maxStringLens"]["topicSerialLen"],
                "topic_elem_len": data["protocolLimits"]["maxStringLens"]["topicElemLen"],
                "id_len": data["protocolLimits"]["maxStringLens"]["idLen"],
                "id_numerical_only": data["protocolLimits"]["maxStringLens"]["idNumericalOnly"],
                "enum_len": data["protocolLimits"]["maxStringLens"]["enumLen"],
                "load_id_len": data["protocolLimits"]["maxStringLens"]["loadIdLen"],
                "order_nodes_max": data["protocolLimits"]["maxArrayLens"]["order.nodes"],
                "order_edges_max": data["protocolLimits"]["maxArrayLens"]["order.edges"],
                "node_actions_max": data["protocolLimits"]["maxArrayLens"]["node.actions"],
                "edge_actions_max": data["protocolLimits"]["maxArrayLens"]["edge.actions"],
                "actions_parameters_max": data["protocolLimits"]["maxArrayLens"]["actions.actionsParameters"],
                "instant_actions_max": data["protocolLimits"]["maxArrayLens"]["instantActions"],
                "trajectory_knot_vector_max": data["protocolLimits"]["maxArrayLens"]["trajectory.knotVector"],
                "trajectory_control_points_max": data["protocolLimits"]["maxArrayLens"]["trajectory.controlPoints"],
                "state_node_states_max": data["protocolLimits"]["maxArrayLens"]["state.nodeStates"],
                "state_edge_states_max": data["protocolLimits"]["maxArrayLens"]["state.edgeStates"],
                "state_loads_max": data["protocolLimits"]["maxArrayLens"]["state.loads"],
                "state_action_states_max": data["protocolLimits"]["maxArrayLens"]["state.actionStates"],
                "state_errors_max": data["protocolLimits"]["maxArrayLens"]["state.errors"],
                "state_information_max": data["protocolLimits"]["maxArrayLens"]["state.information"],
                "error_references_max": data["protocolLimits"]["maxArrayLens"]["error.errorReferences"],
                "information_references_max": data["protocolLimits"]["maxArrayLens"]["information.infoReferences"],
                "min_order_interval": data["protocolLimits"]["timing"]["minOrderInterval"],
                "min_state_interval": data["protocolLimits"]["timing"]["minStateInterval"],
                "default_state_interval": data["protocolLimits"]["timing"]["defaultStateInterval"],
                "visualization_interval": data["protocolLimits"]["timing"]["visualizationInterval"],
                # Protocol Features
                "optional_parameters": json.dumps(data["protocolFeatures"]["optionalParameters"]),
                "agv_actions": json.dumps(data["protocolFeatures"]["agvActions"]),
                # AGV Geometry
                "wheel_definitions": json.dumps(data["agvGeometry"]["wheelDefinitions"]),
                "envelopes_2d": json.dumps(data["agvGeometry"]["envelopes2d"]),
                # Load Specification
                "loadSpecification": json.dumps(data["loadSpecification"]),
                "load_positions": data["loadSpecification"]["loadPositions"],
                "load_sets": json.dumps(data["loadSpecification"]["loadSets"]),
                # Vehicle Config
                "vehicleConfig": json.dumps(data["vehicleConfig"])
            }

            # Define the SQL INSERT statement
            insert_query = sql.SQL("""
                INSERT INTO factsheet (
                    header_id, timestamp, version, manufacturer, serial_number, series_name,
                    agv_kinematic, agv_class, max_load_mass, localization_types, navigation_types,
                    speed_min, speed_max, acceleration_max, deceleration_max, height_min, height_max,
                    width, length, msg_len, topic_serial_len, topic_elem_len, id_len, id_numerical_only,
                    enum_len, load_id_len, order_nodes_max, order_edges_max, node_actions_max,
                    edge_actions_max, actions_parameters_max, instant_actions_max, trajectory_knot_vector_max,
                    trajectory_control_points_max, state_node_states_max, state_edge_states_max,
                    state_loads_max, state_action_states_max, state_errors_max, state_information_max,
                    error_references_max, information_references_max, min_order_interval,
                    min_state_interval, default_state_interval, visualization_interval,
                    optional_parameters, agv_actions, wheel_definitions, envelopes_2d,
                    loadSpecification, load_positions, load_sets, vehicleConfig
                ) VALUES (
                    %(header_id)s, %(timestamp)s, %(version)s, %(manufacturer)s, %(serial_number)s, %(series_name)s,
                    %(agv_kinematic)s, %(agv_class)s, %(max_load_mass)s, %(localization_types)s, %(navigation_types)s,
                    %(speed_min)s, %(speed_max)s, %(acceleration_max)s, %(deceleration_max)s, %(height_min)s, %(height_max)s,
                    %(width)s, %(length)s, %(msg_len)s, %(topic_serial_len)s, %(topic_elem_len)s, %(id_len)s, %(id_numerical_only)s,
                    %(enum_len)s, %(load_id_len)s, %(order_nodes_max)s, %(order_edges_max)s, %(node_actions_max)s,
                    %(edge_actions_max)s, %(actions_parameters_max)s, %(instant_actions_max)s, %(trajectory_knot_vector_max)s,
                    %(trajectory_control_points_max)s, %(state_node_states_max)s, %(state_edge_states_max)s,
                    %(state_loads_max)s, %(state_action_states_max)s, %(state_errors_max)s, %(state_information_max)s,
                    %(error_references_max)s, %(information_references_max)s, %(min_order_interval)s,
                    %(min_state_interval)s, %(default_state_interval)s, %(visualization_interval)s,
                    %(optional_parameters)s, %(agv_actions)s, %(wheel_definitions)s, %(envelopes_2d)s,
                    %(loadSpecification)s, %(load_positions)s, %(load_sets)s, %(vehicleConfig)s
                )
            """)

            # Execute the SQL query
            cursor.execute(insert_query, formatted_data)

            # Commit the transaction
            self.db_conn.commit()

        except Exception as er:
            self.logger.error("Error inserting data: %s.",er)
            self.db_conn.rollback()

        finally:
            cursor.close()


    def subscribe(self, mqtt_client):
        """ subscribe_to_topics """
        topic = f"{self.fleetname}/{self.versions}/+/+/factsheet"
        mqtt_client.subscribe(topic, qos=0)
        # self.logger.info("Subscribed to topic: %s", topic)


    def validate_message(self, message):
        """ validate_message """
        try:
            validate(instance=message, schema=self.factsheet_schema)
        except ValidationError as er:
            self.logger.info("Factsheet schema validation failed: %s", er.message)
            raise


    def process_message(self, message):
        """ process_message """
        try:
            self.validate_message(message)
            self.insert_factsheet_db(message)
        except ValidationError:
            self.logger.error("factsheet message validation failed. Skipping database save.")


    def fetch_fleets(self):
        """
        Fetch all distinct fleet names from the database.
        """
        fleet_ns = []
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("""
                SELECT DISTINCT versions->>'fleetName' AS fleetName
                FROM """ + str(self.table_factsheet) + """, jsonb_array_elements(vehicleConfig->'versions') AS versions;
            """)
            fleet_ns = [str(row[0]) for row in cursor.fetchall()]
        except Exception as er:
            self.logger.error('factsheet Database error while fetching fleet IDs: %s.' ,er)
        return fleet_ns


    def fetch_serial_numbers(self, f_id):
        """
        Fetch all robot serial numbers associated with the provided fleet_name.
        """
        serial_nums = []
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("""
                SELECT DISTINCT serial_number
                FROM """+str(self.table_factsheet)+""" , jsonb_array_elements(vehicleConfig->'versions') AS versions
                WHERE versions->>'fleetName' = %s;
            """, (f_id,))
            serial_nums = [str(row[0]) for row in cursor.fetchall()]
        except Exception as er:
            self.logger.error('factsheet Database error while fetching robot IDs: %s.',er)
        return serial_nums


    def fetch_data(self, f_id, r_id, m_id):
        """Update feedback data from the database record."""

        serial_number = None
        max_load = None
        width = None
        length = None
        speed_min = None
        speed_max = None
        height_max = None

        try:
            cursor = self.db_conn.cursor()

            # Query the factsheet table, extracting JSON data
            # Query the factsheet table, extracting JSON data
            cursor.execute("""
                SELECT serial_number,
                max_load_mass,
                width,
                length,
                speed_min,
                speed_max,
                height_max
                FROM """ + str(self.table_factsheet) + """,
                    jsonb_array_elements(vehicleConfig->'versions') AS version_elem
                WHERE version_elem->>'fleetName' = %s
                AND serial_number = %s
                AND manufacturer = %s
                ORDER BY timestamp DESC
                LIMIT 1;
            """, (f_id, r_id, m_id))

            result = cursor.fetchone()

            if result:
                # Map the result to respective fields in the correct order
                serial_number = result[0]
                max_load = float(result[1]) if result[1] else None
                width = float(result[2]) if result[2] else None
                length = float(result[3]) if result[3] else None
                speed_min = float(result[4]) if result[4] else None
                speed_max = float(result[5]) if result[5] else None
                height_max = float(result[6]) if result[6] else None

            else:
                self.logger.warning("No factsheet data found for serial_number %s and fleet_name %s.", r_id, f_id)

        except Exception as er:
            self.logger.info("fetch_data factsheet Database Error: %s.",er)

        return serial_number, max_load, width, \
            length, speed_min, speed_max, height_max




# ---------------------------------------------- #
#      MAIN                                      #
# ---------------------------------------------- #

if __name__ == "__main__":

    # Assuming this is part of a class, we'll simulate a scenario using these methods
    # Simulate a message to insert into the database
    manufacturer = "birfen"
    fleet_name = "kullar"
    sample_message = {
        "headerId": 1,
        "timestamp": datetime.datetime.now().isoformat(), # "2024-09-15T12:00:00Z",
        "version": "1.0.0",
        "manufacturer": manufacturer,
        "serialNumber": "AGV-001",
        "typeSpecification": {
            "seriesName": "Series A",
            "agvKinematic": "DifferentialDrive",
            "agvClass": "Class 1",
            "maxLoadMass": 500,
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
            "versions": [{"fleetName": fleet_name}],
            "config2": "value2"
        }
    }

    # establish connection to DB
    conn = None
    try:
        conn = psycopg2.connect(host='localhost', dbname='postgres', user='postgres', password='root', port='5432')
        # cur = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)
    except (psycopg2.OperationalError, psycopg2.ProgrammingError) as e:
        print("Failed to connect to PostgreSQL database: %s", e)

    # Step 1: Process and insert factsheet into the database
    manager = FactsheetSubscriber(fleetname=fleet_name, versions="v1", dbconn=conn, mqttclient=None,
                                     dbname = 'postgres', tname = 'factsheet')
    manager.process_message(sample_message)

    time.sleep(0.5)

    # Step 2: Fetch all fleets
    fleet_names = manager.fetch_fleets()
    print(f"Available Fleets: {fleet_names}")

    # Assuming a fleet name is returned, select one fleet to proceed with
    if fleet_names:
        selected_fleet = fleet_names[0]  # Let's assume it's 'Fleet A'

        # Step 3: Fetch all serial numbers for the selected fleet
        serial_numbers_ = manager.fetch_serial_numbers(selected_fleet)
        print(f"Serial Numbers for {selected_fleet}: {serial_numbers_}")

        # If serial numbers exist, fetch detailed data for the first serial number
        if serial_numbers_:
            selected_serial_number_ = serial_numbers_[0]

            serial_number_, max_load_, width_, \
            length_, speed_min_, speed_max_, height_max_ \
                    = manager.fetch_data(selected_fleet, selected_serial_number_, manufacturer)

            print("serial_number: ", serial_number_)
            print("length: ", length_)
            print("speed_min: ", speed_min_)
            print("max_load_: ", max_load_)

    # Close the database connection after use
    manager.db_conn.close()