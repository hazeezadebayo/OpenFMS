import json, os, logging, datetime, time, uuid, sys
from psycopg2 import sql
import psycopg2, psycopg2.extras

class InstantActionsPublisher:
    """  InstantActionsPublisher  """
    def __init__(self, fleetname, versions, dbconn, mqttclient=None,
                 drop_table=False, dbname = 'postgres', tname = 'instant_actions',
                 output_log=False):

        self.fleetname = fleetname
        self.versions = versions
        self.db_conn = dbconn
        self.mqtt_client = mqttclient
        self.table_instant_actions = tname

        self.max_entry_per_robot = 20
        self.drop_table = drop_table # True # False

        self.output_to_screen = output_log
        self.output_to_file = output_log

        # logger; here we use a simple print-based one for clarity.
        self.logger = self._get_logger("InstantActionsPublisher", output_log)

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

    def _load_last_header_id_from_db(self):
        """Veritabanindan en son kullanilan headerId'yi yükler."""
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("SELECT MAX(header_id) FROM instant_actions")
            last_header_id = cursor.fetchone()[0]
            return last_header_id if last_header_id is not None else 0
        except Exception as e:
            self.logger.error("Failed to load last headerId from database: %s.",e)
            return 0

    def create_database(self, dbname):
        """ create database """
        # self.db_conn.autocommit = True
        cursor = self.db_conn.cursor()
        cursor.execute(f"SELECT 1 FROM pg_database WHERE datname = '{dbname}';")
        if not cursor.fetchone():
            cursor.execute(sql.SQL("CREATE DATABASE {}").format(sql.Identifier(dbname)))
            self.db_conn.commit()
            self.logger.info("Veritabani '%s' başariyla oluşturuldu.", dbname)
        if self.drop_table:
            self.drop_instant_actions_table()
        self.create_instant_actions_table()

    def drop_instant_actions_table(self):
        """
        Drops the table_instant_actions table from the database.
        """
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("DROP TABLE IF EXISTS "+self.table_instant_actions)
            self.db_conn.commit()
            self.logger.info("table dropped successfully.")
        except Exception as e:
            self.logger.error("Error dropping table: %s.",e)

    def create_instant_actions_table(self):
        """ create instant action table """
        cursor = self.db_conn.cursor()
        cursor.execute("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = 'instant_actions'
            );
        """)
        if not cursor.fetchone()[0]:
            cursor.execute("""
                CREATE TABLE instant_actions (
                    id SERIAL PRIMARY KEY,
                    header_id INTEGER,
                    timestamp TIMESTAMP,
                    version VARCHAR(50),
                    manufacturer VARCHAR(100),
                    serial_number VARCHAR(100),
                    actions JSONB
                );
            """)
            self.db_conn.commit()
            self.logger.info("table created successfully..")

    def create_action(self, action_type, parameters):
        """ Create action objects based on the schema """
        action = {
            "actionType": action_type,
            "actionId": str(uuid.uuid4()),  # Generate unique UUID
            "blockingType": "NONE",  # Options: [NONE, SOFT, HARD]
            "actionParameters": [{k : v} for k, v in parameters.items()]
        }
        return action

    def build_instant_action_msg(self, f_id, _r_id, header_id, version, manufacturer, action_list):
        """ Create and write instant action object """
        instant_action = {
            "headerId": header_id,
            "timestamp": datetime.datetime.now().isoformat(),
            "version": version,  # Adjust version as needed
            "manufacturer": manufacturer,  # Replace with actual manufacturer
            "serialNumber": _r_id, # serial_number
            "instantActions": action_list
        }
        # Insert new record into instant_actions for robot with the serial number.
        self.insert_instant_actions_db(instant_action)  # Veritabanına kaydet
        if self.mqtt_client is not None:
            self.pub_instant_actions_mqtt(self.mqtt_client, instant_action)

    def insert_instant_actions_db(self, instant_action):
        """ insert instant action into db """
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("""
                INSERT INTO """+self.table_instant_actions+""" (header_id, timestamp, version, manufacturer, serial_number, actions)
                VALUES (%s, %s, %s, %s, %s, %s);
            """, (instant_action["headerId"],
                instant_action["timestamp"],
                instant_action["version"],
                instant_action["manufacturer"],
                instant_action["serialNumber"],
                json.dumps(instant_action["instantActions"])))
            self.db_conn.commit()
            # This logic can involve query for existing entries and deleting the oldest one
            # before inserting the new record if the limit is reached. example approach:

            # Check the number of entries for this robot without ORDER BY and LIMIT.
            cursor.execute("SELECT COUNT(*) FROM " + self.table_instant_actions + " WHERE serial_number = %s AND manufacturer = %s;",
                        (instant_action["serialNumber"], instant_action["manufacturer"]))
            entry_count = cursor.fetchone()[0]

            if entry_count >= self.max_entry_per_robot:
                # Delete the oldest entry using a subquery.
                cursor.execute("""
                    DELETE FROM """ + self.table_instant_actions + """
                    WHERE id IN (
                        SELECT id FROM """ + self.table_instant_actions + """
                        WHERE serial_number = %s AND manufacturer = %s
                        ORDER BY id ASC
                        LIMIT 1
                    );
                """, (instant_action["serialNumber"], instant_action["manufacturer"]))
                self.db_conn.commit()

        except Exception as e:
            self.logger.error("Database Error: failed to save data to database: %s.",e)
            self.db_conn.rollback()


    def pub_instant_actions_mqtt(self, mqtt_client, instant_action):
        """ pub_instant_actions_mqtt """
        message = json.dumps(instant_action)
        topic = f"{self.fleetname}/{self.versions}/{instant_action['manufacturer']}/{instant_action['serialNumber']}/instantActions"
        mqtt_client.publish(topic, message, qos=0, retain=False)
        # self.logger.info("InstantActions message published.")


    def fetch_data(self, f_id, r_id, m_id, action_type):
        """ Fetch the latest action of a given type for a specific robot """
        try:
            cursor = self.db_conn.cursor()
            cursor.execute(f"""
                SELECT actions, timestamp
                FROM {self.table_instant_actions}
                WHERE serial_number = %s AND manufacturer = %s
                ORDER BY timestamp DESC;
            """, (r_id, m_id))
            results = cursor.fetchall()
            if results:
                for res in results:
                    action_data, timestamp = res
                    # Extract only the action with the specified action_type
                    for action in action_data:
                        if action["actionType"] == action_type:
                            return action, timestamp
            else:
                self.logger.info("No instant_actions data found for fleetName %s, manufacturer %s.", f_id, m_id)
        except Exception as e:
            self.logger.error("Database Error: Failed to fetch action: %s.",e)
        return None, None


    # Now, we can query the database to fetch orders for a specific fleet and serial number
    def fetch_all_data(self, f_id, m_id):
        """ Fetch the latest order for each robot in the fleet from the database. """
        instant_actions = []
        try:
            cursor = self.db_conn.cursor()
            query = """
                SELECT DISTINCT ON (serial_number) *
                FROM """ + self.table_instant_actions + """
                WHERE manufacturer = %s
                ORDER BY timestamp DESC;
            """
            cursor.execute(query, (m_id,))
            instant_actions = cursor.fetchall()
            if not instant_actions:
                self.logger.info("No instant_actions data found for fleetName %s, manufacturer %s.", f_id, m_id)
        except Exception as er:
            self.logger.error("fetch_all_data order Database Error: %s", er)
        return instant_actions


if __name__ == "__main__":

    # MQTT Client placeholder (if needed for testing)
    mqtt_client = None  # Initialize with an actual MQTT client if needed

   # establish connection to DB
    conn = None
    try:
        conn = psycopg2.connect(host='localhost', dbname='postgres', user='postgres', password='root', port='5432')
        # cur = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)
    except (psycopg2.OperationalError, psycopg2.ProgrammingError) as e:
        print("Failed to connect to PostgreSQL database: %s", e)

    # Initialize instantactions_publisher
    instantactions_publisher = InstantActionsPublisher(fleetname="Fleet1", versions="v2", dbconn=conn, mqttclient=None,
                                     dbname = 'postgres', tname = 'instant_actions')

    # Generate order message
    fleet_id = "kullar"
    robot_serial_number = "AGV-001"
    header_id = 1
    version = "v1"
    manufacturer = "birfen"

    # Example usage:
    initPositionparameters = {
        "x": "-2.0",
        "y": "-0.5",
        "theta": "0.0",
        "mapId": "map"
    }

    initFlagparameters = {
        "state": "green"
    }

    cancelOrderparameters = {"orderID":str(uuid.uuid4())+"_9",
                             "fleetName":fleet_id}

    cancelOrderparameters2 = {"orderID":str(uuid.uuid4())+"_0",
                             "fleetName":fleet_id}


    traffic_signal_action = instantactions_publisher.create_action("traffic_signal", initFlagparameters)
    action_list = [traffic_signal_action]
    instantactions_publisher.build_instant_action_msg(fleet_id, robot_serial_number, header_id, version, manufacturer, action_list)


    cancel_order_action = instantactions_publisher.create_action("cancelOrder", cancelOrderparameters2)
    action_list = [cancel_order_action]
    # Build the order message and save to the database
    instantactions_publisher.build_instant_action_msg(fleet_id, robot_serial_number, header_id, version, manufacturer, action_list)

    cancel_order_action = instantactions_publisher.create_action("cancelOrder", cancelOrderparameters)
    action_list = [cancel_order_action]
    # Build the order message and save to the database
    instantactions_publisher.build_instant_action_msg(fleet_id, robot_serial_number, header_id, version, manufacturer, action_list)

    init_position_action = instantactions_publisher.create_action("initPosition", initPositionparameters)
    action_list = [init_position_action]
    instantactions_publisher.build_instant_action_msg(fleet_id, robot_serial_number, header_id, version, manufacturer, action_list)

    time.sleep(0.5)


    last_cancel_action_, timestamp_ = instantactions_publisher.fetch_data(fleet_id,
                                                                        robot_serial_number,
                                                                        manufacturer,
                                                                        "cancelOrder")
    if last_cancel_action_:
        print("Last cancelOrder action:", last_cancel_action_['actionParameters'][0]['orderID'])
        print("Last cancelOrder action:", last_cancel_action_['actionParameters'][1]['fleetName'])
        print("Timestamp:", timestamp_)
    else:
        print("No cancelOrder actions found for", robot_serial_number)

    # Close the database connection after use
    instantactions_publisher.db_conn.close()
