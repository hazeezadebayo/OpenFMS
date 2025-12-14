import json, os, logging, datetime, time, sys
from jsonschema import validate, ValidationError
from psycopg2 import sql
import psycopg2, psycopg2.extras

class ConnectionSubscriber:
    """ ConnectionSubscriber """
    def __init__(self, fleetname, versions, dbconn, mqttclient=None,
                 drop_table=False, dbname = 'postgres', tname = 'connection',
                 output_log=False):

        self.fleetname = fleetname
        self.versions = versions
        self.db_conn = dbconn
        self.mqtt_client = mqttclient
        self.table_connection = tname

        self.drop_table = drop_table # True # False

        # Corrected list of column names based on the order table schema
        self.table_col = ['header_id', 'timestamp', 'version', 'manufacturer', 'serial_number', 'connection_state']

        schema_path = os.path.join(os.path.dirname(__file__), 'schemas', 'connection.schema')
        with open(schema_path, 'r', encoding="utf-8") as schema_file:
            self.connection_schema = json.load(schema_file)

        self.output_to_screen = output_log
        self.output_to_file = output_log

        # logger; here we use a simple print-based one for clarity.
        self.logger = self._get_logger("ConnectionSubscriber", output_log)

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
        if not isinstance(dbname, str):
            raise TypeError(f"Database name must be a string, got {type(dbname).__name__}")
        if not dbname.isidentifier():
            raise ValueError(f"Invalid database name: {dbname}")
        cursor = self.db_conn.cursor()
        cursor.execute(f"SELECT 1 FROM pg_database WHERE datname = '{dbname}';")
        if not cursor.fetchone():
            cursor.execute(sql.SQL("CREATE DATABASE {}").format(sql.Identifier(dbname)))
            self.db_conn.commit()
            self.logger.info("Veritabani '%s' başariyla oluşturuldu.", dbname)
        if self.drop_table:
            self.drop_connection_table()
        self.create_connection_table()

    def drop_connection_table(self):
        """
        Drops the table_connection table from the database.
        """
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("DROP TABLE IF EXISTS "+self.table_connection)
            self.db_conn.commit()
            self.logger.info("table_connection table dropped successfully.")
        except Exception as er:
            self.logger.error("Error dropping table '%s': '%s'", self.table_connection, er)

    def create_connection_table(self):
        """connection tablosunu oluşturma fonksiyonu."""
        cursor = self.db_conn.cursor()
        cursor.execute("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = 'connection'
            );
        """)
        if not cursor.fetchone()[0]:
            cursor.execute("""
                CREATE TABLE """+self.table_connection+""" (
                    id SERIAL PRIMARY KEY,
                    header_id INTEGER,
                    timestamp TIMESTAMP,
                    version VARCHAR(50),
                    manufacturer VARCHAR(100),
                    serial_number VARCHAR(100),
                    connection_state VARCHAR(50)
                );
            """)
            self.db_conn.commit()
            self.logger.info("Connection table created successfully..")


    def insert_connection_db(self, message):
        """ insert_factsheet_db """
        cursor = self.db_conn.cursor()
        try:
            query = """
            INSERT INTO """+self.table_connection+""" (header_id, timestamp, version, manufacturer, serial_number, connection_state)
            VALUES (%s, %s, %s, %s, %s, %s)
            """
            cursor.execute(query, (
                message.get("headerId"),
                message.get("timestamp"),
                message.get("version"),
                message.get("manufacturer"),
                message.get("serialNumber"),
                message.get("connectionState")
            ))
            self.db_conn.commit()
        except Exception as er:
            self.logger.error("Failed to write to database: %s", er)
            self.db_conn.rollback()

        finally:
            cursor.close()


    def subscribe(self, mqtt_client):
        """ subscribe_to_topics """
        topic = f"{self.fleetname}/{self.versions}/+/+/connection"
        mqtt_client.subscribe(topic, qos=1)
        # self.logger.info("Subscribed to topic: %s", topic)


    def validate_message(self, message):
        """ validate_message """
        try:
            validate(instance=message, schema=self.connection_schema)
        except ValidationError as er:
            self.logger.info("Connection schema validation failed: %s", er.message)
            raise

    def process_message(self, message):
        """ process_message """
        try:
            self.validate_message(message)
            self.insert_connection_db(message)
        except ValidationError:
            self.logger.error("State message validation failed. Skipping database save.")


    def fetch_data(self, m_id, r_id):
        """
        Fetch the last connection state and timestamp for a specific robot
        based on manufacturer and serial number.
        """
        cursor = self.db_conn.cursor()
        try:
            query = """
            SELECT connection_state, timestamp
            FROM """+self.table_connection+"""
            WHERE manufacturer = %s AND serial_number = %s
            ORDER BY timestamp DESC
            LIMIT 1;
            """
            cursor.execute(query, (m_id, r_id))
            res = cursor.fetchone()

            if res:
                conn_state, time_stamp = res
                return conn_state, time_stamp
            else:
                self.logger.warning("No connection data found for %s", r_id)
                return None, None

        except Exception as er:
            self.logger.error("Error fetching connection data: %s", er)
            return None

        finally:
            cursor.close()


    # Now, we can query the database to fetch orders for a specific fleet and serial number
    def fetch_all_data(self, m_id):
        """ Fetch the latest connection for each robot in the fleet from the database. """
        res_ = []
        try:
            cursor = self.db_conn.cursor()
            query = """
                SELECT DISTINCT ON (serial_number) *
                FROM """ + self.table_connection + """
                WHERE manufacturer = %s
                ORDER BY serial_number, timestamp DESC;
            """
            cursor.execute(query, (m_id,))
            res_ = cursor.fetchall()
            if not res_:
                # self.logger.info(f"No connection data found for manufacturer {m_id}.")
                pass
        except Exception as er:
            self.logger.error("fetch_all_data connection Database Error: %s", er)
        return res_



# ---------------------------------------------- #
#      MAIN                                      #
# ---------------------------------------------- #

if __name__ == "__main__":

    # Establish connection to the PostgreSQL database
    try:
        conn = psycopg2.connect(host='localhost', dbname='postgres', user='postgres', password='root', port='5432')
        # cur = self.conn.cursor(cursor_factory=psycopg2.extras.DictCursor)
        print("Database connection established successfully.")
    except (psycopg2.OperationalError, psycopg2.ProgrammingError) as e:
        print("Failed to connect to PostgreSQL database: %s", e)
        exit(1)

    # Step 1: Initialize the ConnectionSubscriber instance
    manager = ConnectionSubscriber(fleetname="kullar", versions="v1", dbconn=conn, mqttclient=None,
                                     dbname = 'postgres', tname = 'connection')

    # Step 2: Insert a pseudo robot data into the database
    sample_message = {
        "headerId": 1,
        "timestamp": datetime.datetime.now().isoformat(),
        "version": "1.0.0",
        "manufacturer": "birfen",
        "serialNumber": "AGV-001",
        "connectionState": "ONLINE" # ['ONLINE', 'OFFLINE', 'CONNECTIONBROKEN']
    }

    # Step 3: Insert sample message into database
    manager.process_message(sample_message)
    time.sleep(1)  # Pause to ensure the message is inserted

    # Step 4: Fetch the last connection details of the robot by manufacturer and serial number
    connection_state, timestamp = manager.fetch_data("birfen", "AGV-001")
    if connection_state:
        print(f"Last connection state: {connection_state}")
        print(f"Last connection timestamp: {timestamp}")
    else:
        print("No connection data found for this robot.")

    # Close the database connection
    conn.close()
