import json, os, logging, datetime, random, yaml, math, re, sys
from jsonschema import validate, ValidationError
from psycopg2 import sql
import psycopg2, psycopg2.extras
from pathlib import Path

class VisualizationSubscriber:
    """ Visualization handler """
    def __init__(self, fleetname, versions, dbconn, mqttclient=None,
                 task_dict=None, dbname = 'postgres', tname = 'maps',
                 output_log=True): # output_log=False

        self.fleetname = fleetname
        self.versions = versions
        self.db_conn = dbconn
        self.mqtt_client = mqttclient
        self.drop_table = True # False
        self.table_maps = tname

        # initialize the task network dictionary
        self.task_dictionary = task_dict if task_dict else {}

        self.log_level = 'debug' # ['critical', 'info', 'warn', 'error', 'debug']
        self.robot_positions = {}

        schema_path = os.path.join(os.path.dirname(__file__), 'schemas', 'visualization.schema')
        with open(schema_path, 'r', encoding="utf-8") as schema_file:
            self.visualization_schema = json.load(schema_file)

        self.output_to_screen = output_log
        self.output_to_file = output_log

        # logger; here we use a simple print-based one for clarity.
        self.logger = self._get_logger("VisualizationSubscriber", output_log)

        self.create_database(dbname)


    # --------------------------------------------------------------------------------------------

    def _get_logger(self, logger_name, output_log):

        logger = logging.getLogger(logger_name)
        logger.setLevel(logging.INFO)

        # Ensure handlers are not duplicated
        if not logger.hasHandlers() and output_log:
            # This:
            # Get the directory of the current script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            # Go one folder backwards
            parent_dir = os.path.dirname(script_dir)
            # Create "logs" folder there
            logs_dir = os.path.join(parent_dir, "logs")
            os.makedirs(logs_dir, exist_ok=True)
            # Build log file path inside "logs" folder
            log_file_path = os.path.join(logs_dir, "FmLogHandler.log")
            # Or that:
            # Set up logging to log file
            # log_file_path = os.path.abspath("FmLogHandler.log")

            # Append if file exists, otherwise create new
            file_mode = 'a' if os.path.exists(log_file_path) else 'w'

            # File handler
            file_handler = logging.FileHandler(log_file_path, mode=file_mode)
            file_handler.setFormatter(
                logging.Formatter("[%(levelname)s] [%(asctime)s] %(name)s: %(message)s")
            )
            logger.addHandler(file_handler)

            # Optional terminal output
            if self.output_to_screen:
                stream_handler = logging.StreamHandler(sys.stdout)
                stream_handler.setFormatter(
                    logging.Formatter("[%(levelname)s] [%(asctime)s] %(name)s: %(message)s")
                )
                logger.addHandler(stream_handler)

            # Show log file path (optional)
            # logger.info(f"Logs are written to: {log_file_path}")

        return logger

    # --------------------------------------------------------------------------------------------

    # TODO! call close from FmMain!
    def close(self):
        """Ensure the log file is properly closed."""
        for handler in self.logger.handlers[:]:
            handler.close()
            self.logger.removeHandler(handler)

    # --------------------------------------------------------------------------------------------

    def fm_add_landmark_request(self, f_id, map_name, loc_type, loc_pose, loc_id, loc_neighbor_ids, config_file_path, graph=None, itinerary=None):
        """ validate new landmark to be added to graph """

        graph = graph or self.task_dictionary['graph']
        itinerary = itinerary or self.task_dictionary['itinerary']

        if len(loc_pose) < 1:
            return

        curr_landmarks = []

        # Define the regular expression pattern
        pattern = re.compile(r'^[A-Z][1-9]\d*|^[A-Z]10\d*$')

        # Check if input_loc_id matches the pattern
        if not pattern.match(loc_id):
            self.logger.error('Location Number 0 cannot be chosen. Start numbering from 1.\
                Only single capital lettered alphabets can be used. E.g. A1, E54 etc.')
            return
        else: # check if its already in the landmarks

            # Step 1: Convert itinerary to a dictionary for O(1) lookups
            itinerary_dict = {item["loc_id"]: item for item in itinerary if item["fleet_id"] == f_id}

            # Step 2: Check for duplicates and build `curr_landmarks`
            if loc_id in itinerary_dict:
                self.logger.error('Location ID already exists.')
                return

            curr_landmarks = list(itinerary_dict.keys())

            # Step 3: Validate `edges` and compute distances in one pass
            # Remove spaces and split the neighbors into a list
            edges = [edge.strip() for edge in loc_neighbor_ids.split(",")]

            # Define the regular expression pattern for validation
            pattern = re.compile(r'^[A-Z]\d+$')

            edge_list = []  # Initialize the edge list
            x, y = round(float(loc_pose[0]), 2), round(float(loc_pose[1]), 2)

            for neighbor in edges:
                # Validate the neighbor ID format
                if not pattern.match(neighbor):
                    print("okay1 ", neighbor)
                    self.logger.error("Invalid element in Neighbors field: %s.", neighbor)
                    return

                # Check if the neighbor is in `curr_landmarks`
                if neighbor not in curr_landmarks:
                    print("okay2 ", neighbor)
                    self.logger.error("Element %s in 'Neighbors' field not a landmark.", neighbor)
                    return

                # Compute the distance if the neighbor exists in `itinerary_dict`
                neighbor_item = itinerary_dict[neighbor]
                coordinate = neighbor_item["coordinate"]
                distance = round(math.sqrt((float(coordinate[0]) - x) ** 2 + (float(coordinate[1]) - y) ** 2), 2)
                edge_list.append((neighbor, distance))

            self.add_location_db_cmd(f_id, map_name, loc_type, loc_pose, loc_id, edge_list, config_file_path, graph, itinerary)

    # --------------------------------------------------------------------------------------------

    def add_location_db_cmd(self, f_id, map_name, node_type, node_pose, node_id, edge_list, config_file_path=None, graph=None, itinerary=None):
        """ add new landmark to be added to graph """

        graph = graph or self.task_dictionary['graph']
        itinerary = itinerary or self.task_dictionary['itinerary']

        # Check if node matches the pattern of a single capital letter followed by one or more digits e.g. 'A1'
        if not re.match(r'^[A-Z]\d+$', node_id):
            return

        x = round(float(node_pose[0]), 2)
        y = round(float(node_pose[1]), 2)
        z = round(float(node_pose[2]), 2)
        w = round(float(node_pose[3]), 2)

        temp_dict = {"loc_id": node_id,
                    "map_id": map_name,
                    "fleet_id": f_id,
                    "description": node_type,
                    "coordinate": [x, y, w, z]}

        # update the edges and cost of the listed neighbours
        new_graph = self.add_node_and_edges(node_id, edge_list, graph)

        # convert the graph to the desired YAML format update the 'graph' entity in the dictionary
        graph = {}  # clear the old graph
        for node, edges in new_graph.items():
            yaml_edges = []
            for edge in edges:
                yaml_edges.append([edge[0], edge[1]])
            graph[node] = yaml_edges

        # Update the 'nodes' to include the recently added node
        itinerary.append(temp_dict)

        # Write the updated YAML back to the file
        self.dump_to_yaml(graph, itinerary, config_file_path)

        self.logger.info("landmark co_ord: x %s, y %s, z %s, w %s added successfully! yaml file updated!", x, y, z, w)

    # --------------------------------------------------------------------------------------------

    def add_node_and_edges(self, node, edges, graph=None):
        """ update node and edge list """

        # add the node to the graph, if not already present
        if node not in graph:
            graph[node] = []

        for edge in edges:
            # split the tuple into neighbor and cost
            neighbor, cost = edge
            cost = float(cost)
            # check if edge already exists
            existing_edge = None
            for i, n in enumerate(graph[node]):
                if n[0] == neighbor:
                    existing_edge = i
                    break
            # if edge exists, update the cost
            if existing_edge is not None:
                graph[node][existing_edge][1] = cost
                # update the neighbor's edge as well
                for i, n in enumerate(graph[neighbor]):
                    if n[0] == node:
                        graph[neighbor][i][1] = cost
            # if edge does not exist, add the edge
            else:
                graph[node].append([neighbor, cost])
                # add the node to the neighbor's neighbors, if not already present
                if neighbor not in graph:
                    graph[neighbor] = []
                if node not in [n[0] for n in graph[neighbor]]:
                    graph[neighbor].append([node, cost])
        return graph

    # --------------------------------------------------------------------------------------------

    def fm_delete_landmark_request(self, f_id, loc_id, config_file_path=None, graph=None, itinerary=None):
        """ delete any landmark or node. """

        graph = graph or self.task_dictionary['graph']
        itinerary = itinerary or self.task_dictionary['itinerary']
        f_id = f_id or self.fleetname

        # Define the regular expression pattern
        pattern = re.compile(r'^[A-Z][1-9]\d*|^[A-Z]10\d*$')

        # Check if input_loc_id matches the pattern
        if not pattern.match(loc_id):
            self.logger.error('Location Number 0 cannot be chosen. Start numbering from 1.\
                Only single capital lettered alphabets can be used. E.g. A1, E54 etc.')
            return

        for index, item in enumerate(itinerary):
            if item["fleet_id"] == f_id:
                if item["loc_id"] == loc_id:
                    del itinerary[index]
                    break

        if loc_id in graph:
            del graph[loc_id]

        for node, edges in graph.items():
            graph[node] = [edge for edge in edges if edge[0] != loc_id]

        # Update the 'nodes' to include the recently added node
        self.dump_to_yaml(graph, itinerary, config_file_path)

    # --------------------------------------------------------------------------------------------
    # TODO
    # After dumping you need to refresh all instances of task_dictionary!!

    def dump_to_yaml(self, updated_graph, updated_itinerary, config_file_path=None):
        """
        Updates the 'graph' and 'itinerary' sections in a YAML file.

        Parameters:
        config_file_path (str): Path to the YAML file.
        updated_graph (dict): Updated graph data.
        updated_itinerary (list): Updated itinerary data.
        """

        if config_file_path is None:
            self.logger.error("Configuration file path not provided.")
            return

        try:
            # Read the existing YAML file
            with open(config_file_path, 'r', encoding='utf-8') as file:
                yaml_content = yaml.safe_load(file)

            # Update only the graph and itinerary sections
            yaml_content['graph'] = updated_graph
            yaml_content['itinerary'] = updated_itinerary

            # Write the updated content back to the YAML file
            with open(config_file_path, 'w', encoding='utf-8') as outfile:
                yaml.dump(yaml_content, outfile)

            self.logger.info("Successfully updated 'graph' and 'itinerary' in %s", config_file_path)
        except FileNotFoundError:
            self.logger.error("File %s not found.", config_file_path)
        except yaml.YAMLError as e:
            self.logger.error("Error processing YAML file: %s", e)



    # --------------------------------------------------------------------------------------------

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
            self.drop_maps_table()
        self.create_maps_table()

    # --------------------------------------------------------------------------------------------

    def drop_maps_table(self):
        """
        Drops the table_maps table from the database.
        """
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("DROP TABLE IF EXISTS "+self.table_maps)
            self.db_conn.commit()
            self.logger.info("table %s dropped successfully.",self.table_maps)
        except Exception as e:
            self.logger.error("Error dropping table '%s': '%s'", self.table_maps, e)

    # --------------------------------------------------------------------------------------------

    def create_maps_table(self):
        """ create instant action table """
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("""
                SELECT EXISTS (
                    SELECT FROM information_schema.tables
                    WHERE table_name = '"""+self.table_maps+"""'
                );
            """)
            if not cursor.fetchone()[0]:
                # SQL query to create the maps table
                create_table_query = """
                CREATE TABLE IF NOT EXISTS maps (
                    map_id SERIAL PRIMARY KEY,
                    fleet_name VARCHAR(100) NOT NULL,
                    map_name VARCHAR(255) NOT NULL,
                    pgm_data BYTEA NOT NULL,
                    yaml_data TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
                """
                cursor.execute(create_table_query)
                self.db_conn.commit()
                self.logger.info("Maps table created successfully.")

        except Exception as e:
            self.logger.error("Error: %s.", e)
        finally:
            if self.db_conn:
                cursor.close()
                # self.db_conn.close()

    # --------------------------------------------------------------------------------------------

    def insert_maps_db(self, msg):
        """Insert map image.pgm and .yaml into db."""
        try:
            fleet_name = msg['fleet_name']
            map_name = msg['map_name']
            pgm_file_path = msg['pgm_file_path']
            yaml_file_path = msg['yaml_file_path']

            cursor = self.db_conn.cursor()

            # Read PGM file (binary mode)
            with open(pgm_file_path, 'rb') as pgm_file:
                pgm_data = pgm_file.read()

            # Read YAML file (text mode)
            with open(yaml_file_path, 'r', encoding='utf-8') as yamlfile:
                yaml_data = yamlfile.read()

            # Use safe formatting for table names
            query = f"""
                INSERT INTO {self.table_maps}
                (fleet_name, map_name, pgm_data, yaml_data)
                VALUES (%s, %s, %s, %s)
                RETURNING map_id;
            """

            # Execute the query safely with parameters
            cursor.execute(query, (fleet_name, map_name, pgm_data, yaml_data))

            # Commit the changes
            self.db_conn.commit()

            # Retrieve the newly added map's ID
            # map_id = cursor.fetchone()[0]
            # self.logger.info(f"Map '{map_name}' added to database with id {map_id}")
        except Exception as e:
            self.logger.error("Error: %s.", e)
        finally:
            # Close the cursor
            if cursor:
                cursor.close()

    # --------------------------------------------------------------------------------------------


    def fetch_all_data(self, f_id):
        """Fetch all rows and columns of data based on fleet_id from the database."""
        try:
            cursor = self.db_conn.cursor()
            query = """
                SELECT map_id, fleet_name, map_name
                FROM """ + self.table_maps + """
                WHERE fleet_name = %s
                ORDER BY created_at DESC;
            """
            cursor.execute(query, (f_id,))
            records_ = cursor.fetchall()

            if not records_:
                self.logger.error("No maps data found for fleet_name %s", f_id)
                return []

            return [
                {"map_id": map_[0], "fleet_name": map_[1], "map_name": map_[2]}
                for map_ in records_
            ]

        except Exception as er:
            self.logger.error("fetch_data maps Database Error: %s", er)
            return []

        finally:
            cursor.close()

    # --------------------------------------------------------------------------------------------

    def subscribe(self, mqtt_client):
        """ subscribe_to_topics """
        topic = f"{self.fleetname}/{self.versions}/+/+/visualization"
        mqtt_client.subscribe(topic, qos=0)
        self.logger.info("Subscribed to topic: %s", topic)

    # --------------------------------------------------------------------------------------------

    def validate_message(self, message):
        """ validate_message """
        try:
            validate(instance=message, schema=self.visualization_schema)
        except ValidationError as er:
            self.logger.info("Visualization schema validation failed: %s", er.message)
            raise

    # --------------------------------------------------------------------------------------------

    def process_message(self, message):
        """ process_message """
        try:
            self.validate_message(message)
            self.parse_visualization_dt(message)
        except ValidationError:
            self.logger.error("[ConnectionSubscriber] State message validation failed. Skipping database save.")

    # --------------------------------------------------------------------------------------------

    def parse_visualization_dt(self, message):
        """ parse visualization data """
        serial_number = message.get("serialNumber", "")
        agv_position = message.get("agvPosition", {})
        # velocity = message.get("velocity", {})
        # self.logger.info("AGV: %s", serial_number)
        # self.logger.info("AGV Position: x=%s, y=%s, theta=%s", agv_position.get('x'), agv_position.get('y'), agv_position.get('theta'))
        # self.logger.info("AGV Velocity: vx=%s, vy=%s, omega=%s", velocity.get('vx'), velocity.get('vy'), velocity.get('omega'))
        self.robot_positions[serial_number] = {'x': agv_position.get('x'), 'y': agv_position.get('y')}

    # --------------------------------------------------------------------------------------------

    def terminal_log_visualization(self, message, class_name, function_name, log_type="debug"):
        """
        Custom logging function with ANSI escape codes for color formatting.
        Args:
            message (str): The message to log.
            class_name (str): The name of the class where the log originates.
            function_name (str): The name of the function where the log originates.
            log_type (str): The type of log (debug, warn, error, info). Determines color.
        """
        # ANSI color codes
        log_colors = {
            "debug": "\033[34m",  # Blue
            "warn": "\033[33m",   # Yellow
            "error": "\033[31m",  # Red
            "info": "\033[32m",   # Green
            "critical": "\033[35m",  # Orange (ANSI escape code for orange)
            "notice": "\033[38;5;208m"   # Purple
        }

        # Fallback color if type is unknown
        color = log_colors.get(log_type.lower(), "\033[37m")  # Default to white

        # Reset code
        reset = "\033[0m"

        # Current timestamp
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Format class name in uppercase (no additional color change)
        # class_name_formatted = class_name.upper()

        # Build and print the log message
        formatted_message = (
            f"{color}[{log_type.upper():<5}] {reset}"   # Log type with color
            f"\033[36m[{timestamp}]\033[0m "           # Timestamp in cyan
            f"\033[1m{class_name}.{function_name}\033[0m "  # Class and function bold
            f"- {message}"
        )

        if log_type == self.log_level or self.log_level == 'debug':
            # print(formatted_message)
            self.logger.info(formatted_message)

        # if log_type == 'error':
        #    self.logger.error(formatted_message)

    # # --------------------------------------------------------------------------------------------

    def terminal_graph_visualization(self, graph=None, itinerary=None, robot_positions=None):
        """Show motion as ASCII art on terminal with fixed-width grid cells."""
        # Step 0: Use default values if needed
        graph = graph or self.task_dictionary['graph']
        itinerary = itinerary or self.task_dictionary['itinerary']
        robot_positions = robot_positions or self.robot_positions

        # Step 1: Extract node positions (filter by fleet_id)
        node_positions = {
            item['loc_id']: (item['coordinate'][0], item['coordinate'][1])
            for item in itinerary if item['fleet_id'] == self.fleetname
        }

        # Step 2: Normalize coordinates to fit into a 2D ASCII grid
        all_x = [coord[0] for coord in node_positions.values()]
        all_y = [coord[1] for coord in node_positions.values()]
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        grid_width = 50
        grid_height = 20

        def normalize(val, min_val, max_val, grid_size):
            return int((val - min_val) / (max_val - min_val) * (grid_size - 1))

        # Use a fixed cell width (e.g., 4 characters per cell)
        cell_width = 4

        # Initialize grid (each cell is an empty string with cell_width spaces)
        grid = [[' ' * cell_width for _ in range(grid_width)] for _ in range(grid_height)]

        # Step 3: Place nodes on the grid with conflict resolution
        node_grid_positions = {}
        occupied_positions = set()

        for node, (x, y) in node_positions.items():
            gx = normalize(x, min_x, max_x, grid_width)
            gy = normalize(y, min_y, max_y, grid_height)

            # Resolve conflicts by shifting to the nearest free space
            while (gx, gy) in occupied_positions:
                gx += 1
                if gx >= grid_width:
                    gx = 0
                    gy += 1
                if gy >= grid_height:
                    gy = 0

            occupied_positions.add((gx, gy))
            node_grid_positions[node] = (gx, gy)
            # Center the node label within the fixed cell width
            grid[gy][gx] = node.center(cell_width)

        # Step 4: Draw edges between nodes using Bresenham's algorithm
        def draw_line(grid, x1, y1, x2, y2):
            points = list(bresenham(x1, y1, x2, y2))
            for px, py in points:
                # Only draw if this cell hasn't been occupied by a node label or robot symbol
                if grid[py][px].strip() == '':
                    grid[py][px] = '.' * cell_width

        def bresenham(x1, y1, x2, y2):
            """Bresenham's line algorithm."""
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            sx = 1 if x1 < x2 else -1
            sy = 1 if y1 < y2 else -1
            err = dx - dy
            while True:
                yield x1, y1
                if x1 == x2 and y1 == y2:
                    break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x1 += sx
                if e2 < dx:
                    err += dx
                    y1 += sy

        for node, edges in graph.items():
            if node in node_grid_positions:
                x1, y1 = node_grid_positions[node]
                for edge in edges:
                    neighbor = edge[0]
                    if neighbor in node_grid_positions:
                        x2, y2 = node_grid_positions[neighbor]
                        draw_line(grid, x1, y1, x2, y2)

        # Step 5: Assign random colors to robots
        colors = ['\033[34m', '\033[33m', '\033[31m', '\033[32m', '\033[36m', '\033[35m', '\033[38;5;208m']
        robot_colors = {robot: random.choice(colors) for robot in robot_positions}

        # Step 6: Place robots on the grid (using their x,y coordinates)
        for robot, position in robot_positions.items():
            rx, ry = position['x'], position['y']
            gx = normalize(rx, min_x, max_x, grid_width)
            gy = normalize(ry, min_y, max_y, grid_height)

            # Resolve conflicts with other robots (or nodes already placed)
            while (gx, gy) in occupied_positions:
                gx += 1
                if gx >= grid_width:
                    gx = 0
                    gy += 1
                if gy >= grid_height:
                    gy = 0

            occupied_positions.add((gx, gy))
            # Use robot symbol with color, ensuring it fits the fixed width
            grid[gy][gx] = f"{robot_colors[robot]}{robot.center(cell_width)}\033[0m"

        # Step 7: Render the grid
        print(" ")
        for row in grid:
            print(''.join(row))

    # # --------------------------------------------------------------------------------------------


# Example usage
if __name__ == "__main__":

   # establish connection to DB
    conn = None
    try: # Sample database connection setup (assuming the database is already created)
        conn = psycopg2.connect(host='localhost', dbname='postgres', user='postgres', password='root', port='5432')
    except (psycopg2.OperationalError, psycopg2.ProgrammingError) as e:
        print("Failed to connect to PostgreSQL database: %s", e)

    # initialize the task network dictionary
    task_dict_ = None
    task_dic_ = None
    maps_ = {}

    # get config file path:
    agv_dir_prefix = (os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))
    print("Config directory path: ", agv_dir_prefix)
    # file_path = Path(agv_dir_prefix+'/viro_simple_fleet/config/fleet_mngr.yaml')
    file_path = Path(os.path.join(agv_dir_prefix, 'fleet_management', 'config', 'config.yaml'))
    if file_path.is_file():
        with open(file_path, 'r', encoding='utf-8') as yaml_file:
            task_dict_ = yaml.safe_load(yaml_file)

            task_dic_ = {"itinerary": task_dict_["itinerary"],
                        "graph": task_dict_["graph"]}
            maps_ = task_dict_["maps"]
            # print("Graph:", task_dic_["graph"])
            # print("Itinerary:", task_dic_["itinerary"])
    else:
        print("file path not set.")

    fleetname_ = "kullar"
    # new_base_request = robot_data.get("newBaseRequest", False)

    # Initialize VisualizationSubscriber
    visualization_sub = VisualizationSubscriber(fleetname=fleetname_, versions="v1", dbconn=conn, mqttclient=None,
                                    task_dict = task_dic_, dbname = 'postgres')

    # visualization_sub.robot_positions = {
    #     'R1': {'x': -3.0, 'y': 3.0},
    #     'R2': {'x': -3.0, 'y': 6.0},
    # }

    visualization_sub.robot_positions = {
        'R1': {'x': -0.55, 'y': -0.55},
        'R2': {'x': 0.5, 'y': -1.8},
    }


    visualization_sub.terminal_graph_visualization()

    COUNT = 0
    while COUNT <= 13:
        COUNT += 1
        print(" \n")
        visualization_sub.robot_positions['R1']['x'] += 0.15
        visualization_sub.robot_positions['R1']['y'] -= 0.0
        visualization_sub.terminal_graph_visualization()

    visualization_sub.terminal_log_visualization("Executing HARD blocking action: action_type", "FmTaskHandler", "execute_hard_block", "debug")
    visualization_sub.terminal_log_visualization("Battery level lower than minimum required.", "FmTaskHandler", "check_battery", "warn")
    visualization_sub.terminal_log_visualization("Failed to decode order message", "FmTaskHandler", "decode_order", "error")
    visualization_sub.terminal_log_visualization("Reassigning loop task successfully.", "FmScheduleHandler", "reassign_loop_task", "info")
    visualization_sub.terminal_log_visualization("Name was not entered properlly.", "FmMain", "interactive_robot_fleet_startup", "critical")

    if fleetname_ in maps_:
        for map_info in maps_[fleetname_][0]:
            # get pgm and yaml file path
            pgm_file_path_ = map_info['map_pgm_path'] # "/home/birfenberke/fleet_management_visualizer/public/aronyamap.pgm"
            yaml_file_path_ = map_info['map_yaml_path'] # "/home/birfenberke/fleet_management_visualizer/public/aronyamap.yaml"
            # Check if PGM and YAML files exist
            if (os.path.isfile(pgm_file_path_)) and (os.path.isfile(yaml_file_path_)):
                # Create the message dictionary to pass to the insert function
                msg_ = {
                    'fleet_name': fleetname_,
                    'map_name': map_info['map_id'],
                    'pgm_file_path': pgm_file_path_,
                    'yaml_file_path': yaml_file_path_
                }
                # Call the insert function
                # visualization_sub.insert_maps_db(msg_)

    # self.station_type = ['door', 'elevator', 'checkpoint', 'charge_dock', 'station_dock', 'home_dock', 'waitpoint']

    # # door | elevator
    # - coordinate:
    #   - 0.53
    #   - 0.0
    #   - -0.999
    #   - 0.006
    #   description: door # elevator
    #   fleet_id: kullar
    #   map_id: bina_1_floor_0
    #   loc_id: C13

    # # This part expects that fleet, maps, and their respective paths have been set in the config.yaml
    # map_name_ = []
    # records = visualization_sub.fetch_all_data(fleetname_)
    # if records:
    #     for record in records:
    #         map_name_.append(record["map_name"])

    # if map_name_:
    #     # add a node 1
    #     loc_type_ = "elevator"
    #     loc_pose_ = [0.53, 0.0, -0.999, 0.006]
    #     loc_id_ = "C13"
    #     loc_neighbor_ids_ = "C10"
    #     map_id = map_name_[0]
    #     visualization_sub.fm_add_landmark_request(fleetname_, map_id, loc_type_, loc_pose_, loc_id_, loc_neighbor_ids_,file_path)

    #     # add another node 2
    #     loc_type_ = "door"
    #     loc_pose_ = [0.53, 0.0, -0.999, 0.006]
    #     loc_id_ = "C14"
    #     loc_neighbor_ids_ = "C13, C10"
    #     map_id = map_name_[0]
    #     visualization_sub.fm_add_landmark_request(fleetname_, map_id, loc_type_, loc_pose_, loc_id_, loc_neighbor_ids_,file_path)

    #     # delete the node.
    #     loc_id_ = "C13"
    #     visualization_sub.fm_delete_landmark_request(fleetname_, loc_id_, file_path)

    # else:
    #     print('No map found. This part expects that fleet_id, maps and their respective .yaml and .pgm paths \
    #                     have been set in the config.yaml.')












    # # --------------------------------------------------------------------------------------------
    # # --------------------------------------------------------------------------------------------
    # # --------------------------------------------------------------------------------------------

    # def reset_database_release_status(fleet_id=None, floor_id=None):
    #     """
    #     Veritabanındaki tüm `nodes` ve `edges` tablolarındaki `released` durumlarını `True` olarak ayarlar.
    #     Opsiyonel olarak `fleet_id` ve `floor_id` filtreleri kullanılabilir.
    #     """
    #     try:

    #         cursor = self.db_conn.cursor()

    #         # `nodes` tablosunu güncelleme sorgusu
    #         if fleet_id and floor_id:
    #             # Belirtilen `fleet_id` ve `floor_id` için `nodes` tablosunu güncelle
    #             reset_nodes_query = """
    #             UPDATE nodes
    #             SET released = True
    #             WHERE fleet_id = %s AND floor_id = %s;
    #             """
    #             cursor.execute(reset_nodes_query, (fleet_id, floor_id))
    #         else:
    #             # Tüm `nodes` tablosundaki `released` durumlarını True yap
    #             reset_nodes_query = """
    #             UPDATE nodes
    #             SET released = True;
    #             """
    #             cursor.execute(reset_nodes_query)

    #         # `edges` tablosunu güncelleme sorgusu
    #         if fleet_id and floor_id:
    #             # Belirtilen `fleet_id` ve `floor_id` için `edges` tablosunu güncelle
    #             reset_edges_query = """
    #             UPDATE edges
    #             SET released = True
    #             WHERE fleet_id = %s AND floor_id = %s;
    #             """
    #             cursor.execute(reset_edges_query, (fleet_id, floor_id))
    #         else:
    #             # Tüm `edges` tablosundaki `released` durumlarını True yap
    #             reset_edges_query = """
    #             UPDATE edges
    #             SET released = True;
    #             """
    #             cursor.execute(reset_edges_query)

    #         # Güncellemeleri veritabanına yansıtma
    #         self.db_conn.commit()
    #         print(f"[OrderPublisher] Released durumları veritabanında `True` olarak güncellendi.")

    #         # Veritabanı bağlantısını kapatma
    #         cursor.close()
    #         # self.db_conn.close()

    #     except Exception as e:
    #         print(f"[OrderPublisher] Error dropping table_order table: {e}")

    # # --------------------------------------------------------------------------------------------

    # def fetch_filtered_edges(self, fleet_id, floor_id):
    #     cursor = self.conn.cursor()
    #     cursor.execute("""
    #         SELECT id, source_node_id, target_node_id, label, created_at, updated_at, edge_id, fleet_id, floor_id, released
    #         FROM edges
    #         WHERE fleet_id = %s AND floor_id = %s;
    #     """, (fleet_id, floor_id))
    #     edges = cursor.fetchall()
    #     return [
    #         {
    #             "id": edge[0],
    #             "source_node_id": edge[1],
    #             "target_node_id": edge[2],
    #             "label": edge[3],
    #             "created_at": edge[4],
    #             "updated_at": edge[5],
    #             "edge_id": edge[6],
    #             "fleet_id": edge[7],
    #             "floor_id": edge[8],
    #             "released": edge[9]
    #         }
    #         for edge in edges
    #     ]

    # # --------------------------------------------------------------------------------------------

    # def fetch_filtered_nodes(self, fleet_id, floor_id):
    #     """ - - """
    #     cursor = self.conn.cursor()
    #     cursor.execute("""
    #         SELECT id, node_id, label, position_x, position_y, theta, created_at, updated_at, fleet_id, floor_id, released
    #         FROM nodes
    #         WHERE fleet_id = %s AND floor_id = %s;
    #     """, (fleet_id, floor_id))
    #     nodes = cursor.fetchall()
    #     return [
    #         {
    #             "id": node[0],
    #             "node_id": node[1],
    #             "label": node[2],
    #             "position_x": float(node[3]),
    #             "position_y": float(node[4]),
    #             "theta": node[5],
    #             "created_at": node[6],
    #             "updated_at": node[7],
    #             "fleet_id": node[8],
    #             "floor_id": node[9],
    #             "released": node[10]
    #         }
    #         for node in nodes
    #     ]

    # # --------------------------------------------------------------------------------------------

    # def build_graph_and_itinerary(self, fleet_id, floor_id):
    #     # Fetch nodes and edges
    #     nodes = self.fetch_filtered_nodes(fleet_id, floor_id)
    #     edges = self.fetch_filtered_edges(fleet_id, floor_id)

    #     # Build the graph
    #     graph = {}
    #     for node in nodes:
    #         graph[node['node_id']] = []

    #     for edge in edges:
    #         source_node = edge['source_node_id']
    #         target_node = edge['target_node_id']
    #         # Calculate the distance between nodes as the cost
    #         source_position = next(node for node in nodes if node['node_id'] == source_node)
    #         target_position = next(node for node in nodes if node['node_id'] == target_node)
    #         distance = math.sqrt((source_position['position_x'] - target_position['position_x']) ** 2 +
    #                             (source_position['position_y'] - target_position['position_y']) ** 2)
    #         graph[source_node].append([target_node, distance])

    #     # Build the itinerary
    #     itinerary = []
    #     for node in nodes:
    #         itinerary.append({
    #             'coordinate': [node['position_x'], node['position_y'], 0, 1],  # Assuming theta to be 0 and 1 for simplicity
    #             'description': node['label'],
    #             'fleet_id': node['fleet_id'],
    #             'loc_id': node['node_id']
    #         })

    #     return graph, itinerary

    # # --------------------------------------------------------------------------------------------

    # def update_base_nodes_and_edges(self, fleet_id, floor_id, base_nodes, base_edges):
    #     """ - - - - """
    #     try:
    #         cursor = self.conn.cursor()

    #         if base_nodes:
    #             cursor.execute("""
    #                 UPDATE nodes
    #                 SET released = False
    #                 WHERE node_id = ANY(%s) AND fleet_id = %s AND floor_id = %s;
    #             """, (base_nodes, fleet_id, floor_id))
    #             print(f"Base düğümler güncellendi: {base_nodes}")

    #         if base_edges:
    #             base_edge_pairs = [edge.split(" -> ") for edge in base_edges]
    #             for source_node, target_node in base_edge_pairs:
    #                 cursor.execute("""
    #                     UPDATE edges
    #                     SET released = False
    #                     WHERE source_node_id = %s AND target_node_id = %s AND fleet_id = %s AND floor_id = %s;
    #                 """, (source_node, target_node, fleet_id, floor_id))
    #                 print(f"Base kenar güncellendi: {source_node} -> {target_node}")

    #         self.conn.commit()

    #     except Exception as e:
    #         print(f"Base düğümler ve kenarları güncellerken hata oluştu: {e}")
    #         self.conn.rollback()

    # # --------------------------------------------------------------------------------------------

    # import asyncio
    # import aiomqtt
    #     async def main(self):
    #         await self.initialize_database_connection()
    #         await self.initialize_order_publisher()

    #         async with aiomqtt.Client("localhost") as client:
    #             self.client = client
    #             await self.publish_order(client, "robot_1", "N13", "N17", 1, 2)
    #             await client.subscribe("uagv/v2/robots/#")
    #             async for message in client.messages:
    #                 if message.topic.matches("uagv/v2/robots/+/state"):
    #                     await self.handle_state_message(message)
    #                 if message.topic.matches("uagv/v2/robots/+/order"):
    #                     pass

    #     async def run(self):
    #         await self.main()

    # if __name__ == "__main__":
    #     config_path = os.path.join(os.path.dirname(__file__), 'config', 'config.yaml')
    #     fleet_manager = FleetManager(config_path)
    #     asyncio.run(fleet_manager.run())
