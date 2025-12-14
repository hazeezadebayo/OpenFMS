from paho.mqtt import client as mqtt_client
from datetime import datetime
import threading
import time
import json
import random
import math
import re

# mosquitto_sub -h localhost -p 1883 -t kullar/v2/birfen/AGV-002/state
# mosquitto_sub -h localhost -p 1883 -t kullar/v2/birfen/AGV-002/factsheet
# mosquitto_sub -h localhost -p 1883 -t kullar/v2/birfen/AGV-002/connection
# mosquitto_sub -h localhost -p 1883 -t kullar/v1/birfen/AGV-001/factsheet

########################################################################
# what TODO:
# CODE:
# -- High priority:
# 1. Twilio! or just print to screen?
# 2. Docker container for it - avoid it works on my computer!
# 3. Robotsim should be more robust: reject tasks, check update ids etc.
# 4. Mutex groups!? lock and unlock multiple nodes, if the nodes do not have associated waitpoints.

# -- Low priority:
# 1. Tailscale or husarion VPN or k8s for deployment and CI template
# 2. include logic for time based task scheduling e.g daily, hourly etc.
# 3. zone_id is not for fleet_id.  "resting" "forbidden" "restricted" "zone_vehicle_limit" etc. we need a better logic to hold fleet names.
# 4. WebSocket: Used for ROS and HTML stuffs.
# 5. cpp instead? better and faster at multi-threading
# 6. re-assign tasks for a suspended or ignored robot. decommissioning and commissioning.
# 7. user login!! authorization etc.
# 8. include charge duration limits so robots can request removal themselves when time is up.
# 9. BEVY_impulse entity component system.

# TESTS:
# reservation test: check for effects of handled actions, just send running like some times, send finished after time elapse. see if it knows to reserve next node.
# autocharge test: after each task, drop battery by 30 percent or something, see if it goes to charge itself.
# queue test: submit order while robot still working, see if it gets queued.
# cancel test: during motion, cancel task, see if robot goes back home.
# pause resume test: like cancel, but see if resume makes it continue motion.
# analytics test: how many orders fullfilled by x robot, today etc.

########################################################################
# What is assumed or expected for use:
# 1. checkpoints are designated by 'C' and are linked by a straight line.
# 2. checkpoints are at least 2 time the length of agv.
# 3. naming convention or concensus is that you have 'C' representing the type of node and a number as in 'C1', 'C2', 'C45' etc.
# 4. the number written with the designated letter can not be repeated. two nodes should not have same name.
# 4. some checpoints may have waitpoints or wait areas associated with them designated as 'W' and follows similar convention 'W2' would imply the waitpoint associated with 'C2' checkpoint 2.
# 5. waitpoints are only accessible only from the checkpoint node they are connected to and no other node accessibility. 'W3' can only go to 'C3'.
# 6. checkpoints can be connected in any way, order, shape or form as biderectionality or single directional connections are possible.
# 7. reservation is a robot at a time hence, two robots cannot share same edge or lane.
# 8. each node has coordinate, description, fleet_id, map_id as well as loc_id properties.
# 9. cordinate: [x y w z] identifies the physical location of the node on the map
# 10. description: holds string values which is one of
#           a. 'charge_dock': designated by 'Cx'. where x is any none repeating number. identifies robots charge locations or spots. do not have associated waitpoints.
#           b. 'home_dock': 'Cx'. identifies robots home or park locations or spots. do not have associated waitpoints.
#           c. 'station_dock': 'Cx'. identifies physical payload drop off or pickup locations. do not have associated waitpoints.
#           d. 'checkpoint': 'Cx'. identifies traversal points or locations that facilitates navigation. may have associated waitpoints.
#           e. 'elevator': 'Cx'. identifies nodes or trigger points for which robot should switch current map to the horizon node's map. do not have associated waitpoints.
#           f. 'door': 'Cx'. identifies gateway that do not trigger a map change but trigger a 'waitForTrigger' action for which the robot physically have to wait for a physical trigger to
#                proceed perhaps the opening of a door. this also do not have associated waitpoints.
#           g. 'waitpoint': 'Wx'. identifies areas for which robot can go and wait in order to yield their reserved node to a more high priority tasked robot.
# 11. fleet_id: identifies the fleet permitted to use the node or resource. i.e. only robots belonging to X fleet may use this node.
# 12. map_id: identifies the map name. multiple maps may exist in one fleet. perhaps because the floor is so big or perhaps because multiple floors exist but same robots may traverse.
# 13. loc_id: identifies the node's distinct name. e.g 'C52' 'W12' 'C23' etc.
# 14. all robots have there own distinct home dock nodes, also designated by 'C'.
# 15. charging docks are different from home docks. but algorithm also supports cases where they are same, in this case a charge_dock entry with different name is created but its location is set
#       to the coordinate of its home dock.
# 16. queues for charging are not based on priority but FIFO.
# 17. elevator node itself should carry the map of the current floor. robot must trigger a request map change, switching to the horizon node's map.
# 18. elevator themselves should not and do not have wait nodes. however, checkpoint nodes 'C' before any elevator should have wait nodes W. it makes sense for robot to be able to pass once they
#      leave the elevator.
# 19. manager uploads map and provides link, if robot is online, when robot gets node in order/task it can download 'map_id' before starting task.
# 20. use of self explanatory vda5050 compliant action triggers: cancelOrder, stopCharging, startCharging, pick, drop, and waitForTrigger.
# 21. the robot is expected to have a navigation system or controller that can help move between nodes. algorithms like mpc, pure pursuit, carrot chase etc. ros2 integration is supported and tested.
# 22. only one node leads to a station or home or charge node, no multiple entrance.
#

########################################################################
# What is offered:
# 1. Multi-Robot Management: Centralised fleet Monitor and direct/manage multiple robots simultaneously.
# 2. Traffic management or Deadlock resolution: Negotiation to resolve conflicts - (e.g., “reserve node X,” “adjust path go to node Y an alternative path,”
#              “yield your current node for higher-priority robot -only if wait area/node is available for your current occupied node-”),
#               "wait some sec: wait for robot to pass no conflict or standoff detected." -.
#               the central manager handles all resource allocation, all robots openly communicate their current trajectory/target node for monitoring and management.
#               Negotiation is done on the basis of task priority and robot wait time. algorithm addresses:
#               a. Cross Conflicts : happens when two robots cross the intersection point at the same time. the algorithm uses time constraint or task priorities to ascertain which robot waits and
#                   which proceeds.
#               b. Head-on Conflicts : occurs when two robots facing each other try to go in the opposite directions. the algorithm uses node reservations to ensure orderliness. and profers an
#                   alternative route, or a temporary wait point to avoid a deadlock.
#               c. Chasing Conflicts : happens when one mobile robot is following the same route with a slower robot that is in front of it. not addressed in this algorithm as both robot continues
#                   motion either way.
#               d. Stay-on Conflicts : occurs when a robot tries to travel to a location that is occupied by a static (i.e. docked at the intended home dock, or charge dock) robot. the algorithm
#                   terms this as the last mile run and addresses it by finding alternative available dock and reroutes the robots.
#               noting that some of these may lead to a deadlock as it is defined as the inability of multiple robots to move further because each of them aims to occupy the
#               location currently occupied by another robot in the same group. this is avoided mostly by the adaptation of waitpoints, wherein robot may go to certain
#               wait areas thereby giving way for other robots to pass.
# 3. Task queuing: task are queued in the db if no robot is available to undertake them.
# 4. Task scheduling: tasks have associated priority low medium or high reflecting urgency and are sorted as such.
#                    all tasks.
# 5. Task allocation or dispatching: availability (not currently on a task or emergency or maintenace), payload size, battery level,
#                     and proximity are used as allocation criteria.
# 6. Vehicle or Robot Routing: based on the graph (nodes and edges) shortest path is determined for which each robot may traverse and
#                     are released one after the other only if a conflict is sure to not occur.
# 6. vda5050 msg specification: all messages are standardized according to the vda5050 mobile robot message schema.
# 7. battery management, Auto charging and uncharging scheduling: auto charge request is triggered for individual robots if better level is low,
#                    even while on a task. the task is handed over to another robot, for this to go charge. queues for charging are FIFO.
#                    uncharging or undock from charge is triggered after charge as reached max charge threshold. and as such availability for task is renewed.
# 8. Maintenance aware: emergency button or such similar ui button when clicked automatically deselects or deallocates the robot and as such is never considered in the traffic management
#                    or task allocation. it is simply assumed decommissioned. until removed and reset. and as such availability for task is renewed.
# 9. Visualization: direct terminal graph/node/robot visualization as well as vda5050 based visualization message compliant.
# 10. data logging: log history is held in the postgresql database as well as local storage on the robot
# 11. analytics: query db for information on past completed tasks, uncompleted tasks, robot task history or success count, set maintenance required threshold based on robot trip etc.
# 12. communication: all vda5050 messages are done over mqtt support.
# 13. Ease of use: Easily plan and manage any arbitrary layout since any map is quantized to nodes and edges and as such flexible.
# 14. ros2 open source support: nav2 as well as works out of the box with vda5050 connector adapter package by inorbit.

########################################################################
# What to install:
# pip install psycopg2-binary
# pip install pyyaml
# pip install twilio
# pip install paho-mqtt
# pip install -U 'jsonschema<4.0'



# state message -- every 30sec
# connction message -- every 15sec



class Robot():
    """Robot simulator with dynamic, per-message-type publishing intervals."""
    def __init__(self, fleetname, robot_serial_number, versions, version, manufacturer, connection_state,
                 initial_position, bat_charge=50.0, lin_velocity=0.1, ang_velocity=0.04):

        self.fleetname = fleetname
        self.robot_serial_number = robot_serial_number
        self.versions = versions  # e.g. "v2"
        self.version = version    # e.g. "2.0.0"
        self.manufacturer = manufacturer
        self.connection_state = connection_state

        self.position = initial_position
        self.lin_velocity = lin_velocity
        self.ang_velocity = ang_velocity
        self.battery_charge = bat_charge # Battery percentage starts at x%

        self.order = None # Represents the current task; None if no task is active
        self.instant_action = None
        self.driving = False
        self.target_node = None
        self.newbaserequest = False
        self.wait_start_time = None
        self.last_node_id = ""
        self.action_states = []

        # Initialize last published timestamps (using time.time())
        self.last_state_publish = time.time()
        self.last_connection_publish = time.time()
        self.last_factsheet_publish = time.time()

        self.last_battery_update_time = time.time()
        self.depletion_rate_active = 0.1  # Battery percentage points per second when active
        self.depletion_rate_idle = 0.01   # Battery percentage points per second when idle

        # MQTT Client setup : placeholder ('localhost', 1883)
        self.mqtt_client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1) # Define the MQTT client
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60) # connect mqtt

        # Subscribe to topics
        # Subscribe to MQTT order topic
        self.mqtt_client.subscribe(f"{self.fleetname}/{self.versions}/{self.manufacturer}/{self.robot_serial_number}/order")
        # Subscribe to MQTT instantAction topic
        self.mqtt_client.subscribe(f"{self.fleetname}/{self.versions}/{self.manufacturer}/{self.robot_serial_number}/instantAction")

        # Start MQTT loop in its own thread
        self.mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()

        # wether or not to use dynamic publisher:
        # Start the dynamic publishing loop in a dedicated thread.
        # threading.Thread(target=self.dynamic_publish_loop, daemon=True).start()

        # Timers for publishing state and factsheet
        self.create_timer(1.0, self.publish_msg)  # Publish every 1 second

        # Run MQTT client loop
        # self.mqtt_client.loop_start()


    def on_mqtt_connect(self, client, userdata, flags, rc):
        """ on_mqtt_connect """
        print(f"Connected to MQTT broker with result code {rc}")


    def on_mqtt_message(self, client, userdata, msg):
        """ on mqtt message """
        payload = msg.payload.decode()
        try:
            message = json.loads(payload)
            if "order" in msg.topic:
                # Check order is for this robot
                if message.get("serialNumber") == self.robot_serial_number:
                    self.order = message
                    self.driving = True
                    # Initialize target_node from the first node, if available
                    if self.order.get("nodes"):
                        self.target_node = self.order["nodes"][0] # first target node
                    print(f"Order received for {self.robot_serial_number}: {message}")

            elif "instantAction" in msg.topic:

                action_data =  json.loads(msg.payload)
                print(f"Instant Action Received: {action_data}")

                # Validate schema and action type
                action_list = action_data.get("actions", [])
                for action in action_list:
                    action_id = action.get("actionId")
                    action_type = action.get("actionType")
                    blocking_type = action.get("blockingType", "NONE")
                    parameters = action.get("actionParameters", [])
                    action_description = action.get("actionDescription",
                                                    f"Executing {action_type} with parameters {parameters}")
                    # action_status = "RUNNING"  # Default status; you may dynamically set this based on state

                    # Process based on blocking type
                    if blocking_type in ["NONE", "SOFT"]:
                        print(f"Executing HARD blocking action: {action_type}")
                        self.execute_action(action_id, action_type, parameters, action_description, blocking=True)
                    elif blocking_type == "HARD":
                        print(f"Executing SOFT blocking action: {action_type}")
                        self.execute_action(action_id, action_type, parameters, action_description, blocking=False)

        except json.JSONDecodeError:
            print("Failed to decode order message")


    def execute_action(self, action_id, action_type, parameters, action_description, blocking=False):
        """ Execute a given action """
        try:
            # Check if the actionId already exists in action_states
            if any(action["actionId"] == action_id for action in self.action_states):
                print(f"Action with ID {action_id} already exists. Skipping.")
                return

            if blocking:
                print(f"Blocking execution for action {action_type}.")
                self.instant_action = True  # Pause motion simulation
                self.handle_instant_action(action_id, action_type, parameters, action_description)
            else:
                print(f"Executing non-blocking action {action_type}.")
                # Perform the non-blocking action
                self.action_states.append({
                    "actionId": action_id,
                    "actionType": action_type,
                    "actionDescription": action_description,
                    "actionStatus": "FINISHED",
                    "resultDescription": f"Action {action_type} completed non-blocking."
                })
            # Simulate success
            # print(f"Action {action_type} executed successfully with parameters: {parameters}")
        except Exception as e:
            print(f"Error while executing action {action_type}: {str(e)}")


    def handle_instant_action(self, action_id, action_type, parameters, action_description, duration=2):
        """
        Handles the instant action, pauses motion simulation, and updates action states.
        """
        start_time = datetime.now()

        # Initialize actionStates based on the last visited node's actions, if any
        self.action_states.append({
            "actionId": action_id,
            "actionType": action_type,
            "actionDescription": action_description,
            "actionStatus": "INITIALIZING",
            "resultDescription": ""
        })

        while True:
            elapsed_time = (datetime.now() - start_time).total_seconds()

            if elapsed_time < duration / 2:
                # Transition to RUNNING state
                self.action_states[-1]["actionStatus"] = "RUNNING"
                self.action_states[-1]["resultDescription"] = f"Action {action_type} is in progress."

            elif elapsed_time >= duration:
                # Transition to FINISHED state and break the loop
                self.action_states[-1]["actionStatus"] = "FINISHED"
                self.action_states[-1]["resultDescription"] = f"Action {action_type} completed successfully."
                break

            # Publish state with updated action states
            self.publish_state()
            time.sleep(0.5)  # Slight delay for smoother state updates

        # Simulate success
        self.instant_action = False
        # print(f"Action {action_type} executed successfully with parameters: {parameters}")


    # (Optional: remove or comment out the old fixed timer method)
    def publish_msg(self):
        """publish all messages."""
        print(" - ", self.robot_serial_number, " - ")
        self.publish_factsheet()
        self.publish_connection()
        self.publish_state()


    def dynamic_publish_loop(self):
        """Dynamic loop that publishes messages per type based on computed intervals."""
        while True:

            print(" - ", self.robot_serial_number, " - ")

            now = time.time()
            state_interval = self.compute_dynamic_interval("state")
            conn_interval = self.compute_dynamic_interval("connection")
            fact_interval = self.compute_dynamic_interval("factsheet")

            # Publish state if its interval has elapsed
            if now - self.last_state_publish >= state_interval:
                self.publish_state()
                self.last_state_publish = now

            # Publish connection if its interval has elapsed
            if now - self.last_connection_publish >= conn_interval:
                self.publish_connection()
                self.last_connection_publish = now

            # Publish factsheet if its interval has elapsed
            if now - self.last_factsheet_publish >= fact_interval:
                self.publish_factsheet()
                self.last_factsheet_publish = now

            time.sleep(0.1)

    def compute_dynamic_interval(self, message_type):
        """
        Compute a dynamic interval (in seconds) for a given message type.
          - 'state'   : factors in active task (distance, wait time, battery, message size)
          - 'connection': when busy, less frequent (base ~3 sec); when idle, more frequent (base ~1 sec)
          - 'factsheet' : when busy, less frequent (base ~10 sec); when idle, more frequent (base ~2 sec)
        """
        if message_type == "state":
            min_interval = 0.5
            max_interval = 5.0
            active_interval = 1.0
            idle_interval = 3.0 # 5.0
            distance_threshold = 1.0 # 5.0[m] # meters
            waiting_multiplier = 1.5
            battery_low_threshold = 20.0  # percentage
            max_allowed_msg_size_kb = 1.0

            # If no task (or task completed: nodes empty), use idle interval.
            if not self.order or not self.order.get("nodes"):
                return idle_interval

            interval = active_interval

            # Distance factor (if a target exists)
            if self.target_node and self.target_node.get("nodePosition"):
                target = self.target_node["nodePosition"]
                dx = target["x"] - self.position[0]
                dy = target["y"] - self.position[1]
                distance = math.hypot(dx, dy)
                if distance < distance_threshold:
                    # Closer to target → state changes are smaller → slow down updates
                    distance_multiplier = 1 + (distance_threshold - distance) / distance_threshold
                else:
                    distance_multiplier = 1.0
                interval *= distance_multiplier

            # Waiting factor: if the robot is in a wait period at a node
            if self.wait_start_time is not None:
                node_desc = self.target_node.get("nodeDescription", "") if self.target_node else ""
                wait_time = self.extract_wait_time(node_desc)
                if wait_time is not None:
                    elapsed = time.time() - self.wait_start_time
                    if elapsed <= float(wait_time) / 10:
                        interval *= waiting_multiplier

            # Battery factor: if battery is low, update more frequently (reduce interval)
            battery_factor = 0.8 if self.battery_charge < battery_low_threshold else 1.0
            interval *= battery_factor

            # Message size factor: estimate state message size in kB and scale accordingly.
            state_msg = self.get_state_message()
            msg_json = json.dumps(state_msg)
            msg_size_kb = len(msg_json) / 1024.0
            size_factor = msg_size_kb / max_allowed_msg_size_kb
            interval *= (1 + size_factor)

            # Clamp the interval within allowed bounds.
            return max(min_interval, min(interval, max_interval))

        elif message_type == "connection":
            # When busy (i.e. task active) use a higher base interval,
            # but when idle, publish connection more frequently.
            if self.order and self.order.get("nodes"):
                base = 3.0
            else:
                base = 1.0
            battery_factor = 0.8 if self.battery_charge < 20.0 else 1.0
            interval = base * battery_factor
            return interval

        elif message_type == "factsheet":
            # Factsheet is less dynamic during active tasks, but more frequent when idle.
            if self.order and self.order.get("nodes"):
                base = 10.0
            else:
                base = 2.0
            battery_factor = 0.8 if self.battery_charge < 20.0 else 1.0
            interval = base * battery_factor
            return interval

        # Fallback: return a default interval.
        return 1.0


    def get_state_message(self):
        """Generate state message: full when active; trimmed when idle."""

        # build appropriate order message:
        # if self.order and self.order.get("nodes"):
        # Full state message when on a task
        state_message = {
            "headerId": 2,
            "version": self.version,
            "manufacturer": self.manufacturer,
            "serialNumber": self.robot_serial_number, # "SN12345678",
            "timestamp": datetime.now().isoformat(), # "2024-09-17T14:30:00Z",
            "maps": [
                {
                    "mapId": "map1",
                    "mapVersion": "1.0",
                    "mapDescription": "Main warehouse map",
                    "mapStatus": "ENABLED"
                }
            ],
            "orderId": self.order["orderId"] if self.order else "", # "orderId": "order_1234",
            "orderUpdateId": self.order["orderUpdateId"] if self.order else 1,
            "zoneSetId": self.fleetname,
            # if robot has a last successful node, then 'robot1_reserved_node'. and it automatically implies that, its released or reserved "true" state.
            # if robot has no last visited node, then ''.
            "lastNodeId": self.last_node_id, # self.target_node["nodeId"] if self.target_node else "", # robot1_reserved_node,
            "lastNodeSequenceId": 1,
            "driving": self.driving,
            "paused": not self.driving,
            "newBaseRequest": self.newbaserequest,
            "distanceSinceLastNode": 1.0,
            "operatingMode": "AUTOMATIC",
            "nodeStates": self.order["nodes"] if self.order else [],
            "edgeStates": self.order["edges"] if self.order else [],
            "agvPosition": {
                "x": self.position[0],
                "y": self.position[1],
                "theta": self.position[2],
                "mapId": "map1",
                "positionInitialized": True
            },
            "velocity": {
                "vx": self.lin_velocity if self.driving else 0.0,
                "omega": self.ang_velocity if self.driving else 0.0
            },
            "batteryState": {
                "batteryCharge": self.battery_charge,  # Random battery charge
                "charging": False
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
            "actionStates": self.action_states,  # Include populated action states here
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
        return state_message


    def publish_state(self):
        """Publish the state message."""
        # If a task is active and not in an instant action, simulate motion.
        if self.order and not self.instant_action:
            # Simulate motion if there is an order
            self.simulate_motion()
        # build the appropriate state message
        state_message = self.get_state_message()
        # declare the topic name
        topic = f"{self.fleetname}/{self.versions}/{self.manufacturer}/{self.robot_serial_number}/state"
        # Publish state message structure that will be validated, inserted into the database, ...
        self.mqtt_client.publish(topic, json.dumps(state_message))
        # Uncomment for debugging:
        # print(f"Published state for {self.robot_serial_number}: {state_message}")


    def publish_connection(self):
        """Publish the connection message."""
        connection_message = {
            "headerId": 1,
            "timestamp": datetime.now().isoformat(),
            "version": self.version,
            "manufacturer": self.manufacturer,
            "serialNumber": self.robot_serial_number,
            "connectionState": self.connection_state # ['ONLINE', 'OFFLINE', 'CONNECTIONBROKEN']
        }
        topic = f"{self.fleetname}/{self.versions}/{self.manufacturer}/{self.robot_serial_number}/connection"
        self.mqtt_client.publish(topic, json.dumps(connection_message))
        # Uncomment for debugging:
        # print(f"Published connection for {self.robot_serial_number}: {connection_message}")


    def publish_factsheet(self):
        """Publish the factsheet message."""
        factsheet_message = {
            "headerId": 1,
            "timestamp":  datetime.now().isoformat(), # "2024-09-15T12:00:00Z",
            "version": self.version,
            "manufacturer": self.manufacturer,
            "serialNumber": self.robot_serial_number,
            "typeSpecification": {
                "seriesName": "Series A",
                "agvKinematic": "DifferentialDrive",
                "agvClass": "Class 1",
                "maxLoadMass": 500,
                "localizationTypes": ["GPS", "LIDAR"],
                "navigationTypes": ["SLAM", "ODOMETRY"]
            },
            "physicalParameters": {
                "speedMin": self.lin_velocity,
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
                "versions": [{"fleetName" : self.fleetname}], # "versions": [{"key": "fleetName", "value": self.fleetname}],
                "config1": "value1",
                "config2": "value2"
            }
        }
        topic = f"{self.fleetname}/{self.versions}/{self.manufacturer}/{self.robot_serial_number}/factsheet"
        self.mqtt_client.publish(topic, json.dumps(factsheet_message))
        # Uncomment for debugging:
        # print(f"Published factsheet for {self.robot_serial_number}: {factsheet_message}")


    def extract_wait_time(self, node_description):
        """Extract wait time from node description (if provided)."""
        match = re.search(r"Wait Time: ([\d.]+)", node_description)
        return float(match.group(1)) if match else None


    def update_battery(self):
        """Update the battery level based on activity and time elapsed."""
        current_time = time.time()
        elapsed_time = current_time - self.last_battery_update_time
        self.last_battery_update_time = current_time

        if self.order and self.order.get("nodes"):
            # Active task: deplete battery faster
            depletion = self.depletion_rate_active * elapsed_time
        else:
            # Idle: deplete battery slower
            depletion = self.depletion_rate_idle * elapsed_time

        self.battery_charge = max(0.0, self.battery_charge - depletion)
        print(f"Battery charge updated to: {self.battery_charge:.2f}%")


    def simulate_motion(self):
        """Simulate motion towards the current target node."""

        # Initialize actionStates based on the last visited node's actions, if any
        if not self.order or not self.target_node or not self.target_node.get("released", False):
            return

        self.newbaserequest = False
        self.last_node_id = self.target_node["nodeId"]

        target_x = self.target_node["nodePosition"]["x"]
        target_y = self.target_node["nodePosition"]["y"]
        target_theta = self.target_node["nodePosition"]["theta"]

        # Extract wait time from nodeDescription if present
        node_description = self.target_node.get("nodeDescription", "")
        wait_time = self.extract_wait_time(node_description)

        # Check if we have a wait time and manage waiting logic
        if wait_time is not None:
            if self.wait_start_time is None:
                self.wait_start_time = time.time()
            elif (time.time() - self.wait_start_time) <= float(wait_time): # /10
                # Continue waiting until wait time has passed
                print(f"Robot {self.robot_serial_number} is waiting at node {self.target_node['nodeId']}")
                return
            # elif (time.time() - self.wait_start_time) > float(wait_time)/10:
            #    print("Robot wait completed...")
            else:
                # Wait time has passed; reset wait_start_time and proceed
                print("Robot wait completed...")
                self.wait_start_time = None

        # Calculate differences in position and orientation
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        distance_to_target = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = angle_to_target - self.position[2]

        # Update position only if distance is greater than threshold
        if distance_to_target > 0.05:
            # Calculate step movement based on linear velocity
            step_distance = min(self.lin_velocity, distance_to_target)
            self.position[0] += step_distance * math.cos(angle_to_target)
            self.position[1] += step_distance * math.sin(angle_to_target)

            # Update angular position based on angular velocity
            step_angle = min(self.ang_velocity, abs(angle_diff)) * (1 if angle_diff > 0 else -1)
            self.position[2] += step_angle
            self.position[2] = self.position[2] % (2 * math.pi)  # Normalize angle within 0-2pi

            print(f"Robot {self.robot_serial_number}: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}] moving to node {self.target_node['nodeId']} at nodeposition {self.target_node['nodePosition']}")

        # If close to the target, snap to target position and update order
        if distance_to_target <= 0.05:
            self.position = [target_x, target_y, target_theta]
            print(f"Robot {self.robot_serial_number} reached target node {self.target_node['nodeId']}")

            # Remove the completed node and edge, if there are any left
            nodes = self.order.get("nodes", [])
            print(" nodes: ", nodes)
            if nodes:
                nodes.pop(0)
                self.order["nodes"] = nodes

            if "edges" in self.order and self.order["edges"]:
                edges = self.order["edges"]
                edges.pop(0)
                self.order["edges"] = edges

            next_node = self.get_next_node()
            if next_node:
                self.target_node = next_node
                if self.target_node.get("released", False) is False:
                    self.newbaserequest = True
            else:
                # add node action here.
                if self.target_node and "actions" in self.target_node and self.target_node["actions"]:
                    # Populate actionStates from actions in the last node
                    for action in self.target_node["actions"]:
                        action_id = action.get("actionId", "")

                        # Check if actionId already exists in action_states
                        if not any(existing_action.get("actionId") == action_id for existing_action in self.action_states):
                            self.action_states.append({
                                "actionId": action_id,
                                "actionType": action.get("actionType", ""),
                                "actionDescription": action.get("actionDescription", ""),
                                "actionStatus": "FINISHED",  # Default status; you may dynamically set this based on state
                                "resultDescription": str(action.get("actionParameters", ""))  # Modify based on actual status/result
                            })
                self.driving = False
                self.target_node = None
                print(f"Robot {self.robot_serial_number} completed its task.")

        # Simulate time passage
        time.sleep(0.1)

    def get_next_node(self):
        """Fetches the next node to move towards from the order's node sequence."""
        for node in self.order.get("nodes", []):
            return node
        return None

    def create_timer(self, interval, function):
        """Create a repeating timer that executes a function at a regular interval."""
        def timer_thread():
            while True:
                function()
                time.sleep(interval)
        # Create a separate thread for each timer
        threading.Thread(target=timer_thread, daemon=True).start()


def main():
    """Main function to run multiple robot simulators."""

    robots = []

    robot_configurations = [
        # big map
        # 1 - 4
        # # c27
        # {"fleetname": "kullar", "robot_serial_number": "R01", "versions": "v2", "version": "2.0.0", "manufacturer": "birfen",
        #  "connection_state": "ONLINE", "initial_position": [-3.0, 6.0, 0.9], "bat_charge":85.0, "lin_velocity":0.65, "ang_velocity":0.15},
        # # c26
        # {"fleetname": "kullar", "robot_serial_number": "R02", "versions": "v2", "version": "2.0.0", "manufacturer": "birfen",
        #  "connection_state": "ONLINE", "initial_position": [-3.0, 3.0, 0.9], "bat_charge":75.0, "lin_velocity":0.65, "ang_velocity":0.15},
        # # c25
        # {"fleetname": "kullar", "robot_serial_number": "R03", "versions": "v2", "version": "2.0.0", "manufacturer": "birfen",
        #  "connection_state": "ONLINE", "initial_position": [-3.0, 0.0, 0.9], "bat_charge":95.0, "lin_velocity":0.65, "ang_velocity":0.15},
        # c28
        {"fleetname": "kullar", "robot_serial_number": "R04", "versions": "v2", "version": "2.0.0", "manufacturer": "birfen",
          "connection_state": "ONLINE", "initial_position": [-3.0, 9.0, 0.9], "bat_charge":90.0, "lin_velocity":0.65, "ang_velocity":0.15},
        # # 5 - 8
        # # c29
        # {"fleetname": "kullar", "robot_serial_number": "R05", "versions": "v2", "version": "2.0.0", "manufacturer": "birfen",
        #  "connection_state": "ONLINE", "initial_position": [15.0, 3.0, 0.0], "bat_charge":75.0, "lin_velocity":0.65, "ang_velocity":0.15},
        # # c30
        # {"fleetname": "kullar", "robot_serial_number": "R06", "versions": "v2", "version": "2.0.0", "manufacturer": "birfen",
        #  "connection_state": "ONLINE", "initial_position": [15.0, 6.0, 0.0], "bat_charge":85.0, "lin_velocity":0.65, "ang_velocity":0.15},
        # # c31
        # {"fleetname": "kullar", "robot_serial_number": "R07", "versions": "v2", "version": "2.0.0", "manufacturer": "birfen",
        #  "connection_state": "ONLINE", "initial_position": [15.0, 0.0, 0.0], "bat_charge":95.0, "lin_velocity":0.65, "ang_velocity":0.15},
        # # c32
        # {"fleetname": "kullar", "robot_serial_number": "R08", "versions": "v2", "version": "2.0.0", "manufacturer": "birfen",
        #   "connection_state": "ONLINE", "initial_position": [15.0, 9.0, 0.0], "bat_charge":90.0, "lin_velocity":0.65, "ang_velocity":0.15},

        # turtle world
        # {"fleetname": "kullar", "robot_serial_number": "R02", "versions": "v2", "version": "2.0.0", "manufacturer": "birfen",
        # "connection_state": "ONLINE", "initial_position": [-1.45, -1.1, 1.57], "bat_charge":75.0, "lin_velocity":0.25, "ang_velocity":0.15},
        # {"fleetname": "kullar", "robot_serial_number": "R01", "versions": "v2", "version": "2.0.0", "manufacturer": "birfen",
        # "connection_state": "ONLINE", "initial_position": [-1.7, 0.63, 1.57], "bat_charge":85.0, "lin_velocity":0.25, "ang_velocity":0.15},

        # Add more robot configurations here if needed. #--->  "diffamr2" | "robovak"
    ]

    # Typical Range: 0.5 m/s to 2.0 m/s - linear. | Typical Range: 0.2 rad/s to 1.0 rad/s - angular
    # Initialize and start robots
    for config in robot_configurations:
        robot = Robot(**config)
        robots.append(robot)

    # Let robots run concurrently
    try:
        while True:
            print(" - ")
            time.sleep(1)  # Keep the main thread alive.
    except KeyboardInterrupt:
        print("Program interrupted. Exiting...")
        for robot in robots:
            # Clean up MQTT connections when exiting
            robot.mqtt_client.disconnect()

if __name__ == "__main__":
    main()



















# """

# mosquitto_pub -h localhost -p 1883 -t kullar/v2/birfen/diffamr2/order -m '
# {
#     "headerId": 1,
#     "orderId": "1234",
#     "orderUpdateId": 0,
#     "version": "2.0.0",
#     "manufacturer": "robots",
#     "serialNumber": "diffamr2",
#     "nodes": [
#         {
#             "nodeId": "node1",
#             "released": true,
#             "sequenceId": 1,
#             "nodePosition": {
#                 "x": 0.5,
#                 "y": 1.7,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         },
#         {
#             "nodeId": "node1",
#             "released": true,
#             "sequenceId": 3,
#             "nodePosition": {
#                 "x": 0.5,
#                 "y": 1.7,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         }
#     ],
#     "edges": [
#         {
#             "edgeId": "edge0",
#             "released": true,
#             "sequenceId": 2,
#             "startNodeId": "node1",
#             "endNodeId": "node1",
#             "actions": []
#         }
#     ]
# }'

# """



# """

# mosquitto_pub -h localhost -p 1883 -t kullar/v2/birfen/AGV-001/state -m '
# {
#     "headerId": 1,
#     "orderId": "1234",
#     "orderUpdateId": 0,
#     "version": "2.0.0",
#     "manufacturer": "robots",
#     "serialNumber": "AGV-001",
#     "nodes": [
#         {
#             "nodeId": "node1",
#             "released": true,
#             "sequenceId": 1,
#             "nodePosition": {
#                 "x": 0.5,
#                 "y": 1.7,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         },
#         {
#             "nodeId": "node1",
#             "released": true,
#             "sequenceId": 3,
#             "nodePosition": {
#                 "x": 0.5,
#                 "y": 1.7,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         }
#     ],
#     "edges": [
#         {
#             "edgeId": "edge0",
#             "released": true,
#             "sequenceId": 2,
#             "startNodeId": "node1",
#             "endNodeId": "node1",
#             "actions": []
#         }
#     ]
# }'

# """







# USAGE:
# Terminal 1: simulation
# export DOCKERKEY_PERM=tskey-auth-k6pMCP2tNB11CNTRL-PYNihUnnZZeHfD6Jza8xZePyA8QXJbayg
# cd docker_ws/env/dev3
# export DISPLAY=:0.0
# xhost +local:docker
# docker system prune
# docker compose up --build


# Terminal 2: vda5050
# docker exec -it dev3-tailscaled-1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; export TURTLEBOT3_MODEL=waffle;
# ros2 launch vda5050_tb3_adapter connector_tb3.launch.py


# Terminal 3:
# docker exec -it dev3-tailscaled-1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; export TURTLEBOT3_MODEL=waffle;

# Terminal 4:
# docker exec -it dev3-tailscaled-1 bash
# cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; export TURTLEBOT3_MODEL=waffle;
# ros2 topic echo /kullar/v1/OSRF/TB3_1/state



# ## --- go to node1. notice the exact copy of node1 as node0 else it wont work

# mosquitto_pub -h localhost -p 1883 -t kullar/v1/OSRF/TB3_1/order -m '
# {
#     "orderId": "1234",
#     "orderUpdateId": 0,
#     "version": "2.0.0",
#     "manufacturer": "robots",
#     "serialNumber": "diffamr2",
#     "nodes": [
#         {
#             "nodeId": "node0",
#             "released": true,
#             "sequenceId": 1,
#             "nodePosition": {
#                 "x": 0.5,
#                 "y": 1.7,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         },
#         {
#             "nodeId": "node1",
#             "released": true,
#             "sequenceId": 3,
#             "nodePosition": {
#                 "x": 0.5,
#                 "y": 1.7,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         },
#         {
#             "nodeId": "node2",
#             "released": false,
#             "sequenceId": 5,
#             "nodePosition": {
#                 "x": 0.5,
#                 "y": -1.8,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         }
#     ],
#     "edges": [
#         {
#             "edgeId": "edge0",
#             "released": true,
#             "sequenceId": 2,
#             "startNodeId": "node0",
#             "endNodeId": "node1",
#             "actions": []
#         },
#         {
#             "edgeId": "edge1",
#             "released": false,
#             "sequenceId": 4,
#             "startNodeId": "node1",
#             "endNodeId": "node2",
#             "actions": []
#         }
#     ]
# }'

# ### ---- goes to node 2.

# mosquitto_pub -h localhost -p 1883 -t kullar/v1/OSRF/TB3_1/order -m '
# {
#     "orderId": "1234",
#     "orderUpdateId": 1,
#     "version": "2.0.0",
#     "manufacturer": "robots",
#     "serialNumber": "diffamr2",
#     "nodes": [
#         {
#             "nodeId": "node2",
#             "released": true,
#             "sequenceId": 5,
#             "nodePosition": {
#                 "x": 0.5,
#                 "y": -1.8,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         },
#         {
#             "nodeId": "node3",
#             "released": false,
#             "sequenceId": 7,
#             "nodePosition": {
#                 "x": -1.7,
#                 "y": 0.5,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         }
#     ],
#     "edges": [
#         {
#             "edgeId": "edge1",
#             "released": true,
#             "sequenceId": 4,
#             "startNodeId": "node1",
#             "endNodeId": "node2",
#             "actions": []
#         },
#         {
#             "edgeId": "edge2",
#             "released": false,
#             "sequenceId": 6,
#             "startNodeId": "node2",
#             "endNodeId": "node3",
#             "actions": []
#         }
#     ]
# }'

# ### ---- goes to node 3.

# mosquitto_pub -h localhost -p 1883 -t kullar/v1/OSRF/TB3_1/order -m '
# {
#     "orderId": "1234",
#     "orderUpdateId": 2,
#     "version": "2.0.0",
#     "manufacturer": "robots",
#     "serialNumber": "diffamr2",
#     "nodes": [
#         {
#             "nodeId": "node3",
#             "released": true,
#             "sequenceId": 7,
#             "nodePosition": {
#                 "x": -1.7,
#                 "y": 0.5,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         },
#         {
#             "nodeId": "node4",
#             "released": false,
#             "sequenceId": 9,
#             "nodePosition": {
#                 "x": -1.5,
#                 "y": -1.3,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         }
#     ],
#     "edges": [
#         {
#             "edgeId": "edge2",
#             "released": true,
#             "sequenceId": 6,
#             "startNodeId": "node2",
#             "endNodeId": "node3",
#             "actions": []
#         },
#         {
#             "edgeId": "edge3",
#             "released": false,
#             "sequenceId": 8,
#             "startNodeId": "node3",
#             "endNodeId": "node4",
#             "actions": []
#         }
#     ]
# }'


# ### ---- goes to node 4. and yields finished.

# mosquitto_pub -h localhost -p 1883 -t kullar/v1/OSRF/TB3_1/order -m '
# {
#     "orderId": "1234",
#     "orderUpdateId": 3,
#     "version": "2.0.0",
#     "manufacturer": "robots",
#     "serialNumber": "diffamr2",
#     "nodes": [
#         {
#             "nodeId": "node4",
#             "released": true,
#             "sequenceId": 9,
#             "nodePosition": {
#                 "x": -1.5,
#                 "y": -1.3,
#                 "theta": 0.0,
#                 "mapId": "map"
#             },
#             "actions": []
#         }
#     ],
#     "edges": [
#         {
#             "edgeId": "edge3",
#             "released": true,
#             "sequenceId": 8,
#             "startNodeId": "node3",
#             "endNodeId": "node4",
#             "actions": []
#         }
#     ]
# }'
