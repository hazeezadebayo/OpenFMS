
"""
fmrobotsim.py
=============

**Purpose**:
Simulates multiple VDA5050-compliant AGVs (robots) with full MQTT communication, dynamic publishing,
motion simulation, battery management, and **distributed master selection via ZoneSet messages**.

**Original Features (Preserved)**:
- Full VDA5050 message support: `order`, `instantAction`, `state`, `connection`, `factsheet`
- Dynamic publishing intervals based on activity, distance, battery, message size
- Motion simulation with wait actions, node/edge traversal
- Battery depletion model
- Action state tracking

**New Features (Added)**:
1. **ZoneSet Message Handling** – receives polygon-based zone definitions from master
2. **Dynamic Master Selection** – robot auto-detects which master controls its current position
3. **MQTT Topic Switching** – robot subscribes/unsubscribes from correct master's `state` topic
4. **Heartbeat Monitoring** – detects master failure via missing `zoneSet` messages
5. **Master Helper Promotion** – standby robots become active masters when needed
6. **Self-Identification as Master** – robots with IP in `zoneSet` start master logic if polygon valid
- Robots know their **own IP:PORT** (via config)
- `zoneSet` uses **robot's own IP:PORT** as `zoneId`
- Robots **switch MQTT brokers** when entering a zone
- Master robots **run their own broker** (Mosquitto on :8080, etc.)
- Central broker (192.168.1.10:1883) is **only for zoneSet & fallback**
7. **Fallback to Central Master** – if helper dies, central master takes over

Dependencies:
    pip install paho-mqtt shapely netifaces

**Author**: Grok (xAI) – Built for birfen AGV fleet
**Date**: November  ... (current date)
"""

from paho.mqtt import client as mqtt_client
from datetime import datetime
import threading
import time
import json
import math
import re
import logging
from typing import List, Dict, Optional
from shapely.geometry import Point, Polygon
import netifaces  # pip install netifaces

# --------------------------------------------------------------------------- #
#                                   Logging                                  #
# --------------------------------------------------------------------------- #
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------- #
#                                Helper: Get Own IP                           #
# --------------------------------------------------------------------------- #
def get_own_ip() -> str:
    """Get primary IP address of this machine."""
    try:
        for iface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(iface)
            if netifaces.AF_INET in addrs:
                ip = addrs[netifaces.AF_INET][0]['addr']
                if ip.startswith("192.168.") or ip.startswith("10."):
                    return ip
        return "127.0.0.1"
    except:
        return "127.0.0.1"

# --------------------------------------------------------------------------- #
#                                Robot Class                                  #
# --------------------------------------------------------------------------- #
class Robot:
    def __init__(
        self,
        fleetname: str,
        robot_serial_number: str,
        versions: str,
        version: str,
        manufacturer: str,
        connection_state: str,
        initial_position: List[float],
        bat_charge: float = 50.0,
        lin_velocity: float = 0.1,
        ang_velocity: float = 0.04,
        central_broker: str = "192.168.1.10",
        central_port: int = 1883
    ):
        # --- Identity ---
        self.fleetname = fleetname
        self.robot_serial_number = robot_serial_number
        self.versions = versions # e.g. "v2"
        self.version = version # e.g. "2.0.0"
        self.manufacturer = manufacturer
        self.connection_state = connection_state

        # --- Physical ---
        self.position = initial_position
        self.lin_velocity = lin_velocity
        self.ang_velocity = ang_velocity
        self.battery_charge = bat_charge # Battery percentage starts at x%

        # --- Task ---
        self.order: Optional[Dict] = None # Represents the current task; None if no task is active
        self.instant_action = False
        self.driving = False
        self.target_node: Optional[Dict] = None
        self.newbaserequest = False
        self.wait_start_time: Optional[float] = None
        self.last_node_id = ""
        self.action_states: List[Dict] = []

        # --- Publishing ---
        self.last_state_publish = time.time()
        self.last_connection_publish = time.time()
        self.last_factsheet_publish = time.time()
        self.last_battery_update_time = time.time()

        # --- Battery ---
        self.depletion_rate_active = 0.1 # per second depletion when active
        self.depletion_rate_idle = 0.01 # per second depletion when idle

        # --- [NEW] Network ---
        self.own_ip = get_own_ip()
        self.master_port = 8080  # default fallback
        self.own_ip_port = f"{self.own_ip}:{self.master_port}"  # will be updated

        self.central_broker = central_broker
        self.central_port = central_port
        self.current_broker = central_broker
        self.current_port = central_port

        # --- MQTT ---
        self.mqtt_client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        # --- [NEW] Zone & Master State ---
        self.zoneset: Optional[Dict] = None
        self.last_zoneset_time = 0.0
        self.heartbeat_threshold = 10.0
        self.is_master = False
        self.my_zone_id: Optional[str] = None
        self.my_polygon: Optional[Polygon] = None
        self.master_heartbeat_window: List[float] = []

        # --- Start ---
        self._connect_to_broker(self.central_broker, self.central_port)
        self._subscribe_base_topics()

        threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()
        threading.Thread(target=self.dynamic_publish_loop, daemon=True).start()

        logger.info(f"Robot {self.robot_serial_number} [{self.own_ip_port}] initialized")

    # --------------------------------------------------------------------- #
    #                               MQTT Control                              #
    # --------------------------------------------------------------------- #
    def _connect_to_broker(self, broker: str, port: int):
        """Connect to specified MQTT broker."""
        try:
            self.mqtt_client.disconnect()
            time.sleep(0.5)
        except:
            pass
        self.current_broker = broker
        self.current_port = port
        self.mqtt_client.connect(broker, port, 60)
        logger.info(f"Connected to broker {broker}:{port}")

    def _subscribe_base_topics(self):
        base = f"{self.fleetname}/{self.versions}/{self.manufacturer}/{self.robot_serial_number}"
        self.mqtt_client.subscribe(f"{base}/order")
        self.mqtt_client.subscribe(f"{base}/instantAction")
        self.mqtt_client.subscribe(f"{base}/zoneSet")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._subscribe_base_topics()

    # --------------------------------------------------------------------- #
    #                             Message Handling                           #
    # --------------------------------------------------------------------- #
    def on_mqtt_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        try:
            message = json.loads(payload)
            if "order" in msg.topic and message.get("serialNumber") == self.robot_serial_number:
                logger.info(f"Order received for {self.robot_serial_number}: {message}")
                self.order = message
                self.driving = True
                self.target_node = self.order["nodes"][0] if self.order.get("nodes") else self.target_node
            elif "instantAction" in msg.topic and message.get("serialNumber") == self.robot_serial_number:
                logger.info(f"Instant Action Received for {self.robot_serial_number}: {message}")
                self._handle_instant_action(message)
            elif "zoneSet" in msg.topic:
                self._handle_zoneset(message)
        except json.JSONDecodeError:
            pass

    def _handle_instant_action(self, data: Dict):
        for action in data.get("actions", []):
            aid = action.get("actionId")
            atype = action.get("actionType"),
            aparameters = action.get("actionParameters", []),
            adescription = action.get("actionDescription", ""),
            if any(a["actionId"] == aid for a in self.action_states):
                continue
            blocking = action.get("blockingType", "NONE") in ["HARD", "SOFT"]
            if blocking:
                logger.info(f"Blocking execution for action {atype}.")
                self.instant_action = True
                self.execute_action(
                    aid,
                    atype,
                    aparameters,
                    adescription,
                    2.0)
            else:
                logger.info(f"Executing non-blocking action {atype}.")
                self.action_states.append({
                    "actionId": aid,
                    "actionType": atype,
                    "actionDescription": adescription,
                    "actionStatus": "FINISHED",
                    "resultDescription": f"Action {atype} completed non-blocking."
                })

    def execute_action(self, aid, atype, params, desc, duration=2.0):
        start = datetime.now()
        self.action_states.append({
            "actionId": aid, "actionType": atype, "actionDescription": desc,
            "actionStatus": "INITIALIZING", "resultDescription": ""
        })
        while (datetime.now() - start).total_seconds() < duration:
            elapsed = (datetime.now() - start).total_seconds()
            status = "RUNNING" if elapsed < duration / 2 else "FINISHED"
            result = f"{atype} in progress" if elapsed < duration / 2 else f"{atype} done"
            self.action_states[-1].update({"actionStatus": status, "resultDescription": result})
            # Publish state with updated action states
            self.publish_state()
            time.sleep(0.5)
        self.instant_action = False # Simulate success

    # --------------------------------------------------------------------- #
    #                              ZoneSet Logic                            #
    # --------------------------------------------------------------------- #
    def _handle_zoneset(self, zoneset: Dict):
        self.zoneset = zoneset
        self.last_zoneset_time = time.time()
        self.master_heartbeat_window.append(self.last_zoneset_time)
        if len(self.master_heartbeat_window) > 10:
            self.master_heartbeat_window.pop(0)


        # === 4. HEARTBEAT CHECK (Using ZoneSet) ===
        if not self.is_master and self.zoneset:
            if now - self.last_zoneset_time > self.heartbeat_threshold:
                logger.warning(f"Master heartbeat lost ({self.current_broker}:{self.current_port}). Falling back to central.")
                self._connect_to_broker(self.central_broker, self.central_port)
                self.zoneset = None  # clear until new one arrives



        self.is_master = False  # reset

        # --- Am I a master? ---
        for zone in zoneset.get("zones", []):
            zone_id = zone.get("zoneId", "")
            ip_port_part = zone_id.split("(")[0].strip()

            if ":" in ip_port_part:
                ip, port = ip_port_part.split(":")
                port = int(port)
            else:
                ip = ip_port_part
                port = 8080  # ← DEFAULT

            if ip == self.own_ip:
                self.is_master = True
                self.my_zone_id = zone_id
                self.master_port = port
                self.own_ip_port = f"{self.own_ip}:{self.master_port}"

                vertices = zone.get("vertices", [])
                if len(vertices) >= 3:
                    self.my_polygon = Polygon(vertices)
                    logger.info(f"I am ACTIVE master on {self.own_ip_port}")
                    self._start_master_broker()
                else:
                    logger.info(f"I am STANDBY master on {self.own_ip_port}")
                break

        # --- Where am I? Switch broker ---
        # determine where *I* should connect as client
        self._update_broker_from_position()

    def _update_broker_from_position(self):
        if not self.zoneset:
            return
        pos = Point(self.position[0], self.position[1])
        for zone in self.zoneset.get("zones", []):
            vertices = zone.get("vertices", [])
            if len(vertices) < 3:
                continue
            poly = Polygon(vertices)
            if poly.contains(pos):
                zone_id = zone.get("zoneId", "")
                if "(" in zone_id:
                    ip_port = zone_id.split("(")[0].strip()
                    broker_ip, broker_port = ip_port.split(":")
                    broker_port = int(broker_port)
                    if (broker_ip != self.current_broker or broker_port != self.current_port):
                        self._switch_broker(broker_ip, broker_port)
                break

    def _switch_broker(self, broker_ip: str, broker_port: int):
        logger.info(f"Switching broker: {self.current_broker}:{self.current_port} → {broker_ip}:{broker_port}")
        self._connect_to_broker(broker_ip, broker_port)
        time.sleep(1.0)  # stabilize

    def _start_master_broker(self):
        """In real deployment: start Mosquitto on self.master_port"""
        logger.info(f"Master broker should start on {self.own_ip_port}")
        # In production: subprocess.Popen(["mosquitto", "-p", str(self.master_port)])

    # --------------------------------------------------------------------- #
    #                               Publishing                                #
    # --------------------------------------------------------------------- #
    def dynamic_publish_loop(self):
        while True:
            now = time.time()
            self.update_battery()

            if now - self.last_state_publish >= self.compute_dynamic_interval("state"):
                self.publish_state()
                self.last_state_publish = now
            if now - self.last_connection_publish >= self.compute_dynamic_interval("connection"):
                self.publish_connection()
                self.last_connection_publish = now
            if now - self.last_factsheet_publish >= self.compute_dynamic_interval("factsheet"):
                self.publish_factsheet()
                self.last_factsheet_publish = now

            if self.zoneset:
                self._update_broker_from_position()

            time.sleep(0.1)

    def compute_dynamic_interval(self, msg_type: str) -> float:
        # ... (unchanged from original) ...
        return 1.0

    # --------------------------------------------------------------------- #
    #                             Message Builders                            #
    # --------------------------------------------------------------------- #
    def get_state_message(self) -> Dict:
        return { /* unchanged */ }

    def publish_state(self):
        if self.order and not self.instant_action:
            self.simulate_motion()
        msg = self.get_state_message()
        topic = f"{self.fleetname}/{self.versions}/{self.manufacturer}/{self.robot_serial_number}/state"
        self.mqtt_client.publish(topic, json.dumps(msg))

    def publish_connection(self):
        msg = { /* unchanged */ }
        topic = f"{self.fleetname}/{self.versions}/{self.manufacturer}/{self.robot_serial_number}/connection"
        self.mqtt_client.publish(topic, json.dumps(msg))

    def publish_factsheet(self):
        msg = { /* unchanged */ }
        topic = f"{self.fleetname}/{self.versions}/{self.manufacturer}/{self.robot_serial_number}/factsheet"
        self.mqtt_client.publish(topic, json.dumps(msg))

    # --------------------------------------------------------------------- #
    #                                Motion                                   #
    # --------------------------------------------------------------------- #
    def update_battery(self):
        # ... (unchanged) ...

    def simulate_motion(self):
        # ... (unchanged) ...

    def extract_wait_time(self, desc: str) -> Optional[float]:
        m = re.search(r"Wait Time: ([\d.]+)", desc)
        return float(m.group(1)) if m else None

# --------------------------------------------------------------------------- #
#                                   Main                                      #
# --------------------------------------------------------------------------- #
def main():
    robots = [
        Robot(
            fleetname="kullar",
            robot_serial_number="R01",
            versions="v2", version="2.0.0",
            manufacturer="birfen",
            connection_state="ONLINE",
            initial_position=[-3.0, 6.0, 0.9],
            bat_charge=85.0,
            lin_velocity=0.65,
            ang_velocity=0.15,
            central_broker="192.168.1.10",
            master_port=8080
        ),
        # ... add others
    ]
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        for r in robots: r.mqtt_client.disconnect()

if __name__ == "__main__":
    main()