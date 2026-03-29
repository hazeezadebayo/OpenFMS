# submodules/zone_set.py
import json
import os
import logging
import datetime
import sys
from typing import List, Dict, Any

import psycopg2
from psycopg2 import sql
from jsonschema import validate, ValidationError

import numpy as np
from sklearn.cluster import KMeans
from shapely.geometry import Point, Polygon, MultiPoint
from shapely.ops import unary_union


# --------------------------------------------------------------------------- #
#                           ZoneSetPublisher                                 #
# --------------------------------------------------------------------------- #
class ZoneSetPublisher:
    """Handles VDA-5050 zoneSet messages – validation, DB storage & MQTT publish."""

    def __init__(
        self,
        fleetname: str,
        versions: str,
        dbconn,
        mqttclient=None,
        drop_table: bool = False,
        dbname: str = "postgres",
        tname: str = "zone_sets",
        output_log: bool = True,
    ):
        self.fleetname = fleetname
        self.versions = versions
        self.db_conn = dbconn
        self.mqtt_client = mqttclient
        self.table_zone_sets = tname
        self.drop_table = drop_table
        self.max_entry_per_robot = 20
        self.output_to_screen = output_log

        # ------------------- Load & resolve JSON schema -------------------
        schema_path = os.path.join(os.path.dirname(__file__), "schemas", "zoneSet.schema")
        with open(schema_path, "r", encoding="utf-8") as schema_file:
            self.zoneset_schema = json.load(schema_file)

        # ------------------- Logger -------------------
        self.logger = self._get_logger("ZoneSetPublisher", output_log)

        # ------------------- DB init -------------------
        self.create_database(dbname)

    # --------------------------------------------------------------------- #
    # -------------------------- LOGGING --------------------------------- #
    # --------------------------------------------------------------------- #
    def _get_logger(self, logger_name: str, output_log: bool):
        logger = logging.getLogger(logger_name)
        logger.setLevel(logging.INFO)

        if not logger.hasHandlers() and output_log:
            log_file_path = os.path.abspath("FmLogHandler.log")
            file_handler = logging.FileHandler(log_file_path, mode="a")
            file_handler.setFormatter(
                logging.Formatter("[%(levelname)s] [%(asctime)s] %(name)s: %(message)s")
            )
            logger.addHandler(file_handler)

            stream_handler = logging.StreamHandler(sys.stdout)
            stream_handler.setFormatter(
                logging.Formatter("[%(levelname)s] [%(asctime)s] %(name)s: %(message)s")
            )
            logger.addHandler(stream_handler)

        return logger

    # --------------------------------------------------------------------- #
    # -------------------------- DB LIFECYCLE ---------------------------- #
    # --------------------------------------------------------------------- #
    def close(self):
        """Close logger handlers."""
        for handler in self.logger.handlers[:]:
            handler.close()
            self.logger.removeHandler(handler)

    def create_database(self, dbname: str):
        cursor = self.db_conn.cursor()
        cursor.execute(f"SELECT 1 FROM pg_database WHERE datname = '{dbname}';")
        if not cursor.fetchone():
            cursor.execute(sql.SQL("CREATE DATABASE {}").format(sql.Identifier(dbname)))
            self.db_conn.commit()
            self.logger.info("Database '%s' created.", dbname)

        if self.drop_table:
            self.drop_zone_set_table()
        self.create_zone_set_table()

    def drop_zone_set_table(self):
        try:
            cursor = self.db_conn.cursor()
            cursor.execute("DROP TABLE IF EXISTS " + self.table_zone_sets)
            self.db_conn.commit()
            self.logger.info("%s table dropped.", self.table_zone_sets)
        except Exception as e:
            self.logger.error("Error dropping %s: %s", self.table_zone_sets, e)

    def create_zone_set_table(self):
        """Create the zone_sets table – matches zoneSet schema."""
        cursor = self.db_conn.cursor()
        cursor.execute(
            """
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = %s
            );
            """,
            (self.table_zone_sets,),
        )
        exists = cursor.fetchone()[0]

        if not exists:
            cursor.execute(
                f"""
                CREATE TABLE {self.table_zone_sets} (
                    id SERIAL PRIMARY KEY,
                    header_id INTEGER NOT NULL,
                    timestamp TIMESTAMP NOT NULL,
                    version VARCHAR(50) NOT NULL,
                    manufacturer VARCHAR(100) NOT NULL,
                    serial_number VARCHAR(100) NOT NULL,
                    map_id VARCHAR(100) NOT NULL,
                    zone_set_id VARCHAR(100) NOT NULL,
                    zone_set_description TEXT,
                    zones JSONB NOT NULL
                );
                """
            )
            self.db_conn.commit()
            self.logger.info("zone_sets table created.")
        else:
            self.logger.info("zone_sets table already exists.")

    # --------------------------------------------------------------------- #
    # -------------------------- VALIDATION ------------------------------ #
    # --------------------------------------------------------------------- #
    def validate_message(self, message):
        """Validate a message against the JSON schema."""
        try:
            validate(instance=message, schema=self.zoneset_schema)
        except ValidationError as er:
            self.logger.error("ZoneSet schema validation failed: %s", er.message)
            raise

    # --------------------------------------------------------------------- #
    # -------------------------- MESSAGE BUILD --------------------------- #
    # --------------------------------------------------------------------- #
    def build_zone_set_msg(
        self,
        f_id: str,
        _r_id: str,
        header_id: int,
        version: str,
        manufacturer: str,
        map_id: str,
        zones: List[Dict[str, Any]],
        zone_set_description: str = None,
    ) -> None:
        """
        Build, validate, store and (optionally) publish a VDA-5050 zoneSet message.
        """
        zone_set_message = {
            "headerId": header_id,
            "timestamp": datetime.datetime.now().isoformat(),
            "version": version,
            "manufacturer": manufacturer,
            "serialNumber": _r_id,
            "mapId": map_id,
            "zoneSetId": f_id,
            "zones": zones,
        }

        if zone_set_description:
            zone_set_message["zoneSetDescription"] = zone_set_description

        # ---- Validate before DB/MQTT ----
        try:
            self.validate_message(zone_set_message)
        except ValidationError:
            self.logger.error(
                "zoneSet for robot %s (headerId=%s) failed validation – NOT stored/published.",
                _r_id,
                header_id,
            )
            return

        # ---- DB storage ----
        self.insert_zone_set_db(zone_set_message)

        # ---- MQTT publish (optional) ----
        if self.mqtt_client is not None:
            self.pub_zone_set_mqtt(self.mqtt_client, zone_set_message)

    # --------------------------------------------------------------------- #
    # -------------------------- DB INSERT ------------------------------- #
    # --------------------------------------------------------------------- #
    def insert_zone_set_db(self, zone_set_message: Dict[str, Any]) -> None:
        """Insert validated zoneSet; keep only newest N entries per robot."""
        try:
            cursor = self.db_conn.cursor()
            cursor.execute(
                f"""
                INSERT INTO {self.table_zone_sets}
                (header_id, timestamp, version, manufacturer, serial_number,
                 map_id, zone_set_id, zone_set_description, zones)
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s);
                """,
                (
                    zone_set_message["headerId"],
                    zone_set_message["timestamp"],
                    zone_set_message["version"],
                    zone_set_message["manufacturer"],
                    zone_set_message["serialNumber"],
                    zone_set_message["mapId"],
                    zone_set_message["zoneSetId"],
                    zone_set_message.get("zoneSetDescription"),
                    json.dumps(zone_set_message["zones"]),
                ),
            )
            self.db_conn.commit()

            # ---- enforce max_entry_per_robot ----
            cursor.execute(
                f"""
                SELECT COUNT(*) FROM {self.table_zone_sets}
                WHERE serial_number = %s AND manufacturer = %s;
                """,
                (zone_set_message["serialNumber"], zone_set_message["manufacturer"]),
            )
            entry_count = cursor.fetchone()[0]

            if entry_count > self.max_entry_per_robot:
                cursor.execute(
                    f"""
                    DELETE FROM {self.table_zone_sets}
                    WHERE id IN (
                        SELECT id FROM {self.table_zone_sets}
                        WHERE serial_number = %s AND manufacturer = %s
                        ORDER BY id ASC LIMIT 1
                    );
                    """,
                    (zone_set_message["serialNumber"], zone_set_message["manufacturer"]),
                )
                self.db_conn.commit()

        except Exception as e:
            self.logger.error("DB insert error (zoneSet): %s", e)
            self.db_conn.rollback()

    # --------------------------------------------------------------------- #
    # -------------------------- MQTT PUBLISH ---------------------------- #
    # --------------------------------------------------------------------- #
    def pub_zone_set_mqtt(self, mqtt_client, zone_set_message: Dict[str, Any]) -> None:
        """Publish the validated zoneSet on the correct VDA-5050 topic."""
        payload = json.dumps(zone_set_message)
        topic = (
            f"{self.fleetname}/{self.versions}/"
            f"{zone_set_message['manufacturer']}/{zone_set_message['serialNumber']}/zoneSet"
        )
        mqtt_client.publish(topic, payload, qos=0, retain=False)
        self.logger.info(
            "zoneSet published (headerId=%s, robot=%s)",
            zone_set_message["headerId"],
            zone_set_message["serialNumber"],
        )

    # --------------------------------------------------------------------- #
    # -------------------------- FETCH HELPERS --------------------------- #
    # --------------------------------------------------------------------- #
    def fetch_data(self, zone_set_id: str, manufacturer: str):
        """
        Fetch and display all zones belonging to a given zone set.
        Args:
            zone_set_id (str): The unique ID of the zone set (e.g., 'zoneSet-01')
            manufacturer (str): The manufacturer name (e.g., 'birfen')
        Returns:
            dict or None: The zone set data if found, otherwise None
        """
        try:
            cursor = self.db_conn.cursor()
            cursor.execute(
                f"""
                SELECT * FROM {self.table_zone_sets}
                WHERE zone_set_id = %s
                AND manufacturer = %s
                ORDER BY timestamp DESC
                LIMIT 1;
                """,
                (zone_set_id, manufacturer),
            )
            row = cursor.fetchone()
            if not row:
                self.logger.warning("No zoneSet found for id=%s manufacturer=%s", zone_set_id, manufacturer)
                return None

            # Map columns to values
            cols = [desc[0] for desc in cursor.description]
            data = dict(zip(cols, row))

            # Only decode if necessary
            if isinstance(data["zones"], str):
                data["zones"] = json.loads(data["zones"])


            # ---- Print zone information ----
            print(f"\n--- ZoneSet Details ---")
            print(f"ZoneSet ID   : {data['zone_set_id']}")
            print(f"Manufacturer : {data['manufacturer']}")
            print(f"Serial Number: {data['serial_number']}")
            print(f"Map ID       : {data['map_id']}")
            print(f"Version      : {data['version']}")
            print(f"Timestamp    : {data['timestamp']}")
            if data.get("zone_set_description"):
                print(f"Description  : {data['zone_set_description']}")

            print("\nZones:")
            for z in data["zones"]:
                z_id = z.get("zoneId", "<unknown>")
                z_type = z.get("zoneType", "<unknown>")
                verts = z.get("vertices", [])
                print(f" - {z_id} ({z_type}) with {len(verts)} vertices")
                for i, v in enumerate(verts):
                    print(f"     Vertex {i+1}: ({v['x']:.2f}, {v['y']:.2f})")

                # Optional extras like maxSpeed, priorityFactor, etc.
                for k, v in z.items():
                    if k not in ["zoneId", "zoneType", "vertices"]:
                        print(f"     {k}: {v}")
                print()

            return data

        except Exception as e:
            self.logger.error("fetch_data error: %s", e)
            return None
        finally:
            cursor.close()







    def weighted_kmeans(self, nodes: Dict[str, tuple], traffic: Dict[str, str],
                        k: int, offset_m: float = 0.5) -> List[Dict]:
        """
        nodes   = {node_id: (x, y)}
        traffic = {node_id: robot_id}   # occupied nodes → higher weight
        k       = num_master_worker
        offset_m= extra buffer around convex hull
        """
        ids = list(nodes.keys())
        coords = np.array([nodes[nid] for nid in ids])

        # ---- weight vector ----
        weights = np.array([10.0 if nid in traffic else 1.0 for nid in ids])
        # repeat points according to weight (integer trick)
        X = []
        for i, pt in enumerate(coords):
            X.extend([pt] * int(weights[i]))
        X = np.array(X)

        kmeans = KMeans(n_clusters=k, random_state=42, n_init=10).fit(X)
        labels = kmeans.labels_[:len(ids)]  # truncate back

        zones = []
        for cluster in range(k):
            cluster_ids = [ids[i] for i, l in enumerate(labels) if l == cluster]
            if not cluster_ids:
                continue
            pts = [Point(nodes[nid]) for nid in cluster_ids]
            hull = MultiPoint(pts).convex_hull
            if hull.area == 0:
                hull = hull.buffer(0.01)

            # offset outward
            poly = hull.buffer(offset_m)
            vertices = [(round(p[0], 2), round(p[1], 2)) for p in poly.exterior.coords[:-1]]

            zones.append({
                "zoneId": f"master_{cluster+1}",
                "vertices": vertices,
                "masterHost": f"10.0.0.{10+cluster}",   # placeholder
                "mqttTopic": f"fleet/v2/birfen/+/state"
            })
        return zones


    # OPTION 1 – PostgreSQL Advisory Lock (recommended)
    def acquire_decision_lock(self) -> bool:
        """ Zero extra infra - Guarantees exactly one master runs manage_robot at a time. """
        cur = self.dbconn.cursor()
        cur.execute("SELECT pg_try_advisory_lock(1)")
        locked = cur.fetchone()[0]
        if locked:
            cur.execute(
                "UPDATE scheduler_lock SET holder_zone=%s, acquired_at=now() WHERE lock_id=1",
                (self.zone.zone_id,)
            )
        cur.close()
        return locked

    def release_decision_lock(self):
        cur = self.dbconn.cursor()
        cur.execute("SELECT pg_advisory_unlock(1)")
        cur.close()







# ------------------------------------------------------------------------- #
# ------------------------------- DEMO ------------------------------------ #
# ------------------------------------------------------------------------- #
if __name__ == "__main__":
    import random

    try:
        conn = psycopg2.connect(
            host="localhost",
            dbname="postgres",
            user="postgres",
            password="root",
            port="5432",
        )
        print("[INFO] Connected to PostgreSQL successfully.")
    except Exception as e:
        print("[ERROR] Database connection failed:", e)
        sys.exit(1)

    zone_pub = ZoneSetPublisher(
        fleetname="kullar",
        versions="v1",
        dbconn=conn,
        mqttclient=None,
        dbname="postgres",
        tname="zone_sets",
        output_log=True,
        drop_table=True,   # <-- force recreate with correct schema
    )

    def make_square(x0, y0, size):
        return [
            {"x": x0, "y": y0},
            {"x": x0 + size, "y": y0},
            {"x": x0 + size, "y": y0 + size},
            {"x": x0, "y": y0 + size},
        ]

    # sample_zones = [
    #     {"zoneId": "Z_BLOCKED", "zoneType": "BLOCKED", "vertices": make_square(0, 0, 5)},
    #     {"zoneId": "Z_SPEED", "zoneType": "SPEED_LIMIT", "vertices": make_square(6, 0, 4), "maxSpeed": 0.8},
    #     {"zoneId": "Z_PRIORITY", "zoneType": "PRIORITY", "vertices": make_square(0, 6, 4), "priorityFactor": 0.9},
    #     {"zoneId": "Z_PENALTY", "zoneType": "PENALTY", "vertices": make_square(10, 0, 3), "penaltyFactor": 0.7},
    #     {"zoneId": "Z_DIRECTED", "zoneType": "DIRECTED", "vertices": make_square(10, 6, 3), "direction": 1.57, "limitation": "STRICT"},
    # ]

    sample_zones = [
        {"zoneId": "192.168.1.10:1881", "zoneType": "BIDIRECTED", "vertices": make_square(0, 0, 5), "direction": 1.57, "limitation": "RESTRICTED"},
        {"zoneId": "192.168.1.11:1882", "zoneType": "SPEED_LIMIT", "vertices": make_square(6, 0, 4), "maxSpeed": 0.8},
        {"zoneId": "192.168.1.26:1883", "zoneType": "PRIORITY", "vertices": make_square(0, 6, 4), "priorityFactor": 0.9, "penaltyFactor": 0.7, "direction": 1.57, "limitation": "STRICT"},
    ]

    try:
        zone_pub.build_zone_set_msg(
            f_id="kullar",
            _r_id="AGV-001",
            header_id=random.randint(1, 999),
            version="2.0",
            manufacturer="birfen",
            map_id="map-001",
            zones=sample_zones,
            zone_set_description="Demo ZoneSet for testing",
        )
        print("[INFO] zoneSet message built and stored successfully.")
    except Exception as e:
        print("[ERROR] ZoneSet build failed:", e)
        sys.exit(1)

    latest = zone_pub.fetch_data("kullar", "birfen")

    if latest:
        ts = latest["timestamp"]
        print("\n--- Latest zoneSet for AGV-001 ---")
        print(f"Timestamp        : {ts}")
        print(f"zoneSetId        : {latest['zone_set_id']}")
        print(f"Map ID           : {latest['map_id']}")
        print(f"Manufacturer     : {latest['manufacturer']}")
        print(f"Serial Number    : {latest['serial_number']}")
        print(f"Version          : {latest['version']}")
        print("Zones:")
        for z in latest["zones"]:
            print(f"  - {z['zoneId']} ({z['zoneType']}) – {len(z['vertices'])} vertices")
    else:
        print("[WARN] No zoneSet found in database.")

    zone_pub.close()
    conn.close()
    print("\n[INFO] Database connection closed.")

    # TODO
    # build a zone set message in a way that publishes 0 for other masters if k is set to 1.

    # graph = schedule_handler.traffic_handler.task_handler.build_graph(task_dict)
    # traffic_control = schedule_handler.traffic_handler.get_current_occupancy()  # {node: robot}
    # zones = weighted_kmeans(graph['nodes'], traffic_control, num_master_worker, offset_m=0.8)

    # zoneset = {
    #     "headerId": 1,
    #     "timestamp": datetime.now().isoformat(),
    #     "manufacturer": "birfen",
    #     "version": "2.0",
    #     "zoneSetId": f"zoneSet-{int(time.time())}",
    #     "mapId": "map-001",
    #     "zones": zones
    # }
    # # write to DB + publish