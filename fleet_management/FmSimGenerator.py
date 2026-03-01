#!/usr/bin/env python3
"""
Grid-based fleet graph generator with strict dock entry rules + checkpoint pruning + full YAML exports
"""
import os
import random
import math
import yaml
import matplotlib.pyplot as plt
import matplotlib.collections as mc
import numpy as np
import re


RANDOM_SEED = 43
random.seed(RANDOM_SEED)


# ────────────────────────────────────────────────
# G L O B A L C O N F I G U R A T I O N
# ────────────────────────────────────────────────


FLEET_ID = "kullar"
MAP_ID = "bina_1_floor_0"
EDGE_MODE = "bidirectional" # currently only bidirectional implemented
ROBOT_VERSION = "2.0.0"
ROBOT_VERSIONS = "v2"
ROBOT_MANUFACTURER = "birfen"
ROBOT_CONNECTION_STATE = "ONLINE"
ROBOT_LIN_VEL = 0.65
ROBOT_ANG_VEL = 0.15

base_dir = os.path.dirname(os.path.abspath(__file__))
config_dir = os.path.join(base_dir, "..", "config")


# ────────────────────────────────────────────────
# N O D E C L A S S
# ────────────────────────────────────────────────


class Node:
    def __init__(self, loc_id: str, x: float, y: float, desc: str = "checkpoint"):
        self.loc_id = loc_id
        self.x = x
        self.y = y
        self.description = desc
        self.neighbors = set()

    def connect(self, other: 'Node'):
        if other.loc_id != self.loc_id:
            self.neighbors.add(other.loc_id)
            other.neighbors.add(self.loc_id)


# ────────────────────────────────────────────────
# G R I D F L E E T G R A P H C L A S S
# ────────────────────────────────────────────────


class GridFleetGraph:
    DOCK_TYPES = {"home_dock", "station_dock", "charge_dock"}

    def __init__(
        self,
        num_robots: int,
        num_station_docks: int,
        num_charge_docks: int,
        num_waitpoints: int = 10,
        density_factor: float = 3.0,
        output_dir: str = config_dir,
        custom_graph: dict = None,
        custom_chains: list[str] = None,
        edge_length: float = 6.0,
    ):
        self.n_robots = num_robots
        self.n_stations = num_station_docks
        self.n_charges = num_charge_docks if num_charge_docks > 0 else max(2, num_robots // 4)
        self.n_wait = num_waitpoints
        self.density_factor = density_factor
        self.out_dir = output_dir
        self.custom_graph = custom_graph
        self.custom_chains = custom_chains
        self.edge_length = edge_length
        self.nodes = {}
        self.home_nodes = []
        self.robots = []
        self.next_c_id = 1
        self.protected_cps = set() # checkpoints used as dock entries → protected from pruning
        os.makedirs(self.out_dir, exist_ok=True)

        if self.custom_chains is not None:
            self._build_from_chains()
        elif self.custom_graph is not None:
            self._build_from_custom_graph()
        else:
            # Auto-generation mode
            self._create_grid()
            self._rename_nodes_to_convention()
            self._assign_special_nodes()
            self._enforce_single_entry_docks()
            self._prune_isolated_checkpoints()
            self._prune_completely_isolated_checkpoints()
            self._verify_dock_entry_checkpoints()

        # Common steps
        self._add_waitpoints()
        self._assign_robots()
        # ── Export everything to one config.yaml + generate map ──
        self._export_full_config_yaml()
        self._export_robots_yaml() # keep separate if you want
        self.generate_map(filename=MAP_ID, resolution=0.05, padding=3.0)

    def _build_from_custom_graph(self):
        """
        Builds the graph from a custom dictionary input.
        Expected format: custom_graph = {
            'C1': {
                'description': 'home_dock',
                'location': '0,0',  # str 'x,y'
                'neighbors': 'C2,C3'  # str 'id1,id2' or list ['C2', 'C3']
            },
            ...
        }
        Node IDs are uppercased to match convention (e.g., 'c1' -> 'C1').
        Connections are made bidirectional.
        """
        for node_id, info in self.custom_graph.items():
            # Uppercase node_id to match 'C1' convention
            node_id = node_id.upper()
            desc = info['description'].strip()
            loc_str = info['location'].strip()
            x, y = map(float, loc_str.split(','))
            node = Node(node_id, x, y, desc)
            self.nodes[node_id] = node

        # Now connect them
        for node_id, info in self.custom_graph.items():
            node_id = node_id.upper()
            neighbors = info.get('neighbors', '')  # 'neighbour' or 'neighbors'
            if 'neighbour' in info:
                neighbors = info['neighbour']
            if isinstance(neighbors, str):
                nbs = [nb.strip().upper() for nb in neighbors.split(',') if nb.strip()]
            elif isinstance(neighbors, list):
                nbs = [nb.strip().upper() for nb in neighbors if nb.strip()]
            else:
                nbs = []
            for nb in nbs:
                if nb in self.nodes:
                    self.nodes[node_id].connect(self.nodes[nb])
                else:
                    print(f"Warning: Neighbor {nb} for {node_id} not found.")

        # Derive home nodes
        self.home_nodes = [node for node in self.nodes.values() if node.description == "home_dock"]

        # Update counts based on custom graph
        self.n_stations = sum(1 for node in self.nodes.values() if node.description == "station_dock")
        self.n_charges = sum(1 for node in self.nodes.values() if node.description == "charge_dock")

        # Validate num_robots
        if len(self.home_nodes) < self.n_robots:
            raise ValueError(f"Not enough home_docks ({len(self.home_nodes)}) for num_robots ({self.n_robots})")

        # Optionally, find max next_c_id for potential additions
        c_ids = [int(loc_id[1:]) for loc_id in self.nodes if loc_id.startswith('C') and loc_id[1:].isdigit()]
        self.next_c_id = max(c_ids) + 1 if c_ids else 1


    def _build_from_chains(self):
        self.nodes = {}
        self.home_nodes = []
        node_counter = {}
        created = {} # id → Node
        type_map = {
            'C': 'checkpoint',
            'H': 'home_dock',
            'S': 'station_dock',
            'CH': 'charge_dock',
            'W': 'waitpoint',
            'D': 'door',
            'E': 'elevator',
        }
        prefix_pattern = re.compile(r'^([A-Z]+)(\d+)$')
        # Phase 1: reserve highest IDs
        for chain_str in self.custom_chains:
            chain_str = chain_str.strip().strip('[] \t')
            if not chain_str:
                continue
            tokens = [t.strip() for t in chain_str.split('-') if t.strip()]
            for token in tokens:
                explicit_id, abbr = self._parse_node_token(token)
                if explicit_id:
                    m = prefix_pattern.match(explicit_id.upper())
                    if m:
                        pref, num_str = m.groups()
                        num = int(num_str)
                        node_counter[pref] = max(node_counter.get(pref, 0), num + 1)
        def get_next_id(prefix: str) -> str:
            if prefix not in node_counter:
                node_counter[prefix] = 1
            nid = f"{prefix}{node_counter[prefix]}"
            node_counter[prefix] += 1
            return nid
        def snap(val: float) -> float:
            return round(val / self.edge_length) * self.edge_length
        def snap_pos(x: float, y: float) -> tuple[float, float]:
            return snap(x), snap(y)
        half_cell = self.edge_length / 2.0
        # Global starting point
        current_x = 0.0
        current_y = 0.0
        direction = 0 # 0:right, 1:up, 2:left, 3:down
        for chain_idx, chain_str in enumerate(self.custom_chains):
            chain_str = chain_str.strip().strip('[] \t')
            if not chain_str:
                continue
            tokens = [t.strip() for t in chain_str.split('-') if t.strip()]
            print(f"\n Chain {chain_idx+1}: {chain_str}")
            prev_node = None
            # Try to find an attachment point (first existing node in chain)
            attach_node = None
            attach_idx = -1
            for ti, token in enumerate(tokens):
                eid, _ = self._parse_node_token(token)
                if eid and eid.upper() in created:
                    attach_node = created[eid.upper()]
                    attach_idx = ti
                    print(f" → Attaching at {eid.upper()} (position {ti})")
                    break
            if attach_node:
                branch_x = attach_node.x
                branch_y = attach_node.y
                # Backtrack for nodes before attachment point
                back_x, back_y = branch_x, branch_y
                opp_dir = (direction + 2) % 4
                for _ in range(attach_idx):
                    dx = self.edge_length if opp_dir == 0 else -self.edge_length if opp_dir == 2 else 0
                    dy = self.edge_length if opp_dir == 1 else -self.edge_length if opp_dir == 3 else 0
                    back_x += dx
                    back_y += dy
                    back_x, back_y = snap_pos(back_x, back_y)
                branch_x, branch_y = back_x, back_y
            else:
                branch_x, branch_y = snap_pos(current_x, current_y)
            for i, token in enumerate(tokens):
                explicit_id, abbr = self._parse_node_token(token)
                if abbr not in type_map:
                    raise ValueError(f"Unknown type: {abbr}")
                node_type = type_map[abbr]
                # Force prefix 'C' if ID starts with C
                if explicit_id and explicit_id.upper().startswith('C'):
                    prefix = 'C'
                else:
                    prefix = abbr if abbr in ('W','D','E','S','CH','H') else 'C'
                node_id = explicit_id.upper() if explicit_id else get_next_id(prefix)
                if node_id in created:
                    node = created[node_id]
                    actual_type = node.description  # ← use the real type here
                    if actual_type != node_type:
                        print(f" WARNING: {node_id} already '{actual_type}', chain wants '{node_type}' → keeping '{actual_type}'")
                    branch_x = node.x
                    branch_y = node.y
                    # Print the ACTUAL type and position
                    print(f" {node_id:5} {actual_type:12} ({branch_x:6.1f}, {branch_y:6.1f})")
                else:
                    if node_type == 'waitpoint':
                        if not prev_node or prev_node.description != 'checkpoint':
                            raise ValueError(f"Waitpoint {node_id} requires checkpoint parent")
                        px, py = prev_node.x, prev_node.y
                        # Always toward cell center relative to parent
                        cx = snap(px) + half_cell # cell center x
                        cy = snap(py) + half_cell # cell center y
                        dx = cx - px
                        dy = cy - py
                        mag = math.sqrt(dx**2 + dy**2) or 1.0
                        dx /= mag
                        dy /= mag
                        distance = half_cell * 0.75 # slightly inside
                        x = px + dx * distance
                        y = py + dy * distance
                    else:
                        # Grid-aligned placement
                        x = branch_x
                        y = branch_y
                        if i > 0 and prev_node:
                            dx = self.edge_length if direction == 0 else -self.edge_length if direction == 2 else 0
                            dy = self.edge_length if direction == 1 else -self.edge_length if direction == 3 else 0
                            x += dx
                            y += dy
                        x, y = snap_pos(x, y)
                    node = Node(node_id, x, y, node_type)
                    created[node_id] = node
                    self.nodes[node_id] = node
                    if node_type == 'home_dock':
                        self.home_nodes.append(node)
                    branch_x = x
                    branch_y = y
                    # Print the NEW type
                    print(f" {node_id:5} {node_type:12} ({x:6.1f}, {y:6.1f})")
                if prev_node:
                    prev_node.connect(node)
                prev_node = node
            # Separate next chain (diagonal shift to reduce overlap chance)
            current_x += self.edge_length * 2.5
            current_y -= self.edge_length * 1.5 # downward bias like your experiment
            direction = (direction + 1) % 4
        # Finalize counts & next ID
        self.n_stations = sum(1 for n in self.nodes.values() if n.description == "station_dock")
        self.n_charges = sum(1 for n in self.nodes.values() if n.description == "charge_dock")
        if len(self.home_nodes) < self.n_robots:
            raise ValueError(f"Only {len(self.home_nodes)} home_docks found, but num_robots={self.n_robots}")
        c_ids = [int(loc_id[1:]) for loc_id in self.nodes if loc_id.startswith('C') and loc_id[1:].isdigit()]
        self.next_c_id = max(c_ids) + 1 if c_ids else 1
        print(f"\nBuilt custom graph from {len(self.custom_chains)} chains — {len(self.nodes)} nodes")



    def _build_from_chains(self):
        """
        Deterministic incremental graph builder from chain definitions.
        
        Behavior:
        - Chains are processed sequentially.
        - For each chain, nodes are placed/connected exactly as written.
        - If a node already exists (shared), its existing position & type are reused using a top neighbour or bottom neighbour check vertically.
        - New unseen nodes are placed by extending horizontally from the previous node in the chain.
        - Waitpoints are the ONLY nodes placed diagonally (centered in cell relative to parent checkpoint).
        - No backtracking, no jumping — every token is respected.
        - First declaration of a node sets its type & position permanently.
        - Type conflicts are warned but the first type is kept.
        """

        self.nodes = {}
        self.home_nodes = []
        node_counter = {}
        created = {}  # id → Node

        type_map = {
            'C': 'checkpoint',
            'H': 'home_dock',
            'S': 'station_dock',
            'CH': 'charge_dock',
            'W': 'waitpoint',
            'D': 'door',
            'E': 'elevator',
        }

        prefix_pattern = re.compile(r'^([A-Z]+)(\d+)$')

        # Phase 1: reserve highest IDs
        for chain_str in self.custom_chains:
            chain_str = chain_str.strip().strip('[] \t')
            if not chain_str:
                continue
            tokens = [t.strip() for t in chain_str.split('-') if t.strip()]
            for token in tokens:
                explicit_id, abbr = self._parse_node_token(token)
                if explicit_id:
                    m = prefix_pattern.match(explicit_id.upper())
                    if m:
                        pref, num_str = m.groups()
                        num = int(num_str)
                        node_counter[pref] = max(node_counter.get(pref, 0), num + 1)

        def get_next_id(prefix: str) -> str:
            if prefix not in node_counter:
                node_counter[prefix] = 1
            nid = f"{prefix}{node_counter[prefix]}"
            node_counter[prefix] += 1
            return nid

        def snap(val: float) -> float:
            return round(val / self.edge_length) * self.edge_length

        def snap_pos(x: float, y: float) -> tuple[float, float]:
            return snap(x), snap(y)

        half_cell = self.edge_length / 2.0

        # Tolerance to detect if a position is already occupied
        POS_TOLERANCE = 0.1 * self.edge_length

        def position_is_occupied(x: float, y: float) -> bool:
            for node in created.values():
                if abs(node.x - x) < POS_TOLERANCE and abs(node.y - y) < POS_TOLERANCE:
                    return True
            return False

        def get_free_direction_position(prev_x: float, prev_y: float, preferred_order=None) -> tuple[float, float]:
            if preferred_order is None:
                # Most natural order for most warehouse layouts: right → up → left → down
                preferred_order = [0, 1, 2, 3]  # 0:right, 1:up, 2:left, 3:down

            dirs = [
                (self.edge_length, 0),   # right
                (0, self.edge_length),   # up
                (-self.edge_length, 0),  # left
                (0, -self.edge_length),  # down
            ]

            for d_idx in preferred_order:
                dx, dy = dirs[d_idx]
                cand_x = prev_x + dx
                cand_y = prev_y + dy
                cand_x, cand_y = snap_pos(cand_x, cand_y)
                if not position_is_occupied(cand_x, cand_y):
                    return cand_x, cand_y

            # If all directions blocked → fallback: slight diagonal offset (rare case)
            fallback_x = prev_x + self.edge_length * 0.707  # ≈45°
            fallback_y = prev_y + self.edge_length * 0.707
            fallback_x, fallback_y = snap_pos(fallback_x, fallback_y)
            print("  WARNING: All 4 directions occupied → using fallback diagonal position")
            return fallback_x, fallback_y

        # Global spawn for new disconnected components
        current_x = 0.0
        current_y = 0.0

        for chain_idx, chain_str in enumerate(self.custom_chains):
            chain_str = chain_str.strip().strip('[] \t')
            if not chain_str:
                continue

            tokens = [t.strip() for t in chain_str.split('-') if t.strip()]
            print(f"\n Chain {chain_idx+1}: {chain_str}")

            # Find attachment point (first existing node)
            attach_node = None
            attach_idx = -1
            for ti, token in enumerate(tokens):
                eid, _ = self._parse_node_token(token)
                if eid and eid.upper() in created:
                    attach_node = created[eid.upper()]
                    attach_idx = ti
                    print(f" → Attaching at {eid.upper()} (position {ti})")
                    break

            if attach_node:
                branch_x = attach_node.x
                branch_y = attach_node.y
                prev_node = attach_node
            else:
                branch_x, branch_y = snap_pos(current_x, current_y)
                prev_node = None

            # ───────────────────────────────────────────────
            # 1. Backward pass (before attachment)
            # ───────────────────────────────────────────────
            if attach_idx > 0:
                back_x = branch_x
                back_y = branch_y
                back_prev = attach_node

                for i in range(attach_idx - 1, -1, -1):
                    explicit_id, abbr = self._parse_node_token(tokens[i])
                    node_type = type_map[abbr]
                    prefix = 'C' if (explicit_id and explicit_id.upper().startswith('C')) else \
                            abbr if abbr in ('W','D','E','S','CH','H') else 'C'
                    node_id = explicit_id.upper() if explicit_id else get_next_id(prefix)

                    if node_type == 'waitpoint':
                        # Waitpoints are offset toward center — no direction choice needed
                        px, py = back_prev.x, back_prev.y
                        cx = snap(px) + half_cell
                        cy = snap(py) + half_cell
                        dx = cx - px
                        dy = cy - py
                        mag = math.sqrt(dx**2 + dy**2) or 1.0
                        dx /= mag; dy /= mag
                        distance = half_cell * 0.75
                        x = px + dx * distance
                        y = py + dy * distance
                    else:
                        x, y = get_free_direction_position(back_x, back_y)

                    if node_id in created:
                        node = created[node_id]
                        actual_type = node.description
                        if actual_type != node_type:
                            print(f" WARNING: {node_id} type mismatch → keeping {actual_type}")
                        x, y = node.x, node.y
                        print(f" {node_id:5} {actual_type:12} ({x:6.1f}, {y:6.1f}) ← backward (existing)")
                    else:
                        node = Node(node_id, x, y, node_type)
                        created[node_id] = node
                        self.nodes[node_id] = node
                        if node_type == 'home_dock':
                            self.home_nodes.append(node)
                        print(f" {node_id:5} {node_type:12} ({x:6.1f}, {y:6.1f}) ← backward")

                    node.connect(back_prev)
                    # back_prev.connect(node)  # if bidirectional needed

                    back_prev = node
                    back_x, back_y = x, y

                prev_node = back_prev

            # ───────────────────────────────────────────────
            # 2. Forward pass (attachment onward)
            # ───────────────────────────────────────────────
            for i in range(max(attach_idx, 0), len(tokens)):
                explicit_id, abbr = self._parse_node_token(tokens[i])
                node_type = type_map[abbr]
                prefix = 'C' if (explicit_id and explicit_id.upper().startswith('C')) else \
                        abbr if abbr in ('W','D','E','S','CH','H') else 'C'
                node_id = explicit_id.upper() if explicit_id else get_next_id(prefix)

                if node_type == 'waitpoint':
                    px, py = prev_node.x, prev_node.y
                    cx = snap(px) + half_cell
                    cy = snap(py) + half_cell
                    dx = cx - px
                    dy = cy - py
                    mag = math.sqrt(dx**2 + dy**2) or 1.0
                    dx /= mag; dy /= mag
                    distance = half_cell * 0.75
                    x = px + dx * distance
                    y = py + dy * distance
                else:
                    if prev_node is None:
                        # Very first node of disconnected component
                        x, y = branch_x, branch_y
                    else:
                        x, y = get_free_direction_position(prev_node.x, prev_node.y)

                if node_id in created:
                    node = created[node_id]
                    actual_type = node.description
                    if actual_type != node_type:
                        print(f" WARNING: {node_id} type mismatch → keeping {actual_type}")
                    x, y = node.x, node.y
                    print(f" {node_id:5} {actual_type:12} ({x:6.1f}, {y:6.1f})")
                else:
                    node = Node(node_id, x, y, node_type)
                    created[node_id] = node
                    self.nodes[node_id] = node
                    if node_type == 'home_dock':
                        self.home_nodes.append(node)
                    print(f" {node_id:5} {node_type:12} ({x:6.1f}, {y:6.1f})")

                if prev_node:
                    prev_node.connect(node)
                    # node.connect(prev_node)

                prev_node = node

            # Shift global start only for disconnected chains
            if attach_node is None:
                current_x += self.edge_length * 2.5
                current_y -= self.edge_length * 1.5

        # Final stats
        self.n_stations = sum(1 for n in self.nodes.values() if n.description == "station_dock")
        self.n_charges = sum(1 for n in self.nodes.values() if n.description == "charge_dock")

        if len(self.home_nodes) < self.n_robots:
            raise ValueError(f"Only {len(self.home_nodes)} home_docks, need {self.n_robots}")

        c_ids = [int(loc_id[1:]) for loc_id in self.nodes if loc_id.startswith('C') and loc_id[1:].isdigit()]
        self.next_c_id = max(c_ids) + 1 if c_ids else 1

        print(f"\nBuilt custom graph from {len(self.custom_chains)} chains — {len(self.nodes)} nodes")






    def _parse_node_token(self, token: str) -> tuple[str, str]:
        token = token.strip()
        if '(' not in token or ')' not in token:
            raise ValueError(f"Invalid node token: {token} — expected e.g. C12(C)")
        left, right = token.split('(', 1)
        abbr = right.rstrip(')').strip().upper()
        explicit_id = left.strip() if left.strip() else None
        return explicit_id, abbr


    # ────────────────────────────────────────────────
    # E X P O R T M E T H O D S
    # ────────────────────────────────────────────────


    def _prepare_graph_data(self):
        graph_data = {}
        for node in sorted(self.nodes.values(), key=lambda n: n.loc_id):
            edges = [[nb, 0.1] for nb in sorted(node.neighbors)]
            graph_data[node.loc_id] = edges
        return graph_data

    def _prepare_itinerary_data(self):
        itinerary = []
        z_options = [-0.194, -0.097]
        w_options = [0.98, 0.995]
        for idx, node in enumerate(sorted(self.nodes.values(), key=lambda n: n.loc_id)):
            z = z_options[idx % 2]
            w = w_options[idx % 2]
            entry = {
                "coordinate": [round(node.x, 3), round(node.y, 3), z, w],
                "description": node.description,
                "fleet_id": FLEET_ID,
                "map_id": MAP_ID,
                "loc_id": node.loc_id
            }
            itinerary.append(entry)
        return itinerary

    def _update_config_section(self, section_name: str, section_data: dict):
        path = os.path.join(self.out_dir, "config.yaml")
        existing = {}
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    existing = yaml.safe_load(f) or {}
            except yaml.YAMLError:
                existing = {}
        existing[section_name] = section_data
        with open(path, "w", encoding="utf-8") as f:
            yaml.dump(existing, f, sort_keys=False, width=1000, allow_unicode=True)
        print(f"Updated config.yaml → {section_name} section")

    def _export_full_config_yaml(self):
        path = os.path.join(self.out_dir, "config.yaml")
        # Static sections
        config = {
            "mqtt": {
                "broker_address": os.getenv("MQTT_BROKER", "localhost"),
                "broker_port": "1883",
                "keep_alive": 15
            },
            "fleet_info": {
                "fleetname": FLEET_ID,
                "version": ROBOT_VERSION,
                "versions": ROBOT_VERSIONS,
                "manufacturer": ROBOT_MANUFACTURER
            },
            "postgres": {
                "host": os.getenv("POSTGRES_HOST", "localhost"),
                "port": "5432",
                "database": "postgres", # or "fleet_db"
                "user": "postgres",
                "password": "root" # ← CHANGE THIS in production!
            },
            "maps": {
                FLEET_ID: [
                    {
                        "map_id": MAP_ID,
                        "map_pgm_path": f"config/{MAP_ID}.pgm",
                        "map_yaml_path": f"config/{MAP_ID}.yaml"
                    }
                    # You can add more floors here later
                ]
            },
            "graph": self._prepare_graph_data(),
            "itinerary": self._prepare_itinerary_data(),
        }
        # Merge with existing file (preserve unknown sections)
        existing = {}
        if os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    existing = yaml.safe_load(f) or {}
            except yaml.YAMLError:
                print("Warning: config.yaml corrupted → overwriting")
        existing.update(config)
        with open(path, "w", encoding="utf-8") as f:
            yaml.dump(existing, f,
                      sort_keys=False,
                      width=1000,
                      allow_unicode=True,
                      default_flow_style=False)
        print(f"Created/updated full config.yaml → {path}")

    # Keep your original robots export (separate file)
    def _export_robots_yaml(self):
        path = os.path.join(self.out_dir, "robots.yaml")
        with open(path, "w", encoding="utf-8") as f:
            yaml.dump(self.robots, f, sort_keys=False)
        print(f"Exported robots.yaml → {path}")


    # ────────────────────────────────────────────────
    # G R I D D E S I G N M E T H O D S
    # ────────────────────────────────────────────────


    def _create_grid(self):
        total_special = self.n_robots + self.n_stations + self.n_charges
        desired_free = int(self.n_robots * max(2.5, self.density_factor)) + self.n_wait + 30 # increased margin
        total_goal = total_special + desired_free
        cols = math.ceil(math.sqrt(total_goal * 1.15))
        rows = math.ceil(total_goal / cols)
        actual = rows * cols
        print(f"Goal ~{total_goal} nodes → grid {rows}×{cols} = {actual} nodes")
        print(f" → expected checkpoints before docks: {actual - total_special}")
        self.spacing = self.edge_length
        self.max_x = (cols - 1) * self.spacing
        self.max_y = (rows - 1) * self.spacing
        for r in range(rows):
            for c in range(cols):
                temp_id = f"temp_{r}_{c}"
                self.nodes[temp_id] = Node(temp_id, c * self.spacing, r * self.spacing)
        for r in range(rows):
            for c in range(cols):
                here = self.nodes[f"temp_{r}_{c}"]
                if c + 1 < cols:
                    here.connect(self.nodes[f"temp_{r}_{c+1}"])
                if r + 1 < rows:
                    here.connect(self.nodes[f"temp_{r+1}_{c}"])

    def _rename_nodes_to_convention(self):
        old_to_new = {}
        new_nodes = {}
        for old_id, node in list(self.nodes.items()):
            new_id = f"C{self.next_c_id}"
            self.next_c_id += 1
            node.loc_id = new_id
            old_to_new[old_id] = new_id
            new_nodes[new_id] = node
        self.nodes = new_nodes
        for node in self.nodes.values():
            node.neighbors = {old_to_new.get(nid, nid) for nid in node.neighbors}

    def _assign_special_nodes(self):
        all_special = (
            ["home_dock"] * self.n_robots +
            ["station_dock"] * self.n_stations +
            ["charge_dock"] * self.n_charges
        )
        random.shuffle(all_special)
        candidates = [n for n in self.nodes.values() if n.description == "checkpoint"]
        random.shuffle(candidates)
        used = set()
        for dtype in all_special:
            placed = False
            for node in candidates:
                if node.loc_id in used:
                    continue
                if dtype in ("station_dock", "charge_dock") and self._is_interior(node):
                    continue
                node.description = dtype
                used.add(node.loc_id)
                if dtype == "home_dock":
                    self.home_nodes.append(node)
                placed = True
                break
            if not placed:
                raise RuntimeError(f"Failed to place all {dtype}")

    def _is_interior(self, node):
        margin = self.edge_length
        return margin < node.x < self.max_x - margin and margin < node.y < self.max_y - margin

    def _enforce_single_entry_docks(self):
        for node in list(self.nodes.values()):
            if node.description not in self.DOCK_TYPES:
                continue
            neighbors = list(node.neighbors)
            if not neighbors:
                raise RuntimeError(f"Dock {node.loc_id} has no neighbors after grid creation!")
            cp_nbs = [nid for nid in neighbors if self.nodes[nid].description == "checkpoint"]
            if not cp_nbs:
                print(f"Warning: dock {node.loc_id} has no checkpoint neighbor → using random")
                keep = random.choice(neighbors)
            else:
                # Prefer most connected checkpoint
                cp_nbs.sort(key=lambda nid: len(self.nodes[nid].neighbors), reverse=True)
                keep = cp_nbs[0]
            node.neighbors = {keep}
            for nid in neighbors:
                if nid != keep:
                    self.nodes[nid].neighbors.discard(node.loc_id)
            # Protect this entry checkpoint from pruning
            self.protected_cps.add(keep)

    def _prune_isolated_checkpoints(self):
        """First pass: remove deg=1 checkpoints connected only to other checkpoints"""
        removed = 0
        changed = True
        while changed:
            changed = False
            to_remove = []
            for n in list(self.nodes.values()):
                if n.description != "checkpoint" or len(n.neighbors) != 1:
                    continue
                if n.loc_id in self.protected_cps:
                    continue # protect dock entries
                nb_id = next(iter(n.neighbors))
                nb = self.nodes.get(nb_id)
                if nb and nb.description == "checkpoint":
                    to_remove.append(n.loc_id)
            for rid in to_remove:
                n = self.nodes[rid]
                if n.neighbors:
                    nb_id = next(iter(n.neighbors))
                    self.nodes[nb_id].neighbors.discard(rid)
                del self.nodes[rid]
                removed += 1
                changed = True
        if removed:
            print(f"Pruned {removed} deg-1 checkpoint leaves (checkpoint-only)")

    def _prune_completely_isolated_checkpoints(self):
        """Second pass: remove deg=0 or deg=1 only-to-dock checkpoints"""
        removed = 0
        to_remove = []
        for n in list(self.nodes.values()):
            if n.description != "checkpoint":
                continue
            if n.loc_id in self.protected_cps:
                continue # never prune dock entries
            deg = len(n.neighbors)
            if deg == 0:
                to_remove.append(n.loc_id)
                continue
            if deg == 1:
                nb_id = next(iter(n.neighbors))
                nb = self.nodes.get(nb_id)
                if nb and nb.description in self.DOCK_TYPES:
                    to_remove.append(n.loc_id)
        for rid in to_remove:
            n = self.nodes[rid]
            if n.neighbors:
                nb_id = next(iter(n.neighbors))
                self.nodes[nb_id].neighbors.discard(rid)
            del self.nodes[rid]
            removed += 1
        if removed:
            print(f"Pruned {removed} completely isolated or dock-only checkpoints")

    def _verify_dock_entry_checkpoints(self):
        for node in self.nodes.values():
            if node.description not in self.DOCK_TYPES:
                continue
            if not node.neighbors:
                raise RuntimeError(f"Dock {node.loc_id} has ZERO neighbors after pruning!")
            entry_id = next(iter(node.neighbors))
            if entry_id not in self.nodes:
                raise RuntimeError(f"Dock {node.loc_id} entry {entry_id} was removed during pruning!")
            entry = self.nodes[entry_id]
            if entry.description != "checkpoint":
                raise RuntimeError(f"Dock {node.loc_id} entry {entry_id} is not checkpoint")
            nb_ids = list(entry.neighbors)
            if len(nb_ids) < 2:
                raise RuntimeError(
                    f"Dock entry checkpoint {entry_id} has degree {len(nb_ids)} < 2 — not connected"
                )
            has_cp_nb = any(
                nid != node.loc_id and self.nodes[nid].description == "checkpoint"
                for nid in nb_ids
            )
            if not has_cp_nb:
                raise RuntimeError(
                    f"Dock entry checkpoint {entry_id} has no checkpoint neighbor — isolated"
                )
        print("All dock entry checkpoints verified: connected to at least one other checkpoint")

    def _add_waitpoints(self):
        cps = [n for n in self.nodes.values() if n.description == "checkpoint"]
        random.shuffle(cps)
        half_cell = self.edge_length / 2.0
        min_dist = half_cell * 0.85 # at least 90% of half-edge length
        center_bias = 0.75 # very strong center preference
        for parent in cps[:self.n_wait]:
            # Waitpoint name = W + parent's number (e.g. C16 → W16)
            c_num = parent.loc_id[1:] # strip 'C'
            wid = f"W{c_num}"
            # Direction toward center (0,0 relative)
            dx = -parent.x % self.edge_length + half_cell # vector toward cell center x
            dy = -parent.y % self.edge_length + half_cell # vector toward cell center y
            mag = math.sqrt(dx**2 + dy**2) or 1.0
            dx /= mag
            dy /= mag
            # Distance: between min_dist and ~half_cell
            distance = random.uniform(min_dist, half_cell * 1.1)
            # Apply bias + small jitter
            ox = dx * distance * center_bias + random.uniform(-0.4, 0.4)
            oy = dy * distance * center_bias + random.uniform(-0.4, 0.4)
            wp = Node(wid, parent.x + ox, parent.y + oy, "waitpoint")
            wp.connect(parent)
            self.nodes[wid] = wp

    def _assign_robots(self):
        random.shuffle(self.home_nodes)
        num_assign = min(self.n_robots, len(self.home_nodes))
        for i, home in enumerate(self.home_nodes[:num_assign], 1):
            self.robots.append({
                "fleetname": FLEET_ID,
                "robot_serial_number": f"R{i:02d}",
                "versions": ROBOT_VERSIONS,
                "version": ROBOT_VERSION,
                "manufacturer": ROBOT_MANUFACTURER,
                "connection_state": ROBOT_CONNECTION_STATE,
                "initial_position": [round(home.x, 3), round(home.y, 3), 0.0],
                "bat_charge": round(random.uniform(55.0, 95.0), 1),
                "lin_velocity": ROBOT_LIN_VEL,
                "ang_velocity": ROBOT_ANG_VEL,
            })

    def validate(self):
        print("\nValidation:")
        by_type = {}
        for n in self.nodes.values():
            by_type[n.description] = by_type.get(n.description, 0) + 1
        for t in sorted(by_type):
            print(f" {t:14} : {by_type[t]:3d}")
        dock_nodes = [n for n in self.nodes.values() if n.description in self.DOCK_TYPES]
        if dock_nodes:
            degrees = [len(n.neighbors) for n in dock_nodes]
            print(f" Dock degrees → min/avg/max : {min(degrees)} / {sum(degrees)/len(degrees):.1f} / {max(degrees)}")
            entry_degs = [len(self.nodes[list(n.neighbors)[0]].neighbors) for n in dock_nodes if n.neighbors]
            if entry_degs:
                print(f" Entry CP degrees → min/avg/max : {min(entry_degs)} / {sum(entry_degs)/len(entry_degs):.1f} / {max(entry_degs)}")
        isolated = sum(
            1 for n in self.nodes.values()
            if n.description == "checkpoint" and len(n.neighbors) <= 1
        )
        print(f" Remaining low-degree checkpoints (deg ≤1): {isolated} (should be 0)")

    def plot(self, fname="grid_layout.png"):
        # Compute bounds for dynamic sizing and padding
        if not self.nodes:
            print("No nodes to plot.")
            return
        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        x_span = max_x - min_x if max_x > min_x else 1.0
        y_span = max_y - min_y if max_y > min_y else 1.0
        pad_x = x_span * 0.2  # 20% padding
        pad_y = y_span * 0.2 + 2.0  # Extra for labels/bottom offset
        aspect = x_span / y_span

        # Dynamic figsize: base on max dimension
        base_width = 12.0
        base_height = 10.0
        if aspect > 1:
            fig_width = base_width
            fig_height = base_width / aspect
        else:
            fig_height = base_height
            fig_width = base_height * aspect
        fig, ax = plt.subplots(figsize=(fig_width, fig_height))

        lines = []
        seen = set()
        for node in self.nodes.values():
            for nb_id in node.neighbors:
                pair = tuple(sorted([node.loc_id, nb_id]))
                if pair in seen: continue
                seen.add(pair)
                nb = self.nodes[nb_id]
                lines.append([(node.x, node.y), (nb.x, nb.y)])
        ax.add_collection(mc.LineCollection(lines, color="#999", lw=1.0, alpha=0.65))
        colmap = {
            "checkpoint": "#555555",
            "home_dock": "#c0392b",
            "station_dock": "#e67e22",
            "charge_dock": "#27ae60",
            "waitpoint": "#9b59b6",
            "door": "#3498db",
            "elevator": "#f39c12",
        }
        for node in self.nodes.values():
            c = colmap.get(node.description, "#777")
            m = "s" if node.description == "waitpoint" else "o"
            s = 180 if node.description == "waitpoint" else 260
            ax.scatter(node.x, node.y, s=s, c=c, edgecolor="white", lw=1.1, marker=m, zorder=3)
            if node.description in self.DOCK_TYPES:
                ax.text(node.x, node.y + 1.3, node.loc_id,
                        ha="center", va="bottom", fontsize=7, fontweight="bold")
        ax.set_aspect("equal")
        ax.set_xlim(min_x - pad_x, max_x + pad_x)
        ax.set_ylim(min_y - pad_y, max_y + pad_y)
        ax.axis("off")
        plt.tight_layout()
        path = os.path.join(self.out_dir, fname)
        plt.savefig(path, dpi=160, bbox_inches="tight")
        print(f"Plot saved → {path}")

    def generate_map(self, filename="turtlebot3_world", resolution=0.05, padding=2.0):
        """
        Generates .pgm and .yaml files — writes PGM manually since Pillow doesn't support it natively.
        Produces ROS-compatible map YAML with inline origin: [x, y, z]
        """
        import numpy as np
        # 1. Calculate Bounds
        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        min_x, max_x = min(xs) - padding, max(xs) + padding
        min_y, max_y = min(ys) - padding, max(ys) + padding
        # 2. Pixel dimensions
        width_px = int((max_x - min_x) / resolution)
        height_px = int((max_y - min_y) / resolution)
        # 3. Create map array (255 = free/white)
        map_data = np.ones((height_px, width_px), dtype=np.uint8) * 255
        # 4. Save binary PGM (P5 format – most compatible with map_server)
        pgm_path = os.path.join(self.out_dir, f"{filename}.pgm")
        with open(pgm_path, "wb") as f:
            f.write(b"P5\n")
            f.write(f"{width_px} {height_px}\n".encode())
            f.write(b"255\n")
            f.write(map_data.tobytes())
        print(f"Saved binary PGM: {pgm_path} ({width_px}×{height_px})")
        # 5. Prepare origin
        origin = [
            round(float(min_x), 6),
            round(float(min_y), 6),
            0.0
        ]
        yaml_content = {
            "image": f"{filename}.pgm",
            "resolution": resolution,
            "origin": origin,
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196
        }
        yaml_path = os.path.join(self.out_dir, f"{filename}.yaml")
        # Step A: Write initial YAML (block style is fine here)
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.dump(
                yaml_content,
                f,
                sort_keys=False,
                width=1000,
                allow_unicode=True,
                default_flow_style=False
            )
        # Step B: Post-process to force inline origin and remove any leftover block items
        with open(yaml_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
        with open(yaml_path, "w", encoding="utf-8") as f:
            skip_until_next_key = False
            for line in lines:
                stripped = line.strip()
                if stripped.startswith("origin:"):
                    # Write inline version
                    origin_str = f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]"
                    f.write(origin_str + "\n")
                    skip_until_next_key = True # skip following indented lines
                    continue
                if skip_until_next_key:
                    # Skip indented lines or empty lines after origin until we hit a new top-level key
                    if stripped == "" or stripped.startswith("-") or stripped.startswith(" "):
                        continue
                    else:
                        # New top-level key → stop skipping
                        skip_until_next_key = False
                        f.write(line)
                else:
                    f.write(line)
        print(f"Saved map YAML → {yaml_path}")
        print(f" Origin : {origin}")
        print(f" Size : {width_px} × {height_px} pixels")
        print(f" Resolution : {resolution} m/px")


# ────────────────────────────────────────────────
# M A I N E X E C U T I O N
# ────────────────────────────────────────────────


if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(base_dir, "..", "config")

    # Example 1: Auto mode
    # g = GridFleetGraph(
    #     num_robots=3,
    #     num_station_docks=6,
    #     num_charge_docks=2,
    #     num_waitpoints=4,
    #     density_factor=1.3,
    #     output_dir=config_dir,
    # )

    # Example 2: Custom graph (dict) mode
    # custom_graph_example = {
    #     'c1': {
    #         'description': 'home_dock',
    #         'location': '0,0',
    #         'neighbors': 'c2,c3'
    #     },
    #     'c2': {
    #         'description': 'checkpoint',
    #         'location': '0,6',
    #         'neighbors': 'c1,c4'
    #     },
    #     # Add more nodes...
    # }
    # g = GridFleetGraph(
    #     num_robots=1,
    #     num_station_docks=2,
    #     num_charge_docks=1,
    #     num_waitpoints=0,
    #     density_factor=0.0,
    #     output_dir=config_dir,
    #     custom_graph=custom_graph_example,
    # )

    # # Example 3: Custom chains mode
    custom_chains_example = [
            "[C2(H) - C10(C) - C11(C) - C3(S)]",
            "[C9(H) - C10(C) - C12(C) - C13(S)]",
            "[C12(C) - W12(W)]"
        ]
    # custom_chains_example = [
    #         "[C2(H) - C10(C) - C11(C) - C3(S)]",
    #         "[C9(H) - C10(C) - C12(C) - C13(S)]",
    #     ]
    # custom_chains_example = [
    #         "[C10(H) - C11(C) - C12(C) - C13(C) - C14(S)]",
    #         "[C9(H) - C11(C) - C15(C) - C16(S)]",
    #         "[C12(C) - C17(C) - C18(S)]",
    #         "[C13(C) - C19(C)]",
    #     ]
    g = GridFleetGraph(
        num_robots=1,
        num_station_docks=1,
        num_charge_docks=0,
        num_waitpoints=0,
        density_factor=0.0,
        output_dir=config_dir,
        custom_chains=custom_chains_example,
    )

    g.validate()
    g.plot("fleet_grid_final.png")