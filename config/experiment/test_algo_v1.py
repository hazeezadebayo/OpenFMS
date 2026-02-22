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

RANDOM_SEED = 43
random.seed(RANDOM_SEED)

# ────────────────────────────────────────────────
# G L O B A L   C O N F I G U R A T I O N
# ────────────────────────────────────────────────

FLEET_ID = "kullar"
MAP_ID = "bina_1_floor_0"
EDGE_MODE = "bidirectional"  # currently only bidirectional implemented

ROBOT_VERSION = "2.0.0"
ROBOT_VERSIONS = "v2"
ROBOT_MANUFACTURER = "birfen"
ROBOT_CONNECTION_STATE = "ONLINE"
ROBOT_LIN_VEL = 0.65
ROBOT_ANG_VEL = 0.15

# ────────────────────────────────────────────────
# N O D E   C L A S S
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
# G R I D   F L E E T   G R A P H   C L A S S
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
        output_dir: str = "./output",
    ):
        self.n_robots = num_robots
        self.n_stations = num_station_docks
        self.n_charges = num_charge_docks if num_charge_docks > 0 else max(2, num_robots // 4)
        self.n_wait = num_waitpoints
        self.density_factor = density_factor
        self.out_dir = output_dir

        self.nodes = {}
        self.home_nodes = []
        self.robots = []
        self.next_c_id = 1
        self.protected_cps = set()  # checkpoints used as dock entries → protected from pruning

        os.makedirs(self.out_dir, exist_ok=True)

        self._create_grid()
        self._rename_nodes_to_convention()
        self._assign_special_nodes()
        self._enforce_single_entry_docks()
        self._prune_isolated_checkpoints()
        self._prune_completely_isolated_checkpoints()
        self._verify_dock_entry_checkpoints()
        self._add_waitpoints()
        self._assign_robots()

        # ── Export everything to one config.yaml + generate map ──
        self._export_full_config_yaml()
        self._export_robots_yaml()  # keep separate if you want
        self.generate_map(filename=MAP_ID, resolution=0.05, padding=3.0)

    # ────────────────────────────────────────────────
    #   E X P O R T   M E T H O D S
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
                "broker_address": "localhost",
                "broker_port": 1883,
                "keep_alive": 15
            },
            "fleet_info": {
                "fleetname": FLEET_ID,
                "version": ROBOT_VERSION,
                "versions": ROBOT_VERSIONS,
                "manufacturer": ROBOT_MANUFACTURER
            },
            "postgres": {
                "host": "localhost",
                "port": "5432",
                "database": "postgres",  # or "fleet_db"
                "user": "postgres",
                "password": "root"       # ← CHANGE THIS in production!
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
    #   G R I D   D E S I G N   M E T H O D S
    # ────────────────────────────────────────────────

    def _create_grid(self):
        total_special = self.n_robots + self.n_stations + self.n_charges
        desired_free = int(self.n_robots * max(2.5, self.density_factor)) + self.n_wait + 30  # increased margin
        total_goal = total_special + desired_free

        cols = math.ceil(math.sqrt(total_goal * 1.15))
        rows = math.ceil(total_goal / cols)
        actual = rows * cols

        print(f"Goal ~{total_goal} nodes → grid {rows}×{cols} = {actual} nodes")
        print(f"  → expected checkpoints before docks: {actual - total_special}")

        self.spacing = 6.0
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
        margin = self.spacing
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
                    continue  # protect dock entries
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
                continue  # never prune dock entries
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

        half_cell = self.spacing / 2.0
        min_dist = half_cell * 0.85          # at least 90% of half-edge length
        center_bias = 0.75                   # very strong center preference

        for parent in cps[:self.n_wait]:
            # Waitpoint name = W + parent's number (e.g. C16 → W16)
            c_num = parent.loc_id[1:]       # strip 'C'
            wid = f"W{c_num}"

            # Direction toward center (0,0 relative)
            dx = -parent.x % self.spacing + half_cell  # vector toward cell center x
            dy = -parent.y % self.spacing + half_cell  # vector toward cell center y

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
        for i, home in enumerate(self.home_nodes, 1):
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
            print(f"  {t:14} : {by_type[t]:3d}")

        dock_nodes = [n for n in self.nodes.values() if n.description in self.DOCK_TYPES]
        if dock_nodes:
            degrees = [len(n.neighbors) for n in dock_nodes]
            print(f"  Dock degrees    → min/avg/max : {min(degrees)} / {sum(degrees)/len(degrees):.1f} / {max(degrees)}")
            entry_degs = [len(self.nodes[list(n.neighbors)[0]].neighbors) for n in dock_nodes]
            print(f"  Entry CP degrees → min/avg/max : {min(entry_degs)} / {sum(entry_degs)/len(entry_degs):.1f} / {max(entry_degs)}")

        isolated = sum(
            1 for n in self.nodes.values()
            if n.description == "checkpoint" and len(n.neighbors) <= 1
        )
        print(f"  Remaining low-degree checkpoints (deg ≤1): {isolated} (should be 0)")

    def plot(self, fname="grid_layout.png"):
        fig, ax = plt.subplots(figsize=(12, 10))
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
            "checkpoint":   "#555555",
            "home_dock":    "#c0392b",
            "station_dock": "#e67e22",
            "charge_dock":  "#27ae60",
            "waitpoint":    "#9b59b6",
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
                    skip_until_next_key = True  # skip following indented lines
                    continue

                if skip_until_next_key:
                    # Skip indented lines or empty lines after origin until we hit a new top-level key
                    if stripped == "" or stripped.startswith("-") or stripped.startswith("  "):
                        continue
                    else:
                        # New top-level key → stop skipping
                        skip_until_next_key = False
                        f.write(line)
                else:
                    f.write(line)

        print(f"Saved map YAML → {yaml_path}")
        print(f"  Origin     : {origin}")
        print(f"  Size       : {width_px} × {height_px} pixels")
        print(f"  Resolution : {resolution} m/px")

# ────────────────────────────────────────────────
# M A I N   E X E C U T I O N
# ────────────────────────────────────────────────

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(base_dir, "..", "config")

    g = GridFleetGraph(
        num_robots=3,
        num_station_docks=6,
        num_charge_docks=2,
        num_waitpoints=4,
        density_factor=1.3,
        output_dir=config_dir,
    )
    g.validate()
    g.plot("fleet_grid_final.png")