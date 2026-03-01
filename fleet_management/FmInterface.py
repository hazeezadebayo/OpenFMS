#!/usr/bin/env python3

import sys
import os
import time
import random
import argparse

print("[FmInterface] Starting imports...")

# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# Append the path to the directory containing the file you want to import
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'submodules')))

from FmSimGenerator import GridFleetGraph

print("[FmInterface] Imports complete.")


# ────────────────────────────────────────────────
# S C E N A R I O   D E F I N I T I O N S
# ────────────────────────────────────────────────


SCENARIOS = {
    "S1": {
        "description": "C10 - C11 - C3: [Mutex Group Conflict], C12 - C13: [Only one has free C_node]",
        "custom_chains": [
            "[C2(H) - C10(C) - C11(C) - C3(S)]",
            "[C9(H) - C10(C) - C12(C) - C13(S)]",
            "[C12(C) - W12(W)]"
        ],
        "num_robots": 2,
        "tasks": [
            {"delay": 2, "robot_id": "R01", "from": "C3", "to": "C13", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            {"delay": 8, "robot_id": "R02", "from": "C13", "to": "C3", "type": "transport", "priority": "low",  "payload": 5, "sent": False}
        ]
    },
    "S2": {
        "description": "C12 - C13: [No waitpoints, both has free C_nodes], [Only one has free C_node], [Both have waitpoints].",
        "custom_chains": [
            "[C10(H) - C11(C) - C12(C) - C13(C) - C14(S)]",
            "[C9(H) - C11(C) - C15(C) - C16(S)]",
            "[C12(C) - C17(C) - C18(S)]",
            "[C13(C) - C19(C)]"
        ],
        "num_robots": 2,
        "tasks": [
            {"delay": 2, "robot_id": "R01", "from": "C14", "to": "C16", "type": "transport", "priority": "high", "payload": 5, "sent": False},
            {"delay": 8, "robot_id": "R02", "from": "C14", "to": "C16", "type": "transport", "priority": "low", "payload": 5, "sent": False}
        ]
    },
    "S3": {
        "description": "No-swap Conflict  – [charge tasks]",
        "custom_chains": [
            "[C10(H) - C11(C) - C12(CH)]",
            "[C9(CH) - C11(C) - C13(H)]"
        ],
        "num_robots": 2,
        "tasks": [
            {"delay": 2, "robot_id": "R01", "from": "C12", "to": "C12", "type": "charge", "priority": "high", "payload": 5, "sent": False},
            {"delay": 8, "robot_id": "R02", "from": "C9", "to": "C9", "type": "charge", "priority": "low", "payload": 5, "sent": False}
        ]
    }
}

# ────────────────────────────────────────────────
# M A I N   E X E C U T I O N
# ────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OpenFMS Live Test Interface")
    parser.add_argument("cmd", choices=["generate", "run"], help="Command to execute")
    parser.add_argument("mode", help="Test scenario to run (e.g. S1, S2, S3) or 'random'.")
    args = parser.parse_args()

    mode = args.mode.upper()
    cmd = args.cmd

    # ==========================================
    # 1. MAP GENERATION PHASE
    # ==========================================
    g = None
    
    if cmd == "generate":
        print(f"--- Generating Graph for Scenario: {mode} ---")
        if mode in SCENARIOS:
            scenario = SCENARIOS[mode]
            g = GridFleetGraph(
                num_robots=scenario["num_robots"],
                num_station_docks=0,
                num_charge_docks=0,
                num_waitpoints=0,
                density_factor=0.0,
                custom_chains=scenario["custom_chains"]
            )
        else:
            # Random mode generation
            attempt = 1
            while g is None:
                try:
                    g = GridFleetGraph(
                        num_robots=3,
                        num_station_docks=6,
                        num_charge_docks=2,
                        num_waitpoints=4,
                        density_factor=1.3
                    )
                except RuntimeError as e:
                    print(f"[Attempt {attempt}] Grid generation failed ({e}). Retrying...")
                    attempt += 1
        
        print(f"\n✅ Map generation complete. Exiting.")
        if g is not None:
            g.plot(f"{mode}_grid_layout.png")
        sys.exit(0)

    # ==========================================
    # 2. FLEET MANAGER RUN PHASE
    # ==========================================
    from FmMain import FmMain
    print(f"\n--- Initializing Fleet Manager for Scenario: {mode} ---")
    fm = FmMain()
    
    # Apply mutex groups if defined in scenario
    if mode in SCENARIOS and "mutex_groups" in SCENARIOS[mode]:
        print(f"Applying Mutex Groups: {SCENARIOS[mode]['mutex_groups']}")
        fm.traffic_handler.mutex_groups = SCENARIOS[mode]["mutex_groups"]

    fm.start_main_loop_thread()
    time.sleep(5)

    start_time = time.time()
    task_trigger_time = 5 
    
    if mode in SCENARIOS:
        tasks = SCENARIOS[mode]["tasks"]
        print(f"Goal: {SCENARIOS[mode]['description']}")
    else:
        # Generate some random tasks if in random mode
        station_docks = [n['loc_id'] for n in fm.task_dictionary.get('itinerary', []) if n.get('description') == "station_dock"]
        if not station_docks: station_docks = ["C1", "C2"]
        tasks = []
        for i, robot in enumerate(["R01", "R02"]):
            tasks.append({
                "delay": i * 5,
                "robot_id": robot,
                "from": random.choice(station_docks),
                "to": random.choice(station_docks),
                "priority": "medium",
                "payload": 5,
                "sent": False
            })

    # Warmup: request factsheets from all robots so the FM has robot data
    # before the first real task fires. (Previously this was a no-arg
    # fm_dispatch_task call that silently used a stale 'A12' default.)
    fm.schedule_handler.fm_send_factsheet_request(fm.manufacturer, fm.version)

    analytics_interval = 30  # seconds between dashboard snapshot updates
    last_analytics_time = -analytics_interval  # fire immediately on first cycle

    try:
        while True:
            elapsed_time = time.time() - start_time
            if int(elapsed_time) % 5 == 0:
                print(f"\r[Elapsed Time: {int(elapsed_time)}s]", end="", flush=True)

            for t in tasks:
                if elapsed_time >= task_trigger_time + t["delay"] and not t["sent"]:
                    print(f"\n[Time: {int(elapsed_time)}s] Dispatching {t['type']} task for {t['robot_id']}: {t['from']} -> {t['to']}")
                    fm.fm_dispatch_task(
                        fleet_id="kullar",
                        robot_id=t["robot_id"],
                        from_loc=t["from"],
                        to_loc=t["to"],
                        task_name=t["type"],
                        priority=t["priority"],
                        payload=t["payload"]
                    )
                    t["sent"] = True

            # Periodically write analytics snapshot so the dashboard stays current.
            if elapsed_time - last_analytics_time >= analytics_interval:
                fm.schedule_handler.fm_analytics(
                    f_id="kullar", m_id=fm.manufacturer,
                    r_id=None, debug_level="info", write_to_file=True
                )
                last_analytics_time = elapsed_time

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
        fm.cleanup()
        