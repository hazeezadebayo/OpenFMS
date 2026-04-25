#!/usr/bin/env python3

import sys
import os
import time
import random
import argparse
import math

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
#  ['C10', 'C11', 'C3'] Store list of lists e.g. [['C1', 'C2'], ['C10', 'C11']]
SCENARIOS = {
    "S1": {
        "description": "C10 - C11 - C3: [Mutex Group Conflict], C12 - C13: [Only one has free C_node]",
        "custom_chains": [
            "[C2(H) - C10(C) - C11(C) - C3(S)]",
            "[C9(H) - C10(C) - C12(C) - C13(S)]",
            "[C12(C) - W12(W)]"
        ],
        "mutex_groups": [['C10', 'C11', 'C3'],], # Store list of lists
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
# S C E N A R I O   G E N E R A T I O N 
# ────────────────────────────────────────────────

def get_fleet_config(num_robots):
    # Ensure we always have a bit of 'breathing room' for swaps
    num_station_docks = math.ceil(num_robots * 1.2) + 2
    
    # Standard industrial charging ratio (1 charger per 5 robots)
    num_charge_docks = max(1, math.ceil(num_robots / 5))
    
    # Waitpoints are the 'pockets' for robots to pull into during conflicts
    num_waitpoints = math.ceil(num_robots * 0.5) + 2
    
    # 1.3 provides good redundancy without making the graph 
    # overly complex for the O(K) find_nearest_node search.
    density_factor = 1.3
    
    return {
        "num_station_docks": num_station_docks,
        "num_charge_docks": num_charge_docks,
        "num_waitpoints": num_waitpoints,
        "density_factor": density_factor
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
        # ==========================================
        # CASE 1: FIXED SCENARIOS (S1, S2, etc.)
        # ==========================================
        if mode.startswith('S') and mode in SCENARIOS:
            print(f"--- Generating Graph for Scenario: {mode} ---")
            scenario = SCENARIOS[mode]
            g = GridFleetGraph(
                num_robots=scenario["num_robots"],
                num_station_docks=0,
                num_charge_docks=0,
                num_waitpoints=0,
                density_factor=0.0,
                custom_chains=scenario["custom_chains"]
            )         
        # ==========================================
        # CASE 2: PROCEDURAL SCALING (N10, N50, etc.)
        # ==========================================
        elif mode.startswith('N'):
            try:
                # Extract the integer after 'N'
                num_robots = int(mode[1:]) 
                print(f"--- Generating Procedural Graph for {num_robots} Robots ---")
                
                # Fetch the 'Reasonable' configuration
                config = get_fleet_config(num_robots)

                attempt = 1
                while g is None:
                    try:
                        g = GridFleetGraph(
                            num_robots=num_robots, # 3,
                            num_station_docks=config["num_station_docks"], # 6,
                            num_charge_docks=config["num_charge_docks"], # 2,
                            num_waitpoints=config["num_waitpoints"], # 4,
                            density_factor=config["density_factor"] # 1.3,
                        ) 
                    except RuntimeError as e:
                        print(f"[Attempt {attempt}] Grid generation failed ({e}). Retrying...")
                        attempt += 1
                        if attempt > 20: 
                            print("CRITICAL: Could not generate valid graph. Adjust density.")
                            break
            except ValueError:
                print(f"❌ Error: Invalid format '{mode}'. Use N followed by a number (e.g., N25).")
                sys.exit(1)
        else:
            print(f"❌ Error: Unknown mode '{mode}'. Use S<number> for scenarios or N<number> for scaling.")
            sys.exit(1)

        print(f"\n✅ Map generation complete. Exiting.")
        if g is not None:
            g.plot(f"{mode}_grid_layout.png")

        # generation phase completed.
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
        fm.schedule_handler.traffic_handler.mutex_groups = SCENARIOS[mode]["mutex_groups"]

    fm.start_main_loop_thread()
    time.sleep(5)

    start_time = time.time()
    task_trigger_time = 5 
    
    if mode in SCENARIOS:
        tasks = SCENARIOS[mode]["tasks"]
        print(f"Goal: {SCENARIOS[mode]['description']}")

    else:
        # 1. Parse robot count from mode (e.g., N10 -> 10)
        # We exit strictly if the format is incorrect to prevent undefined behavior.
        try:
            if not mode.startswith('N'):
                raise ValueError("Random mode must start with 'N' followed by a number.")
            num_robots = int(mode[1:])
        except (ValueError, IndexError) as e:
            print(f"❌ Critical Error: Could not evaluate robot count from '{mode}'.")
            print(f"Details: {e}")
            sys.exit(1)

        # 2. Prepare remaining docks for task randomization
        itinerary = fm.task_dictionary.get('itinerary', [])
        station_docks = [n['loc_id'] for n in itinerary if n.get('description') == "station_dock"]
        charge_docks = [n['loc_id'] for n in itinerary if n.get('description') == "charge_dock"]

        # Uniqueness guarantee: shuffle and pop
        random.shuffle(station_docks)
        random.shuffle(charge_docks)

        priorities = ["high", "medium", "low"]
        tasks = []

        # 1. Determine the necessary padding length
        # log10 or len(str()) both work; len(str()) is more "Pythonic"
        padding_limit = max(2, len(str(num_robots))) 

        for i in range(num_robots):
            # 2. Generate ID dynamically: R01, R02.. R100... R0001, R0002, etc.
            robot_id = f"R{str(i+1).zfill(padding_limit)}"
            
            # Randomize priority for this specific robot's task
            current_priority = random.choice(priorities)
            
            # 3. Task Selection Logic
            if i % 5 == 0 and len(charge_docks) > 0:
                # CHARGE TASK: from == to
                dock = charge_docks.pop(0)
                tasks.append({
                    "delay": i * 5,
                    "robot_id": robot_id,
                    "from": dock,
                    "to": dock,
                    "type": "charge",
                    "priority": current_priority,
                    "payload": 0,
                    "sent": False
                })
                print(f"[Task Gen] {robot_id} ({current_priority.upper()}): Charging at {dock}")
            
            elif len(station_docks) >= 2:
                # TRANSPORT TASK: Unique Start and unique End
                start_node = station_docks.pop(0)
                end_node = station_docks.pop(0)
                tasks.append({
                    "delay": i * 5,
                    "robot_id": robot_id,
                    "from": start_node,
                    "to": end_node,
                    "type": "transport",
                    "priority": current_priority,
                    "payload": 5,
                    "sent": False
                })
                print(f"[Task Gen] {robot_id} ({current_priority.upper()}): {start_node} -> {end_node}")
            
            else:
                print(f"⚠️ Warning: Insufficient docks for {robot_id}. Task generation halted.")
                break

        print(f"\n✅ Fleet Configuration Ready: {len(tasks)} robots initialized with mixed priorities.")

    # Warmup: request factsheets from all robots so the FM has robot data
    # set the fleet_ids and robot_ids
    fm.schedule_handler.fm_send_factsheet_request(fm.manufacturer, fm.version)
    fm.fleetnames = fm.schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_fleets()
    fm.fleetname = "kullar" if "kullar" in fm.fleetnames else None
    fm.upload_all_maps(fm.fleetname)
    # fm.job_ids = fm.process_itinerary(fm.task_dictionary.get("itinerary", []), fm.fleetname)
    fm.serial_numbers = fm.schedule_handler.traffic_handler.task_handler.factsheet_handler.fetch_serial_numbers(fm.fleetname)
    
    # # Categorize docks for event monitoring (simulation completion and station arrivals)
    # itinerary = fm.task_dictionary.get('itinerary', [])
    # home_docks = {n['loc_id'] for n in itinerary if n.get('description') == "home_dock"}
    # all_station_docks = {n['loc_id'] for n in itinerary if n.get('description') == "station_dock"}

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

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    
    fm.cleanup()
        