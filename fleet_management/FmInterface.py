#!/usr/bin/env python3

import sys
import os
import time
import random

# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# Append the path to the directory containing the file you want to import
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'submodules')))

# Now you can import the module
from FmMain import FmMain
from FmSimGenerator import GridFleetGraph


# ────────────────────────────────────────────────
# M A I N   E X E C U T I O N
# ────────────────────────────────────────────────

if __name__ == "__main__":

    # 1. Initialize the Fleet Manager
    fm = FmMain()
    fm.start_main_loop_thread() # Starts the background thread (timer disabled)

    print("--- Initializing System (Waiting 5s for MQTT) ---")
    time.sleep(5)

    # 2. Set the start time reference
    start_time = time.time()
    task_trigger_time = 160 # [seconds]
    analytics_interval = 120 # [snapshot every 60s → 1 minute.]
    last_analytics_time = 0   # means "no snapshot yet"

    print("--- Simulation Loop Started ---")

    fm.fm_dispatch_task(fleet_id="kullar")

    # The GridFleetGraph might occasionally generate an invalid grid layout
    # due to random checkpoint pruning. Auto-retry until it passes.
    g = None
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
    # The GridFleetGraph automatically writes configuration yaml files.
    # We just need to extract the station docs to randomize tasks.
    
    station_docks = [n.loc_id for n in g.nodes.values() if n.description == "station_dock"]
    if len(station_docks) < 2:
        print("Warning: Not enough station docks generated to create tasks!")
        station_docks = ["C11", "C12"] # Absolute fallback

    # Dynamically generate tasks picking random station_docks for start and end
    tasks = []
    robots_to_assign = ["R01", "R02", "R03"]
    priorities = ["low", "medium", "high"]
    
    for i, robot in enumerate(robots_to_assign):
        from_loc = random.choice(station_docks)
        to_loc = random.choice([d for d in station_docks if d != from_loc])
        
        tasks.append({
            "delay": i * 7,
            "robot_id": robot,
            "from": from_loc,
            "to": to_loc,
            "priority": priorities[i % len(priorities)],
            "payload": random.randint(2, 10)
        })

    # Add a flag to each task
    for t in tasks:
        t["sent"] = False

    # try to keep main thread alive to observe tasks
    try:

        # Inside your loop:
        while True:
            elapsed_time = time.time() - start_time

            print(f"------- [Elapsed Time: {int(elapsed_time)}s] -------")

            # periodically call analytics
            if elapsed_time - last_analytics_time >= analytics_interval:
                fm.schedule_handler.fm_analytics(
                    f_id="kullar", m_id=fm.manufacturer,
                    r_id=None, debug_level="info", write_to_file=True
                )
                last_analytics_time = elapsed_time

            # loop through static initial tasks
            for t in tasks:
                if elapsed_time >= task_trigger_time + t["delay"] and not t["sent"]:
                    print(f"[Time: {int(elapsed_time)}s] Triggering {t['robot_id']}...")
                    fm.fm_dispatch_task(
                        fleet_id="kullar",
                        robot_id=t["robot_id"],
                        from_loc=t["from"],
                        to_loc=t["to"],
                        task_name="transport",
                        priority=t["priority"],
                        payload=t["payload"]
                    )
                    t["sent"] = True
                
            time.sleep(1)

    except KeyboardInterrupt:
        print("Simulation stopped by user.")
        fm.cleanup()