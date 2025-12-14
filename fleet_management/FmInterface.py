#!/usr/bin/env python3

import sys
import os
import time

# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# Append the path to the directory containing the file you want to import
sys.path.append('/home/hazeezadebayo/Desktop/fleet_management_v5/fleet_management/submodules') # ubuntu
# sys.path.append('/Users/Azeez/Desktop/fleet_management_v5/fleet_management/submodules') # windows

# Now you can import the module
from FmMain import FmMain

# ---------------------------------------------- #
#      MAIN                                      #
# ---------------------------------------------- #

if __name__ == "__main__":

    # 1. Initialize the Fleet Manager
    fm = FmMain()
    fm.start_main_loop_thread() # Starts the background thread (timer disabled)

    print("--- Initializing System (Waiting 5s for MQTT) ---")
    time.sleep(5)

    # 2. Set the start time reference
    start_time = time.time()
    task_trigger_time = 150 #  150 # [seconds]
    analytics_interval = 50 # [snapshot every 60s → 1 minute.]
    last_analytics_time = 0   # means "no snapshot yet"

    print("--- Simulation Loop Started ---")

    fm.fm_dispatch_task(fleet_id="kullar")

    # Define all tasks in one place
    tasks = [
        # {"delay": 0, "robot_id": "R01", "from": "C11", "to": "C12", "priority": "high",   "payload": 10},
        # {"delay": 7, "robot_id": "R02", "from": "C12", "to": "C11", "priority": "medium", "payload": 2},
        # {"delay": 14,"robot_id": "R03", "from": "C9",  "to": "C10", "priority": "low",    "payload": 2},
        {"delay": 21,"robot_id": "R04", "from": "C10", "to": "C9",  "priority": "high",   "payload": 10},
        # {"delay": 28,"robot_id": "R05", "from": "C21", "to": "C22", "priority": "medium", "payload": 10},
        # {"delay": 35,"robot_id": "R06", "from": "C23", "to": "C21", "priority": "low",    "payload": 0},
        # {"delay": 42,"robot_id": "R07", "from": "C22", "to": "C23", "priority": "medium", "payload": 10},
        # {"delay": 49,"robot_id": "R08", "from": "C23", "to": "C22", "priority": "low",    "payload": 0},
    ]

    # Add a flag to each task
    for t in tasks:
        t["sent"] = False

    # try to keep main thread alive to observe tasks
    try:

        # Inside your loop:
        while True:
            elapsed_time = time.time() - start_time

            print("-----------------------------------------------------------")
            print("-----------------------------------------------------------")
            print("-----------------------------------------------------------")
            print("-----------------------------------------------------------")
            print(f"[Elapsed Time: {int(elapsed_time)}s]...")
            print("-----------------------------------------------------------")
            print("-----------------------------------------------------------")
            print("-----------------------------------------------------------")
            print("-----------------------------------------------------------")

            # periodically call analytics
            if elapsed_time - last_analytics_time >= analytics_interval:
                fm.schedule_handler.fm_analytics(
                    f_id="kullar", m_id=fm.manufacturer,
                    r_id=None, debug_level="info", write_to_file=True
                )
                last_analytics_time = elapsed_time

            # loop through tasks
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