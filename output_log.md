---
## 2026-04-26T20:44 — Snapshot persistence fix

**Command:**
```
python3 -c "import ast; ast.parse(open('fleet_management/FmScheduleHandler.py').read()); print('Syntax OK')"
```

**Output:**
```
Syntax OK
```

**Change applied:** `fleet_management/FmScheduleHandler.py`
- Removed `glob` from imports (now unused).
- Replaced numbered-file snapshot scheme (`result_snapshot_N.txt` + delete old) with a single fixed filename (`result_snapshot.txt`) using atomic `rename`.
- Root cause: each event call created `result_snapshot_N+1.txt` and deleted `result_snapshot_N.txt`. The last robot to finish owned the final file. Fix: always overwrite one file. `_shared_analytics_data` is a class-level accumulator — every write already holds ALL robots; the numbered scheme was the only defect.
---
## 2026-04-26T21:05 — Host visibility and cleanup fix

**Changes applied:**

1. `fleet_management/FmScheduleHandler.py`:
   - Updated success message to explicitly mention the host-mapped path: `(Access on host at: logs/result_snapshot.txt)`.
   - This helps clarify that `/app/logs` in the container is indeed the host's `logs/` directory.
2. `run_openfms.sh`:
   - Updated pre-flight cleanup to remove both old indexed snapshots and the new fixed-name snapshot: `rm -f logs/result_snapshot*.txt`.
   - Added a direct shortcut in the "Scenario complete" summary: `🔍 Analytics: cat logs/result_snapshot.txt`.

**Note:** User must run `./run_openfms.sh` (or `docker compose build`) to rebuild the container image and see these changes.












VisualizationSubscriber: [INFO  ] [2026-05-10 13:37:04] FmTrafficHandler.update_robot_status  - r_id: R02 --> estimated wait time 9.23076923076923.
[INFO] [2026-05-10 13:37:04,236] OrderPublisher: Swapping waitpoint with the first checkpoint.
[INFO] [2026-05-10 13:37:04,240] OrderPublisher: Robot R02: Logged duration 18.46s for order 35951a6e-e497-4983-a55e-f6a69a9aef74. (Cumulative: 18.46s, Completed: False)
[INFO] [2026-05-10 13:37:04,240] OrderPublisher: 🛸 [WAIT RECORDED] Robot R02: Order 35951a6e-e497-4983-a55e-f6a69a9aef74_4 wait duration 18.46153846153846
 🛸 Order Wait Time Triggered: 18.46153846153846 🛸
[INFO] [2026-05-10 13:37:04,240] OrderPublisher: Publishing Order Message...
[INFO] [2026-05-10 13:37:04,240] OrderPublisher: {
    "headerId": 5,
    "timestamp": "2026-05-10T13:37:04.236952",
    "version": "2.0.0",
    "manufacturer": "birfen",
    "serialNumber": "R02",
    "orderId": "35951a6e-e497-4983-a55e-f6a69a9aef74_4",
    "orderUpdateId": 4,
    "zoneSetId": "kullar",
    "nodes": [
        {
            "nodeId": "C12",
            "released": true,
            "sequenceId": 1,
            "nodeDescription": "current base node.",
            "nodePosition": {
                "x": 6.0,
                "y": -6.0,
                "theta": -0.39011656541402734,
                "mapId": "bina_1_floor_0",
                "allowedDeviationXY": 0.5,
                "allowedDeviationTheta": 3.1
            },
            "actions": []
        },
        {
            "nodeId": "W12",
            "released": true,
            "sequenceId": 3,
            "nodeDescription": "Task Priority: low, Task Type: transport, Node Type: Waitpoint, Wait Time: 18.46153846153846",
            "nodePosition": {
                "x": 7.591,
                "y": -4.409,
                "theta": -0.19425138730222632,
                "mapId": "bina_1_floor_0",
                "allowedDeviationXY": 0.5,
                "allowedDeviationTheta": 3.1
            },
            "actions": []
        },
        {
            "nodeId": "C12",
            "released": true,
            "sequenceId": 3,
            "nodeDescription": "Task Priority: low, Task Type: transport, Node Type: Checkpoint, Wait Time: 18.46153846153846",
            "nodePosition": {
                "x": 6.0,
                "y": -6.0,
                "theta": -0.39011656541402734,
                "mapId": "bina_1_floor_0",
                "allowedDeviationXY": 0.5,
                "allowedDeviationTheta": 3.1
            },
            "actions": []
        },
        {
            "nodeId": "C13",
            "released": true,
            "sequenceId": 5,
            "nodeDescription": "Task Priority: low, Task Type: transport, Node Type: Checkpoint, Wait Time: 18.46153846153846",
            "nodePosition": {
                "x": 12.0,
                "y": -6.0,
                "theta": -0.19425138730222632,
                "mapId": "bina_1_floor_0",
                "allowedDeviationXY": 0.5,
                "allowedDeviationTheta": 3.1
            },
            "actions": [
                {
                    "actionType": "dock",
                    "actionId": "d10480a3-09d6-4c4a-a77f-52f9bff2ed53",
                    "actionDescription": "task priority pick_place dock_location_info.",
                    "blockingType": "NONE",
                    "actionParameters": [
                        {
                            "key": "landmark",
                            "value": [
                                5,
                                "low",
                                "transport",
                                "C13",
                                "C3",
                                "C2",
                                "C9"
                            ]
                        }
                    ]
                }
            ]
        }
    ],
    "edges": [
        {
            "edgeId": "edge_C12",
            "released": true,
            "sequenceId": 2,
            "startNodeId": "C12",
            "endNodeId": "W12",
            "actions": []
        },
        {
            "edgeId": "edge_W12",
            "released": true,
            "sequenceId": 0,
            "startNodeId": "W12",
            "endNodeId": "C12",
            "actions": []
        },
        {
            "edgeId": "edge_C12",
            "released": true,
            "sequenceId": 2,
            "startNodeId": "C12",
            "endNodeId": "C13",
            "actions": []
        }
    ]
}
[OrderPublisher] NEW ORDER PUBLISHED -> Robot: R02 | Nodes: ['C12', 'W12', 'C12', 'C13']
VisualizationSubscriber: [INFO  ] [2026-05-10 13:37:04] FmTrafficHandler.update_robot_status  - mex_r_id: R01 --> mex estimated wait time -1.0.
[INFO] [2026-05-10 13:37:04,258] OrderPublisher: Publishing Order Message...
[INFO] [2026-05-10 13:37:04,258] OrderPublisher: {
    "headerId": 9,
    "timestamp": "2026-05-10T13:37:04.244118",
    "version": "2.0.0",
    "manufacturer": "birfen",
    "serialNumber": "R01",
    "orderId": "bf1237c3-f141-4720-92f8-3901c0ed47d0_9",
    "orderUpdateId": 9,
    "zoneSetId": "kullar",
    "nodes": [
        {
            "nodeId": "C13",
            "released": true,
            "sequenceId": 1,
            "nodeDescription": "current base node.",
            "nodePosition": {
                "x": 12.0,
                "y": -6.0,
                "theta": -0.19425138730222646,
                "mapId": "bina_1_floor_0",
                "allowedDeviationXY": 0.5,
                "allowedDeviationTheta": 3.1
            },
            "actions": []
        },
        {
            "nodeId": "C12",
            "released": true,
            "sequenceId": 3,
            "nodeDescription": "Task Priority: high, Task Type: transport, Node Type: Checkpoint, Wait Time: None",
            "nodePosition": {
                "x": 6.0,
                "y": -6.0,
                "theta": -0.39011656541402734,
                "mapId": "bina_1_floor_0",
                "allowedDeviationXY": 0.5,
                "allowedDeviationTheta": 3.1
            },
            "actions": []
        }
    ],
    "edges": [
        {
            "edgeId": "edge_C13",
            "released": true,
            "sequenceId": 2,
            "startNodeId": "C13",
            "endNodeId": "C12",
            "actions": []
        }
    ]
}
[OrderPublisher] NEW ORDER PUBLISHED -> Robot: R01 | Nodes: ['C13', 'C12']
VisualizationSubscriber: [INFO  ] [2026-05-10 13:37:04] FmTrafficHandler.update_robot_status  - r_id: R02 completed negotiation.

>>>>>>>> : END : R02 : >>>>>>>>
>>>>>>>> : START : R01 : >>>>>>>>
>>>>>>>> VisualizationSubscriber: [INFO  ] [2026-05-10 13:37:04] FmTaskHandler.verify_robot_fitness - Robot R01: last node id not a home dock.
>>>>>>>>
>>>>>>>
>>>>>>
>>>>>
>>>>
>>>
>>

1. Rbt : R01 : -> state_base : C13 & last_released : C12 & within_range : False.
   : -> horizon : ['C10', 'C9'].
2. Rbt : R02 : -> state_base : C12 & last_released : C13 & within_range : False.
   : -> horizon : ['C12', 'C10', 'C11', 'C3', 'C11', 'C10', 'C2'].
3. cool.
4. cool.
5. cool.

>>>>>>>> : END : R01 : >>>>>>>>
>>>>>>>> VisualizationSubscriber: [CRITICAL] [2026-05-10 13:37:04] FmMain.run_cycle            - Traffic Control: {R01: ['C12', 'W12'], R02: ['W12', 'C12', 'C13']}.
>>>>>>>>
>>>>>>>
>>>>>>
>>>>>
>>>>
>>>
>>
>

---
## 2026-05-17T16:43 — Resolve RoutePlan unknown type and relocate publish_route_plan to FmTrafficHandler

**Command:**
```
python3 -m py_compile submodules/order.py fleet_management/FmTrafficHandler.py fleet_management/FmTaskHandler.py
```

**Output:**
```
(No output — Compilation successful)
```

**Changes applied:**
1. `submodules/order.py`:
   - Resolved the unresolved name error for `RoutePlan` by:
     - Importing it conditionally within a `from typing import TYPE_CHECKING` guard at the top.
     - Changing the type-hint annotations inside the method signatures of the helper methods (`_merge_trajectory_itinerary`, `_compose_order_geometry`, `_dispatch_order_payload`) to string literal forward references (`"RoutePlan"`).
   - Removed the `publish_route_plan` method.
2. `fleet_management/FmTrafficHandler.py`:
   - Relocated the `publish_route_plan` method to the `FmTrafficHandler` class.
   - Low-level geometry and MQTT dispatch operations within `publish_route_plan` are securely routed through `self.task_handler.order_handler`.
   - Updated all local call chains within `FmTrafficHandler.py` to target `self.publish_route_plan` directly.

---
## 2026-05-17T16:54 — Evolve to Acyclic Dependency Architecture with RoutePlan in order.py

**Command:**
```
python3 -m py_compile submodules/order.py fleet_management/FmTrafficHandler.py fleet_management/FmTaskHandler.py
```

**Output:**
```
(No output — Compilation successful)
```

**Changes applied:**
1. `submodules/order.py`:
   - Defined `RoutePlan` dataclass locally. This makes logical sense as `RoutePlan` is the core data structure representing an order structure that `OrderPublisher` consumes.
   - Cleaned up the `TYPE_CHECKING` guard imports and fully restored direct type annotations `plan: RoutePlan` in method signatures (`_merge_trajectory_itinerary`, `_compose_order_geometry`, `_dispatch_order_payload`).
2. `fleet_management/FmTrafficHandler.py`:
   - Removed the duplicate `RoutePlan` class definition.
   - Imported `RoutePlan` directly from `order` submodule, establishing a perfect linear, acyclic dependency model without circular imports.

---
## 2026-05-18T13:23 — Verify modified files compilation

**Command:**
```
python3 -m py_compile submodules/order.py fleet_management/FmTrafficHandler.py fleet_management/FmTaskHandler.py
```

**Output:**
```
(No output — Compilation successful)
```

**Changes applied:**
1. `submodules/order.py`:
   - Surgically adjusted off-by-one verification check in `build_order_msg` to `len(h_edges) >= released_count - 1` to support empty remaining edges.
   - Updated the test `__main__` block to use `RoutePlan` constructor and pipeline correctly.
2. `fleet_management/FmTrafficHandler.py`:
   - Standardized `fetch_mex_data` early return key (`traffic_control` -> `traffic_control_dict`).
   - Integrated check `if not fb_rec or not fb_rec.get("robot_data"): return [], [], {}, None` to prevent KeyErrors on cold cache.
3. `fleet_management/FmTaskHandler.py`:
   - Imported `RoutePlan` from `order`.
   - Upgraded `fm_file_task` to correctly construct `RoutePlan` and call `create_order` / `build_order_msg` matching VDA-5050 architecture.

---
## 2026-05-18T13:25 — Run OpenFMS N2 Simulation

**Command:**
```
./run_openfms.sh N2
```

**Output:**
```
================================================================
🧹 Pre-flight: cleaning stale containers and analytics files...
================================================================
   ✓ No lingering openfms containers found
   ✓ Stale analytics logs cleared

================================================================
🏃 MODE 1: Automated Scenario (N2)
================================================================
[1/4] 🗺️  Generating Map Topology for 'N2'...
🏗️  Building isolated Docker images (cached layer enforcement)...
docker compose -p openfms_v2 run --rm scenario python3 fleet_management/FmInterface.py generate "N2"
[FmInterface] Starting imports...
[FmInterface] Imports complete.
--- Generating Procedural Graph for 2 Robots ---
Goal ~49 nodes → grid 7×8 = 56 nodes
 → expected checkpoints before docks: 45
Pruned 5 deg-1 checkpoint leaves (checkpoint-only)
Pruned 1 completely isolated or dock-only checkpoints
All dock entry checkpoints verified: connected to at least one other checkpoint
Created/updated config.yaml → /app/fleet_management/../config/config.yaml
Plot saved → /app/fleet_management/../config/N2_grid_layout.png
   🔧 Patching config.yaml for Docker networking (localhost → db/mqtt)...
[2/4] 🚀 Starting MQTT, PostgreSQL, and Robot Simulator...
Container openfms_v2-db-1 Running
Container openfms_v2-mqtt-1 Running
Container openfms_v2-simulator-1 Started
   Waiting for PostgreSQL to be ready...
   ✅ PostgreSQL is ready!

[4/4] 🎮 Launching FmInterface (Fleet Manager & Dispatcher)...
   📋 Live output is also saved to: logs/FmLogHandler.log
[FmInterface] Starting imports...
[FmInterface] Imports complete.
🚀 FmScheduleHandler Loaded - Cumulative Snapshot Version: 2026-04-26-v1

--- Initializing Fleet Manager for Scenario: N2 ---
Connected to database postgres at db
table maps dropped successfully.
Maps table created successfully.
VisualizationSubscriber: [CRITICAL] [2026-05-18 10:24:11] FmMain.mqtt_connect         - connection done.
VisualizationSubscriber: [CRITICAL] [2026-05-18 10:24:11] FmMain.on_mqtt_connect      - Connected to MQTT broker with result code Success.
...
>>>>>>>> : START : R02 : >>>>>>>>
VisualizationSubscriber: [INFO  ] [2026-05-18 10:25:53] FmTaskHandler.verify_robot_fitness - Robot R02: appears to already be on a task.
1. Rbt : R01 : -> state_base : C12 & last_released : C11 & within_range : True.
       : -> horizon : ['C10', 'C9', 'C1', 'C9', 'C10', 'C11', 'C12', 'C20'].
1. Rbt : R02 : -> state_base : C29 & last_released : C30 & within_range : False.
       : -> horizon : ['C31', 'C32', 'C24', 'C16', 'C24', 'C32', 'C31', 'C30', 'C29', 'C28', 'C27', 'C19', 'C11', 'C12', 'C13', 'C14', 'C13', 'C12', 'C11', 'C19', 'C27', 'C28', 'C29', 'C21'].
VisualizationSubscriber: [INFO  ] [2026-05-18 10:25:53] FmTrafficHandler._handle_robot_traffic_status - 
R_id: R02
[Checkpoint] Reserved: C29 → Next: C31
Status: green | Docked: True
Horizon: ['C31', 'C32', 'C24', 'C16', 'C24', 'C32', 'C31', 'C30', 'C29', 'C28', 'C27', 'C19', 'C11', 'C12', 'C13', 'C14', 'C13', 'C12', 'C11', 'C19', 'C27', 'C28', 'C29', 'C21']

>>>>>>>> : END : R02 : >>>>>>>>
>>>>>>>> : START : R01 : >>>>>>>>
VisualizationSubscriber: [INFO  ] [2026-05-18 10:25:53] FmTaskHandler.verify_robot_fitness - Robot R01: appears to already be on a task.
>>>>>>>> : END : R01 : >>>>>>>>
VisualizationSubscriber: [CRITICAL] [2026-05-18 10:25:53] FmMain.run_cycle            - Traffic Control: {R01: ['C11'], R02: ['C30']}.
[DEBUG] Writing dashboard loop trace - found 1 snapshots. Array has 54 lines.

================================================================
✅ Scenario complete.
   🔍 RealTime Nav:    docker compose -p openfms_v2 up dashboard
   📋 Output log:     cat logs/FmLogHandler.log
   🔍 Simulator feed: docker compose -p openfms_v2 logs -f simulator
   🔍 Analytics:      cat logs/result_snapshot.txt
   🛑 Stop all:       ./kill_openfms.sh
================================================================
```

---
## 2026-05-18T13:26 — Tear Down and Clean Up

**Command:**
```
./kill_openfms.sh
```

**Output:**
```
================================================================
💀 kill_openfms.sh — Killing OpenFMS completely
================================================================
[1/5] Stopping and removing compose services...
   ✓ compose down done
[2/5] Force-stopping any lingering openfms containers...
   ✓ Stopped: ebcce9cd2607 3278f1f2ce4d b1d4a8bcc900 91174e688b24 7a9eade39321
[3/5] Removing all openfms containers (including dead ones)...
   ✓ Removed: ebcce9cd2607 3278f1f2ce4d b1d4a8bcc900 91174e688b24 7a9eade39321
[4/5] Removing openfms networks...
   ✓ Networks removed
[5/5] Verifying — checking for any surviving openfms resources...
   ✓ All clear — no openfms containers remain

================================================================
✅ OpenFMS is dead.
================================================================
```

[2026-05-23] Command Execution Log
Command: `git config --global credential.helper store`
Output: (silent)

Command: `git add README.md config/N2_grid_layout.png fleet_management/FmInterface.py fleet_management/FmMain.py fleet_management/FmRobotSimulator.py fleet_management/FmScheduleHandler.py fleet_management/FmSimGenerator.py fleet_management/FmTaskHandler.py fleet_management/FmTrafficHandler.py submodules/order.py submodules/visualization.py project_report.md output_log.md`
Output: (silent)

Command: `git rm config/N10_grid_layout.png`
Output: (silent)

Command: `git commit -m "Update OpenFMS traffic handling, visualization, and trajectory planning"`
Output: (Commit hash output)

Command: `git push origin main`
Output: (Push to GitHub output)
