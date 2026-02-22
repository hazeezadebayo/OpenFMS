# OpenFMS (Open Source Fleet Management System)

OpenFMS is a Python-based Fleet Management System designed to control, schedule, and route Autonomous Guided Vehicles (AGVs) or mobile robots. It utilizes a VDA5050-compliant messaging structure over MQTT, integrates with a PostgreSQL database for state persistence, and features a fuzzy-logic-based task dispatcher and a sophisticated traffic conflict resolution system.

## 📂 Project Structure

The codebase is organized into the following core modules:

```text
OpenFMS/
├── config.yaml                     # Central configuration (Graph, MQTT, DB settings)
├── docker-compose.yml              # Cluster orchestration
├── fleet_management/
│   ├── FmMain.py                   # Entry point: Initializes handlers & terminal GUI
│   ├── FmTaskHandler.py            # Task generation, path planning, & fuzzy allocation
│   ├── FmTrafficHandler.py         # Collision avoidance & deadlock resolution
│   ├── FmScheduleHandler.py        # Lifecycle management, analytics, & auto-charging
│   ├── FmRobotSimulator.py         # VDA5050 Robot Simulator for testing
│   └── FmInterface.py              # Script to run automated simulation scenarios
├── submodules/                     # DB & MQTT wrappers (Order, State, Factsheet, etc.)
└── requirements.txt                # Python dependencies
```

## 🚀 Key Features

*   **Fuzzy Logic Task Dispatching**: Uses `scikit-fuzzy` to assign tasks based on variables like battery level, idle time, travel distance, and payload efficiency.
*   **Traffic Management**: Handles multi-robot traffic conflicts including:
    *   **Cross Conflicts**: Priority-based yielding at intersections.
    *   **Last-Mile Conflicts**: Checks availability of destination docks before release.
    *   **Deadlock Prevention**: Uses wait-points (`Wx` nodes) to allow robots to yield.
*   **VDA5050 Compliance**: Communication via MQTT using standard topics (`order`, `state`, `factsheet`, `instantActions`).
*   **PostgreSQL Integration**: Persists robot states, active orders, and map data.
*   **Analytics & Notifications**: Tracks fleet throughput, latency, and idle time; sends SMS alerts via Twilio for critical errors.
*   **Built-in Simulator**: Includes `FmRobotSimulator.py` to simulate robot kinematics and VDA5050 messaging without physical hardware.

## ⚙️ Configuring Your Fleet (`config.yaml`)

The `config.yaml` is the heart of the system. Here is how to replicate your environment:

### 1. Infrastructure Settings
*   **`mqtt`**: Connection details for your broker.
*   **`fleet_info`**: Metadata identifying your fleet (e.g., `kullar`).
*   **`postgres`**: Database credentials for state persistence.
*   **`twilio_server`**: (Optional) SMS alerting settings.

### 2. The Logic Layers
*   **`graph`**: Defines connectivity. Every node (key) lists its neighbors and the distance (cost) between them.
    *   *Convention*: `Nx: [[Ny, cost], ...]`
*   **`itinerary`**: Defines the physical properties of nodes.
    *   **`coordinate`**: `[x, y, theta, w]` (Quaternion orientation).
    *   **`description`**: Defines node behavior.
        *   `checkpoint`: Standard path node.
        *   `waitpoint`: Safe yielding spot for traffic resolution.
        *   `home_dock`: Where robots park when idle.
        *   `charge_dock`: Automated charging station.
        *   `station_dock`: Fixed loading/unloading point.
        *   `elevator`: Triggers map switching logic.
*   **`maps`**: Links fleet names to specific map files (`.pgm`, `.yaml`).

### 3. Naming Conventions (TL;DR)
To ensure the brain (`FmTaskHandler`) and police (`FmTrafficHandler`) work correctly:
-   **`Cx`**: **Checkpoints**. General nodes used for navigation.
-   **`Wx`**: **Waitpoints**. Must be associated with a nearby `Cx` node to allow priority-based yielding.
-   **`Ex`**: **Elevators**. Required for multi-floor navigation and dynamic map loading.

## 🗺️ Dynamic Map Handling (TL;DR)

*   **Robots start naked**: They have no maps and no idea where they are initially (Tabula Rasa).
*   **Just-in-Time Maps**: The manager issues a `downloadMap` action (with a download link) whenever a task requires a specific map.
*   **Navigation**: The simulator downloads and uses the map from the link to navigate. No map = no navigation.
*   **Elevator Switching**: Moving between floors via elevators automatically triggers map changes.
*   **Config Alert**: Users **must** populate the `maps` field in `config.yaml` with valid files/links for their specific environment.

## 🗄️ PostgreSQL Setup (TL;DR)

OpenFMS uses PostgreSQL for state persistence, order tracking, and map storage. All configuration is centralized in `config.yaml`.

### 1. Configure Connection
Edit the `postgres` block in `config.yaml` to point to your database:
```yaml
postgres:
  host: "localhost"       # Use "db" if running via Docker
  port: "5432"
  database: "postgres"    # The database name
  user: "postgres"
  password: "your_password"
```

### 2. Table Auto-Creation
**No manual SQL required.** The system automatically initializes the following tables on startup:
- `state`: Stores real-time AGV status (battery, position, etc.).
- `orders`: Tracks active and historical VDA5050 orders.
- `maps`: Stores binary map data (.pgm) and metadata (.yaml).

### 3. Docker vs Local
- **Docker**: The `docker compose` stack handles everything. The manager and scenario runners connect to the sibling `db` container automatically.
- **Local**: Ensure a Postgres instance is running and accessible from your terminal.

### 4. Verification
Check if data is being recorded by running:
```bash
docker exec -it openfms-db-1 psql -U postgres -d postgres -c "SELECT * FROM state LIMIT 5;"
```

## 🐳 Docker (Zero to Hero)

The easiest way to run OpenFMS along with all its dependencies (PostgreSQL, Mosquitto MQTT) is using Docker Compose.

### 1. Build and Run
The simplest way to start the environment is using the provided startup script:
```bash
bash run_openfms.sh
```
Alternatively, using direct Docker commands:
```bash
docker compose build
docker compose up -d
```

### 2. Verify Execution
To see the automated scenario in action (tasks being dispatched and processed):
```bash
docker compose logs -f scenario
```

### 3. Manual vs Automated Mode (Dynamic Tasks)
By default, the system runs an automated script (`FmInterface.py`). To switch to **Interactive Manual Mode**:

1.  Open `docker-compose.yml`.
2.  **Comment out** the `scenario` service.
3.  **Uncomment** the `manager` service.
4.  Restart the stack:
    ```bash
    docker compose up -d --remove-orphans
    ```
5.  **Attach to the Manager**: Since the manager needs player input for its terminal GUI, you must attach to it:
    ```bash
    docker attach openfms-manager-1
    ```
    *Note: Use `Ctrl+P, Ctrl+Q` to detach without killing the process.*

### 4. Stopping the Simulation
To stop all running services and remove the containers:
```bash
docker compose down
```

-----


![CLI1](https://github.com/hazeezadebayo/viro_simple_fleet/blob/main/media/b.png)
![CLI2](https://github.com/hazeezadebayo/viro_simple_fleet/blob/main/media/c.png)

![running](https://github.com/hazeezadebayo/viro_simple_fleet/blob/main/media/a.png)

*Social: [OpenFMS on GitHub](https://github.com/hazeezadebayo/viro_simple_fleet)*



1. i made an assumption that all checkpoints would have a waitpoint. which is not a bad assumption but can not always be true, as is the case for many factory floors. lets create a logic for mutex groups. what this means is that nodes can now be grouped together e.g a list of list that just holds group of nodes: [[c1,c24,c56, c27],[c3, c5, c19],..] with the idea being that if a robot is occupying say c1, then because c1 is in the mutex group of c1 c24, c56 and c27, then we mark those as occupied as well. i.e. if another robot wishes to reserve c56 for instance, then it would be rejected until the robot that occupies c1 leaves. this guarantees that while not all checkpoints have a waitpoint navigation and conflict is closely monitored and solved.

please examine the function "_handle_robot_traffic_status" so that you understand deeply how you may add your new patch function and where to call it.


2. furthermore, since we now agree that not all checkpoints will have a waitpoint. it also makes sense that robot that occupies checkpoints that are not in a mutex group but that another robot wishes to use or reserve said node, perhaps the other robot couldnt find an alternative route. it makes sense for either robot, a. to be able to peep the graph, b. check which nodes are the neighbours of the node they currently reserve. c. find which amongst them is unoccupied. d. publish a 3 goal at once message just like we do for a wait case wherein we; Cx Wx then back to Cx where x is any integer id of the node. in the case, it would go to Cx Cy Cx where cy is the free neighbour node it found. by making this journey cx cy cx, it is guaranteed to give way to the uncoming robot to pass. e. traffic or conflict avoided and navigation can continue as expected.

for all of the above concerns you are not allowed to remove or alter any codebase as they have been validated and tested extensively. you are to write a patch and add to the existing areas wherein these concerns can be addressed perfectly.

ensure to validate your logic with "if __name__ == "__main__":" section of each script you upgrade and use docker as we dont wanna use the host machine.


 Traffic Management Patches: Mutex Groups & Neighborhood Yielding
Proposed Additions
1. Dynamic Mutex Groups (Node Blocking)
[MODIFY] 
FmTrafficHandler.py
Introduce self.mutex_groups = [] inside 
init
.
Add two new API methods: fm_add_mutex_groups(self, group_list) and fm_remove_mutex_group(self, group_list).
Inside 
_handle_robot_traffic_status
, immediately after parsing traffic_control, extend it dynamically:
python
# --- MUTEX GROUP PATCH ---
expanded_traffic = list(traffic_control)
for group in self.mutex_groups:
    if any(node in traffic_control for node in group):
        expanded_traffic.extend([n for n in group if n not in expanded_traffic])
traffic_control = expanded_traffic
[MODIFY] 
FmMain.py
Update the options array in 
interactive_robot_fleet_startup
 to include 'fm_add_mutex_groups' and 'fm_remove_mutex_group'.
Implement user 
input
 validation loops identically to 
fm_add_landmark_request
 to parse lists like ["C1", "C2", "C3"] securely from terminal and pass them to the new TrafficHandler APIs.
2. Neighborhood Yielding (Dynamic Waitpoints)
[MODIFY] 
FmTrafficHandler.py
As you perfectly identified, forcefully injecting a Cy checkpoint ID directly into downstream functions built explicitly for Wy nodes will explode the internal regex 
check_waitpoint_association
 checkers. To guarantee safe injection:

Inside 
handle_priority_higher
, when mex_wait_traffic is None (meaning no Wx exists for mex_base), we will execute a clean fallback maneuver:

Retrieve graph neighbors safely: graph = self.task_handler.build_graph(self.task_dictionary).
Extract all adjacent nodes of mex_base from the graph.
Select an isolated neighbor (Cy) that is unoccupied (not in expanded_traffic).
The Security Trick: Generate its physical layout using normal node API (cy_itinerary = self.task_handler.fm_get_itinerary([Cy], self.task_dictionary)), BUT programmatically rewrite the variable name passed downstream back into Wx format immediately before appending!
python
numeric_part = ''.join(filter(str.isdigit, Cy))
spoofed_wy = f'W{numeric_part}'
mex_waitpoints.insert(0, spoofed_wy)
mex_wait_itinerary.insert(0, cy_itinerary[0])
mex_wait_traffic = spoofed_wy
By temporarily spoofing the label Cy -> Wy after fetching physical coordinates, the internal 
check_waitpoint_association
 regex checker perfectly validates spoofed_wy, seamlessly displacing the robot to Cy's spatial coordinates safely under the Wy label!

Verification Plan
Automated Testing
Spin up docker compose up -d scenario.
Send an array to fm_add_mutex_groups in the interactive console.
Run test routines monitoring 
live_dashboard.txt
 to witness Cx -> Cy -> Cx displacement without system faults.


1. i wanna track collisions if they ever exists and this is easily detected if two or more robots have been assigned the same target, then flag this and ensure that it is set to a new variable like collision_tracker or something and then ensure that this is written in the dashboard so that we can know how many collisions have occured in during the test runs etc. this is an analytics worth pursuing.

2. no robot should be coloured white since white is the coulour of the graphs and nodes. this makes it difficult to find. ensure all robots have colours that are not white. so that robots are visible.

3. confirm that robots under maintenance  which user marks to be in the ignore_list are excluded truly excluded in planning and not altering anything else.

/home/azeez/ws/dev_env/py_code/projects/phd/OpenFMS/fleet_management/FmTaskHandler.py


how about if we get traffic. since traffic control represent either approved targets or occupied targets then we could make them light up. the only problem is we wont know who desired to visit said goal. i.e. is it robot A  that wants to got to C1 or robot B. you get what i mean


after a waitpoint case was observed, the robot that was tasked with going to wait at the waitpoint, either waited forever, or was not issued a new order or new release node afterwards. this was not the case before. please examine carefully and fix the problem

you can create a script whose job would be to test different types of conflicts and examine systems behaviour for 2 robot case and 3 robot case.

for 2 robot case,
1. what happens if robot A wants to reserve same node as robot B. and both have different base nodes.
2. what happens if robot A's reserved node or base node is the target or next node of robot B. 
3. what happens if robot A and robot be desires each others positions as target i.e a swap is expect. how does it happen:
a. does one find a free neighbour checks if its a checkpoint and uses it as a temporary wait area. 
b. does it use an actual waitpoint if it had been available
c. how does a mutex group help this or prevent this.
d. does navigation continue after any of these decisions or system freezes?

can you do a thorough test of the fleet manager and its decision. it was all working before. please write a report of your answer in a .md file and each scenario tested and what was the log observed conclusion.

furthermore, avoid interpretations that do not exist only interpretations with concrete proof are allowed. 



not 2 things, because think about it like this:

B occupies Cx so cx shows in the trafficcontrol. then A occupies Cy for which B targets or desires to reserve. SInce A has access to a waitpoint or a temporary free checkpoint. A may yield, and that means Since traffic is currently [Cy, Cx] we can automatically grant B the node Cy as its goal, while we send A to Wy. this temporary implies that Cx is unoccupied. 

if A desires Cx, we keep Cx in the traffic control and grant A a goal Wy Cy Cx . this is essentially an exchange.
because we dont want another robot to reserve Cx while a waits at Wy. that would lead to an endless loop of traffic. so we leave Cx in the traffic if an exchange was desired. however, if A's next node is not Cx, then we remove Cx from the traffic control since its actually unoccupied.






