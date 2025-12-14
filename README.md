Based on the code analysis of the files provided in the repository structure, here is the comprehensive **README.md** and **requirements.txt**.

-----

### 1\. Detailed Documentation

Create a file named `README.md`. This documentation reflects the actual architecture found in the `fleet_management` scripts, including the fuzzy logic dispatcher, traffic negotiation, and VDA5050-style messaging.

````markdown
# OpenFMS (Open Source Fleet Management System)

OpenFMS is a Python-based Fleet Management System designed to control, schedule, and route Autonomous Guided Vehicles (AGVs) or mobile robots. It utilizes a VDA5050-compliant messaging structure over MQTT, integrates with a PostgreSQL database for state persistence, and features a fuzzy-logic-based task dispatcher and a sophisticated traffic conflict resolution system.

## 📂 Project Structure

The codebase is organized into the following core modules:

```text
OpenFMS/
├── config.yaml                     # Central configuration (Graph, MQTT, DB settings)
├── fleet_management/
│   ├── FmMain.py                   # Entry point: Initializes handlers & terminal GUI
│   ├── FmTaskHandler.py            # Task generation, path planning, & fuzzy allocation
│   ├── FmTrafficHandler.py         # Collision avoidance & deadlock resolution
│   ├── FmScheduleHandler.py        # Lifecycle management, analytics, & auto-charging
│   ├── FmRobotSimulator.py         # VDA5050 Robot Simulator for testing
│   ├── FmInterface.py              # Script to run automated simulation scenarios
│   └── submodules/                 # DB & MQTT wrappers (Order, State, Factsheet, etc.)
└── requirements.txt                # Python dependencies
````

## 🚀 Key Features

  * **Fuzzy Logic Task Dispatching**: Uses `scikit-fuzzy` to assign tasks based on variables like battery level, idle time, travel distance, and payload efficiency.
  * **Traffic Management**: Handles multi-robot traffic conflicts including:
      * **Cross Conflicts**: Priority-based yielding at intersections.
      * **Last-Mile Conflicts**: Checks availability of destination docks before release.
      * **Deadlock Prevention**: Uses wait-points (`Wx` nodes) to allow robots to yield.
  * **VDA5050 Compliance**: Communication via MQTT using standard topics (`order`, `state`, `factsheet`, `instantAction`).
  * **PostgreSQL Integration**: Persists robot states, active orders, and map data.
  * **Analytics & Notifications**: Tracks fleet throughput, latency, and idle time; sends SMS alerts via Twilio for critical errors.
  * **Built-in Simulator**: Includes `FmRobotSimulator.py` to simulate robot kinematics and VDA5050 messaging without physical hardware.

## 🛠️ Installation

### Prerequisites

1.  **Python 3.8+**
2.  **PostgreSQL**: Ensure a database (default: `postgres`) is running.
3.  **MQTT Broker**: Mosquitto or similar (default: `localhost:1883`).

### Setup

1.  Clone the repository.
2.  Install dependencies:
    ```bash
    pip install -r requirements.txt
    ```
3.  **Database Setup**: The system assumes a PostgreSQL database exists. Ensure the credentials in `config.yaml` match your local setup. The tables are auto-generated/managed by the submodule handlers (based on `FmTaskHandler.py` logic).

## ⚙️ Configuration (`config.yaml`)

The `config.yaml` file defines the fleet environment. Key sections:

  * **`mqtt`**: Broker address and port.
  * **`postgres`**: Database credentials.
  * **`graph`**: Defines the node connectivity and edge weights (distances).
  * **`itinerary`**: Defines node properties (coordinates, types like `station_dock`, `charge_dock`, `waitpoint`).
  * **`maps`**: links `.pgm` and `.yaml` map files to fleet IDs.

## 🏃 Usage

### 1\. Running the Fleet Manager

This starts the central control server. It opens an interactive terminal GUI for monitoring fleets and manually issuing tasks.

```bash
python3 fleet_management/FmMain.py
```

  * **Interactive Menu**: Allows you to send `transport`, `move`, or `charge` tasks, pause robots, or manage landmarks.

### 2\. Running the Robot Simulator

Simulates one or more robots (AGVs) that connect to the manager via MQTT.

```bash
python3 fleet_management/FmRobotSimulator.py
```

  * *Note*: You can configure multiple robots inside the `main()` function of this script.

### 2\. Running an Automated Scenario

To run a predefined scenario (dispatching specific tasks at specific times):

```bash
python3 fleet_management/FmInterface.py
```

## 🧠 System Modules Explained

### `FmMain.py`

The orchestrator. It establishes DB/MQTT connections and spins up the `FmScheduleHandler`. It handles the main loop that triggers periodic traffic checks.

### `FmTaskHandler.py`

The "Brain".

  * **Path Planning**: Implements shortest-path algorithms to generate routes.
  * **Fuzzy Dispatcher**: A class `FuzzyTaskDispatcher` that calculates a "fitness" score (0.0 - 1.0) for every robot candidate for a task.
  * **Order Generation**: Converts routes into VDA5050 `Order` messages containing nodes and edges.

### `FmTrafficHandler.py`

The "Police".

  * It monitors the `state` of all robots.
  * **`manage_traffic()`**: Checks if a robot's next node is occupied.
  * If a conflict is detected, it negotiates based on task priority. Lower priority robots are sent to associated **Waitpoints** (`W` nodes) or instructed to wait.

### `FmScheduleHandler.py`

The "Manager".

  * Monitors battery levels and triggers auto-charging if below threshold.
  * Manages the "Unassigned Task Queue". If no robot is fit for a task, it queues it and retries later.
  * Generates analytics reports (saved to `logs/`).

## 📊 Analytics

The system automatically logs performance metrics to the `logs/` directory, including:

  * Throughput (Orders/hour)
  * Average Latency per robot
  * Fleet Idle time metrics

-----

*Documentation generated based on source code analysis.*

```
```





[OUTDATED INFO]

## USAGE:

### Terminal 1:
launch the viro simple fleet client with the below command after you have filled in the necessary information of the `postgresql` db address and other client-specific information in its launch parameter:

```bash
'hostname': "xxx.xxx.xx.xxx"
'database': "postgres"
'username': "postgres"
'pwd': "xxx"
'port': 123
'robot_id': X
# etc...
```

```bash
$ python3 fleet_management/FmMain.py
```

### Terminal 2:
similarly, fill the master/server/manager messaging `twilio` information as well as the same `postgresql` db address, before you run the user command line interface with the below command and follow the prompt. A sample graph; with respective nodes and their connecting edges is auto generated when you use the `fm_add_landmark_request` option to build a graph. the notation `A1`, `A2`, `A3`... are used in identifying 'charge_dock', 'station_dock', 'home_dock', 'waypoint' landmarks and `E1`, `E2` is the expected convention to identify 'elevator_dock'.

```bash
$ python3 fleet_management/FmRobotSimulator.py
```

![CLI1](https://github.com/hazeezadebayo/viro_simple_fleet/blob/main/media/b.png)
![CLI2](https://github.com/hazeezadebayo/viro_simple_fleet/blob/main/media/c.png)

![running](https://github.com/hazeezadebayo/viro_simple_fleet/blob/main/media/a.png)
