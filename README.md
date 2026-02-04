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
