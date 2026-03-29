#!/bin/bash

# OpenFMS Unified Test & Run Script
# ======================================================
# MODE 1 (default): Custom or Automated Scenario via 
#   FmInterface Generates a map topology, boots infra, 
#   and runs a scripted task scenario end-to-end.
#     Usage:  ./run_openfms.sh S1   (Run S1 or S2, S3..)
#     Usage:  ./run_openfms.sh N20  (Spawn 20 amr or N99..)
#
# MODE 2: Interactive FmMain (Direct Terminal Control)
#   Boots infra and the manager container, then attaches
#   to it so you can dispatch orders, pause, cancel, and
#   manage the fleet from the live terminal menu.
#     Usage:  ./run_openfms.sh --interactive
# ======================================================

# ── Parse arguments ─────────────────────────────────────
if [ "$1" == "--interactive" ] || [ "$1" == "-i" ]; then
    MODE=interactive
else
    MODE=scenario
    # If no argument is provided, default to N2 (Procedural 2-robot setup)
    # because the Python FmInterface now rejects the 'random' keyword.
    SCENARIO=${1:-N2}
fi

# ── Shared pre-flight: wipe stale containers and logs ───
# Done at startup (not shutdown) to guarantee a clean state
# even when the previous run crashed or was not properly killed.
echo "================================================================"
echo "🧹 Pre-flight: cleaning stale containers and analytics files..."
echo "================================================================"

# Tear down any running compose services and orphan containers
docker compose down --remove-orphans > /dev/null 2>&1

# Force-remove any lingering openfms containers from previous broken runs
STALE=$(docker ps -aq --filter "name=openfms")
if [ -n "$STALE" ]; then
    docker rm -f $STALE > /dev/null 2>&1
    echo "   ✓ Removed stale containers"
else
    echo "   ✓ No stale containers found"
fi

# Wipe analytics snapshot files and the live dashboard.
# Without this, the dashboard reads snapshots from previous runs,
# e.g. showing 3 robots when this run only has 2.
rm -f logs/result_snapshot_*.txt logs/live_dashboard.txt 2>/dev/null
echo "   ✓ Stale analytics logs cleared"
echo ""

# ════════════════════════════════════════════════════════
# MODE 1 — Automated Scenario
# ════════════════════════════════════════════════════════
if [ "$MODE" == "scenario" ]; then

    echo "================================================================"
    echo "🏃 MODE 1: Automated Scenario ($SCENARIO)"
    echo "================================================================"

    echo "[1/4] 🗺️  Generating Map Topology for '$SCENARIO'..."
    # Always rebuild to ensure the latest fms project files are copied into the container without relying on volumes
    echo "🏗️  Building isolated Docker images (cached layer enforcement)..."
    docker compose build
    if [ $? -ne 0 ]; then
        echo "❌ Error: Docker build failed. Aborting."
        exit 1
    fi

    # 'docker compose run' is designed specifically to ignore the command defined in the YAML file.
    # the 'command' in docker-compose.yml service only executes if we use 'docker compose up'.
    docker compose run --rm scenario python3 fleet_management/FmInterface.py generate "$SCENARIO"
    if [ $? -ne 0 ]; then
        echo "❌ Failed to generate map. Aborting."
        exit 1
    fi

    echo "   🔧 Patching config.yaml for Docker networking (localhost → db/mqtt)..."
    sed -i 's|broker_address: "localhost"|broker_address: "mqtt"|g' config/config.yaml
    sed -i 's|broker_address: localhost|broker_address: mqtt|g' config/config.yaml
    sed -i 's|host: "localhost"|host: "db"|g' config/config.yaml
    sed -i 's|host: localhost|host: db|g' config/config.yaml

    echo "[2/4] 🚀 Starting MQTT, PostgreSQL, and Robot Simulator..."
    docker compose up -d mqtt db simulator
    if [ $? -ne 0 ]; then
        echo "❌ Error: Failed to start infrastructure containers."
        exit 1
    fi

    echo "   Waiting for PostgreSQL to be ready..."
    READY=1
    for i in {1..30}; do
        docker compose exec db pg_isready -U postgres > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ PostgreSQL is ready!"
            READY=0
            break
        fi
        echo -n "."
        sleep 2
    done

    if [ $READY -ne 0 ]; then
        echo ""
        echo "❌ PostgreSQL failed to initialize within 60 seconds. Aborting."
        exit 1
    fi

    echo ""
    echo "[4/4] 🎮 Launching FmInterface (Fleet Manager & Dispatcher)..."
    echo "   📋 Live output is also saved to: logs/FmLogHandler.log"
    docker compose run --rm scenario \
        python3 -u fleet_management/FmInterface.py run "$SCENARIO" 2>&1 | tee logs/FmLogHandler.log

    echo ""
    echo "================================================================"
    echo "✅ Scenario complete."
    echo "   🔍 RealTime Nav:    docker compose up dashboard"
    echo "   📋 Output log:     cat logs/FmLogHandler.log"
    echo "   🔍 Simulator feed: docker compose logs -f simulator"
    echo "   🛑 Stop all:       ./kill_openfms.sh"
    echo "================================================================"

# ════════════════════════════════════════════════════════
# MODE 2 — Interactive FmMain (Direct Terminal Control)
# ════════════════════════════════════════════════════════
elif [ "$MODE" == "interactive" ]; then

    echo "================================================================"
    echo "🎮 MODE 2: Interactive FmMain — Direct Terminal Control"
    echo "================================================================"
    # Always rebuild to ensure the latest fms project files are copied into the container without relying on volumes
    echo "🏗️  Building isolated Docker images (cached layer enforcement)..."
    docker compose build
    if [ $? -ne 0 ]; then
        echo "❌ Error: Docker build failed. Aborting."
        exit 1
    fi

    echo "   🔧 Patching config.yaml for Docker networking (localhost → db/mqtt)..."
    sed -i 's|broker_address: "localhost"|broker_address: "mqtt"|g' config/config.yaml
    sed -i 's|broker_address: localhost|broker_address: mqtt|g' config/config.yaml
    sed -i 's|host: "localhost"|host: "db"|g' config/config.yaml
    sed -i 's|host: localhost|host: db|g' config/config.yaml
    echo "[1/2] 🚀 Booting MQTT, PostgreSQL, Robot Simulator, and Fleet Manager..."
    docker compose up -d mqtt db simulator manager
    if [ $? -ne 0 ]; then
        echo "❌ Error: Failed to start containers."
        exit 1
    fi

    echo "   Waiting for PostgreSQL to be ready..."
    READY=1
    for i in {1..30}; do
        docker compose exec db pg_isready -U postgres > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ PostgreSQL is ready!"
            READY=0
            break
        fi
        echo -n "."
        sleep 2
    done

    if [ $READY -ne 0 ]; then
        echo ""
        echo "❌ PostgreSQL failed to initialize within 60 seconds. Aborting."
        exit 1
    fi

    echo ""
    echo "[2/2] 🔗 Attaching to FmMain interactive terminal..."
    echo ""
    echo "================================================================"
    echo "📋 FLEET MANAGER — TERMINAL MENU GUIDE"
    echo "================================================================"
    echo "  Once attached, the FmMain menu lets you:"
    echo ""
    echo "  [1] Dispatch a new task (robot, from_loc, to_loc, task_type,"
    echo "      priority, payload)"
    echo "  [2] Pause / Resume a robot mid-task"
    echo "  [3] Cancel a task (robot returns home)"
    echo "  [4] Ignore a specific robot from the management cycle"
    echo "  [5] Add / remove map nodes and landmarks"
    echo "  [6] Manage mutex groups (shared exclusive path sections)"
    echo "  [q] Quit the manager"
    echo ""
    echo "  ⌨️  Input format follow on-screen prompts (e.g. node IDs"
    echo "      must match pattern: C or W + number e.g. C3, W12)"
    echo ""
    echo "  🔓 To DETACH without stopping: press Ctrl+P, then Ctrl+Q"
    echo "  🛑 To STOP everything after detaching: ./kill_openfms.sh"
    echo "================================================================"
    echo ""
    echo "Attaching in 3 seconds..."
    sleep 3

    docker attach openfms-manager-1

    echo ""
    echo "================================================================"
    echo "👋 Detached from FmMain."
    echo "   🔍 Manager logs:    docker compose logs -f manager"
    echo "   🔍 Simulator feed:  docker compose logs -f simulator"
    echo "   🔍 RealTime Nav:    docker compose up dashboard"
    echo "   🛑 Stop all:        ./kill_openfms.sh"
    echo "================================================================"
fi