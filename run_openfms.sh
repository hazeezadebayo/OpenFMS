#!/bin/bash

# OpenFMS Unified Test & Run Script
PROJ="openfms"
COMPOSE_FILE="docker/docker-compose.yml"

show_help() {
    echo "================================================================"
    echo " OpenFMS Management Script"
    echo "================================================================"
    echo "Usage: ./run_openfms.sh <command> [options]"
    echo ""
    echo "Commands:"
    echo "  build   Build the docker images"
    echo "  up      Start the containers in scenario or interactive mode i.e. takes extra argument"
    echo "          Example: ./run_openfms.sh up S1"
    echo "          Example: ./run_openfms.sh up S1 -log dashboard"
    echo "          Example: ./run_openfms.sh up --interactive"
    echo "          Example: ./run_openfms.sh up dashboard"
    echo "  clean   Remove containers and orphans, and clear analytics logs"
    echo "  kill    Stop and remove all containers completely (nuclear shutdown)"
    echo "  help    Show this help message"
    echo "================================================================"
}

cmd_clean() {
    echo "================================================================"
    echo "🧹 Cleaning stale containers and analytics files..."
    echo "================================================================"
    docker compose -f $COMPOSE_FILE -p $PROJ down --remove-orphans > /dev/null 2>&1
    
    STALE=$(docker ps -aq --filter "label=com.docker.compose.project=$PROJ")
    if [ -n "$STALE" ]; then
        docker rm -f $STALE > /dev/null 2>&1
        echo "   ✓ Removed all lingering $PROJ containers"
    else
        echo "   ✓ No lingering $PROJ containers found"
    fi
    rm -f logs/result_snapshot*.txt logs/live_dashboard.txt 2>/dev/null
    echo "   ✓ Stale analytics logs cleared"
}

cmd_kill() {
    echo "================================================================"
    echo "💀 kill_openfms.sh — Killing OpenFMS completely"
    echo "================================================================"
    
    echo "[1/5] Stopping and removing compose services..."
    docker compose -f $COMPOSE_FILE -p $PROJ down --remove-orphans --volumes 2>/dev/null
    echo "   ✓ compose down done"
    
    echo "[2/5] Force-stopping any lingering openfms containers..."
    RUNNING=$(docker ps -q --filter "name=openfms")
    if [ -n "$RUNNING" ]; then
        docker stop $RUNNING
        echo "   ✓ Stopped: $RUNNING"
    else
        echo "   ✓ No running openfms containers found"
    fi
    
    echo "[3/5] Removing all openfms containers (including dead ones)..."
    ALL=$(docker ps -aq --filter "name=openfms")
    if [ -n "$ALL" ]; then
        docker rm -f $ALL
        echo "   ✓ Removed: $ALL"
    else
        echo "   ✓ No openfms containers to remove"
    fi
    
    echo "[4/5] Removing openfms networks..."
    NETS=$(docker network ls --filter "name=openfms" -q)
    if [ -n "$NETS" ]; then
        docker network rm $NETS 2>/dev/null && echo "   ✓ Networks removed" \
            || echo "   ⚠ Some networks still in use. Try: docker network prune"
    else
        echo "   ✓ No openfms networks found"
    fi
    
    echo "[5/5] Verifying — checking for any surviving openfms resources..."
    LEFTOVER=$(docker ps -aq --filter "name=openfms")
    if [ -n "$LEFTOVER" ]; then
        echo "   ⚠ WARNING: These containers could not be removed:"
        docker ps -a --filter "name=openfms" --format "   → {{.Names}} ({{.Status}})"
    else
        echo "   ✓ All clear — no openfms containers remain"
    fi
    echo "================================================================"
    echo "✅ OpenFMS is dead."
    echo "================================================================"
}

cmd_build() {
    echo "================================================================"
    echo "🏗️  Building OpenFMS Image..."
    echo "================================================================"
    docker compose -f $COMPOSE_FILE -p $PROJ build
    if [ $? -ne 0 ]; then
        echo "❌ Error: Docker build failed. Aborting."
        exit 1
    fi
    
    echo "✅ Build complete."
}

cmd_up() {
    ARG=$1
    shift
    
    if [ "$ARG" == "--interactive" ] || [ "$ARG" == "-i" ]; then
        MODE=interactive
    elif [ "$ARG" == "dashboard" ]; then
        MODE=dashboard
    else
        MODE=scenario
        SCENARIO=${ARG:-N2}
    fi

    LOG_MODE=""
    if [ "$1" == "-log" ]; then
        LOG_MODE=$2
        shift 2
    fi

    if [ "$MODE" != "dashboard" ]; then
        cmd_clean
    fi

    if [ "$MODE" == "scenario" ]; then
        echo "================================================================"
        echo "🏃 MODE 1: Automated Scenario ($SCENARIO)"
        echo "================================================================"
        
        echo "[1/4] 🗺️  Generating Map Topology for '$SCENARIO'..."
        cmd_build
        
        docker compose -f $COMPOSE_FILE -p $PROJ run --rm scenario python3 fleet_management/FmInterface.py generate "$SCENARIO"
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
        # We start mqtt db simulator. We don't abort-on-container-exit here because these are background
        docker compose -f $COMPOSE_FILE -p $PROJ up -d mqtt db simulator
        if [ $? -ne 0 ]; then
            echo "❌ Error: Failed to start infrastructure containers."
            exit 1
        fi
        
        echo "   Waiting for PostgreSQL to be ready..."
        READY=1
        for i in {1..30}; do
            docker compose -f $COMPOSE_FILE -p $PROJ exec db pg_isready -U postgres > /dev/null 2>&1
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
            echo "❌ PostgreSQL failed to initialize. Aborting."
            exit 1
        fi
        
        echo ""
        echo "[4/4] 🎮 Launching FmInterface (Fleet Manager & Dispatcher)..."
        echo "   📋 Live output is also saved to: logs/FmLogHandler.log"
        
        export SCENARIO_NAME=$SCENARIO
        docker compose -f $COMPOSE_FILE -p $PROJ up -d dashboard scenario
        
        echo ""
        echo "================================================================"
        echo "✅ Scenario '$SCENARIO' started in the background."
        echo "   To view the dashboard log:   docker compose -f $COMPOSE_FILE -p $PROJ logs -f dashboard"
        echo "   To view the scenario log:    docker compose -f $COMPOSE_FILE -p $PROJ logs -f scenario"
        echo "   To view the simulator log:   docker compose -f $COMPOSE_FILE -p $PROJ logs -f simulator"
        echo "   🛑 Stop all:                 ./run_openfms.sh kill"
        echo "================================================================"

        if [ "$LOG_MODE" == "dashboard" ]; then
            echo "Attaching to dashboard logs..."
            docker compose -f $COMPOSE_FILE -p $PROJ logs -f dashboard
        elif [ "$LOG_MODE" == "scenario" ]; then
            echo "Attaching to scenario logs..."
            docker compose -f $COMPOSE_FILE -p $PROJ logs -f scenario
        elif [ "$LOG_MODE" == "simulator" ]; then
            echo "Attaching to simulator logs..."
            docker compose -f $COMPOSE_FILE -p $PROJ logs -f simulator
        elif [ "$LOG_MODE" == "manager" ]; then
            echo "Attaching to manager logs..."
            docker compose -f $COMPOSE_FILE -p $PROJ logs -f manager
        fi

    elif [ "$MODE" == "interactive" ]; then
        echo "================================================================"
        echo "🎮 MODE 2: Interactive FmMain — Direct Terminal Control"
        echo "================================================================"
        cmd_build
        
        echo "   🔧 Patching config.yaml for Docker networking (localhost → db/mqtt)..."
        sed -i 's|broker_address: "localhost"|broker_address: "mqtt"|g' config/config.yaml
        sed -i 's|broker_address: localhost|broker_address: mqtt|g' config/config.yaml
        sed -i 's|host: "localhost"|host: "db"|g' config/config.yaml
        sed -i 's|host: localhost|host: db|g' config/config.yaml
        
        echo "[1/2] 🚀 Booting MQTT, PostgreSQL, Robot Simulator, and Fleet Manager..."
        docker compose -f $COMPOSE_FILE -p $PROJ up -d mqtt db simulator manager
        if [ $? -ne 0 ]; then
            echo "❌ Error: Failed to start containers."
            exit 1
        fi
        
        echo "   Waiting for PostgreSQL to be ready..."
        READY=1
        for i in {1..30}; do
            docker compose -f $COMPOSE_FILE -p $PROJ exec db pg_isready -U postgres > /dev/null 2>&1
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
            echo "❌ PostgreSQL failed to initialize. Aborting."
            exit 1
        fi
        
        echo ""
        echo "[2/2] 🔗 Attaching to FmMain interactive terminal..."
        echo ""
        echo "================================================================"
        echo "📋 FLEET MANAGER — TERMINAL MENU GUIDE"
        echo "================================================================"
        echo "  [1] Dispatch a new task"
        echo "  [2] Pause / Resume a robot"
        echo "  [3] Cancel a task"
        echo "  [4] Ignore a robot"
        echo "  [5] Add / remove map nodes"
        echo "  [6] Manage mutex groups"
        echo "  [q] Quit the manager"
        echo ""
        echo "  🔓 To DETACH without stopping: press Ctrl+P, then Ctrl+Q"
        echo "  🛑 To STOP everything: ./run_openfms.sh kill"
        echo "================================================================"
        echo ""
        echo "Attaching in 3 seconds..."
        sleep 3
        
        docker attach ${PROJ}-manager-1
        
        echo ""
        echo "================================================================"
        echo "👋 Detached from FmMain."
        echo "   🛑 Stop all:        ./run_openfms.sh kill"
        echo "================================================================"
    elif [ "$MODE" == "dashboard" ]; then
        echo "================================================================"
        echo "📊 Attaching to Real-time Dashboard..."
        echo "================================================================"
        docker compose -f $COMPOSE_FILE -p $PROJ up dashboard
    fi
}

COMMAND=$1
shift

case "$COMMAND" in
    build)
        cmd_build
        ;;
    up)
        cmd_up "$@"
        ;;
    clean)
        cmd_clean
        ;;
    kill)
        cmd_kill
        ;;
    help|*)
        show_help
        ;;
esac