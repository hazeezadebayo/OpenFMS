#!/bin/bash

# OpenFMS Unified Test & Run Script
PROJ="openfms"
COMPOSE_FILE="docker/docker-compose.yml"

# Disable Docker Buildx/Bake for compose builds.
# Compose v2 defaults to Bake, and with this project's services all exporting the
# same openfms:latest tag the parallel Bake export races ("image already exists")
# and fails the build spuriously. The classic builder is sequential and cached.
export COMPOSE_BAKE=0

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
    echo "  experiments  Rerun the OpenFMS experiment suite (Docker-only) and"
    echo "          collate analytics into logs/analytics/Open_fms_v2/."
    echo "          Args (any order): sizes as numbers (1..32), publish mode as"
    echo "          throttled|non_throttled|both (default: both). Examples:"
    echo "          ./run_openfms.sh experiments                      (2..8, both modes)"
    echo "          ./run_openfms.sh experiments 2 throttled         # E2, throttled only"
    echo "          ./run_openfms.sh experiments 2 non_throttled     # E2, non-throttled only"
    echo "  clean   Remove containers and orphans, and clear analytics logs"
    echo "  kill    Stop and remove all containers completely (nuclear shutdown)"
    echo "  help    Show this help message"
    echo "================================================================"
}

cmd_experiments() {
    # -------------------------------------------------------------------------
    # Automated experiment suite: reruns OpenFMS simulations and collates the
    # analytics snapshots into logs/analytics/Open_fms_v2/{throttled,non_throttled}/
    #
    # Everything runs inside Docker; nothing is installed on the host.
    #
    # Usage:   ./run_openfms.sh experiments [sizes...] [throttled|non_throttled|both]
    # Positional arguments may be mixed in any order, quoted or not:
    #   ./run_openfms.sh experiments                    (2..8, both modes)
    #   ./run_openfms.sh experiments 2                  (2 only, both modes)
    #   ./run_openfms.sh experiments 2 throttled        (E2, throttled only)
    #   ./run_openfms.sh experiments 2 non_throttled    (E2, non-throttled only)
    #   ./run_openfms.sh experiments 2 4 8 non_throttled (E2 E4 E8, non-throttled only)
    # Env vars:
    #   SIZES        Robot counts to test      (default: "2 3 4 5 6 7 8"; used when no size differs)
    #   MODES        Modes to test             (default: "throttled non_throttled"; used when no mode differs)
    #   TRIALS       Repetitions per (mode,N)  (default: 1)
    #   EXP_TIMEOUT  Hard cap per run, seconds (default: 1400 — matches the
    #                1400 s timeout documented in the paper).
    #                Set EXP_TIMEOUT=0 to wait forever (no timeout).
    # -------------------------------------------------------------------------
    # Positional arguments, any order, words and numbers may be mixed (quoted ok):
    #   numbers                  -> robot counts to test (1..32)
    #   throttled|non_throttled  -> run only that publish mode
    #   both                     -> run both modes (same as default)
    # Example: ./run_openfms.sh experiments 2 throttled     (E2 throttled only)
    #          ./run_openfms.sh experiments 2 8 non_throttled
    ARG_SIZES=""
    ARG_MODES=""
    for tok in $@; do
        case "$tok" in
            *[!0-9]*|'')
                case "$tok" in
                    throttled|non_throttled) ARG_MODES="$ARG_MODES $tok" ;;
                    both)                    ARG_MODES="$ARG_MODES throttled non_throttled" ;;
                    *) echo "❌ Unknown argument '$tok'. Numbers = sizes (1..32); words = throttled|non_throttled|both."
                       exit 2 ;;
                esac
                ;;
            *)
                if [ "$tok" -ge 1 ] && [ "$tok" -le 32 ]; then
                    ARG_SIZES="$ARG_SIZES $tok"
                else
                    echo "❌ Size '$tok' out of range (1..32)."; exit 2
                fi
                ;;
        esac
    done
    ARG_MODES=$(echo "$ARG_MODES" | xargs | tr ' ' '\n' | awk '!seen[$0]++' | tr '\n' ' ' | sed 's/ $//')
    if [ -n "$ARG_SIZES" ]; then SIZES=$(echo "$ARG_SIZES" | xargs); else SIZES=${SIZES:-"2 3 4 5 6 7 8"}; fi
    if [ -n "$ARG_MODES" ]; then MODES="$ARG_MODES"; else MODES=${MODES:-"throttled non_throttled"}; fi
    TRIALS=${TRIALS:-1}
    TRIAL_START=${TRIAL_START:-1}
    # Per-trial RNG seeds (workload variation). One word = reuse for all trials;
    # otherwise must provide >= TRIALS words (trial t uses word t).
    SEEDS=${SEEDS:-43}
    _NW=$(echo "$SEEDS" | wc -w)
    if [ "$_NW" -eq 1 ]; then
        _S=$(echo "$SEEDS" | xargs); SEEDS=""
        for _i in $(seq 1 "$TRIALS"); do SEEDS="$SEEDS $_S"; done
        SEEDS=$(echo "$SEEDS" | xargs)
    elif [ "$_NW" -lt "$TRIALS" ]; then
        echo "❌ SEEDS has $_NW word(s) but TRIALS=$TRIALS (need 1 or >= TRIALS)."; exit 2
    fi
    if [ "$TRIAL_START" -gt "$TRIALS" ]; then echo "❌ TRIAL_START ($TRIAL_START) > TRIALS ($TRIALS)."; exit 2; fi
    EXP_TIMEOUT=${EXP_TIMEOUT:-1400}

    V2_DIR="${OUTDIR:-logs/analytics/Open_fms_v2}"
    mkdir -p "$V2_DIR/throttled" "$V2_DIR/non_throttled"
    MANIFEST="$V2_DIR/run_manifest.txt"

    echo "================================================================"
    echo "🧪 OpenFMS Experiment Suite (Docker-only, host untouched)"
    echo "   Sizes:    $SIZES"
    echo "   Modes:    $MODES"
    echo "   Trials:   $TRIALS"
    echo "   Timeout:  ${EXP_TIMEOUT}s per run"
    echo "   Output:   $V2_DIR/{throttled,non_throttled}/R<N>_result_snapshot.txt"
    echo "   Seeds:    $SEEDS (trial t uses word t)"
    echo "================================================================"

    echo "   Modes are applied by sed-patching fleet_management/FmRobotSimulator.py's hardcoded"
    echo "   sim_publish_type line before each per-mode build, then restored via git checkout"
    echo "   afterwards — the tree is left byte-identical to the remote at exit."

    for MODE in $MODES; do
        if [ "$MODE" != "throttled" ] && [ "$MODE" != "non_throttled" ]; then
            echo "❌ Unknown MODE '$MODE' (use throttled|non_throttled). Skipping."
            continue
        fi

        echo "================================================================"
        echo "🔧 Mode '$MODE': sed-patching simulator publishing mode and building..."
        echo "================================================================"
        sed -i -E "s/^    sim_publish_type = \"(throttled|non_throttled)\"$/    sim_publish_type = \"$MODE\"/" fleet_management/FmRobotSimulator.py
        grep -n "sim_publish_type = " fleet_management/FmRobotSimulator.py
        cmd_build || { echo "❌ Build failed for mode '$MODE'. Restoring simulator file and aborting."; git checkout -- fleet_management/FmRobotSimulator.py; exit 1; }
        # Record the exact image under test for this mode (traceability per run).
        docker images openfms:latest --format '{{.ID}} {{.CreatedAt}}' > "$V2_DIR/$MODE/image_digest.txt" 2>/dev/null || true
        grep -c "CYCLE_BREAK" fleet_management/FmTrafficHandler.py > "$V2_DIR/$MODE/fix_marker.txt" 2>/dev/null || echo 0 > "$V2_DIR/$MODE/fix_marker.txt"
        echo "   🔖 image: $(cat $V2_DIR/$MODE/image_digest.txt), CYCLE_BREAK sites: $(cat $V2_DIR/$MODE/fix_marker.txt)"

        for N in $SIZES; do
            for TRIAL in $(seq "$TRIAL_START" "$TRIALS"); do
                SCENARIO="E${N}"
                SEED=$(echo "$SEEDS" | cut -d' ' -f"$TRIAL")
                export OPENFMS_SEED="$SEED"
                echo "   🌱 seed=$SEED (trial $TRIAL/$TRIALS)"
                echo ""
                echo "================================================================"
                echo "▶️  RUN: mode=$MODE  N=$N  trial=$TRIAL/$TRIALS   ($(date '+%F %T'))"
                echo "================================================================"

                # --- 1. Clean slate (containers + stale snapshot) ---
                cmd_clean
                docker compose -f $COMPOSE_FILE -p $PROJ down --remove-orphans > /dev/null 2>&1

                # --- 2. Generate map/graph for N robots ---
                echo "[1/5] 🗺️  Generating map for $SCENARIO (mode=$MODE)..."
                docker compose -f $COMPOSE_FILE -p $PROJ run --rm scenario python3 fleet_management/FmInterface.py generate "$SCENARIO"
                if [ $? -ne 0 ]; then
                    echo "❌ Map generation failed for $SCENARIO. Skipping run."
                    echo "$(date '+%F %T') mode=$MODE N=$N trial=$TRIAL outcome=MAP_GEN_FAILED" >> "$MANIFEST"
                    continue
                fi

                echo "   🔧 Patching config.yaml for Docker networking (localhost → db/mqtt)..."
                sed -i 's|broker_address: "localhost"|broker_address: "mqtt"|g' config/config.yaml
                sed -i 's|broker_address: localhost|broker_address: mqtt|g' config/config.yaml
                sed -i 's|host: "localhost"|host: "db"|g' config/config.yaml
                sed -i 's|host: localhost|host: db|g' config/config.yaml

                # --- 3. Start infra (simulator in the requested mode) ---
                echo "[2/5] 🚀 Starting MQTT, PostgreSQL, Robot Simulator ($MODE)..."
                docker compose -f $COMPOSE_FILE -p $PROJ up -d mqtt db simulator
                if [ $? -ne 0 ]; then
                    echo "❌ Failed to start infrastructure. Skipping run."
                    echo "$(date '+%F %T') mode=$MODE N=$N trial=$TRIAL outcome=INFRA_FAILED" >> "$MANIFEST"
                    continue
                fi

                echo "   Waiting for PostgreSQL to be ready..."
                READY=1
                for i in {1..30}; do
                    docker compose -f $COMPOSE_FILE -p $PROJ exec -T db pg_isready -U postgres > /dev/null 2>&1
                    if [ $? -eq 0 ]; then echo "   ✅ PostgreSQL is ready!"; READY=0; break; fi
                    echo -n "."; sleep 2
                done
                if [ $READY -ne 0 ]; then
                    echo "❌ PostgreSQL failed to initialize. Skipping run."
                    echo "$(date '+%F %T') mode=$MODE N=$N trial=$TRIAL outcome=DB_FAILED" >> "$MANIFEST"
                    docker compose -f $COMPOSE_FILE -p $PROJ down --remove-orphans > /dev/null 2>&1
                    continue
                fi

                # --- 4. Launch scenario + dashboard ---
                echo "[3/5] 🎮 Launching scenario $SCENARIO..."
                export SCENARIO_NAME=$SCENARIO
                docker compose -f $COMPOSE_FILE -p $PROJ up -d dashboard scenario

                # --- 5. Wait for completion (all orders done, active=0) or timeout ---
                echo "[4/5] ⏳ Waiting for completion (timeout ${EXP_TIMEOUT}s)..."
                START=$SECONDS
                OUTCOME="TIMEOUT"
                # Remote (stock) counters count ALL finalized orders: N transport + N
                # return-to-dock move finals ≈ 2N. Threshold N with active==0 and
                # unassigned==0 is the terminal state; the snapshot itself is the proof.
                EXPECTED=$N
                while :; do
                    if [ "$EXP_TIMEOUT" -gt 0 ] && [ $((SECONDS - START)) -ge "$EXP_TIMEOUT" ]; then
                        break
                    fi
                    sleep 15
                    if [ -f logs/result_snapshot.txt ]; then
                        COMPLETED=$(grep -oP 'completed Orders: \K[0-9]+' logs/result_snapshot.txt | head -1)
                        ACTIVE=$(grep -oP 'active Orders: \K[0-9]+' logs/result_snapshot.txt | head -1)
                        UNASSIGNED=$(grep -oP 'unassigned Orders: \K[0-9]+' logs/result_snapshot.txt | head -1)
                        ELAPSED=$((SECONDS - START))
                        echo "   … t=${ELAPSED}s completed=${COMPLETED:-?} active=${ACTIVE:-?} unassigned=${UNASSIGNED:-?}"
                        if [ -n "$COMPLETED" ] && [ "$COMPLETED" -ge "$EXPECTED" ] && [ "${ACTIVE:-1}" = "0" ] && [ "${UNASSIGNED:-1}" = "0" ]; then
                            OUTCOME="COMPLETED"
                            break
                        fi
                    fi
                done
                ELAPSED=$((SECONDS - START))
                echo "   → outcome: $OUTCOME after ${ELAPSED}s"

                # --- 5b. Settle: give the analytics a final chance to flush ---
                sleep 20

                # --- 6. Collate results into Open_fms_v2 ---
                echo "[5/5] 📦 Collating results into $V2_DIR/$MODE/ ..."
                if [ "$TRIALS" -gt 1 ]; then
                    DEST="$V2_DIR/$MODE/R${N}_trial${TRIAL}_result_snapshot.txt"
                    DASH_DEST="$V2_DIR/$MODE/R${N}_trial${TRIAL}_live_dashboard.txt"
                    META_DEST="$V2_DIR/$MODE/R${N}_trial${TRIAL}_run_meta.txt"
                else
                    DEST="$V2_DIR/$MODE/R${N}_result_snapshot.txt"
                    DASH_DEST="$V2_DIR/$MODE/R${N}_live_dashboard.txt"
                    META_DEST="$V2_DIR/$MODE/R${N}_run_meta.txt"
                fi
                cp -f logs/result_snapshot.txt "$DEST" 2>/dev/null || echo "   ⚠ result_snapshot.txt missing"
                cp -f logs/live_dashboard.txt "$DASH_DEST" 2>/dev/null || true
                # --- Collation guard: never log what was not verified on disk ---
                COLLATED="OK"
                if [ ! -s "$DEST" ]; then
                    COLLATED="MISSING_SNAPSHOT"; echo "   ❌ COLLATION FAILED: $DEST absent/empty"
                else
                    V_ACT=$(grep -oP 'active Orders: \K[0-9]+' "$DEST" | head -1)
                    V_COMP=$(grep -oP 'completed Orders: \K[0-9]+' "$DEST" | head -1)
                    V_TC=$(grep -oP 'Task Completion \(s\): \K[0-9.]+' "$DEST" | head -1)
                    if [ "$OUTCOME" = "COMPLETED" ] && { [ "${V_ACT:-9}" != "0" ] || [ -z "$V_COMP" ]; }; then
                        COLLATED="UNVERIFIED_SNAPSHOT"; echo "   ❌ COLLATION FAILED: $DEST not terminal (active=${V_ACT:-?} completed=${V_COMP:-?})"
                    fi
                fi
                {
                    echo "date=$(date '+%F %T')"
                    echo "mode=$MODE"
                    echo "robots=$N"
                    echo "trial=$TRIAL/$TRIALS"
                    echo "scenario=$SCENARIO"
                    echo "outcome=$OUTCOME"
                    echo "wall_time_sec=$ELAPSED"
                    echo "timeout_sec=$EXP_TIMEOUT"
                    echo "sim_publish_type=$MODE"
                    echo "seed=$SEED"
                } > "$META_DEST"
                echo "$(date '+%F %T') mode=$MODE N=$N trial=$TRIAL outcome=$OUTCOME wall=${ELAPSED}s collation=$COLLATED seed=$SEED" >> "$MANIFEST"
                if [ "$COLLATED" != "OK" ]; then
                    echo "   ❌ Run logged as $OUTCOME but collation=$COLLATED — files kept, flagged, NOT silently trusted."
                fi

                # --- 7. Tear down before next run; refuse to proceed on lingering state ---
                docker compose -f $COMPOSE_FILE -p $PROJ down --remove-orphans > /dev/null 2>&1
                docker rm -f openfms-scenario-1 openfms-dashboard-1 openfms-simulator-1 openfms-manager-1 > /dev/null 2>&1
                LEFTOVER=$(docker ps -aq --filter "name=openfms" 2>/dev/null | wc -l)
                if [ "$LEFTOVER" -gt 0 ]; then
                    echo "   ❌ $LEFTOVER lingering openfms container(s) after teardown — forcing removal:"
                    docker ps -a --filter "name=openfms" --format "   → {{.Names}} ({{.Status}})"
                    docker rm -f $(docker ps -aq --filter "name=openfms") > /dev/null 2>&1
                    LEFTOVER2=$(docker ps -aq --filter "name=openfms" 2>/dev/null | wc -l)
                    [ "$LEFTOVER2" -gt 0 ] && { echo "   ❌ STILL $LEFTOVER2 lingering — aborting suite to prevent contamination."; exit 3; }
                    echo "   ✓ Forced cleanup done."
                fi
                # Fresh DB is structural: db service has no volumes (ephemeral per container)
                # AND the FMS drops/recreates tables on startup; verified empirically by
                # completed==exactly 2N every run (counts never accumulate across runs).
            done
        done

        # --- Restore the remote-identical simulator file for this mode ---
        git checkout -- fleet_management/FmRobotSimulator.py && echo "   🔄 fleet_management/FmRobotSimulator.py restored to remote (f4de7e3) state."
    done

    echo ""
    echo "================================================================"
    echo "✅ Experiment suite finished."
    echo "   Results:  $V2_DIR/{throttled,non_throttled}/R<N>_result_snapshot.txt"
    echo "   Manifest: $MANIFEST"
    echo "   Next:     python3 scratch/collate_v2.py   (host, read-only, no installs)"
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
    experiments)
        cmd_experiments "$@"
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