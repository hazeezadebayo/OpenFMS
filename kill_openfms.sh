#!/bin/bash

# kill_openfms.sh — Nuclear shutdown for OpenFMS
# Guarantees complete removal of ALL openfms containers, networks,
# and dangling resources — from both current and previous runs.

echo "================================================================"
echo "💀 kill_openfms.sh — Killing OpenFMS completely"
echo "================================================================"

# ── Step 1: docker compose down (handles the current project) ────
echo "[1/5] Stopping and removing compose services..."
docker compose down --remove-orphans --volumes 2>/dev/null
echo "   ✓ compose down done"

# ── Step 2: Force-stop any openfms containers still running ──────
# Catches containers from previous broken runs that compose didn't own.
echo "[2/5] Force-stopping any lingering openfms containers..."
RUNNING=$(docker ps -q --filter "name=openfms")
if [ -n "$RUNNING" ]; then
    docker stop $RUNNING
    echo "   ✓ Stopped: $RUNNING"
else
    echo "   ✓ No running openfms containers found"
fi

# ── Step 3: Remove ALL openfms containers (running or not) ───────
echo "[3/5] Removing all openfms containers (including dead ones)..."
ALL=$(docker ps -aq --filter "name=openfms")
if [ -n "$ALL" ]; then
    docker rm -f $ALL
    echo "   ✓ Removed: $ALL"
else
    echo "   ✓ No openfms containers to remove"
fi

# ── Step 4: Remove openfms networks ──────────────────────────────
echo "[4/5] Removing openfms networks..."
NETS=$(docker network ls --filter "name=openfms" -q)
if [ -n "$NETS" ]; then
    docker network rm $NETS 2>/dev/null && echo "   ✓ Networks removed" \
        || echo "   ⚠ Some networks still in use (another container may be attached). Try: docker network prune"
else
    echo "   ✓ No openfms networks found"
fi

# ── Step 5: Confirm nothing is left ──────────────────────────────
echo "[5/5] Verifying — checking for any surviving openfms resources..."
LEFTOVER=$(docker ps -aq --filter "name=openfms")
if [ -n "$LEFTOVER" ]; then
    echo "   ⚠ WARNING: These containers could not be removed:"
    docker ps -a --filter "name=openfms" --format "   → {{.Names}} ({{.Status}})"
    echo "   Try: docker rm -f \$(docker ps -aq --filter name=openfms)"
else
    echo "   ✓ All clear — no openfms containers remain"
fi

echo ""
echo "================================================================"
echo "✅ OpenFMS is dead."
echo "================================================================"
