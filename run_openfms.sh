#!/bin/bash

# OpenFMS Convenience Startup Script
# This script builds and starts the OpenFMS fleet environment in Docker.

echo "🚀 Starting OpenFMS Environment..."

# 1. Build and start services in detached mode
# --build ensures any code changes are baked into the images
# --remove-orphans cleans up services that were commented out in docker-compose.yml
docker compose up -d --build --remove-orphans

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ OpenFMS started successfully!"
    echo "================================================================"
    
    # Check if scenario service is running (automated mode)
    if docker compose ps | grep -q "scenario"; then
        echo "🏃 MODE: Automated Scenario"
        echo "📌 Action: To monitor the robot tasks, run:"
        echo "   docker compose logs -f scenario"
    fi

    # Check if manager service is running (manual mode)
    if docker compose ps | grep -q "manager"; then
        echo "🎮 MODE: Interactive Manager"
        echo "📌 Action: To control the fleet manually, run:"
        echo "   docker attach openfms-manager-1"
        echo "   (Use Ctrl+P, Ctrl+Q to detach without stopping)"
    fi

    echo "================================================================"
    echo "💡 To stop everything, run: docker compose down"
else
    echo ""
    echo "❌ Error: Failed to start OpenFMS."
    echo "Please ensure Docker is installed and the daemon is running."
fi
