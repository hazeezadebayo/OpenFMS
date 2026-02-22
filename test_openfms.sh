#!/bin/bash

# OpenFMS Docker Test Script
# This script builds and starts the OpenFMS fleet environment in Docker,
# then runs the conflict test script inside the manager container so
# the user can see the FM decisions in realtime.

echo "🚀 Starting OpenFMS Environment for Testing..."
docker compose up -d --build --remove-orphans

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ OpenFMS started successfully!"
    echo "================================================================"
    
    echo "🏃 MODE: Automated Tests"
    echo "📌 Action: Running conflict_test.py inside the openfms-manager-1 container..."
    echo ""
    
    # Run the tests directly inside the manager container
    docker exec -it openfms-manager-1 python3 /app/fleet_management/tests/conflict_test.py
    
    echo "================================================================"
    echo "💡 Tests completed. To stop everything, run: docker compose down"
else
    echo ""
    echo "❌ Error: Failed to start OpenFMS."
    echo "Please ensure Docker is installed and the daemon is running."
fi
