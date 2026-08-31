#!/bin/bash
# Script to test the hardware layer in READ mode
# This script checks that CAN messages are being parsed correctly

echo "=========================================="
echo "TESTING THE HARDWARE LAYER IN READ MODE"
echo "=========================================="
echo ""

# Check that the CAN bus is active
echo "1. Checking CAN bus..."
if ! ip link show can0 | grep -q "UP"; then
    echo "CAN bus is not active!"
    echo "   Configuring CAN bus..."
    sudo ip link set can0 down 2>/dev/null
    sudo ip link set can0 type can bitrate 250000 restart-ms 100
    sudo ip link set can0 txqueuelen 1000
    sudo ip link set can0 up
    echo "CAN bus configured (250 kbps)"
else
    echo "CAN bus active"
fi

# Show CAN parameters
echo ""
echo "CAN parameters:"
ip -details link show can0 | grep -E "can|bitrate|txqueue"

# Check CAN messages
echo ""
echo "2. Checking CAN messages (5 seconds)..."
echo "   (If you see messages, the ODrives are communicating)"
echo ""
timeout 5 candump can0 | head -20

if [ $? -eq 124 ]; then
    echo ""
    echo "Capture complete"
else
    echo ""
    echo "No CAN messages detected!"
    echo "   -> Check that the ODrives are in CLOSED_LOOP_CONTROL"
    echo "   -> Check the CAN connections (CANH, CANL, GND)"
    exit 1
fi

echo ""
echo "3. Launching the ROS2 hardware layer..."
echo "   (Logs will be saved to /tmp/opendog_read_test.log)"
echo ""

# Source ROS2
cd /home/dev/opendog_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the hardware layer and log output
echo "Starting..."
echo ""
ros2 launch opendog_bringup opendog_bringup_can.launch.py 2>&1 | tee /tmp/opendog_read_test.log
