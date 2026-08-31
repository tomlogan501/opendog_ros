#!/bin/bash

# Script to activate all ODrives in CLOSED_LOOP_CONTROL over CAN
# Node IDs: 0 to 11

echo "==============================================================="
echo "  Activating all ODrives in CLOSED_LOOP_CONTROL"
echo "==============================================================="

# Check that can0 is UP
if ! ip link show can0 | grep -q "UP"; then
    echo "Error: can0 is not UP"
    exit 1
fi

echo ""
echo "CAN bus active"
echo ""

# Function to send a CAN command
send_can_command() {
    local node_id=$1
    local cmd_id=$2
    local data=$3
    local can_id=$((node_id * 32 + cmd_id))
    
    cansend can0 $(printf "%03X" $can_id)#$data
}

# CMD 0x07: Set Axis State
# Data: 08 00 00 00 (AXIS_STATE_CLOSED_LOOP_CONTROL = 8)

echo "Sending CLOSED_LOOP_CONTROL command to all ODrives..."
echo ""

for node_id in {0..11}; do
    echo "  -> Node ID $node_id: Set Axis State = 8 (CLOSED_LOOP_CONTROL)"
    send_can_command $node_id 7 "08000000"
    sleep 0.1
done

echo ""
echo "Commands sent."
echo ""
echo "Waiting 2 seconds for stabilization..."
sleep 2

echo ""
echo "Checking heartbeats (Ctrl+C to stop)..."
echo ""
timeout 5 candump can0 | grep " 001 " | head -12

echo ""
echo "==============================================================="
echo "  Verify the axes are in state 8 (CLOSED_LOOP)"
echo "  Byte 4 of the heartbeat must be 08"
echo "==============================================================="
