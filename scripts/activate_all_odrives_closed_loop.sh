#!/bin/bash

# Script to activate ALL ODrives in CLOSED_LOOP_CONTROL via CAN
# Node IDs: 0 to 11 (12 axes total)

echo "=========================================="
echo "ACTIVATING ALL ODRIVES IN CLOSED_LOOP_CONTROL"
echo "=========================================="
echo ""

# Function to send Set_Axis_State = CLOSED_LOOP_CONTROL (8)
send_closed_loop() {
    local node_id=$1
    local cmd_id=0x07  # Set_Axis_State
    local can_id=$((node_id * 32 + cmd_id))
    local can_id_hex=$(printf "0x%03X" $can_id)
    
    # CLOSED_LOOP_CONTROL state = 8 (0x08000000 little-endian, 4 bytes)
    local data="08 00 00 00"
    
    echo "Node ID $node_id: sending Set_Axis_State = CLOSED_LOOP_CONTROL (8)"
    cansend can0 ${can_id_hex}#${data}
    
    # Small delay to avoid saturating the bus
    sleep 0.1
}

echo "Sending Set_Axis_State = CLOSED_LOOP_CONTROL commands..."
echo ""

# Loop over all Node IDs (0 to 11)
for node_id in {0..11}; do
    send_closed_loop $node_id
done

echo ""
echo "=========================================="
echo "COMMANDS SENT"
echo "=========================================="
echo ""
echo "Checking in 2 seconds..."
sleep 2

echo ""
echo "=========================================="
echo "CHECKING HEARTBEATS (5 seconds)"
echo "=========================================="
echo ""
echo "Looking for Heartbeat (CAN ID 0x001, 0x021, 0x041, etc.)"
echo "Expected format: [8] XX XX 00 00 08 00 00 00"
echo "                              ^^-- State 8 = CLOSED_LOOP"
echo ""

# Capture 50 messages and filter for Heartbeat
candump can0 -n 50 | grep -E "0[0-9A-F]1   \[8\]"

echo ""
echo "=========================================="
echo "CHECKING /joint_states"
echo "=========================================="
echo ""
echo "If the ODrives are in CLOSED_LOOP, you should see values"
echo "instead of '.nan' for all joints."
echo ""
echo "Now run:"
echo "  ros2 topic echo /joint_states --once"
echo ""
