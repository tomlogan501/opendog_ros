#!/bin/bash
# Script to check ODrive errors via CAN
# Reads Get_Error messages on the CAN bus

echo "=========================================="
echo "CHECKING ODRIVE ERRORS via CAN"
echo "=========================================="
echo ""

# Function to request errors from a node
request_errors() {
    local node_id=$1
    local cmd_id=$(printf "%03X" $((0x003 + (node_id << 5))))
    
    echo "Node $node_id (CMD: 0x$cmd_id):"
    
    # Send Get_Error (CMD 0x003)
    cansend can0 ${cmd_id}#
    
    # Wait for the response (0.1 sec)
    sleep 0.1
}

echo "Requesting errors for all nodes..."
echo ""

# Request errors for nodes 0-11
for node in {0..11}; do
    request_errors $node
done

echo ""
echo "Capturing responses (3 seconds)..."
echo ""

# Capture messages for 3 seconds
timeout 3 candump can0 2>&1 | grep -E "can0  0[0-9a-fA-F]{2}   \[8\]"

echo ""
echo "=========================================="
echo "ANALYSIS:"
echo "  - Heartbeat (0x001): Axis state"
echo "  - Get_Error (0x003): Error codes"
echo "  - 00 00 00 00 = No error"
echo "  - Other value = Error present"
echo "=========================================="
