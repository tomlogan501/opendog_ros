#!/bin/bash
# Script to test parsing of 0x009 messages

echo "=========================================="
echo "TESTING 0x009 MESSAGE PARSING"
echo "=========================================="
echo ""

echo "1. Capturing CAN messages for 5 seconds..."
echo "   Looking for 0x009 messages WITH DATA (8 bytes)"
echo ""

# Capture messages for 5 seconds
timeout 5 candump can0 2>&1 > /tmp/can_capture.txt

echo "2. Analyzing 0x009 messages..."
echo ""

# Look for 0x009 messages (all nodes)
echo "=== 0x009 requests (0 bytes) ==="
grep -E "can0  0[0-9a-fA-F]9   \[0\]" /tmp/can_capture.txt | head -20

echo ""
echo "=== 0x009 responses (8 bytes) ==="
grep -E "can0  0[0-9a-fA-F]9   \[8\]" /tmp/can_capture.txt | head -20

echo ""
echo "=========================================="
echo "ANALYSIS:"
echo "  - If you see [0] -> requests sent"
echo "  - If you see [8] -> responses received"
echo "  - If NO [8] at all -> ODrives are not responding"
echo "=========================================="
echo ""

# Count messages
req_count=$(grep -c -E "can0  0[0-9a-fA-F]9   \[0\]" /tmp/can_capture.txt)
resp_count=$(grep -c -E "can0  0[0-9a-fA-F]9   \[8\]" /tmp/can_capture.txt)

echo "STATISTICS:"
echo "   0x009 requests: $req_count"
echo "   0x009 responses: $resp_count"
echo ""

if [ $resp_count -eq 0 ]; then
    echo "NO responses! The ODrives are not in CLOSED_LOOP"
    echo ""
    echo "SOLUTION:"
    echo "   1. Check that the ODrives are in CLOSED_LOOP:"
    echo "      candump can0 -n 20 | grep 'can0  0[0-9a-f]1'"
    echo "   2. Look for state '08' (CLOSED_LOOP) in byte 5"
    echo "   3. If state is '01' (IDLE), run:"
    echo "      /home/dev/opendog_ws/scripts/start_closed_loop_can.sh"
else
    echo "The ODrives are responding! Parsing is working!"
    echo ""
    echo "Response detail:"
    grep -E "can0  0[0-9a-fA-F]9   \[8\]" /tmp/can_capture.txt | while read line; do
        can_id=$(echo $line | awk '{print $2}')
        node_id=$((0x$can_id >> 5))
        echo "   Node $node_id responded"
    done
fi
