#!/bin/bash
# Script to restore the configuration of all ODrive boards

echo "RESTORING ODRIVE CONFIGURATIONS"
echo "=========================================="
echo ""
echo "INSTRUCTIONS:"
echo "   1. Connect ONE ODrive board at a time via USB"
echo "   2. Press ENTER to restore its configuration"
echo "   3. Disconnect it and connect the next one"
echo ""

# List of backup files
declare -a configs=(
    "/home/divin/my_odrive_config_card1.json"
    "/home/divin/my_odrive_config_card2.json"
    "/home/divin/my_odrive_config_card3.json"
    "/home/divin/my_odrive_config_card4.json"
    "/home/divin/my_odrive_config_card5.json"
    "/home/divin/my_odrive_config_card6.json"
)

for i in "${!configs[@]}"; do
    card_num=$((i + 1))
    config_file="${configs[$i]}"
    
    echo ""
    echo "=========================================="
    echo "BOARD $card_num / 6"
    echo "=========================================="
    echo ""
    echo "1. Connect BOARD $card_num via USB"
    echo "2. Press ENTER to restore..."
    read -p ""
    
    echo ""
    echo "Searching for the ODrive..."
    if ! odrivetool shell -c "print('ODrive found')" 2>/dev/null; then
        echo "No ODrive detected!"
        echo "   -> Check the USB connection"
        echo "   -> Press ENTER to retry..."
        read -p ""
        continue
    fi
    
    echo "ODrive detected!"
    echo ""
    echo "Restoring configuration..."
    echo "   File: $config_file"
    
    if odrivetool restore-config "$config_file"; then
        echo ""
        echo "BOARD $card_num RESTORED SUCCESSFULLY!"
        echo ""
        echo "Checking configuration..."
        odrivetool shell << 'INNER_MARKER'
import odrive
odrv0 = odrive.find_any()
print(f"\n=== RESTORED CONFIGURATION ===")
print(f"Serial: {odrv0.serial_number}")
print(f"CAN baud rate: {odrv0.can.config.baud_rate}")
print(f"Axis0 node_id: {odrv0.axis0.config.can.node_id}")
print(f"Axis1 node_id: {odrv0.axis1.config.can.node_id}")
print(f"Axis0 motor calibrated: {odrv0.axis0.motor.config.pre_calibrated}")
print(f"Axis0 encoder ready: {odrv0.axis0.encoder.is_ready}")
print(f"Axis1 motor calibrated: {odrv0.axis1.motor.config.pre_calibrated}")
print(f"Axis1 encoder ready: {odrv0.axis1.encoder.is_ready}")
print(f"\nBoard $card_num ready to use!")
INNER_MARKER
    else
        echo ""
        echo "RESTORE FAILED"
        echo "   -> Check that the file exists"
        echo "   -> Check that the ODrive is properly connected"
    fi
    
    if [ $card_num -lt 6 ]; then
        echo ""
        echo "Disconnect BOARD $card_num and connect BOARD $((card_num + 1))"
        echo "    Press ENTER to continue..."
        read -p ""
    fi
done

echo ""
echo "=========================================="
echo "RESTORE COMPLETE"
echo "=========================================="
echo ""
echo "All 6 ODrive boards have been restored"
echo ""
echo "NEXT STEPS:"
echo "   1. Disconnect USB from all boards"
echo "   2. Check that the CAN bus is active (can0)"
echo "   3. Launch the ROS2 hardware layer"
echo ""
