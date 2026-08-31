#!/usr/bin/env python3
"""
Script to configure all ODrives with reduced current limits
to run on a limited power supply (24V 10A = 240W)
"""

import odrive
from odrive.enums import *
import sys
import time

def configure_odrive_low_current(odrv, node_ids):
    """
    Configure an ODrive with very low current limits
    
    Args:
        odrv: ODrive instance
        node_ids: Tuple (axis0_node_id, axis1_node_id)
    """
    print(f"\n{'='*60}")
    print(f"Configuring ODrive - Node IDs: {node_ids[0]} and {node_ids[1]}")
    print(f"{'='*60}")
    
    # Clear errors
    odrv.clear_errors()
    time.sleep(0.1)
    
    # Axis 0 configuration
    print(f"\nConfiguring Axis 0 (Node ID {node_ids[0]})...")
    odrv.axis0.motor.config.current_lim = 2.0  # 2A max
    odrv.axis0.motor.config.current_lim_margin = 2.0
    odrv.axis0.motor.config.calibration_current = 2.0
    odrv.axis0.motor.config.requested_current_range = 10.0
    
    # Axis 1 configuration
    print(f"Configuring Axis 1 (Node ID {node_ids[1]})...")
    odrv.axis1.motor.config.current_lim = 2.0  # 2A max
    odrv.axis1.motor.config.current_lim_margin = 2.0
    odrv.axis1.motor.config.calibration_current = 2.0
    odrv.axis1.motor.config.requested_current_range = 10.0
    
    # CAN configuration (check)
    print(f"\nChecking CAN configuration...")
    print(f"  CAN enabled: {odrv.config.enable_can_a}")
    print(f"  Baud rate: {odrv.can.config.baud_rate}")
    print(f"  Axis 0 node_id: {odrv.axis0.config.can.node_id}")
    print(f"  Axis 1 node_id: {odrv.axis1.config.can.node_id}")
    
    # Control mode configuration
    print(f"\nConfiguring control mode...")
    odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    odrv.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    
    odrv.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    odrv.axis1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    
    # Watchdog (normally already configured)
    print(f"\nChecking Watchdog...")
    print(f"  Axis 0 watchdog: {odrv.axis0.config.enable_watchdog}, timeout: {odrv.axis0.config.watchdog_timeout}")
    print(f"  Axis 1 watchdog: {odrv.axis1.config.enable_watchdog}, timeout: {odrv.axis1.config.watchdog_timeout}")
    
    # Save and reboot
    print(f"\nSaving configuration...")
    odrv.save_configuration()
    
    print(f"Rebooting the ODrive...")
    try:
        odrv.reboot()
    except:
        pass  # Connection is lost during reboot
    
    print(f"Configuration complete for Node IDs {node_ids[0]} and {node_ids[1]}")
    print(f"Waiting 5 seconds for reboot...")
    time.sleep(5)


def verify_odrive_config(odrv, node_ids):
    """Check the configuration after reboot"""
    print(f"\n{'='*60}")
    print(f"Verifying ODrive - Node IDs: {node_ids[0]} and {node_ids[1]}")
    print(f"{'='*60}")
    
    print(f"\nCurrent state:")
    print(f"  VBUS: {odrv.vbus_voltage:.2f} V")
    print(f"  Axis 0 current_lim: {odrv.axis0.motor.config.current_lim} A")
    print(f"  Axis 1 current_lim: {odrv.axis1.motor.config.current_lim} A")
    print(f"  Axis 0 state: {odrv.axis0.current_state}")
    print(f"  Axis 1 state: {odrv.axis1.current_state}")
    print(f"  Axis 0 error: {hex(odrv.axis0.error)}")
    print(f"  Axis 1 error: {hex(odrv.axis1.error)}")
    
    # Test CLOSED_LOOP_CONTROL
    print(f"\nTesting CLOSED_LOOP_CONTROL...")
    odrv.clear_errors()
    time.sleep(0.1)
    
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(2)
    
    print(f"  Axis 0 state after command: {odrv.axis0.current_state}")
    print(f"  Axis 1 state after command: {odrv.axis1.current_state}")
    print(f"  Axis 0 error: {hex(odrv.axis0.error)}")
    print(f"  Axis 1 error: {hex(odrv.axis1.error)}")
    
    if odrv.axis0.current_state == 8 and odrv.axis1.current_state == 8:
        print(f"  Both axes are in CLOSED_LOOP_CONTROL!")
    else:
        print(f"  Failed to enter CLOSED_LOOP_CONTROL")
        print(f"  Axis 0 motor error: {hex(odrv.axis0.motor.error)}")
        print(f"  Axis 1 motor error: {hex(odrv.axis1.motor.error)}")
    
    # Return to IDLE
    odrv.axis0.requested_state = AXIS_STATE_IDLE
    odrv.axis1.requested_state = AXIS_STATE_IDLE


def main():
    # Node ID mapping (axis0, axis1) for each ODrive
    odrive_node_mapping = [
        (0, 1),    # ODrive 1
        (2, 3),    # ODrive 2
        (4, 5),    # ODrive 3
        (6, 7),    # ODrive 4
        (8, 9),    # ODrive 5
        (10, 11),  # ODrive 6
    ]
    
    print("="*60)
    print("CONFIGURING ODRIVES WITH REDUCED CURRENT")
    print("="*60)
    print("\nIMPORTANT:")
    print("  - Connect ONLY ONE ODrive via USB at a time")
    print("  - Current limit: 2A per motor")
    print("  - Total power: ~50W per ODrive (2 motors)")
    print("  - With 6 ODrives: ~300W total (your supply: 240W max)")
    print("\nYOUR POWER SUPPLY IS STILL INSUFFICIENT!")
    print("  - Recommendation: test with 4 ODrives max (8 motors)")
    print("  - Or reduce current further to 1A per motor")
    print("\n")
    
    for i, node_ids in enumerate(odrive_node_mapping, 1):
        print(f"\n{'#'*60}")
        print(f"# ODrive {i}/6 - Node IDs: {node_ids[0]} and {node_ids[1]}")
        print(f"{'#'*60}")
        
        input(f"\nConnect ODrive {i} via USB and press ENTER...")
        
        print(f"\nSearching for the ODrive...")
        try:
            odrv = odrive.find_any()
            print(f"ODrive found: {odrv.serial_number}")
            
            # Check that the node IDs match
            actual_node0 = odrv.axis0.config.can.node_id
            actual_node1 = odrv.axis1.config.can.node_id
            
            if (actual_node0, actual_node1) != node_ids:
                print(f"WARNING: Node IDs do not match!")
                print(f"   Expected: {node_ids}")
                print(f"   Found: ({actual_node0}, {actual_node1})")
                response = input("Continue anyway? (y/n): ")
                if response.lower() != 'y':
                    print("Skipped.")
                    continue
            
            # Configure
            configure_odrive_low_current(odrv, node_ids)
            
            # Reconnect after reboot
            print(f"\nReconnecting after reboot...")
            odrv = odrive.find_any()
            
            # Verify
            verify_odrive_config(odrv, node_ids)
            
        except Exception as e:
            print(f"Error: {e}")
            response = input("Continue with the next ODrive? (y/n): ")
            if response.lower() != 'y':
                sys.exit(1)
    
    print(f"\n{'='*60}")
    print("CONFIGURATION COMPLETE FOR ALL ODRIVES")
    print(f"{'='*60}")
    print("\nSummary:")
    print("  - Current limit: 2A per motor")
    print("  - 12 motors x 2A = 24A total")
    print("  - Theoretical power: 24V x 24A = 576W")
    print("\nYOUR POWER SUPPLY (240W) IS INSUFFICIENT!")
    print("\nSolutions:")
    print("  1. Test with only 4 ODrives (disconnect 2)")
    print("  2. Buy a 24V 30A minimum power supply")
    print("  3. Use several power supplies in parallel")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
