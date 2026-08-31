#!/usr/bin/env python3
"""
Script to configure an ODrive in CAN mode
Usage: python3 configure_odrive_can.py <node_id>
"""

import odrive
from odrive.enums import *
import sys
import time

def configure_odrive_can(node_id):
    """Configure an ODrive for the CAN bus"""
    
    if node_id < 0 or node_id > 5:
        print(f"Invalid Node ID: {node_id} (must be between 0 and 5)")
        return False
    
    print(f"\n{'='*60}")
    print(f"Configuring ODrive for CAN Node ID {node_id}")
    print(f"{'='*60}\n")
    
    print("Searching for an ODrive connected via USB...")
    try:
        odrv = odrive.find_any(timeout=15)
    except Exception as e:
        print(f"No ODrive found: {e}")
        print("\nCheck:")
        print("  1. The ODrive is powered on")
        print("  2. The USB cable is connected")
        print("  3. Only one ODrive is connected via USB at a time")
        return False
    
    print(f"ODrive found: SN {odrv.serial_number}\n")
    
    # Show current configuration
    print("--- Current configuration ---")
    print(f"CAN Enabled: {odrv.config.enable_can_a}")
    print(f"CAN Node ID: {odrv.can.node_id}")
    print(f"CAN Baud Rate: {odrv.can.config.baud_rate}")
    print(f"Axis 0 Watchdog: {odrv.axis0.config.enable_watchdog}")
    print(f"Axis 1 Watchdog: {odrv.axis1.config.enable_watchdog}")
    
    # Ask for confirmation
    print(f"\nThis operation will:")
    print(f"  1. Configure CAN at 250000 baud")
    print(f"  2. Set the Node ID to {node_id}")
    print(f"  3. Enable watchdogs (0.1s timeout)")
    print(f"  4. Save and reboot the ODrive")
    
    response = input(f"\nContinue? (yes/no): ").strip().lower()
    if response not in ['oui', 'o', 'yes', 'y']:
        print("Operation cancelled")
        return False
    
    print("\n--- Applying configuration ---")
    
    try:
        # CAN configuration
        print("1. Configuring CAN bus...")
        odrv.config.enable_can_a = True
        odrv.can.config.baud_rate = 250000
        odrv.can.node_id = node_id
        print(f"   CAN enabled, Node ID = {node_id}, Baud = 250000")
        
        # Watchdog configuration
        print("2. Configuring watchdogs...")
        odrv.axis0.config.enable_watchdog = True
        odrv.axis0.config.watchdog_timeout = 0.1
        odrv.axis1.config.enable_watchdog = True
        odrv.axis1.config.watchdog_timeout = 0.1
        print("   Watchdogs enabled (0.1s)")
        
        # Clear errors
        print("3. Clearing errors...")
        odrv.clear_errors()
        print("   Errors cleared")
        
        # Save
        print("4. Saving configuration...")
        odrv.save_configuration()
        print("   Configuration saved")
        
        print("\n5. Rebooting the ODrive...")
        print("   (Wait 5 seconds...)")
        try:
            odrv.reboot()
        except:
            pass  # Connection will be lost during reboot
        
        time.sleep(5)
        
        print("\n{'='*60}")
        print("CONFIGURATION COMPLETE!")
        print(f"{'='*60}")
        print(f"\nThe ODrive with Node ID {node_id} is now configured.")
        print("\nNext steps:")
        print("  1. Disconnect the USB cable from this ODrive")
        print("  2. Connect the next ODrive via USB")
        print("  3. Rerun this script with the next Node ID")
        print(f"     python3 configure_odrive_can.py {node_id + 1}")
        
        return True
        
    except Exception as e:
        print(f"\nERROR during configuration: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 configure_odrive_can.py <node_id>")
        print("\nExamples:")
        print("  python3 configure_odrive_can.py 0  # configure the first ODrive")
        print("  python3 configure_odrive_can.py 1  # configure the second ODrive")
        print("  python3 configure_odrive_can.py 2  # configure the third ODrive")
        print("  ... etc up to 5")
        print("\nConnect ONLY ONE ODrive via USB at a time!")
        sys.exit(1)
    
    try:
        node_id = int(sys.argv[1])
    except ValueError:
        print(f"Invalid Node ID: {sys.argv[1]} (must be a number between 0 and 5)")
        sys.exit(1)
    
    success = configure_odrive_can(node_id)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
