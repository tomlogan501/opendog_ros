#!/usr/bin/env python3
"""
Diagnostic script to check the CAN configuration of all ODrives
"""

import odrive
from odrive.enums import *
import sys

def check_odrive_can(odrv, index):
    """Check an ODrive's CAN configuration"""
    print(f"\n{'='*60}")
    print(f"ODrive #{index}")
    print(f"{'='*60}")
    
    try:
        # Basic information
        print(f"Serial Number: {odrv.serial_number}")
        print(f"Hardware Version: v{odrv.hw_version_major}.{odrv.hw_version_minor}")
        print(f"Firmware Version: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
        
        # CAN configuration
        print(f"\n--- CAN Configuration ---")
        print(f"CAN Enabled: {odrv.config.enable_can_a}")
        print(f"CAN Node ID: {odrv.can.node_id}")
        print(f"CAN Baud Rate: {odrv.can.config.baud_rate}")
        
        # Axis states
        print(f"\n--- Axis States ---")
        for axis_num in [0, 1]:
            axis = getattr(odrv, f"axis{axis_num}")
            print(f"\nAxis {axis_num}:")
            print(f"  Current State: {axis.current_state}")
            print(f"  Axis Error: 0x{axis.error:08X}")
            print(f"  Motor Error: 0x{axis.motor.error:08X}")
            print(f"  Encoder Error: 0x{axis.encoder.error:08X}")
            print(f"  Controller Error: 0x{axis.controller.error:08X}")
            print(f"  Watchdog Enabled: {axis.config.enable_watchdog}")
            print(f"  Watchdog Timeout: {axis.config.watchdog_timeout}s")
        
        # Recommendations
        print(f"\n--- Diagnostic ---")
        issues = []
        
        if not odrv.config.enable_can_a:
            issues.append("CAN is NOT enabled!")
        else:
            print("CAN is enabled")
        
        if odrv.can.config.baud_rate != 250000:
            issues.append(f"Incorrect baud rate: {odrv.can.config.baud_rate} (should be 250000)")
        else:
            print("Baud rate correct (250000)")
        
        if odrv.can.node_id < 0 or odrv.can.node_id > 5:
            issues.append(f"Node ID out of range: {odrv.can.node_id} (should be 0-5)")
        else:
            print(f"Valid Node ID: {odrv.can.node_id}")
        
        if issues:
            print("\nISSUES DETECTED:")
            for issue in issues:
                print(f"  {issue}")
            return False
        else:
            print("\nCAN configuration correct!")
            return True
            
    except Exception as e:
        print(f"ERROR while reading: {e}")
        return False

def main():
    print("="*60)
    print("ODRIVE DIAGNOSTIC - CAN Configuration")
    print("="*60)
    
    print("\nSearching for all connected ODrives...")
    print("(This can take 10-30 seconds...)\n")
    
    # Find all ODrives
    odrives = []
    try:
        # Method 1: general search
        print("Searching...")
        odrv = odrive.find_any(timeout=10)
        if odrv:
            odrives.append(odrv)
            print(f"ODrive found: SN {odrv.serial_number}")
            
            # Try to find other ODrives
            for i in range(5):  # look for up to 5 more
                try:
                    print(f"Searching for another ODrive...")
                    odrv = odrive.find_any(timeout=5)
                    if odrv and odrv not in odrives:
                        odrives.append(odrv)
                        print(f"ODrive found: SN {odrv.serial_number}")
                except:
                    break
    except Exception as e:
        print(f"Error during search: {e}")
    
    if not odrives:
        print("\nNO ODrive found!")
        print("\nCheck:")
        print("  1. The ODrives are powered on (LEDs lit)")
        print("  2. The USB cables are properly connected")
        print("  3. You have the right permissions (add your user to the dialout group)")
        print("     sudo usermod -a -G dialout $USER")
        return
    
    print(f"\n{'='*60}")
    print(f"NUMBER OF ODRIVES FOUND: {len(odrives)}")
    print(f"{'='*60}")
    
    # Check each ODrive
    results = []
    for i, odrv in enumerate(odrives):
        result = check_odrive_can(odrv, i)
        results.append((i, odrv.serial_number, result))
    
    # Final summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    
    configured_count = sum(1 for _, _, ok in results if ok)
    print(f"\nCorrectly configured ODrives: {configured_count}/{len(odrives)}")
    
    print("\nODrive list:")
    for i, sn, ok in results:
        status = "OK" if ok else "NEEDS CONFIGURATION"
        print(f"  ODrive #{i} (SN: {sn}): {status}")
    
    if configured_count < len(odrives):
        print("\nSome ODrives need configuration!")
        print("Use the 'configure_odrive_can.py' script to configure them.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\nFATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
