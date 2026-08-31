#!/usr/bin/env python3
"""
Script to check the state of all ODrives connected via USB
"""

import odrive
from odrive.enums import *
import sys

def check_odrive(odrv, odrv_serial):
    """Check the state of an ODrive"""
    print(f"\n{'='*70}")
    print(f"  ODrive Serial: {odrv_serial}")
    print(f"{'='*70}")
    
    # Check both axes
    for axis_num in [0, 1]:
        axis = getattr(odrv, f'axis{axis_num}')
        node_id = axis.config.can.node_id
        
        print(f"\n--- Axis {axis_num} (Node ID: {node_id}) ---")
        print(f"  Axis error: {hex(axis.error)}")
        print(f"  Motor error: {hex(axis.motor.error)}")
        print(f"  Encoder error: {hex(axis.encoder.error)}")
        print(f"  Current state: {axis.current_state}")
        print(f"  Motor calibrated: {axis.motor.is_calibrated}")
        print(f"  Encoder ready: {axis.encoder.is_ready}")
        print(f"  Encoder shadow_count: {axis.encoder.shadow_count}")
        
        # Decode errors
        if axis.error != 0:
            print(f"  AXIS ERROR DETECTED:")
            if axis.error & 0x0001:
                print(f"      - INITIALIZING")
            if axis.error & 0x0002:
                print(f"      - SYSTEM_LEVEL")
            if axis.error & 0x0004:
                print(f"      - TIMING_ERROR")
            if axis.error & 0x0008:
                print(f"      - MISSING_ESTIMATE")
            if axis.error & 0x0010:
                print(f"      - BAD_CONFIG")
            if axis.error & 0x0020:
                print(f"      - DRV_FAULT")
            if axis.error & 0x0040:
                print(f"      - MISALIGNMENT")
            if axis.error & 0x0080:
                print(f"      - MOTOR_FAILED")
            if axis.error & 0x0100:
                print(f"      - SENSORLESS_ESTIMATOR_FAILED")
            if axis.error & 0x0200:
                print(f"      - ENCODER_FAILED")
            if axis.error & 0x0400:
                print(f"      - CONTROLLER_FAILED")
            if axis.error & 0x0800:
                print(f"      - POS_CTRL_DURING_SENSORLESS")
            if axis.error & 0x1000:
                print(f"      - WATCHDOG_TIMER_EXPIRED")
            if axis.error & 0x2000:
                print(f"      - MIN_ENDSTOP_PRESSED")
            if axis.error & 0x4000:
                print(f"      - MAX_ENDSTOP_PRESSED")
            if axis.error & 0x8000:
                print(f"      - ESTOP_REQUESTED")
            if axis.error & 0x10000:
                print(f"      - HOMING_WITHOUT_ENDSTOP")
            if axis.error & 0x20000:
                print(f"      - OVER_TEMP")
            if axis.error & 0x40000:
                print(f"      - UNKNOWN_POSITION")

def main():
    print("=" * 70)
    print("  Checking all ODrives connected via USB")
    print("=" * 70)
    
    try:
        print("\nSearching for an ODrive...")
        print("Connect ONLY ONE ODrive at a time via USB\n")
        
        odrv = odrive.find_any(timeout=10)
        
        if not odrv:
            print("No ODrive found!")
            sys.exit(1)
        
        print(f"ODrive found: {odrv.serial_number}")
        
        check_odrive(odrv, odrv.serial_number)
        
        print(f"\n{'='*70}")
        print("  Check complete")
        print(f"{'='*70}\n")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
