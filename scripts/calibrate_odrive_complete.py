#!/usr/bin/env python3
"""
Complete ODrive v3.6 calibration script with AMS AS5047P SPI encoders
Follows the official OpenDog procedure
"""

import odrive
from odrive.enums import *
import time
import sys

def wait_for_calibration(axis, timeout=30):
    """Wait for calibration to finish"""
    print("  Calibrating", end="", flush=True)
    start_time = time.time()
    
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > timeout:
            print(" TIMEOUT")
            return False
        print(".", end="", flush=True)
        time.sleep(0.5)
    
    print(" Done")
    return True

def dump_errors(odrv):
    """Print all of the ODrive's errors"""
    print("\n=== ERRORS ===")
    print(f"System: {odrv.error}")
    print(f"Axis0: {odrv.axis0.error}")
    print(f"  Motor: {odrv.axis0.motor.error}")
    print(f"  Encoder: {odrv.axis0.encoder.error}")
    print(f"  Controller: {odrv.axis0.controller.error}")
    print(f"Axis1: {odrv.axis1.error}")
    print(f"  Motor: {odrv.axis1.motor.error}")
    print(f"  Encoder: {odrv.axis1.encoder.error}")
    print(f"  Controller: {odrv.axis1.controller.error}")
    print()

def check_errors(odrv, axis_name):
    """Check an axis's errors"""
    axis = getattr(odrv, axis_name)
    
    if axis.error != 0:
        print(f"  {axis_name} error: 0x{axis.error:X}")
        dump_errors(odrv)
        return False
    
    if axis.motor.error != 0:
        print(f"  {axis_name} motor error: 0x{axis.motor.error:X}")
        return False
    
    if axis.encoder.error != 0:
        print(f"  {axis_name} encoder error: 0x{axis.encoder.error:X}")
        return False
    
    print(f"  {axis_name} OK")
    return True

def main():
    print("=" * 60)
    print("COMPLETE ODRIVE v3.6 CALIBRATION - SPI ENCODERS")
    print("=" * 60)
    
    # Connect
    print("\n[1/8] Connecting to the ODrive...")
    try:
        odrv0 = odrive.find_any()
        print(f"  Connected: {odrv0.serial_number}")
    except Exception as e:
        print(f"  Error: {e}")
        sys.exit(1)
    
    # Check encoders
    print("\n[2/8] Checking SPI encoders...")
    shadow0_before = odrv0.axis0.encoder.shadow_count
    shadow1_before = odrv0.axis1.encoder.shadow_count
    print(f"  Axis0 shadow_count: {shadow0_before}")
    print(f"  Axis1 shadow_count: {shadow1_before}")
    
    if shadow0_before == 0 and shadow1_before == 0:
        print("  Encoders are not reading anything! Check the SPI wiring.")
        sys.exit(1)
    
    print("  Encoders are working")
    
    # Base configuration (normally already done)
    print("\n[3/8] Checking base configuration...")
    print(f"  brake_resistance: {odrv0.config.brake_resistance}")
    print(f"  dc_max_positive_current: {odrv0.config.dc_max_positive_current}")
    print(f"  Axis0 current_lim: {odrv0.axis0.motor.config.current_lim}")
    print(f"  Axis1 current_lim: {odrv0.axis1.motor.config.current_lim}")
    
    # IMPORTANT: increase calibration current
    print("\n[4/8] Configuring calibration current...")
    odrv0.axis0.motor.config.calibration_current = 10.0
    odrv0.axis1.motor.config.calibration_current = 10.0
    print("  calibration_current = 10A (for both axes)")
    
    # Check that the motor is securely mounted
    print("\nWARNING")
    print("  The motors MUST be securely mounted!")
    print("  They will spin during calibration.")
    response = input("\n  Motors mounted? (yes/no): ")
    if response.lower() not in ['oui', 'o', 'yes', 'y']:
        print("  Calibration cancelled")
        sys.exit(0)
    
    # Clear errors
    print("\n[5/8] Clearing errors...")
    odrv0.clear_errors()
    time.sleep(0.5)
    
    # MOTOR calibration only (not the encoder yet)
    print("\n[6/8] Calibrating Axis0 MOTOR...")
    odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    if not wait_for_calibration(odrv0.axis0, timeout=30):
        print("  Axis0 motor calibration failed")
        dump_errors(odrv0)
        sys.exit(1)
    
    if not check_errors(odrv0, "axis0"):
        print("  Error after Axis0 motor calibration")
        sys.exit(1)
    
    # Save motor parameters
    print("  Saving phase_resistance and phase_inductance...")
    print(f"     phase_resistance: {odrv0.axis0.motor.config.phase_resistance:.6f}")
    print(f"     phase_inductance: {odrv0.axis0.motor.config.phase_inductance:.9f}")
    odrv0.axis0.motor.config.pre_calibrated = True
    
    print("\n[6/8] Calibrating Axis1 MOTOR...")
    odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    if not wait_for_calibration(odrv0.axis1, timeout=30):
        print("  Axis1 motor calibration failed")
        dump_errors(odrv0)
        sys.exit(1)
    
    if not check_errors(odrv0, "axis1"):
        print("  Error after Axis1 motor calibration")
        sys.exit(1)
    
    print("  Saving phase_resistance and phase_inductance...")
    print(f"     phase_resistance: {odrv0.axis1.motor.config.phase_resistance:.6f}")
    print(f"     phase_inductance: {odrv0.axis1.motor.config.phase_inductance:.9f}")
    odrv0.axis1.motor.config.pre_calibrated = True
    
    # ENCODER offset calibration
    print("\n[7/8] Calibrating Axis0 ENCODER OFFSET...")
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    
    if not wait_for_calibration(odrv0.axis0, timeout=30):
        print("  Axis0 encoder calibration failed")
        dump_errors(odrv0)
        sys.exit(1)
    
    if not check_errors(odrv0, "axis0"):
        print("  Error after Axis0 encoder calibration")
        sys.exit(1)
    
    print(f"  Offset saved: {odrv0.axis0.encoder.config.offset}")
    odrv0.axis0.encoder.config.pre_calibrated = True
    
    print("\n[7/8] Calibrating Axis1 ENCODER OFFSET...")
    odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    
    if not wait_for_calibration(odrv0.axis1, timeout=30):
        print("  Axis1 encoder calibration failed")
        dump_errors(odrv0)
        sys.exit(1)
    
    if not check_errors(odrv0, "axis1"):
        print("  Error after Axis1 encoder calibration")
        sys.exit(1)
    
    print(f"  Offset saved: {odrv0.axis1.encoder.config.offset}")
    odrv0.axis1.encoder.config.pre_calibrated = True
    
    # Startup configuration
    print("\n[8/8] Configuring closed-loop startup...")
    odrv0.axis0.config.startup_motor_calibration = False
    odrv0.axis0.config.startup_encoder_index_search = False
    odrv0.axis0.config.startup_encoder_offset_calibration = False
    odrv0.axis0.config.startup_closed_loop_control = True
    
    odrv0.axis1.config.startup_motor_calibration = False
    odrv0.axis1.config.startup_encoder_index_search = False
    odrv0.axis1.config.startup_encoder_offset_calibration = False
    odrv0.axis1.config.startup_closed_loop_control = True
    
    print("  Startup configured")
    
    # Final save
    print("\nSaving configuration...")
    odrv0.save_configuration()
    print("  Configuration saved")
    
    print("\n" + "=" * 60)
    print("FULL CALIBRATION SUCCEEDED!")
    print("=" * 60)
    print("\nBoth axes are calibrated and ready for CLOSED_LOOP_CONTROL")
    print("\nThe ODrive is about to reboot...")
    
    odrv0.reboot()
    
    print("\nNEXT STEPS:")
    print("  1. Wait 5 seconds for the ODrive to reboot")
    print("  2. Reconnect with odrivetool")
    print("  3. The axes should automatically be in CLOSED_LOOP_CONTROL")
    print("  4. Test with: odrv0.axis0.controller.input_pos = 1")
    print("  5. Launch the ROS2 hardware layer!")

if __name__ == "__main__":
    main()
