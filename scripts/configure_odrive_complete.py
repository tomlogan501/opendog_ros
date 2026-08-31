#!/usr/bin/env python3
"""
Complete configuration script for an ODrive v3.6 for OpenDog v3
Configures motor + encoder + calibration + automatic startup

Usage:
    python3 configure_odrive_complete.py [serial_number]
    
If serial_number is not provided, configures the first ODrive found.
"""

import odrive
from odrive.enums import *
import sys
import time

def wait_for_state(axis, target_state, timeout=30):
    """Wait for the axis to reach a given state"""
    start_time = time.time()
    while axis.current_state != target_state:
        if time.time() - start_time > timeout:
            print(f"TIMEOUT: axis did not reach state {target_state}")
            return False
        time.sleep(0.1)
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

def configure_axis(odrv, axis_num):
    """Configure a full axis (motor + encoder + calibration)"""
    
    axis = getattr(odrv, f"axis{axis_num}")
    
    print(f"\n{'='*60}")
    print(f"CONFIGURING AXIS {axis_num}")
    print(f"{'='*60}\n")
    
    # 1. Motor configuration
    print(f"1. Configuring motor...")
    axis.motor.config.pole_pairs = 20
    axis.motor.config.torque_constant = 8.27 / 100
    axis.motor.config.motor_type = 0  # MOTOR_TYPE_HIGH_CURRENT
    axis.motor.config.current_lim = 10.0  # Operating current limit
    axis.motor.config.current_lim_margin = 5.0
    axis.motor.config.calibration_current = 5.0  # Reduced calibration current, 5A
    axis.motor.config.resistance_calib_max_voltage = 2.0  # Reduced voltage
    axis.motor.config.requested_current_range = 15.0  # Reduced current range
    print("   Motor configured (calibration at 5A, operating limit at 10A)")
    
    # 2. SPI encoder configuration
    print(f"2. Configuring SPI encoder...")
    axis.encoder.config.mode = 257  # MODE_SPI_ABS_AMS
    axis.encoder.config.cpr = 16384
    axis.encoder.config.calib_scan_distance = 150.0
    
    if axis_num == 0:
        axis.encoder.config.abs_spi_cs_gpio_pin = 5
    else:
        axis.encoder.config.abs_spi_cs_gpio_pin = 4
    
    print("   SPI encoder configured")
    
    # 3. Controller configuration
    print(f"3. Configuring controller...")
    axis.controller.config.control_mode = 3  # CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = 5    # INPUT_MODE_TRAP_TRAJ
    axis.controller.config.vel_limit = 50.0
    axis.controller.config.pos_gain = 20.0
    axis.controller.config.vel_gain = 0.16
    axis.controller.config.vel_integrator_gain = 0.32
    print("   Controller configured")
    
    # 4. Clear errors
    print(f"4. Clearing errors...")
    odrv.clear_errors()
    time.sleep(0.5)
    
    # 5. Motor calibration
    print(f"5. Calibrating motor (can take 10-15 seconds)...")
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    if not wait_for_state(axis, AXIS_STATE_IDLE, timeout=30):
        print("   Motor calibration failed")
        dump_errors(odrv)
        return False
    
    if axis.motor.error != 0:
        print("   Motor error detected")
        dump_errors(odrv)
        return False
    
    print(f"   Motor calibrated")
    print(f"      Phase resistance: {axis.motor.config.phase_resistance:.6f} ohm")
    print(f"      Phase inductance: {axis.motor.config.phase_inductance:.9f} H")
    
    # Mark the motor as pre-calibrated
    axis.motor.config.pre_calibrated = True
    
    # 6. Encoder offset calibration
    print(f"6. Calibrating encoder offset (can take 10-15 seconds)...")
    axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    
    if not wait_for_state(axis, AXIS_STATE_IDLE, timeout=30):
        print("   Encoder calibration failed")
        dump_errors(odrv)
        return False
    
    if axis.encoder.error != 0:
        print("   Encoder error detected")
        dump_errors(odrv)
        return False
    
    print(f"   Encoder calibrated")
    
    # Mark the encoder as pre-calibrated
    axis.encoder.config.pre_calibrated = True
    
    # 7. Automatic startup configuration
    print(f"7. Configuring automatic startup...")
    axis.config.startup_encoder_index_search = False
    axis.config.startup_encoder_offset_calibration = False
    axis.config.startup_motor_calibration = False
    axis.config.startup_closed_loop_control = True
    print("   Automatic CLOSED_LOOP startup enabled")
    
    # 8. Quick CLOSED_LOOP test
    print(f"8. Testing CLOSED_LOOP...")
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(1)
    
    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("   Failed to enter CLOSED_LOOP")
        dump_errors(odrv)
        return False
    
    print(f"   CLOSED_LOOP OK")
    print(f"      Position: {axis.encoder.pos_estimate:.3f} turns")
    print(f"      Velocity: {axis.encoder.vel_estimate:.3f} turns/s")
    
    # Return to IDLE
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.5)
    
    print(f"\nAXIS {axis_num} CONFIGURED SUCCESSFULLY!\n")
    return True

def main():
    print("="*60)
    print("COMPLETE ODRIVE v3.6 CONFIGURATION FOR OPENDOG v3")
    print("="*60)
    
    # Connect to the ODrive
    print("\nSearching for an ODrive...")
    
    if len(sys.argv) > 1:
        serial_number = sys.argv[1]
        print(f"   Connecting to serial: {serial_number}")
        try:
            odrv = odrive.find_any(serial_number=serial_number)
        except:
            print(f"Could not find ODrive with serial {serial_number}")
            return 1
    else:
        print("   Connecting to the first ODrive found...")
        try:
            odrv = odrive.find_any()
        except:
            print("No ODrive found!")
            print("\nCheck that the ODrive is connected via USB")
            return 1
    
    print(f"ODrive connected!")
    print(f"   Serial: {odrv.serial_number}")
    print(f"   Firmware: v{odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"   Hardware: v{odrv.hw_version_major}.{odrv.hw_version_minor}")
    
    # Base configuration
    print("\nBase configuration...")
    odrv.config.enable_brake_resistor = True
    odrv.config.brake_resistance = 2.0
    odrv.config.dc_bus_overvoltage_trip_level = 56.0
    odrv.config.dc_max_positive_current = 20.0
    odrv.config.dc_max_negative_current = -3.0
    print("   Base configuration OK")
    
    # Ask which axes to configure
    print("\nWhich axes do you want to configure?")
    print("   1. Axis 0 only")
    print("   2. Axis 1 only")
    print("   3. Both axes")
    
    choice = input("\nYour choice (1/2/3) [3]: ").strip()
    if not choice:
        choice = "3"
    
    axes_to_configure = []
    if choice == "1":
        axes_to_configure = [0]
    elif choice == "2":
        axes_to_configure = [1]
    else:
        axes_to_configure = [0, 1]
    
    # Configure the axes
    success = True
    for axis_num in axes_to_configure:
        if not configure_axis(odrv, axis_num):
            success = False
            print(f"\nFailed to configure Axis {axis_num}")
            break
    
    if not success:
        print("\nCONFIGURATION FAILED")
        print("\nCheck:")
        print("   - Motor connections (3 phases)")
        print("   - Encoder connections (SPI)")
        print("   - Power supply (> 24V)")
        return 1
    
    # Save the configuration
    print("\nSaving configuration...")
    try:
        odrv.save_configuration()
        print("   Configuration saved")
    except:
        print("   Save failed")
        return 1
    
    print("\n" + "="*60)
    print("CONFIGURATION COMPLETED SUCCESSFULLY!")
    print("="*60)
    print("\nSummary:")
    for axis_num in axes_to_configure:
        print(f"   Axis {axis_num}: Motor calibrated, Encoder calibrated, CLOSED_LOOP startup enabled")
    
    print("\nRESTART RECOMMENDED")
    print("   After restarting, the ODrive will start automatically in CLOSED_LOOP")
    
    reboot = input("\nRestart the ODrive now? (y/n) [y]: ").strip().lower()
    if not reboot or reboot == "y":
        print("\nRestarting the ODrive...")
        try:
            odrv.reboot()
            print("   ODrive restarted")
        except:
            print("   Normal disconnect during restart")
    
    print("\nDONE!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
