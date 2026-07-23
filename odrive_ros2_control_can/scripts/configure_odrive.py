#!/usr/bin/env python3
"""
One-time ODrive setup script. Configures each axis FULLY, one at a time,
sequentially, so calibration current draw never overlaps between axes.
"""
import sys
import time
import atexit
import odrive
from odrive.utils import dump_errors

AXES_TO_CONFIGURE = ["axis0", "axis1"]

VEL_LIMIT = 15.0
CURRENT_LIM = 15.0
POS_GAIN = 20.0
VEL_GAIN = 0.16
VEL_INTEGRATOR_GAIN = 0.32

WAIT_MOTOR_CAL_S = 5.0
WAIT_ENCODER_CAL_S = 5.0
WAIT_CLOSED_LOOP_S = 1.0


def force_idle(odrv):
    try:
        for axis_name in AXES_TO_CONFIGURE:
            getattr(odrv, axis_name).requested_state = 1
        print("Safety: all axes forced to IDLE.")
    except Exception as e:
        print(f"Safety: could not force IDLE ({e}). Power-cycle the ODrive if unsure of its state.")


def configure_axis_fully(odrv, axis_name):
    axis = getattr(odrv, axis_name)

    print(f"[{axis_name}] Setting limits and gains...")
    axis.controller.config.vel_limit = VEL_LIMIT
    axis.motor.config.current_lim = CURRENT_LIM
    axis.controller.config.pos_gain = POS_GAIN
    axis.controller.config.vel_gain = VEL_GAIN
    axis.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN

    print(f"[{axis_name}] Motor calibration...")
    axis.requested_state = 4
    time.sleep(WAIT_MOTOR_CAL_S)
    if axis.motor.error != 0 or axis.error != 0:
        print(f"[{axis_name}] FAILED motor calibration.")
        dump_errors(odrv)
        return False

    print(f"[{axis_name}] Encoder offset calibration...")
    axis.requested_state = 7
    time.sleep(WAIT_ENCODER_CAL_S)
    if not axis.encoder.is_ready:
        print(f"[{axis_name}] FAILED: encoder not ready.")
        dump_errors(odrv)
        return False

    print(f"[{axis_name}] Entering closed-loop control...")
    axis.requested_state = 8
    time.sleep(WAIT_CLOSED_LOOP_S)
    if axis.current_state != 8:
        print(f"[{axis_name}] FAILED: not in closed-loop (state={axis.current_state}).")
        dump_errors(odrv)
        return False

    print(f"[{axis_name}] OK - closed-loop control active")
    return True


def main():
    print("Connecting to ODrive...")
    odrv = odrive.find_any(timeout=10)
    print(f"Connected: {odrv.serial_number}")

    print("Clearing errors...")
    odrv.clear_errors()

    all_ok = True
    try:
        for axis_name in AXES_TO_CONFIGURE:
            ok = configure_axis_fully(odrv, axis_name)
            all_ok = all_ok and ok
            if not ok:
                break
    except (KeyboardInterrupt, Exception):
        print("Interrupted or crashed - forcing IDLE for safety.")
        force_idle(odrv)
        raise

    if not all_ok:
        print("FAILED: one or more axes did not configure correctly.")
        force_idle(odrv)
        sys.exit(1)

    print("SUCCESS: all axes configured and in closed-loop control. Leaving them active.")
    sys.exit(0)


if __name__ == "__main__":
    main()
