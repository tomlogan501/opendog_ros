import odrive
from odrive.enums import *
import time
import math

# =======================
# Global constants
# =======================
CURRENT_LIMIT_HIGH = 40.0      # A -> for high-current motors (formerly shoulders)
CURRENT_LIT_LOW = 2.5          # A -> for the other motors
TEST_INCREMENT = 0.2           # rad -> small displacement test
LOOP_PERIOD = 0.02             # s -> loop frequency (50 Hz)
SIN_FREQ = 2.0                 # Hz -> sinusoidal motion frequency
SIN_AMP_LEG = 0.4              # rad -> amplitude for legs
SIN_AMP_HIP = 0.2              # rad -> amplitude for hips

# Hip definitions and offsets
HIPS_AXES = [0x2, 0x3, 0x8, 0x9]
HIPS_OFFSETS = {0x2: 0.2, 0x3: 0.1, 0x8: 0.15, 0x9: 0.15}

# Shoulder-specific offsets (for holding position continuously)
SHOULDER_OFFSETS = {
    "FRONT_LEFT": 0.2,
    "FRONT_RIGHT": -0.2,
    "BACK_LEFT": 0.2,
    "BACK_RIGHT": -0.2
}


def main():
    print("Connecting to ODrives...")

    # Serial numbers of the 6 ODrives
    serials = [
        "335536633539",  # other
        "335836543539",  # other
        "3359366C3539",  # high current front left/right
        "3359366F3539",  # other
        "335436563539",  # other
        "3673385F3030"   # high current back left/right
    ]

    odrives = []
    for sn in serials:
        print(f"Connecting to ODrive SN {sn} ...")
        odrv = odrive.find_any(serial_number=sn)
        odrives.append(odrv)
        print(f"  -> Connected: {odrv.serial_number}")

    print("\nAll ODrives are connected.")

    # --- Build axis tables ---
    AxisTab = []  # all axes (odrive, axis_id)
    for odrv in odrives:
        AxisTab.append((odrv, 0))  # axis0
        AxisTab.append((odrv, 1))  # axis1

    # --- High-current motors (formerly shoulders) ---
    high_current_indices = [2, 5]  # indices of the high-current ODrives
    AxisTabHighCurrent = []
    for i, odrv in enumerate(odrives):
        if i in high_current_indices:
            AxisTabHighCurrent.append((odrv, 0))
            AxisTabHighCurrent.append((odrv, 1))

    # --- Function to read an axis's state ---
    def print_axis_state(odrv, axis_id, axis_name=""):
        if axis_id == 0:
            axis = odrv.axis0
        else:
            axis = odrv.axis1

        print(f"\n{axis_name} | Board {odrv.serial_number} Axis{axis_id}:")
        print(f"  -> Current State: {axis.current_state}")
        print(f"  -> Axis Error: {axis.error}")
        print(f"  -> Motor Error: {axis.motor.error}")
        print(f"  -> Encoder Error: {axis.encoder.error}")
        print(f"  -> Encoder Pos Estimate: {axis.encoder.pos_estimate:.3f} rad")
        print(f"  -> Encoder Vel Estimate: {axis.encoder.vel_estimate:.3f} rad/s")

    # --- Current limits ---
    print("\nConfiguring current limits:")
    for odrv, axis_id in AxisTab:
        is_high_current = any(odrv is hc_odrv for hc_odrv, _ in AxisTabHighCurrent)
        
        if is_high_current:
            current_limit = CURRENT_LIMIT_HIGH
            print(f"  -> HIGH CURRENT | Board {odrv.serial_number} Axis{axis_id} | Current = {current_limit} A")
        else:
            current_limit = CURRENT_LIT_LOW
            print(f"  -> LOW CURRENT | Board {odrv.serial_number} Axis{axis_id} | Current = {current_limit} A")

        if axis_id == 0:
            odrv.axis0.motor.config.current_lim = current_limit
        else:
            odrv.axis1.motor.config.current_lim = current_limit

    # Switch to position mode (Closed Loop)
    for odrv in odrives:
        for axis in [odrv.axis0, odrv.axis1]:
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

    time.sleep(1)  # settle

    # --- Read initial positions ---
    pos_init_map = {}
    for odrv in odrives:
        pos_init_map[(odrv, 0)] = odrv.axis0.encoder.pos_estimate
        pos_init_map[(odrv, 1)] = odrv.axis1.encoder.pos_estimate

    print("\n=== INITIAL POSITIONS OF ALL AXES ===")
    for idx, ((odrv, axis_id), pos) in enumerate(pos_init_map.items()):
        if (odrv, axis_id) in AxisTabHighCurrent:
            axis_type = "HIGH CURRENT"
        elif axis_id in HIPS_AXES:
            axis_type = "HIP"
        else:
            axis_type = "LEG"
        print(f"  {axis_type} | Board {odrv.serial_number} Axis{axis_id} = {pos:.3f} rad")

    # --- Apply shoulder-specific offsets ---
    shoulder_offsets_map = {
        (odrives[2], 0): pos_init_map[(odrives[2], 0)] + SHOULDER_OFFSETS["FRONT_LEFT"],
        (odrives[2], 1): pos_init_map[(odrives[2], 1)] + SHOULDER_OFFSETS["FRONT_RIGHT"],
        (odrives[5], 0): pos_init_map[(odrives[5], 0)] + SHOULDER_OFFSETS["BACK_LEFT"],
        (odrives[5], 1): pos_init_map[(odrives[5], 1)] + SHOULDER_OFFSETS["BACK_RIGHT"]
    }

    print("\nHolding shoulders at initial offsets...")
    for (odrv, axis_id), target_pos in shoulder_offsets_map.items():
        if axis_id == 0:
            odrv.axis0.controller.input_pos = target_pos
        else:
            odrv.axis1.controller.input_pos = target_pos
        
        # Determine the shoulder name for display
        if odrv == odrives[2]:
            position = "FRONT"
            side = "LEFT" if axis_id == 0 else "RIGHT"
        else:
            position = "BACK"
            side = "LEFT" if axis_id == 0 else "RIGHT"
        
        print(f"  -> {position} {side} SHOULDER | Board {odrv.serial_number} Axis{axis_id} = {target_pos:.3f} rad")

    # --- Sinusoidal loop (legs + hips only) ---
    t0 = time.monotonic()
    last_display_time = t0
    last_state_display_time = t0
    print("\nSinusoidal motion loop running (shoulders held fixed)... Ctrl+C to stop")
    print("Monitoring positions every 3 seconds...")

    try:
        while True:
            t1 = time.monotonic()

            # Periodically display position + current status
            if t1 - last_display_time > 3.0:  # every 3 seconds
                total_current = 0.0

                print("\n=== CURRENT POSITION & CURRENT STATE ===")
                print("HIGH CURRENT (shoulders with offsets):")
                for (odrv, axis_id) in shoulder_offsets_map:
                    if axis_id == 0:
                        current_pos = odrv.axis0.encoder.pos_estimate
                        m = odrv.axis0.motor
                    else:
                        current_pos = odrv.axis1.encoder.pos_estimate
                        m = odrv.axis1.motor

                    current_cur = (abs(m.current_meas_phA) + abs(m.current_meas_phB) + abs(m.current_meas_phC)) / 3.0
                    target_pos = shoulder_offsets_map[(odrv, axis_id)]
                    deviation = abs(current_pos - target_pos)
                    total_current += current_cur

                    # Determine the shoulder name for display
                    if odrv == odrives[2]:
                        position = "FRONT"
                        side = "LEFT" if axis_id == 0 else "RIGHT"
                    else:
                        position = "BACK"
                        side = "LEFT" if axis_id == 0 else "RIGHT"
                    
                    print(f"  {position} {side} SHOULDER | Board {odrv.serial_number} Axis{axis_id}: "
                          f"{current_pos:.3f} rad (target: {target_pos:.3f}, deviation: {deviation:.3f}) "
                          f"| Current: {current_cur:.2f} A")

                print("\nOTHER JOINTS (in sinusoidal motion):")
                for (odrv, axis_id) in AxisTab:
                    if (odrv, axis_id) not in shoulder_offsets_map:
                        if axis_id == 0:
                            current_pos = odrv.axis0.encoder.pos_estimate
                            m = odrv.axis0.motor
                        else:
                            current_pos = odrv.axis1.encoder.pos_estimate
                            m = odrv.axis1.motor

                        current_cur = (abs(m.current_meas_phA) + abs(m.current_meas_phB) + abs(m.current_meas_phC)) / 3.0
                        init_pos = pos_init_map[(odrv, axis_id)]
                        total_current += current_cur

                        if axis_id in HIPS_AXES:
                            axis_type = "HIP"
                        else:
                            axis_type = "LEG"
                            
                        print(f"  {axis_type} | Board {odrv.serial_number} Axis{axis_id}: "
                              f"{current_pos:.3f} rad (init: {init_pos:.3f}) "
                              f"| Current: {current_cur:.2f} A")

                print(f"\n>>> Estimated total consumption = {total_current:.2f} A <<<")
                last_display_time = t1

            # Periodically display axis states
            if t1 - last_state_display_time > 10.0:
                print("\n=== AXIS STATES ===")
                for odrv, axis_id in AxisTab:
                    if (odrv, axis_id) in shoulder_offsets_map:
                        if odrv == odrives[2]:
                            position = "FRONT"
                            side = "LEFT" if axis_id == 0 else "RIGHT"
                            axis_type = f"{position} {side} SHOULDER"
                        else:
                            position = "BACK"
                            side = "LEFT" if axis_id == 0 else "RIGHT"
                            axis_type = f"{position} {side} SHOULDER"
                    elif axis_id in HIPS_AXES:
                        axis_type = "HIP"
                    else:
                        axis_type = "LEG"
                    print_axis_state(odrv, axis_id, axis_type)
                last_state_display_time = t1

            # Sine setpoints for hips and legs
            setpoint_leg = SIN_AMP_LEG * math.sin((t1 - t0) * SIN_FREQ)
            setpoint_hip = SIN_AMP_HIP * math.sin((t1 - t0) * SIN_FREQ)

            for (odrv, axis_id) in AxisTab:
                if (odrv, axis_id) in shoulder_offsets_map:
                    # Hold shoulders fixed at their offsets
                    sp = shoulder_offsets_map[(odrv, axis_id)]
                elif axis_id in HIPS_AXES:
                    # Sinusoidal motion for hips
                    sp = setpoint_hip + HIPS_OFFSETS.get(axis_id, 0.0)
                else:
                    # Sinusoidal motion for legs
                    sp = setpoint_leg

                if axis_id == 0:
                    odrv.axis0.controller.input_pos = sp
                else:
                    odrv.axis1.controller.input_pos = sp

            time.sleep(LOOP_PERIOD)

    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":
    main()
