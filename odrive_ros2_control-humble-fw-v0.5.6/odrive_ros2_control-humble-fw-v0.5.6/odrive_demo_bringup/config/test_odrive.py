import odrive
from odrive.enums import *
import time
import math
def main():
    # Find a connected ODrive (this might take a few seconds)
    print("Finding an ODrive...")
    odrv0 = odrive.find_any()
    print("Found ODrive with serial number: {}".format(odrv0.serial_number))
    # Ensure the motor is in closed-loop control and position control mode
    if odrv0.axis1.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("Setting motor to closed loop control mode...")
        odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)  # Wait for the state to be updated
    odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    # Parameters for the sinusoidal motion
    amplitude = 1.0  # Amplitude of the sine wave in encoder counts or rotations
    frequency = 0.5  # Frequency of the sine wave in Hz (cycles per second)
    duration = 10  # Duration to run the sinusoidal motion in seconds
    start_time = time.time()
    print("Starting sinusoidal motion...")
    while (time.time() - start_time) < duration:
        elapsed_time = time.time() - start_time
        target_position = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)
        odrv0.axis1.controller.input_pos = target_position
        time.sleep(0.01)  # Small delay to avoid flooding the ODrive with commands
    print("Sinusoidal motion complete.")
if __name__ == "__main__":
    main()
