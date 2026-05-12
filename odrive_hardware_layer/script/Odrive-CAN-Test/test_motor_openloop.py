import can
import struct
import time


# CONFIGURATION
CAN_CHANNEL   = 'can0'
CAN_INTERFACE = 'socketcan'
NODE_ID       = 0

# ODrive CANSimple COMMAND IDs
# arbitration_id = (node_id << 5) | cmd_id
CMD_HEARTBEAT        = 0x001
CMD_SET_AXIS_STATE   = 0x007
CMD_GET_ENCODER_EST  = 0x009
CMD_SET_CTRL_MODE    = 0x00B
CMD_SET_INPUT_POS    = 0x00C
CMD_SET_INPUT_VEL    = 0x00D
CMD_SET_INPUT_TORQUE = 0x00E
CMD_CLEAR_ERRORS     = 0x018


# ODrive AXIS STATES
AXIS_STATE_IDLE              = 1
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_CLOSED_LOOP       = 8

# ODrive CONTROL MODES
CONTROL_MODE_TORQUE   = 1
CONTROL_MODE_VELOCITY = 2
CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1

# HELPER
def arb_id(node_id, cmd_id):
    return (node_id << 5) | cmd_id

def connect():
    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE)
    print(f"Connected to {CAN_CHANNEL}")
    return bus

# SEND COMMANDS  (Host → ODrive)
def set_axis_state(bus, state):
    bus.send(can.Message(
        arbitration_id=arb_id(NODE_ID, CMD_SET_AXIS_STATE),
        data=struct.pack('<I', state),
        is_extended_id=False
    ))
    names = {1:'IDLE', 4:'MOTOR_CALIBRATION', 8:'CLOSED_LOOP'}
    print(f"  [SEND] Axis state {names.get(state, str(state))}")

def set_control_mode(bus, ctrl_mode, input_mode):
    bus.send(can.Message(
        arbitration_id=arb_id(NODE_ID, CMD_SET_CTRL_MODE),
        data=struct.pack('<II', ctrl_mode, input_mode),
        is_extended_id=False
    ))
    print(f"  [SEND] Control mode {ctrl_mode}, Input mode {input_mode}")

def set_input_velocity(bus, velocity, torque_ff=0.0):
    bus.send(can.Message(
        arbitration_id=arb_id(NODE_ID, CMD_SET_INPUT_VEL),
        data=struct.pack('<ff', velocity, torque_ff),
        is_extended_id=False
    ))
    print(f"  [SEND] Velocity {velocity} turns/sec")

def set_input_position(bus, position, vel_ff=0, torque_ff=0):
    bus.send(can.Message(
        arbitration_id=arb_id(NODE_ID, CMD_SET_INPUT_POS),
        data=struct.pack('<fhh', position, vel_ff, torque_ff),
        is_extended_id=False
    ))
    print(f"  [SEND] Position {position} turns")

def set_input_torque(bus, torque):
    bus.send(can.Message(
        arbitration_id=arb_id(NODE_ID, CMD_SET_INPUT_TORQUE),
        data=struct.pack('<f', torque),
        is_extended_id=False
    ))
    print(f"  [SEND] Torque {torque} Nm")

def clear_errors(bus):
    bus.send(can.Message(
        arbitration_id=arb_id(NODE_ID, CMD_CLEAR_ERRORS),
        data=[],
        is_extended_id=False
    ))
    print(f"  [SEND] Clear errors")


def get_heartbeat(bus, timeout=3.0):
    """Returns (axis_error, axis_state) or None on timeout."""
    target = arb_id(NODE_ID, CMD_HEARTBEAT)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=0.5)
        if msg and msg.arbitration_id == target:
            error = struct.unpack('<I', bytes(msg.data[0:4]))[0]
            state = msg.data[4]
            return error, state
    return None

def get_encoder_estimates(bus, timeout=2.0):
    """Returns (pos_turns, vel_turns_per_sec) or (None, None)."""
    target = arb_id(NODE_ID, CMD_GET_ENCODER_EST)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=0.5)
        if msg and msg.arbitration_id == target:
            pos, vel = struct.unpack('<ff', bytes(msg.data[0:8]))
            return pos, vel
    return None, None

def print_status(bus):

    result = get_heartbeat(bus, timeout=2.0)
    if result:
        error, state = result
        names = {1:'IDLE', 4:'MOTOR_CALIBRATION', 8:'CLOSED_LOOP'}
        print(f"  State : {names.get(state, str(state))} ({state})")
        print(f"  Error : {hex(error)} {'OK' if error == 0 else 'ERROR!'}")
    else:
        print("  Heartbeat: no response")
    pos, vel = get_encoder_estimates(bus, timeout=1.0)
    if pos is not None:
        print(f"  Pos   : {pos:.4f} turns  ({pos*360:.2f} deg)")
        print(f"  Vel   : {vel:.4f} turns/sec")
    
# MAIN
def main():
    print("=" * 50)
    print("  ODrive CAN Motor Test — Python")
    print("  SocketCAN | DSD Tech SH-C30G")
    print("=" * 50)

    bus = connect()

    # Wait for heartbeat
    print("\nWaiting for ODrive heartbeat...")
    result = get_heartbeat(bus, timeout=5.0)
    if result is None:
        print("No heartbeat")
        bus.shutdown()
        return
    error, state = result
    print(f"ODrive alive! State={state}, Error={hex(error)}")

    # Clear errors
    clear_errors(bus)
    time.sleep(0.5)

    # Calibrate
    print("\nCalibrating motor (listen for beep)...")
    set_axis_state(bus, AXIS_STATE_MOTOR_CALIBRATION)
    time.sleep(8)

    # Check calibration result
    result = get_heartbeat(bus, timeout=3.0)
    if result:
        error, state = result
        if error != 0:
            print(f"Calibration failed: {hex(error)}")
            bus.shutdown()
            return
        print(f"Calibration done! State={state}, Error={hex(error)}")

    # Clear post-calibration errors
    clear_errors(bus)
    time.sleep(0.5)

    # Set velocity control mode
    set_control_mode(bus, CONTROL_MODE_VELOCITY, INPUT_MODE_PASSTHROUGH)
    time.sleep(0.3)

    # Enter closed loop
    print("\nEntering closed loop...")
    set_axis_state(bus, AXIS_STATE_CLOSED_LOOP)
    time.sleep(2)

    # Status before spinning
    print_status(bus)

    # Spin
    print("Spinning at 2.0 turns/sec for 5 seconds...")
    set_input_velocity(bus, 2.0)

    # Read feedback while spinning
    print("\nReading feedback:")
    for i in range(5):
        set_input_velocity(bus, 2.0)
        result = get_heartbeat(bus, timeout=1.0)
        if result:
            error, state = result
            names = {1:'IDLE', 8:'CLOSED_LOOP'}
            print(f"  [{i+1}] State={names.get(state,str(state))} Error={hex(error)}")
        time.sleep(0.8)

    # Stop
    print("\nStopping motor...")
    set_input_velocity(bus, 0.0)
    time.sleep(1)
    set_axis_state(bus, AXIS_STATE_IDLE)

    # Final status
    print_status(bus)

    bus.shutdown()
    print("=" * 50)
    print("  Test complete.")
    print("=" * 50)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted — stopping motor...")
        bus = connect()
        set_input_velocity(bus, 0.0)
        set_axis_state(bus, AXIS_STATE_IDLE)
        bus.shutdown()
        print("Stopped safely.")
