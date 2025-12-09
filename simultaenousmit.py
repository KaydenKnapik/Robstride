import can
import time
import struct
import math
import numpy as np
import sys
import traceback

# --- CONFIGURATION ---
CAN_INTERFACE = 'can1'
HOST_ID = 0xFD

# The motors we want to control simultaneously
MOTORS_TO_TEST = [2, 5] 

# Control Gains (Adjust if motors vibrate or are too loose)
# O5 usually handles higher gains, O2 needs slightly lower, 
# but 60/4 is usually a safe starting point for position control.
KP_GAIN = 3.0 
KD_GAIN = 1.0

# --- PROTOCOL CONSTANTS ---
MUX_ENABLE = 0x03
MUX_CONTROL = 0x01
MUX_DISABLE = 0x04

# --- MOTOR PARAMETERS ---
MOTOR_TYPE_PARAMS = {
    'O2': { 
        'name': 'Robstride O2',
        'P_MIN': -12.57, 'P_MAX': 12.57,
        'V_MIN': -44.0,  'V_MAX': 44.0,
        'T_MIN': -17.0,  'T_MAX': 17.0,
        'KP_MIN': 0.0,   'KP_MAX': 500.0,
        'KD_MIN': 0.0,   'KD_MAX': 5.0,
    },
    'O3': { 
        'name': 'Robstride O3',
        'P_MIN': -12.57, 'P_MAX': 12.57,
        'V_MIN': -20.0,  'V_MAX': 20.0,
        'T_MIN': -60.0,  'T_MAX': 60.0,
        'KP_MIN': 0.0,   'KP_MAX': 5000.0,
        'KD_MIN': 0.0,   'KD_MAX': 100.0,
    }, # Added missing comma here
    'O5': { 
        'name': 'Robstride O5',
        'P_MIN': -12.57, 'P_MAX': 12.57,
        'V_MIN': -50.0,  'V_MAX': 50.0,
        'T_MIN': -5.5,   'T_MAX': 5.5,
        'KP_MIN': 0.0,   'KP_MAX': 500.0,
        'KD_MIN': 0.0,   'KD_MAX': 5.0,
    }
}

# Map IDs to their Types
MOTOR_ID_TO_TYPE_MAP = {
    2: 'O2', # The 02 Motor
    5: 'O5'  # The 05 Motor
}

def scale_value_to_u16(value, v_min, v_max):
    """Clips and scales a float value to a 16-bit unsigned integer."""
    scaled = 65535.0 * (np.clip(value, v_min, v_max) - v_min) / (v_max - v_min)
    return int(scaled)

def send_control_command(bus, motor_id, pos, vel, kp, kd, torque, params):
    """
    Builds and sends the MIT control command using the correct scaling
    parameters for the specified motor type.
    """
    # 1. Scale all values using the provided params dictionary
    angle_u16 = scale_value_to_u16(pos, params['P_MIN'], params['P_MAX'])
    vel_u16 = scale_value_to_u16(vel, params['V_MIN'], params['V_MAX'])
    kp_u16 = scale_value_to_u16(kp, params['KP_MIN'], params['KP_MAX'])
    kd_u16 = scale_value_to_u16(kd, params['KD_MIN'], params['KD_MAX'])
    torque_u16 = scale_value_to_u16(torque, params['T_MIN'], params['T_MAX'])

    # 2. Build the CAN Arbitration ID
    mux_part = (MUX_CONTROL & 0xFF) << 24
    torque_part = (torque_u16 & 0xFFFF) << 8
    id_part = motor_id & 0xFF
    arbitration_id = mux_part | torque_part | id_part

    # 3. Build the 8-byte Data Payload (Big-Endian)
    data = struct.pack('>HHHH', angle_u16, vel_u16, kp_u16, kd_u16)

    msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True, dlc=8)
    bus.send(msg)

def get_motor_params(motor_id):
    m_type = MOTOR_ID_TO_TYPE_MAP.get(motor_id)
    if not m_type:
        raise ValueError(f"Motor ID {motor_id} not found in TYPE MAP.")
    return MOTOR_TYPE_PARAMS[m_type]

# --- MAIN EXECUTION BLOCK ---
if __name__ == "__main__":
    bus = None

    print("="*50)
    print(f"Simultaneous Test for Motor IDs: {MOTORS_TO_TEST}")
    print("="*50)

    # 1. Get User Input
    try:
        user_input = input("Enter Target Position (Radians) > ")
        target_pos = float(user_input)
    except ValueError:
        print("Invalid number entered. Exiting.")
        sys.exit(1)

    input("Ensure motors are powered. Press Enter to START...")

    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
        print(f"Connected to CAN.")

        # --- STEP 1: ENABLE MOTORS ---
        print("\n[1] Enabling Motors...")
        for mid in MOTORS_TO_TEST:
            enable_id = (MUX_ENABLE << 24) | (HOST_ID << 8) | mid
            bus.send(can.Message(arbitration_id=enable_id, is_extended_id=True, dlc=8))
        
        time.sleep(1) # Wait for them to wake up

        # --- STEP 2: MOVE TO TARGET ---
        print(f"\n[2] Moving both motors to {target_pos} rad...")
        
        # We loop quickly to send commands "simultaneously"
        for mid in MOTORS_TO_TEST:
            params = get_motor_params(mid)
            send_control_command(
                bus, mid, target_pos, 0.0, KP_GAIN, KD_GAIN, 0.0, params
            )
        
        print("-> Holding for 5 seconds...")
        time.sleep(5)

        # --- STEP 3: MOVE TO ZERO ---
        print(f"\n[3] Moving both motors back to 0.0 rad...")
        
        for mid in MOTORS_TO_TEST:
            params = get_motor_params(mid)
            send_control_command(
                bus, mid, 0.0, 0.0, KP_GAIN, KD_GAIN, 0.0, params
            )

        print("-> Holding for 2 seconds before disable...")
        time.sleep(2)

    except Exception as e:
        print(f"\n--- AN ERROR OCCURRED ---")
        traceback.print_exc()

    finally:
        if bus:
            print(f"\n[Final] Disabling all motors...")
            for mid in MOTORS_TO_TEST:
                disable_id = (MUX_DISABLE << 24) | (HOST_ID << 8) | mid
                bus.send(can.Message(arbitration_id=disable_id, is_extended_id=True, dlc=8))
            
            bus.shutdown()
            print("Sequence Complete.")