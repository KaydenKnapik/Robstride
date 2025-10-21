import can
import time
import struct
import math
import numpy as np
import enum
import sys
import traceback

# --- CONFIGURATION ---
CAN_INTERFACE = 'can0'
# <-- IMPORTANT: SET THE ID OF THE MOTOR YOU WANT TO TEST
MOTOR_ID = 20 # Example: 18=HipRoll(O3), 20=Ankle(O2)
HOST_ID = 0xFD

# --- YOUR TEST TARGETS ---
# These values will be sent to the selected motor.
TARGET_POSITION_RAD = 0.0
TARGET_VELOCITY_RAD_S = 0.0
TARGET_TORQUE_NM = 0.0
KP_GAIN = 60.0  # Remember to use gains appropriate for the motor being tested
KD_GAIN = 4

# --- PROTOCOL CONSTANTS ---
MUX_ENABLE = 0x03
MUX_CONTROL = 0x01
MUX_DISABLE = 0x04

# --- NEW: Centralized Parameter Definitions for Each Motor Type ---
MOTOR_TYPE_PARAMS = {
    'O2': { # For the Ankle motor
        'name': 'Robstride O2',
        'P_MIN': -12.57, 'P_MAX': 12.57,
        'V_MIN': -44.0,  'V_MAX': 44.0,
        'T_MIN': -17.0,  'T_MAX': 17.0,
        'KP_MIN': 0.0,   'KP_MAX': 500.0,
        'KD_MIN': 0.0,   'KD_MAX': 5.0,
    },
    'O3': { # For Hip and Knee motors
        'name': 'Robstride O3',
        'P_MIN': -12.57, 'P_MAX': 12.57,
        'V_MIN': -20.0,  'V_MAX': 20.0,
        'T_MIN': -60.0,  'T_MAX': 60.0,
        'KP_MIN': 0.0,   'KP_MAX': 5000.0,
        'KD_MIN': 0.0,   'KD_MAX': 100.0,
    }
}

# --- NEW: Map each specific motor ID to its type ---
MOTOR_ID_TO_TYPE_MAP = {
    # --- O3 Motors ---
    19: 'O3', # R_HipYaw
    18: 'O3', # R_HipRoll
    16: 'O3', # R_HipPitch
    17: 'O3', # R_Knee
    # --- O2 Motor ---
    20: 'O2'  # R_Ankle
}

def scale_value_to_u16(value, v_min, v_max):
    """Clips and scales a float value to a 16-bit unsigned integer."""
    scaled = 65535.0 * (np.clip(value, v_min, v_max) - v_min) / (v_max - v_min)
    return int(scaled)

# --- MODIFIED: Function now requires motor-specific params ---
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

# --- MAIN EXECUTION BLOCK (Simplified for clarity) ---
if __name__ == "__main__":
    bus = None

    # --- NEW: Pre-flight check to get correct parameters ---
    motor_type = MOTOR_ID_TO_TYPE_MAP.get(MOTOR_ID)
    if not motor_type:
        print(f"\n--- FATAL CONFIGURATION ERROR ---", file=sys.stderr)
        print(f"Motor with ID '{MOTOR_ID}' is not defined in the MOTOR_ID_TO_TYPE_MAP.", file=sys.stderr)
        print("Please add it to the dictionary before running.", file=sys.stderr)
        sys.exit(1)

    motor_params = MOTOR_TYPE_PARAMS[motor_type]
    
    print("="*50)
    print(f"Preparing to test Motor ID: {MOTOR_ID}")
    print(f"Detected Motor Type: {motor_params['name']}")
    print("="*50)

    input("Have you power-cycled the motor? Press Enter to run the script.")

    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
        print(f"Connected. Executing kscale-based command sequence...\n")

        # Step 1: Enable the motor.
        print("Step 1: Sending Enable command...")
        enable_id = (MUX_ENABLE << 24) | (HOST_ID << 8) | MOTOR_ID
        bus.send(can.Message(arbitration_id=enable_id, is_extended_id=True, dlc=8))
        time.sleep(1)
        print("-> Motor should be enabled.")

        # Step 2: Send the correctly formatted control command.
        print(f"\nStep 2: Sending Control Command for {TARGET_POSITION_RAD:.2f} rad...")
        print(f"  using Kp={KP_GAIN}, Kd={KD_GAIN} with '{motor_params['name']}' scaling.")
        send_control_command(
            bus,
            MOTOR_ID, # Pass the specific ID
            TARGET_POSITION_RAD,
            TARGET_VELOCITY_RAD_S,
            KP_GAIN,
            KD_GAIN,
            TARGET_TORQUE_NM,
            motor_params # Pass the correct parameters
        )
        print("-> Command sent. It should move correctly now. Holding for 10 seconds...")
        time.sleep(30)

    except Exception as e:
        print(f"\n--- AN ERROR OCCURRED ---")
        traceback.print_exc()
    finally:
        if bus:
            print(f"\nFinal Step: Disabling motor...")
            disable_id = (MUX_DISABLE << 24) | (HOST_ID << 8) | MOTOR_ID
            bus.send(can.Message(arbitration_id=disable_id, is_extended_id=True, dlc=8))
            bus.shutdown()
            print("Motor disabled and CAN bus shut down.")