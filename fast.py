import can
import time
import struct
import math
import numpy as np
import sys
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
CAN_INTERFACE = 'can0'
MOTOR_ID = 1
HOST_ID = 0xFD

# --- SPEED SETTING ---
# We know you can do 3000Hz, so 1000Hz is safe and very smooth.
CONTROL_FREQ = 1000.0 
DT = 1.0 / CONTROL_FREQ

# --- MOTION PARAMETERS ---
SINE_AMPLITUDE = 1.0 
SINE_FREQUENCY = 1.0 # Faster wave (1 full cycle per second)

# --- GAINS ---
KP_GAIN = 20.0  
KD_GAIN = 1.0   

# --- PROTOCOL CONSTANTS ---
MUX_ENABLE = 0x03
MUX_CONTROL = 0x01
MUX_DISABLE = 0x04

# --- MOTOR PARAMS ---
MOTOR_TYPE_PARAMS = {
    'O2': { 'name': 'Robstride O2', 'P_MIN': -12.57, 'P_MAX': 12.57, 'V_MIN': -44.0, 'V_MAX': 44.0, 'T_MIN': -17.0, 'T_MAX': 17.0, 'KP_MIN': 0.0, 'KP_MAX': 500.0, 'KD_MIN': 0.0, 'KD_MAX': 5.0 },
    'O3': { 'name': 'Robstride O3', 'P_MIN': -12.57, 'P_MAX': 12.57, 'V_MIN': -20.0, 'V_MAX': 20.0, 'T_MIN': -60.0, 'T_MAX': 60.0, 'KP_MIN': 0.0, 'KP_MAX': 5000.0, 'KD_MIN': 0.0, 'KD_MAX': 100.0 },
    'O5': { 'name': 'Robstride O5', 'P_MIN': -12.57, 'P_MAX': 12.57, 'V_MIN': -50.0, 'V_MAX': 50.0, 'T_MIN': -5.5,  'T_MAX': 5.5,  'KP_MIN': 0.0, 'KP_MAX': 500.0, 'KD_MIN': 0.0, 'KD_MAX': 5.0 }
}
MOTOR_ID_TO_TYPE_MAP = { 19: 'O3', 18: 'O3', 16: 'O3', 17: 'O3', 2: 'O2', 5: 'O5', 127: 'O2', 1: 'O2' }

current_motor_state = {'pos': 0.0, 'vel': 0.0, 'last_update': 0.0}
history = {"time": [], "target": [], "actual": []}

def scale_value_to_u16(value, v_min, v_max):
    return int(65535.0 * (np.clip(value, v_min, v_max) - v_min) / (v_max - v_min))

def unscale_u16_to_float(val_u16, v_min, v_max):
    return (float(val_u16) / 65535.0) * (v_max - v_min) + v_min

def send_control_command(bus, motor_id, pos, vel, kp, kd, torque, params):
    angle_u16 = scale_value_to_u16(pos, params['P_MIN'], params['P_MAX'])
    vel_u16 = scale_value_to_u16(vel, params['V_MIN'], params['V_MAX'])
    kp_u16 = scale_value_to_u16(kp, params['KP_MIN'], params['KP_MAX'])
    kd_u16 = scale_value_to_u16(kd, params['KD_MIN'], params['KD_MAX'])
    torque_u16 = scale_value_to_u16(torque, params['T_MIN'], params['T_MAX'])

    mux_part = (MUX_CONTROL & 0xFF) << 24
    torque_part = (torque_u16 & 0xFFFF) << 8
    id_part = motor_id & 0xFF
    arbitration_id = mux_part | torque_part | id_part

    data = struct.pack('>HHHH', angle_u16, vel_u16, kp_u16, kd_u16)
    bus.send(can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True, dlc=8))

def read_feedback(bus, params):
    while True:
        msg = bus.recv(timeout=0)
        if msg is None: break
        if msg.is_error_frame: continue

        # Check standard and biped ID locations
        extracted_motor_id = (msg.arbitration_id & 0xFF00) >> 8
        if extracted_motor_id != MOTOR_ID:
            extracted_motor_id = msg.arbitration_id & 0xFF

        if extracted_motor_id == MOTOR_ID:
            try:
                p_raw = struct.unpack('>H', msg.data[0:2])[0]
                v_raw = struct.unpack('>H', msg.data[2:4])[0]
                pos_rad = unscale_u16_to_float(p_raw, params['P_MIN'], params['P_MAX'])
                vel_rad = unscale_u16_to_float(v_raw, params['V_MIN'], params['V_MAX'])
                
                current_motor_state['pos'] = pos_rad
                current_motor_state['vel'] = vel_rad
                current_motor_state['last_update'] = time.time()
            except: pass

if __name__ == "__main__":
    motor_type = MOTOR_ID_TO_TYPE_MAP.get(MOTOR_ID)
    params = MOTOR_TYPE_PARAMS[motor_type]
    
    print(f"--- 1000 Hz HIGH PERFORMANCE TEST ---")
    print(f"Motor: {params['name']} (ID {MOTOR_ID})")
    
    bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')

    try:
        print("Enabling...")
        bus.send(can.Message(arbitration_id=(MUX_ENABLE << 24) | (HOST_ID << 8) | MOTOR_ID, is_extended_id=True, dlc=8))
        time.sleep(1)
        
        print("Running 1kHz Loop. Press Ctrl+C to stop.")
        start_time = time.time()
        last_loop_time = start_time
        
        while True:
            loop_start = time.time()
            elapsed_total = loop_start - start_time
            
            # 1. Read
            read_feedback(bus, params)
            
            # 2. Calculate (Fast Sine Wave)
            target_pos = SINE_AMPLITUDE * math.sin(2 * math.pi * SINE_FREQUENCY * elapsed_total)
            target_vel = (SINE_AMPLITUDE * 2 * math.pi * SINE_FREQUENCY) * math.cos(2 * math.pi * SINE_FREQUENCY * elapsed_total)
            
            # 3. Write
            send_control_command(bus, MOTOR_ID, target_pos, target_vel, KP_GAIN, KD_GAIN, 0.0, params)
            
            # 4. Record
            history["time"].append(elapsed_total)
            history["target"].append(target_pos)
            history["actual"].append(current_motor_state['pos'])

            # 5. Print Stats (Every 100 cycles = 10Hz print rate)
            if int(elapsed_total * CONTROL_FREQ) % 100 == 0:
                real_dt = loop_start - last_loop_time
                real_hz = 1.0 / real_dt if real_dt > 0 else 0.0
                print(f"Tgt: {target_pos:6.3f} | Act: {current_motor_state['pos']:6.3f} | Rate: {real_hz:4.0f}Hz")
            
            last_loop_time = loop_start
            
            # 6. Sleep to maintain 1000Hz exactly
            process_duration = time.time() - loop_start
            sleep_time = DT - process_duration
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\nSTOPPING...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Disabling...")
        bus.send(can.Message(arbitration_id=(MUX_DISABLE << 24) | (HOST_ID << 8) | MOTOR_ID, is_extended_id=True, dlc=8))
        time.sleep(0.5)
        bus.shutdown()
        
        # Plot
        if len(history["time"]) > 0:
            print("Plotting...")
            # Downsample plot data so it doesn't freeze matplotlib (1000Hz generates a LOT of points)
            step = 5 
            plt.figure(figsize=(10, 6))
            plt.plot(history["time"][::step], history["target"][::step], label='Target', linestyle='--', color='orange')
            plt.plot(history["time"][::step], history["actual"][::step], label='Actual', color='blue', alpha=0.6)
            plt.title(f"1000 Hz Control Response - {params['name']}")
            plt.legend()
            plt.grid(True)
            plt.show()