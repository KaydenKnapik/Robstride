# graphical_tuner.py
# A graphical step-response tool for tuning motor PD gains objectively.

import robstride.client
import can
import time
import matplotlib.pyplot as plt

# =========================================================================
#                          TUNING CONFIGURATION
# This is the ONLY section you need to edit between runs.
# =========================================================================

# 1. --- Identify the motor you want to tune ---
CAN_PORT = 'can0'
MOTOR_ID = 20

# 2. --- Set the PD gains you want to test ---
TUNE_LOC_KP = 10   # The "Stiffness"
TUNE_SPD_KP = 0.2   # The "Damping"
TUNE_SPD_FILT_GAIN = 1
SPEED = 20

# 3. --- Define the test motion (a "step" from one angle to another) ---
MOVE_FROM_DEG = 0.0
MOVE_TO_DEG = 30.0

# 4. --- How long to record data after the step command is sent ---
RECORDING_DURATION_SEC = 1.0
# How fast to poll the motor for data (in Hz)
POLLING_FREQUENCY_HZ = 100

# =========================================================================
#                             SCRIPT LOGIC
# =========================================================================

# --- Setup ---
print("--- Graphical PD Gain Tuner ---")
print(f"TARGET: Motor ID {MOTOR_ID} on {CAN_PORT}")
print(f"GAINS:  loc_kp = {TUNE_LOC_KP}, spd_kp = {TUNE_SPD_KP}")

# Data storage lists
timestamps = []
target_positions_deg = []
actual_positions_deg = []
actual_velocities_rps = []

# Convert degrees to radians
FULL_ROTATION_RAD = 2 * 3.14159
start_pos_rad = (MOVE_FROM_DEG * FULL_ROTATION_RAD) / 360.0
end_pos_rad = (MOVE_TO_DEG * FULL_ROTATION_RAD) / 360.0

# Connect to the bus
bus = can.interface.Bus(interface='socketcan', channel=CAN_PORT)
client = robstride.client.Client(bus)

try:
    # --- Configure and Enable Motor ---
    print("Configuring motor...")
    client.write_param(MOTOR_ID, 'run_mode', robstride.client.RunMode.Position)
    client.write_param(MOTOR_ID, 'loc_kp', TUNE_LOC_KP)
    client.write_param(MOTOR_ID, 'spd_kp', TUNE_SPD_KP)
    client.write_param(MOTOR_ID, 'spd_filt_gain', TUNE_SPD_FILT_GAIN)
    client.write_param(MOTOR_ID, 'limit_spd', SPEED)

    client.enable(MOTOR_ID)

# =========================================================
    # NEW: Consistent Starting Procedure
    # =========================================================
    print("Performing pre-move to ensure consistent starting state...")
    # 1. Go to a position PAST the start line. This ensures we take up all backlash.
    pre_move_rad = start_pos_rad + 0.2 # Go 0.2 radians before the start
    client.write_param(MOTOR_ID, 'loc_ref', pre_move_rad)
    time.sleep(1.0)
    
    # 2. Slowly approach and settle at the official start line.
    print(f"Moving to start position ({MOVE_FROM_DEG}°)...")
    client.write_param(MOTOR_ID, 'loc_ref', start_pos_rad)
    time.sleep(1.5) # Let it settle completely.
    # =========================================================

    # Now the motor is guaranteed to be in a consistent state.
    # The rest of the test can proceed.
    print(f"Executing step to {MOVE_TO_DEG}° and recording data...")
    start_time = time.time()
    client.write_param(MOTOR_ID, 'loc_ref', end_pos_rad)
    

    # The rest of the loop is the same
    loop_delay = 1.0 / POLLING_FREQUENCY_HZ
    while (time.time() - start_time) < RECORDING_DURATION_SEC:
        loop_start_time = time.time()
        
        # Read data from motor
        pos_rad = client.read_param(MOTOR_ID, 'mechpos')
        vel_rps = client.read_param(MOTOR_ID, 'mechvel')
        
        # Store data
        timestamps.append(time.time() - start_time)
        target_positions_deg.append(MOVE_TO_DEG)
        actual_positions_deg.append((pos_rad * 360.0) / FULL_ROTATION_RAD)
        actual_velocities_rps.append(vel_rps)
        
        # Maintain polling frequency
        time_to_sleep = loop_delay - (time.time() - loop_start_time)
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)

    print("Data collection complete.")

except KeyboardInterrupt:
    print("\nScript stopped by user.")
except Exception as e:
    print(f"\nAN ERROR OCCURRED: {e}")
finally:
    # --- Cleanup ---
    print(f"Disabling motor {MOTOR_ID}...")
    try:
        client.disable(MOTOR_ID)
    except:
        pass # Ignore errors if already disconnected
    bus.shutdown()
    print("Cleanup complete.")


# --- Plotting ---
if timestamps:
    print("Generating plot...")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Position Plot
    ax1.plot(timestamps, target_positions_deg, 'r--', label='Target Position')
    ax1.plot(timestamps, actual_positions_deg, 'b-', label='Actual Position')
    ax1.set_title(f'Step Response (kp={TUNE_LOC_KP}, kd={TUNE_SPD_KP})')
    ax1.set_ylabel('Position (degrees)')
    ax1.legend()
    ax1.grid(True)

    # Velocity Plot
    ax2.plot(timestamps, actual_velocities_rps, 'g-', label='Actual Velocity')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.legend()
    ax2.grid(True)
    
    fig.tight_layout()
    # at the very end of the script
    # New and improved version
    plot_filename = f"MotorID_{MOTOR_ID}_kp{TUNE_LOC_KP}_kd{TUNE_SPD_KP}.png"
    plt.savefig(plot_filename)
    print(f"\nPlot saved successfully as: {plot_filename}")