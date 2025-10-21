import robstride.client
import can
import time
import atexit

FULL_ROTATION = 2 * 3.14159

CAN_CONFIGS = [
    ('can1', [1,2,3,4,5]),
    ('can0', [19, 18, 17, 16, 20])
]

def degrees(rad):
    return (rad * 360.0) / FULL_ROTATION

# --- Helper functions ---
def flush_bus(bus):
    while True:
        msg = bus.recv(timeout=0.01)
        if msg is None:
            break

def safe_write_param(client, bus, motor_id, param_name, value, retries=3):
    for attempt in range(retries):
        try:
            flush_bus(bus)
            client.write_param(motor_id, param_name, value)
            return
        except Exception as e:
            print(f"[ERROR] Write attempt {attempt+1} failed for motor {motor_id}, param '{param_name}': {e}")
    print(f"[FAILED] Could not write '{param_name}' to motor {motor_id} after {retries} attempts.")

def safe_read_param(client, bus, motor_id, param_name, retries=3):
    for attempt in range(retries):
        try:
            flush_bus(bus)
            return client.read_param(motor_id, param_name)
        except Exception as e:
            print(f"[ERROR] Read attempt {attempt+1} failed for motor {motor_id}, param '{param_name}': {e}")
    print(f"[FAILED] Could not read '{param_name}' from motor {motor_id} after {retries} attempts.")
    return None

def safe_disable(client, bus, motor_id, retries=3):
    for attempt in range(retries):
        try:
            flush_bus(bus)
            client.disable(motor_id)
            return
        except Exception as e:
            print(f"[ERROR] Disable attempt {attempt+1} failed for motor {motor_id}: {e}")
    print(f"[FAILED] Could not disable motor {motor_id} after {retries} attempts.")

# --- Setup buses and clients ---
buses = [can.interface.Bus(interface='socketcan', channel=port) for port, _ in CAN_CONFIGS]
clients = [robstride.client.Client(bus) for bus in buses]
motor_ids_list = [ids for _, ids in CAN_CONFIGS]

# Ensure proper shutdown
for bus in buses:
    atexit.register(bus.shutdown)

# Init min/max tracking
min_angles = {motor_id: float('inf') for ids in motor_ids_list for motor_id in ids}
max_angles = {motor_id: float('-inf') for ids in motor_ids_list for motor_id in ids}

try:
    print("Enabling motors in OPERATIONAL mode context...")
    for client, bus, motor_ids in zip(clients, buses, motor_ids_list):
        for motor_id in motor_ids:
            # THIS IS THE ONLY CHANGE NEEDED
            # We set the mode to Operation (0) to read the correct encoder value.
            safe_write_param(client, bus, motor_id, 'run_mode', robstride.client.RunMode.Operation)
            
            flush_bus(bus)
            client.enable(motor_id)
            # In this mode, we don't send an iq_ref. The motor is just enabled.
            # You can gently turn it by hand.

    print("\nRotate the motors by hand. Press Ctrl+C to stop.")
    print("Recording min/max angles from the LOW-LEVEL OPERATIONAL encoder...")

    while True:
        output_lines = []
        for client, bus, motor_ids in zip(clients, buses, motor_ids_list):
            for motor_id in motor_ids:
                # 'mechpos' will now return the value relevant to the current 'run_mode'
                current_angle_raw = safe_read_param(client, bus, motor_id, 'mechpos')
                velocity_rps = safe_read_param(client, bus, motor_id, 'mechvel')

                if current_angle_raw is None or velocity_rps is None:
                    continue

                rad = current_angle_raw
                velocity_rps = velocity_rps

                min_angles[motor_id] = min(min_angles[motor_id], rad)
                max_angles[motor_id] = max(max_angles[motor_id], rad)

                output_lines.append(
                    f"Motor {motor_id:2d} | Pos: {rad:7.3f} rad | Vel: {velocity_rps:7.3f} rad/s | "
                    f"Min: {min_angles[motor_id]:7.3f} rad | Max: {max_angles[motor_id]:7.3f} rad"
                )

        # This clears the screen and reprints the lines for a live-updating display
        if output_lines:
            print("\n".join(output_lines))
            print("\033[F" * len(output_lines), end='', flush=True)
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\nFinal angle ranges (from operational mode encoder):")
    for motor_id in sorted(min_angles.keys()):
        if min_angles[motor_id] != float('inf'):
            range_rad = max_angles[motor_id] - min_angles[motor_id]
            print(f"Motor {motor_id}: Min = {min_angles[motor_id]:.3f} rad, "
                  f"Max = {max_angles[motor_id]:.3f} rad, "
                  f"Range = {range_rad:.3f} rad")

finally:
    print("\nDisabling all motors...")
    for client, bus, motor_ids in zip(clients, buses, motor_ids_list):
        for motor_id in motor_ids:
            safe_disable(client, bus, motor_id)