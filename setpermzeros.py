# correct_permanent_zeroing.py

import robstride.client
import can
import time
import atexit

# --- CONFIGURATION ---
# Make sure this matches your robot's CAN setup
CAN_CONFIGS = [
    ('can1', [1, 2, 3, 4, 5]),
    ('can0', [19, 18, 16, 17, 20])
]

# --- Helper Functions (Good Practice) ---
def flush_bus(bus):
    """Clears any pending messages from the CAN bus receive buffer."""
    while bus.recv(timeout=0) is not None:
        pass

# --- Main Zeroing Logic ---
def main():
    print("="*60)
    print("--- MOTOR PERMANENT ZERO OFFSET CALIBRATION UTILITY ---")
    print("\n! ! ! WARNING ! ! !")
    print("This utility will permanently write the current physical joint")
    print("positions as the new 'zero' to the motor controllers' memory.")
    print("This is a one-time setup procedure.")
    print("="*60)

    print("\nACTION REQUIRED:")
    print("STEP 1: Manually move every robot joint to its desired 'home' or 'zero' pose.")
    print("        (e.g., standing perfectly straight, arms at sides, etc.)")

    if input("\nIs the robot in the desired zero pose? Type 'yes' to continue: ").lower() != 'yes':
        print("Aborted by user.")
        return

    # --- Setup buses and clients ---
    buses = [can.interface.Bus(interface='socketcan', channel=port) for port, _ in CAN_CONFIGS]
    clients = [robstride.client.Client(bus) for bus in buses]
    motor_ids_list = [ids for _, ids in CAN_CONFIGS]
    for bus in buses:
        atexit.register(bus.shutdown)

    try:
        # STEP 1: Tell each motor to measure its current angle and treat it as the new zero offset.
        # This is a temporary change for the current session.
        print("\nSending 'zero_pos' command to all motors...")
        for client, bus, motor_ids in zip(clients, buses, motor_ids_list):
            for motor_id in motor_ids:
                flush_bus(bus) # Clear buffer before sending
                print(f"  - Zeroing motor {motor_id}...")
                client.zero_pos(motor_id)
                time.sleep(0.05) # Brief pause between commands
        print("... 'zero_pos' commands sent successfully.")
        time.sleep(0.5)

        # STEP 2: Make the change permanent.
        # This writes the temporary offset from step 1 into the motor's internal EEPROM.
        print("\n--- CRITICAL STEP ---")
        print("Saving configuration to each motor's permanent memory...")
        for client, bus, motor_ids in zip(clients, buses, motor_ids_list):
            for motor_id in motor_ids:
                flush_bus(bus)
                print(f"  - Saving configuration for motor {motor_id}...")
                client.save_configuration(motor_id)
                time.sleep(0.05)

        print("\n\nSUCCESS! Motor zero positions have been permanently saved.")
        print("You MUST now power cycle the robot for the changes to take effect.")

    except Exception as e:
        print(f"\nAN ERROR OCCURRED: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nDisabling motors and shutting down CAN buses.")
        # The motors are already disabled by the process, but this is a safe fallback.
        for client, bus, motor_ids in zip(clients, buses, motor_ids_list):
            for motor_id in motor_ids:
                try:
                    client.disable(motor_id)
                except Exception:
                    pass

if __name__ == '__main__':
    main()