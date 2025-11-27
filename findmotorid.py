import robstride.client
import can
import time

# --- CONFIG ---
INTERFACE = 'can0'

def flush_bus(bus):
    """Clear the buffer to ensure we aren't reading old messages"""
    while True:
        msg = bus.recv(timeout=0)
        if msg is None:
            break

print(f"Initializing {INTERFACE}...")
# We set a short timeout (0.05s) so the scan is fast
bus = can.interface.Bus(interface='socketcan', channel=INTERFACE, timeout=0.05)
client = robstride.client.Client(bus)

print("-" * 40)
print("STARTING PASSIVE SCAN (ID 1 - 127)")
print("This will NOT move or enable the motor.")
print("-" * 40)

found_ids = []

for motor_id in range(1, 128):
    # Print progress on the same line
    print(f"Scanning ID: {motor_id}", end='\r')
    
    try:
        flush_bus(bus)
        
        # We try to read 'vbus' (Voltage Bus). 
        # This is a READ-ONLY command. It asks the motor "What is your voltage?"
        # It cannot cause movement.
        val = client.read_param(motor_id, 'vbus')
        
        if val is not None:
            # If we get a valid number back, the motor exists!
            print(f"Scanning ID: {motor_id} -> [FOUND!] (Voltage: {val:.2f}V)")
            found_ids.append(motor_id)
            
    except Exception:
        # If the read times out (no motor there), we just move to the next one
        pass

print("\n" + "-" * 40)
if found_ids:
    print(f"SUCCESS! Your Motor ID is: {found_ids}")
else:
    print("No motors found. Check power and CAN cables.")

# Clean up
bus.shutdown()