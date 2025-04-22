import robstride.client
import can
import time

CAN_PORT = 'can0'
MOTOR_ID = 127
FULL_ROTATION = 2 * 3.14159

with can.interface.Bus(interface='socketcan', channel=CAN_PORT) as bus:
    client = robstride.client.Client(bus)
    
    try:
        # Enable motor in current control mode (allows manual rotation)
        print("Enabling motor in current control mode...")
        client.write_param(MOTOR_ID, 'run_mode', robstride.client.RunMode.Current)
        client.enable(MOTOR_ID)
        client.write_param(MOTOR_ID, 'iq_ref', 0.0)  # Set current to 0 to allow free movement
        
        print("\nRotate the motor by hand. Press Ctrl+C to stop.")
        print("Recording min/max angles...")
        
        min_angle = float('inf')
        max_angle = float('-inf')
        
        while True:
            current_angle = client.read_param(MOTOR_ID, 'mechpos')
            degrees = (current_angle * 360.0) / FULL_ROTATION
            
            # Update min/max
            min_angle = min(min_angle, degrees)
            max_angle = max(max_angle, degrees)
            
            print(f"\rCurrent: {degrees:6.1f}°  |  Min: {min_angle:6.1f}°  |  Max: {max_angle:6.1f}°", end='', flush=True)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nFinal range:")
        print(f"Min angle: {min_angle:.1f}°")
        print(f"Max angle: {max_angle:.1f}°")
        print(f"Total range: {max_angle - min_angle:.1f}°")
    
    finally:
        print("\nDisabling motor...")
        client.disable(MOTOR_ID)
