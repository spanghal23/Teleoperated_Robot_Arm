"""
Interactive position control for ODrive via CANSimple protocol.
"""
import can
import struct
import threading
import time

node_id = 0  # must match `<odrv>.axis0.config.can.node_id`
bus = can.interface.Bus("can0", interface="socketcan")  # Fixed deprecation warning

# Flush CAN RX buffer
while not (bus.recv(timeout=0) is None): 
    pass

print("Putting axis into closed loop control...")
# Put axis into closed loop control state
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
    data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))

# Wait for closed loop control by scanning heartbeat messages
print("Waiting for closed loop state...")
for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x01):  # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        print(f"State: {state}, Error: {error}")
        if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
            print("Entered closed loop control!")
            break

# Globals for encoder feedback
running = True
current_position = 0.0
current_velocity = 0.0

# Thread to continuously read encoder feedback
def read_encoder_feedback():
    global current_position, current_velocity, running
    while running:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == (node_id << 5 | 0x09):  # 0x09: Get_Encoder_Estimates
            current_position, current_velocity = struct.unpack('<ff', bytes(msg.data))

# Start feedback thread
feedback_thread = threading.Thread(target=read_encoder_feedback, daemon=True)
feedback_thread.start()

print("\n" + "="*60)
print("Interactive Position Control")
print("="*60)
print("Commands:")
print("  <number> - Move to position in turns (e.g., 2.5)")
print("  's' - Show current position and velocity")
print("  'q' or 'exit' - Quit")
print("="*60 + "\n")

# Main loop for position input
try:
    while True:
        user_input = input("Enter position [turns]: ").strip().lower()
        
        if user_input in ['q', 'quit', 'exit']:
            print("Exiting...")
            break
        
        if user_input == 's':
            print(f"Current position: {current_position:.3f} [turns], velocity: {current_velocity:.3f} [turns/s]")
            continue
        
        try:
            # Convert input to float
            target_position = float(user_input)
            
            # Send position command
            # Format: position (float32), vel_ff (int16), torque_ff (int16)
            bus.send(can.Message(
                arbitration_id=(node_id << 5 | 0x0c),  # 0x0c: Set_Input_Pos
                data=struct.pack('<fhh', target_position, 0, 0),  # pos, vel_ff, torque_ff
                is_extended_id=False
            ))
            
            print(f"✓ Moving to position: {target_position:.3f} turns")
            time.sleep(0.2)
            print(f"  Current: {current_position:.3f} [turns], velocity: {current_velocity:.3f} [turns/s]\n")
            
        except ValueError:
            print("Invalid input! Please enter a number, 's' for status, or 'q' to quit.\n")
        except Exception as e:
            print(f"Error sending command: {e}\n")

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    # Clean up
    running = False
    time.sleep(0.2)  # Give thread time to finish
    
    try:
        print("Stopping motor (going to IDLE state)...")
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
            data=struct.pack('<I', 1),  # 1: AxisState.IDLE
            is_extended_id=False
        ))
        time.sleep(0.1)
    except Exception as e:
        print(f"Error setting IDLE state: {e}")
    
    bus.shutdown()
    print("Done!")