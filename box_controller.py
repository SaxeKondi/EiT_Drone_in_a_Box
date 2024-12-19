import time
# import keyboard
from dynamixel_sdk import *  # Assuming you're using the Dynamixel SDK

# Constants
MOTOR_1_ID = 1
MOTOR_2_ID = 2
BAUDRATE = 57600
DEVICE_NAME = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2.0
LOAD_THRESHOLD = 500  # Load threshold (0-1000 scale representing 0-100% of motors max torque)

# Gear and gearrack properties
GEAR_DIAMETER = 35
STEPS_PER_REVOLUTION = 4096
GEAR_CIRCUMFERENCE = 3.14159 * GEAR_DIAMETER
STEPS_PER_MM = STEPS_PER_REVOLUTION / GEAR_CIRCUMFERENCE

# Target positions in millimeters
OPEN_POSITION_MM_1 = 630
OPEN_POSITION_MM_2 = 616
CLOSED_POSITION_MM_1 = 0
CLOSED_POSITION_MM_2 = 0

# Convert target positions to steps
OPEN_POSITION_1 = int(OPEN_POSITION_MM_1 * STEPS_PER_MM)
OPEN_POSITION_2 = int(OPEN_POSITION_MM_2 * STEPS_PER_MM)
CLOSED_POSITION_1 = int(CLOSED_POSITION_MM_1 * STEPS_PER_MM)
CLOSED_POSITION_2 = int(CLOSED_POSITION_MM_2 * STEPS_PER_MM)

# Other motor settings
MAX_SPEED = 256

# Create an instance of the port handler
port_handler = PortHandler(DEVICE_NAME)
packet_handler = PacketHandler(PROTOCOL_VERSION)

# Open the port
port_handler.openPort()
port_handler.setBaudRate(BAUDRATE)

class LidController:  

    def __init__(self):
        self.motor_1_start_pos = None
        self.motor_2_start_pos = None
        self.set_velocity_mode(MOTOR_1_ID)
        self.set_velocity_mode(MOTOR_2_ID)
        self.initialize_start_positions()
        self.lid_open = False
        self.current_state = "Closed"

    def __del__(self):
        packet_handler.write4ByteTxRx(port_handler, MOTOR_1_ID, 104, 0)
        packet_handler.write4ByteTxRx(port_handler, MOTOR_2_ID, 104, 0)
        self.disable_torque(MOTOR_1_ID)
        self.disable_torque(MOTOR_2_ID)
        port_handler.closePort()

    def set_velocity_mode(self, motor_id):
        result, error = packet_handler.write1ByteTxRx(port_handler, motor_id, 11, 1)

    def enable_torque(self, motor_id):
        packet_handler.write1ByteTxRx(port_handler, motor_id, 64, 1)

    def disable_torque(self, motor_id):
        packet_handler.write1ByteTxRx(port_handler, motor_id, 64, 0)

    def check_load(self, motor_id):
        load = packet_handler.read2ByteTxRx(port_handler, motor_id, 126)[0]
        return load if load <= 32767 else load - 65536

    def get_current_position(self, motor_id, motor_start_pos):
        pos = packet_handler.read4ByteTxRx(port_handler, motor_id, 132)[0]
        relative_pos = pos - motor_start_pos
        return relative_pos

    def calculate_velocity(self, distance_remaining, max_distance, ramp_distance_mm=50, load=None):
        ramp_distance_steps = ramp_distance_mm * STEPS_PER_MM

        if distance_remaining < ramp_distance_steps:
            # Ramp down as we approach the target
            velocity = int(MAX_SPEED * (distance_remaining / ramp_distance_steps))
        elif distance_remaining > (max_distance - ramp_distance_steps):
            # Ramp up at the start of movement
            velocity = int(MAX_SPEED * ((max_distance - distance_remaining) / ramp_distance_steps))
        else:
            # Full speed in the middle portion
            velocity = MAX_SPEED

        # Slow down if load exceeds the threshold (optional)
        if load and load > (0.75 * LOAD_THRESHOLD):
            velocity = int(velocity * 0.5)

        # Ensure velocity is within bounds
        return max(10, min(abs(velocity), MAX_SPEED))

    def move_lid_tandem(self, desired_state):
        # Enable torque for both motors
        self.enable_torque(MOTOR_1_ID)
        self.enable_torque(MOTOR_2_ID)

        # Determine target positions and velocity direction
        if desired_state == "open":
            target_position1 = OPEN_POSITION_1
            target_position2 = OPEN_POSITION_2
            velocity_sign = 1
            self.current_state = "Opening"
            print("Moving to OPEN position...")
        elif desired_state == "closed":
            target_position1 = CLOSED_POSITION_1
            target_position2 = CLOSED_POSITION_2
            velocity_sign = -1
            self.current_state = "Closing"
            print("Moving to CLOSED position...")
        else:
            print("Invalid state")
            return

        # Maximum distance to cover for ramp calculations
        max_distance_1 = abs(target_position1 - CLOSED_POSITION_1 if desired_state == "open" else OPEN_POSITION_1)
        max_distance_2 = abs(target_position2 - CLOSED_POSITION_2 if desired_state == "open" else OPEN_POSITION_2)
	
        motor_1_done = False
        motor_2_done = False

        while not (motor_1_done and motor_2_done):
            # Motor 1
            if not motor_1_done:
                load_1 = self.check_load(MOTOR_1_ID)
                pos_1 = self.get_current_position(MOTOR_1_ID, self.motor_1_start_pos)
                distance_1_remaining = abs(target_position1 - pos_1)

                # Check load for safety
                if load_1 > LOAD_THRESHOLD:
                    print(f"Motor 1 load threshold exceeded ({load_1}), stopping motor.")
                    packet_handler.write4ByteTxRx(port_handler, MOTOR_1_ID, 104, 0)
                    self.disable_torque(MOTOR_1_ID)
                    motor_1_done = True
                else:
                    velocity_1 = self.calculate_velocity(distance_1_remaining, max_distance_1, ramp_distance_mm=100, load=load_1) * velocity_sign
                    packet_handler.write4ByteTxRx(port_handler, MOTOR_1_ID, 104, velocity_1)

                if (desired_state == "open" and pos_1 >= OPEN_POSITION_1) or \
                   (desired_state == "closed" and pos_1 <= CLOSED_POSITION_1):
                    print("Motor 1 reached target position.")
                    packet_handler.write4ByteTxRx(port_handler, MOTOR_1_ID, 104, 0)
                    self.disable_torque(MOTOR_1_ID)
                    motor_1_done = True

            # Motor 2
            if not motor_2_done:
                load_2 = self.check_load(MOTOR_2_ID)
                pos_2 = self.get_current_position(MOTOR_2_ID, self.motor_2_start_pos)
                distance_2_remaining = abs(target_position2 - pos_2)

                # Check load for safety
                if load_2 > LOAD_THRESHOLD:
                    print(f"Motor 2 load threshold exceeded ({load_2}), stopping motor.")
                    packet_handler.write4ByteTxRx(port_handler, MOTOR_2_ID, 104, 0)
                    self.disable_torque(MOTOR_2_ID)
                    motor_2_done = True
                else:
                    velocity_2 = self.calculate_velocity(distance_2_remaining, max_distance_2, ramp_distance_mm=100, load=load_2) * velocity_sign
                    packet_handler.write4ByteTxRx(port_handler, MOTOR_2_ID, 104, velocity_2)

                if (desired_state == "open" and pos_2 >= OPEN_POSITION_2) or \
                   (desired_state == "closed" and pos_2 <= CLOSED_POSITION_2):
                    print("Motor 2 reached target position.")
                    packet_handler.write4ByteTxRx(port_handler, MOTOR_2_ID, 104, 0)
                    self.disable_torque(MOTOR_2_ID)
                    motor_2_done = True

            time.sleep(0.01)

        print(f"Both motors have completed the movement to {desired_state}.")

        if desired_state == "open":
            self.lid_open = True
            self.current_state = "Open"
        else:
            self.lid_open = False
            self.current_state = "Closed"

    def initialize_start_positions(self):
        self.motor_1_start_pos = packet_handler.read4ByteTxRx(port_handler, MOTOR_1_ID, 132)[0]
        self.motor_2_start_pos = packet_handler.read4ByteTxRx(port_handler, MOTOR_2_ID, 132)[0]
        print(f"Motor 1 starting position: {self.motor_1_start_pos}")
        print(f"Motor 2 starting position: {self.motor_2_start_pos}")

#     def control_lid(self):
#         while True:
#             if keyboard.is_pressed('o'):
#                 print("Opening the box...")
#                 self.move_lid_tandem("open")

#             elif keyboard.is_pressed('c'):
#                 print("Closing the box...")
#                 self.move_lid_tandem("closed")

#             elif keyboard.is_pressed('space'):
#                 print("Stopping the motors and exiting...")
#                 packet_handler.write4ByteTxRx(port_handler, MOTOR_1_ID, 104, 0)
#                 packet_handler.write4ByteTxRx(port_handler, MOTOR_2_ID, 104, 0)
#                 self.disable_torque(MOTOR_1_ID)
#                 self.disable_torque(MOTOR_2_ID)
#                 break

#             time.sleep(0.1)

# controller = LidController()

# controller.control_lid()


