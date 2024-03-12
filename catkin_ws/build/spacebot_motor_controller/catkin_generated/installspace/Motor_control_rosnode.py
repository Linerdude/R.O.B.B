#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import pigpio
import time
from math import radians, degrees
from std_msgs.msg import String
from queue import Queue
from threading import Thread
from sensor_msgs.msg import JointState

class MotorControllerNode:
    def __init__(self):
        rospy.init_node('motor_controller_node', anonymous=True)
        
        # Create a command queue
        self.command_queue = Queue()
        
        self.joint_positions = {}  # Dictionary to store joint positions
        # Define motor pins and IDs
        self.motors_config = {
            1: {"type": "BLDC", "name": "Motor 1", "pins": (17, 27, 22)},
            2: {"type": "BLDC", "name": "Motor 2", "pins": (10, 9, 11)},
            3: {"type": "BLDC", "name": "Motor 3", "pins": (0, 5, 6)},
            4: {"type": "Servo", "name": "Servo 1", "pin": 13},
            5: {"type": "Servo", "name": "Servo 2", "pin": 19},
            6: {"type": "Servo", "name": "Servo 3", "pin": 26},
            7: {"type": "Servo", "name": "Servo 4", "pin": 20},
            8: {"type": "Servo", "name": "Servo 5", "pin": 21}
        }

        # Initialize motor controllers
        self.motors = {}
        for motor_id, config in self.motors_config.items():
            if config["type"] == "BLDC":
                self.motors[motor_id] = BLDCMotorController(config["name"], *config["pins"], self.command_queue)
            elif config["type"] == "Servo":
                self.motors[motor_id] = ServoMotorController(config["name"], config["pin"], self.command_queue)



        # Subscribe to motor commands
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        # Start the motor control loop
        self.control_thread = Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

    def joint_states_callback(self, msg):
        # Store the joint positions from the received message
        self.joint_positions = dict(zip(msg.name, msg.position))
        for joint, position in self.joint_positions.items():
            joint_number = int(joint.replace('joint_', ''))
            angle = degrees(position)
            self.command_queue.put((joint_number, angle))
            print(f"{joint_number}: {angle}")
            # print(self.joint_positions[0])

    def command_callback(self, msg):
        command = msg.data.split(',')
        if len(command) == 2:
            motor_id = int(command[0])
            angle_command = float(command[1])
            if motor_id in self.motors:
                self.command_queue.put((motor_id, angle_command))
        

    def control_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Run motors
            for motor_id, motor in self.motors.items():
                motor.run()

            # Check for pending angle commands
            while not self.command_queue.empty():
                motor_id, angle_command = self.command_queue.get()
                if motor_id in self.motors:
                    self.motors[motor_id].set_angle_command(angle_command)

            rate.sleep()

class BLDCMotorController:
    PWM_FREQUENCY = 200
    SIGNAL_MIN = 1050
    SIGNAL_MAX = 1950
    DEG_PER_PULSE = 0.136335
    INTEGRAL_TOL = 0.1
    PID_INTERVAL = 0.02

    def __init__(self, motor_name, cha, chb, esc_pin, command_queue):
        self.motor_name = motor_name
        self.esc_pin = esc_pin
        self.encoder_pin1, self.encoder_pin2 = cha, chb
        self.encoder_value, self.encoder_direction = 0, 0
        self.setpoint, self.output = 40.0, 0.0
        self.Kp, self.Ki, self.Kd = 2, 0.8, .25
        self.integral = 0.0
        self.prev_millis = 0
        self.prev_position = 0.0
        self.command_queue = command_queue

        # Initialize GPIO and interrupts
        self.pi = pigpio.pi()
        self.pi.set_mode(self.esc_pin, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.esc_pin, self.PWM_FREQUENCY)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.encoder_pin1, GPIO.IN)
        GPIO.setup(self.encoder_pin2, GPIO.IN)
        
        GPIO.add_event_detect(cha, GPIO.BOTH, callback=self.update_encoder)
        self.angle_thread = Thread(target=self.receive_angle_commands)
        self.angle_thread.daemon = True
        self.angle_thread.start()

    def update_encoder(self, channel):
        channel1, channel2 = GPIO.input(self.encoder_pin1), GPIO.input(self.encoder_pin2)

        self.encoder_direction = 1 if channel1 == channel2 else -1
        self.encoder_value += self.encoder_direction

    def calculate_pid(self):
        current_millis = time.time() * 1000
        if current_millis - self.prev_millis >=20:
            position_value = self.encoder_value * self.DEG_PER_PULSE
            error = self.setpoint - position_value
            error = min(max(error, -40), 40)
            derivative = (position_value - self.prev_position) *5
            # print(derivative)
            # print(position_value)
            if abs(derivative) < 0.1 and self.INTEGRAL_TOL < abs(error) < 20:
                self.integral += error
            elif abs(error) < self.INTEGRAL_TOL:
                self.integral = 0

            self.output = self.Kp * error + self.Ki * self.integral - self.Kd * derivative
            # print(self.Kp*error)
            self.output = min(max(self.output, -90), 90)
            self.run_motor(self.output)
            # print(self.output)

            self.prev_position = position_value
            self.prev_millis = current_millis

    def run_motor(self, output):
        pulse_width = int(self.map_range(output, -100, 100, self.SIGNAL_MIN, self.SIGNAL_MAX))
        self.pi.set_servo_pulsewidth(self.esc_pin, pulse_width)

    def map_range(self, output, in_min, in_max, out_min, out_max):
        return (output - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def set_angle_command(self, angle):
        self.setpoint = angle
        print("Setting", self.motor_name, "to:", angle)

    def receive_angle_commands(self):
        while True:
            try:
                command = input()#"Enter motor_id, angle command (e.g., '1, 180'): ")
                motor_id, angle = map(int, command.split(','))
                self.command_queue.put((motor_id, angle))
            except ValueError:
                print("Invalid input")


    def run(self):
        # while True:
        self.calculate_pid()
        time.sleep(0.005)

class ServoMotorController:
    PWM_FREQUENCY = 200
    SIGNAL_MIN = 500
    SIGNAL_MAX = 2500
    ANGLE_MIN = 0
    ANGLE_MAX = 270


    def __init__(self, motor_name, pwm_pin, command_queue):
        self.motor_name = motor_name
        self.setpoint = 40.0
        self.pwm_pin = pwm_pin
        self.integral = 0.0
        self.prev_millis = 0
        self.prev_position = 0.0
        self.command_queue = command_queue

        # Initialize GPIO and interrupts
        self.pi = pigpio.pi()
        self.pi.set_mode(self.pwm_pin, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.pwm_pin, self.PWM_FREQUENCY)
        GPIO.setmode(GPIO.BCM)
        
        self.angle_thread = Thread(target=self.receive_angle_commands)
        self.angle_thread.daemon = True
        self.angle_thread.start()


    def run_motor(self):
        pulse_width = int(self.map_range(self.setpoint, self.ANGLE_MIN, self.ANGLE_MAX, self.SIGNAL_MIN, self.SIGNAL_MAX))
        self.pi.set_servo_pulsewidth(self.pwm_pin, pulse_width)

    def map_range(self, output, in_min, in_max, out_min, out_max):
        return (output - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def set_angle_command(self, angle):
        self.setpoint = angle
        print("Setting", self.motor_name, "to:", angle)


    def receive_angle_commands(self):
        while True:
            try:
                command = input()#"Enter motor_id, angle command (e.g., '1, 180'): ")
                motor_id, angle = map(int, command.split(','))
                self.command_queue.put((motor_id, angle))
            except ValueError:
                print("Invalid input")


    def run(self):
        # while True:
        self.run_motor()
        time.sleep(0.005)


if __name__ == '__main__':
    try:
        MotorControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
