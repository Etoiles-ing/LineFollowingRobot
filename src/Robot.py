#!/usr/bin/env pybricks-micropython

import sys
import math
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import DataLog, wait, StopWatch
from pybricks.robotics import DriveBase
import time

# Set time definition in ms
TAU = 0.100

# Choose correction type - "P", "PI", "PID", "NONE"
CORRECTION = "PID"

# Proportional gain KP
KP = 1
 
# Integral gain ki	
KI = 0.3
 
# Derivative gain
KD = 0.4

# Calculate the light threshold //Faire un auto seuillage en mode il tourne sur droit, il mesure noir, il tourne a gauche, il mesure blanc
BLACK = 8
WHITE = 62
THRESHOLD = (WHITE - BLACK) / 2

# Define command boundaries
ERROR_LIMIT = 1400

# Define Kalman errors
MEASUREMENT_ERROR = 0.5
PREDICTION_ERROR = 0.5

class Robot:
    def __init__(self, wheel_diameter, axle_track, motor_1_sensor_port, motor_2_sensor_port, color_sensor_port, gyro_sensor_port) -> None:
        self.motors = self.init_drivebase_sensor(self.init_motor_sensor(motor_1_sensor_port), 
                                                   self.init_motor_sensor(motor_2_sensor_port), 
                                                 wheel_diameter, 
                                                  axle_track)
        self.color_sensor = self.init_color_sensor(color_sensor_port)
        self.gyro_sensor = self.init_gyro_sensor(gyro_sensor_port)
        self.gyro_sensor.reset_angle(0)

        self.kp = KP
        self.ki = KI
        self.kd = KD

        self.make_threshold()

        # Default speed value
        self.speed = 150
    
        # Set the gain of the proportional line controller. This means that for every
        # percentage point of light deviating from the threshold, we set the turn
        # rate of the drivebase to 1.2 degrees per second.

        self.cumulativeError = 0.0
        self.last_error = 0
        
        self.pose = (0.0, 0.0, 0.0)
        
        #self.stop_watch = self.init_stop_watch()
        self.data_log = DataLog('x', 'y', 'theta')
        self.data_log.log(*self.pose)
        
    def init_stop_watch(self):
        # Initialize the stop watch.
        return StopWatch()
    
    def init_datalog(self):
        # Initialize the data logger.
        return DataLog('x', 'y', 'theta')

    def init_motor_sensor(self, port):
        # Initialize the motor sensor.
        return Motor(port)

    def init_drivebase_sensor(self, motor_l, motor_r, wheel_diameter, axle_track):
        return DriveBase(motor_l, motor_r, wheel_diameter=wheel_diameter, axle_track=axle_track)

    def init_color_sensor(self, port):
        # Initialize the color sensor.
        return ColorSensor(port)

    def init_gyro_sensor(self, port):
        # Initialize the color sensor.
        return GyroSensor(port)

    def init_ultrasonic_sensor(self, port):
        return UltrasonicSensor(port)

    def make_threshold(self):

        self.motors.drive(0,90)
        wait(500)
        self.motors.drive(0,0)
        wait(1000)
        black = self.color_sensor.reflection()

        self.motors.drive(0,-90)
        wait(1000)
        self.motors.drive(0,0)
        wait(1000)
        white = self.color_sensor.reflection()
  
        self.motors.drive(0,90)
        wait(500)

        self.motors.drive(0,0)
        wait(1000)

        self.threshold = (black + white)/2
        
    def run(self):
        while True:
            self.previous_pose = self.pose
            # Process speed variations
            self.turn_rate = self.get_pid_turn_rate()
            #self.speed = self.speedRegulation(self.speed, self.turn_rate)
            self.motors.drive(self.speed, self.turn_rate)
            
            
            self.pose = self.get_pose_by_state(self.pose)
            self.data_log.log(*self.pose)
            # print(self.pose, self.stopwatch.time())
            #print(round(self.pose[0], 2), round(self.pose[1], 2), round(self.pose[2], 2), self.stopwatch.time())
            
            wait(TAU)

    def get_pid_turn_rate(self):
        self.error = self.color_sensor.reflection() - self.threshold
        if (CORRECTION == "P"):
            self.turn_rate = self.p_correction(self.error)
        elif (CORRECTION == "PI"):		
            self.turn_rate = self.pi_correction(self.error)
        elif (CORRECTION == "PID"):
            self.turn_rate = self.pid_correction_v2(self.error)
        else:
            self.turn_rate = self.error
        return self.turn_rate
        
    def p_correction(self, error):
        turn_rate = self.kp * error
        return turn_rate

    def pi_correction(self, error):
        
        # Sum tracked errors to process integration
        self.cumulativeError += error
        if abs(self.cumulativeError) > ERROR_LIMIT:
            self.cumulativeError = ERROR_LIMIT * (error/abs(error))
                            
        turn_rate = self.p_correction(error) + self.ki * TAU * self.cumulativeError

        return turn_rate

    def pid_correction_v2(self, error):
        # Clear the error memory if the detected color cross the threshold

        derivate_error = error - self.last_error
        self.last_error = error
        
        turn_rate = self.pi_correction(error) + (self.kd/TAU) * derivate_error #KP/TAU (erreur -erreur-1 dans errorTraking)
        return turn_rate

    def speedRegulation(self, current_speed, turn_rate):
        
        # speed = current_speed
        if abs(turn_rate) < 30:
            speed = current_speed * 1.01
        elif abs(turn_rate) > 30:
            speed = current_speed / 1.01
        else: speed = current_speed
    
        if speed > 800: speed = 800
        if speed < 40: speed = 40
        return speed

    def record_values(self, list_values):
        #self.data_log.log(BREAK_DISTANCE, ACCELERATION_DISTANCE, BREAK_FACTOR, ACCELERATION_FACTOR)
        self.data_log.log(*list_values)
        #print(self.data_log)
        # if len(self.speed_tab)% RECORD_VALUE == 0:
        # 	print(self.speed_tab)

    def comeback(self):
        self.motors.drive(0,90)
        wait(2000)
        self.motors.drive(0,0)
        
    def unpark_you(self):
        self.motors.drive(0,90)
        wait(1000)
        self.motors.drive(100,0)
        wait(2000)
        self.motors.drive(0,-90)
        wait(1000)
        self.motors.drive(10,0)
        wait(100)
        self.motors.drive(0,0)
    
    def park_you(self):
        self.motors.drive(0,-90)
        wait(1000)
        self.motors.drive(100,0)
        wait(2000)
        self.motors.drive(0,90)
        wait(1000)
        self.motors.drive(10,0)
        wait(100)
        self.motors.drive(0,0)
        
    def get_pose_by_speed_rate(self, τ: int, previous_pose: tuple(float, float, float), speed: float, angular_speed: float) -> tuple(float, float, float):
        # Recover previous pose
        previous_x, previous_y, previous_θ = previous_pose
        
        # Process new pose base on previous one and current speed and angular speed value
        θ = previous_θ + τ * angular_speed
        x = previous_x + speed * τ * math.cos((previous_θ+θ)*(2*math.pi/360))
        y = previous_y + speed * τ * math.sin((previous_θ+θ)*(2*math.pi/360))
        return (x, y, θ)
    
    def get_pose_by_state(self, previous_pose: tuple(float, float, float)) -> tuple(float, float, float):
        # Recover previous pose
        previous_x, previous_y, previous_θ = previous_pose
        distance, speed, angle, rotational_speed = self.motors.state()

        θ = math.radians(angle)
        print(θ, angle, previous_θ)
        
        delta_x = distance * math.cos(θ)
        delta_y = distance * math.sin(θ)
        
        x = previous_x + delta_x
        y = previous_y + delta_y
        # Process new pose base on previous one and current speed and angular speed value
        
        self.motors.reset()

        return (x, y, θ)
    
    def get_pose_by_gyro(self, previous_pose: tuple(float, float, float)) -> tuple(float, float, float):
        previous_x, previous_y, previous_θ = previous_pose
        
        vitesse_angulaire = self.gyro_sensor.speed()  
        drive_speed = self.speed  # Remplacez par la valeur appropriée
        turn_rate = ...  # Remplacez par la valeur appropriée  
        
        angle_cumulé = angle_cumulé + vitesse_angulaire # * temps
        
        delta_x = drive_speed * math.cos(math.radians(angle_cumulé))
        delta_y = drive_speed * math.sin(math.radians(angle_cumulé))
        
        x = previous_x + delta_x
        y = previous_y + delta_y
        
        return (x, y, angle_cumulé)

    def estimate_next_pose(self, τ: int, previous_estimated_θ: float, current_measured_θ: float, angular_speed :float):
        
        a_priori_θ = previous_estimated_θ + τ * angular_speed
        
        # Process Kalman gain
        K = PREDICTION_ERROR / PREDICTION_ERROR + MEASUREMENT_ERROR
        
        # State of the system
        y = current_measured_θ - a_priori_θ
        
        # Next predicted angle based on Kalman gain and current state of the system
        a_posteriori_θ = a_priori_θ + K * y
    
        # Update prediction error
        PREDICTION_ERROR = (1-K) * PREDICTION_ERROR
        
        return a_posteriori_θ