#!/usr/bin/env pybricks-micropython

import sys
import math
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
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

class Robot:
	def __init__(self, wheel_diameter, axle_track, motor_1_sensor_port, motor_2_sensor_port, color_sensor_port) -> None:
		self.motors = self.init_drivebase_sensor(self.init_motor_sensor(motor_1_sensor_port), 
										   		self.init_motor_sensor(motor_2_sensor_port), 
											 	wheel_diameter, 
											  	axle_track)
		self.color_sensor = self.init_color_sensor(color_sensor_port)

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

	def init_motor_sensor(self, port):
		# Initialize the motor sensor.
		return Motor(port)

	def init_drivebase_sensor(self, motor_l, motor_r, wheel_diameter, axle_track):
		return DriveBase(motor_l, motor_r, wheel_diameter=wheel_diameter, axle_track=axle_track)

	def init_color_sensor(self, port):
		# Initialize the color sensor.
		return ColorSensor(port)

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
			# Process speed variations
			self.turn_rate = self.get_pid_turn_rate()
			#self.speed = self.speedRegulation(self.speed, self.turn_rate)
			self.motors.drive(self.speed, self.turn_rate)
			wait(1)

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

	def comeback(self):
		self.motors.drive(0,90)
		wait(2000)
		self.motors.drive(0,0)