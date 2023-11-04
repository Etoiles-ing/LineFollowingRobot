#!/usr/bin/env pybricks-micropython

from pybricks.robotics import DriveBase
from pybricks.ev3devices import (UltrasonicSensor)
from pybricks.messaging import BluetoothMailboxClient, TextMailbox
from pybricks.parameters import Port
from pybricks.tools import DataLog, wait
from Robot import Robot



# Robot driving straight and angular speed
MAX_SPEED = 300
RECORD_VALUE = 2000
DEFAULT_TURN_RATE = 0

# Parameters
BREAK_DISTANCE = 100 # Distance threshold to start breaking
ACCELERATION_DISTANCE = 100 # Distance threshold to start accelerating
BREAK_FACTOR = 1.2 # Break factor
ACCELERATION_FACTOR = 1.2 # Acceleration factor


class RobotFollower(Robot):
	def __init__(self, wheel_diameter, axle_track, motor_1_sensor_port, motor_2_sensor_port, color_sensor_port, ultrasonic_sensor_port) -> None:
		super().__init__(wheel_diameter, axle_track, motor_1_sensor_port, motor_2_sensor_port, color_sensor_port)
		self.ultrasonic_sensor = self.init_ultrasonic_sensor(ultrasonic_sensor_port)
	 
	def follow_driving(self, policy: int):
		previous_speed = 0
		while(1):
			self.distance = self.ultrasonic_sensor.distance()
			previous_speed = self.speed
			self.get_speed(policy,previous_speed)
			self.motors.drive(self.speed, 0)
			self.record_values(self.distance, self.speed)
			wait(1)
   
	def get_speed(self, policy, previous_speed):
		if policy == 0: self.speed = self.all_or_none(self.distance)
		elif policy == 1: self.speed = self.one_point(self.distance)
		elif policy == 2: self.speed = self.two_point(self.distance, previous_speed)
		elif policy == 3: self.speed = self.wireless(self.distance) #ça marche pas
		else:
			print("Wrong policy, please enter a number between 0 and 3")
			return ValueError

			
 
	# Robot stop driving if it is closer than 15cm of the lead robot
	def all_or_none(self, distance: int):
		if distance < 150: speed = 0.0
		else: speed = 0.5 * MAX_SPEED
		return speed

	# Robot's speed is proportional to a defined factor and break distance, unless it overcomes MAX_SPEED 
	def one_point(self, distance: int):
		speed = max(min(MAX_SPEED, BREAK_FACTOR * (distance - BREAK_DISTANCE)), 0)
		return speed

	# Robot's acceleration and breaking are proportional to defined factors and distances, unless they overcomes MAX_SPEED 
	def two_point(self, distance: int, previous_speed: int):
		break_speed = min(max(BREAK_FACTOR * (distance - BREAK_DISTANCE), 0), MAX_SPEED)
		acceleration_speed = min(max(ACCELERATION_FACTOR * (distance - ACCELERATION_DISTANCE), 0, previous_speed), MAX_SPEED)
		speed = min(break_speed, acceleration_speed)
		return speed

	def wireless(self, distance):
		# This is the name of the remote EV3 or PC we are connecting to.
		serveur = "ev3dev"

		# Instanciate this robot as a Client
		client = BluetoothMailboxClient()
		mbox = TextMailbox("SpaceTab", client)

		print("Establishing connection...")
		client.connect(serveur) #SERVER
		print("Connected !")

		# Speed equals to LEAD robot's speed
		# If there is a communication message, it will use all_or_none policy
		if mbox.read() == None:
			speed = self.all_or_none(distance)
		else: speed = int(mbox.read())
		
		return speed