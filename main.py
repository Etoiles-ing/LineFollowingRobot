#!/usr/bin/env pybricks-micropython

from pybricks.robotics import DriveBase
from pybricks.ev3devices import (UltrasonicSensor)
from pybricks.messaging import BluetoothMailboxClient, TextMailbox
from pybricks.parameters import Port
from pybricks.tools import DataLog
from Robot import Robot

from RobotFollower import RobotFollower
from RobotLeader import RobotLeader


def main():
    wheel_diameter=56 
    axle_track=114
    my_robot = Robot(wheel_diameter, axle_track, Port.B, Port.C, Port.S4, Port.S2)
    
    #TODO : call get_pose() and initialize the pose
    #my_robot.unpark_you()
    my_robot.make_threshold()
    my_robot.run()
    #my_robot.park_you()
    
    # my_robot = RobotLeader(wheel_diameter, axle_track, Port.A, Port.B, Port.S1)
    # my_robot.lead_driving(True)
    
    # Initialize robots main components
    #my_robot = RobotFollower(wheel_diameter, axle_track, Port.A, Port.B, Port.S1, Port.S2)
    
    # my_robot.follow_driving(0)
    # my_robot.follow_driving(1)
    #my_robot.follow_driving(2)
    # Start following the line endlessly.
    
    #my_robot = RobotFollower(wheel_diameter, axle_track, Port.B, Port.C, Port.S4, Port.S3)
    #my_robot.follow_driving(3)
    #my_robot.comeback()
    

if __name__=='__main__':
    main()