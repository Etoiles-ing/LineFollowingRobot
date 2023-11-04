#!/usr/bin/env pybricks-micropython
from Robot import Robot
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.messaging import BluetoothMailboxServer, TextMailbox
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port

# Duration of run / wait (in ms)
T = 3000

class RobotLeader(Robot):
    def __init__(self, wheel_diameter, axle_track, motor_1_sensor_port, motor_2_sensor_port, color_sensor_port) -> None:
        super().__init__(wheel_diameter, axle_track, motor_1_sensor_port, motor_2_sensor_port, color_sensor_port)

    def lead_driving(self, send_speed: bool):
        
        if send_speed:
            # Instanciate this robot as a Server
            server = BluetoothMailboxServer()
            mbox = TextMailbox("SpaceTab", server)

            # The server must be started before the client!
            print("Waiting for connection...")
            server.wait_for_connection()
            print("Connected !")
        
        timer = StopWatch()
        timer.reset()
        
        while(True):
            # Drive 3sec then wait 3sec
            self.motors.drive(self.speed, 0)
            
            if timer.time() > T:
                if send_speed: mbox.send(str(self.speed))
                self.motors.drive(0, 0)
                wait(T)
                timer.reset()



# def main():
#     wheel_diameter=56 
#     axle_track=114
#     # Initialize robots main components
#     my_robot = RobotLeader(wheel_diameter, axle_track, Port.B, Port.C, Port.S4)
#     my_robot.lead_driving(1)
#     # Start following the line endlessly.
    

#if __name__=='__main__':
#    main()