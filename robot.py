import os
from commands2 import CommandScheduler, TimedCommandRobot
from wpilib import Joystick, Spark
from subsystems.drivetrain import Drivetrain
import ntcore


os.environ["HALSIMWS_HOST"] = "10.0.0.2"
os.environ["HALSIMWS_PORT"] = "3300"


class LilRedRobot(TimedCommandRobot):

    def robotInit(self):
        self.joystick = Joystick(0)
        self.drivetrain = Drivetrain()
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.scheduler = CommandScheduler.getInstance()

    def robotPeriodic(self):
        pass

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        forward = self.joystick.getX()
        rotation = self.joystick.getY()
        self.drivetrain.arcadeDrive(forward, rotation)


