import math

import commands2
import ntcore
import wpilib
import wpilib.drive
import romi
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveKinematics



class Drivetrain(commands2.Subsystem):
    kCountsPerRevolution = 1440.0
    kWheelDiameterInch = 2.75591
    MAX_LINEAR_SPEED=.75 #meters per second (down from theoretical max of .89)
    MAX_ANGULAR_SPEED=10 # radians per second (down from theoretical max of 12.6)

    def __init__(self) -> None:
        super().__init__()

        # The Romi has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftMotor = wpilib.Spark(0)
        self.rightMotor = wpilib.Spark(1)

        # The Romi has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = wpilib.Encoder(4, 5)
        self.rightEncoder = wpilib.Encoder(6, 7)

        # Set up the differential drive controller
        self.drive = wpilib.drive.DifferentialDrive(self.leftMotor, self.rightMotor)

        # Set up the RomiGyro
        self.gyro = romi.RomiGyro()

        # Set up the BuiltInAccelerometer
        self.accelerometer = wpilib.BuiltInAccelerometer()

        # Use inches as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.resetEncoders()

        # Values calculated for ROMI 1 at home on 3/22/25
        self.feedforward_left = SimpleMotorFeedforwardMeters(kS=0.4436, kV=2.3234, kA=0)  # Your constants
        self.feedforward_right = SimpleMotorFeedforwardMeters(kS=0.39921, kV=2.3418, kA=0.3)  # Your constants

        self.kinematics = DifferentialDriveKinematics(trackWidth=0.14)  # ROMI width in meters

    def arcadeDrive(self, fwd: float, rot: float) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        # 1. Get arcade drive input
        fwd = -fwd * self.MAX_LINEAR_SPEED  # forward
        rot = rot * self.MAX_ANGULAR_SPEED # rotation

        # 2. Convert to chassis speeds
        chassis_speeds = ChassisSpeeds(fwd, 0.0, rot)

        # 3. Convert chassis speeds to wheel speeds
        wheel_speeds = self.kinematics.toWheelSpeeds(chassis_speeds)

        left_speed = wheel_speeds.left / (self.kWheelDiameterInch *.0254 * math.pi) # Rotations per second
        right_speed = wheel_speeds.right/ (self.kWheelDiameterInch *.0254 * math.pi)

        # 4. Use feedforward to get voltages
        left_voltage = self.feedforward_left.calculate(left_speed)
        right_voltage = self.feedforward_right.calculate(right_speed)

        # 5. Send voltages to motors
        self.leftMotor.setVoltage(left_voltage)
        self.rightMotor.setVoltage(right_voltage)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getLeftEncoderCount(self) -> int:
        return self.leftEncoder.get()

    def getRightEncoderCount(self) -> int:
        return self.rightEncoder.get()

    def getLeftDistanceInch(self) -> float:
        return self.leftEncoder.getDistance()

    def getRightDistanceInch(self) -> float:
        return self.rightEncoder.getDistance()

    def getAverageDistanceInch(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return (self.getLeftDistanceInch() + self.getRightDistanceInch()) / 2.0

    def getAccelX(self) -> float:
        """The acceleration in the X-axis.

        :returns: The acceleration of the Romi along the X-axis in Gs
        """
        return self.accelerometer.getX()

    def getAccelY(self) -> float:
        """The acceleration in the Y-axis.

        :returns: The acceleration of the Romi along the Y-axis in Gs
        """
        return self.accelerometer.getY()

    def getAccelZ(self) -> float:
        """The acceleration in the Z-axis.

        :returns: The acceleration of the Romi along the Z-axis in Gs
        """
        return self.accelerometer.getZ()

    def getGyroAngleX(self) -> float:
        """Current angle of the Romi around the X-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleX()

    def getGyroAngleY(self) -> float:
        """Current angle of the Romi around the Y-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleY()

    def getGyroAngleZ(self) -> float:
        """Current angle of the Romi around the Z-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleZ()

    def resetGyro(self) -> None:
        """Reset the gyro"""
        self.gyro.reset()

    def periodic(self) -> None:
        self.nt_drivetrain=ntcore.NetworkTableInstance.getDefault().getTable("Drivetrain")
        self.nt_drivetrain.putNumber("Left Encoder", self.getLeftEncoderCount())
        self.nt_drivetrain.putNumber("Right Encoder", self.getRightEncoderCount())
        self.nt_drivetrain.putNumber("Left Distance", self.getLeftDistanceInch())
        self.nt_drivetrain.putNumber("Right Distance", self.getRightDistanceInch())
        self.nt_drivetrain.putNumber("Average Distance", self.getAverageDistanceInch())
        self.nt_drivetrain.putNumber("Accel X", self.getAccelX())
        self.nt_drivetrain.putNumber("Accel Y", self.getAccelY())
        self.nt_drivetrain.putNumber("Accel Z", self.getAccelZ())
        self.nt_drivetrain.putNumber("Gyro X", self.getGyroAngleX())
        self.nt_drivetrain.putNumber("Gyro Y", self.getGyroAngleY())
        self.nt_drivetrain.putNumber("Gyro Z", self.getGyroAngleZ())
