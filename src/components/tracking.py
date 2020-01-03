import math

import wpilib
from wpilib.kinematics.swerve import SwerveDriveKinematics, SwerveDriveOdometry, SwerveModuleState
from wpilib.geometry import Rotation2d, Pose2d
import navx

from components.drive import SwerveModuleList


class Odometry:
    """Tracks the robot's pose as it moves"""
    WHEEL_CIRCUMFERENCE = 0.1524 * math.pi  # Meters
    TICKS_PER_REV = 1024

    modules: SwerveModuleList
    swerve_kinematics: SwerveDriveKinematics
    navx: navx.AHRS

    def setup(self):
        """Sets up odometry object"""
        self.odometry = SwerveDriveOdometry(self.swerve_kinematics,
                                            Rotation2d.fromDegrees(-self.navx.getAngle()))

    def meters_to_ticks(self, meters: int):
        """
        Converts meters to encoder ticks based on the wheel circumference

        :param meters: The number of meters
        :return: The number of encoder ticks
        """
        return (self.TICKS_PER_REV / self.WHEEL_CIRCUMFERENCE) * meters

    def ticks_to_meters(self, ticks: int):
        """
        Converts encoder ticks to meters based on the wheel circumference

        :param ticks: The number of encoder ticks
        :return: The number of meters
        """
        return (self.WHEEL_CIRCUMFERENCE / self.TICKS_PER_REV) * ticks

    @property
    def current_pose(self) -> Pose2d:
        return self.odometry.getPose()

    def execute(self):
        """Updates the odometry object with the new pose"""
        self.odometry.updateWithTime(
            wpilib.Timer.getFPGATimestamp(),
            Rotation2d.fromDegrees(-self.navx.getAngle()),
            *[SwerveModuleState(module.speed, module.angle) for module in self.modules]
        )
