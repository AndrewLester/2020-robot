import math
from typing import Tuple

import wpilib
from wpilib.kinematics.swerve import SwerveDriveKinematics, SwerveDriveOdometry, SwerveModuleState
from wpilib.geometry import Rotation2d, Pose2d
import navx

from components.drive import SwerveModule, SwerveModuleList


class Odometry:
    WHEEL_CIRCUMFERENCE = 0.1524 * math.pi # Meters
    TICKS_PER_REV = 1024

    modules: SwerveModuleList
    swerve_kinematics: SwerveDriveKinematics
    navx: navx.AHRS

    def setup(self):
        self.odometry = SwerveDriveOdometry(self.swerve_kinematics, 
                                            Rotation2d.fromDegrees(-self.navx.getAngle()))
    
    def meters_to_ticks(self, meters: int):
        return (self.TICKS_PER_REV / self.WHEEL_CIRCUMFERENCE) * meters

    def ticks_to_meters(self, ticks: int):
        return (self.WHEEL_CIRCUMFERENCE / self.TICKS_PER_REV) * ticks

    @property
    def current_pose(self) -> Pose2d:
        return self.odometry.getPose()

    def execute(self):
        self.odometry.updateWithTime(
            wpilib.Timer.getFPGATimestamp(),
            Rotation2d.fromDegrees(-self.navx.getAngle()),
            *[SwerveModuleState(module.speed, module.angle) for module in self.modules]
        )
