from typing import Tuple

import wpilib
from wpilib.kinematics.swerve import SwerveDriveKinematics, SwerveDriveOdometry
import navx

from components.drive import SwerveModule


class Odometry:
    modules: Tuple[SwerveModule]
    swerve_kinematics: SwerveDriveKinematics
    navx: navx.AHRS

    def setup(self):
        self.odometry = SwerveDriveOdometry(self.swerve_kinematics, self.navx)
