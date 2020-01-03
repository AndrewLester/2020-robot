import os
import sys
from typing import Tuple

import magicbot
import navx
import wpilib
from ctre import WPI_TalonSRX
from robotpy_ext.control.toggle import Toggle
from wpilib.geometry import Rotation2d, Translation2d
from wpilib.kinematics import ChassisSpeeds
from wpilib.kinematics.swerve import SwerveDriveKinematics

from components.drive import SwerveDrive, SwerveModule, SwerveModuleConfig, SwerveModuleList
from components.tracking import Odometry
from components.follower import Follower


r"""
/ \                / \
\ /       (+)      \ /
           |
           |X
(+) -------|--Y----  (-)
           |
           |
/ \       (-)      / \
\ /                \ /

Counter-Clockwise is Positive
   /-\ ^ 
   |X| | (+)
   \-/ |
   -->
"""


class Robot(magicbot.MagicRobot):
    TRAJECTORY_DIRECTORY = 'trajectories'
    PICKLE_FILE = os.path.join(TRAJECTORY_DIRECTORY, 'trajectories.pickle')

    odometry: Odometry
    swerve_drive: SwerveDrive
    follower: Follower

    def createObjects(self):
        self.navx = navx.AHRS.create_spi()
        self.navx.reset()

        self.joystick_l = wpilib.Joystick(0)
        self.joystick_r = wpilib.Joystick(1)
        self.joystick_alt = wpilib.Joystick(2)

        self.field_relative = Toggle(self.joystick_l, 5)

        self.fl_motor = WPI_TalonSRX(0)
        self.fr_motor = WPI_TalonSRX(1)
        self.rl_motor = WPI_TalonSRX(2)
        self.rr_motor = WPI_TalonSRX(3)

        self.modules = SwerveModuleList(
            # TODO: Measure offsets
            # TODO: Tune PID Controllers
            # Only the Front-Right and Rear-Left modules have Drive encoders
            SwerveModule(
                self.fl_motor, wpilib.VictorSP(4), Translation2d(0.5, 0.5),
                wpilib.Encoder(12, 13), wpilib.Encoder(0, 1),
                SwerveModuleConfig(0,
                    KS=3, KV=0.23, KA=0.12, KP_DRIVE=0.1, KI_DRIVE=0,
                    KD_DRIVE=0.05, KP_TURN=0.7, KI_TURN=0.1, KD_TURN=0.01
                )
            ),  # Front-Left
            SwerveModule(
                self.fr_motor, wpilib.VictorSP(5), Translation2d(0.5, -0.5),
                wpilib.Encoder(2, 3), wpilib.Encoder(4, 5),
                SwerveModuleConfig(1,
                    KS=3, KV=0.23, KA=0.12, KP_DRIVE=0.1, KI_DRIVE=0,
                    KD_DRIVE=0.05, KP_TURN=0.7, KI_TURN=0.1, KD_TURN=0.01
                )
            ),  # Front-Right
            SwerveModule(
                self.rl_motor, wpilib.VictorSP(6), Translation2d(-0.5, 0.5),
                wpilib.Encoder(6, 7), wpilib.Encoder(8, 9),
                SwerveModuleConfig(2,
                    KS=3, KV=0.23, KA=0.12, KP_DRIVE=0.1, KI_DRIVE=0,
                    KD_DRIVE=0.05, KP_TURN=0.7, KI_TURN=0.1, KD_TURN=0.01
                )
            ),  # Rear-Left
            SwerveModule(
                self.rr_motor, wpilib.VictorSP(7), Translation2d(-0.5, -0.5),
                wpilib.Encoder(14, 15), wpilib.Encoder(10, 11),
                SwerveModuleConfig(3,
                    KS=3, KV=0.23, KA=0.12, KP_DRIVE=0.1, KI_DRIVE=0,
                    KD_DRIVE=0.05, KP_TURN=0.7, KI_TURN=0.1, KD_TURN=0.01
                )
            )  # Rear-Right
        )

        self.swerve_kinematics = SwerveDriveKinematics(*[module.offset for module in self.modules])

    def robotInit(self):
        super().robotInit()
        """
        if wpilib.RobotBase.isSimulation():
            from pyfrc.sim import get_user_renderer
            
            renderer = get_user_renderer()
            if renderer:
                renderer.draw_pathfinder_trajectory(modifier.getLeftTrajectory(), 
                                                    '#0000ff', offset=(-0.9, 0))
                renderer.draw_pathfinder_trajectory(modifier.source, '#00ff00', show_dt=True)
                renderer.draw_pathfinder_trajectory(modifier.getRightTrajectory(), 
                                                    '#0000ff', offset=(0.9, 0))
        """

    def disabledInit(self):
        self.follower.load_trajectories(self.PICKLE_FILE)
        super().disabledInit()

    def robotPeriodic(self):
        pass

    def teleopPeriodic(self):
        self.swerve_drive.joystick_move(
            self.joystick_l.getY(),
            self.joystick_l.getX(),
            -self.joystick_r.getX(),
            field_relative=(Rotation2d.fromDegrees(-self.navx.getAngle()) 
                            if self.field_relative.get() else None)
        )


if __name__ == '__main__':
    wpilib.run(Robot)
