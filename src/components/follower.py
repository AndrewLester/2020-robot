import pickle
from typing import Tuple

import pathfinder as pf
import wpilib
from wpilib.geometry import Pose2d, Rotation2d
from magicbot import tunable

from components.drive import SwerveDrive, SwerveModule
from components.tracking import Odometry


class Follower:
    TICKS_PER_REV = 1024
    WHEEL_DIAMETER = 0.152  # Meters

    KP = tunable(0)
    KI = tunable(0)
    KD = tunable(0)

    odometry: Odometry
    swerve_drive: SwerveDrive
    modules: Tuple[SwerveModule]

    @staticmethod
    def load_trajectories(file_name):
        """
        Either generate and write trajectories if in a sim or read them if on the robot.
        """
        try:
            with open(file_name, 'rb') as f:
                generated_trajectories = pickle.load(f)
        except FileNotFoundError:
            generated_trajectories = {}

        return generated_trajectories

    def setup(self):
        self.followers = [pf.followers.DistanceFollower(None) for _ in range(4)]

    # Using a DistanceFollower with Odometry now
    # def configureEncoders(self):
    #     follower: pf.followers.DistanceFollower
    #     for i, follower in enumerate(self.followers):
    #         follower.configureEncoder(self.modules[i].drive_encoder.get(),
    #                                   self.TICKS_PER_REV,
    #                                   self.WHEEL_DIAMETER)

    def configurePIDVA(self):
        for module, follower in zip(self.modules, self.followers):
            follower.configurePIDVA(self.KP, self.KI, self.KD, module.config.KV, module.config.KA)

    def follow_trajectory(self, trajectory_name):
        # self.configureEncoders()
        self.configurePIDVA()

    def execute(self):
        module: SwerveModule
        follower: pf.followers.DistanceFollower
        for module, follower in zip(self.modules, self.followers):
            output = follower.calculate(Pose2d(module.offset, Rotation2d()).relativeTo(self.odometry.current_pose))
            heading = pf.r2d(follower.getHeading())

            # module.percent = output
            print(output)
            # module.angle = heading
