import pickle
from typing import Tuple

import pathfinder as pf
import wpilib
from magicbot import tunable

from components.drive import SwerveDrive, SwerveModule


class Follower:
    TICKS_PER_REV = 1024
    WHEEL_DIAMETER = 0.152  # Meters

    KP = tunable(0)
    KI = tunable(0)
    KD = tunable(0)

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
        self.followers = [pf.followers.EncoderFollower(None) for _ in range(4)]

    def configureEncoders(self):
        follower: pf.followers.EncoderFollower
        for i, follower in enumerate(self.followers):
            follower.configureEncoder(self.modules[i].drive_encoder.get(),
                                      self.TICKS_PER_REV,
                                      self.WHEEL_DIAMETER)

    def configurePIDVA(self):
        for i, follower in enumerate(self.followers):
            module = self.modules[i]
            follower.configurePIDVA(self.KP, self.KI, self.KD, module.config.KV, module.config.KA)

    def follow_trajectory(self, trajectory_name):
        self.configureEncoders()
        self.configurePIDVA()

    def execute(self):
        pass
