import pickle

import pathfinder as pf
from wpilib.geometry import Pose2d, Rotation2d
from magicbot import tunable

from components.drive import SwerveDrive, SwerveModule, SwerveModuleList
from components.tracking import Odometry


class Follower:
    """Uses Jaci's pathfinding follower classes to follow a trajectory"""
    TICKS_PER_REV = 1024
    WHEEL_DIAMETER = 0.152  # Meters

    KP = tunable(0)
    KI = tunable(0)
    KD = tunable(0)

    odometry: Odometry
    swerve_drive: SwerveDrive
    modules: SwerveModuleList

    def setup(self):
        """Sets up distance followers"""
        self.followers = [pf.followers.DistanceFollower(None) for _ in range(4)]
        self.current_trajectory = None

    def load_trajectories(self, file_name):
        """
        Load trajectories from a file and store them in the follower

        :param file_name: The file to read trajectories from
        """
        try:
            with open(file_name, 'rb') as f:
                generated_trajectories = pickle.load(f)
        except FileNotFoundError:
            generated_trajectories = {}

        self.generated_trajectories = generated_trajectories

    # Using a DistanceFollower with Odometry now
    # def configureEncoders(self):
    #     follower: pf.followers.DistanceFollower
    #     for i, follower in enumerate(self.followers):
    #         follower.configureEncoder(self.modules[i].drive_encoder.get(),
    #                                   self.TICKS_PER_REV,
    #                                   self.WHEEL_DIAMETER)

    def configurePIDVA(self):
        """Configures PIDVA constants for the followers"""
        for module, follower in zip(self.modules, self.followers):
            follower.configurePIDVA(self.KP, self.KI, self.KD, module.config.KV, module.config.KA)

    def follow_trajectory(self, trajectory_name):
        """
        Sets the current trajectory to follow and starts followers

        :param trajectory_name: The name of the trajectory to follow
        """
        self.current_trajectory = self.generated_trajectories[trajectory_name]

        # self.configureEncoders()
        self.configurePIDVA()
        for follower in self.followers:
            follower.reset()
            follower.setTrajectory(self.current_trajectory)

    def execute(self):
        """Uses followers to calculate speeds and angles which are sent to the Swerve Modules"""
        if self.current_trajectory is None:
            return

        # If all the followers are finished, stop executing
        for follower in self.followers:
            if not follower.finished:
                break
        else:
            self.current_trajectory = None
            return

        module: SwerveModule
        follower: pf.followers.DistanceFollower
        for module, follower in zip(self.modules, self.followers):
            output = follower.calculate(Pose2d(module.offset, Rotation2d()).relativeTo(self.odometry.current_pose))
            heading = pf.r2d(follower.getHeading())

            # module.percent = output
            print(output, heading)
            # module.angle = heading
