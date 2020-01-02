import math

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains
from wpilib.kinematics import ChassisSpeeds
from wpilib.kinematics.swerve import SwerveDriveKinematics, SwerveModuleState, SwerveDriveOdometry
from wpilib.geometry import Rotation2d, Translation2d

class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Swerve Drive joystick control
    """
    DRIVE_ENCODER_CONSTANT = (1 / 1024) * 0.152 * math.pi # Units meters per tick
    TURN_ENCODER_CONSTANT = (1 / 256) * 0.1 * math.pi # Units meters per tick

    def __init__(self, physics_controller: PhysicsInterface):
        """
        :param physics_controller: `pyfrc.physics.core.PhysicsInterface` object
                                       to communicate simulation effects to
        """
        self.physics_controller = physics_controller
        self.physics_controller.add_device_gyro_channel('navxmxp_spi_4_angle')

        self.kinematics = SwerveDriveKinematics(
            Translation2d(0.5, 0.5),
            Translation2d(0.5, -0.5),
            Translation2d(-0.5, 0.5),
            Translation2d(-0.5, -0.5)
        )

        self.odometry = SwerveDriveOdometry(self.kinematics, Rotation2d(self.physics_controller.angle))

    def update_sim(self, hal_data, now, tm_diff):
        """
        Called when the simulation parameters for the program need to be
        updated.
        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        fl_motor = hal_data['CAN'][0]['value']
        fr_motor = hal_data['CAN'][1]['value']
        rl_motor = hal_data['CAN'][2]['value']
        rr_motor = hal_data['CAN'][3]['value']

        fl_turn_motor = hal_data['pwm'][4]['value']
        fr_turn_motor = hal_data['pwm'][5]['value']
        rl_turn_motor = hal_data['pwm'][6]['value']
        rr_turn_motor = hal_data['pwm'][7]['value']

        # for i, encoder in enumerate(hal_data['encoder']):
        #     print(f'{i}: {encoder["config"]["ASource_Channel"]} {encoder["config"]["BSource_Channel"]}')

        # print('**********')
        hal_data['encoder'][1]['count'] += int(fl_turn_motor)
        hal_data['encoder'][3]['count'] += int(fr_turn_motor)
        hal_data['encoder'][5]['count'] += int(rl_turn_motor)
        hal_data['encoder'][7]['count'] += int(rr_turn_motor)

        # 5 Corresponds to speed=5 in swerve_drivetrain creation

        vx, vy, vw = drivetrains.four_motor_swerve_drivetrain(
            rl_motor, rr_motor, fl_motor, fr_motor, hal_data['encoder'][5]['count'],
            hal_data['encoder'][7]['count'], hal_data['encoder'][1]['count'],
            hal_data['encoder'][3]['count']
        )

        # Invert vw because Counter-Clockwise needs to be positive
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, -vw, Rotation2d(self.physics_controller.angle))
        states = self.kinematics.toSwerveModuleStates(speeds)
        self.odometry.updateWithTime(now,
                                     Rotation2d(self.physics_controller.angle),
                                     *states)

        for i, state in enumerate(states):
            hal_data['encoder'][i * 2]['count'] += int(state.speed * tm_diff * (1 / self.DRIVE_ENCODER_CONSTANT))
            hal_data['encoder'][i * 2]['rate'] = state.speed


        self.physics_controller.vector_drive(vx, vy, vw, tm_diff)
