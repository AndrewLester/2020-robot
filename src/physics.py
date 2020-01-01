from pyfrc.physics import drivetrains


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Swerve Drive joystick control
    """
    def __init__(self, physics_controller):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                       to communicate simulation effects to
        """
        self.physics_controller = physics_controller
        self.physics_controller.add_device_gyro_channel('navxmxp_spi_4_angle')

        self.fl_turn_encoder = 0
        self.fr_turn_encoder = 0
        self.rl_turn_encoder = 0
        self.rr_turn_encoder = 0

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

        # 5 Corresponds to speed=5 in swerve_drivetrain creation
        self.fl_turn_encoder += hal_data['CAN'][0]['value'] * tm_diff * 5
        self.fr_turn_encoder += hal_data['CAN'][1]['value'] * tm_diff * 5
        self.rl_turn_encoder += hal_data['CAN'][2]['value'] * tm_diff * 5
        self.rr_turn_encoder += hal_data['CAN'][3]['value'] * tm_diff * 5

        vx, vy, vw = drivetrains.four_motor_swerve_drivetrain(
            rl_motor, rr_motor, fl_motor, fr_motor, self.rl_turn_encoder, self.rr_turn_encoder,
            self.fl_turn_encoder, self.fr_turn_encoder
        )

        self.physics_controller.vector_drive(vx, vy, vw, tm_diff)
