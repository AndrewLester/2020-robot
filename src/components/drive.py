import math
from collections import namedtuple
from dataclasses import dataclass
from typing import List

import wpilib
from ctre import WPI_TalonSRX
from magicbot import will_reset_to
from networktables.util import _NtProperty
from wpilib.geometry import Rotation2d, Translation2d
from wpilib.kinematics import ChassisSpeeds
from wpilib.kinematics.swerve import SwerveDriveKinematics, SwerveModuleState

from common import PIDController, SimpleMotorFeedforward

SwerveModuleList = namedtuple('SwerveModuleList', ['module1', 'module2', 'module3', 'module4'])


@dataclass(init=False)
class SwerveModuleConfig:
    """Holds the constants used by a swerve module"""
    KP_DRIVE: float
    KI_DRIVE: float
    KD_DRIVE: float

    KP_TURN: float
    KI_TURN: float
    KD_TURN: float

    KEY_PREFIX = '/robot/swerve_module_{}/constants/'

    def __init__(
            self, module_num: int, KS: float, KV: float, KA: float,
            KP_DRIVE: float, KI_DRIVE: float, KD_DRIVE: float,
            KP_TURN: float, KI_TURN: float, KD_TURN: float):
        """
        Construct a SwerveModuleConfig object with the specified constants

        :param module_num: The module number
        :param KS: The static constant
        :param KV: The velocity constant
        :param KA: The acceleration constant
        :param KP_DRIVE: The proportional gain for the drive motor
        :param KI_DRIVE: The integral gain for the drive motor
        :param KD_DRIVE: The derivative gain for the drive motor
        :param KP_TURN: The proportional gain for the turn motor
        :param KI_TURN: The integral gain for the turn motor
        :param KD_TURN: The derivative gain for the turn motor
        """
        self.KEY_PREFIX = SwerveModuleConfig.KEY_PREFIX.format(module_num)

        self.KS = KS
        self.KV = KV
        self.KA = KA

        self.KP_DRIVE = self._ntvariable(f'{self.KEY_PREFIX}kp_drive', KP_DRIVE)
        self.KI_DRIVE = self._ntvariable(f'{self.KEY_PREFIX}ki_drive', KI_DRIVE)
        self.KD_DRIVE = self._ntvariable(f'{self.KEY_PREFIX}kd_drive', KD_DRIVE)

        self.KP_TURN = self._ntvariable(f'{self.KEY_PREFIX}kp_turn', KP_TURN)
        self.KI_TURN = self._ntvariable(f'{self.KEY_PREFIX}ki_turn', KI_TURN)
        self.KD_TURN = self._ntvariable(f'{self.KEY_PREFIX}kd_turn', KD_TURN)

    def _ntvariable(self, key: str, default: float) -> _NtProperty:
        """
        Creates a network tables property dynamically. Must be used alongside overidden
        __getattribute__ and __setattr__

        :param key: The network tables key
        :param default: The default value for the property
        :return: The NTProperty
        """
        return _NtProperty(key, default, writeDefault=True, persistent=False)

    def __getattribute__(self, name):
        """
        Overrides the method of getting an NTProperty to get its value instead

        :param name: The name of the attribute
        :return: The attribute
        """
        if isinstance(object.__getattribute__(self, name), _NtProperty):
            return object.__getattribute__(self, name).get(None)
        return object.__getattribute__(self, name)

    def __setattr__(self, name, value):
        """
        Uses the network tables internal set function for an ntproperty

        :param name: The name of the attribute
        :param value: The value to set the attribute to
        """
        try:
            if isinstance(object.__getattribute__(self, name), _NtProperty):
                object.__getattribute__(self, name).set(None, value)
                return
        except AttributeError:
            pass
        object.__setattr__(self, name, value)


class SwerveModule:
    """Represents a module (drive motor and turn motor) in a swerve drivetrain"""
    MOTOR_VOLTAGE = 12
    DRIVE_ENCODER_PULSE_PER_REV = 1024
    DRIVE_WHEEL_DIAMETER = 0.152  # Meters
    TURN_ENCODER_PULSE_PER_REV = 256
    # TODO: Find real value
    TURN_DIAMETER = 0.1  # Meters

    def __init__(
            self, motor: WPI_TalonSRX, turn_motor: wpilib.MotorSafety,
            offset: Translation2d, drive_encoder: wpilib.Encoder,
            turn_encoder: wpilib.Encoder, config: SwerveModuleConfig,
            master: WPI_TalonSRX = None):
        """
        :param motor: The drive motor for the module
        :param turn_motor: The turn motor for the module
        :param offset: The offset of the module from the robot's center
        :param drive_encoder: The drive encoder for the module
        :param turn_encoder: The turning encoder for the module
        :param config: The config which holds this module's constants
        :param master: The drive motor which this module's drive motor should follow
        """
        self.motor = motor
        self.turn_motor = turn_motor
        self.offset = offset
        self.config = config

        self.drive_encoder = drive_encoder
        self.drive_encoder.setDistancePerPulse(
            (1 / self.DRIVE_ENCODER_PULSE_PER_REV) * self.DRIVE_WHEEL_DIAMETER * math.pi
        )
        self.turn_encoder = turn_encoder
        self.turn_encoder.setDistancePerPulse(
            (1 / self.TURN_ENCODER_PULSE_PER_REV) * self.TURN_DIAMETER * math.pi
        )

        self.speed = 0.0
        self._angle = Rotation2d(0)

        self.turn_controller = PIDController(config.KP_TURN, config.KI_TURN, config.KD_TURN)
        self.turn_controller.enableContinuousInput(0, 360)
        self.turn_controller.setIntegratorRange(0, 30)

        if self.drive_encoder is not None:
            self.speed_controller = PIDController(config.KP_DRIVE, config.KI_DRIVE, config.KD_DRIVE)
            self.speed_feedforward = SimpleMotorFeedforward(config.KS, config.KV, config.KA)
        else:
            self.speed_controller = None
            self.speed_feedforward = None
            self.motor.follow(master)

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, rotation: Rotation2d):
        # If the angle is more than 90 degrees away, simply rotate to the flipped version of the angle
        # and multiply the speed by -1
        if rotation == self._angle:
            return

        # Reset Total and Previous error values
        self.turn_controller.reset()

        new = math.degrees(rotation.value)
        current = math.degrees(self._angle.value)

        angle_between = abs(new - current)
        angle_between = 360 - angle_between if angle_between > 180 else angle_between
        if angle_between > 90:
            self._angle = Rotation2d.fromDegrees((new + 180) % 360)
            self.speed *= -1
        else:
            self._angle = rotation

    def volts_to_pvbus(self, volts):
        """
        Turn volts into percent output

        :param volts: The number of volts
        :return: The percent output
        """
        return volts / self.MOTOR_VOLTAGE

    def ticks_to_degrees(self, ticks, ticks_per_rev):
        """
        Turn encoder ticks into degrees

        :param ticks: The number of encoder ticks
        :param ticks_per_rev: The number of encoder ticks per revolution
        :return: The number of degrees
        """
        return ticks * (360 / ticks_per_rev)

    def execute(self):
        """
        Set the outputs for the drive motor and the turning motor
        """
        if self.speed_controller:
            output = self.volts_to_pvbus(
                self.speed_feedforward.calculate(self.speed) +
                self.speed_controller.calculate(self.drive_encoder.getRate(), self.speed)
            )

            self.motor.set(output)

        # FIXME: For some reason the PID Controller does not go to a setpoint of 0
        output = self.turn_controller.calculate(
            self.ticks_to_degrees(
                self.turn_encoder.get(),
                self.TURN_ENCODER_PULSE_PER_REV
            ), math.degrees(self.angle.value)
        )
        self.turn_motor.set(output)

        # Reset speed manually because this not a RobotPY Component, nor is it a will_reset_to variable
        # Angle can stay at whatever it was because it's more unsafe to move it than keep it still
        self.speed = 0


class SwerveDrive:
    """Accepts drive input and sets module speeds and angles accordingly"""
    # Max speed along 1 axis
    MAX_SPEED = 10.0  # TODO: Decide whether using units library. This is currently in meters per second
    # One rotation per second
    MAX_TURNING_SPEED = math.tau  # Radians per second.

    modules = SwerveModuleList
    swerve_kinematics: SwerveDriveKinematics

    speeds = will_reset_to(ChassisSpeeds())
    raw_states = will_reset_to(None)
    using_raw = will_reset_to(False)
    using_joystick = will_reset_to(False)

    def vector_move(self, vx: float, vy: float, omega: float,
                    field_relative: Translation2d = None, joystick=False):
        """
        Moves the robot with specified speeds

        :param vx: The x speed
        :param vy: The y speed
        :param omega: The rotational speed
        :param field_relative: Whether the speeds should be treated as field relative
        :param joystick: Whether the speeds should be treated as percentages of the maximum speed
        """
        if joystick:
            # Treat vx, vy, and omega like percentages of max speed
            vx *= self.MAX_SPEED
            vy *= self.MAX_SPEED
            omega *= self.MAX_TURNING_SPEED

        self.speeds = ChassisSpeeds(vx, vy, omega)
        if field_relative is not None:
            self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, field_relative)

        self.using_joystick = True

    def move(self, speeds: List[float], angles: List[float]):
        """
        Move the robot with raw speeds and angles.
        Module order: FL, FR, RL, RR
        :param speeds: The individual module speeds
        :param angles: The individual module angles
        """
        self.raw_states = []
        for speed, angle in zip(speeds, angles):
            self.raw_states.append(SwerveModuleState(speed, angle))

        self.using_raw = True

    def execute(self):
        """
        Turn the chassis speeds into individual module speeds and give them to the modules
        """
        states = self.swerve_kinematics.toSwerveModuleStates(self.speeds)
        self.swerve_kinematics.normalizeWheelSpeeds(states, self.MAX_SPEED)

        if self.using_raw:
            states = self.raw_states

        state: SwerveModuleState
        module: SwerveModule
        for module, state in zip(self.modules, states):
            module.speed = state.speed

            # If the robot wasn't told to move, have the turning motors, by default,
            # keep their current rotations. This should be safer than moving them back to 0 degrees
            if self.using_joystick or self.using_raw:
                module.angle = state.angle
            module.execute()
