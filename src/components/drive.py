from collections import namedtuple
from typing import Tuple, List
from dataclasses import dataclass
import math

import wpilib
from ctre import WPI_TalonSRX
from magicbot import will_reset_to
from wpilib.geometry import Translation2d, Rotation2d
from wpilib.kinematics import ChassisSpeeds
from wpilib.kinematics.swerve import SwerveDriveKinematics, SwerveModuleState
from networktables.util import ntproperty


SwerveModuleList = namedtuple('SwerveModuleList', ['module1', 'module2', 'module3', 'module4'])


@dataclass(init=False)
class SwerveModuleConfig:
    # Feedforward
    KS: float  # Units: Volts
    KV: float  # Units: Volts * seconds / meter
    KA: float  # Units: Volts * seconds^2 / meter. Value may be ommitted

    # PID Speed on drive wheel
    KP_DRIVE: float
    KI_DRIVE: float
    KD_DRIVE: float

    # PID Angle on turning motor
    KP_TURN: float
    KI_TURN: float
    KD_TURN: float

    def __init__(
            self, KS: float, KV: float, KA: float,
            KP_DRIVE: float, KI_DRIVE: float, KD_DRIVE: float,
            KP_TURN: float, KI_TURN: float, KD_TURN: float):
        self.KS = KS
        self.KV = KV
        self.KA = KA

        self.KP_DRIVE = ntproperty('/robot/constants/kp_drive', KP_DRIVE)
        self.KI_DRIVE = ntproperty('/robot/constants/ki_drive', KI_DRIVE)
        self.KD_DRIVE = ntproperty('/robot/constants/kd_drive', KD_DRIVE)

        self.KP_TURN = ntproperty('/robot/constants/kp_turn', KP_TURN)
        self.KI_TURN = ntproperty('/robot/constants/ki_turn', KI_TURN)
        self.KD_TURN = ntproperty('/robot/constants/kd_turn', KD_TURN)


class SwerveModule:
    def __init__(
            self, motor: WPI_TalonSRX, turn_motor: wpilib.MotorSafety,
            offset: Translation2d, drive_encoder: wpilib.Encoder,
            turn_encoder: wpilib.Encoder, config: SwerveModuleConfig, master: WPI_TalonSRX=None):
        self.motor = motor
        self.turn_motor = turn_motor
        self.offset = offset
        self.config = config

        self.drive_encoder = drive_encoder
        self.turn_encoder = turn_encoder

        self.speed = 0
        self.angle = Rotation2d(0)

        # TODO: Create copy PID Controller class of new wpilibj PID Controller class in the common folder
        self.turn_controller = wpilib.PIDController(
            config.KP_TURN, config.KI_TURN, config.KD_TURN, self.turn_encoder.get, self.turn_motor
        )
        self.turn_controller.setInputRange(0, 360)
        self.turn_controller.setContinuous(True)

        if self.drive_encoder is not None:
            self.speed_controller = wpilib.PIDController(
                config.KP_DRIVE, config.KI_DRIVE, config.KD_DRIVE, self.drive_encoder.getRate, self.motor
            )
        else:
            self.speed_controller = None
            self.motor.follow(master)

    def execute(self):
        # TODO: Use Feedforward + Feedback (PID) calculations for speed setting
        # self.motor.set(self.speed)
        print(self.speed)
        if self.speed_controller:
            self.speed_controller.setSetpoint(self.speed)
        # TODO: Use Feedback (PID) calculations for angle setting
        # self.turn_motor.set(self.angle)
        self.turn_controller.setSetpoint(math.degrees(self.angle.value))

        # Reset speed manually because this not a RobotPY Component, nor is it a will_reset_to variable
        # Angle can stay at whatever it was because it's more unsafe to move it than keep it still
        self.speed = 0


class SwerveDrive:
    MAX_SPEED = 5  # TODO: Decide whether using units library. This is currently in meters per second

    modules = SwerveModuleList
    swerve_kinematics: SwerveDriveKinematics

    speeds = will_reset_to(ChassisSpeeds())
    raw_states = will_reset_to([])
    using_joystick = will_reset_to(False)

    def joystick_move(self, vx: float, vy: float, omega: float, field_relative: Translation2d=None):
        self.speeds = ChassisSpeeds(vx, vy, omega)
        if field_relative:
            self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, field_relative)
        
        self.using_joystick = True

    def move(self, speeds: List[float], angles: List[float]):
        for speed, angle in zip(speeds, angles):
            self.raw_states.append(SwerveModuleState(speed, angle))

    def execute(self):
        states = self.swerve_kinematics.toSwerveModuleStates(self.speeds)
        self.swerve_kinematics.normalizeWheelSpeeds(states, self.MAX_SPEED)

        if not self.using_joystick:
            states = self.raw_states

        state: SwerveModuleState
        module: SwerveModule
        for module, state in zip(self.modules, states):
            module.speed = state.speed
            module.angle = state.angle
            module.execute()
