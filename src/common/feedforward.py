"""
----------------------------------------------------------------------------
 Copyright (c) 2019 FIRST. All Rights Reserved.                             
 Open Source Software - may be modified and shared by FRC teams. The code   
 must be accompanied by the FIRST BSD license file in the root directory of 
 the project.                                                               
----------------------------------------------------------------------------
"""

import math


class SimpleMotorFeedforward:
    """
    A helper class that computes feedforward outputs for a simple permanent-magnet DC motor.
    """

    def __init__(self, ks: float, kv: float, ka: float=0):
        """
        Creates a new SimpleMotorFeedforward with the specified gains.  Units of the gain values
        ill dictate units of the computed feedforward.
        """
        self.ks = ks
        self.kv = kv
        self.ka = ka

    def _sign(self, value: float):
        return math.copysign(1 if value != 0 else 0, value)

    def calculate(self, velocity: float, acceleration: float=0) -> float:
        """
        Calculates the feedforward from the gains and velocity setpoint (acceleration defaults to
        zero).
        """
        return self.ks * self._sign(velocity) + self.kv * velocity + self.ka * acceleration

    def maxAchievableVelocity(self, maxVoltage: float, acceleration: float) -> float:
        """
        Calculates the maximum achievable velocity given a maximum voltage supply
        and an acceleration.  Useful for ensuring that velocity and
        acceleration constraints for a trapezoidal profile are simultaneously
        achievable - enter the acceleration constraint, and this will give you
        a simultaneously-achievable velocity constraint.
   
        :param maxVoltage: The maximum voltage that can be supplied to the motor.
        :param acceleration: The acceleration of the motor.
        :return: The maximum possible velocity at the given acceleration.
        """
        # Assume max velocity is positive
        return (maxVoltage - self.ks - acceleration * self.ka) / self.kv

    def minAchievableVelocity(self, maxVoltage: float, acceleration: float) -> float:
        """
        Calculates the minimum achievable velocity given a maximum voltage supply
        and an acceleration.  Useful for ensuring that velocity and
        acceleration constraints for a trapezoidal profile are simultaneously
        achievable - enter the acceleration constraint, and this will give you
        a simultaneously-achievable velocity constraint.
   
        :param maxVoltage: The maximum voltage that can be supplied to the motor.
        :param acceleration: The acceleration of the motor.
        :return: The minimum possible velocity at the given acceleration.
        """
        # Assume min velocity is negative, ks flips sign
        return (-maxVoltage + self.ks - acceleration * self.ka) / self.kv

    def maxAchievableAcceleration(self, maxVoltage: float, velocity: float) -> float:
        """
        Calculates the maximum achievable acceleration given a maximum voltage
        supply and a velocity. Useful for ensuring that velocity and
        acceleration constraints for a trapezoidal profile are simultaneously
        achievable - enter the velocity constraint, and this will give you
        a simultaneously-achievable acceleration constraint.
   
        :param maxVoltage: The maximum voltage that can be supplied to the motor.
        :param velocity: The velocity of the motor.
        :return: The maximum possible acceleration at the given velocity.
        """
        return (maxVoltage - self.ks * self._sign(velocity) - velocity * self.kv) / self.ka

    def minAchievableAcceleration(self, maxVoltage: float, velocity: float) -> float:
        """
        Calculates the minimum achievable acceleration given a maximum voltage
        supply and a velocity. Useful for ensuring that velocity and
        acceleration constraints for a trapezoidal profile are simultaneously
        achievable - enter the velocity constraint, and this will give you
        a simultaneously-achievable acceleration constraint.
   
        :param maxVoltage: The maximum voltage that can be supplied to the motor.
        :param acceleration: The velocity of the motor.
        :return: The minimum possible acceleration at the given acceleration.
        """
        return self.maxAchievableAcceleration(-maxVoltage, velocity)
