"""
----------------------------------------------------------------------------
 Copyright (c) 2018-2019 FIRST. All Rights Reserved.
 Open Source Software - may be modified and shared by FRC teams. The code
 must be accompanied by the FIRST BSD license file in the root directory of
 the project.
----------------------------------------------------------------------------
"""


class PIDController:
    """
    Implements a PID control loop.
    """

    maximumIntegral = 1.0
    minimumIntegral = -1.0
    maximumInput = 0
    minimumInput = 0
    inputRange = 0
    continuous = False
    positionError = 0
    velocityError = 0
    previousError = 0
    totalError = 0
    positionTolerance = 0.05
    velocityTolerance = float('inf')
    setpoint = 0

    def __init__(self, Kp: float, Ki: float, Kd: float, period: float = 0.02):
        """
        Allocates a PIDController with the given constants for Kp, Ki, and Kd and a default period of
        0.02 seconds.
        """
        self.P = Kp
        self.I = Ki
        self.D = Kd
        self.period = period

    def setPID(self, Kp: float, Ki: float, Kd: float):
        self.P = Kp
        self.I = Ki
        self.D = Kd

    def setP(self, Kp: float):
        self.P = Kp

    def setI(self, Ki: float):
        self.I = Ki

    def setD(self, Kd: float):
        self.D = Kd

    def getP(self):
        return self.P

    def getI(self):
        return self.I

    def getD(self):
        return self.D

    def getPeriod(self):
        return self.period

    def setSetpoint(self, setpoint: float):
        if self.maximumInput > self.minimumInput:
            self.setpoint = max(min(setpoint, self.maximumInput), self.minimumInput)
        else:
            self.setpoint = setpoint

    def getSetpoint(self):
        return self.setpoint

    def atSetpoint(self):
        return (abs(self.positionError) < self.positionTolerance
                and abs(self.velocityError) < self.velocityTolerance)

    def enableContinuousInput(self, minimumInput, maximumInput):
        self.continuous = True
        self.setInputRange(minimumInput, maximumInput)

    def disableContinuousInput(self):
        self.continuous = False

    def setIntegratorRange(self, minimumIntegral, maximumIntegral):
        self.minimumIntegral = minimumIntegral
        self.maximumIntegral = maximumIntegral

    def setTolerance(self, positionTolerance, velocityTolerance=float('inf')):
        self.positionTolerance = positionTolerance
        self.velocityTolerance = velocityTolerance

    def getPositionError(self):
        return self.getContinuousError(self.positionError)

    def getVelocityError(self):
        return self.velocityError

    def calculate(self, measurement, setpoint=None):
        if setpoint is not None:
            self.setSetpoint(setpoint)

        self.previousError = self.positionError
        self.positionError = self.getContinuousError(self.setpoint - measurement)
        self.velocityError = (self.positionError - self.previousError) / self.period

        if self.I != 0:
            self.totalError = max(min(self.totalError + self.positionError * self.period, self.maximumIntegral / self.I), self.minimumIntegral / self.I)

        return self.P * self.positionError + self.I * self.totalError + self.D * self.velocityError

    def reset(self):
        self.previousError = 0
        self.totalError = 0

    def getContinuousError(self, error):
        if self.continuous and self.inputRange > 0:
            error %= self.inputRange
            if abs(error) > self.inputRange / 2:
                if error > 0:
                    return error - self.inputRange
                else:
                    return error + self.inputRange
        return error

    def setInputRange(self, minimumInput, maximumInput):
        self.minimumInput = minimumInput
        self.maximumInput = maximumInput
        self.inputRange = maximumInput - minimumInput

        if self.maximumInput > self.minimumInput:
            self.setpoint = max(min(self.setpoint, self.maximumInput), self.minimumInput)
