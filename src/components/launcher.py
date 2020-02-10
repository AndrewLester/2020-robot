from common.rev import CANSparkMax, ControlType
from magicbot import will_reset_to
import wpilib
from wpilib.controllers import PIDController


class Launcher:
    launcher_motors: wpilib.SpeedControllerGroup
    launcher_solenoid: wpilib.Solenoid
    launcher_encoder: wpilib.Encoder

    speed = will_reset_to(0)
    decimal = will_reset_to(0)
    control_velocity = will_reset_to(False)
    shoot = will_reset_to(False)

    RPM_KP = ntproperty('/components/launcher/rpm_kp', 0)
    RPM_KI = ntproperty('/components/launcher/rpm_ki', 0)
    RPM_KD = ntproperty('/components/launcher/rpm_kd', 0)

    def setup(self):
        self.rpm_controller = PIDController(self.RPM_KP, self.RPM_KI, self.RPM_KD)

    def setVelocity(self, speed):
        self.speed = speed
        self.control_velocity = True
        # TODO: make dictionary(?) that lets us find speed of motor depending on distance

    def setPercentOutput(self, decimal):
        self.decimal = decimal

    # def getSpeed(self):
        # return self.encoder.getVelocity()

    def fire(self):
        self.shoot = True

    def execute(self):
        if self.control_velocity:
            self.rpm_controller.setP(self.RPM_KP)
            self.rpm_controller.setI(self.RPM_KI)
            self.rpm_controller.setD(self.RPM_KD)
            self.launcher_motor.set(self.rpm_controller.calculate(self.launcher_encoder.getRate(), self.speed))
        else:
            self.launcher_motors.set(self.decimal)

        self.launcher_solenoid.set(self.shoot)
