#!/usr/bin/env python3
from math import pi
from ev3dev.ev3 import Sound, Button, GyroSensor, UltrasonicSensor, LargeMotor, OUTPUT_A, OUTPUT_B

from Clock import Clock
from libs.PID import PID

from libs.Localization import Localization
from libs.VelocityController import VelocityController

from libs.Point import Point
from libs.TrajectoryStuff import OnlyPoint, StraightLine
from libs.TrajectoryControllers import ControllerWithCoordinateTransform

from libs.Mapping import Mapping


class LegoCar:

    # Parameters of the car
    MAX_PHI = 0.61
    R = 0.0432 / 2          # wheel radius
    L = 0.21 - R            # wheelbase
    D = 0.112               # distance between the axes of rotation of the front wheels
    # Ultrasonic Sensors positions respect of the middle of the rear axle
    US_FRONT = (L, 0.06)
    US_REAR = (-0.015, 0.06)

    def __init__(self, port_motor_rear=OUTPUT_A, port_motor_steer=OUTPUT_B,
                 port_sensor_gyro='in1', port_sensor_us_rear='in2',
                 port_sensor_us_front='in3'):
        # initialization of devices
        self.__button_halt = Button()

        self.__motor_rear = LargeMotor(port_motor_rear)
        self.__motor_steer = LargeMotor(port_motor_steer)

        self.__sensor_gyro = GyroSensor(port_sensor_gyro)
        self.__sensor_us_rear = UltrasonicSensor(port_sensor_us_rear)
        self.__sensor_us_front = UltrasonicSensor(port_sensor_us_front)

        self.__localization = Localization(self)
        self.__velocity_controller = VelocityController(self, 0, 0)

        # NOTE: possible using other controllers. Change only here!
        self.__trajectory_controller = ControllerWithCoordinateTransform()

        self.reset()
        Sound.speak('Car initialization complete!').wait()

    def reset(self):
        # reset encoders of motors
        self.__motor_rear.reset()
        self.__motor_steer.reset()
        # reset gyro sensor. The robot needs to be perfectly still!
        self.__sensor_gyro.mode = self.__sensor_gyro.modes[1]
        self.__sensor_gyro.mode = self.__sensor_gyro.modes[0]

    def getDevice(self, what):
        if what in ['us_sensors']:
            return self.__sensor_us_front, self.__sensor_us_rear

    def turnWheel(self, phi_desired):
        pid_phi = PID(100.0, 500.0, 5.0, 100, -100)
        t = 0
        clock = Clock()
        while t < 0.3:  # FIXME: seems that need steady state condition
            t, dt = clock.getTandDT()

            phi_current = self.__motor_steer.position
            error_phi = (phi_desired - phi_current) * pi / 180

            u_phi = pid_phi.getControl(error_phi, dt)
            self.__motor_steer.run_direct(duty_cycle_sp=u_phi)

    def move(self, vel_linear=None, vel_angular=None, trajectory=None, mapping=None):
        """
            Examples:
                mode_1. move(trajectory=given_trajectory) --- following given trajectory;
                mode_2. move(vel_linear=0.3, vel_angular=0.3) --- moving with given velocities.
        """
        # initialization for current mode
        if vel_linear and vel_angular not in [None]:
            self.__velocity_controller.setTargetVelocities(vel_linear, vel_angular)

        clock = Clock()
        while True:
            try:
                t, dt = clock.getTandDT()
                theta, omega = [-x for x in self.__sensor_gyro.rate_and_angle]  # !!! returns ANGLE AND RATE :)

                # WONTFIX: In the mode of move is only velocities it is idle
                x, y, dx, dy = self.__localization.getData(theta * pi / 180, self.__motor_rear.speed * pi / 180, dt)

                if trajectory is not None:
                    point = trajectory.getCoordinates(t)
                    v_des, omega_des = self.__trajectory_controller.getControls(point, x, y, dx, dy, theta * pi / 180, dt)
                    self.__velocity_controller.setTargetVelocities(v_des, omega_des)

                u_v, u_phi = self.__velocity_controller.getControls(self.__motor_rear.speed * pi / 180.0,
                                                                    self.__motor_steer.position * pi / 180,
                                                                    omega * pi / 180, dt)

                if mapping is not None:
                    mapping.updateMap(x, y, theta * pi / 180)

                self.__motor_rear.run_direct(duty_cycle_sp=u_v)
                self.__motor_steer.run_direct(duty_cycle_sp=u_phi)

                if self.__button_halt.enter:
                    break
            except KeyboardInterrupt:
                break
        # off motors
        self.__motor_rear.duty_cycle_sp = 0
        self.__motor_steer.duty_cycle_sp = 0
        raise SystemExit


if __name__ == '__main__':
    car = LegoCar()

    # turn forward wheels on the 10 degrees
    car.turnWheel(10)

    # move with desired velocities
    car.move(vel_linear=0.3, vel_angular=0.3)

    # follow trajectory
    trajectory_line = StraightLine(Point(0, 1), Point(4, 1), 0.2)
    car.move(trajectory=trajectory_line)

    trajectory_point = OnlyPoint(Point(0, 0))
    car.move(trajectory=trajectory_point)

    # moving straight and mapping
    susf, susr = car.getDevice('us_sensors')
    mapping = Mapping(car, susf, susr)
    car.move(trajectory=trajectory_line, mapping=mapping)
    # car.move(vel_linear=0.3, vel_angular=0, mapping=mapping)  # also possible
