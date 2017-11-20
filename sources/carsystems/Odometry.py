from math import sin, cos, tan


class Odometry:
    """
        Class provides simple odometry for car
        with Ackermann steering geometry.

        Usage:
        odom = Odometry(Robots.LegoCar)
        x, y, theta, vx, vy = odom.getData(angle_of_front_left_wheel, motor_speed, dt)
    """
    def __init__(self, car):
        self.__car = car
        self.__theta_old = 0
        self.__x = 0
        self.__y = 0

    def getData(self, phi_1, motor_speed, dt):
        """
        Returns odom.
        :param phi_1: angle of front left wheel
        :param motor_speed: speed of rear motor
        :param dt:  delta time
        :return: x, y, theta, dx, dy
        """
        # robot's velocity
        v = motor_speed * self.__car.R
        tg_phi = self.__car.L * tan(phi_1) / \
                 (self.__car.L - self.__car.D * tan(phi_1) / 2)
        # TODO: computations for angle right wheel (phi_2)

        # robot's angle
        theta = self.__theta_old + v / self.__car.L * tg_phi * dt
        self.__theta_old = theta

        # robot's linear velocities
        vx = v * cos(theta)
        vy = v * sin(theta)

        # robot's coordinates
        self.__x += vx * dt
        self.__y += vy * dt

        return self.__x, self.__y, theta, vx, vy
