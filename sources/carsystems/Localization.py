from math import sin, cos, tan


class Localization:

    def __init__(self, car):
        self.__car = car
        self.__x = 0
        self.__y = 0

    def getData(self, theta, motor_speed, dt):

        # robot's velocity
        v = motor_speed * self.__car.R

        # robot's linear velocities
        vx = v * cos(theta)
        vy = v * sin(theta)

        # robot's coordinates
        self.__x += vx * dt
        self.__y += vy * dt

        return self.__x, self.__y, vx, vy
