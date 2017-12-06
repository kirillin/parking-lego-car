#!/usr/bin/env python3
import math


class TrajectroryController:

    Kp_1 = 5.76
    Kd_1 = 4.8
    Kp_2 = 5.76
    Kd_2 = 4.8

    def __init__(self, xi0):
        self.__xi = xi0
        self.error_file = open('error.txt', 'w')

    def __delete__(self):
        self.error_file.close()

    def getControls(self, trajectory_point, x_cur, y_cur, dx_cur, dy_cur, theta, dt):
        x_r, dx_r, ddx_r, y_r, dy_r, ddy_r = trajectory_point.getPoint()

        # simple controller
        err_x = x_r - x_cur
        err_dx = dx_r - dx_cur
        u1 = ddx_r + self.Kp_1 * err_x + self.Kd_1 * err_dx

        err_y = y_r - y_cur
        err_dy = dy_r - dy_cur
        u2 = ddy_r + self.Kp_2 * err_y + self.Kd_2 * err_dy

        # converter
        dot_xi = u1 * math.cos(theta) + u2 * math.sin(theta)
        self.__xi += dot_xi * dt
        if self.__xi > 0.35:
            self.__xi = 0.35
        if self.__xi < -0.35:
            self.__xi = -0.35

        self.error_file.write("{0} {1} {2} {3} {4} {5} {6}\n".format(err_x, err_y, err_dx, err_dy, self.__xi, u1, u2))

        v_des = self.__xi
        if 0 <= self.__xi <= 0.01:
            self.__xi = 0.01
        elif -0.01 <= self.__xi < 0:
            self.__xi = -0.01

        omega_des = (- u1 * math.sin(theta) + u2 * math.cos(theta)) / self.__xi

        return v_des, omega_des
