#!/usr/bin/env python3


class PID:

    def __init__(self, kp, ki, kd, u_max, u_min, need_limit=True):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.u_max, self.u_min = u_max, u_min
        self.need_limit = need_limit
        self.counter = 0
        self.ei = 0
        self.last_ed = 0

    def getControl(self, e, dt):

        if self.counter == 1:
            self.ei = 0.0
            ed = 0
        else:
            self.ei += e * dt
            ed = (e - self.last_ed) / dt
        self.last_ed = e

        u = self.kp * e + self.ki * self.ei + self.kd * ed

        if self.need_limit:
            if u > self.u_max:
                u = self.u_max
            elif u < self.u_min:
                u = self.u_min

        self.counter += 1
        return u
