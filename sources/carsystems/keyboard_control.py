#!/usr/bin/env python3
import fcntl, os, sys, select, termios, time
from math import pi, copysign

import ev3dev.ev3 as ev3

from libs.Robots import LegoCar
from libs.VelocityController import VelocityController

from libs.PID import PID


moveBindings = {
                'i': (1, 0),
                ',': (-1, 0),
                'u': (1, 1),
                'o': (1, -1),
                'm': (-1, 1),
                '.': (-1, -1),
                'j': (1, 2),
                'l': (1, -2),
                'k': (0, 0)
                }

speedBindings = {
                'q': (1.1, 1.1),
                'z': (.9, .9),
                }


class KeyReader:
    '''
    Read keypresses one at a time, without waiting for a newline.
    echo: should characters be echoed?
    block: should we block for each character, or return immediately?
           (If !block, we'll return None if nothing is available to read.)
    '''
    def __init__(self, echo=False, block=True):
        '''Put the terminal into cbreak and noecho mode.'''
        self.fd = sys.stdin.fileno()

        self.block = block

        self.oldterm = termios.tcgetattr(self.fd)
        self.oldflags = fcntl.fcntl(self.fd, fcntl.F_GETFL)

        # Sad hack: when the destructor __del__ is called,
        # the fcntl module may already be unloaded, so we can no longer
        # call fcntl.fcntl() to set the terminal back to normal.
        # So just in case, store a reference to the fcntl module,
        # and also to termios (though I haven't yet seen a case
        # where termios was gone -- for some reason it's just fnctl).
        # The idea of keeping references to the modules comes from
        # http://bugs.python.org/issue5099
        # though I don't know if it'll solve the problem completely.
        self.fcntl = fcntl
        self.termios = termios

        newattr = termios.tcgetattr(self.fd)
        # tcgetattr returns: [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
        # where cc is a list of the tty special characters (length-1 strings)
        # except for cc[termios.VMIN] and cc[termios.VTIME] which are ints.
        self.cc_save = newattr[6]
        newattr[3] = newattr[3] & ~termios.ICANON
        if not echo:
            newattr[3] = newattr[3] & ~termios.ECHO

        if block and False:
            # VMIN and VTIME are supposed to let us do blocking reads:
            # VMIN is the minimum number of characters before it will return,
            # VTIME is how long it will wait if for characters < VMIN.
            # This is documented in man termios.
            # However, it doesn't work in python!
            # In Python, read() never returns in non-canonical mode;
            # even typing a newline doesn't help.
            cc = self.cc_save[:]   # Make a copy so we can restore VMIN, VTIME
            cc[termios.VMIN] = 1
            cc[termios.VTIME] = 0
            newattr[6] = cc
        else:
            # Put stdin into non-blocking mode.
            # We need to do this even if we're blocking, see above.
            fcntl.fcntl(self.fd, fcntl.F_SETFL, self.oldflags | os.O_NONBLOCK)

        termios.tcsetattr(self.fd, termios.TCSANOW, newattr)

    def __del__(self):
        '''Reset the terminal before exiting the program.'''
        self.termios.tcsetattr(self.fd, self.termios.TCSAFLUSH, self.oldterm)
        self.fcntl.fcntl(self.fd, self.fcntl.F_SETFL, self.oldflags)

    def getch(self):
        '''Read keyboard input, returning a string.
           Note that one key may result in a string of more than one character,
           e.g. arrow keys that send escape sequences.
           There may also be multiple keystrokes queued up since the last read.
           This function, sadly, cannot read special characters like VolumeUp.
           They don't show up in ordinary CLI reads -- you have to be in
           a window system like X to get those special keycodes.
        '''
        # Since we can't use the normal cbreak read from python,
        # use select to see if there's anything there:
        if self.block:
            inp, outp, err = select.select([sys.stdin], [], [])
        try:
            return sys.stdin.read(1)
        except IOError:
            # print "IOError:", e
            return None


class KeyboardTeleop:

    def __init__(self):
        self.keyreader = KeyReader(echo=False, block=False)

        # initialization LegoCar
        self.halt = ev3.Button()
        self.gyro_sensor = ev3.GyroSensor('in1')
        self.sensor_us_front = ev3.UltrasonicSensor('in2')
        self.sensor_us_rear = ev3.UltrasonicSensor('in3')

        self.rear_motor = ev3.LargeMotor(ev3.OUTPUT_A)
        self.steer_motor = ev3.LargeMotor(ev3.OUTPUT_B)

        self.rear_motor.reset()
        self.steer_motor.reset()

        self.velocity_controller = VelocityController(LegoCar, 0, 0)

        # initialization of keyboard cupturing
        self.speed = 70
        self.turn = 15

        self.pid_phi = PID(100.0, 500.0, 5.0, 100, -100)

    def startWhellSteering(self):
        self.speed = 70
        self.turn = 15

        print("The wheel control was started!")
        print(self.vels(self.speed, self.turn))

        while True:
            try:
                key = self.keyreader.getch()
                if key in moveBindings.keys():
                    command = moveBindings[key]
                    x = command[0]
                    th = command[1]

                    speed_des = x * self.speed
                    omega_des = th * self.turn

                    self.rear_motor.run_direct(duty_cycle_sp=speed_des)

                    # desired front wheels angle
                    self.turnWheel(omega_des)

                if self.halt.enter:
                    break

            except KeyboardInterrupt:
                break

        self.keyreader = None
        self.rear_motor.duty_cycle_sp = 0
        self.steer_motor.duty_cycle_sp = 0
        raise SystemExit

    def startSpeedSteering(self):
        """BAD WORK"""
        self.speed = 0.3
        self.turn = 0.2

        print("The speed control was started!")
        print(self.vels(self.speed, self.turn))

        start_time = time.time()
        t = time.time() - start_time
        while True:
            try:
                last_time = t
                t = time.time() - start_time
                dt = t - last_time  # invalid within first iteration but it's ok

                theta, omega = [-x for x in self.gyro_sensor.rate_and_angle]  # !!! returns ANGLE AND RATE :)

                key = self.keyreader.getch()
                if key in moveBindings.keys():
                    command = moveBindings[key]
                    x = command[0]
                    th = command[1]

                    v, phi = self.velocity_controller.getTargetVelocities()

                    # :) need for speed and wheel control compatibility
                    if x < 0:
                        omega_des = - th * self.turn
                    else:
                        omega_des = th * self.turn

                    if key in ['j', 'l']:
                        speed_des = x * v
                    else:
                        speed_des = x * self.speed

                    self.velocity_controller.setTargetVelocities(speed_des, omega_des)
                    print(self.vels(speed_des, omega_des))

                u_v, u_phi = self.velocity_controller.getControls(self.rear_motor.speed * pi / 180.0,
                                                                  self.steer_motor.position * pi / 180,
                                                                  omega * pi / 180, dt)
                self.rear_motor.run_direct(duty_cycle_sp=u_v)
                self.steer_motor.run_direct(duty_cycle_sp=u_phi)

                if self.halt.enter:
                    break

            except KeyboardInterrupt:
                break

        self.keyreader = None
        self.rear_motor.duty_cycle_sp = 0
        self.steer_motor.duty_cycle_sp = 0
        raise SystemExit

    def vels(self, speed, turn):
        return "Currently:\tspeed %s\tturn %s " % (speed, turn)

    def turnWheel(self, phi_desired):
        start_time = time.time()

        if abs(phi_desired) > 15:
            phi_desired = copysign(1, phi_desired) * 15

        t = 0
        while t < 1.0:
            last_time = t
            t = time.time() - start_time
            dt = t - last_time  # invalid within first iteration but it's ok

            phi_current = self.steer_motor.position
            error_phi = (phi_desired - phi_current) * pi / 180

            u_phi = self.pid_phi.getControl(error_phi, dt)
            self.steer_motor.run_direct(duty_cycle_sp=u_phi)


if __name__ == '__main__':
    kb_teleop = KeyboardTeleop()
    kb_teleop.startWhellSteering()
    # kb_teleop.startSpeedSteering()
