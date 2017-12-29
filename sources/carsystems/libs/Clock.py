import time


class Clock:
    """
        Clock for compute delta time
        !!! Don't forget updating before geting DT
    """

    __start_time = 0
    __current_time = 0
    __last_time = 0
    __dt = 0

    def __init__(self):
        self.__old_dt = -1  # for checking you updated -_-
        self.reset()

    def reset(self):
        self.__start_time = time.time()
        self.__current_time = 0
        self.__last_time = 0
        self.__dt = 0

    def update(self):
        self.__current_time = time.time() - self.__start_time
        self.__dt = self.__current_time - self.__last_time
        self.__last_time = self.__current_time

    def getDT(self):
        if self.__dt == self.__old_dt:
            print('Heey! Update clock!!1')
        self.__old_dt = self.__dt
        return self.__dt

    def getCurrentTime(self):
        return self.__current_time

    def getTandDT(self):
        return self.getCurrentTime(), self.getDT()

if __name__ == '__main__':
    clock = Clock()
    while True:
        clock.update()
        t = clock.getCurrentTime()
        dt = clock.getDT()
        print(t, dt)
        time.sleep(0.3)
