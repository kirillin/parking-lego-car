from ev3dev.auto import *
import time
from sources.carsystems.Mapping import Mapping

halt = Button()
front = UltrasonicSensor('in2')
rear = UltrasonicSensor('in3')

rear_motor = LargeMotor(OUTPUT_A)

fileus = open('us.txt', 'w')

start_time = time.time()
while True:
    try:
        t = time.time() - start_time
        print(t)

        f = front.distance_centimeters * 0.01
        r = rear.distance_centimeters * 0.01

        fileus.write("{} {} {}\n".format(t, f, r))

        rear_motor.run_direct(duty_cycle_sp=30)

        time.sleep(0.05)
        if t > 40:
            break
        if halt.enter:
            break
    except KeyboardInterrupt:
        break

rear_motor.duty_cycle_sp = 0

fileus.close()
