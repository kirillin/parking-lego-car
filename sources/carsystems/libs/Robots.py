

class LegoCar:

    MAX_PHI = 0.61
    R = 0.0432 / 2          # wheel radius
    L = 0.21 - R            # wheelbase
    D = 0.112               # distance between the axes of rotation of the front wheels

    # Ultrasonic Sensors positions respect of the middle of the rear axle
    US_FRONT = (L, 0.06)
    US_REAR = (-0.015, 0.06)
