

class LegoCar:

    MAX_TAN_PHI = 0.36
    R = 0.0432 / 2          # wheel radius
    L = 0.21 - R            # wheelbase
    D = 0.112               # distance between the axes of rotation of the front wheels

    # Ultrasonic Sensors positions respect of the middle of the rear axle
    US_FRONT = (L, 0.07)
    US_BACK = (0, 0.07)
