from Point import TrajectoryPoint


class Trajectory:

    def __init__(self):
        self.trajectory = []

    def addTrajectoryPoint(self, trajectory_point):
        self.trajectory.append(trajectory_point)

    def getTrajectory(self):
        return self.trajectory