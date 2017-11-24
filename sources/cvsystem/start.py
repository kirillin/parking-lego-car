#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import cv2
from camera_trajectory import Camera

TRAJECTORY_FILE = 'cv_trajectory.txt'


if __name__ == '__main__':

    camera = Camera(0)
    camera.init()

    trajectory = {'x': [], 'y': [], 'theta': []}

    file_trajectory = open(TRAJECTORY_FILE, 'w')
    blank_image = np.zeros_like(camera.frame)  # for real-time trajectory
    while True:
        # finding red and blue markers on a frame
        markers = camera.get_markers()

        if len(markers) == 2:
            point_start, point_end = [], []
            for marker in markers:
                cv2.drawContours(camera.frame, [marker.cnt], -1, marker.color, 5)
                cv2.circle(camera.frame, (marker.cx, marker.cy), 3, marker.color, 5)
                if marker.color[2] > 100:   # if red marker
                    point_end = [marker.cx, marker.cy]
                elif marker.color[2] < 90:  # if blue marker
                    point_start = [marker.cx, marker.cy]

            if len(point_start) > 1 and len(point_end) > 1: # if and red and blue found in one moment
                cv2.line(camera.frame, (point_start[0], point_start[1]), (point_end[0], point_end[1]), (0, 0, 0), 1)

                robot_vector = [point_end[0] - point_start[0], point_end[1] - point_start[1]]
                theta = np.arctan2(robot_vector[1], robot_vector[0])    # orientation of robot

                trajectory['x'].append(Camera.px2m(point_start[0]))
                trajectory['y'].append(Camera.px2m(point_start[1]))
                trajectory['theta'].append(theta)

                file_trajectory.write("{} {} {}\n".format(Camera.px2m(point_start[0]), Camera.px2m(point_start[1]), theta))

                Camera.draw_traj_real_time('blue', blank_image, trajectory)

                # small visualization for orientation of the robot in the top left corner (yellow color)
                cv2.line(camera.frame, (50, 50), (53+int(50*np.cos(theta)), 53+int(50*np.sin(theta))), (0,200, 200), 5)
                cv2.putText(camera.frame, str(theta*180/np.pi), (60, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 200), 2)

        cv2.imshow('frame', camera.frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    file_trajectory.close()

    camera.cap.release()
    cv2.destroyAllWindows()

    # ploting last frame to matplotlib scaled from px to meters
    plt.imshow(cv2.cvtColor(camera.frame, cv2.COLOR_BGR2RGB), interpolation='none',
               extent=[0, Camera.px2m(camera.frame.shape[1]), Camera.px2m(camera.frame.shape[0]),0])
    # plt.plot(trajectory['x'], trajectory['y'], '-', color='red')

    Camera.draw_arrows(plt, camera.frame, trajectory['x'], trajectory['y'], 'red', trajectory['theta'])
    plt.show()