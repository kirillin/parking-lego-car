import matplotlib.pyplot as plt
import numpy as np

import cv2

from camera_trajectory import Camera

RED = (0, 0, 255)
BLUE = (255, 0, 0)
TRAJ_FILE = 'traj.txt'

class Point:

    def __init__(cx, cy, theta):
        self.cx = cx
        self.cy = cy
        self.cnt = theta


class Marker:

    def __init__(self, cx, cy, cnt, color):
        self.cx = cx
        self.cy = cy
        self.cnt = cnt
        self.color = color

class Robot:

    def __init__(self, m_b=None, m_r=None):
        self.m_b = m_b
        self.m_r = m_r

    def setBlueMarker(self, m):
	self.m_b = m

    def setRedMarker(self, m):
	self.m_r = m


def unitVector(vector):
    return vector / np.linalg.norm(vector)


def angleBetween(v1, v2):
    v1_u = unitVector(v1)
    v2_u = unitVector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


if __name__ == '__main__':

    camera = Camera(0)

    red_traj = {'x': [], 'y': []}
    blue_traj = {'x': [], 'y': []}
    traj = {'x': [], 'y': []}

    points = []

    file_traj = open(TRAJ_FILE, 'w')

    while True:

        markers = camera.get_markers()

	if len(markers) >= 2:
		robot = Robot()
		cnt = 0
		for m in markers:
		    if m.color[2] > 111:
			robot.setRedMarker(m)
			cnt += 1
		    elif m.color[2] < 100:
		        robot.setBlueMarker(m)
			cnt += 1
		print(cnt)
		#if cnt >= 2:

			#ox_vector = (1, 0)
			#robot_vector = (robot.m_r.cx - robot.m_b.cx, robot.m_r.cy - robot.m_b.cy)
			#theta = angleBetween(ox_vector, robot_vector)

			#points.append(robot.m_b.cx, robot.m_b.cy)

        if len(markers) > 0:
            for m in markers:
                traj['x'].append(Camera.px2m(m.cx))
                traj['y'].append(Camera.px2m(m.cy))
                cv2.drawContours(camera.frame, [m.cnt], -1, m.color, 5)
                cv2.circle(camera.frame, (m.cx, m.cy), 3, m.color, 5)
		
	#camera.frame = cv2.line(camera.frame, (camera.frame.shape[1]/2, camera.frame.shape[0]/2), (x*cos(theta), y*sin(theta)), (0,255, 0), 2)	

        # blank_image = np.zeros_like(camera.frame)  # for real-time trajectory
        # Camera.draw_traj_real_time('red', blank_image, red_traj)
        # Camera.draw_traj_real_time('blue', blank_image, blue_traj)

	camera.frame = cv2.line(camera.frame, (0, camera.frame.shape[0]/2), (camera.frame.shape[1], camera.frame.shape[0]/2), (0,0,255), 1)
	camera.frame = cv2.line(camera.frame, (camera.frame.shape[1]/2, 0), (camera.frame.shape[1]/2, camera.frame.shape[0]), (0,0,255), 1)


        cv2.imshow('frame', camera.frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.cap.release()
    cv2.destroyAllWindows()

    file_traj.close()

    # ploting trajectories to matplotlib
    plt.imshow(cv2.cvtColor(camera.frame, cv2.COLOR_BGR2RGB), interpolation='none',
               extent=[0, Camera.px2m(camera.frame.shape[1]), Camera.px2m(camera.frame.shape[0]),0])

    # plt.plot(traj['x'], traj['y'], '-', color='red')


    # Camera.draw_arrows(plt, camera.frame, x, y, color='blue')
    Camera.draw_arrows(plt, camera.frame, traj['x'], traj['y'], color='blue')
    # Camera.draw_arrows(plt, camera.frame, blue_traj['x'], blue_traj['y'], color='blue')
    # Camera.draw_arrows(plt, camera.frame, red_traj['x'], red_traj['y'], color='red')

    plt.show()


    # plt.plot(blue_traj['x'], blue_traj['y'], '-', color='blue')
    # plt.plot(red_traj['x'], red_traj['y'], '-o', color='red')
    #
