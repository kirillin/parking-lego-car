import matplotlib.pyplot as plt
import numpy as np

import cv2

from camera_trajectory import Camera

RED = (0, 0, 255)
BLUE = (255, 0, 0)
TRAJ_FILE = 'traj.txt'

if __name__ == '__main__':

    camera = Camera(1)

    red_traj = {'x': [], 'y': []}
    blue_traj = {'x': [], 'y': []}
    traj = {'x': [], 'y': []}

    file_traj = open(TRAJ_FILE, 'w')

    while True:
        markers = camera.get_markers()

        if len(markers) > 0:
            for m in markers:
                # red_traj['x'].append(rm.cx)
                # red_traj['y'].append(rm.cy)
                traj['x'].append(Camera.px2m(m.cx))
                traj['y'].append(Camera.px2m(m.cy))
                cv2.drawContours(camera.frame, [m.cnt], -1, m.color, 2)
                cv2.circle(camera.frame, (m.cx, m.cy), 3, m.color, 2)

        blank_image = np.zeros_like(camera.frame)  # for real-time trajectory
        # Camera.draw_traj_real_time('red', blank_image, red_traj)
        # Camera.draw_traj_real_time('blue', blank_image, blue_traj)

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
