import rospy
from ardrone_autonomy.msg import Navdata
from multiprocessing import Process, Value
import cv2
import numpy as np
from scipy import ndimage
import eulerangles
import matplotlib.pyplot as plt
import libardrone

def dewarp_img(img, alpha, beta, gamma, h, w):

    # alpha: pitch
    # beta: roll
    # gamma: yaw

    f = 1000

    dx = 0
    dy = h / 2
    dz = 1000
    
    # Convert degrees to radians
    alpha = np.deg2rad(alpha)
    beta = np.deg2rad(beta)
    gamma = np.deg2rad(gamma)
    
    # Projection 2D -> 3D
    A1 = np.matrix(
        [[1, 0, -w / 2],
         [0, 1, -h / 2],
         [0, 0, 0],
         [0, 0, 1]])

    RX = np.matrix(
        [[1,             0,               0, 0],
        [0,  np.cos(alpha), - np.sin(alpha), 0],
        [0, np.sin(alpha),    np.cos(alpha), 0],
        [0,             0,             0,    1]])
    

    RY = np.matrix(
        [[np.cos(beta), 0, - np.sin(beta), 0],
        [0,            1,              0, 0],
        [np.sin(beta), 0, np.cos(beta), 0],
        [0,             0,             0, 1]])


    RZ = np.matrix(
        [[np.cos(gamma), - np.sin(gamma), 0, 0],
        [np.sin(gamma),    np.cos(gamma), 0, 0],
        [0,                            0, 1, 0],
        [0,                            0, 0, 1]])

    # Composed matrix
    R = RX * RY * RZ

    # Translation matrix
    T = np.matrix([[1, 0, 0, dx],
                   [0, 1, 0, dy],
                   [0, 0, 1, dz],
                   [0, 0, 0,  1]])

    # Translation vector
    tvec = np.array([[dx], [dy], [dz], [1]])

    tvec = -R * tvec

    T_new = np.matrix([[1, 0, 0, tvec[0]],
                       [0, 1, 0, tvec[1]],
                       [0, 0, 1, tvec[2]],
                       [0, 0, 0,      1]])

    # 3D -> 2D matrix
    A2 = np.matrix(
              [[f, 0, w/2, 0],
              [0,  f, h/2, 0],
              [0,  0,   1, 0]])

    trans = A2 * (T_new * (R * A1))

    result = cv2.warpPerspective(img, trans, (h * 2, w * 2))

    return result


def derotator():
    cap = cv2.VideoCapture(0)

    startvideo = True
    video_waiting = False
    use_drone_cam = False

    drone = libardrone.ARDrone(is_ar_drone_2=True)
    drone.set_camera_view(False)
    
    while True:

        if use_drone_cam:
            try:
                cv2.imshow("Drone camera", cv2.cvtColor(drone.get_image(), cv2.COLOR_BGR2RGB))
                cv2.waitKey(1)
            except:
                if not video_waiting:
                    print("Video will display when ready")
                    video_waiting = True
                    pass

        else:
            ret, frame = cap.read()

            color = True

            if color:
                h, w, c = frame.shape
            else:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                h, w = frame.shape

            nav = drone.get_navdata()
            beta = nav[0]['phi']
            alpha = nav[0]['theta']
            gamma = nav[0]['psi']

            derotated_frame = dewarp_img(frame, alpha, beta, gamma, h, w)

            cv2.imshow("Derotated Image", derotated_frame)
            cv2.waitKey(1)


if __name__ == "__main__":

    p1 = Process(target=derotator)
    p1.start()
    p1.join()

    
