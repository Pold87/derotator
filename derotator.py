import rospy
from ardrone_autonomy.msg import Navdata
from multiprocessing import Process, Value
import cv2
import numpy as np
from scipy import ndimage
import eulerangles

def rotate_rod(pitch, roll, yaw):
    
    # Projection 2D -> 3D
    Mat_A1 = np.matrix(
        [[1, 0, -w / 2],
         [0, 1, -h / 2],
         [0, 0, 0],
         [0, 0, 1]])


    Mat_R = np.matrix(
        [[np.cos(beta), 0, - np.sin(beta), 0],
         0, ])



def rotateImage(image, angle):
    image_center = tuple(np.array(image.shape) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape, flags=cv2.INTER_LINEAR)
    return result

def callback(data, (rot_x, rot_y, rot_z)):
    rot_x.value = data.rotX
    rot_y.value = data.rotY
    rot_z.value = data.rotZ

def listener(rot_x, rot_y, rot_z):

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/ardrone/navdata', Navdata, callback, callback_args=(rot_x, rot_y, rot_z))

    rospy.spin()

def slow_printer(rot_x, rot_y, rot_z):
    cap = cv2.VideoCapture(1)

    while True:

        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = frame.shape

        src = np.array([rot_z.value, rot_x.value, rot_y.value])

        euler_m = eulerangles.euler2mat(rot_z.value, rot_x.value, rot_y.value)

        rod_m, _ = cv2.Rodrigues(src)
        #print(dst)

        print(rot_z.value)

        M = np.dot(rod_m, euler_m)

        print(M)

        new = cv2.warpPerspective(frame, rod_m, (h, w))
        #new = rotateImage(frame, rot_z.value)
        
        cv2.imshow("Frame", new)
        cv2.waitKey(1)


if __name__ == "__main__":

    rot_x = Value('d', 0)
    rot_y = Value('d', 0)
    rot_z = Value('d', 0)

    p1 = Process(target=listener, args=(rot_x, rot_y, rot_z))
    p1.start()
    p2 = Process(target=slow_printer, args=(rot_x, rot_y, rot_z))
    p2.start()
#    p1.join()
#    p2.join()

    
