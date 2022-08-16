#! /usr/bin/env python2

import cv2
import cv2.aruco as aruco
import numpy as np
import math, sys, time
import os
import rospy
from rxt_aruco_recognition.msg import aruco_pose
from  subprocess import Popen, PIPE
import time
from math import pow, atan2, sqrt


"""
Aruco used is 5x5
camera fram - 1980 * 1080
opencv version '3.3.1-dev'
camera module - " Logitech Business Brio Ultra HD Camera "
published topic of detected Aruco marker as list
     - Id of the Aruco
     - center_x pixel in camera frame of the detected Aruco marker
     - center_y pixel in camera frame of the detected Aruco marker
     - theta orientation or Yaw of the detected aruco marker
fixed Bug of camera before due to multiple camera its very hard to find the webcam. (e.g. device like intel realsense * 2)
"""

# Path for calibrated camera parameters. 
calib_path  = "/home/panda/ros_workspace/src/rxt_aruco_recognition/scripts/" # Change this in future if you use this folder
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')


#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0


# Checks if a matrix is a valid rotation matrix. refernce - https://stackoverflow.com/questions/53808503/how-to-test-if-a-matrix-is-a-rotation-matrix
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# refence - https://github.com/thodan/bop_toolkit/blob/53150b649467976b4f619fbffb9efe525c7e11ca/bop_toolkit_lib/transform.py#L1112
# Calculates rotation matrix to euler angles
# we need this function because to find orientation of the Aruco marker
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])



def increase_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] = v[v <= lim] + value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img


########################################################################################################################################################################################################################################################

#below is the main function which will take care of detection and publishing the point

def detect_aruco():
    while True:
        """
        Try except is introduced because we have serial communication between 2D camera and intel realsense to get the exact output of 2DCamera we use this try and except method
        """
        
        time.sleep(3)
        try:
            """
            This code will identify the correct device where the image puplished this is required by opencv. Right now it identifies the BRIO

            if you use different camera use "v4l2-ctl --list-devices"to list all the camera device from that change the if statement. eg. if you have IC98 came change the if statement to if "IC98" in find_cam:
            """
            #------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            cmd = ["/usr/bin/v4l2-ctl", "--list-devices"]
            out, err = Popen(cmd, stdout=PIPE, stderr=PIPE).communicate()
            out, err = out.strip(), err.strip()
            for l in [i.split("\n\t") for i in out.split("\n\n")]:
                find_cam = l[0].split(" ")
                if "BRIO" in find_cam:
                    frame_cam = int(l[1][-1])
            #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            fourcc = cv2.VideoWriter_fourcc('M','J','P','G') # changing the codec to MJPG because logitec brio doesn't support default codec [ Default codec is lagging and doesn't provide good fps ]
            cap = cv2.VideoCapture(frame_cam) # capturing video ouput at device 0 or maybe different [ other ports are occupied with intel realsense ]
            cap.set(cv2.CAP_PROP_FOURCC, fourcc) # applying codec to the video
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920) # describing the width of the frame
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080) # describing the length of the frame
            cap.set(cv2.CAP_PROP_FPS,30) # setting fps to the video max it gives 30fps at 1080p
            font = cv2.FONT_HERSHEY_PLAIN
            marker_size = 11.5 ##### size is in cm. change before using it.
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            break
        except:
            pass


    """
    code for publishing the acquired values to a topic
    """
    pub_aruco = rospy.Publisher('/aruco_data',aruco_pose,queue_size=10)
    rospy.init_node("aruco_detection",anonymous=True)
    data = aruco_pose()


    while(True):
        ret, frame = cap.read()
        kernel = np.array([[0, -1, 0],
                        [-1, 5,-1],
                        [0, -1, 0]])
        image_sharp = cv2.filter2D(src=frame, ddepth=-1, kernel=kernel)
        frame2 = increase_brightness(image_sharp,-30)
        gray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY) # converting colored frame into gray so we can easily identify aruco
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250) # to find the aruco we need set the size of the aruco which we are using 5x5
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters) # detcting markers
        output = aruco.drawDetectedMarkers(image_sharp, corners, ids)  # detect the aruco markers and display its aruco id.
        
        
        """
        opencv will give the value of four corners of the Aruco marker. I mean 4 edges position in the frame
        Below code will get all the Aruco markers center position 
        """
        if ids != None:
            ar_id = []
            center_x = []
            center_y = []
            theta = []

            for i in range(len(ids)):

                """
                Detailed documentation - https://drive.google.com/drive/folders/1Q7_VoCeFyUbi4Xwd0OHxyhaN1vc7QJOj?usp=sharing
                """

                x_sum = corners[i][0][0][0]+ corners[i][0][1][0]+ corners[i][0][2][0]+ corners[i][0][3][0] # used to identify the center x of the recognised Aruco marker
                y_sum = corners[i][0][0][1]+ corners[i][0][1][1]+ corners[i][0][2][1]+ corners[i][0][3][1] # used to identify the center y of the recognised Aruco marker
                x_centerPixel = (x_sum*.25) - (1980/2) # after multiply with 0.25 we get the center x
                y_centerPixel = ((y_sum*.25) - (1080/2))* -1 # after multiply with 0.25 we get the center y. -1 is multiplied because generally its inverted.
                list_x = np.array([corners[i][0][1][0],corners[i][0][2][0]]) # storing top and bottom right corners x pixel
                list_y = np.array([corners[i][0][1][1],corners[i][0][2][1]]) # storing top and bottom right corners y pixel
                ## we are storing those 2 corners because we need to find the x2 and y2 to find the theta were (x1,y1) is known which is the center pixel of the recognised marker
                pointx_theta = ((np.amax(list_x) - np.amin(list_x))/2) + np.amin(list_x)  # this will find the mid point x pixel of the 2 corners which is x2
                pointy_theta = ((np.amax(list_y) - np.amin(list_y))/2) + np.amin(list_y)  # this will find the mid point y pixel of the 2 corners which is y2
                theta_2d = int(math.degrees((atan2(pointy_theta - (y_sum*.25),pointx_theta - (x_sum*.25))) * -1))  # finding angle between 2 points. tan^-1((y2-y1)/(x2-x1)) this will give in radian then converting these values to degree.

                """
                Here we obtained the ids, center_x, center_y and theta
                """
                
                """
                refernce
                visual representation of ARUCO
                http://amroamroamro.github.io/mexopencv/matlab/cv.estimatePoseSingleMarkers.html
                """
                ret = aruco.estimatePoseSingleMarkers(corners[i], marker_size,camera_matrix, camera_distortion) # this gives array of rotational and translational vectors respective specific corners
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] # getting rotational vector and translational vector 
                
                aruco.drawAxis(image_sharp, camera_matrix, camera_distortion, rvec, tvec, 10) # draws 3D axis in frame

                R_ct    = np.matrix(cv2.Rodrigues(rvec)[0]) # Rodrigues is a way to parameter a rotation in the 3d space. Ref - https://stackoverflow.com/questions/67088230/what-is-the-aim-of-cv2-rodrigues-applying-on-rvec-in-context-of-camera-calibra
                R_tc    = R_ct.T # transposing the matrix

                roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc) # converting rotational matrix to euler 

                """
                Below list is meant for publishing acquired data to the message file
                """
                ar_id.append(ids[i])
                center_x.append(x_centerPixel)
                center_y.append(y_centerPixel)
                theta.append(theta_2d)


                # print(ids[i],x_centerPixel,y_centerPixel,math.degrees(yaw_marker))

            # publishing the topic "/aruco_data"
            data.ar_id = ar_id
            data.center_x = center_x
            data.center_y = center_y
            data.theta = theta
            pub_aruco.publish(data)
     
        else:
            pass


        cv2.imshow("captured video", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
    

if __name__ == '__main__':
    try:
        detect_aruco()
    except rospy.ROSInterruptException: pass


## Enjoy docking and tracking;-)