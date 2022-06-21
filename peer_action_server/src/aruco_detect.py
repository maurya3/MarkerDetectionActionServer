#!/usr/bin/env python
import cv2
import cv_bridge 
import numpy as np 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler

class aruco:
    def __init__(self):
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.camera_coefficients = np.array([[1206.8897719532354, 0.0,960.5],[0.0, 1206.8897719532354, 540.5],[0.0, 0.0, 1.0]])
        self.distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    def pose_esitmation(self,frame):
        """
        All the camera parameters are given default from the camera_info topic.
        """
        corners, ids, rejectedImgPoints =  cv2.aruco.detectMarkers(self.gray, self.aruco_dict,cameraMatrix= self.camera_coefficients,parameters=self.parameters)
        cv2.waitKey(10)
            # If markers are detected      
        if len(corners) > 0:
            for i in range(0, len(ids)):
                print(ids[i])
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                self.rvec, self.tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.08, self.camera_coefficients,
                                                                        self.distortion_coefficients)
        



        

