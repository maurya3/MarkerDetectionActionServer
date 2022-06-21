#!/usr/bin/env python
from ctypes import sizeof
import math
from turtle import shape
import cv2
import cv_bridge 
import numpy as np 
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

class aruco:
    def __init__(self):
        #rospy.init_node("aruco_detection")
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.camera_coefficients = np.array([[1206.8897719532354, 0.0,960.5],[0.0, 1206.8897719532354, 540.5],[0.0, 0.0, 1.0]])
        self.distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.bridge = cv_bridge.CvBridge()
        self.gray = np.zeros((480,640),np.uint8)
        self.rate = rospy.Rate(30)
        self.rvec = np.array([[0,0,0]])
        self.tvec = np.array([[0,0,0]])


    def pose_esitmation(self,frame):
        """
        All the camera parameters are given default from the camera_info topic.
        """
        corners, ids, rejectedImgPoints =  cv2.aruco.detectMarkers(self.gray, self.aruco_dict,cameraMatrix= self.camera_coefficients,parameters=self.parameters)
        cv2.waitKey(10)
    
        rvec = []
        tvec = []
        id = None
            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                print(ids[i])
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                self.rvec, self.tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.08, self.camera_coefficients,
                                                                        self.distortion_coefficients)
                # Draw a square around the markers
                #cv2.aruco.drawDetectedMarkers(frame, corners) 
                # Draw Axis
                #cv2.aruco.drawAxis(frame, self.camera_coefficients, self.distortion_coefficients, self.rvec, self.tvec, 0.01)  
        else :
            print("No ArUco detected")       
        
        return rvec, tvec, id


class Tb3Move(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()
        self.publish()


        

