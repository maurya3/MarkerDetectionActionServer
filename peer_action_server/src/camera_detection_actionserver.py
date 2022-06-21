#!/usr/bin/env python
## written by Deepak
# Import the core Python modules for ROS and to implement ROS Actions:

import rospy
import actionlib

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

from peer_marker_msgs.msg import CameraDetectionFeedback, CameraDetectionResult, CameraDetectionAction
from sensor_msgs.msg import Image

# Import some helper functions from the tb3.py module within this package
from tb3 import Tb3Move

# Import some other useful Python Modules
import numpy as np 
cv_bridge = CvBridge()

class CameraDetection():
    feedback = CameraDetectionFeedback() 
    result = CameraDetectionResult() 

    def __init__(self):
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image, self.camera_callback)
        self.actionserver = actionlib.SimpleActionServer("/camera_Detection_action_server", CameraDetectionAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        self.camera_image = None #np.array((640,480),cv2.CV_8UC1)
        # self.cv_image = CvBridge()

        ## Aruco marker detection parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.camera_coefficients = np.array([[402.2965906510784, 0.0, 320.5],[0.0, 402.2965906510784, 240.5],[0.0, 0.0, 1.0]])
        self.distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        ## Robot movement 
        self.robot_controller = Tb3Move()

        
    
    def camera_callback(self, img):
        self.image_to_capture = cv_bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        self.camera_image = cv2.cvtColor(self.image_to_capture, cv2.COLOR_BGR2GRAY)
        
        
    def action_server_launcher(self, goal):
        r = rospy.Rate(10)
        turn_vel = -0.5 # rad/s

        print(f"\n#####\n"
            f"The 'camera_detection_action_server' has been called.\n"
            f"Goal: Get position of the robot from the marker id  {goal.marker_id} \n\n"
            f"Commencing the action...\n"
            f"Starting robot rotation at its place with  {turn_vel} rad/sec velocity."
            f"#####\n")
        
        while not rospy.is_shutdown():
            self.robot_controller.set_move_cmd(0.0, turn_vel)
            self.robot_controller.publish()
            corners, ids, rejectedImgPoints =  cv2.aruco.detectMarkers(self.camera_image, self.aruco_dict,cameraMatrix= self.camera_coefficients,parameters=self.parameters)
            # If markers are detected      
            if len(corners) > 0:
                if ids[0] == goal.marker_id:
                    # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.08, self.camera_coefficients,
                                                                            self.distortion_coefficients)
                    # Draw a square around the markers
                    cv2.aruco.drawDetectedMarkers(self.image_to_capture, corners) 
                    # Draw Axis
                    cv2.aruco.drawAxis(self.image_to_capture, self.camera_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)  
                    self.feedback.current_marker = int(ids[0])
                    self.feedback.current_pose = f"current position of the robot is {tvec[0]} and orientation is {rvec[0]}"
                    self.actionserver.publish_feedback(self.feedback)
                    success = True
                    print("Found marker id =" + str(ids[0][0]))
                    print("Current robot pose with respect to marker " + str(ids[0][0]) + " is " + str(tvec[0][0]) + "and orientation is " + str(rvec[0][0]))
                    self.robot_controller.stop()
                    cv2.imshow("---",self.image_to_capture)
                    cv2.waitKey(5000)
                    cv2.destroyAllWindows()

                    break
            


            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the camera detection node.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.robot_controller.stop()
                success = False
                # exit the loop:
                break
            

        
        if success==True:
            rospy.loginfo("robot pose detection completed sucessfully.")
            self.result.robot_pose_x = tvec[0] 
            self.result.robot_pose_y = tvec[1]
            self.result.robot_pose_theta = rvec[1]
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
            rospy.loginfo("Waiting for the new goal.....")




            
if __name__ == '__main__':
    rospy.init_node("camera_sweep_action_server")
    try:
        CameraDetection()
        rospy.spin()
    except:
        print("wait")

