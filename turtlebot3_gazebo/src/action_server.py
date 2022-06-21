#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib


# Import all the necessary ROS message types:
# from com2009_msgs.msg import CameraSweepFeedback, CameraSweepResult, CameraSweepAction
# from sensor_msgs.msg import CompressedImage
from peer_marker_msgs.msg import MarkerDetectionActionFeedback, MarkerDetectionActionGoal, MarkerDetectionActionResult, MarkerDetectionActionAction
# Import some helper functions from the tb3.py module within this package
from aruco_detect import aruco, Tb3Move

# Import some other useful Python Modules
from math import radians
import datetime as dt
import os

class MarkerDetect(object):
    feedback = MarkerDetectionActionFeedback()
    result = MarkerDetectionActionResult()
    goal = MarkerDetectionActionGoal()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/marker_detection_action_server", 
            MarkerDetectionActionAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.robot_controller = Tb3Move()
        self.detect_marker = aruco()    

    
    def action_server_launcher(self, goal):
        r = rospy.Rate(10)

        success = True

        if goal.marker_number < 0:
            print("Please give a valid marker number")
            success = False

        # if id != goal.marker_number:
        #     print(" I could't find the given marker")
        #     success = False

      

        print(f"\n#####\n"
            f"The 'marker_detection_action_server' has been called.\n"
            f"Goal: capture  marker id {goal.marker_number} and get pose\n\n"
            f"Commencing the action...\n"
            f"#####\n")
        
        success, self.rvec, self.tvec = self.detect_marker.get_pose(goal)

        # set the robot velocity:
        if not success:
            turn_vel = 0.8
            print("setting the robot to take a full turn around and search the marker")
            self.feedback.status = "Trying to find the marker"
            self.actionserver.publish_feedback(self.feedback)
            self.robot_controller.set_move_cmd(0.40, turn_vel)
            self.robot_controller.publish()
            print("marker search in progress")

        if self.actionserver.is_preempt_requested():
            rospy.loginfo("Cancelling the camera sweep.")
            self.actionserver.set_preempted()
            # stop the robot:
            self.robot_controller.stop()
            success = False
            # exit the loop:
            self.feedback.status = "action server called off!!!"
            self.actionserver.publish_feedback(self.feedback)
    
        
        if success:
            rospy.loginfo("task successfully completed.")
            self.result.robot_pose_P = str(self.tvec)
            self.result.robot_pose_R = str(self.rvec)
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
            
        if not success:
            self.actionserver.set_aborted()
            return

if __name__ == '__main__':

    rospy.init_node("camera_action_server")
    
    MarkerDetect()
    rospy.spin()
  