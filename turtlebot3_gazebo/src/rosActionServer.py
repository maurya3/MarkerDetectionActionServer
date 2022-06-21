from charset_normalizer import detect
import rospy 
import actionlib
from actionlib_msgs.msg import GoalStatus
from peer_marker_msgs.msg import MarkerDetectionActionAction, MarkerDetectionActionGoal, MarkerDetectionActionFeedback, MarkerDetectionActionResult, MarkerDetectionActionGoal
from aruco_detect import aruco, Tb3Move
from sensor_msgs.msg import Image
import cv_bridge

brg = cv_bridge.CvBridge()

class actionServer():
    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/marker_detection_action_server", 
            MarkerDetectionActionAction, self.detect, auto_start=False)
        self.actionserver.start()
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback) 
        self.r = rospy.Rate(10)

        self.robot_controller = Tb3Move()
        self.detect_marker = aruco() 
        self.x = aruco.pose_esitmation()
   
        self.feedback = MarkerDetectionActionFeedback()
        self.result = MarkerDetectionActionResult()
        self.goal = MarkerDetectionActionGoal()
        #self.actionserver.register_goal_callback(self.goalCB)
        self.id = None
        self.rvec = None
        self.tvec = None
        self.image = None
        self.image_received = False
        self.image_id = None
        self.image_id_received = False
    
    def image_callback(self, image):
        self.image = image
        self.image_received = True
        self.frame = brg.imgmsg_to_cv2(self.image, "bgr8")


    def detect(self, goal):
        self.image_id = goal.marker_number
        #self.detect_marker.detect_marker(self.image, id)
        self.rvec, self.tvec, self.id = self.x(self.frame)
        if id ==self.goal.marker_number:
            self.image_id = self.id
            self.image_id_received = True
        

    # def action_server_launcher(self, goal):
    #     r = rospy.Rate(10)

    #     success = True

    #     if self.id != None and self.id < 0:
    #         print("Please give a valid marker number")
    #         success = False

    #     if self.id != goal.marker_number:
    #         print(" I could't find the given marker")
    #         success = False

    #     if self.actionserver.is_preempt_requested():
    #         rospy.loginfo("Cancelling the camera sweep.")
    #         self.as_.set_preempted()
    #         # stop the robot:
    #         self.robot_controller.stop()
    #         self.feedback.status = "action server called off!!!"
    #         self.actionserver.publish_feedback(self.feedback)
    
    def main(self):    
        if self.image_id_received:
            print("marker detected")
            self.feedback.status = "marker detected"
            self.actionserver.publish_feedback(self.feedback)
            self.result.robot_pose_x = str(self.tvec)
            self.result.robot_pose_y = str(self.rvec)
            self.set_succeeded(self.result)
            self.set_aborted()
        else:
            print("marker not detected")
            self.feedback.status = "marker not detected, rotating"
            self.actionserver.publish_feedback(self.feedback)
            self.robot_controller.set_move_cmd(0, 0.8)
            self.robot_controller.publish()

        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("marker_detection_action_server")
    server = actionServer()
    server.main()

