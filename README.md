# MarkerDetectionActionServer
A ros action server-based marker detector with the below functionalities:
a. Should be able to detect any given external marker (aruco or QR code-based marker, any one of your choice)
b. Determine the position of the marker(x, y and theta) with respect to the robot.
c. INPUT: aruco marker OUTPUT: position of marker wrt robot
NOTE: It requires setup of ROS on your local system.

* ROS setup: 
		Please refer to Data & Codes drive link. *https://drive.google.com/drive/folders/18AiKhb8YkJHe-kd7CNuMCNlQENerJb8I?usp=sharing*
  I have setup a ROS simulation environment using the Turtlebot3_gazebo model with a camera. 
  Created a gazebo world file and included a aruco marker id 26 and size of 8 cm. 
  Created a launch file including the robot model and gazebo world.

* Workspace Setup: 
Created a workspace named peer_marker_msgs, inside that created an action file that consists of the action server messages structure: 

```
#goal        
int64 marker_number
---
#result
int64 robot_pose_x
int64 robot_pose_y
int64 robot_pose_theta
---
#feedback
string status

```

Made necessary changes in the Cmakelist.txt file to generate the action-specific messages.

* Marker detection Algorithm:
Aruco marker detection has been done using OpenCV function. I have used the image topic/ camera_info node data to give to the marker detection parameter. 
After detection of the marker pose is estimated using the openCV aruco function that gives the Rvec, and Tvec, Tvec contains the x, y, z position of the marker and Rvec tells the rotation of the marker w.r.t camera frame. 
	* Robot Navigation Algorithm:
Created a simple robot navigation file that is used to control the robot. This function is used to rotate the robot to search the market in the world and stop the robot when the marker is detected. 
	* Action Server Node:
The action server node is created that accepts the marker id as a goal and executes the search process of the marker. When the given marker is detected it sends the status message and the position as a  result message. It waits for the next goal until the node is terminated. 
	* Run Simulation:
To run the simulation, run the below command to open the simulation

``` roslaunch turtlebot3_gazebo turtlebot3_marker26.launch ```

In another window, run the marker action server launch command, 

``` roslaunch peer_action_server camera_detection.launch ```

![turtle26](https://user-images.githubusercontent.com/58929684/174861498-c37664bb-f631-49ce-b3e7-4917b46701d5.png)

In other terminal window publish some marker id using command 
```

rostopic pub /camera_Detection_action_server/goal peer_msgs/CameraDetectionActionGoal "header:
  seq: 0
  stamp:
	secs: 0
	nsecs: 0
  frame_id: ''
goal_id:
  stamp:
	secs: 0
	nsecs: 0
  id: ''
goal:
  marker_number: 26"

```
In place of the marker id, we can specify the marker number to be detect and get relative pose of the robot.
The robot will start rotating in its place if it does not detect the given goal. 

If it detects the specified marker id, it will generate the feedback and result command in the action-specific topic.
![arucom](https://user-images.githubusercontent.com/58929684/174861990-f50330da-7054-4bd8-a6bb-d1ca40055a25.png)





