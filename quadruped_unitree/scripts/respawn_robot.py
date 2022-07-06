#! /usr/bin/env python3

"""
Respawn quadruped robot in Gazebo Simulator

Usage)
rosrun 
"""

# gazebo_msgs/SetModelState

import rospy
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState 

rospy.init_node("respawn_robot_node")
rospy.loginfo("==== Robot Respawn Client Started ====")

rospy.wait_for_service("/gazebo/set_model_state")
service_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

request_msg = ModelState()

def is_invalid_angle(angle):

    if abs(angle) < 3.141592:
        return True
    else:
        return False

while not rospy.is_shutdown():
    try:
        robot_name = input("> Choose Your Robot Name \n1) a1 \n2) go1 \n\n>>> ")

        if robot_name == "1":
            robot_name = "a1_gazebo"
        elif robot_name == "2":
            robot_name = "go1_gazebo"

        # x, y, z, _ = input("> Type xyz robot spawn location ex) 0 0 0 : ").split()
        # r, p, y, _ = input("> Type xyz robot spawn location ex) 0 0 1.5707 : ").split()

        # for elem in (r, p, y):
        #     if is_invalid_angle(elem):
        #         raise ArithmeticError("Invalid Angle Value !!")

        request_msg.model_name = robot_name

        request_msg.pose.position.x = 0.0
        request_msg.pose.position.y = 0.0
        request_msg.pose.position.z = 0.3

        request_msg.pose.orientation.x = 0.0
        request_msg.pose.orientation.y = 0.0
        request_msg.pose.orientation.z = 0.0
        request_msg.pose.orientation.w = 1.0

        break
    except ArithmeticError as e:
        rospy.logerr(e)
    except Exception as e:
        rospy.logerr("Not a number type number plz !!")

result = service_client(request_msg)

print(result)