#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#Author: lnotspotl

import rospy

from sensor_msgs.msg import Joy,Imu
from geometry_msgs.msg import Twist

from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64

# 쓰는 게 좋은지 아닌지 솔직히 잘 모르겠음...
USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
# 몸체 가로 / 세로
# 0.183 * 2 / 0.047 * 2
# body = [0.366, 0.094]

# 0.1881 * 2 / 0.04675 * 2
body = [0.3762, 0.0935]

# 각 다리 link 길이
# 0 0.08 0.213 0.213
# hip => thigh sholder => thigh => calf => foot
# 
# hip => thigh_shoulder (0.081)  / (0.08)
# hip => thigh (0.08505) / (0.08)
# thigh => calf (0.2)  / (0.213)
# calf => foot (0.2) / (0.213)

# legs = [0.,0.08505, 0.2, 0.2] 
legs = [0.,0.08, 0.213, 0.213] 


a1_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

command_topics = ["/go1_gazebo/FR_hip_joint/command",
                  "/go1_gazebo/FR_thigh_joint/command",
                  "/go1_gazebo/FR_calf_joint/command",
                  "/go1_gazebo/FL_hip_joint/command",
                  "/go1_gazebo/FL_thigh_joint/command",
                  "/go1_gazebo/FL_calf_joint/command",
                  "/go1_gazebo/RR_hip_joint/command",
                  "/go1_gazebo/RR_thigh_joint/command",
                  "/go1_gazebo/RR_calf_joint/command",
                  "/go1_gazebo/RL_hip_joint/command",
                  "/go1_gazebo/RL_thigh_joint/command",
                  "/go1_gazebo/RL_calf_joint/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

# imu 값은 gazebo plugin에서 받으며, 이는 회전 보상 시 사용됨
if USE_IMU:
    rospy.Subscriber("go1_imu/base_link_orientation",Imu,a1_robot.imu_orientation)
# rospy.Subscriber("a1_joy/joy_ramped",Joy,a1_robot.joystick_command)
rospy.Subscriber("/twist_mux/cmd_vel", Twist, a1_robot.cmd_vel_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

while not rospy.is_shutdown():
    # foot location을 받는다.
    leg_positions = a1_robot.run()
    a1_robot.change_controller()

    dx = a1_robot.state.body_local_position[0]
    dy = a1_robot.state.body_local_position[1]
    dz = a1_robot.state.body_local_position[2]
    
    roll = a1_robot.state.body_local_orientation[0]
    pitch = a1_robot.state.body_local_orientation[1]
    yaw = a1_robot.state.body_local_orientation[2]

    try:
        # IK를 통해 각 joint 각도 계산
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                               dx, dy, dz, roll, pitch, yaw)

        for i in range(len(joint_angles)):
            publishers[i].publish(joint_angles[i])
    except:
        pass

    rate.sleep()
