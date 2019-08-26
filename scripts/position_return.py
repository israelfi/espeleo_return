#!/usr/bin/env python

import rospy
import time
from math import cos, sin
import numpy as np
from geometry_msgs.msg import Twist, Polygon, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_msgs.msg import TFMessage

x_n = 0.0
y_n = 0.0
theta_n = 0.0


# Inicializando matriz que armazena as posicoes (x,y)
position_buffer = np.zeros((2,200))

# Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global x_n, y_n, theta_n
    global pos, rpy


    #print data
    for T in data.transforms:
        if (T.child_frame_id == "EspeleoRobo"):

            x_n = data.transforms[0].transform.translation.x  # posicao 'x' do robo no mundo
            y_n = data.transforms[0].transform.translation.y  # posicao 'y' do robo no mundo
            x_q = data.transforms[0].transform.rotation.x
            y_q = data.transforms[0].transform.rotation.y
            z_q = data.transforms[0].transform.rotation.z
            w_q = data.transforms[0].transform.rotation.w
            euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
            theta_n = euler[2]

            pos[0] = data.transforms[0].transform.translation.x
            pos[1] = data.transforms[0].transform.translation.y
            pos[2] = data.transforms[0].transform.translation.z
            rpy = euler

    return
# ---------------------------------------------------------

# Buffer que armazena a posicao do robo
def get_position():
    global position_buffer
    # Guarda a posicao x do robo
    position_buffer[0][:-1] = position_buffer[0][1:]
    position_buffer[0][-1] = x_n
    # Guarda a posicao y do robo
    position_buffer[1][:-1] = position_buffer[1][1:]
    position_buffer[1][-1] = y_n
    time.sleep(0.1)
# --------------------------------------------------------



# Function to send a array of markers, representing the curve, to rviz
def send_curve_to_rviz(traj,pub_rviz):

    # Create messsage
    points_marker = MarkerArray()
    marker = Marker()
    # Iterate over the points
    for k in range(len(traj[0])):
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        marker.id = k
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        # Size of sphere
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        # Color and transparency
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # Pose
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = traj[0][k]
        marker.pose.position.y = traj[1][k]
        marker.pose.position.z = 0.1
        # Append marker to array
        points_marker.markers.append(marker)

    pub_rviz.publish(points_marker)

    return (points_marker)
# ----------  ----------  ----------  ----------  ----------



# Function to create a message of the type polygon, which will carry the points of the curve
def create_traj_msg(position_buffer):

    # Create 'Polygon' message (array of messages of type 'Point')
    traj_msg = Polygon()
    p = Point()
    for k in range(len(position_buffer[0])):
        # Create point
        p = Point()
        # Atribute values
        p.x = position_buffer[0][k]
        p.y = position_buffer[1][k]
        p.z = 0.0
        # Append point to polygon
        traj_msg.points.append(p)

    return traj_msg
# -------------------------------------------------------

def talker():
    rospy.init_node('return', anonymous=True)
    pub_traj = rospy.Publisher("/espeleo/traj_points", Polygon, queue_size=1)
    rospy.Subscriber("/tf", TFMessage, callback_pose)

    rate = rospy.Rate(freq)
    i = 0

    while time < 10:
        i = i + 1
        time = i / float(freq)
        get_position()
        # print position_buffer
        print time

    traj_msg = create_traj_msg(position_buffer)
    pub_traj.publish(traj_msg)
    print "-----------"



    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == '__main__':

    # Frequency of field computation
    global freq
    freq = 20.0  # Hz

    global pos, rpy
    pos = [0, 0, 0]
    rpy = [0, 0, 0]

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
