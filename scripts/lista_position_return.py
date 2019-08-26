#!/usr/bin/env python

import rospy
import time
from math import cos, sin
import numpy as np
from geometry_msgs.msg import Twist, Polygon, Point
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

x_n = 0.0
y_n = 0.0
theta_n = 0.0
duration = 18.0

# Inicializando lista de pontos que armazena as posicoes (x,y)
position_buffer = []

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
    time.sleep(0.1)
    # Guarda o ponto (x,y) do robo
    position_buffer.append((x_n,y_n))

# --------------------------------------------------------



# Function to send a array of markers, representing the curve, to rviz
def send_curve_to_rviz(position_buffer,pub_rviz):

    # Create messsage
    points_marker = MarkerArray()
    marker = Marker()
    # Iterate over the points
    for k in range(len(position_buffer)):
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
        marker.pose.position.x = position_buffer[k][0]
        marker.pose.position.y = position_buffer[k][1]
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
    position_buffer.reverse()
    length = len(position_buffer)
    for k in range(length):
        # Create point
        p = Point()
        # Atribute values
        p.x = position_buffer[k][0]
        p.y = position_buffer[k][1]
        p.z = 0.0
        # Append point to polygon
        traj_msg.points.append(p)

    return traj_msg
# -------------------------------------------------------

def talker():
    rospy.init_node('return', anonymous=True)

    # Obs.: o simulador esta subscrevendo ao topico /espelo/traj_points e nao ao /espeleo/traj_points
    pub_traj = rospy.Publisher("/espelo/traj_points", Polygon, queue_size=1)
    pub_rviz_curve = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
    rospy.Subscriber("/tf", TFMessage, callback_pose)

    rate = rospy.Rate(freq)



    while (time.time() - t_initial) < duration:
        get_position()

    traj_msg = create_traj_msg(position_buffer)
    pub_traj.publish(traj_msg)
    print traj_msg

    time.sleep(1.0)
    send_curve_to_rviz(position_buffer, pub_rviz_curve)


    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == '__main__':

    # Frequency of field computation
    global freq, t_initial
    freq = 20.0  # Hz
    t_initial = time.time()
    global pos, rpy
    pos = [0, 0, 0]
    rpy = [0, 0, 0]

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
