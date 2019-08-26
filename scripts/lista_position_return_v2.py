#!/usr/bin/env python

import rospy
import time
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
from geometry_msgs.msg import Twist, Polygon, Point
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
import copy

x_n = 0.0
y_n = 0.0
theta_n = 0.0
duration = 6.0
v_gain = 2.0

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
    time.sleep(0.01)
    # Guarda o ponto (x,y) do robo
    position_buffer.append((x_n,y_n))

# --------------------------------------------------------


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


# Obtain the trajectory to be followed by the robot
def get_trajectory(traj_msg):

    global traj_temp
    global state_k, state_k_temp
    global has_trajectory_flag
    global N_temp



    traj_temp = [[],[]]

    for k in range(len(traj_msg.points)):
        p = traj_msg.points[k]
        traj_temp[0].append(p.x)
        traj_temp[1].append(p.y)
        #traj_temp[[z],[k]] = p.z# z is not used

    # Update the closest index
    N_temp = len(traj_temp[0])
    state_k_temp = 0
    D = 100000
    for k in range(N_temp):
        D_temp = sqrt((pos[0]-traj_temp[0][k])**2 + (pos[1]-traj_temp[1][k])**2)
        if (D_temp<D):
            state_k_temp = k
            D = D_temp


    global traj, N, state_k
    traj = copy.deepcopy(traj_temp)
    state_k = copy.deepcopy(state_k_temp)
    N = copy.deepcopy(N_temp)

    has_trajectory_flag = True


    return
# ----------  ----------  ----------  ----------  ----------




# Compute the vectro field that will guide the robot
def vec_field(pos):

    global state_k, state_k_temp, state_k_delta
    global traj, traj_temp
    global N

    x = pos[0]
    y = pos[1]


    #print "traj_temp = \n", traj_temp

    # Compute the closest ponit on the curve
    #k_min = state_k
    k_vec = [state_k-state_k_delta+i for i in range(state_k_delta)]
    #k_vec.append([state_k-1-i for i in range(state_k_delta)])
    k_vec.append(state_k)
    k_vec = k_vec + [state_k+1+i for i in range(state_k_delta)]
    for k in range(len(k_vec)):
        if k_vec[k]<0:
            k_vec[k] = k_vec[k] + N
        if k_vec[k]>=N:
            k_vec[k] = k_vec[k] - N
    D = 100000
    #print "AAAA ", len(traj[0]), ", ", k_vec
    for k in k_vec:
        #print "CCC ", len(traj[0]), len(traj[1]), ", ", k
        D_temp = sqrt((x-traj[0][k])**2 + (y-traj[1][k])**2)
        if (D_temp<D):
            k_min = k
            D = D_temp
    # Update state_k
    state_k = k_min


    # Compute the distance vector
    D_vec = [x-traj[0][k_min], y-traj[1][k_min]]
    # Compute the gradient of the distance Function
    grad_D = [D_vec[0]/(D+0.000001), D_vec[1]/(D+0.000001)]

    # Compute the tangent vector of the curve at k_min
    k1 = k_min - 1
    if k1 == -1:
        k1 = N-1
    k2 = k_min + 1
    if k2 == N:
        k2 = 0
    T = [traj[0][k2]-traj[0][k1], traj[1][k2]-traj[1][k1]]
    norm_T = sqrt(T[0]**2 + T[1]**2)
    T = [T[0]/norm_T, T[1]/norm_T]

    # Lyapunov Function
    P = (0.5)*D**2
    # Gain functions
    G = -(2/pi)*atan(kf*sqrt(P))
    H = sqrt(1-G**2)


    Vx = G*grad_D[0] + H*T[0]
    Vy = G*grad_D[1] + H*T[1]

    #print "k_min = [",k_min , "]"
    #print "k_vec = ", k_vec
    #print "D = [",D , "]"
    #print "kf = [",kf , "]"
    #print "G = [",G , "]"
    #print "grad_D = [",grad_D , "]"
    #print "F = [", Vx, ", ", Vy, "]\n"




    #Vx = 0.7071
    #Vy = 0.7071



    return (Vx, Vy)
# ----------  ----------  ----------  ----------  ----------


# Rotina feedback linearization
def feedback_linearization(Ux, Uy):
    #global x_n, y_n, theta_n
    global d

    psi = rpy[2]

    VX = cos(psi) * Ux + sin(psi) * Uy
    WZ = (-sin(psi) / d) * Ux + (cos(psi) / d) * Uy

    # print "Ux, Uy = ", Ux, Uy

    return (VX, WZ)

# ----------  ----------  ----------  ----------  ----------


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



def talker():
    rospy.init_node('return', anonymous=True)

    # Obs.: o simulador esta subscrevendo ao topico /espelo/traj_points e nao ao /espeleo/traj_points
    # pub_traj = rospy.Publisher("/espelo/traj_points", Polygon, queue_size=1)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    pub_rviz_curve = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
    rospy.Subscriber("/tf", TFMessage, callback_pose)


    vel = Twist()

    rate = rospy.Rate(freq)



    while (time.time() - t_initial) < duration:
        get_position()

    traj_msg = create_traj_msg(position_buffer)
    get_trajectory(traj_msg)
    print traj_msg

    traj = copy.deepcopy(traj_temp)
    state_k = copy.deepcopy(state_k_temp)
    N = copy.deepcopy(N_temp)



    time.sleep(1.0)
    send_curve_to_rviz(position_buffer, pub_rviz_curve)


    while not rospy.is_shutdown():

        [Vx_ref, Vy_ref] = vec_field(pos)

        [V_forward, w_z] = feedback_linearization(Vx_ref, Vy_ref)

        vel.linear.x = V_forward * v_gain
        vel.angular.z = w_z * v_gain

        pub_cmd_vel.publish(vel)
        print vel

        rate.sleep()



if __name__ == '__main__':

    # Frequency of field computation
    global freq, t_initial
    freq = 100.0  # Hz

    t_initial = time.time()

    # Robot position and orientation
    global pos, rpy
    pos = [0, 0, 0]
    rpy = [0, 0, 0]

    # Convergence intensity of the vector field
    global kf
    kf = 2.0

    # Constant relative to the feedback linearization controller
    global d
    d = - 0.2

    global has_trajectory_flag
    has_trajectory_flag = False

    global state_k, state_k_temp, state_k_delta
    state_k = 0
    state_k_delta = 10

    global traj, traj_temp
    global N, N_temp

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
