#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Polygon
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
from tf2_msgs.msg import TFMessage
import numpy as np
import copy
from std_msgs.msg import Bool




ganho = 0.3
flag = False
lost_signal = False
sinal = False

def callback_flag(data):
    global flag
    flag = data.data

def callback_signal(data):
    global lost_signal, sinal
    sinal = data.data
    lost_signal = data.data



# Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global x_n, y_n, theta_n
    global pos, rpy

    for T in data.transforms:
        if (T.child_frame_id == "base_link"):

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

# ----------  ----------  ----------  ----------  ----------



# Obtain the trajectory to be followed by the robot
def callback_trajectory(data):

    global traj_temp
    global state_k, state_k_temp
    global has_trajectory_flag
    global N_temp

    traj_temp = [[],[]]

    for k in range(len(data.points)):
        p = data.points[k]
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




# Compute the vector field that will guide the robot
def vec_field(pos):

    global state_k, state_k_temp, state_k_delta
    global traj, traj_temp
    global N

    x = pos[0]
    y = pos[1]

    # Compute the closest ponit on the curve
    k_vec = [state_k-state_k_delta+i for i in range(state_k_delta)]
    k_vec.append(state_k)
    k_vec = k_vec + [state_k+1+i for i in range(state_k_delta)]
    for k in range(len(k_vec)):
        if k_vec[k]<0:
            k_vec[k] = k_vec[k] + N
        if k_vec[k]>=N:
            k_vec[k] = k_vec[k] - N
    D = 100000

    for k in k_vec:
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
        k1 = 0
    k2 = k_min + 1
    if k2 == N:
        k2 = N-1
    T = [traj[0][k2]-traj[0][k1], traj[1][k2]-traj[1][k1]]
    norm_T = sqrt(T[0]**2 + T[1]**2)
    T = [T[0]/norm_T, T[1]/norm_T]

    # Lyapunov Function
    P = (0.5)*D**2
    # Gain functions
    G = -(2/pi)*atan(kf*sqrt(P))*2.0 # Ganho de 2.0
    H = sqrt(1-G**2)


    Vx = G*grad_D[0] + H*T[0]
    Vy = G*grad_D[1] + H*T[1]

    # print "V = [", Vx, ", ", Vy, "]"
    # print "V_conv = [", G*grad_D[0], ", ", G*grad_D[1], "]"
    # print "V_tang = [", H*T[0], ", ", H*T[1], "]\n"

    #print "k_min = [",k_min , "]"
    #print "k_vec = ", k_vec
    #print "D = [",D , "]"
    #print "kf = [",kf , "]"
    #print "G = [",G , "]"
    #print "grad_D = [",grad_D , "]"
    #print "F = [", Vx, ", ", Vy, "]\n"


    return (Vx, Vy)
# ----------  ----------  ----------  ----------  ----------


# Rotina feedback linearization
def feedback_linearization(Ux, Uy):

    global d

    psi = rpy[2]

    VX = cos(psi) * Ux + sin(psi) * Uy
    WZ = (-sin(psi) / d) * Ux + (cos(psi) / d) * Uy

    VX, WZ = (ganho*VX, ganho*WZ )

    return (VX, WZ)

# ----------  ----------  ----------  ----------  ----------


# Rotina executada apenas uma vez para mostrar a ellipse no rviz
def send_curve_to_rviz(pub_rviz):

    points_marker = MarkerArray()
    marker = Marker()

    for k in range(N):
        x = traj[0][k]
        y = traj[1][k]
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        marker.id = k
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        points_marker.markers.append(marker)

    pub_rviz.publish(points_marker)

    return (points_marker)
# ----------  ----------  ----------  ----------  ----------


# Rotina para piblicar informacoes no rviz
def send_marker_to_rviz(pub_rviz, Vx, Vy):

    mark_ref = Marker()

    mark_ref.header.frame_id = "/world"
    mark_ref.header.stamp = rospy.Time.now()
    mark_ref.id = 0
    mark_ref.type = mark_ref.ARROW
    mark_ref.action = mark_ref.ADD
    mark_ref.scale.x = 1.5 * (Vy ** 2 + Vx ** 2) ** (0.5)
    mark_ref.scale.y = 0.08
    mark_ref.scale.z = 0.08
    mark_ref.color.a = 1.0
    mark_ref.color.r = 0.0
    mark_ref.color.g = 0.0
    mark_ref.color.b = 0.0
    mark_ref.pose.position.x = pos[0]
    mark_ref.pose.position.y = pos[1]
    mark_ref.pose.position.z = pos[2]
    quaternio = quaternion_from_euler(0, 0, atan2(Vy, Vx))
    mark_ref.pose.orientation.x = quaternio[0]
    mark_ref.pose.orientation.y = quaternio[1]
    mark_ref.pose.orientation.z = quaternio[2]
    mark_ref.pose.orientation.w = quaternio[3]

    pub_rviz_ref.publish(mark_ref)

    return

# ----------  ----------  ----------  ----------  ----------


# Rotina primaria
def controller():
    global freq
    global x_n, y_n, theta_n
    global pub_rviz_ref, pub_rviz_pose
    global traj
    global N

    vel = Twist()

    i = 0

    # pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    pub_cmd_vel = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=1)
    rospy.init_node("vector_field")
    # Topico contendo a pose do robo
    rospy.Subscriber("/tf", TFMessage, callback_pose)
    # Topico contendo os pontos da trajetoria de retorno
    rospy.Subscriber("/return/traj_points", Polygon, callback_trajectory)
    # Topico com a flag que anuncia se o robo ja chegou no ponto desejado
    rospy.Subscriber("/flag/distance_target", Bool, callback_flag)
    # Topico com a flag que anuncia se o sinal foi perdido
    rospy.Subscriber("/flag/signal", Bool, callback_signal)

    lost_signal = Bool()

    pub_rviz_ref = rospy.Publisher("/visualization_marker_ref", Marker, queue_size=1) #rviz marcador de velocidade de referencia
    pub_rviz_curve = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1) #rviz array de marcadores no espaco da elipse

    rate = rospy.Rate(freq)

    while not has_trajectory_flag and not rospy.is_shutdown():
        # print "Waitting for trajectory ..."
        rate.sleep()

    # traj = copy.deepcopy(traj_temp)
    # state_k = copy.deepcopy(state_k_temp)
    # N = copy.deepcopy(N_temp)

    rate.sleep()

    traj

    while not rospy.is_shutdown():

        i = i + 1
        time = i / float(freq)

        try:

            # Caso o sinal de radio seja perdido
            if (sinal == True):

                traj = copy.deepcopy(traj_temp)
                state_k = copy.deepcopy(state_k_temp)
                N = copy.deepcopy(N_temp)
                rospy.sleep(1.)

                # Rotina realizada enquanto o robo nao retorna ao ponto desejado
                while (flag == False):
                    [Vx_ref, Vy_ref] = vec_field(pos)

                    [V_forward, w_z] = feedback_linearization(Vx_ref, Vy_ref)

                    vel.linear.x = V_forward
                    vel.angular.z = w_z

                    send_curve_to_rviz(pub_rviz_curve)
                    send_marker_to_rviz(pub_rviz_ref, Vx_ref, Vy_ref)

                    pub_cmd_vel.publish(vel)

            else:
                continue

        except:
            # This is due to the changes in the curve's change
            print "Temporary problem in the computation of the field !"

        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------




# Funcao inicial
if __name__ == '__main__':

    # Frequency of field computation
    global freq
    freq = 100.0  # Hz

    # Robot position and orientation
    global pos, rpy
    pos = [0, 0, 0]
    rpy = [0, 0, 0]

    # Convergence intensity of the vector field
    global kf
    kf = 2.0

    # Constant relative to the feedback linearization controller
    global d
    d = -0.2 # ------------------------------SINAL--------------------------------------------------

    global has_trajectory_flag
    has_trajectory_flag = False

    global state_k, state_k_temp, state_k_delta
    state_k = 0
    state_k_delta = 10

    global traj, traj_temp
    global N, N_temp

    try:
        controller()
    except rospy.ROSInterruptException:
        pass
