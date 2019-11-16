#!/usr/bin/env python

import rospy
import time
from math import cos, sin, sqrt
import numpy as np
from geometry_msgs.msg import Twist, Polygon, Point
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool

# Esse codigo visa coletar a pose ao longo do tempo do robo e guarda-la em um buffer

x_n = 0.0       # x atual
y_n = 0.0       # y atual
theta_n = 0.0   # theta atual
x0 = 0.0        # Guarda o x da posicao de retorno
y0 = 0.0        # Guarda o y da posicao de retorno
delta = 0.01    # Variacao minima na posicao para que o buffer seja atualizado
tolerance = 0.2 # Tolerancia entre o ponto tido como objetivo e posicao de parada
size = 5.0      # Distancia de retorno maxima


lost_signal = False

# Inicializando lista de pontos que armazena as posicoes (x,y)
position_buffer = []


# Funcao que analisa o status do sinal do radio
def callback_signal(data):
    global lost_signal
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
# ---------------------------------------------------------

# Buffer que armazena a posicao do robo
def get_position():
    global position_buffer
    # Guarda o ponto (x,y) do robo no mundo
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
# ----------------------------------------------------------


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


def position_return():
    rospy.init_node('return', anonymous=True)

    # Topico onde eh publicado os pontos por onde o robo percorreu
    pub_traj = rospy.Publisher("/return/traj_points", Polygon, queue_size=1)
    pub_rviz_curve = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
    # Topico com a flag que anuncia se o robo ja chegou no ponto desejado
    pub_flag = rospy.Publisher("/flag/distance_target", Bool, queue_size=1)
    # Topico com a odometria do robo
    rospy.Subscriber("/tf", TFMessage, callback_pose)
    # Flag que anuncia se o sinal foi perdido
    rospy.Subscriber("/flag/signal", Bool, callback_signal)

    rate = rospy.Rate(freq)

    flag_position = Bool()
    flag_position = False

    # Variaveis pra guardar o ponto (x,y) anterior
    x_n_previous = 0.0
    y_n_previous = 0.0

    flag_position = True
    
	# Variavel auxiliar para controle de rotina
	a = 1
	
	# Guarda a distancia em metros que o buffer tem armazenado
    buffer_size = 0.0 

    while not rospy.is_shutdown():
	
        # Rotina realizada enquanto o sinal de radio nao foi perdido e o robo tiver chegado no ponto desejado
        while (lost_signal != True and flag_position == True):
            # Distancia entre (x,y) atual e anterior
            distance_variation = sqrt((x_n - x_n_previous)**2 + (y_n - y_n_previous)**2)

            # Adiciona o ponto atual no buffer se a diferenca entre ele e o ponto anterior for maior que delta
            if (distance_variation > delta):
                x_n_previous = x_n
                y_n_previous = y_n
                buffer_size = buffer_size + distance_variation

                # Elimina a primeira posicao do buffer se ele ja estiver cheio
                if buffer_size > size:
                    position_buffer.pop(0)
                    get_position()
                else:
                    get_position()

            else:
				buffer_size = buffer_size + distance_variation
                continue

        # Caso o sinal de radio seja perdido:
		
		# Rotina feita somente uma vez quando o sinal é perdido
        if(lost_signal == True and a == 1):
            flag_position = False
            pub_flag.publish(flag_position)
						
			try:
				# x0 e y0 sao as coord. do ponto de retorno
				x0, y0 = (position_buffer[0][0], position_buffer[0][1])
				
			except:
				print('A problem occurred creating the buffer')
				
            traj_msg = create_traj_msg(position_buffer)
            pub_traj.publish(traj_msg)
            time.sleep(1.0)
            send_curve_to_rviz(position_buffer, pub_rviz_curve)
            a = 0

        # Calcula a distancia entre o ponto atual e o ponto de retorno
        distance = sqrt((x_n - x0)**2 + (y_n - y0)**2)

        # Checa continuamente se o robo chegou ao ponto desejado
        if distance < 1.0*tolerance:
            flag_position = True
            a = 1
			
			# Limpando o buffer
			try:
				del position_buffer[:] 
				buffer_size = 0
			except:
				print('Buffer already empty')
        elif a == 0:
            flag_position = False

        pub_flag.publish(flag_position)
        rate.sleep()



if __name__ == '__main__':

    # Frequency of field computation
    global freq, t_initial, x_n_previous, y_n_previous#, j
    global distance

    # Informa a distancia entre posicao atual e a do inicio do buffer
    global d_percorrido
    freq = 20.0  # Hz

    t_initial = time.time()
    
    global pos, rpy
    pos = [0, 0, 0]
    rpy = [0, 0, 0]

    try:
        position_return()
    except rospy.ROSInterruptException:
        pass
