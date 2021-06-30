#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from math import pi, atan2
import cv2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseArray, Pose
from sound_play.libsoundplay import SoundClient
import random
from numpy.random import choice
from scipy import spatial
import math
import time
import os
from collections import Counter
import threading

# HACER ANGULO RANDOM

class VirtualRobot( object ):

    def __init__( self ):
        rospy.init_node( 'VirtualRobot' )
        # Para parlantes
        self.soundhandle = SoundClient( blocking = True )
        # Para movimiento
        self.contador_vueltas = 0
        self.girar_45 = False
        self.dist_speed = 0.1
        self.angulo_speed = 0.785
        self.finish = False
        self.front = False
        self.left_obs = 0.0
        self.front_obs = 0.0
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size = 1)
        self.send_part_pub = rospy.Publisher('/part', PoseArray, queue_size = 1)
        # Para mapa 
        self.mapimg = np.array( [] )
        self.map_resolution = 0
        rospy.Subscriber( '/map', OccupancyGrid, self.set_map )
        rospy.Subscriber( '/scan', LaserScan, self.laser_scan_hd, queue_size = 1 )
        self.rate_obj = rospy.Rate( 10 )
        # Para MCL
        self.iteracion = 1
        self.poses = None
        self.X = []
        self.M = 10000
        self.z = None
        self.angles = None
        self.X_barra_x = []
        self.X_barra_w = []
        self.X_barra_w_normalize = []

    def set_map(self, occupancy_grid):
        self.occupancy_grid = occupancy_grid
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        self.map_resolution = occupancy_grid.info.resolution
        print( 'map resolution: (%d,%d)' % (height, width) )
        self.mapimg = 100 - np.array( occupancy_grid.data ).reshape( (height, width) )
        self.mapimg = ( self.mapimg * (255/100.0) ).astype( np.uint8 )
        self.mapimg = cv2.cvtColor( self.mapimg, cv2.COLOR_GRAY2RGB )
        #self.mapimg = cv2.resize( self.mapimg, (width, height), interpolation = cv2.INTER_NEAREST )
        self.obstacles_x = np.where(self.mapimg == 0)[1]
        self.obstacles_x = self.map_resolution*self.obstacles_x + 0.0
        self.obstacles_y = np.where(self.mapimg == 0)[0]
        self.obstacles_y = - self.map_resolution*self.obstacles_y +  2.7
        lista = []
        for i in range(len(self.obstacles_x)):
            lista.append([self.obstacles_x[i], self.obstacles_y[i]])
        self.obstacles = np.asarray(lista)
        self.tree = spatial.cKDTree( self.obstacles )
        self.libres_x = np.where(self.mapimg > 207)[1]
        self.libres_x = self.map_resolution*self.libres_x + 0.0
        self.libres_y = np.where(self.mapimg > 207)[0]
        self.libres_y = - self.map_resolution*self.libres_y +  2.7
        print("Creando particulas...")
        self.create_particles()
        print("Inicio loop")
        rospy.sleep(1)
        self.loop()
        self.finish = True
        print("Termine")
        self.say("Ya se donde estoy cabros")
        print("Termine de hablar")

    def laser_scan_hd(self, scan):
        self.angles = np.linspace( scan.angle_min, scan.angle_max, int( (scan.angle_max-scan.angle_min)/scan.angle_increment ) )
        self.z = scan.ranges
        self.angles = self.angles[:180]
        self.z = self.z[:180]
        self.left_obs = (self.z[76] + self.z[77])/2.0
        self.right_obs = (self.z[104] + self.z[105])/2.0
        self.front_obs = (self.z[90] + self.z[91])/2.0
        self.angles = self.angles[63:117]
        self.z = scan.ranges[63:117]
        if self.right_obs < 0.5 or self.left_obs < 0.5 or self.front_obs < 0.5:
            self.front = True
        else:
            self.front = False
        self.rate_obj.sleep()

    def girar(self):
        if self.contador_vueltas == 4:
            self.girar_45 = True
            self.contador_vueltas = 0
            # aqui giramos a 45 
        else:
            self.contador_vueltas += 1
        tiempo_inicio = time.time()
        if self.girar_45:
            tiempo_giro = 1.1
        else:
            tiempo_giro = 2.2
        while time.time() < tiempo_inicio + tiempo_giro:
            speed = Twist()
            speed.linear.x = 0
            speed.linear.y = 0
            speed.linear.z = 0
            speed.angular.x = 0
            speed.angular.y = 0
            speed.angular.z = self.angulo_speed
            self.cmd_vel_mux_pub.publish(speed)
            self.rate_obj.sleep()

    def avanzar(self, cantidad):
        self.contador_vueltas = 0
        self.girar_45 = False
        tiempo_inicio = time.time()
        tiempo = int(cantidad / 0.1)
        while time.time() < tiempo_inicio + tiempo:
            speed = Twist()
            speed.linear.x = self.dist_speed
            speed.linear.y = 0
            speed.linear.z = 0
            speed.angular.x = 0
            speed.angular.y = 0
            speed.angular.z = 0
            self.cmd_vel_mux_pub.publish(speed)
            self.rate_obj.sleep()

    def movimiento(self):
        tiempo_inicio = time.time()
        if self.front:
            self.girar()
            if self.girar_45:
                self.MCL(0.785)
            else:
                self.MCL(1.57)
            self.girar_45 = False
        else:
            cantidad = min(self.right_obs, self.left_obs, self.front_obs)
            cantidad = cantidad - 0.355
            self.avanzar(round(cantidad, 2))
            self.MCL(cantidad)
        speed = Twist()
        speed.linear.x = 0
        speed.linear.y = 0
        speed.linear.z = 0
        speed.angular.x = 0
        speed.angular.y = 0
        speed.angular.z = 0
        self.cmd_vel_mux_pub.publish(speed)
        tiempo = time.time() - tiempo_inicio
        print(tiempo)

    def send_part(self):
        pose_msg = PoseArray()
        for part in self.X:
            new_pose = Pose()
            new_pose.position.x = float(part[0])
            new_pose.position.y = float(part[1])
            new_pose.position.z = 0.0
            new_pose.orientation.x = 0.0
            new_pose.orientation.y = 0.0
            new_pose.orientation.z = float(part[2])
            new_pose.orientation.w = 0.0
            pose_msg.poses.append(new_pose)
        self.poses = pose_msg

    def measurement_model(self, x):
        angles_nuevos = self.angles + x[2]
        cosenos = np.cos(angles_nuevos)
        senos = np.sin(angles_nuevos)
        z_x = np.multiply(cosenos, self.z)
        z_y = np.multiply(senos, self.z)
        z_x = z_x + x[0]
        z_y = z_y + x[1]
        q = 1
        tuplas = zip(z_x, z_y)
        dist, point_id = self.tree.query( [tuplas] )
        gauss_vect = np.vectorize(self.gaussiana)
        dist_final = gauss_vect(dist[0])
        q = np.prod(dist_final)
        return q

    def measurement_model_antiguo(self, x):
        q = 1
        for angle, zk in zip( self.angles, self.z ):
            if zk < 4.0:
                zkx = zk * np.cos( angle + x[2] ) + x[0]
                zky = zk * np.sin( angle + x[2] ) + x[1]
                dist, point_id = self.tree.query( [[zkx, zky]] )
                # aqui falta el zhit
                q = q*self.gaussiana(dist)
        return q

    def loop(self):
        std_x = 10
        std_y = 10
        speed = Twist()
        speed.linear.x = 0
        speed.linear.y = 0
        speed.linear.z = 0
        speed.angular.x = 0
        speed.angular.y = 0
        speed.angular.z = 0
        while std_x > 0.04 or std_y > 0.04:
            print("Iteracion numero %d" % self.iteracion)
            self.iteracion += 1
            self.movimiento()
            x = np.array(self.X)
            std = np.std(x, axis=0)
            std_x = std[0]
            std_y = std[1]
            self.cmd_vel_mux_pub.publish(speed)
        indicemax = np.argmax(self.X_barra_w_normalize)
        print("Pose mas probable:...")
        print(self.X_barra_x[indicemax])

    def gaussiana(self, valor):
        des = 0.2
        formula = (1/math.sqrt(2*math.pi*(des**2)))*math.exp(-(valor**2)/(2*(des**2)))
        return formula
        
    def MCL(self, u):
        self.X_barra_x = []
        self.X_barra_w = []
        X_nuevo = []
        speed = Twist()
        speed.linear.x = 0
        speed.linear.y = 0
        speed.linear.z = 0
        speed.angular.x = 0
        speed.angular.y = 0
        speed.angular.z = 0
        for i in range(self.M):
            self.cmd_vel_mux_pub.publish(speed)
            x = self.sample_motion_model(u, self.X[i])
            w = self.measurement_model(x)
            self.X_barra_x.append(x)
            self.X_barra_w.append(w)
        norma = sum(self.X_barra_w)
        norma = 1/norma
        X_barra_w_normalize = np.asarray(self.X_barra_w)
        X_barra_w_normalize = X_barra_w_normalize*norma
        self.X_barra_w_normalize = X_barra_w_normalize.tolist()
        for i in range(self.M):
            particula = choice(range(len(self.X_barra_x)), 1, p = self.X_barra_w_normalize)
            copia = self.X_barra_x[particula[0]][:]
            X_nuevo.append(copia)
        self.X = X_nuevo
        self.send_part()
        self.send_part_pub.publish(self.poses)

    def sample_motion_model(self, u, x_anterior):
        if u != 1.57 and u != 0.785:
            sample = random.gauss(u, 0.03)
            x_anterior[0] = x_anterior[0] + sample*math.cos(x_anterior[2])
            x_anterior[1] = x_anterior[1] + sample*math.sin(x_anterior[2])
        else:
            sample = random.gauss(u, 0.17)
            x_anterior[2] = sample + x_anterior[2]
            if x_anterior[2] > math.pi:
                x_anterior[2] = x_anterior[2] - (2*math.pi)
        return x_anterior

    def create_particles(self):
        xy = random.sample(range(len(self.libres_x)), self.M)
        for i in xy:
            angulo = random.randint(-180, 180)
            angulo = angulo*math.pi/180.0
            self.X.append([self.libres_x[i], self.libres_y[i], angulo])
        while self.send_part_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep( 0.2 )
        self.send_part()
        self.send_part_pub.publish(self.poses)
        timer = threading.Timer(1.0, self.timer_particulas)
        timer.start()
        return self.X

    def timer_particulas(self):
        self.send_part_pub.publish(self.poses)
        if not self.finish:
                timer = threading.Timer(3.0, self.timer_particulas)
                timer.start()

    def say( self, s, voice = 'voice_kal_diphone', volume = 40.0 ):
        # rospy.loginfo( 'Saying: \'%s\'', s )
        self.soundhandle.say( s, voice )



if __name__ == '__main__':
    robot = VirtualRobot()
    rospy.spin()




# Optimizacion codigo
# Mejor criterio
