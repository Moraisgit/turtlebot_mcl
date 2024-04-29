#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import tf.transformations as tr
import math
import os
from std_msgs.msg import String, Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt, cos, sin, pi, atan2
from threading import Thread, Lock
from math import pi, log, exp
import random
import numpy as np
import sys
import rospy
import bagpy
import pickle
import yaml
import random
from bagpy import bagreader
from matplotlib import pyplot
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import pandas as pd
from visualization_msgs.msg import Marker, MarkerArray
import re
import numpy
import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Point, Quaternion

from random import gauss
import scipy
from scipy import ndimage
from matplotlib import transforms

class Particle(object):
    def __init__(self, x, y, teta, peso):
        self.weight = peso
        self.x = x    
        self.y = y
        self.theta = teta
        #self.x_laser = x
        #self.y_laser = y
        #self.theta_laser = teta
        
        




class MonteCarloLocalization(object):
    
    def __init__(self, num_particles):

        rospy.init_node('MCL_algoritmo', anonymous=True)
        self.MkArray = MarkerArray()
        self.MkArraymax = MarkerArray()
        self.num_particles=num_particles
        self.width = 0
        self.height = 0
        self.t1 = 0
        self.matrix_pixeis=self.matrix()# contém uma matriz com  valor máximo que um pixel pode ter. Em um arquivo PGM, os valores dos pixels variam de 0 a esse valor máximo
        #o numero de linhas e colunas da matriz = a largura e altura maxima do mapa 

        self.mapa = np.where(self.matrix_pixeis == 205, 0, self.matrix_pixeis) #poe a zero a posição que não faz parte do mapa-cinzento
        self.mapa = scipy.ndimage.distance_transform_edt(self.mapa) #deteta a borda do mapa através da transformada de distância euclidiana    
        self.particles = []

        #lista vazia
        #lista de listas de zeros com o número de elementos round(0.01*self.num_particles)
        #self.mcl = mcl
        self.livre = None
        self.vetor = np.argwhere(self.matrix_pixeis == 254) # as zonas que estão a branco são guardadas numa matriz [x,y]
        self.livre=len(self.vetor) #tamanho da matriz
        x = np.random.choice(self.livre, size=self.num_particles, replace=True) #escolhe a posição random para todas as paticulas
        self.posicao = np.array(self.vetor)[x]
        for i in self.posicao:
            x, y = self.posicao_metros(i[1], i[0])
            teta = np.random.uniform(-np.pi, 0)
            self.particles.append(Particle(x, y, teta, 1/self.num_particles)) #criam-se particulas com peso igual
        rospy.Subscriber("/odom", Odometry, self.odometry_callback,queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback,queue_size=1)
        self.max_weight = rospy.Publisher("/MaxWeight", MarkerArray, queue_size=1)
        self.particle_pos = rospy.Publisher("/particles", MarkerArray, queue_size=1)
        
    def posicao_metros(self,x,y):
        x = x * self.resolution + self.origin[0] # chat 
        y = (self.height - y) * self.resolution + self.origin[1]
        return x, y

    def as_pose(self,particle,):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,particle.theta)
        return Pose(position=Point(x=particle.x,y=particle.y,z=0), orientation=Quaternion(orientation_tuple[0], orientation_tuple[1], orientation_tuple[2], orientation_tuple[3]))

    
  
    def publish(self):
        self.MkArray.markers = []

        marker_array = []
        if self.livre is not None:
            marker_index=0

            for index, particle in enumerate(self.particles):
                marker = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id="map"),
                                    action = Marker.ADD,
                                    pose=self.as_pose(particle),
                                    type=Marker.ARROW,
                                    id=index,scale=Vector3(x=0.15,y=0.02,z=0.02),
                                    color=ColorRGBA(r=1,a=1,b=1,g=1))
                marker_array.append(marker)
               
                self.MkArray.markers.append(marker)

                marker_index += 1
                if marker_index > self.num_particles + 1:
                    self.MkArray.markers.pop(0)

            self.particle_pos.publish(self.MkArray)

      
    def ler_arquivo_pgm(self, caminho_arquivo, byteorder='>'): #standard stackoverflow
        with open(caminho_arquivo, 'rb') as arquivo:
            buffer = arquivo.read()
        try:
            header, self.width, self.height, max_val = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
        except AttributeError:
            raise ValueError("Not a raw PGM file: '%s'" % caminho_arquivo)
        self.width = int(self.width) # são strings tem de se converter em int para se poder fazer calculos com posicao_metros
        self.height = int(self.height)

        return numpy.frombuffer(buffer,
                            dtype='u1' if int(max_val) < 256 else byteorder+'u2',
                            count=int(self.width)*int(self.height),
                            offset=len(header)
                            ).reshape((int(self.height), int(self.width)))
    
    def ler_arquivo_yaml(self,caminho_arquivo):

        # Abrir o arquivo YAML e carregar os dados
        with open(caminho_arquivo, 'r') as arquivo:
            dados = yaml.safe_load(arquivo)

        # Acessar os dados
        self.resolution = dados['resolution'] # quantos pixels correspondem a uma unidade de medida, como metros
        self.origin = dados['origin']

    def predict_odometry (self,particle,angular_velocity,linear_velocity,delta_t):

        nx = np.random.normal(0, 0.006)
        ny = np.random.normal(0, 0.006)
        ntheta = np.random.normal(0, 0.003)
        
       # particle.theta = particle.theta + (angular_velocity) * delta_t + ntheta
       # particle.x = particle.x + (linear_velocity) * np.cos(particle.theta) *delta_t + nx
       # particle.y = particle.y + (linear_velocity) * np.sin(particle.theta) * delta_t + ny
    
      
  
    def odometry_callback(self,msg):


        # Acessando a velocidade linear e angular e timestamp
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z
        
        self.timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** -9)
        
            
        if self.t1 != 0 and self.livre is not None:
            for particle in self.particles:
                self.predict_odometry(particle,self.angular_velocity ,self.linear_velocity, self.timestamp-self.t1)
        self.t1 = self.timestamp
    
    





    def scan_callback(self,msg):
        self.scan = msg

    def matrix(self):
        matriz_pixels = self.ler_arquivo_pgm("/home/morais/mcl_examples/MonteCarloLocalization/maps/gmapping_02.pgm", byteorder='<')

        #pyplot.imshow(matriz_pixels, pyplot.cm.gray)
        #pyplot.show()
        self.ler_arquivo_yaml("/home/morais/mcl_examples/MonteCarloLocalization/maps/gmapping_02.yaml")
        

        return matriz_pixels
    def continua (self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #self.laser_scan_update()
            self.publish()
            rate.sleep()

if __name__ == "__main__":
    num_particles = 600
    mcl = MonteCarloLocalization(num_particles)
    #mcl.matrix()
    mcl.continua()