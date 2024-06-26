#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Versão a funcionar 20/05/2024 

"""
        ####   Monte Carlo Localization Microsimulator   ####
        ####          Sistemas Autónomos                 ####
        ####          2ºSemestre 2023/2024 (P4)          ####
        ####          L02    G02                         ####
 
Autores:    António Vasco Morais de Carvalho,  (ist1102643-LEEC)
            Catarina Andreia Duarte Caramalho, (ist1102644-LEEC)
            Miguel Angélico Gonçalves,         (ist1102539-LEEC)
            Neelam Jaiesh Visueshcumar,        (ist1103376-LEEC)

"""

import time
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
        self.x_scan = x
        self.y_scan = y
        self.theta_scan = teta

class MonteCarloLocalization(object):
    
    def __init__(self):
        rospy.init_node('MCL_algoritmo', anonymous=True)

        self.robot_x = 0
        self.robot_y = 2
        self.robot_teta = 0
        
        self.wallcolission = 0 # Apenas para kidnapping
        self.weight_average =0 # Apenas para kidnapping
        self.weight_slow =0    # Apenas para kidnapping
        self.weight_fast =0    # Apenas para kidnapping

        self.MkArray = MarkerArray()
        self.MkArraymax = MarkerArray()
        self.MkArrayrobot = MarkerArray()
        # Number of particles from launch file
        self.num_particles = rospy.get_param("~num_particles", default=1000)
        self.width = 0
        self.height = 0
        self.resolution = 0  # Resolução do mapa
        self.origin = []  # Origem do mapa
        self.t1 = 0
        # Sigma value from launch file
        self.sigma = rospy.get_param("~sigma", default = 40)
        self.denominador = np.sqrt(2*np.pi*(self.sigma**90)) #Fixo, logo metendo aqui assim acelera as contas
        self.matrix_pixeis=self.matrix()# contém uma matriz com  valor máximo que um pixel pode ter. Em um arquivo PGM, os valores dos pixels variam de 0 a esse valor máximo
        #o numero de linhas e colunas da matriz = a largura e altura maxima do mapa 
        self.mapa = np.where(self.matrix_pixeis == 205, 0, self.matrix_pixeis) #poe a zero a posição que não faz parte do mapa-cinzento
        self.mapa = scipy.ndimage.distance_transform_edt(self.mapa) #deteta a borda do mapa através da transformada de distância euclidiana    
        self.particles = []
        #lista vazia
        self.particle_weights = np.array([[0.0]*int(round(0.01*self.num_particles)), [0.0]*int(round(0.01*self.num_particles)), [0.0]*int(round(0.01*self.num_particles)), [0.0]*int(round(0.01*self.num_particles))]) #pesos das particulas/ posicao x/ posicao y
#lista de listas de zeros com o número de elementos round(0.01*self.num_particles)
        #self.mcl = mcl
        self.livre = None
        self.vetor = np.argwhere(self.matrix_pixeis == 254) # as zonas que estão a branco são guardadas numa matriz [x,y]
        self.livre=len(self.vetor) #tamanho da matriz
        x = np.random.choice(self.livre, size=self.num_particles, replace=True) #escolhe a posição random para todas as paticulas
        self.posicao = np.array(self.vetor)[x]
        for i in self.posicao:
            x, y = self.posicao_metros(i[1], i[0])
            #teta = np.random.uniform(-np.pi, 0)
            teta = np.random.uniform(0, 2*np.pi)
            self.particles.append(Particle(x, y, teta, 1/self.num_particles)) #criam-se particulas com peso igual
        self.pose = rospy.Publisher("/Pose", MarkerArray, queue_size=1)
        self.particle_pos = rospy.Publisher("/particles", MarkerArray, queue_size=1)
        self.robot_pos = rospy.Publisher("/robotpose", MarkerArray, queue_size=1)
        
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
        if (linear_velocity >= 0.001 or linear_velocity <= -0.001 or angular_velocity >= 0.01 or angular_velocity <= -0.01):
            particle.theta = particle.theta + (angular_velocity) * delta_t + ntheta
            particle.x = particle.x + (linear_velocity) * np.cos(particle.theta) *delta_t + nx
            particle.y = particle.y + (linear_velocity) * np.sin(particle.theta) * delta_t + ny

    def posicao_pixeis(self, x, y):
        a = ((x - self.origin[0]) / self.resolution).astype(int)
        x = np.where((a >= self.width) | (a < 0), 0, a)
        b = (self.height - ((y - self.origin[1]) / self.resolution)).astype(int)
        y = np.where((b >= self.height) | (b < 0), 0, b)
        return x, y
    
    
    def forever_running_scan(self):
         
        self.laser_data_range_min = 0.012
        self.laser_data_range_max = 3.5

        ranges_interval = self.raycasting(self.robot_x, self.robot_y, self.robot_teta, self.laser_data_range_max)
        for i in range(len(ranges_interval[:,2])):
            ranges_interval[i][2] += np.random.normal(0, 0.006)

        
        #ranges_clipped = np.clip(ranges_interval, self.laser_data_range_min , self.laser_data_range_max)
    
        for particle in self.particles:
            particle.x_scan = particle.x #guardar o momento que atualizou
            particle.y_scan = particle.y
            particle.theta_scan = particle.theta

        for particle in self.particles:
            self.diferenca(ranges_interval[:,2], particle)
   
        self.when_to_resample()

            

    def diferenca(self,ranges_clipped,particle):

        #excluir valores que nunca poderão ser
        #linha, coluna = self.posicao_pixeis_pixeis(particle.x_scan,particle.y_scan)
        linha,coluna = self.grid_to_pixel_conversion(particle.x_scan,particle.y_scan)
        if self.matrix_pixeis[coluna][linha] == 205 or  self.matrix_pixeis[coluna][linha] == 0 : #poe a zero a posição que não faz parte do mapa que está a cinzento ou preto
            particle.weight = 0
            self.wallcolission+=1
            return
         
        predicted_ranges = self.raycasting(particle.x_scan, particle.y_scan, particle.theta_scan,self.laser_data_range_max)
        predicted_ranges = np.array(predicted_ranges)
       
        ranges_clipped = np.around(ranges_clipped, 2)
        diff = ranges_clipped -  predicted_ranges[:,2] 
        diff = np.array(diff)
        
        diff_transpose = diff[:, np.newaxis] 
        
        numerador = np.exp(-0.5*np.dot(diff, diff_transpose)*(1/self.sigma))
      
        particle.weight = (numerador[0] / self.denominador) * particle.weight

        for indice, peso in enumerate(self.particle_weights[0,:]):
            if particle.weight > peso:
                aux_1 = indice
                break
            aux_1 = -1

        if aux_1 != -1:
            self.particle_weights[0,aux_1] = particle.weight
            self.particle_weights[1,aux_1] = particle.x
            self.particle_weights[2,aux_1] = particle.y
            self.particle_weights[3,aux_1] = particle.theta


    def when_to_resample(self):
        all_weight = sum([float(particle.weight) for particle in self.particles])
        #print("Peso total", all_weight)
        if all_weight == 0:
            #self.particles = self.reeiniciar(self.particle_weights[:,0])
            print("Particulas a ZERO")
            self.particles = self.resample(self.num_particles)
        else: 
            self.weight_average = 0 # Apenas para kidnapping
            for particle in self.particles:
                    #particle.weight = particle.weight / all_weight
                    self.weight_average += particle.weight *(1/(self.num_particles-self.wallcolission)) # Apenas para kidnapping
                    particle.weight = particle.weight / all_weight
            self.wallcolission = 0 # Apenas para kidnapping
            novas_particulas = 0   # Apenas para kidnapping
             # Get alpha_fast and alpha_slow from launchfile
            self.weight_fast += rospy.get_param("~alpha_fast", default=0.8) * (self.weight_average - self.weight_fast) # Apenas para kidnapping era 0.8
            self.weight_slow += rospy.get_param("~alpha_slow", default=0.2) * (self.weight_average - self.weight_slow) # Apenas para kidnapping
            #print(self.weight_fast)
            #print(self.weight_slow)
            n_eff = 1 / sum([particle.weight**2 for particle in self.particles])
            self.publish_weight()
            for i in range(len(self.particle_weights[0,:])):
                    self.particle_weights[0,i] = 0
            if n_eff <= 0.5 * self.num_particles:
                print("RESAMPLE")
                b = np.random.uniform(0,1, size = self.num_particles)    # Apenas para kidnapping
                maximo = max(0, 1.0 - self.weight_fast/self.weight_slow) # Apenas para kidnapping
                print(maximo)
                for i in b:                 # Apenas para kidnapping
                    if(i < maximo):         # Apenas para kidnapping
                        print(i,maximo)
                        novas_particulas+=1 # Apenas para kidnapping
                
                self.particles = self.resample(novas_particulas)

    def publish_weight(self):
        self.MkArraymax.markers = []
        if self.livre is not None:  
                marker_index = 0; 
                x = 0
                y = 0
                div = 0
                for i in range(len(self.particle_weights[0,:])):
                    x += self.particle_weights[1,i]*self.particle_weights[0,i]
                    y += self.particle_weights[2,i]*self.particle_weights[0,i]
                    while(self.particle_weights[3,i] > np.pi or self.particle_weights[3,i] < -np.pi):
                        if self.particle_weights[3,i] > np.pi:
                            self.particle_weights[3,i] -= 2*np.pi
                        elif self.particle_weights[3,i] < -np.pi:
                            self.particle_weights[3,i] += 2*np.pi
                    div += self.particle_weights[0,i]
                soma_seno = sum(w * math.sin(angle) for w, angle in zip(self.particle_weights[0,:], self.particle_weights[3,:]))
                soma_cosseno = sum(w * math.cos(angle) for w, angle in zip(self.particle_weights[0,:], self.particle_weights[3,:]))
                theta = math.atan2(soma_seno, soma_cosseno)
                x = x/div
                y = y/div
                qz = tf.transformations.quaternion_from_euler(0, 0, theta)[2]
                qw = tf.transformations.quaternion_from_euler(0, 0, theta)[3]
                marker = Marker()
                marker.action = Marker.ADD
                marker.type = Marker.SPHERE
                marker.id = 1
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.orientation.w = qw
                marker.pose.orientation.z = qz
                marker.color.a = 0.5
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "map"
                self.MkArraymax.markers.append(marker)
                marker_index += 1
                if marker_index > 11:
                    self.MkArraymax.markers.pop(0)
                self.pose.publish(self.MkArraymax.markers)

    def publish_robot(self):
        self.MkArrayrobot.markers = []
        if self.livre is not None:  
                marker_index = 0; 
                qz = tf.transformations.quaternion_from_euler(0, 0, self.robot_teta)[2]
                qw = tf.transformations.quaternion_from_euler(0, 0, self.robot_teta)[3]
                marker = Marker()
                marker.action = Marker.ADD
                marker.type = Marker.SPHERE
                marker.id = 1
                marker.pose.position.x = self.robot_x
                marker.pose.position.y = self.robot_y
                marker.pose.orientation.w = qw
                marker.pose.orientation.z = qz
                marker.color.a = 0.5
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "map"
                self.MkArrayrobot.markers.append(marker)
                marker_index += 1
                if marker_index > 11:
                    self.MkArrayrobot.markers.pop(0)
                self.robot_pos.publish(self.MkArrayrobot.markers)


    def resample(self, novas_particulas):

        new_particles = []

        if(novas_particulas != self.num_particles):
            print("normal")
            r = np.random.uniform(0, 1/(self.num_particles-novas_particulas))
            c = self.particles[0].weight    
            i = 0
            for m in range(self.num_particles-novas_particulas):
                u = r + m * (1/(self.num_particles-novas_particulas))
                while u >= c:
                    i += 1
                    c += self.particles[i].weight  
                new_particles.append(Particle(self.particles[i].x, self.particles[i].y, self.particles[i].theta, 1/self.num_particles))

        if novas_particulas > 0:
            print("NOVAS PARTICULAS A SEREM GERADAS:  ---->", novas_particulas)
            for i in range(novas_particulas):
                h = np.random.choice(self.livre, size = 1, replace=True)
                x,y = self.posicao_metros(self.vetor[h,1], self.vetor[h,0])
                theta = np.random.uniform(0, 2*np.pi)
                new_particles.append(Particle(x[0], y[0], theta, 1/self.num_particles))  
        
        return new_particles
    

    def raycasting(self, x, y, theta, laser_max_range):
        """
        Perform ray casting to calculate ranges from a robot to objects or obstacles in its environment using laser range sensors.

        Args:
        - x (float): X-coordinate of the robot's position.
        - y (float): Y-coordinate of the robot's position.
        - theta (float): Orientation angle of the robot (in radians).
        - laser_max_range (float): Maximum range of the laser sensor.

        Returns:
        - numpy.ndarray: Array containing the calculated ranges for each ray.
        """
        # Initialize an empty list to store the calculated ranges
        ranges = []
        # Define ray casting angles in radians (360 degrees with a step size of 4 degrees)
        angles = np.radians(np.arange(0, 360, 4))
        # Calculate absolute ray casting angles relative to the robot's orientation
        phi_angles = theta + angles
        # Generate range values from 0 to laser_max_range with a step size of self.resolution
        range_values = np.arange(0, laser_max_range, self.resolution)
        # Calculate the x and y coordinates (in grid type) of the endpoints of each ray
        x_coordinates = x + np.outer(range_values, np.cos(phi_angles))
        y_coordinates = y + np.outer(range_values, np.sin(phi_angles))
        # Convert grid coordinates to pixel coordinates
        x_pixels, y_pixels = self.grid_to_pixel_conversion(x_coordinates, y_coordinates)
        # Find wall indices (0 indicates free space, 205 indicates obstacles)
        mask = (self.matrix_pixeis[y_pixels, x_pixels] == 0) | (self.matrix_pixeis[y_pixels, x_pixels] == 205)
        # Find the index of the first occurrence of True along each column of the mask
        hit_indices = np.argmax(mask, axis=0)
        # Iterate over hit indices to extract information about obstacles encountered by the rays
        for i, hit_index in enumerate(hit_indices):
            if hit_index > 0:  # If an intersection (hit) is found
                row = x_pixels[hit_index, i]
                col = y_pixels[hit_index, i]
                range_val = range_values[hit_index]
            else:  # If no intersection is found within the sensor's range
                row = x_pixels[-1, i]
                col = y_pixels[-1, i]
                range_val = laser_max_range
            # Append the extracted information as [row, col, range] to the ranges list
            ranges.append([row, col, range_val])
        # Convert the ranges list to a numpy array and return it
        ranges = np.array(ranges)

        return ranges
    
    def grid_to_pixel_conversion(self, x_coordinates, y_coordinates): #same as posicao_pixeis
        """
        Convert grid coordinates to pixel coordinates.

        Args:
            x_coordinates (numpy.ndarray): Array of x coordinates in the grid.
            y_coordinates (numpy.ndarray): Array of y coordinates in the grid.

        Returns:
            Tuple[numpy.ndarray, numpy.ndarray]: Arrays of corresponding x and y pixel coordinates.
        """
        # Calculate pixel coordinates for x axis
        # Subtract the origin from each point to normalize it, divide by the resolution to convert, cast to int because it is float
        aux_x_pixel = ((x_coordinates - self.origin[0]) / self.resolution).astype(int)
        # Apply boundary conditions for x pixel coordinates
        x_pixels = np.where((aux_x_pixel >= self.width) | (aux_x_pixel < 0), 0, aux_x_pixel)
        # Calculate pixel coordinates for y axis
        # Subtract the origin from each point to normalize it, divide by the resolution to convert, cast to int because it is float
        aux_y_pixel = (self.height - ((y_coordinates - self.origin[1]) / self.resolution)).astype(int)
        # Apply boundary conditions for y pixel coordinates
        y_pixels = np.where((aux_y_pixel >= self.height) | (aux_y_pixel < 0), 0, aux_y_pixel)
        
        return x_pixels, y_pixels

    def matrix(self):
        matriz_pixels = self.ler_arquivo_pgm(rospy.get_param("~map_path_pgm", default="/home/morais/turtle_ws/src/turtlebot_mcl/maps/real/mapa_2.pgm"), byteorder='<')
        #pyplot.imshow(matriz_pixels, pyplot.cm.gray)
        #pyplot.show()
        self.ler_arquivo_yaml(rospy.get_param("~map_path_yaml", default="/home/morais/turtle_ws/src/turtlebot_mcl/maps/real/mapa_2.yaml"))
        return matriz_pixels
    
    def continua (self):
        #self.publish_robot()
        self.publish()
        time.sleep(2)

        #miguel       
        #angular_velocity =  [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.2,-0.2,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.0,0.0] #miguel
        #bateu na parede e notou logo:
        #angular_velocity =  [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.2,-0.2,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.0,0.0] #bateu na parede e notou logo
       
        #exemplo do que não bate na parede para o kidnapping:
        #angular_velocity =  [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.2,-0.2,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.3,0.0,0.0,0.0,-0.3,-0.3,-0.3] #exemplo do kidnapping
       
        #este bate na parede
        #angular_velocity =  [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.2,-0.2,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3] #bate na parede
        
        #exemplo do que não bate na parede para o kidnapping:
        
        angular_velocity =  [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.2,-0.2,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,0.0,0.0] #exemplo do que não bate na parede para o kidnapping
        
        #angular_velocity_1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3] # caminho engraçado no final acha duas pisições iguais
        angular_velocity_1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.3,0.3,0.3,0.3,0.5,0.5,0.5,0.5,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.5,0.0,0.0,0.0,0.0,0.3,0.3,0.3,0.5,0.5,0.5,0.5,0.5,0.3,0.3,0.3,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.0,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,0.5-0.5,-0.5,-0.5,-0.5,-0.5,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.1,0.1,0.1] # caminho kidnapping
        angular_velocity_1 = np.negative(angular_velocity_1)
        #para o quadrado:
        #angular_velocity =  [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,-0.2,-0.2,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,0.0,0.0,0.3,0.3,0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] #exemplo quadrado

        linear_velocity_1 =  0.4
        linear_velocity =   0.4
        delta_t =           0.2
        aux_2 = 0
        while not rospy.is_shutdown():
            
            self.robot_teta =  self.robot_teta + (angular_velocity[aux_2]) * delta_t
            self.robot_x = self.robot_x + (linear_velocity) * np.cos(self.robot_teta) * delta_t
            self.robot_y = self.robot_y + (linear_velocity) * np.sin(self.robot_teta) * delta_t
            #print("Robot anda:",self.robot_x, self.robot_y, self.robot_teta,angular_velocity[aux_2])
            self.publish_robot()

            #print("Odometria")
            for particle in self.particles:
                self.predict_odometry (particle,angular_velocity[aux_2],linear_velocity,delta_t)

            #print("Laser Scan")
            self.forever_running_scan()

            self.publish()
            aux_2 += 1
            time.sleep(0.2)

            if aux_2 == len(angular_velocity) - 1:
                aux_2 = 0
                break
        
        
        # #Simulate kidnapping
        # self.robot_x = 1.8
        # self.robot_y = 4
        # self.robot_teta = -1.2
        
        # while not rospy.is_shutdown():

        #     self.robot_teta =  self.robot_teta + (angular_velocity_1[aux_2]) * delta_t
        #     self.robot_x = self.robot_x + (linear_velocity_1) * np.cos(self.robot_teta) * delta_t
        #     self.robot_y = self.robot_y + (linear_velocity_1) * np.sin(self.robot_teta) * delta_t
        #     #print("Robot anda:",self.robot_x, self.robot_y, self.robot_teta,angular_velocity_1[aux_2])
        #     self.publish_robot()

        #     #print("Odometria")
        #     for particle in self.particles:
        #         self.predict_odometry (particle,angular_velocity_1[aux_2],linear_velocity_1,delta_t)

        #     #print("Laser Scan")
        #     self.forever_running_scan()

        #     self.publish()
        #     aux_2 += 1
        #     time.sleep(0.2)

        #     if aux_2 == len(angular_velocity_1) - 1:
        #         aux_2 = 0
        #         break
        
        print("FIM")

if __name__ == "__main__":
    mcl = MonteCarloLocalization()
    #mcl.matrix()
    mcl.continua()