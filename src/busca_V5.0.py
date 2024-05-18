#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Versão a funcionar 18/05/2024 

"""
        ####    Monte Carlo Localization Node       ####
        ####        Sistemas Autónomos              ####
        ####        2ºSemestre 2023/2024 (P4)       ####
        ####        L02    G02                      ####

Autores:    António Vasco Morais de Carvalho,  (ist1102643-LEEC)
            Catarina Andreia Duarte Caramalho, (ist1102644-LEEC)
            Miguel Angélico Gonçalves,         (ist1102539-LEEC)
            Neelam Jaiesh Visueshcumar,        (ist1103376-LEEC)

"""

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
import re
import numpy
import numpy as np
import sys
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import bagpy
import pickle
import yaml
import random
from bagpy import bagreader
from matplotlib import pyplot
import pandas as pd
from visualization_msgs.msg import Marker, MarkerArray
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
    
    def __init__(self, num_particles):
        rospy.init_node('MCL_algoritmo', anonymous=True)
        self.MkArray = MarkerArray()
        self.MkArrayweight = MarkerArray()
        self.num_particles=num_particles
        self.width = 0
        self.height = 0
        self.resolution = 0  # Resolução do mapa
        self.origin = []  # Origem do mapa
        self.t1 = 0
        self.sigma = 60
        self.matrix_pixeis=self.matrix()# contém uma matriz com  valor máximo que um pixel pode ter. Em um arquivo PGM, os valores dos pixels variam de 0 a esse valor máximo
        #o numero de linhas e colunas da matriz = a largura e altura maxima do mapa 
        self.mapa = np.where(self.matrix_pixeis == 205, 0, self.matrix_pixeis) #poe a zero a posição que não faz parte do mapa-cinzento
        self.mapa = scipy.ndimage.distance_transform_edt(self.mapa) #deteta a borda do mapa através da transformada de distância euclidiana    
        self.particles = []
        #lista vazia
        self.colission = 0 # Apenas para kidnapping
        self.weight_average = 0 # Apenas para kidnapping
        self.weight_slow = 0    # Apenas para kidnapping
        self.weight_fast = 0    # Apenas para kidnapping
        self.particle_weights = np.array([[0.0]*int(round(0.01*self.num_particles)), [0.0]*int(round(0.01*self.num_particles)), [0.0]*int(round(0.01*self.num_particles)), [0.0]*int(round(0.01*self.num_particles))]) #pesos das particulas/ posicao x/ posicao y
        self.scan_data = None 
        self.livre = None
        self.denominador = np.sqrt(2*np.pi*(self.sigma**90)) 
        self.vetor = np.argwhere(self.matrix_pixeis == 254) # as zonas que estão a branco são guardadas numa matriz [x,y]
        self.livre=len(self.vetor) #tamanho da matriz
        x = np.random.choice(self.livre, size=self.num_particles, replace=True) #escolhe a posição random para todas as paticulas
        self.posicao = np.array(self.vetor)[x]
        for i in self.posicao:
            x, y = self.posicao_metros(i[1], i[0])
            teta = np.random.uniform(0, 2*np.pi)
            self.particles.append(Particle(x, y, teta, 1/self.num_particles)) #criam-se particulas com peso igual
        rospy.Subscriber("/odom", Odometry, self.odometry_callback,queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback,queue_size=1)
        self.pose = rospy.Publisher("/Pose", MarkerArray, queue_size=1)
        self.particle_pos = rospy.Publisher("/particles", MarkerArray, queue_size=1)
        
    def posicao_metros(self,x,y):
        x = x * self.resolution + self.origin[0]  
        y = (self.height - y) * self.resolution + self.origin[1]
        return x, y

    def as_pose(self,particle):
        # A helper function to convert a particle to a geometry_msgs/Pose message 
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
        nx = np.random.normal(0, 0.009) #MUDA
        ny = np.random.normal(0, 0.009) #MUDA
        ntheta = np.random.normal(0, 0.006) #MUDA
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
        if self.scan_data is not None: 
            self.laser_data_range_min = self.scan_data.range_min
            self.laser_data_range_max = self.scan_data.range_max
            # diminuir o numero de amostras com intervalo de 4 em 4
            ranges_interval = np.array(self.scan_data.ranges)
            ranges_interval = np.array(ranges_interval[::4]) # escapa lasers de 4 em 4
            ranges_clipped = np.clip(ranges_interval, self.laser_data_range_min , self.laser_data_range_max)
    
            if (self.linear_velocity >= 0.001 or self.linear_velocity <= -0.001 or self.angular_velocity >= 0.01 or self.angular_velocity <= -0.01): 
                for particle in self.particles:
                    particle.x_scan = particle.x #guardar o momento que atualizou
                    particle.y_scan = particle.y
                    particle.theta_scan = particle.theta

                for particle in self.particles:
                    self.diferenca(ranges_clipped, particle)
   
                self.when_to_resample()

            self.scan_data = None 

    def diferenca(self,ranges_clipped,particle):

        #excluir valores que nunca poderão ser
        linha,coluna = self.grid_to_pixel_conversion(particle.x_scan,particle.y_scan)
        if self.matrix_pixeis[coluna][linha] == 205 or  self.matrix_pixeis[coluna][linha] == 0 : #poe a zero a posição que não faz parte do mapa que está a cinzento ou preto
            particle.weight = 0
            self.colission+=1

            return
         
        predicted_ranges = self.raycasting(particle.x_scan, particle.y_scan, particle.theta_scan,self.laser_data_range_max)
        predicted_ranges = np.array(predicted_ranges)
       
        ranges_clipped = np.around(ranges_clipped, 2)
        diff = ranges_clipped -  predicted_ranges 
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
        if all_weight == 0:
            self.particles = self.resample(self.num_particles)
        else: 
            self.weight_average = 0 # Apenas para kidnapping
            for particle in self.particles:
                    self.weight_average += particle.weight *(1/(self.num_particles-self.colission)) # Apenas para kidnapping
                    particle.weight = particle.weight / all_weight

            self.colission = 0 # Apenas para kidnapping
            novas_particulas = 0   # Apenas para kidnapping
            self.weight_fast += 0.8 * (self.weight_average - self.weight_fast) # Apenas para kidnapping era 0.8
            self.weight_slow += 0.2 * (self.weight_average - self.weight_slow) # Apenas para kidnapping era 0.2
            n_eff = 1 / sum([particle.weight**2 for particle in self.particles])
            self.publish_weight()
            for i in range(len(self.particle_weights[0,:])):
                    self.particle_weights[0,i] = 0
            if n_eff <= 0.5 * self.num_particles:
                b = np.random.uniform(0,1, size = self.num_particles)    # Apenas para kidnapping
                maximo = max(0, 1.0 - self.weight_fast/self.weight_slow) # Apenas para kidnapping
                for i in b:                 # Apenas para kidnapping
                    if(i < maximo):         # Apenas para kidnapping
                        novas_particulas+=1 # Apenas para kidnapping
                self.particles = self.resample(novas_particulas)

    def publish_weight(self):
        self.MkArrayweight.markers = []
        if self.livre is not None:  
                marker_index = 0; 
                div = 0
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

   
                marker = Marker(header=Header(stamp=rospy.Time.now(),
                                    frame_id="map"),
                                    action = Marker.ADD,
                                    pose=Pose(position=Point(x=x, y=y, z=0),
                                    orientation=Quaternion(x=0, y=0, z=tf.transformations.quaternion_from_euler(0, 0, theta)[2],w=tf.transformations.quaternion_from_euler(0, 0, theta)[3])),
                                    type=Marker.SPHERE,
                                    id=1,
                                    scale=Vector3(x=0.1,y=0.1,z=0.1),
                                    color=ColorRGBA(r=1,a=1,b=1,g=1))
                self.MkArrayweight.markers.append(marker)
                marker_index += 1
                if marker_index > 11:
                    self.MkArrayweight.markers.pop(0)
                self.pose.publish(self.MkArrayweight.markers)

    def resample(self, novas_particulas):
        new_particles = []
        
        if(novas_particulas != self.num_particles):
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
        self.scan_data = msg

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
                range_val = range_values[hit_index]
            else:  # If no intersection is found within the sensor's range
                range_val = laser_max_range
            # Append the extracted information a range to the ranges list
            ranges.append(range_val)
        # Convert the ranges list to a numpy array and return it
        ranges = np.array(ranges)
        #self.draw_line_until_dark_dot(x_coordinates,y_coordinates,ranges)

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
        matriz_pixels = self.ler_arquivo_pgm("/media/sf_VM_Sauto/Dados_Novos/maps/simulation/house_gazebo.pgm", byteorder='<')
        self.ler_arquivo_yaml("/media/sf_VM_Sauto/Dados_Novos/maps/simulation/house_gazebo.yaml")
        return matriz_pixels


    def continua (self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.forever_running_scan()
            self.publish()
            rate.sleep()

if __name__ == "__main__":
    num_particles = 1000
    mcl = MonteCarloLocalization(num_particles)
    #mcl.matrix()
    mcl.continua()
