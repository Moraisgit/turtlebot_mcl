#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Versão a funcionar 20/05/2024 

"""
        ####          Monte Carlo Localization          ####
        ####    Graph Node 2 (RMSE and trajectories )   ####
        ####          Sistemas Autónomos                ####
        ####          2ºSemestre 2023/2024 (P4)         ####
        ####          L02    G02                        ####

Autores:    António Vasco Morais de Carvalho,  (ist1102643-LEEC)
            Catarina Andreia Duarte Caramalho, (ist1102644-LEEC)
            Miguel Angélico Gonçalves,         (ist1102539-LEEC)
            Neelam Jaiesh Visueshcumar,        (ist1103376-LEEC)

"""
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
import pandas as pd
import time
import rospy
import yaml
import pandas as pd
import re
import numpy
import math
from nav_msgs.msg import Odometry
import numpy as np
import signal
import tf.transformations as tr
import sys
import matplotlib.animation as animation
from itertools import count
import random
import csv

amcl_angles = []
aux2 = 0
teste = 0
particles_angles=[]
amcl_values = []
amcl_time_values = []
time_values = []
particles_values = []
resolution = 0
origin = [0, 0, 0]
width = 0
height = 0
map = None
fig = None
ax = None
scatter_amcl = None
scatter_particles = None

def conversao_pixeis(x, y):
    a = ((x - origin[0]) / resolution).astype(int)
    x = np.where((a >= width) | (a < 0), 0, a)
    b = (height - ((y - origin[1]) / resolution)).astype(int)
    y = np.where((b >= height) | (b < 0), 0, b)
    return x, y

def amcl_callback(msg):
    position = msg.pose.pose.position
    angle = msg.pose.pose.orientation
    amcl_values.append(position)
    amcl_angles.append(angle)
    tempo = [msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** -9)]
    amcl_time_values.extend(tempo)


def particles_callback(msg):
    positions = [particle.pose.position for particle in msg.markers]
    angle = [particle.pose.orientation for particle in msg.markers]
    particles_values.extend(positions)
    particles_angles.extend(angle)
    tempo = [particle.header.stamp.secs + particle.header.stamp.nsecs * (10 ** -9) for particle in msg.markers]
    time_values.extend(tempo)

def get_position():
    global resolution, origin, width, height, map, scatter_amcl, scatter_particles, aux2, teste

   
    amcl_x = [position.x for position in amcl_values]
    amcl_y = [position.y for position in amcl_values]

    amcl_x_pixels, amcl_y_pixels = conversao_pixeis(amcl_x, amcl_y) 

    
    particles_x = [position.x for position in particles_values]
    particles_y = [position.y for position in particles_values]

    particles_x_pixels, particles_y_pixels = conversao_pixeis(particles_x, particles_y)
  
    scatter_amcl.set_offsets(np.column_stack((amcl_x_pixels, amcl_y_pixels)))
    scatter_particles.set_offsets(np.column_stack((particles_x_pixels, particles_y_pixels)))
    plt.pause(0.005)

    #####RMSE####
    
    if ( len(amcl_y) and len(amcl_x) and len(particles_x) and len(particles_y) ) >0:

        current_time = time.time() - start_time - aux2
        x_vals.append(current_time)
        aux1= math.sqrt((amcl_x[-1]-particles_x[-1])**2+(amcl_y[-1]-particles_y[-1])**2) #cálculo do RMSE 
        if teste == -20:
            for i in range(len(y_vals)):
                y_vals[i] = aux1
            teste = 0

    else:
         aux1= -1
         teste= -20
         x_vals.append(0)
         aux2 += 1

    y_vals.append(aux1)  
    line.set_data(x_vals, y_vals)
    ax.relim()
    ax.autoscale_view()
    plt.draw()


def read_pgm(filename, byteorder='>'):
    global width, height
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder + 'u2',
                            count=int(width) * int(height),
                            offset=len(header) 
                            ).reshape((int(height), int(width))), int(width), int(height)

def initialization():
    global x_vals, y_vals, line, resolution, origin, width, height, map, fig, ax, scatter_amcl, scatter_particles, start_time, aux2

    map, width, height = read_pgm(rospy.get_param("~map_path_pgm", default="/home/morais/turtle_ws/src/turtlebot_mcl/maps/simulation/gmapping_02.pgm"), byteorder='<')

    with open(rospy.get_param("~map_path_yaml", default="/home/morais/turtle_ws/src/turtlebot_mcl/maps/simulation/gmapping_02.yaml"), 'r') as file:
        # Load the YAML contents
        yaml_data = yaml.safe_load(file)

    # Access the parameters
    resolution = yaml_data['resolution']
    origin = np.array(yaml_data['origin'])

    clean_string = str(width)
    width = int(clean_string)

    clean_string = str(height)
    height = int(clean_string)

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.imshow(map, cmap='gray', origin='lower')
    scatter_amcl = ax.scatter([], [], color='red', s=5, label='Robot trajectory (/amcl_pose)')
    scatter_particles = ax.scatter([], [], color='blue', s=5, label='Position Estimate')
    ax.legend()
    plt.xlabel('x (pixels)')
    plt.ylabel('y (pixels)')

    #####RMSE####
    fig, ax = plt.subplots()
    x_vals = []
    y_vals = []

    line, = ax.plot([], [], 'b-')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('RMSE (metres)')
    ax.set_title('RMSE over Time')

    start_time = time.time()

    aux2 = 0

def main():
    plt.ion()
    rospy.init_node('Node_Graphs2', anonymous=True)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback)
    rospy.Subscriber("/Pose", MarkerArray, particles_callback)

    initialization()

    rate = rospy.Rate(1)  #Não alterar, está feito para 1 segundo
    while not rospy.is_shutdown():
        get_position()
        with open('/home/morais/turtle_ws/src/turtlebot_mcl/csv/busca/rmse_data.csv', 'w', newline='') as file:  # Change the path as needed
            writer = csv.writer(file)
            writer.writerow(['Time (seconds)', 'RMSE (meters)'])
            writer.writerows(zip(x_vals, y_vals))
        rate.sleep()


if __name__ == '__main__':
    main()
    
   