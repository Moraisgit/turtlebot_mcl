#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Versão a funcionar 18/05/2024 

"""
        ####          Monte Carlo Localization            ####
        ####  Graph Node 1 (Particles and Robot Position) ####
        ####          Sistemas Autónomos                  ####
        ####          2ºSemestre 2023/2024 (P4)           ####
        ####          L02    G02                          ####

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
import numpy as np


amcl_values = []
time_values= []
amcl_time_values= []
allParticles_values = []
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
    global amcl_values, amcl_time_values
    amcl_values = []
    position = msg.pose.pose.position
    amcl_values.append(position)
    tempo = [msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** -9)]
    amcl_time_values.extend(tempo)

def allParticles_callback(msg):
    global allParticles_values
    allParticles_values = []
    positions = [particle.pose.position for particle in msg.markers]
    allParticles_values.extend(positions)
    tempo = [particle.header.stamp.secs + particle.header.stamp.nsecs * (10 ** -9)for particle in msg.markers]
    time_values.extend(tempo)

def clear_scatter():
    plt.pause(0.2)
    plt.cla()


def allParticles_position():
    global resolution, origin, width, height, map, scatter_amcl, scatter_particles
    
    # Plot the AMCL positions
    amcl_x = [position.x for position in amcl_values]
    amcl_y = [position.y for position in amcl_values]
    amcl_x_pixels, amcl_y_pixels = conversao_pixeis(amcl_x, amcl_y) 
    
    # Plot the particle positions
    particles_x = [position.x for position in allParticles_values]
    particles_y = [position.y for position in allParticles_values]
    particles_x_pixels, particles_y_pixels = conversao_pixeis(particles_x, particles_y)
    
    plt.scatter(particles_x_pixels, particles_y_pixels, color='green', label='Particles positions',s=6,alpha=0.6)
    plt.scatter(amcl_x_pixels, amcl_y_pixels, color='blue', label='Robot position',s=20)
    plt.legend()
    plt.xlabel('x (pixels)')
    plt.ylabel('y (pixels)')

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

def plot_map():
    global map
    ax.imshow(map, cmap='gray', origin='lower')


def initialization():
    global resolution, origin, width, height, map, fig, ax, scatter_amcl, scatter_particles,scatter_dead_reckoning

    map, width, height = read_pgm("/media/sf_VM_Sauto/Dados_Novos/maps/simulation/house_gazebo.pgm", byteorder='<')

    with open("/media/sf_VM_Sauto/Dados_Novos/maps/simulation/house_gazebo.yaml", 'r') as file:
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
    scatter_amcl = ax.scatter([], [], color='blue', s=5)
    scatter_particles = ax.scatter([], [], color='green', s=5)
   

def main():
    plt.ion()
    rospy.init_node('Node_Graphs1', anonymous=True)

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback)
    rospy.Subscriber("/particles", MarkerArray, allParticles_callback)
    initialization()
    

    rate = rospy.Rate(1)  
    
    while not rospy.is_shutdown():
        allParticles_position()
        plot_map()
        clear_scatter()
        rate.sleep()


if __name__ == '__main__':
    main()
    
   