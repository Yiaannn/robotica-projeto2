from random import randint, choice
import math
import random
from pf import Particle
import numpy as np
import matplotlib.pyplot as plt
import cv2
import inspercles
reload(inspercles)

def create_particles(minx, miny, maxx, maxy, n):
    """
        Cria num particulas
        situadas no intervalo x - var_x a x + var_x, y - var_x at'e y + var_y e theta - var_theta a theta + var_theta
    """
    particle_cloud = []
    for i in range(n):
        x = random.uniform(minx, maxx)
        y = random.uniform(miny, maxy)
        
        theta = random.uniform(0, 2*math.pi)
        p = Particle(float(x), float(y), float(theta), w=1.0) # A prob. w vai ser normalizada depois
        particle_cloud.append(p)
    return particle_cloud
    
movements = [[1, 0, 0], [0, 1, 0], [-2,0,0], [0, -3, 0],
          [5,0,math.pi/12.0], [0, 8, math.pi/12.0], [-13, 0, math.pi/12],[0, -21,-math.pi/4],
          [34, 0, 0],[55,0,0], [-89,0,0], [-144,0,0],
          [0,0,-math.radians(90)],
          ]

def apply_uncertainty(value): 
    value+= np.random.normal(loc=0.0, scale=1.5)
    return value
    
def apply_uncertainty_angle(value):
    value+= np.random.normal(loc=0.0, scale=0.01*value)
    return value

def uncertain_movements(particles, movements):

    plt.ioff() # Desliga o modo interativo, para nao aparecerem muitas imagens no meio

    frames = 1
    pose=[400, 400, 0]
    angles = np.linspace(0.0, 2*math.pi, num=8)
    color_image = cv2.imread("sparse_obstacles.png")
    np_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    for delta in movements:
        for i in range(len(pose)):
            uncertainty= apply_uncertainty(delta[i])
            if (i==3):
                    uncertainty= apply_uncertainty_angle(delta[i])
            pose[i]+=uncertainty
            for particle in particles:
                uncertainty= apply_uncertainty(delta[i])
                if (i==3):
                    uncertainty= apply_uncertainty_angle(delta[i])
                particle[i]=uncertainty+particle[i]
            
                
        # Simula a leitura do lidar
        
        leituras, lidar_map = inspercles.nb_simulate_lidar(pose, angles, np_image)
        
        # Desenha as particulas
        ax = inspercles.nb_draw_map(color_image, pose=pose, robot=True, particles= particles)
        #ax.imshow(occupancy_image, alph.2)
        # Desenha o mapa do lidar
        ax.imshow(lidar_map, alpha=0.5)

        plt.savefig("syteste%04d.png"%frames, bounds="tight")
    
        frames+=1
        plt.close('all')

    plt.ion()
