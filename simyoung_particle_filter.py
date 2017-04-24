from random import randint, choice
import math
import random
from pf import Particle
import numpy as np
import matplotlib.pyplot as plt
import cv2
import inspercles
from copy import deepcopy
reload(inspercles)

angles = np.linspace(0.0, 2*math.pi, num=8)
#por enquanto angles sera sempre 7
counter= 0

def resample(key, particles):
    w=0
    best= None
    for particle in particles:
        if particle.w > w:
            w= particle.w
            best= particle
    
    best.w=1.0/len(particles)
    
    #if (key == "angle"):
    if (True):
        disp= math.pi
        for i in range(len(particles)):
            particles[i]= deepcopy(best)
            uncertainty= apply_uncertainty(disp)
            uncertainty-=disp
            particles[i].theta+= uncertainty
            particles[i].theta%= 2*math.pi
    
    #if (key == "position"):
    if (True):
        disp= 100.0
        
        for i in range(len(particles)):
            #particles[i]= deepcopy(best)
            uncertainty= apply_uncertainty(disp)
            uncertainty-=disp
            particles[i].x+= uncertainty
            particles[i].x= (particles[i].x+800)%800
            uncertainty= apply_uncertainty(disp)
            uncertainty-=disp
            particles[i].y+= uncertainty
            particles[i].y= (particles[i].y+800)%800
            
#nao esta sendo usado
            
def resample2(particles):
            
    copy= deepcopy(particles)
    #hey_wait= 1-greatest_probability(particles)**2
    hey_wait= 1.0/32
    #TODO acho que existe uma relacao entre hey_wait e alpha
    for p in particles:
        p.w=1.0/len(particles)
        
        diff_x=0
        diff_y=0
        diff_theta=0
        for c in copy:
            diff_x+= (c.x - p.x)*(c.w**2)
            diff_y+= (c.y - p.y)*(c.w**2)
            diff_theta+= (c.theta - p.theta)*(c.w**2)
            
        p.x+= diff_x*hey_wait
        p.y+= diff_y*hey_wait
        p.theta+=diff_theta*hey_wait
        p.theta%=2*math.pi
        
   
            

def create_particles(minx, miny, maxx, maxy, n):
    """
        Cria num particulas
        situadas no intervalo x - var_x a x + var_x, y - var_x at'e y + var_y e theta - var_theta a theta + var_theta
    """
    particle_cloud = []
    for i in range(n):
        x = random.uniform(minx, maxx)
        y = random.uniform(miny, maxy)
        
        theta = (random.uniform(0, 2*math.pi)+math.pi)%(2*math.pi)
        p = Particle(float(x), float(y), float(theta), 1.0/n)
        particle_cloud.append(p)
    return particle_cloud

def generate_movement(shape="square"):
    movements=[]
    for j in range(3):
        if (shape=="square"):
            i=0
            while(i!=10):
                movements.append([5, 0, math.pi/5])
                i+=1
            i=0
            while(i!=10):
                movements.append([0, 0, math.pi/20])
                i+=1
            i=0
            while(i!=10):
                movements.append([0, 5, math.pi/5])
                i+=1
            i=0
            while(i!=10):
                movements.append([0, 0, math.pi/20])
                i+=1
            i=0
            while(i!=10):
                movements.append([-5, 0, math.pi/5])
                i+=1
            i=0
            while(i!=10):
                movements.append([0, 0, math.pi/20])
                i+=1
            i=0
            while(i!=10):
                movements.append([0, -5, math.pi/5])
                i+=1
            i=0
            while(i!=10):
                movements.append([0, 0, math.pi/20])
                i+=1
        
    return movements
        

movements = [[1, 0, 0], [0, 1, 0], [-2,0,0], [0, -3, 0],
          [5,0,math.pi/12.0], [0, 8, math.pi/12.0], [-13, 0, math.pi/12],[0, -21,-math.pi/4],
          [34, 0, 0],[55,0,0], [-89,0,0], [-144,0,0],
          [0,0,-math.radians(90)],
          ]

def apply_uncertainty(value): 
    if (value!=0):
        value+= np.random.normal(loc=0.0, scale=abs(0.1*value))
    return value
    
def laser_probability(dist_roboto, dist_particle):
    std= 1.5
    #std=10.0 #com valores menores caio em um nan
    #TODO: serio, qual deve ser este valor?
    return math.e**(-(dist_roboto-dist_particle)/(2*std**2))

def particle_probability(lasers_roboto, lasers_particle):
    angles = np.linspace(0.0, 2*math.pi, num=8)
    #TODO: remover esse angles depois, deixar aqui por enquanto por estar meio perdido
    result=1
    
    #print lasers_roboto.keys()
    #print lasers_particle.keys()
    for i in range(len(angles)-1):
        result*= laser_probability(lasers_roboto[round(angles[i], 3)], lasers_particle[round(angles[i], 3)])
    
    return result
    
def position_speculation(particle_list, pose):
    color_image = cv2.imread("sparse_obstacles.png")
    np_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
    lasers_roboto, garbage= inspercles.nb_simulate_lidar(pose, angles, np_image)
    
    for particle in particle_list:
        lasers_particle, garbage= inspercles.nb_simulate_lidar(particle, angles, np_image)
        particle.w*=particle_probability(lasers_roboto, lasers_particle)
    
    alpha=0
    for particle in particle_list:
        alpha+=particle.w
        
    alpha=1.0/alpha
    for particle in particle_list:
        particle.w*= alpha
    
def greatest_probability(particles):
    w=0
    for particle in particles:
        if particle.w > w:
            w= particle.w
    
    return w

def uncertain_movements(particles, movements):

    plt.ioff() # Desliga o modo interativo, para nao aparecerem muitas imagens no meio

    frames = 1
    pose=[400, 400, 0]
    color_image = cv2.imread("sparse_obstacles.png")
    np_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    for delta in movements:
        for i in range(len(pose)):
            uncertainty= apply_uncertainty(delta[i])
            pose[i]+= uncertainty
            for particle in particles:
                uncertainty= apply_uncertainty(delta[i])
                particle[i]=uncertainty+particle[i]
                if i!=2:
                    if particle[i]<0:
                        particle[i]=0
                    if particle[i]>800:
                        particle[i]=800
        
        #pondera as particulas levando em conta o movimento que realizamos
           
        position_speculation(particles, pose)
        
        
        
        resample2(particles)
        #
        #print("iteracao")
        #for particle in particles:
        #    print(particle.w)
           
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
