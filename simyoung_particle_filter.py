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
import scipy as sp

angles = np.linspace(0.0, 2*math.pi, num=8)
#por enquanto angles sera sempre 7
counter= 0
            
def resample(particles):
    #Criamos uma funcao customizada para realizar a reamostragem das particulas
    #em proporcao a probabilidade de uma particula coincidir com a posicao do robo
    
    #Realizamos isso adicionando a cada particula a diferenca dela com as
    #particulas vizinhas, normalizando a diferenca pelo peso de cada particula
            
    copy= deepcopy(particles)
    #hey_wait= 1-greatest_probability(particles)**2
    hey_wait= 1.0/64
    for p in particles:
        print p.w
        p.w=1.0/len(particles)
        
        cx=np.array([c.x for c in copy])
        cy=np.array([c.y for c in copy])
        ctheta=np.array([c.theta for c in copy])
        cw=np.array([c.w for c in copy])
        
        diff_x=np.sum((cx-p.x)*cw)
        diff_y=np.sum((cy-p.y)*cw)
        diff_theta=np.sum((ctheta-p.theta)*cw)
        
        p.x+= diff_x*hey_wait
        p.y+= diff_y*hey_wait
        p.theta+=(diff_theta%(2*math.pi))*hey_wait
        p.theta%=2*math.pi
        
   
            

def create_particles(minx, miny, maxx, maxy, n):
    #Cria n particulas
    #no quadrado de aresta menor minx, miny e maior maxx, maxy
    #com posicao aleatoria dentro quadrado
    #e um angulo aleatorio entre 0 e 2*pi
        
    particle_cloud = []
    for i in range(n):
        x = random.uniform(minx, maxx)
        y = random.uniform(miny, maxy)
        
        theta = (random.uniform(0, 2*math.pi)+math.pi)%(2*math.pi)
        p = Particle(float(x), float(y), float(theta), 1.0/n)
        particle_cloud.append(p)
    return particle_cloud

def generate_movement(shape="square"):
    #Gera movimentos para o robo, descrevendo um quadrado
    #e girando durante o trajeto
    
    
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
    

def apply_uncertainty(value):
    #Calcula a incerteza do movimento, essa incerteza eh dada
    #como um valor aleatorio de uma distribuicao normal proporcional ao
    #movimento feito, com desvio padrao 10% do valor
    
    if (value!=0):
        value+= np.random.normal(loc=0.0, scale=abs(0.1*value))
    return value
    
def laser_probability(dist_roboto, dist_particle):
    #Calcula a probabilidade de uma particula coincidir com a posicao do
    #robo dada a leitura real de um unico laser
    
    #Dado que este codigo eh uma simulacao, o desvio padrao pode assumir um
    #valor arbitrario. Escolhemos 5.0 por se aproximar do desvio real dos robos
    #neato usados em aula
    
    std= 5.0
    #TODO 5.0 acabou sendo muito pouco, valores estao extremamente dificeis de
    #tratar, preciso aumentar por necessidade
    std= 50.0
    
    #return math.e**(-(dist_roboto-dist_particle)/(2*std**2))
    
    return 2*sp.stats.norm.cdf(-abs(dist_roboto-dist_particle), loc=0.0, scale=std)

def particle_probability(lasers_roboto, lasers_particle):
    #Dada uma lista de lasers de uma particula e do robo, calcula
    #o produto das probabilidades individuais de cada laser
    #para a particula

    angles = np.linspace(0.0, 2*math.pi, num=8)
    #TODO: remover esse angles depois, deixar aqui por enquanto por estar meio perdido
    result=1
    
    for i in range(len(angles)-1):
        result*= laser_probability(lasers_roboto[round(angles[i], 3)], lasers_particle[round(angles[i], 3)])
    
    return result
    
def position_speculation(particle_list, pose):
    #Dada uma lista de particulas e a posicao do robo a funcao
    #calcula a probabilidade normalizada de cada particula coincidir com
    #a posicao do robo
    
    
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
    #Procura a particula com a maior probabilidade
    #de coincidir com o robo e devolve o valor dessa
    #probabilidade
    
    w=0
    for particle in particles:
        if particle.w > w:
            w= particle.w
    
    return w

def uncertain_movements(particles, movements, pose):
    #Dado uma lista de movimentos para o robo, aplica esse movimento
    #num mapa frame-a-frame calculando tambem um desvio padrao probabilistico da
    #trajetoria e termina criando um gif representando o trajeto
    
    #Adicionalmente, tambem chamamos a funcao de resample para as particulas

    plt.ioff() # Desliga o modo interativo, para nao aparecerem muitas imagens no meio

    frames = 1
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
        
        
        if greatest_probability(particles)>0.9:
            resample(particles)
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
