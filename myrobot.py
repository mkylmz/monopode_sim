import pygame
from math import sin, cos
from math import pi
import numpy as np

def draw_ground(display):

    width,height = display.get_size()
    ground = pygame.draw.rect(display,(150,75,0),(0,height-height/5,width,height/5))

    return ground

class robot():

    def __init__(self,display,mass,radius,leg_length, orientation=0,position=0 ):

        self.width,self.height = display.get_size()
        
        self.mass = mass
        self.radius = radius
        self.leg_length = leg_length

        self.groundheight = self.height - self.height/5

        if position == 0:
            self.pos = np.array([self.width/10, self.height-2*self.height/5])
        else:
            self.pos = np.array([self.width/10, position])
        self.ori = orientation+pi/2
        self.vel = np.array([0,0])
        self.acc = np.array([0,0])
        self.weight = self.mass * 9.81 # 9.81 m/s2 changed as we update every 2 ms

    def draw(self,display):
        cur_pos = (int(round(self.pos[0])),int(round(self.pos[1])))
        pygame.draw.circle(display, (255,0,0), cur_pos, self.radius)
        end_of_line = [0,0]
        end_of_line[0] = cur_pos[0] + int(round(self.leg_length*cos(self.ori)))
        end_of_line[1] = cur_pos[1] + int(round(self.leg_length*sin(self.ori)))
        pygame.draw.line(display, (0,0,0), cur_pos, tuple(end_of_line), 3)
    
    def calc_friction(self):
        pass

    def update(self):
        leg_height = int(round(self.leg_length*sin(self.ori)))
        lowest_point = int(round(self.pos[1] + self.leg_length*sin(self.ori)))
        
        if ( lowest_point <= self.groundheight):
            self.acc[1] = self.weight
        else:
            self.acc[1] = self.weight*10
            self.vel = -self.vel
            self.pos[1] = self.groundheight-leg_height-1

        self.vel[1] = self.vel[1] + self.acc[1] * 0.03
        self.pos[1] = self.pos[1] + self.vel[1] * 0.03