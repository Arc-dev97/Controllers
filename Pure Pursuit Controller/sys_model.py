#from tkinter import Y
import numpy as np
from math import atan2, cos, sin, tan, pi


# This module defines the dynamics of the vehicle

class model():
    def __init__(self, x, y, theta, L, v=0):
        self.L = L          #Length of the vehicle
        self.theta = theta  #Heading angle
        self.x = x          #x coordinate of vehicle
        self.y = y          #y coordinate of vehicle
        self.v = v          #velocity of vehicle

    def kine(self, Th, psi):

        dt = 0.1            #time step
        l_r=self.L/2        #length from rear axle to COM 
        beta= atan2(l_r*tan(psi),self.L)    # Slip angle
        
        #State equations:
        self.v += Th*dt
        self.x += self.v*cos(self.theta+beta)*dt
        self.y += self.v*sin(self.theta+beta)*dt
        self.theta += (self.v/self.L)*cos(beta)*tan(psi)*dt
        #self.y=0
        #self.theta=0

        # Adjusting the range of heading angle from -pi to pi
        if self.theta < -1*pi:
             self.theta+= 2*pi
        elif self.theta> pi:
             self.theta-= 2*pi

        return (self.x, self.y, self.theta,self.v)