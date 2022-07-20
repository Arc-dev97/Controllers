from math import sqrt, atan2
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from math import pi

# Defining the Controller class

class PI():
    
    def __init__(self, Kp, Ki, Kh, Kpv, Kiv):
        # Kp,Ki,Kh are tuning parameters for vel and heading cmd
        # Kpv,Kih are tuning parameters for Throttle cmd
        
        self.Kp = Kp                         
        self.Ki = Ki
        self.Kh = Kh
        self.Kpv = Kpv
        self.Kiv = Kiv
        self.e_q = deque()
        self.t_q = deque() #throttle error
        self.v=0

    def ctrl(self, x_d, y_d, x, y, theta_a,vel):
        #theta_a is actual heading of vehicle(-pi to pi)

        Th_max=3.41

        e_x = x - x_d
        e_y = y - y_d
        d_str = 1.2
        self.v=vel

        e = sqrt(e_x**2 + e_y**2) - d_str            # Euclidean error
        v_str = self.Kp*e + self.Ki*(sum(self.e_q))*0.1   # linear velocity
        #print(sum(self.e_q))
        theta_d = atan2(e_y, e_x)                    # Desired theta
        
        #Adjusting the range of angle from -pi to pi
        if theta_d < -1*pi:
            theta_d+= 2*pi
        elif theta_d> pi:
            theta_d-= 2*pi
        
        e_theta = theta_a - theta_d                 # Error in theta
        
        #Adjusting the range of angle from -pi to pi
        if e_theta < -1*pi:
             e_theta+= 2*pi
        elif e_theta> pi:
             e_theta-= 2*pi

        steering = self.Kh*e_theta                   # Proportional control for steering

        self.e_q.append(e)

        e_v= v_str - self.v
        
        Th= self.Kpv*e_v + self.Kiv*(sum(self.t_q))*0.1   #Throttle
        if Th>Th_max:
            Th=Th_max
        if Th < -Th_max:
            Th= -Th_max
        
        if len(self.e_q) > 10:
            self.e_q.popleft()
        
        self.t_q.append(e_v)
        if len(self.t_q)>10:
           self.t_q.popleft()
        
        print("th:",Th,"steering:",steering)
        print("tq:",sum(self.t_q))
       
        return(Th,steering,e,e_theta,theta_d)

#Test Input
v_h=PI(0.1,0.1,0.1,1,1)
print(v_h.ctrl(21,0,20,0,0,0))
