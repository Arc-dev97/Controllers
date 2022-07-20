from math import sin, cos, atan2,sqrt
from time import time
import numpy as np
import PI_ctrl
import sys_model
import matplotlib.pyplot as plt
from math import pi

dt = 0.1    # time step
show_animation= True
T = 120     #Total simulation time in sec

class sim():
    def __init__(self, Kp, Ki, Kh, Kpv, Kiv):
        self.Kp = Kp
        self.Ki = Ki
        self.Kh = Kh
        self.Kpv = Kpv
        self.Kiv = Kiv

    # initial states as arguments // change xi=0 for figure-8 traj
    def run(self, xi=18, yi=0, ti=pi/2, L=2.2,v=0):
        #Trajectory starting point
        x_ir=20
        y_ir=0
        
        curr_time = 0

        car = sys_model.model(xi, yi, ti, L,v)    #Initialise model
        v_ctrl = PI_ctrl.PI(self.Kp, self.Ki, self.Kh,
                            self.Kpv, self.Kiv)  #Initialise controller

        x, y, t, Th, psi = xi, yi, ti, 0, 0
        x_g, y_g, t_g, v_g, psi_g = [xi], [yi], [ti], [0], [0]
        xd_g, yd_g = [x_ir], [y_ir]
        theta_cir, v_ref, c_time, e_g,eyaw_g, theta_dg, Th_g = [0], [0], [0], [0], [0], [0], [0]
        i=1
        x_d=0
        y_d=0
        while T > curr_time:
        # Trajectories:
            #Circle:
            x_d = 20*cos(0.1*curr_time)
            y_d = 20*sin(0.1*curr_time)

            #Straight Line:
            
            # x_d=3
            # y_d+= 0.1

            #Infinity shape:
            # x_d=75*sin(0.1*curr_time)
            # y_d= 45*sin(0.1*curr_time)*cos(0.1*curr_time)
            

            # Traj--> ctrl --> Kine--> ctrl           
            Thye = v_ctrl.ctrl(x_d, y_d, x, y, t,v)
            xytv = car.kine(Th, psi)
            theta = atan2(y_d, x_d)
            theta_cir.append(theta)

            #Update States
            x, y, t,v = xytv[0], xytv[1], xytv[2],xytv[3]
            Th = Thye[0]
            psi = Thye[1]
            e = Thye[2]
            eyaw=Thye[3]
            theta_d= Thye[4]
            curr_time += dt

            # Add data to lists for visualization
            x_g.append(x)
            y_g.append(y)
            t_g.append(t)
            v_g.append(v)
            psi_g.append(psi)
            xd_g.append(x_d)
            yd_g.append(y_d)
            e_g.append(e)
            eyaw_g.append(eyaw)
            theta_dg.append(theta_d)
            Th_g.append(Th)
            c_time.append(curr_time)
            
            if show_animation:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(xd_g, yd_g, "-r", label="input")
                plt.plot(x_g, y_g, "-b", label="output")
                plt.plot(x_g[i], y_g[i], "xg", label="output")
                plt.grid(True)
                plt.title("Speed[m/s]:"+str(round(v,3)) +" "+"|"+" "+"Time:"+str(round(curr_time,3)))
                plt.pause(0.001)
            i+=1
        return xd_g, yd_g, x_g, y_g, t_g, v_g, psi_g, c_time, v_ref, e_g, theta_cir ,eyaw_g ,theta_dg



car1=sim(Kp=1,Ki=1,Kh=0.08 ,Kpv=1,Kiv=0.08)       # Tune parametes here

out = car1.run()
xd_g, yd_g, x_g, y_g, t_g, v_g, psi_g, c_time, v_ref, e_g, theta_cir , eyaw ,theta_dg= out
zeroline=[0]*701
roc_e=[0]   #Rate of change of euclidean error
for i in range(1,len(e_g)):
    roc_e.append((e_g[i]-e_g[i-1])/0.1)


plt.figure(figsize=(12,7))
plt.subplot(1, 3, 1)
plt.plot(xd_g, yd_g, "-r", label="input")
plt.plot(x_g, y_g, "--", label="output")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.legend()


plt.subplot(1, 3, 2)
#plt.plot(c_time, xd_g, label="xd")
plt.plot(c_time, v_g, label="actual velocity")
#plt.plot(c_time, zeroline,"--b", label="zeroline")
plt.plot(c_time,e_g,"g",label="error")
plt.xlabel("time")
plt.ylabel("distance(m)")
plt.legend()


plt.subplot(1, 3, 3)
#plt.plot(c_time, v_ref, ".", label="reference velocity")
#plt.plot(c_time, v_g, label="velocity_actual")
plt.plot(xd_g, e_g,"g", label="euclidean error")
plt.plot(xd_g, roc_e,"b", label="rate of change of error")
plt.xlabel("x (m)")
plt.ylabel("error")
plt.legend()
plt.show()

