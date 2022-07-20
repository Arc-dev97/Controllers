# A simple Pure pursuit and PI controller
## Brief
A pure pursuit controller is implemented on the [Bicycle model](https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357).
It is a velocity and path tracking controller that takes in [x,y,theta,v] as inputs where (x,y) are the current coordinates of the vehicle, 'theta' is the heading angle of the vehicle and 'v' is the current velocity of the vehicle.
It outputs Throttle and steering commands for the vehicle.

## Simulation
The simulation is setup in simulation.py 
1. Set the initial states of the vehicle: 
``` python
def run(self, xi=18, yi=0, ti=pi/2, L=2.2,v=0):
```
2. Set the starting point for reference trajectory:
```python
 #Trajectory starting point
   x_ir=20
   y_ir=0
```
3. Select the desired reference trajectory:
```python
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
```
4. Run the code:

![circle](/images/circle.png)
