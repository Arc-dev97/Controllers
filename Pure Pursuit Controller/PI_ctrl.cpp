#include<bits/stdc++.h>
#define dd double
#define dt 0.1
#define pi 3.14159265359
using namespace std;

dd q_sum(queue <dd> x)  // Function to find the sum of a queue
{
    dd sum=0;
    int size=x.size();
    for(int i=0;i<size;i++){
        x.push(x.front());
        sum+= x.front();
        x.pop();
    }
    return sum;
}


class PI
{
    public:
    dd Kp,Ki,Kh,Kpv,Kiv,v;
    queue <dd> e_q;
    queue <dd> t_q;
    PI(dd kp,dd ki,dd kh,dd kpv,dd kiv)
    {
        Kp=kp;
        Ki=ki;
        Kh=kh;
        Kpv=kpv;
        Kiv=kiv;
        v=0;   
    }

    vector<dd> ctrl(dd x_d,dd y_d,dd x,dd y,dd theta_a,dd vel);
};

vector<dd> PI::ctrl(dd x_d,dd y_d,dd x,dd y,dd theta_a,dd vel )
{
    dd Th_max=3.41;
    dd e_x= x - x_d;
    dd e_y= y - y_d;
    dd d_str=1.2;
    v=vel;

    dd e= sqrt(pow(e_x,2) + pow(e_y,2)) - d_str;    //Euclidean error
    dd v_str= Kp*e + Ki*q_sum(e_q)*dt;                     //Linear velocity

    dd theta_d = atan2(e_y,e_x);     //Desired theta

    // Adjusting the range of angle from -pi to pi
    if(theta_d < -1*pi){
        theta_d+= 2*pi;
    }else if(theta_d > pi){
        theta_d-= 2*pi;
    }

    dd e_theta = theta_a - theta_d;  // Error in theta

    // Adjusting the range of angle from -pi to pi
    if(e_theta < -1*pi){
        e_theta+= 2*pi;
    }else if(e_theta > pi){
        e_theta-= 2*pi;
    }

    dd steering= Kh*e_theta;       //Proportional control for steering
       
    
    dd e_v= v_str - v;
    dd Th= Kpv*e_v + Kiv*q_sum(t_q)*dt;
    if (Th>Th_max){
        Th=Th_max;
    }
    
    // Accumulated error
    e_q.push(e);  

    if (e_q.size()>10){
        e_q.pop();
    }

    t_q.push(e_v);
    if (t_q.size()>10){
        t_q.pop();
    }
    vector<dd>out;
    out.push_back(Th);
    out.push_back(steering);
    return out;
}

//Test input
int main()
{
    PI Pi=PI(0.1,0.1,0.1,1,1);
    cout << Pi.ctrl(21,0,20,0,0,0)[0]<<" "<< Pi.ctrl(21,0,20,0,0,0)[1]<< endl;
}