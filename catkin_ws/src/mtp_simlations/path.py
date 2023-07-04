#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2,sin,cos,pi,tan,sqrt,exp
import math
import numpy as np
import matplotlib.pyplot as plt 

x = 0.0
y = 0.0 
theta = 0.0
av=0
aw=0


def newOdom(msg):
    global x
    global y
    global theta
    global av
    global aw

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    av = msg.twist.twist.linear.x
    aw= msg.twist.twist.angular.z

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def map_angle(t):
    while t > pi:
        t = -(2*pi-t)
    while t < -pi:
        t = (2*pi+t)
    return t


def reached(g):
    global x
    global y
    global theta
    if ((g[0]-x)**2+(g[1]-y)**2)**0.5<0.2 and (map_angle(abs(g[2]-theta)))<0.0175*1:
        return 1
    else:
        print(((g[0]-x)**2+(g[1]-y)**2)**0.5)
        print(map_angle(abs(g[2]-theta)))
        return 0 



rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(25)

goal = Point()

k=0



x1=[0]
y1=[0]


th=[0.0]
x_start = [x,y,theta]
goal=[3,6,pi/4]
Path=[[-1,0],[-2,-1],[-3,-2],[-4,-2],[-5,-2],[-6,-1],[-7,0]]



K=1
K1=0.2
Ks=0.1

n=0
while n<len(Path):
    goal[0]=Path[n][0]
    goal[1]=Path[n][1]
    if n==len(Path)-1:
        goal[2]= pi
    else:
        goal[2]=atan2(Path[n+1][1]-Path[n][1],Path[n+1][0]-Path[n][0])
    
    add=0
    print(n)
    while 1:
        x_start=[x,y,theta]     
        r2=((x-goal[0])**2+(y-goal[1])**2)**0.5
        r1=r2*(1+K**2)**0.5
        r3=r2*r1*(((K*K)+4)**0.5)*exp(-K*(theta+add-goal[2]))/(r2*K)
        # print(r2)
        # print(r3)
        if r3<0.1:
            break
        else:
            add=add+2*pi
            print(add)


    while not reached(goal):



        x0=(-goal[0]+x)
        y0=(-goal[1]+y)
        th_d=goal[2]

        alpha=add+theta-goal[2]

        #Frame rotation
        xg=cos(th_d)*x0+sin(th_d)*y0
        yg=cos(th_d)*y0-sin(th_d)*x0

        #Transformation

        z1=alpha
        z2=xg*cos(alpha)+yg*sin(alpha)
        z3=xg*sin(alpha)-yg*cos(alpha)
        S=z2-K*z3

        if abs(S)<0.05:
            omega=(-K1*z1)
            v=(-K1*z1*(z3+K*z2))
        else:
             omega=0;
             v=(-Ks*np.sign(S))



        k=k+1

        x1.append(x)
        y1.append(y)



        th.append(theta)




        speed.linear.x =v
        speed.angular.z =omega


        pub.publish(speed)
        r.sleep()
    n=n+1
    print(n)
speed.linear.x =0
speed.angular.z =0

pub.publish(speed)

#matplotlib notebook



plt.figure(1)
plt.plot(x1,y1,label='Robot position')
plt.xlabel('x co-ordinate')
plt.ylabel('y co-ordinate')
plt.legend()

plt.figure(2)
plt.plot(th)
plt.xlabel('time')
plt.ylabel('orientation')
plt.legend()
plt.show()
# print(x1)
