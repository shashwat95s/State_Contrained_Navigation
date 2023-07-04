#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2,sin,cos,pi,tan,sqrt
import math
import numpy as np
import matplotlib.pyplot as plt 



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
    if(t > pi):
        t = -(2*pi-t)
    elif(t < -pi):
        t = (2*pi+t)
    return t


def reached(g):
    global x
    global y
    global theta
    if ((g[0]-x)**2+(g[1]-y)**2)**0.5<0.2 and abs(g[2]-theta)<0.175:
        return 1
    else:
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

# while not rospy.is_shutdown() and k<2000:
while not reached(goal):
  
    
    # x_start = [x,y,theta+2*pi]



    goal=[3,6,pi/4]
    K=2
    K1=0.2
    Ks=0.3

    x0=(-goal[0]+x)
    y0=(-goal[1]+y)
    th_d=goal[2]

    alpha=6*pi+theta-goal[2]

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
    # print("sent")
    # print(omega)
    # print("actual")
    # print(aw)
    r.sleep()
    
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
