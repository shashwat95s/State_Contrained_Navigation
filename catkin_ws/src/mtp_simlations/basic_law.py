#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2,sin,cos,pi,tan
import math
import numpy as np
import matplotlib.pyplot as plt 

x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def map_angle(t):
    if(t > pi):
        t = -(2*pi-t)
    elif(t < -pi):
        t = (2*pi+t)
    return t


rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()

k=0
kv=1.2
ka=1.1

l=[0]
x1=[0]
x2=[0]
y1=[0]
y2=[0]
u1=[0]
e1=[0,0]
e2=[0,0]
u2=[0]
e11=[0]
e22=[0]
u1=0.1
u2=0.1
th=[0.0]

while not rospy.is_shutdown() and k<628:
  
    K_samp=0.1
    kmain1=5
    kmain2=20
    kmain3=1

    goal.x = 0#x2[k]+u1*cos(th[k])*K_samp
    goal.y = y2[k]+u1*sin(th[k])*K_samp
    th.append(u2*K_samp+th[k])
    
    inc_x = goal.x -x
    inc_y = goal.y -y
    k=k+1
    l.append((k+1)/10)
    x1.append(x)
    y1.append(y)
    x2.append(goal.x)
    y2.append(goal.y)
    angle_to_goal = atan2(inc_y, inc_x)
    
    dis_er=(inc_x**2+inc_y**2)**0.5
    theta_err = map_angle(angle_to_goal - theta)
    
    e11.append(dis_er)
    e22.append(theta_err)
    
    z1=cos(th[k])*(-inc_x)+sin(th[k])*(-inc_y)
    z2=-sin(th[k])*(-inc_x)+cos(th[k])*(-inc_y)
    z3=tan(theta-th[k])

    
    w1=-kmain1*abs(u1)*(z1+z2*z3)
    w2=-kmain2*u1*z2-kmain3*abs(u1)*z3 
    
    ua=(w1-u1)/cos(theta-th[k])
    ub=u2+w2*(cos(theta-th[k])**2)
    
    speed.linear.x =ua
    speed.angular.z =ub
    

    pub.publish(speed)
    r.sleep()
    
speed.linear.x =0
speed.angular.z =0

pub.publish(speed)

#matplotlib notebook
plt.figure(1)
plt.plot(l,x1, label='Robot position')
plt.plot(l,x2 , label='Goal')
plt.xlabel('time')
plt.ylabel('x co-ordinate')
plt.legend()
plt.figure(2)
plt.plot(l,y1, label='Robot position' )
plt.plot(l,y2 , label='Goal')
plt.xlabel('time')
plt.ylabel('y co-ordinate')
plt.legend()
plt.figure(3)
plt.plot(x1,y1 , label='Robot position')
plt.plot(x2,y2, label='Goal')
plt.xlabel('x co-ordinate')
plt.ylabel('y co-ordinate')
plt.legend()

plt.figure(4)
plt.subplot(2,1,1)
plt.plot(l,e11)
plt.xlabel('time')
plt.ylabel('distance error')
plt.subplot(2,1,2)
plt.plot(l,e22)
plt.xlabel('time')
plt.ylabel('orientation error')
plt.legend()
plt.show()
