#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
from tf.transformations import euler_from_quaternion
import math
import numpy

from math import sqrt,pi
from math import atan2
import matplotlib.pyplot as plt

global pose
pose = []


## quaternion to euler
def quat2euler(x,y,z,w):
    quat = [x,y,z,w]
    return euler_from_quaternion(quat)
########################



    
# ## Euclidean Distance
# def dist(p1, p2):
#     return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)
# ########################

# def bearing(p1, p2):
#     return ( math.atan2(p2[1]-p1[1],p2[0]-p1[0]) )
# ########################

# def signum(p1):
#     if(p1>=0):
#         return 1
#     else:
#         return -1
    
# def waypoints():
    #Add waypoint obtained algorithm here
def path_l(l):
    if l <= 1:
        x = -l
        y = 0
    elif l <= 3.8:
        x = -1 - (l-1)/sqrt(2)
        y = x + 1
    elif l <= 5.8:
        x = -3 - (l-3.82)
        y = -2
    elif l <= 8.6:
        x = -5 - (l-5.82)/sqrt(2)
        y = -x - 7
    elif l <= 10.6:
        x = -7 - (l-8.65)
        y = 0
    else:
        x = -9 - (l-10.6)/sqrt(2)
        y = x + 9
    return x, y

import numpy as np

def p2l(q, p1, p2):
    a, b, c = compute_line_through_two_points(p1, p2)
    x1 = (b**2) * q[0] - a * b * q[1] - a * c
    if b == 0:
        y1 = q[1]
    else:
        y1 = -(1/b) * (a*x1 + c)
        
    d1 = distance_point_to_point([x1, y1], p1)  # perpendicular point to point 1
    d2 = distance_point_to_point([x1, y1], p2)  # perpendicular point to point 2
    d3 = distance_point_to_point(p1, p2)        # distance between p1 and p2
    
    if d1 + d2 > d3:
        if d1 > d2:
            w = 2
            dist = distance_point_to_point(q, p2)
            T = p2
        else:
            w = 1
            dist = distance_point_to_point(q, p1)
            T = p1
    else:
        w = 0
        dist = compute_distance_point_to_line(q, p1, p2)
        T = [x1, y1]
    
    return dist


def compute_line_through_two_points(p1, p2):
    x1 = p1[0]
    x2 = p2[0]
    y1 = p1[1]
    y2 = p2[1]
    a = y2 - y1
    b = x1 - x2
    c = -(a * x1 + b * y1)
    s = math.sqrt(a**2 + b**2)
    
    a = a / s
    b = b / s
    c = c / s
    
    return a, b, c


def distance_point_to_point(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def compute_distance_point_to_line(q, p1, p2):
    num = np.abs((p2[1]-p1[1])*q[0] - (p2[0]-p1[0])*q[1] + p2[0]*p1[1] - p2[1]*p1[0])
    den = np.sqrt((p2[1]-p1[1])**2 + (p2[0]-p1[0])**2)
    return num/den


def nearest(l, obs1, obs2):
    d = 999
    a, b = path_l(l)
    for i in range(1, len(obs1)):
        dist = p2l([a,b], [obs1[i-1,0],obs1[i-1,1]], [obs1[i,0],obs1[i,1]])
        if dist < d:
            d = dist
    for i in range(1, len(obs2)):
        dist = p2l([a,b], [obs2[i-1,0],obs2[i-1,1]], [obs2[i,0],obs2[i,1]])
        if dist < d:
            d = dist
    return d

def next(l, curr, K, obs1, obs2):
    r2 = 0.1
    gx, gy = curr[0], curr[1]
    while ((gx - curr[0])**2 + (gy - curr[1])**2)**0.5 <= r2:
        l += 0.1
        gx, gy = path_l(l)
        r1 = nearest(l, obs1, obs2)
        r2 = r1 / (1 + K**2)**0.5
    l -= 0.1
    return l


def nextSP1(l, curr, K, obs1, obs2):
    l1 = next(l, curr, K, obs1, obs2)
    #global lmax
    if l1 > lmax:
        l1 = lmax
        gx, gy = path_l(l1)
        goal = [gx, gy, np.pi, l1]
    else:
        gx, gy = path_l(l1)
        l2 = next(l1, [gx, gy], K, obs1, obs2)
        if l2 > lmax:
            l2 = lmax
        gx1, gy1 = path_l(l2)
        goal = [gx, gy, atan2(gy1-gy, gx1-gx), l1]
    return goal, l1

def reached(A, B):
    dist = math.sqrt((A[1] - B[1])**2 + (A[0] - B[0])**2)
    angle_diff = abs(A[2] - B[2]) % (2*math.pi)
    if dist < 0.1 and (angle_diff < math.radians(2) or angle_diff > (2*math.pi - math.radians(2))):
        return 1
    else:
        return 0

def map_angle(t):
    if(t > pi):
        t = -(2*pi-t)
    elif(t < -pi):
        t = (2*pi+t)
    return t

# Odometry callback
def callback_vicon(data): #Use appropriate data types here
    # global pose
    global x
    global y
    global theta
    #To be modified based on data types
    x  = data.transform.translation.x
    y  = data.transform.translation.y
    x1  = data.transform.rotation.x
    y1  = data.transform.rotation.y
    z  = data.transform.rotation.z
    w  = data.transform.rotation.w
    theta =quat2euler(x1,y1,z,w)[2]
#######################
# def newOdom(msg):
#     global x
#     global y
#     global theta

#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y

#     rot_q = msg.pose.pose.orientation
#     (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
## Main Node
x=0
y=0
theta=0
def controller():
    rospy.init_node("zigzag")          # initialise the node called move
    rate = rospy.Rate(5)
    # sub = rospy.Subscriber("/odom",Odometry,newOdom)        #to get odometry data
    rospy.Subscriber('/vicon/tb3_1/tb3_1', TransformStamped, callback_vicon) #Insert Vicon Subscriber here
    # sub1 = rospy.Subscriber("/scan",LaserScan,newSc)        #to get laser data
    pub = rospy.Publisher("/tb3_1/cmd_vel",Twist,queue_size=5)  #to send velocity data

    global x
    global y
    global theta

    while not rospy.is_shutdown():

        print("x:",x)
        print("y:",y)
        print("theta:",theta)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
