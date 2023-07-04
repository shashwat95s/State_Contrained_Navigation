#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
from tf.transformations import euler_from_quaternion
import math
from math import sqrt
from math import atan2
import matplotlib.pyplot as plt


global pose
pose = []


## quaternion to euler
def quat2euler(x,y,z,w):
    quat = [x,y,z,w]s
    return euler_from_quaternion(quat)
########################

## Odometry callback
def callback_vicon(data): #Use appropriate data types here
    global pose
    #To be modified based on data types
    x  = data.transform.rotation.x
    y  = data.transform.rotation.y
    z  = data.transform.rotation.z
    w  = data.transform.rotation.w
    pose = [data.transform.translation.x, data.transform.translation.y, quat2euler(x,y,z,w)[2]]
########################

    
## Euclidean Distance
def dist(p1, p2):
    return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)
########################

def bearing(p1, p2):
    return ( math.atan2(p2[1]-p1[1],p2[0]-p1[0]) )
########################

def signum(p1):
    if(p1>=0):
        return 1
    else:
        return -1
    
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


def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

## Main Node
def controller():

    # Declarations
    #global obs1
    #global obs2
    obs1 = np.array([[0,-1.586,-4,-6.414,-9.586,-14.141], [1.414, 1.414,-1,1.414,1.414,-3.141]]).T
    obs2 = np.array([[0,-0.414,-2.414,-5.586,-8,-10.141], [-1.414,-1.414,-3.414,-3.414,-1,-3.141]]).T

    samp_T = 0.05

    x_start = np.array([0, 0, -np.pi])
    X = np.zeros((3, 1))
    X[:, 0] = x_start
    # print(X)
    t_euler = [0]
    Path = np.empty((0, 2))
    V = [0]
    W = [0]
    surf=[]
    #global K
    K = 1
    K1 = 0.1
    Ks = 2
    #global l
    #global lmax
    lmax = 13.5
    l = 0
    l1 = 0

    fl = 0
    global pose,scan

    
    #Robot_x = pose[0]
    #Robot_y = pose[1]
    #Robot_alpha = pose[2]
    #time.sleep(5)

    rospy.init_node('main_controller', anonymous=True)
    # rospy.Subscriber('/vicon/tb3_0/tb3_0', TransformStamped, callback_vicon) #Insert ROS Subscriber here
    sub = rospy.Subscriber("/odom", Odometry, newOdom)

    
    pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10) #Insert ROS velocity publisher here
    velocity_msg = Twist()
    rate = rospy.Rate(40)

    while not rospy.is_shutdown():

        

        # Robot_position = [pose[0],pose[1]]
        
        while l < lmax:             #loop to
            global x
            global y
            global theta
            X = np.column_stack(X,[x,y,theta] )

            goal, l1 = nextSP1(l1, [X[0, -1], X[1, -1]], K, obs1, obs2)

            Path = np.vstack((Path, [goal[0], goal[1]]))
            l = goal[3]
            if (np.mod(X[2, -1] - goal[2], 2 * np.pi)) > np.pi:
                sg = -1
                K = -1
            else:
                sg = 1
                K = 1

            while True:             # loop to increase angle 
                x_start = X[:, -1]
                r2 = np.sqrt((x_start[0] - goal[0]) ** 2 + (x_start[1] - goal[1]) ** 2)
                r1 = r2 * np.sqrt(1 + K ** 2)
                r3 = r2 * r1 * np.sqrt((K ** 2) + 4) * np.exp(-K * (x_start[2] - goal[2])) / (r2 * sg * K)
                if r3 < 0.1:
                    break
                else:
                    X[2, -1] = X[2, -1] + sg * 2 * np.pi

            j = 0
            sumS = 0
            while not reached(X[:, -1], goal):      #loop to run to interim goal
                j = j + 1
                if j > 10:
                    break
                x0 = (-goal[0] + X[0, -1])
                y0 = (-goal[1] + X[1, -1])
                th_d = goal[2]
                alpha = X[2, -1] - goal[2]

                # Frame rotation
                x = np.cos(th_d) * x0 + np.sin(th_d) * y0
                y = np.cos(th_d) * y0 - np.sin(th_d) * x0

                # Transformation
                z1 = alpha
                z2 = x * np.cos(alpha) + y * np.sin(alpha)
                z3 = x * np.sin(alpha) - y * np.cos(alpha)
                S = z2 - K * z3
                sumS = sumS + np.sign(S) * samp_T
                if fl == 0:
                    surf = np.append(surf, S)
                    surf = np.append(surf, S)
                    fl = 1
                else:
                    surf = np.append(surf, S)
                if np.abs(S) < 0.005:
                    omega = (-K1 * z1)
                    v = (-K1 * z1 * (z3 + K * z2))
                else:
                    omega = 0
                    v = (-Ks * np.sign(S) * np.sqrt(np.abs(S)))# - 0.5 * sumS
                #omega = -K1 * z1
                #v = sg * (-K1 * z1 * (z3 + K * z2)) - Ks * 20 * np.sign(S) * np.sqrt(np.abs(S)) - 0.1 * sumS
                V = np.append(V, v)
                W = np.append(W, omega)
                #X = np.column_stack((X, X[:, -1] + [v * np.cos(X[2, -1]), v * np.sin(X[2, -1]), omega] * samp_T))
                linear_vel = V
                velocity_msg.linear.x = linear_vel
                angular_vel = W
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
                rate.sleep()

                t_euler = np.append(t_euler, t_euler[-1] + samp_T)
        
        
        
        
########################

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
