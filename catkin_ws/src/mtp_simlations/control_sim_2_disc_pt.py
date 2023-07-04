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

## Odometry callback
# def callback_vicon(data): #Use appropriate data types here
#     global pose
#     #To be modified based on data types
#     x  = data.transform.rotation.x
#     y  = data.transform.rotation.y
#     z  = data.transform.rotation.z
#     w  = data.transform.rotation.w
#     pose = [data.transform.translation.x, data.transform.translation.y, quat2euler(x,y,z,w)[2]]
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


def reached1(A, B):  # B is goal
    global K
    global obs1
    global obs2
    global l
    global lmax

    l1 = next(l, [A[0], A[1]], K, obs1, obs2)
    x, y = path_l(lmax)

    if math.sqrt((y - B[1])**2 + (x - B[0])**2) < 0.1:
        dir = math.pi
    else:
        if l1 > lmax:
            l1 = lmax

        gx1, gy1 = path_l(l1)
        dir = math.atan2(gy1 - A[1], gx1 - A[0])

    if math.sqrt((A[1] - B[1])**2 + (A[0] - B[0])**2) < 0.2 and (
        (abs(A[2] - dir) % (2 * math.pi)) < 0.0173 * 3 or
        (abs(A[2] - dir) % (2 * math.pi)) > 2 * math.pi - 0.0173 * 3
    ):
        t = 1
    else:
        t = 0
    
    return t



def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("zigzag")          # initialise the node called move

sub = rospy.Subscriber("/odom",Odometry,newOdom)        #to get odometry data
# sub1 = rospy.Subscriber("/scan",LaserScan,newSc)        #to get laser data
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)  #to send velocity data

velocity_msg = Twist()
r= rospy.Rate(40)        #set refresh rate
global obs1
global obs2
obs1 = np.array([[0,-1.586,-4,-6.414,-9.586,-14.141], [1.414, 1.414,-1,1.414,1.414,-3.141]]).T
obs2 = np.array([[0,-0.414,-2.414,-5.586,-8,-10.141], [-1.414,-1.414,-3.414,-3.414,-1,-3.141]]).T

# samp_T = 0.05


# print(X)
t_euler = [0]
Path = np.empty((0, 2))
V = [0]
W = [0]
surf=[]
global K
K = 0.01
K1 = 0.01
Ks = 0.2
global l
global lmax
lmax = 13.5
l = 0
l1 = 0

fl = 0
global x
global y
global theta

x=0
y=0
theta=pi
x_start = np.array([x, y, theta])
X = np.zeros((3, 1))
X[:, 0] = x_start

ad=0
while not rospy.is_shutdown():    #loop to provide delay
    try:
        while l < lmax:             #loop to select intermediate goal
            # global x
            # global y
            # global theta
            print(l)
            # X = np.column_stack((X,[x,y,theta]) )

            goal, l1 = nextSP1(l1, [X[0, -1], X[1, -1]], K, obs1, obs2)

            Path = np.vstack((Path, [goal[0], goal[1]]))
            l = goal[3]
            if (np.mod(X[2, -1] - goal[2], 2 * np.pi)) > np.pi:
                sg = -1
                K = -1
            else:
                sg = 1
                K = 1

            ad=0

            while True:             # loop to increase angle 
                x_start = X[:, -1]
                r2 = np.sqrt((x_start[0] - goal[0]) ** 2 + (x_start[1] - goal[1]) ** 2)
                r1 = r2 * np.sqrt(1 + K ** 2)
                r3 = r2 * r1 * np.sqrt((K ** 2) + 4) * np.exp(-K * (x_start[2] - goal[2])) / (r2 * sg * K)
                if r3 < 0.1:
                    break
                else:
                    ad=ad+ sg * 2 * np.pi
                    X[2, -1] = X[2, -1] + sg * 2 * np.pi

            
            sumS = 0
            while not reached1(X[:, -1], goal):      #loop to run to interim goal
                
                x0 = (-goal[0] + X[0, -1])
                y0 = (-goal[1] + X[1, -1])
                th_d = goal[2]
                alpha = X[2, -1] - goal[2]

                # Frame rotation
                a1 = np.cos(th_d) * x0 + np.sin(th_d) * y0
                b1 = np.cos(th_d) * y0 - np.sin(th_d) * x0

                # Transformationsin(alpha) 
                z1 = alpha
                z2 = a1 * np.cos(alpha) + b1 * np.sin(alpha)
                z3 = a1 * np.sin(alpha) - b1 * np.cos(alpha)
                S = z2 - K * z3
                # sumS = sumS + np.sign(S) * samp_T
                if fl == 0:
                    surf = np.append(surf, S)
                    surf = np.append(surf, S)
                    fl = 1
                else:
                    surf = np.append(surf, S)
                if np.abs(S) < 0.5:
                    omega = (-K1 * z1)
                    v = (-K1 * z1 * (z3 + K * z2))
                else:
                    omega = 0
                    v = (-Ks * np.sign(S) * np.sqrt(np.abs(S)))# - 0.5 * sumS
                #omega = -K1 * z1
                #v = sg * (-K1 * z1 * (z3 + K * z2)) - Ks * 20 * np.sign(S) * np.sqrt(np.abs(S)) - 0.1 * sumS
                V = np.append(V, v)
                W = np.append(W, omega)
                linear_vel = v
                velocity_msg.linear.x = linear_vel
                angular_vel = omega
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
                print(velocity_msg)
                r.sleep()
                #X = np.column_stack((X, X[:, -1] + [v * np.cos(X[2, -1]), v * np.sin(X[2, -1]), omega] * samp_T))
                X = np.column_stack((X,[x,y,theta+ad]))

                # numpy.unwrap(X[2,:])
                # X[2,-1]=X[2,-1]+ad
                print("goal",goal)
                print(X[:,-1])
                print("ad:",ad)
                # t_euler = np.append(t_euler, t_euler[-1] + samp_T)
    except KeyboardInterrupt:
        break
        
    # r.sleep()

#velocity to turn right
velocity_msg.linear.x=0
velocity_msg.angular.z=0
print(velocity_msg)


a, b = [], []
for i in range(1, 88):
    a_, b_ = path_l(i/10)
    a.append(a_)
    b.append(b_)
# Plot position
plt.plot(X[0,:], X[1,:])
plt.plot(X[0,-1], X[1,-1], 'o', color='red', linewidth=1.5)
plt.plot(X[0,0], X[1,0], 'o', color='green', linewidth=1.5)
plt.plot([0, -1.586, -4, -6.414, -9.586, -14.141], [1.414, 1.414, -1, 1.414, 1.414, -3.141], color='black')
plt.plot([0, -0.414, -2.414, -5.586, -8, -10.141], [-1.414, -1.414, -3.414, -3.414, -1, -3.141], color='black')
plt.plot(a, b, ':', linewidth=1)
for i in range(len(Path)-1):
    plt.plot(Path[i,0], Path[i,1], 'o', color='black', linewidth=1.5)
plt.ylabel('Position Y')
plt.xlabel('Position X')
plt.title('Position')
plt.legend(['path', 'last', 'start', 'obstacle', '', '', 'waypoint'])

# Plot orientation vs time
plt.figure()
plt.plot( X[2,:])
plt.axhline(y=goal[2], linestyle='--', color='black', linewidth=1.5)
plt.ylabel('Orientation')
plt.xlabel('Time')
plt.title('Orientation vs Time')

# Plot X-coordinate vs time
plt.figure()
plt.plot(X[0,:])
plt.ylabel('X-coordinate')
plt.xlabel('Time')

# Plot Y-coordinate vs time
plt.figure()
plt.plot(X[1,:])
plt.ylabel('Y-coordinate')
plt.xlabel('Time')

# Plot surface vs time
plt.figure()
plt.plot(surf)
plt.ylabel('Surface')
plt.xlabel('Time')

# Plot linear velocity vs time
plt.figure()
plt.plot(V)
plt.ylabel('Linear Vel (v)')
plt.xlabel('Time')

# Plot omega vs time
plt.figure()
plt.plot(W)
plt.ylabel('Omega')
plt.xlabel('Time')

plt.show()
