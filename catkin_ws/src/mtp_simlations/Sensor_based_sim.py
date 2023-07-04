#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist,TransformStamped
from tf.transformations import euler_from_quaternion
import math
import numpy
import numpy as np

from math import sqrt, cos, sin, exp, pi


    

def closest_gap(pos, xc, yc, threshold):
    # xc = xc[~np.isnan(xc)]
    # yc = yc[~np.isnan(yc)]
    dis = []
    for i in range(len(xc) - 1):
        dis.append(np.sqrt((xc[i] - xc[i+1])**2 + (yc[i] - yc[i+1])**2))
    
    gap_indices = []
    midpoints = []
    for i in range(len(dis)):
        if dis[i] > threshold:
            gap_indices.append(i)
    
    for i in range(len(gap_indices)):
        xa = xc[gap_indices[i]]
        ya = yc[gap_indices[i]]
        xb = xc[gap_indices[i]+1]
        yb = yc[gap_indices[i]+1]
        print("xa:",xa,"ya:",ya,"xb:",xb,"yb:",yb)
        mDis = 999
        if np.linalg.norm(pos - np.array([xa, ya])) < np.linalg.norm(pos - np.array([xb, yb])):
            ang = np.arctan2(ya - pos[1], xa - pos[0])
            for j in range(len(xc)):
                tang = np.arctan2(yc[j] - pos[1], xc[j] - pos[0])
                if np.mod(tang - ang, 2 * np.pi) <= 3 * np.pi / 4 and np.mod(tang - ang, 2 * np.pi) > 0:
                    dist = np.linalg.norm(np.array([xa, ya]) - np.array([xc[j], yc[j]]))
                    if dist < mDis:
                        mDis = dist
                        mx = xc[j]
                        my = yc[j]
            midpoints.append([(xa + mx) / 2, (ya + my) / 2])
        else:
            ang = np.arctan2(yb - pos[1], xb - pos[0])
            for j in range(len(xc)):
                tang = np.arctan2(yc[j] - pos[1], xc[j] - pos[0])
                if np.mod(tang - ang, 2 * np.pi) >= 5 * np.pi / 4 and np.mod(tang - ang, 2 * np.pi) > 0:
                    dist = np.linalg.norm(np.array([xb, yb]) - np.array([xc[j], yc[j]]))
                    if dist < mDis:
                        mDis = dist
                        mx = xc[j]
                        my = yc[j]
            midpoints.append([(xb + mx) / 2, (yb + my) / 2])
        # print("xa:",xa,"ya:",ya,"xb:",xb,"yb:",yb)
        # print("mx",mx,"my",my)
    
    if np.sqrt((xc[-1] - xc[0])**2 + (yc[-1] - yc[0])**2) > threshold:
        xa = xc[-1]
        ya = yc[-1]
        xb = xc[0]
        yb = yc[0]
        mDis = 999
        if np.linalg.norm(pos - np.array([xa, ya])) < np.linalg.norm(pos - np.array([xb, yb])):
            ang = np.arctan2(ya - pos[1], xa - pos[0])
            for j in range(len(xc)):
                tang = np.arctan2(yc[j] - pos[1], xc[j] - pos[0])
                if np.mod(tang - ang, 2 * np.pi) <= 3 * np.pi / 4 and np.mod(tang - ang, 2 * np.pi) > 0:
                    dist = np.linalg.norm(np.array([xa, ya]) - np.array([xc[j], yc[j]]))
                    if dist < mDis:
                        mDis = dist
                        mx = xc[j]
                        my = yc[j]
            midpoints.append([(xa + mx) / 2, (ya + my) / 2])
        else:
            ang = np.arctan2(yb - pos[1], xb - pos[0])
            for j in range(len(xc)):
                tang = np.arctan2(yc[j] - pos[1], xc[j] - pos[0])
                if np.mod(tang - ang, 2 * np.pi) >= 5 * np.pi / 4 and np.mod(tang - ang, 2 * np.pi) > 0:
                    dist = np.linalg.norm(np.array([xb, yb]) - np.array([xc[j], yc[j]]))
                    if dist < mDis:
                        mDis = dist
                        mx = xc[j]
                        my = yc[j]
            midpoints.append([(xb + mx) / 2, (yb + my) / 2])
    
    return midpoints


def motionDir(pos):
    if pos[0] < -3:
        direction = -pi/2
    else:
        direction = -pi
    return direction


def chooseGoalByOrientation(goals, startPosition, desiredOrientation):
    # Calculate the orientation of each goal point relative to the start position
    goals = np.array(goals)
    orientations = np.arctan2(goals[:,1]-startPosition[1], goals[:,0]-startPosition[0])
    
    # Compute the angular difference between the orientations and desired orientation
    angle = (orientations - desiredOrientation)
    angle = angle - 2*np.pi*(angle > np.pi)
    angle = angle + 2*np.pi*(angle < -np.pi)
    angularDifference = np.abs(angle)
    
    # Find the goal point with the minimum angular difference
    idx = np.argmin(angularDifference)
    
    # Select the closest goal point
    selectedGoal = goals[idx, :]
    
    return selectedGoal



def findNearestObstacle(obstacles, target):
    # Initialize variables
    minDistance = np.inf
    nearestObstacle = None

    # Iterate through each obstacle
    for i in range(obstacles.shape[0]):
        obstacle = obstacles[i]

        # Calculate distance between obstacle and target point
        distance = np.linalg.norm(obstacle - target)

        # Update minimum distance and nearest obstacle if necessary
        if distance < minDistance:
            minDistance = distance
            nearestObstacle = obstacle

    return nearestObstacle, minDistance


def newSP(curr, goal, K, xc, yc):
    l = 0
    dist = np.linalg.norm(goal - curr)
    angle = np.arctan2(goal[1] - curr[1], goal[0] - curr[0])

    gx = curr[0]
    gy = curr[1]
    r2 = 0.1

    inc = 0.01

    while np.linalg.norm(curr - np.array([gx, gy])) <= r2:
        l = l + inc
        gx = l * np.cos(angle) + curr[0]
        gy = l * np.sin(angle) + curr[1]
        _, minDistance = findNearestObstacle(np.column_stack((xc, yc)), np.array([gx, gy]))
        r1 = minDistance
        r2 = r1 / np.sqrt(1 + K ** 2)

    l = l - inc

    if l > dist:
        l = dist
        dir = motionDir(np.array([gx, gy]))
        sp = np.array([goal[0], goal[1], dir])
    else:
        sp = np.array([gx, gy, angle])

    return sp




def reached(A, B):
    if np.sqrt((A[1] - B[1]) ** 2 + (A[0] - B[0]) ** 2) < 0.05 and ((np.abs(A[2] - B[2]) % (2 * np.pi)) < 0.0173 * 1 or (np.abs(A[2] - B[2]) % (2 * np.pi)) > 2 * np.pi - 0.0173 * 1):
        t = 1
    else:
        t = 0
    return t



def getLidarCoordinates(ranges, angles, robotPosition):
    # Convert robot position to radians
    robotAngle = robotPosition[2]
    
    # Calculate the coordinates of each reading
    x = ranges * np.cos(angles + robotAngle) + robotPosition[0]
    y = ranges * np.sin(angles + robotAngle) + robotPosition[1]
    
    return x, y

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

from sensor_msgs.msg import LaserScan

def newSc(data):
    # Access the laser scan data
    global ranges
    global angles
    ranges = data.ranges
    angles=np.linspace(0, data.angle_max, 360)
    # print("Angles")

    # print(data.angle_min)
    # print(data.angle_max)
    # print(data.angle_increment)

def map_angle(t):
    if(t > pi):
        t = -(2*pi-t)
    elif(t < -pi):
        t = (2*pi+t)
    return t

rospy.init_node("zigzag")          # initialise the node called move

sub = rospy.Subscriber("/odom",Odometry,newOdom)        #to get odometry data
sub1 = rospy.Subscriber("/scan",LaserScan,newSc)        #to get laser data
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)  #to send velocity data

velocity_msg = Twist()
r= rospy.Rate(40)        #set refresh rate
global ranges
global angles 
global x
global y
global theta
x=[]
y=[]
theta=[]
ranges=[]
angles=[]

new_g=0
old_g=0
x=2
y=1.6
theta=pi
ang_new=theta
ang_old=theta
x_start = np.array([x, y, theta])
X = np.zeros((3, 1))
X[:, 0] = x_start

print(X)
t_euler = [0]
Path = []
V = [0]
W = [0]
R = [0]

Path = np.empty((0, 2))
surf=[]

K = 1
K1 = 0.1
Ks = 0.2
fl = 0

# X = np.zeros((3, 1))

while not rospy.is_shutdown():    #loop to provide delay
    try:
        while len(ranges)<1:
           r.sleep() 
           # X[:, 0] = np.array([x,y,theta])
        # print("Ranges:")
        # print(ranges)
        # print("Angles:")
        # print(angles)
        xr, yr = getLidarCoordinates(ranges, angles,[x,y,theta])
        xr, yr = xr[np.isfinite(xr)], yr[np.isfinite(yr)]
        

        # print(xr)

        a = closest_gap([x,y], xr, yr, 0.75)
        
        a = np.array(a)

        if x < -3:
            in_dir = -np.pi/2
        else:
            in_dir = -np.pi

        selected_goal = chooseGoalByOrientation(a, [x,y], in_dir)
        
        goal = newSP([x,y], selected_goal, K, xr, yr)
        print("goal:",goal)

        # plt.figure(1)
        # plt.scatter(xr, yr)
        # plt.plot(selected_goal[0], selected_goal[1], 'o', color='green', linewidth=1.5)
        # plt.plot(X[0, -1], X[1, -1], 'o', color='red', linewidth=1.5)
        # plt.show()
        
        Path = np.vstack((Path, [goal[0], goal[1]]))
        # Path.append([goal[0], goal[1]])

        if (np.mod(X[2, -1] - goal[2], 2 * np.pi)) > np.pi:
            sg = -1
            K = -1
        else:
            sg = 1
            K = 1

        ad=0

        # old_g=new_g
        # new_g=goal
        # if new_g!=old_g:
        while True:             # loop to increase angle 
            x_start = X[:, -1]
            r2 = np.sqrt((x_start[0] - goal[0]) ** 2 + (x_start[1] - goal[1]) ** 2)
            r1 = r2 * np.sqrt(1 + K ** 2)
            r3 = r2 * r1 * np.sqrt((K ** 2) + 4) * np.exp(-K * (x_start[2] - goal[2])) / (r2 * sg * K)
            if r3 < 0.1:
                break
            else:
                # ad=ad+ sg * 2 * np.pi
                X[2, -1] = X[2, -1] + sg * 2 * np.pi

        j = 0
        sumS = 0
        print(goal)
        while not reached(X[:, -1], goal):      #loop to run to interim goal
            j = j + 1
            if j > 10:
                break
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
            sumS = sumS + np.sign(S) * (1/40)
            if fl == 0:
                surf = np.append(surf, S)
                surf = np.append(surf, S)
                fl = 1
            else:
                surf = np.append(surf, S)
            if np.abs(S) < 0.05:
                omega = (-K1 * z1)
                v = (-K1 * z1 * (z3 + K * z2))
            else:
                omega = 0
                v = (-Ks * np.sign(S) * np.sqrt(np.abs(S)))# - 0.5 * sumS
            omega = -K1 * z1
            v = (-K1 * z1 * (z3 + K * z2)) - Ks * np.sign(S) * np.sqrt(np.abs(S)) - 0.1 * sumS
            V = np.append(V, v)
            W = np.append(W, omega)
            linear_vel = v
            velocity_msg.linear.x = linear_vel
            angular_vel = omega
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            # print(velocity_msg)
            r.sleep()
            #X = np.column_stack((X, X[:, -1] + [v * np.cos(X[2, -1]), v * np.sin(X[2, -1]), omega] * samp_T))
            ang_old=ang_new
            ang_new=theta
            diff=map_angle(ang_new-ang_old)

            
            X = np.column_stack((X,[x,y,X[2, -1]+diff]))

        # break

        # print(len(ranges))

        # linear_vel = v
        # velocity_msg.linear.x = linear_vel
        # angular_vel = omega
        # velocity_msg.angular.z = angular_vel
        # pub.publish(velocity_msg)
        # print(velocity_msg)
        # r.sleep()

    except KeyboardInterrupt:
        break
        
    # r.sleep()

#velocity to turn right
velocity_msg.linear.x=0
velocity_msg.angular.z=0
print(velocity_msg)


# a, b = [], []
# for i in range(1, 88):
#     a_, b_ = path_l(i/10)
#     a.append(a_)
#     b.append(b_)
# Plot position


# p = 1
# a1 = np.zeros(1170)
# b1 = np.zeros(1170)

# for i in np.arange(0, 11.61, 0.01):
#     a1[p], b1[p] = obs_1(i)
#     p += 1

# p = 1
# a2 = np.zeros(1060)
# b2 = np.zeros(1060)

# for i in np.arange(0, 10.51, 0.01):
#     a2[p], b2[p] = obs_2(i)
#     p += 1

# Plot position
plt.plot(X[0,:], X[1,:])
plt.plot(X[0,-1], X[1,-1], 'o', color='red', linewidth=1.5)
plt.plot(X[0,0], X[1,0], 'o', color='green', linewidth=1.5)
# plt.plot(a1, b1, color='black')
# plt.plot(a2, b2, color='black')
# plt.plot(a, b, ':', linewidth=1)
for i in range(len(Path)-1):
    plt.plot(Path[i,0], Path[i,1], 'o', color='black', linewidth=1.5)
plt.ylabel('Position Y')
plt.xlabel('Position X')
plt.title('Position')
plt.legend(['path', 'last', 'start', 'waypoint'])

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


# plt.figure(1)
# plt.scatter(xr, yr)
# plt.scatter(x, y,color='green')
# plt.plot(goal[0], goal[1], 'o', color='black', linewidth=1.5)
# plt.plot(selected_goal[0], selected_goal[1], 'o', color='red', linewidth=1.5)

# plt.show()