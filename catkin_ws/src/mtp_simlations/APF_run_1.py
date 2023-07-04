#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
from math import atan2, pi,cos,sin
import numpy as np
# import scipy.io



# Load .mat file
# x_data = scipy.io.loadmat('x_goal.mat')
# y_data = scipy.io.loadmat('y_goal.mat')

# goal_x = x_data['x_goal']
# goal_y = x_data['y_goal']

def quat2euler(x,y,z,w):
    quat = [x,y,z,w]
    return euler_from_quaternion(quat)


def goal_x(p):
    global ref_path
    return ref_path[p][0]
def goal_y(p):
    global ref_path
    return ref_path[p][1]
def ang_sat(th):
    if th>pi:
        th-=2*pi
    elif th<-pi:
        th+=2*pi
    return th
        
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


def ref_transformer(ref,initial_pos, target_gap):
    x_gap_sim=106
    y_gap_sim=161
    last_x=ref[0,-1]
    last_y=ref[1,-1]
    d=((x_gap_sim-last_x)**2+(y_gap_sim-last_y)**2)**0.5
    ang=atan2(y_gap_sim-last_y,x_gap_sim-last_x)

    M=[[initial_pos[0],initial_pos[1],0,0],[0,0,initial_pos[0],initial_pos[1]],[target_gap[0]-d*cos(ang),target_gap[1]-d*sin(ang),0,0],[0,0,target_gap[0]-d*cos(ang),target_gap[1]-d*sin(ang)]]

    P=np.linalg.inv(M)*[[ref[0,0]],[ref[0,1]],[ref[-1,0]],[ref[-1,1]]]

    for x in ref:
        mod_ref[k,0]=P[0]*x[k,0]


# def newOd(msg):
#     global x
#     global y
#     global theta
#     x=msg.pose.pose.position.x   #get x coordinate
#     y=msg.pose.pose.position.y   #get y coordinate
#     ori = msg.pose.pose.orientation
#     (roll, pitch, theta) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

from geometry_msgs.msg import Twist,TransformStamped
rospy.init_node("point1")          # initialise the node called move

# sub = rospy.Subscriber("/odom",Odometry,newOd)        #to get odometry data
rospy.Subscriber('/vicon/tb3_1/tb3_1', TransformStamped, callback_vicon) #Insert Vicon Subscriber here
# pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)  #to send velocity data
pub = rospy.Publisher("/tb3_1/cmd_vel",Twist,queue_size=5)  #to send velocity data


import csv

def read_csv_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            data.append(row)
    return data

global ref_path
file_path = 'output_scaled.txt'
ref_path = read_csv_file(file_path)

a=[]
b=[]
e=[0,0,0]
u=[0,0,0]
h=1/20.0
k=0
t=2
p=0

kp_th=0.1               
kp_dis=0.4
speed =Twist()          #message type for velocities 
r= rospy.Rate(20)        #set refresh rate
kp=0.1
ki=0.00
kd=0.00


x=0.0     #initialise variable
y=0.0
theta=0.0

# speed.linear.x=0.5
# speed.angular.z=0
# pub.publish(speed)      #send velocity data to start motion
# yout = open("output_y.txt", "a")
# xout = open("output_x.txt", "a")
i=0


while x==0:
    r.sleep()

xd=x-float(ref_path[0][0])
yd=y-float(ref_path[0][1])
for i in range(len(ref_path)):
    ref_path[i][0]=float(ref_path[i][0])+xd
    ref_path[i][1]=float(ref_path[i][1])+yd

print(ref_path)

while not rospy.is_shutdown():    #loop to provide delay




    if k<75:
        
        des_ang=atan2(goal_y(k)-y,goal_x(k)-x)
        print('goal')
        print([goal_x(k),goal_y(k)])
        print('curr_pose')
        print([x,y,theta])
        print('errors')
        ang_err=ang_sat(des_ang-theta)
        dis = ((goal_y(k)-y)**2+(goal_x(k)-x)**2)**0.5
        print([dis,ang_err])
        e.append(dis*cos(ang_err))
        t+=1
        # u.append(u[t-1]+kp*(e[t]-e[t-1])+ki*h*e[t]+kd*(e[t]-e[t-1]+e[t-2])/h)

        speed.linear.x=-kp_dis*dis
        speed.angular.z=-kp_th*ang_err
        print(speed)
        pub.publish(speed)
            
        #else:
            #speed.linear.x=0
            #speed.angular.z=0
            #pub.publish(speed)
    else:
        speed.linear.x=0
        speed.angular.z=0
        pub.publish(speed)
        
        break
    a.append(x)                             #adding x coordinate
    b.append(y)                             #adding y coordinate
    # xout.write("%s ," % x)
    # yout.write("%s ," % y)
#     e.append(dis)
    
    if p>10:
        p=0
    else:
        p+=1;
    
    if p==1:
        k+=1
#     print(k)
    r.sleep()


#velocity to stop
speed.linear.x=0
speed.angular.z=0
pub.publish(speed)
# xout.close()
# yout.close()

# print(a)
# print(b)
plt.plot(a,b)
# plt.plot(e)
