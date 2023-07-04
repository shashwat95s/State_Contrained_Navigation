#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
from sympy import Point, Polygon
from statistics import stdev

# creating points using Point()
p1, p2, p3, p4 = map(Point, [(0, 1), (0, 2), (-1, 1), (-1, 2)])
  
# creating polygon using Polygon()
poly = Polygon(p1, p2, p3, p4)

x=0.0     #initialise variable
y=0.0
l=[]
di=[]
diff=[]
def newOd(msg):
    global x
    global y
    x=msg.pose.pose.position.x   #get x coordinate
    y=msg.pose.pose.position.y   #get y coordinate

def newSc(msg):
    global least
    global l
    global di
    global poly
    global x
    global y
    global diff
    up=msg.range_max
    dn=msg.range_min
    ls=msg.ranges
    A1 = np.array(ls)  
    least=min(A1)
    sDist = float(poly.distance(Point(x,y)))
    l.append(least) 
    di.append(sDist)
    diff.append(least-sDist)


rospy.init_node("zigzag")          # initialise the node called move

sub = rospy.Subscriber("/odom",Odometry,newOd)        #to get odometry data
sub1 = rospy.Subscriber("/scan",LaserScan,newSc)        #to get laser data
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)  #to send velocity data

a=[]
b=[]

K=1

speed =Twist()          #message type for velocities 
r= rospy.Rate(5)        #set refresh rate

speed.linear.x=0.3
speed.angular.z=0
pub.publish(speed)      #send velocity data to start motion

while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    a.append(x)                             #adding x coordinate
    b.append(y) 
    # l.append(least)                            #adding y coordinate
    r.sleep()

#velocity to turn right
speed.linear.x=0
speed.angular.z=0.2
pub.publish(speed)

K=1
while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    r.sleep()
    
#velocity to move forward
speed.linear.x=0.3
speed.angular.z=0
pub.publish(speed)

K=1
while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    a.append(x)                             #adding x coordinate
    b.append(y)                             #adding y coordinate
    # l.append(least)     
    r.sleep()

#velocity to turn left
speed.linear.x=0
speed.angular.z=-0.2
pub.publish(speed)

K=1
while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    r.sleep()
    
#velocity to move forward
speed.linear.x=0.3
speed.angular.z=0
pub.publish(speed)

K=1
while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    a.append(x)                             #adding x coordinate
    b.append(y)                             #adding y coordinate
    # l.append(least)     
    r.sleep()

#velocity to turn right
speed.linear.x=0
speed.angular.z=0.2
pub.publish(speed)

K=1
while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    r.sleep()
    
#velocity to move forward
speed.linear.x=0.3
speed.angular.z=0
pub.publish(speed)

K=1
while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    a.append(x)                             #adding x coordinate
    b.append(y)                             #adding y coordinate
    # l.append(least)     
    r.sleep()


#velocity to stop
speed.linear.x=0
speed.angular.z=0
pub.publish(speed)

A2 = np.array(diff) 
print("standard deviation:")
print(stdev(A2))

plt.figure(1)
plt.plot(a,b)
plt.plot([0,0],[1,2],[0,-1],[2,2],[-1,-1],[2,1],[-1,0],[1,1])
plt.xlabel('x co-ordinate')
plt.ylabel('y co-ordinate')
plt.figure(2)
plt.plot(l,label='Measured Distance')
plt.plot(di,label='Calculated Distance')
plt.xlabel('Time')
plt.ylabel('Least Distance')
plt.legend()
plt.show()
