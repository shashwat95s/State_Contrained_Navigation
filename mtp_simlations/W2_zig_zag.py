#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

x=0.0     #initialise variable
y=0.0

def newOd(msg):
    global x
    global y
    x=msg.pose.pose.position.x   #get x coordinate
    y=msg.pose.pose.position.y   #get y coordinate


rospy.init_node("zigzag")          # initialise the node called move

sub = rospy.Subscriber("/odom",Odometry,newOd)        #to get odometry data
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)  #to send velocity data

a=[]
b=[]
K=1

speed =Twist()          #message type for velocities 
r= rospy.Rate(5)        #set refresh rate

speed.linear.x=0.5
speed.angular.z=0
pub.publish(speed)      #send velocity data to start motion

while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    a.append(x)                             #adding x coordinate
    b.append(y)                             #adding y coordinate
    r.sleep()

#velocity to turn right
speed.linear.x=0
speed.angular.z=0.5
pub.publish(speed)

K=1
while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    r.sleep()
    
#velocity to move forward
speed.linear.x=0.5
speed.angular.z=0
pub.publish(speed)

K=1
while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    a.append(x)                             #adding x coordinate
    b.append(y)                             #adding y coordinate
    r.sleep()

#velocity to turn left
speed.linear.x=0
speed.angular.z=-0.5
pub.publish(speed)

K=1
while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    r.sleep()
    
#velocity to move forward
speed.linear.x=0.5
speed.angular.z=0
pub.publish(speed)

K=1
while not rospy.is_shutdown() and K<21:    #loop to provide delay
    K+=1
    a.append(x)                             #adding x coordinate
    b.append(y)                             #adding y coordinate
    r.sleep()

#velocity to stop
speed.linear.x=0
speed.angular.z=0
pub.publish(speed)

print(a)
print(b)
plt.plot(a,b)
