#! /usr/bin/env python3

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#from detect_gaps import gaps,range_smoother,R_max_finder,waypoint_finder

global xdata,ydata,line,scan

def callback(msg):
    global scan
    scan = msg
    #print(Gaps)

def range_smoother(range_info,min,max):
    smooth_array = []
    for i in range(len(range_info)):
        if(range_info[i]==min):
            continue
        if(range_info[i]==max):
            continue
        smooth_array.append(range_info[i])
    return smooth_array

def main_live_plotter():

    rospy.init_node('Live_Plotter')

    global xdata,ydata,line,scan

    sub = rospy.Subscriber('/tb3_1/scan',LaserScan,callback)

    fig, ax = plt.subplots(figsize=(100, 100), subplot_kw={'projection': 'polar'})
    line, = ax.plot([], [], lw = 2)
    #line = ax.scatter([], [])
    # initializing empty values
    # for x and y co-ordinates
    xdata, ydata = [], []

    while not rospy.is_shutdown():

        # calling the animation function	
        anim = animation.FuncAnimation(fig, animate,init_func = init,frames = 5000,interval = 20,blit = True)
        #plt.clf()
        # saves the animation in our desktop
        #anim.save('growingCoil.mp4', writer = 'ffmpeg', fps = 30)
        #xdata = np.linspace(scan.angle_min,scan.angle_max,num = len(scan.ranges))
        #ydata = scan.ranges
        #line.set_data(xdata, ydata)

        #plt.pause(0.5)
        plt.show()

# what will our line dataset
# contain?
def init():
    global line
    line.set_data([], [])
    return line,



# animation function
def animate(i):

    global xdata,ydata,line,scan
    xdata = []
    ydata = []
	# appending values to the previously
	# empty x and y data holders
    #for i in range(len(Gaps)):
        #for j in range(2):
            #xdata.append(scan.angle_min + Gaps[i][j]*scan.angle_increment)
            #ydata.append(scan.ranges[Gaps[i][j]])
    
    
    ydata = range_smoother(scan.ranges,scan.range_min,scan.range_max)
    #ydata = R_max_set
    
    xdata = np.linspace(scan.angle_min,scan.angle_max,num = len(ydata))
    if(len(ydata)>0):
        line.set_data(xdata, ydata)
    #line.set_offsets([[xdata],[ydata]])

    return line,




if __name__ == '__main__':
    try:
        main_live_plotter()
    except rospy.ROSInterruptException:
        pass