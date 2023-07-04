#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan_data):
    # Parameters for gap detection
    gap_threshold = 1.0  # Minimum gap width to consider (in meters)
    
    # Initialize variables for closest gap
    closest_gap_width = float('inf')
    closest_gap_center = None
    
    # Loop through all the laser scan ranges
    for i, range_val in enumerate(scan_data.ranges):
        # Check if the range value exceeds the gap threshold
        if range_val > gap_threshold:
            # Calculate gap width
            gap_width = range_val * scan_data.angle_increment
            
            # Update closest gap if necessary
            if gap_width < closest_gap_width:
                closest_gap_width = gap_width
                closest_gap_center = scan_data.angle_min + i * scan_data.angle_increment
    
    if closest_gap_center is not None:
        rospy.loginfo("Closest Gap Width: %.2f meters", closest_gap_width)
        rospy.loginfo("Closest Gap Center: %.2f desgree", closest_gap_center*180/3.1415)
    else:
        rospy.loginfo("No suitable gaps found.")

def gap_detection():
    rospy.init_node('gap_detection')
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

# Run the gap detection function
if __name__ == '__main__':
    try:
        gap_detection()
    except rospy.ROSInterruptException:
        pass
