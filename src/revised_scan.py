#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 10)
scan = LaserScan()

def callback(msg):
    current_time = rospy.Time.now()
    scan.header.stamp = current_time
    scan.header.frame_id = 'laser'
    scan.angle_min = -3.1415
    scan.angle_max = 3.1415
    scan.angle_increment = 0.0174533
    scan.time_increment = 1.0 / (1800 * 360)
    scan.range_min = 0.12
    scan.range_max = 3.5
    scan.ranges = msg.ranges
    scan.intensities = msg.intensities
    pub.publish(scan)

def listener():
    rospy.init_node('revised_scan', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
