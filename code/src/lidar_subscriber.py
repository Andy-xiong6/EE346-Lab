#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import csv
import os

def lidar_callback(msg):
    angle = 0  
    index = int(angle / msg.angle_increment)  
    distance = msg.ranges[index]  
    csv_file_path = '/home/ubuntu/lidar_data.csv'    
    print(f"Angle: {angle}°, Distance: {distance} m")
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([angle,distance])

def lidar_listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    lidar_listener()
