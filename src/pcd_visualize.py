#!/usr/bin/env python

import rospy
import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
import struct
import ctypes
from random import randint

def parse_bin_data(bin_content):
    print('fetching new lidar data...')
    points = []
    line_width = 16
    for lindex in range(0, int(len(bin_content) / line_width)):
        points.append([
            struct.unpack('f', bin_content[lindex * line_width:(lindex * line_width) + 4])[0],
            struct.unpack('f', bin_content[lindex * line_width + 4:lindex * line_width + 8])[0],
            struct.unpack('f', bin_content[lindex * line_width + 8:lindex * line_width + 12])[0]
        ])
    return points


def pcl_to_ros(pcl_array): 
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "world"
    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="i",
                            offset=12,
                            datatype=PointField.FLOAT32, count=1))
  


    ros_msg.is_bigendian = False
    ros_msg.point_step = 12
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
    	float_rgb = struct.unpack('f', struct.pack('i', 0xff0000))[0]
    	try:
        	buffer.append(struct.pack('fff', data[0], data[1], data[2]))
        except Exception as e:
        	print(e)
        else:
        	pass
    ros_msg.data = "".join(buffer)

    return ros_msg

if __name__ == '__main__':
    
    rospy.init_node('pointcloud', anonymous = True)
    pcl_pub = rospy.Publisher("/pcl_full_scene",PointCloud2,queue_size=10)
    
    p = pcl.PointCloud()
    with open('../sample_kitti_data/000000.bin', mode='rb') as file: # b is important -> binary
        fileContent = file.read()
    point_array = parse_bin_data(fileContent)
    p.from_list(point_array)
    ros_pointcloud = pcl_to_ros(p)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        print("Node_Running")       
        pcl_pub.publish(ros_pointcloud)

        rate.sleep()
