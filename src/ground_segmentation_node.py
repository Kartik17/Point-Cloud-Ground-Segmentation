#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2,PointField, PointCloud
from sensor_msgs import point_cloud2
import pcl
import struct

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

    ground_points_pub.publish(ros_msg)



def ros_to_array(ros_cloud):
    points_list = []

    for data in point_cloud2.read_points(ros_cloud, skip_nans=True,field_names=("x", "y", "z")):
		points_list.append([data[0], data[1], data[2]])

    ground_segmentation(points_list)

def ground_segmentation(pcd_array, iter_cycle = 10, threshold = 0.40):
	xyz = np.asarray(pcd_array)
	print(xyz.shape)
	height_col = int(np.argmin(np.var(xyz, axis = 0)))

	# Make a new column for index 
	temp = np.zeros((len(xyz[:,1]),4), dtype= float)
	temp[:,:3] = xyz[:,:3]
	temp[:,3] = np.arange(len(xyz[:,1]))
	xyz = temp

	# Filter the points based on the height value
	z_filter = xyz[(xyz[:,height_col]< np.mean(xyz[:,height_col]) + 1.5*np.std(xyz[:,height_col])) & (xyz[:,height_col]> np.mean(xyz[:,height_col]) - 1.5*np.std(xyz[:,height_col]))]
	
	# Normalize as per the height filter value
	max_z, min_z = np.max(z_filter[:,height_col]), np.min(z_filter[:,height_col])
	z_filter[:,height_col] = (z_filter[:,height_col] - min_z)/(max_z - min_z) 

	for i in range(iter_cycle):
		covariance = np.cov(z_filter[:,:3].T)
		w,v,h = np.linalg.svd(np.matrix(covariance))
		normal_vector = w[np.argmin(v)]
		
		#Resample points
		filter_mask = np.asarray(np.abs(np.matrix(normal_vector)*np.matrix(z_filter[:,:3]).T )<threshold)
		z_filter = np.asarray([z_filter[index[1]] for index,a in np.ndenumerate(filter_mask) if a == True])

	z_filter[:,height_col] = z_filter[:,height_col]*(max_z - min_z) + min_z
	world = np.array([row for row in xyz if row[3] not in z_filter[:,3]])

	
	pcl_ground= pcl.PointCloud()
	pcl_ground.from_list(z_filter[:,:3])
	ground_points_msg = pcl_to_ros(pcl_ground)

	
if __name__ == '__main__':
	rospy.init_node('ground_segment', anonymous = True)
	ground_points_pub = rospy.Publisher('/ground_points', PointCloud2,queue_size = 10)
	rospy.Subscriber("/pcl_full_scene", PointCloud2, ros_to_array)
	
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep()

