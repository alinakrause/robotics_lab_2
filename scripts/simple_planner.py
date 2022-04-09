#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
import math

from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams

sphere_data = SphereParams()

def get_sphere(data):
	global sphere_data
	global xc
	global yc
	global zc
	sphere_data = data
	xc = data.xc
	yc = data.yc
	zc = data.zc


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('ros_tf_example', anonymous = True)
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	img_sub = rospy.Subscriber("/sphere_params", SphereParams, get_sphere) 
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	plan = Plan()
	
	
	
	q_rot = Quaternion()	
	while not rospy.is_shutdown():

		# try getting the most update transformation between the tool frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "checkerboard", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available!!!')
			loop_rate.sleep()
			continue
		# extract the xyz coordinates
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# extract the quaternion and converto RPY
		q_rot = trans.transform.rotation
		roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w])
		# a quick check of the readings
		print('Tool frame position and orientation w.r.t base: x= ', format(x, '.3f'), '(m),  y= ', format(y, '.3f'), '(m), z= ', format(z, '.3f'),'(m)')
		print('roll= ', format(roll, '.2f'), '(rad), pitch= ', format(pitch, '.2f'), '(rad), yaw: ', format(yaw, '.2f'),'(rad)') 
		# define a testpoint in the tool frame (let's say 10 cm away from flange)
		pt_in_tool = tf2_geometry_msgs.PointStamped()
		pt_in_tool.header.frame_id = 'checkerboard'
		pt_in_tool.header.stamp = rospy.get_rostime()
		pt_in_tool.point.z= 0.1 # 10 cm away from flange
		# convert the 3D point to the base frame coordinates
		pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
		print('Test point in the TOOL frame:  x= ', format(pt_in_tool.point.x, '.3f'), '(m), y= ', format(pt_in_tool.point.y, '.3f'), '(m), z= ', format(pt_in_tool.point.z, '.3f'),'(m)')
		print('Transformed point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
		print('-------------------------------------------------')
		
		relative_frame_base_checker_x =pt_in_tool.point.x - pt_in_base.point.x
		relative_frame_base_checker_y =pt_in_tool.point.y - pt_in_base.point.y
		relative_frame_base_checker_z =pt_in_tool.point.z - pt_in_base.point.z
		
		
		
		# try getting the most update transformation between the tool frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("camera_color_optical_frame", "checkerboard", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available!!!')
			loop_rate.sleep()
			continue
		# extract the xyz coordinates
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# extract the quaternion and converto RPY
		q_rot = trans.transform.rotation
		roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w])
		# a quick check of the readings
		print('Tool frame position and orientation w.r.t base: x= ', format(x, '.3f'), '(m),  y= ', format(y, '.3f'), '(m), z= ', format(z, '.3f'),'(m)')
		print('roll= ', format(roll, '.2f'), '(rad), pitch= ', format(pitch, '.2f'), '(rad), yaw: ', format(yaw, '.2f'),'(rad)') 
		# define a testpoint in the tool frame (let's say 10 cm away from flange)
		pt_in_tool = tf2_geometry_msgs.PointStamped()
		pt_in_tool.header.frame_id = 'checkerboard'
		pt_in_tool.header.stamp = rospy.get_rostime()
		pt_in_tool.point.z= 0.1 # 10 cm away from flange
		# convert the 3D point to the base frame coordinates
		pt_in_base = tfBuffer.transform(pt_in_tool,'camera_color_optical_frame', rospy.Duration(1.0))
		print('Test point in the TOOL frame:  x= ', format(pt_in_tool.point.x, '.3f'), '(m), y= ', format(pt_in_tool.point.y, '.3f'), '(m), z= ', format(pt_in_tool.point.z, '.3f'),'(m)')
		print('Transformed point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
		print('-------------------------------------------------')
		
		
		relative_frame_camera_checker_x =pt_in_tool.point.x - pt_in_base.point.x
		relative_frame_camera_checker_y =pt_in_tool.point.y - pt_in_base.point.y
		relative_frame_camera_checker_z =pt_in_tool.point.z - pt_in_base.point.z
		
		# wait for 0.1 seconds until the next loop and repeat
		plan_point1 = Twist()
		plan_point1.linear.x = -0.7
		plan_point1.linear.y = -0.23
		plan_point1.linear.z = 0.363
		plan_point1.angular.x = 1.57
		plan_point1.angular.y = 0.0
		plan_point1.angular.z = 0.0
		# add this point to the plan
		#plan.points.append(plan_point1)
		
		plan_point2 = Twist()
		# define a point away from the initial position
		plan_point2.linear.x = -0.6
		plan_point2.linear.y = -0.13
		plan_point2.linear.z = 0.36
		plan_point2.angular.x = 3.14
		plan_point2.angular.y = 0.0
		plan_point2.angular.z = 1.57
		# add this point to the plan
		plan.points.append(plan_point2)
		
		plan_point3 = Twist()
		# define a point away from the initial position
		plan_point3.linear.x = sphere.data.xc + relative_frame_camera_checker_x + relative_frame_base_checker_x
		plan_point3.linear.y = yc + relative_frame_camera_checker_y + relative_frame_base_checker_y
		plan_point3.linear.z = zc + relative_frame_camera_checker_z + relative_frame_base_checker_z
		plan_point3.angular.x = 3.14
		plan_point3.angular.y = 0.0
		plan_point3.angular.z = 1.57
		# add this point to the plan
		plan.points.append(plan_point3)
		
		plan_pub.publish(plan)
		loop_rate.sleep()
