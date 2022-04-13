#!/usr/bin/env python3

import rospy
import math
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from robot_vision_lectures.msg import SphereParams

sphere_data = SphereParams()

motion = Bool()

def get_sphere(data):
	global xc
	global yc
	global zc

	xc = data.xc
	yc = data.yc
	zc = data.zc

	
def initiate_motion(data):
	global motion
	motion = data.data
	
	

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	img_sub = rospy.Subscriber("/sphere_params", SphereParams, get_sphere) 
	motion_sub = rospy.Subscriber("/motion", Bool, initiate_motion)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	q_rot = Quaternion()	
	# define a plan variable
	plan = Plan()

	
	
	while not rospy.is_shutdown():
		
		
			
		# try getting the most update transformation between the tool frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
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
		 
		# define coorinates in camera frame
		pt_in_camera = tf2_geometry_msgs.PointStamped()
		pt_in_camera.header.frame_id = 'camera_color_optical_frame'
		pt_in_camera.header.stamp = rospy.get_rostime()
		
		pt_in_camera.point.x= xc
		pt_in_camera.point.y= yc
		pt_in_camera.point.z= zc
		
		
		
		# convert the 3D point to the base frame coordinates
		pt_in_base = tfBuffer.transform(pt_in_camera,'base', rospy.Duration(1.0))
		print('Points in the camera frame:  x= ', format(pt_in_camera.point.x, '.3f'), '(m), y= ', format(pt_in_camera.point.y, '.3f'), '(m), z= ', format(pt_in_camera.point.z, '.3f'),'(m)')
		print('Transformed points in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
		print('-------------------------------------------------')
		
		
		plan_point1 = Twist()
		# define a point close to the initial position
		plan_point1.linear.x = -0.7
		plan_point1.linear.y = -0.23
		plan_point1.linear.z = 0.363
		plan_point1.angular.x = 3.14
		plan_point1.angular.y = 0.0
		plan_point1.angular.z = 1.57
		# add this point to the plan
		plan.points.append(plan_point1)

		
		
		plan_point2 = Twist()
		# point for the ball position
		plan_point2.linear.x = pt_in_base.point.x
		plan_point2.linear.y = pt_in_base.point.y
		plan_point2.linear.z = pt_in_base.point.z
		plan_point2.angular.x = 3.14
		plan_point2.angular.y = 0.0
		plan_point2.angular.z = 1.57
		# add this point to the plan
		plan.points.append(plan_point2)
		
		
		
		# publish the plan if movemnt true

		if motion == True:

			plan_pub.publish(plan)

		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()





