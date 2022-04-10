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
	global sphere_data
	global xc
	global yc
	global zc
	global radius
	sphere_data = data
	xc = data.xc
	yc = data.yc
	zc = data.zc
	radius = data.radius
	
def initiate_motion(data):
	global motion
	motion = data.data
	print(motion)
	

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
		
		if motion:
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
			print('Tool frame position and orientation w.r.t base: x= ', format(x, '.3f'), '(m),  y= ', format(y, '.3f'), '(m), z= ', format(z, '.3f'),'(m)')
			print('roll= ', format(roll, '.2f'), '(rad), pitch= ', format(pitch, '.2f'), '(rad), yaw: ', format(yaw, '.2f'),'(rad)') 
			# define a testpoint in the tool frame (let's say 10 cm away from flange)
			pt_in_tool = tf2_geometry_msgs.PointStamped()
			pt_in_tool.header.frame_id = 'camera_color_optical_frame'
			pt_in_tool.header.stamp = rospy.get_rostime()
			
			pt_in_tool.point.x= xc
			pt_in_tool.point.y= yc
			pt_in_tool.point.z= zc
			
			
			
			# convert the 3D point to the base frame coordinates
			pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
			print('Test point in the TOOL frame:  x= ', format(pt_in_tool.point.x, '.3f'), '(m), y= ', format(pt_in_tool.point.y, '.3f'), '(m), z= ', format(pt_in_tool.point.z, '.3f'),'(m)')
			print('Transformed point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
			print('-------------------------------------------------')
			
			plan_point1 = Twist()
			# just a quick solution to send two target points
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
			# define a point away from the initial position
			plan_point2.linear.x = pt_in_base.point.x - radius
			plan_point2.linear.y = pt_in_base.point.y
			plan_point2.linear.z = pt_in_base.point.z
			plan_point2.angular.x = 3.14
			plan_point2.angular.y = 0.0
			plan_point2.angular.z = 1.57
			# add this point to the plan
			plan.points.append(plan_point2)
			
			# publish the plan
			plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()


