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

#motion = Bool()
motion = False
ball_ready = False 
robot_ready = False
def get_sphere(data):
	global xc
	global yc
	global zc
	global ball_ready 
	ball_ready = True
	xc = data.xc
	yc = data.yc
	zc = data.zc
	
	
def initiate_motion(data):
	global motion
	motion = data.data
	
def get_pos(data):
	global posX
	global posY
	global posZ
	global r
	global p
	global y
	
	posX = data.linear.x
	posY = data.linear.y
	posZ = data.linear.z
	r = data.angular.x
	p = data.angular.y
	y = data.angular.z
	
	robot_ready = True
	
def set_plan(plan,linX,linY,linZ,roll,pitch,yaw)
	point = Twist()
		
	point.linear.x = linX
	point.linear.y = linY
	point.linear.z = linZ
	point.angular.x = roll
	point.angular.y = pitch
	point.angular.z = yaw
		
	plan.points.append(point)
	

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	img_sub = rospy.Subscriber("/sphere_params", SphereParams, get_sphere) 
	motion_sub = rospy.Subscriber("/motion", Bool, initiate_motion)
	pos_sub = rospy.Subscriber("/ur5e/toolpose",Twist,get_pos)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	q_rot = Quaternion()	
	# define a plan variable
	plan = Plan()

	plan_generated = False
	
	while not rospy.is_shutdown():

		if robot_ready and ball_ready and not plan_generated:	 
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
			
			#roll, pitch, yaw = 3.12614, 0.0166, 1.5308
			
			set_plan(posX,posY,posZ,r,p,y,plan)
			set_plan(pt_in_base.point.x,pt_in_base.point.y,pt_in_base.point.z + 0.1,r,p,y,plan)
			set_plan(pt_in_base.point.x,pt_in_base.point.y,pt_in_base.point.z + 0.015,r,p,y,plan)
			set_plan(pt_in_base.point.x,pt_in_base.point.y,pt_in_base.point.z + 0.1,r,p,y,plan)
			set_plan(pt_in_base.point.x + 0.3,pt_in_base.point.y + 0.1,pt_in_base.point.z + 0.2,r,p,y,plan)
			set_plan(pt_in_base.point.x + 0.3,pt_in_base.point.y + 0.1,pt_in_base.point.z + 0.1,r,p,y,plan)
			
			
			#plan_point1 = Twist()
			# define a point close to the initial position
			#plan_point1.linear.x = -0.0143
			#plan_point1.linear.y = -0.4087
			#plan_point1.linear.z = 0.2743
			#plan_point1.angular.x = roll
			#plan_point1.angular.y = pitch
			#plan_point1.angular.z = yaw
			# add this point to the plan
			#plan.points.append(plan_point1)
			
			#plan_point2 = Twist()
			# point for the ball position
			#plan_point2.linear.x = pt_in_base.point.x
			#plan_point2.linear.y = pt_in_base.point.y
			#plan_point2.linear.z = pt_in_base.point.z + 0.1
			#plan_point2.angular.x = roll
			#plan_point2.angular.y = pitch
			#plan_point2.angular.z = yaw
			# add this point to the plan
			#plan.points.append(plan_point2)
			
			
			#plan_point3 = Twist()
			# point for the ball position
			#plan_point3.linear.x = pt_in_base.point.x
			#plan_point3.linear.y = pt_in_base.point.y
			#plan_point3.linear.z = pt_in_base.point.z + 0.015
			#plan_point3.angular.x = roll
			#plan_point3.angular.y = pitch
			#plan_point3.angular.z = yaw
			# add this point to the plan
			#plan.points.append(plan_point3)
			
			#plan_point4 = Twist()
			# point for the ball position
			#plan_point4.linear.x = pt_in_base.point.x
			#plan_point4.linear.y = pt_in_base.point.y
			#plan_point4.linear.z = pt_in_base.point.z + 0.1
			#plan_point4.angular.x = roll
			#plan_point4.angular.y = pitch
			#plan_point4.angular.z = yaw
			# add this point to the plan
			#plan.points.append(plan_point4)
			
			#plan_point5 = Twist()
			# point for the ball position
			#plan_point5.linear.x = pt_in_base.point.x + 0.3
			#plan_point5.linear.y = pt_in_base.point.y + 0.1
			#plan_point5.linear.z = pt_in_base.point.z + 0.2
			#plan_point5.angular.x = roll
			#plan_point5.angular.y = pitch
			#plan_point5.angular.z = yaw
			# add this point to the plan
			#plan.points.append(plan_point5)
			
			#plan_point6 = Twist()
			# point for the ball position
			#plan_point6.linear.x = pt_in_base.point.x + 0.3
			#plan_point6.linear.y = pt_in_base.point.y + 0.1
			#plan_point6.linear.z = pt_in_base.point.z + 0.1
			#plan_point6.angular.x = roll
			#plan_point6.angular.y = pitch
			#plan_point6.angular.z = yaw
			# add this point to the plan
			#plan.points.append(plan_point6)
			#plan_generated = True
			
		
		# publish the plan if movemnt true

		if motion == True:

			plan_pub.publish(plan)

		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()





