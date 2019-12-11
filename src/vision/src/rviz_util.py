#!/usr/bin/env python


import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped
from std_msgs.msg import Header, ColorRGBA
import tf

Node = "rviz_util"


# Turns an x, y, z, theta coordinate into a pose
def xytheta_to_pose(x, y, z, theta):
	pose = Pose()
	pose.position.x = x
	pose.position.y = y
	pose.position.z = z
	quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
	pose.orientation.x = quaternion[0]
	pose.orientation.y = quaternion[1]
	pose.orientation.z = quaternion[2]
	pose.orientation.w = quaternion[3]
	return pose


def add_text_label(text, x, y, z, angle, color=ColorRGBA(1.0, 1.0, 1.0, 1.0)):
	marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10, latch=True)
	marker = Marker(
		type=Marker.TEXT_VIEW_FACING,
    	# id=0,
    	# lifetime=rospy.Duration(1.5),
    	pose=xytheta_to_pose(x,y,z,angle),
    	scale=Vector3(0.5, 0.5, 0.5),
    	header=Header(frame_id='map'),
    	color=color,
    	text=text)
	marker_publisher.publish(marker)



def add_door_label():
	marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10, latch=True)
	marker = Marker(
		type=Marker.TEXT_VIEW_FACING,
		action=Marker.ADD,
    	# id=0,
    	# lifetime=rospy.Duration(1.5),
    	pose=xytheta_to_pose(3.5,1.8,0.1,0),
    	scale=Vector3(1,1,1),
    	header=Header(frame_id='map'),
    	color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
    	text="DOOR")
	marker_publisher.publish(marker)

def add_computers_label():
	marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10, latch=True)
	marker = Marker(
		type=Marker.TEXT_VIEW_FACING,
		action=Marker.ADD,
    	# id=0,
    	# lifetime=rospy.Duration(1.5),
    	pose=xytheta_to_pose(-4,1.8,0.1,0),
    	scale=Vector3(1,1,1),
    	header=Header(frame_id='map'),
    	color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
    	text="COMPUTERS")
	marker_publisher.publish(marker)



def main():
	rospy.init_node(Node)
	# add_text_label("Testing", 0, 0, 0, 0)
	add_door_label()
	add_computers_label()

	rospy.spin()

if __name__ == "__main__":
	main()