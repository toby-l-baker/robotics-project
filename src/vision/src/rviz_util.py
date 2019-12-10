#!/usr/bin/env python


import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import tf

Node = "Add Marker"


# Turns an x, y, z, theta coordinate into a pose
def xytheta_to_pose(x, y, z, theta):
	pose = PS()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "map"
	pose.pose.position.x = x
	pose.pose.position.y = y
	pose.pose.position.z = z
	quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
	pose.pose.orientation.x = quaternion[0]
	pose.pose.orientation.y = quaternion[1]
	pose.pose.orientation.z = quaternion[2]
	pose.pose.orientation.w = quaternion[3]
	return pose


def add_text_label(text, x, y, z, angle, color=ColorRGBA(0.0, 1.0, 0.0, 0.8)):
	marker_publisher = rospy.Publisher('visualization_marker', Marker)
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=xytheta_to_pose(x,y,z,angle)
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='map'),
                color=color,
                text=text)
    marker_publisher.publish(marker)


	

def main():
	rospy.init_node(Node, anonymous=True)
	add_text_label("Testing", 0, 0, 0, 0)
	rospy.spin()

if __name__ == "__main__":
	main()