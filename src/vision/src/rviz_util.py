#!/usr/bin/env python


import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped
from std_msgs.msg import Header, ColorRGBA
import tf

Node = "rviz_util"


# Turns an x, y, z, theta coordinate into a pose
def xyztheta_to_pose(x, y, z, theta):
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


class Markers:
	def __init__(self, refresh_rate = 1):
		self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
		self.Markers = MarkerArray()
		self.current_id = 1
		self.refresh_rate = refresh_rate
	def publish(self):
		self.publisher.publish(self.Markers)
	def add_text_label(self, x, y, z, theta, text, size=1, color=ColorRGBA(1.0, 1.0, 1.0, 1.0)):
		marker = Marker(
			type=Marker.TEXT_VIEW_FACING,
			action=Marker.ADD,
	    	id=self.current_id,
	    	lifetime=rospy.Duration(self.refresh_rate),
	    	pose=xyztheta_to_pose(x,y,z,theta),
	    	scale=Vector3(size,size,size),
	    	header=Header(frame_id='map'),
	    	color=color,
	    	text=text
		)
		self.Markers.markers.append(marker)
		# print(self.Markers)
		
		self.current_id = self.current_id + 1

	def add_door_label(self, size = 1, color=ColorRGBA(1.0, 1.0, 1.0, 1.0)):
		self.add_text_label(3.5, 1.8, 0.1, 0.0, "DOOR", size, color)
	def add_computers_label(self, size=1, color=ColorRGBA(1.0, 1.0, 1.0, 1.0)):
		self.add_text_label(-4.0, 1.8, 0.1, 0.0, "COMPUTERS", size, color)

	def plot_path(pts):
		# Points should be in the form xyz
		print("Test")


def main():
	rospy.init_node(Node)
	markers = Markers()
	markers.add_door_label()
	markers.add_computers_label()
	while not rospy.is_shutdown():
		markers.publish()
		rospy.sleep(rospy.Duration(markers.refresh_rate))
	

if __name__ == "__main__":
	main()