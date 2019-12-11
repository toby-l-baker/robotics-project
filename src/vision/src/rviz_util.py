#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped
from std_msgs.msg import Header, ColorRGBA
import tf
from path_planner.msg import NavigationTargets

Node = "rviz_util"
Path_Topic = "/path_plan"



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
		self.subscriber  = rospy.Subscriber(Path_Topic, NavigationTargets, self.path_plan_cb) 
		
		self.Markers = MarkerArray()
		self.current_id = 1
		self.refresh_rate = refresh_rate
		self.nav_targets = NavigationTargets()



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
		print("Added text label")

	def add_door_label(self, size = 1, color=ColorRGBA(1.0, 1.0, 1.0, 1.0)):
		self.add_text_label(3.5, 1.8, 0.1, 0.0, "DOOR", size, color)
	def add_computers_label(self, size=1, color=ColorRGBA(1.0, 1.0, 1.0, 1.0)):
		self.add_text_label(-4.0, 1.8, 0.1, 0.0, "COMPUTERS", size, color)
	
	def add_leader_plot(self, msg, height=0.25):
		print("Adding leader.")
		leader_marker = Marker()
		leader_marker.id=self.current_id
		leader_marker.action=Marker.ADD
		leader_marker.header.frame_id = 'map'
		leader_marker.type = Marker.LINE_STRIP
		leader_marker.ns = "points"
		leader_marker.scale.x = 0.25
		leader_marker.colors = []

		leader_marker.points = []
		leader_marker.points.append(Point(msg.leader.initial[0], msg.leader.initial[1], height))
		leader_marker.points.append(Point(msg.leader.line_start[0], msg.leader.line_start[1], height))
		leader_marker.points.append(Point(msg.leader.line_end[0], msg.leader.line_end[1], height))
		leader_marker.points.append(Point(msg.leader.goal[0], msg.leader.goal[1], height))
		leader_marker.colors.append(ColorRGBA(1.0, 0.0, 0.0, 1.0))
		leader_marker.colors.append(ColorRGBA(1.0, 0.0, 0.0, 1.0))
		leader_marker.colors.append(ColorRGBA(1.0, 0.0, 0.0, 1.0))
		leader_marker.colors.append(ColorRGBA(1.0, 0.0, 0.0, 1.0))
		leader_marker.lifetime=rospy.Duration(self.refresh_rate)

		self.current_id = self.current_id + 1
		self.Markers.markers.append(leader_marker)


	def add_follower_plot(self, msg, height=0.25):
		print("Adding follower")
		follower_marker = Marker()
		follower_marker.id=self.current_id
		follower_marker.action=Marker.ADD
		follower_marker.header.frame_id = 'map'
		follower_marker.type = Marker.LINE_STRIP
		follower_marker.ns = "points"
		follower_marker.scale.x = 0.25
		follower_marker.colors = []

		follower_marker.points = []
		follower_marker.points.append(Point(msg.follower.initial[0], msg.follower.initial[1], height))
		follower_marker.points.append(Point(msg.follower.line_start[0], msg.follower.line_start[1], height))
		follower_marker.points.append(Point(msg.follower.line_end[0], msg.follower.line_end[1], height))
		follower_marker.points.append(Point(msg.follower.goal[0], msg.follower.goal[1], height))
		follower_marker.colors.append(ColorRGBA(0.0, 0.0, 1.0, 1.0))
		follower_marker.colors.append(ColorRGBA(0.0, 0.0, 1.0, 1.0))
		follower_marker.colors.append(ColorRGBA(0.0, 0.0, 1.0, 1.0))
		follower_marker.colors.append(ColorRGBA(0.0, 0.0, 1.0, 1.0))
		follower_marker.lifetime=rospy.Duration(self.refresh_rate)
		self.Markers.markers.append(follower_marker)
		self.current_id = self.current_id + 1



	def path_plan_cb(self, msg, size=1,):
		print("Inside path plan callback")
		self.add_follower_plot(msg)
		rospy.sleep(0.5)
		self.add_leader_plot(msg)

		



		# print(follower_marker)
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