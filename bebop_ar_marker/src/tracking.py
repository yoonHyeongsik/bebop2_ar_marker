#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped, Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

class DetectMarker():

	def __init__(self):
		rospy.init_node("bebop_ar_marker")

		# Read in an optional list of valid tag ids
		self.tag_ids = rospy.get_param('~tag_ids', None)

		# Publish the COG on the /target_pose topic as a PoseStamped message
		self.tag_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=5)
		#self.mov_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=5)
		rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tags)

		rospy.loginfo("Publishing combine dmarkers Twist...")
				
	def get_tags(self, msg):
		# Initialize the COG as a PoseStamped message
		p = PoseStamped()

		t = Twist()
		# Get the number of markers
		n = len(msg.markers)

		# If no markers detected, just return
		if n == 0:
		    m_flag = 0
		    t.linear.x = 0
		    t.linear.y = 0
		    t.linear.z = 0
		    t.angular.z= 0
		    self.tag_pub.publish(t)
		    rospy.loginfo("1. m_flag = %d"%(m_flag))
		    return
		
		else:
		    m_flag = 1
		    rospy.loginfo("2. m_flag = %d"%(m_flag))

		# Iterate through the tags and sum the x, y and z coordinates            
		for tag in msg.markers:
            
			# Skip any tags that are not in our list
			if self.tag_ids is not None and not tag.id in self.tag_ids:
				continue
			
			p = tag.pose

			# Give the tag a unit orientation
			#p.pose.orientation.w = 1

			# Add a time stamp and frame_id
			p.header.stamp = rospy.Time.now()
			p.header.frame_id = msg.markers[0].header.frame_id
			marker_id = msg.markers[0].id
			rospy.loginfo("marker id :%d"%(marker_id))
			
			if m_flag == 0:
			    rospy.loginfo("Can't found")
			    t.linear.x = 0
			    t.linear.y = 0
			    t.linear.z = 0
			    t.angular.z= 0
			    
			else:
			    if p.pose.position.x < -0.4:
			       rospy.loginfo("Go left")
			       t.linear.x = 0
			       t.linear.y = 0
			       t.linear.z = 0
			       t.angular.z= 0.5
			       
			    elif p.pose.position.x > 0.4:
			       rospy.loginfo("Go Right")
			       t.linear.x = 0
			       t.linear.y = 0
			       t.linear.z = 0
			       t.angular.z= -0.5
			       
			    elif p.pose.position.x < 0.4 and p.pose.position.x > -0.4:
			       rospy.loginfo("1.Stop")
#			       t.linear.x = 0
#			       t.linear.y = 0
#			       t.linear.z = 0
#			       t.angular.z= 0
			       if p.pose.position.z > 1.5:
			           rospy.loginfo("Go Forward")
			           t.linear.x = 0.5
			           t.linear.y = 0
			           t.linear.z = 0
			           t.angular.z= 0
			        
			       elif p.pose.position.z < 0.8:
			           rospy.loginfo("Go Back")
			           t.linear.x = -0.5
			           t.linear.y = 0
			           t.linear.z = 0
			           t.angular.z= 0
			           
			       elif p.pose.position.z < 1.5 and p.pose.position > 0.8:
			           rospy.loginfo("Stop")
			           t.linear.x = 0
			           t.linear.y = 0
			           t.linear.z = 0
			           t.angular.z= 0
			    

			
			# Publish the COG
			#self.tag_pub.publish(p)
			self.tag_pub.publish(t)
  
if __name__ == '__main__':
	try:
		DetectMarker()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("AR Tag Tracker node terminated.")
