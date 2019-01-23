#!/usr/bin/env python

"""
    ar_tags_cog.py - Version 1.0 2013-11-10
    
    Find the COG of AR tags that are detected in the field of view and publish the
    result as a PoseStamped message on the /target_pose topic
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from geometry_msgs.msg import Point, PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

class DetectMarker():

	def __init__(self):
		rospy.init_node("ar_tags_cog")

		# Read in an optional list of valid tag ids
		self.tag_ids = rospy.get_param('~tag_ids', None)

		# Publish the COG on the /target_pose topic as a PoseStamped message
		self.tag_pub = rospy.Publisher("target_pose", PoseStamped, queue_size=5)

		rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tags)

		rospy.loginfo("Publishing combined tag COG on topic /target_pose...")
				
	def get_tags(self, msg):
		# Initialize the COG as a PoseStamped message
		p = PoseStamped()

		# Get the number of markers
		n = len(msg.markers)

		# If no markers detected, just return
		if n == 0:
			return

		# Iterate through the tags and sum the x, y and z coordinates            
		for tag in msg.markers:
            
			# Skip any tags that are not in our list
			if self.tag_ids is not None and not tag.id in self.tag_ids:
				continue

			rospy.loginfo("------------- id ---------------------------")
			rospy.loginfo("  id = %d"%(tag.id))
			
			# Sum up the x, y and z position coordinates of all tags
			p.pose.position.x = tag.pose.pose.position.x
			p.pose.position.y = tag.pose.pose.position.y
			p.pose.position.z = tag.pose.pose.position.z
			
			p.pose.orientation.x = tag.pose.pose.orientation.x
			p.pose.orientation.y = tag.pose.pose.orientation.y
			p.pose.orientation.z = tag.pose.pose.orientation.z
			p.pose.orientation.w = tag.pose.pose.orientation.w

			rospy.loginfo("------------- positions -------------------")
			rospy.loginfo("  px = %f"%(p.pose.position.x))
			rospy.loginfo("  py = %f"%(p.pose.position.y))
			rospy.loginfo("  pz = %f"%(p.pose.position.z))

			rospy.loginfo("------------- orientaions -----------------")
			rospy.loginfo("  ox = %f"%(p.pose.orientation.x))
			rospy.loginfo("  oy = %f"%(p.pose.orientation.y))
			rospy.loginfo("  oz = %f"%(p.pose.orientation.z))
			rospy.loginfo("  ow = %f"%(p.pose.orientation.w))

			# Give the tag a unit orientation
			#p.pose.orientation.w = 1

			# Add a time stamp and frame_id
			p.header.stamp = rospy.Time.now()
			p.header.frame_id = msg.markers[0].header.frame_id

			# Publish the COG
			self.tag_pub.publish(p)      
  
if __name__ == '__main__':
	try:
		DetectMarker()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("AR Tag Tracker node terminated.")
