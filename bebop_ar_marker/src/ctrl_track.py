#!/usr/bin/env python
 
from __future__ import print_function
import rospy, os, time, sys, termios, select, atexit, tty, math
from geometry_msgs.msg import Point, PoseStamped,Twist
from std_msgs.msg import Empty, Bool
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospy
 
msg = """
Control your Bebop2.0!
-----------------------------------------------------------
Left hand around:                    Right hand around:
        w                                    u                      
   a    s    d                         h     j     k

' ' : stop( hover )

w/s : going   up / down
a/d : rotate ccw / cw
i/k : go  foward / backward
j/l : go    left / righ
 
' ' : hovering
 1  : take off
 2  : landing
 3  : emergency
 ~  : tracking 
+/- : increase / decrease speed
 
CTRL-C to quit
"""
e = """
Communications Failed
"""
#         +---+-------------+---+-------------+---+-------------+---+-------------+---+------------+
#         |'w'| linear.z +  |'a'| angular.z + |'i'| linear.x +  |'j'| linear.y +  |' '|all param=0 |
#         +---+-------------+---+-------------+---+-------------+---+-------------+---+------------+
#         |'s'| linear.z -  |'d'| angular.z - |'k'| linear.x -  |'l'| linear.y +  |   |            |
#         +---+-------------+---+-------------+---+-------------+---+-------------+---+------------+
action = { 'w':( 0, 0, 1, 0),'a':( 0, 0, 0, 1),'i':( 1, 0, 0, 0),'j':( 0, 1, 0, 0),' ':( 0, 0, 0, 0),
           's':( 0, 0,-1, 0),'d':( 0, 0, 0,-1),'k':(-1, 0, 0, 0),'l':( 0,-1, 0, 0), }
"""
class GetMedia:
    def __init__(self):
        
        self.MEDIA_PATH  = '/internal_000/Bebop_2/media'
                                            # Filepath on the Bebop2
        self.IMG_NAME_HD = ''
        
        try:
            self.ftp = FTP('192.168.42.1')  # IP-Address of the drone itself
            login = self.ftp.login()
            print("FTP login is %s" % login)
            
        except:
            print("ERROR: ftp login is Fail.")
            self.ftp = None

    def _close(self):
        if (self.ftp is not None):
            self.ftp.close()
    
    def get_img(self,fname):
        pass
"""

global lx
global ly
global lz
global az

class DetectMarker():

    def __init__(self):
        #rospy.init_node("bebop_ar_marker")

        # Read in an optional list of valid tag ids
        self.tag_ids = rospy.get_param('~tag_ids', None)

        # Publish the COG on the /target_pose topic as a PoseStamped message
        # self.tag_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=5)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tags)

        rospy.loginfo("Publishing combine dmarkers Twist...")
	
    def get_tags(self, msg):
        # Initialize the COG as a PoseStamped message
        p = PoseStamped()

        #t = Twist()
        # Get the number of markers
        n = len(msg.markers)

        # If no markers detected, just return
        if n == 0:
            m_flag = 0
            lx = 0 #t.linear.x = 0
            ly = 0 #t.linear.y = 0
            lz = 0 #t.linear.z = 0
            az = 0 #t.angular.z= 0
            #self.tag_pub.publish(t)
            #rospy.loginfo("1. m_flag = %d"%(m_flag))
            return
	    
        else:
            m_flag = 1
            #rospy.loginfo("2. m_flag = %d"%(m_flag))

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
                lx = 0 # t.linear.x = 0
                ly = 0 # t.linear.y = 0
                lz = 0 # t.linear.z = 0
                az = 0 # t.angular.z= 0
                
            else:
                if p.pose.position.x < -0.8:
                   rospy.loginfo("Go left")
                   lx = 0 #t.linear.x = 0
                   ly = 0 #t.linear.y = 0
                   lz = 0 #t.linear.z = 0
                   az = 0.3 #t.angular.z= 0.3
                   
                elif p.pose.position.x > 1.2:
                   rospy.loginfo("Go Right")
                   lx = 0 #t.linear.x = 0
                   ly = 0 #t.linear.y = 0
                   lz = 0 #t.linear.z = 0
                   az = -0.3 #t.angular.z= -0.3
                   
                elif p.pose.position.x < 1.2 and p.pose.position.x > -0.8:
                   rospy.loginfo("1.Stop")
                   lx = 0 #t.linear.x = 0
                   ly = 0 #t.linear.y = 0
                   lz = 0 #t.linear.z = 0
                   az = 0 #t.angular.z= 0
                
                elif p.pose.position.z > 1.5:
                   rospy.loginfo("Go Forward")
                   lx = 0.3 #t.linear.x = 0.3
                   ly = 0 #t.linear.y = 0
                   lz = 0 #t.linear.z = 0
                   az = 0 #t.angular.z= 0
                
                elif p.pose.position.z < 0.5:
                   rospy.loginfo("Go Back")
                   lx = -0.3 #t.linear.x = -0.3
                   ly = 0 #t.linear.y = 0
                   lz = 0 #t.linear.z = 0
                   az = 0 #t.angular.z= 0
                   
                elif p.pose.position.z < 1.5 and p.pose.position > 0.5:
                   rospy.loginfo("Stop")
                   lx = 0 #t.linear.x = 0
                   ly = 0 #t.linear.x = 0
                   lz = 0 #t.linear.z = 0
                   t.angular.z= 0
                
            #return t.linear.x, t.linear.y, t.linear.z, t.angular.z
			
			# Publish the COG
			# self.tag_pub.publish(p)
			# self.tag_pub.publish(t)

class GetChar:                          # class for get 1 byte from standard input( key board )
	def __init__(self):
		# Save the terminal settings
		self.fd = sys.stdin.fileno()
		self.new_term = termios.tcgetattr(self.fd)
		self.old_term = termios.tcgetattr(self.fd)

		# New terminal setting unbuffered
		self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
		termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

		# Support normal-terminal reset at exit
		atexit.register(self.set_normal_term)
	
	
	def set_normal_term(self):
		termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

	def getch(self):	    # get 1 byte from stdin
		""" Returns a keyboard character after getch() has been called """
		return sys.stdin.read(1)

	def chk_stdin(self):	# check keyboard input
		""" Returns True if keyboard character was hit, False otherwise. """
		dr, dw, de = select([sys.stdin], [], [], 0)
		return dr
 
def get_currunt_speed(lin_speed,ang_speed):
    return "currently:\tlinear speed: %s\tangular speed: %s " % (lin_speed, ang_speed)


if __name__=="__main__":
    # settings = termios.tcgetattr(sys.stdin)
    
    pub0 = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
    pub1 = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
    pub2 = rospy.Publisher('bebop/land',    Empty, queue_size = 1)
    pub3 = rospy.Publisher('bebop/reset',   Empty, queue_size = 1)
    pub4 = rospy.Publisher('bebop/snapshot',Empty, queue_size = 1)
    pub5 = rospy.Publisher('bebop/record',  Bool,  queue_size = 1)
    
    empty_msg   = Empty()
    kb          = GetChar()    
    rospy.init_node('ctrl_track')
 
    lin_speed   = rospy.get_param('~speed',  0.5)
    ang_speed   = rospy.get_param('~turn' ,  1.0)
    
    lin_offset  =  0.1;    ang_offset  =  math.radians(15)
    
    max_lin_spd =  1.5;    min_lin_spd = -1.5
    
    max_ang_spd =  math.radians(45)   #  45 degree 
    min_ang_spd = -math.radians(45)   # -45 degree    

    lin_x = 0;    lin_y = 0;    lin_z = 0;    ang_z = 0;    count = 0
 
    try:
        m = DetectMarker()
        print(msg)
        print(get_currunt_speed(lin_speed,ang_speed))
        
        key = ''
        
        while(key != 'Q'):
        
            key = kb.getch()
            
            if key in action.keys():    # set linear x,y,z or angular z
                lin_x  = action[key][0]
                lin_y  = action[key][1]
                lin_z  = action[key][2]
                ang_z  = action[key][3]
            
            elif key == '?':            # display help
                print(msg)
                print(get_currunt_speed(lin_speed,ang_speed))
                
            elif key == '1':            # take off
                pub1.publish(empty_msg)
                
            elif key == '2':            # land
                pub2.publish(empty_msg)
                
            elif key == '3':            # emergency stop( just drop )
                pub3.publish(empty_msg)
               
            elif key == '4':            # snapshot( take a picture )
                pub4.publish(empty_msg)
                
            elif key == '5':            # Tracking(object tracking)
                pass
                #lin_x, lin_y, lin_z, ang_z = m.get_tags()
                
            elif key == 'o':            # start recording
                pub5.publish(True)
                
            elif key == 'p':            # stop recording
                pub5.publish(False)
            
            elif key == '=':            # linear & angular speed up
            
                if( lin_speed <= max_lin_spd - lin_offset ):
                    lin_speed = lin_speed + lin_offset
                else:
                    lin_speed = max_lin_spd
                    
                if( ang_speed <= max_ang_spd - ang_offset ):
                    ang_speed = ang_speed + ang_offset
                else:
                    ang_speed = max_ang_spd
            
            elif key == '-':            # linear & angular speed down
            
                if( lin_speed <= min_lin_spd + lin_offset ):
                    lin_speed = lin_speed + lin_offset
                else:
                    lin_speed = min_lin_spd
                    
                if( ang_speed >= min_ang_spd + ang_offset ):
                    ang_speed = ang_speed - ang_offset
                else:
                    ang_speed = min_ang_spd
                
            else:                       # just hover
                lin_x = 0;  lin_y = 0;  lin_z = 0;  ang_z = 0

            twist = Twist()
            
            twist.linear.x  = lin_speed * lin_x
            twist.linear.y  = lin_speed * lin_y
            twist.linear.z  = lin_speed * lin_z
            
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = ang_speed * ang_z
            
            pub0.publish(twist)
        
        pub2.publish(empty_msg)
        print("bebop have landed!")

    except KeyboardInterrupt:
        pub2.publish(empty_msg)
        print("bebop have landed!")
        
