#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped



#############################################################################
class Velocity:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("V_calculation")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
      
        self.R = rospy.get_param('~R',4)
        self.rate = rospy.get_param('~rate',10.0)
        self.vr = 0
        self.vl = 0
        
        rospy.Subscriber("speed", Vector3Stamped, self.vrCallback)
        rospy.Subscriber("speed", Vector3Stamped, self.vlCallback)
        self.vrpub = rospy.Publisher("vr", Float64, queue_size=10)
        self.vlpub = rospy.Publisher("vl", Float64, queue_size=10)
        self.vpub = rospy.Publisher("v", Float64, queue_size=10)
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()   
       
     
    #############################################################################
    def update(self):
    #############################################################################
        #velocity = R/2*(vr + vl)
        vel = self.R/2*(self.vr + self.vl)
        rospy.loginfo(vel)
        self.vpub.publish(vel)
       
            
            


    #############################################################################
    def vrCallback(self, msg):
    #############################################################################
        self.vr = msg.vector.x
        self.vrpub.publish(self.vr)
        
    #############################################################################
    def vlCallback(self, msg):
    #############################################################################
        self.vl = msg.vector.y
        self.vlpub.publish(self.vl)
        

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        velocity = Velocity()
        velocity.spin()
    except rospy.ROSInterruptException:
        pass
