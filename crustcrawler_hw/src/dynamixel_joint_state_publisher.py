#!/usr/bin/env python

"""
    dynamixel_joint_state_publisher.py - Version 1.0 2010-12-28
    
    Publish the dynamixel_controller joint states on the /joint_states topic
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

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

import roslib; roslib.load_manifest('crustcrawler_hw')
import rospy

from sensor_msgs.msg import JointState as JointStatePR2
from dynamixel_msgs.msg import JointState as JointStateDynamixel

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatePublisher():
    def __init__(self):
        rospy.init_node('dynamixel_joint_state_publisher', anonymous=True)
        
        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)
        rospy.loginfo("publishing rate is: %f ", rate)
        
        # The namespace and joints parameter needs to be set by the servo controller
        # (The namespace is usually null.)
        namespace = rospy.get_namespace()
        self.joints = rospy.get_param(namespace + '/crustcrawler/joints', '')
                                                                
        self.servos = list()
        self.controllers = list()
        self.joint_states = dict({})
        
        for controller in sorted(self.joints):
            self.joint_states[controller] = JointStateMessage(controller, 0.0, 0.0, 0.0)
            self.controllers.append(controller)
            #rospy.loginfo("controller name is (checking) " + str(controller))
                           
        # Start controller state subscribers
        [rospy.Subscriber('/crustcrawler/' + c + '_position_controller/state', JointStateDynamixel, self.controller_state_handler) for c in self.controllers]
        for c in self.controllers:
            if(c == 'left_finger_joint'):
            	rospy.Subscriber('/crustcrawler/joint_left_finger_position_controller/state', JointStateDynamixel, self.controller_state_handler)
            if(c == 'right_finger_joint'):
            	rospy.Subscriber('/crustcrawler/joint_right_finger_position_controller/state', JointStateDynamixel, self.controller_state_handler)
         
                 
        #for c in self.controllers:
            #rospy.loginfo("controller name is " + str(c))

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointStatePR2, queue_size=9)
       
        rospy.loginfo("Starting Dynamixel Joint State Publisher at " + str(rate) + "Hz")
               
        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()
           
    def controller_state_handler(self, msg):
        js = JointStateMessage(msg.name, msg.current_pos, msg.velocity, msg.load)
        self.joint_states[msg.name] = js
       
    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointStatePR2()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
       
        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            msg.velocity.append(joint.velocity)
            msg.effort.append(joint.effort)
           
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'
        self.joint_states_pub.publish(msg)
        
if __name__ == '__main__':
    try:
        s = JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass

