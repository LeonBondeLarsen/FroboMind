#!/usr/bin/env python
#/****************************************************************************
# FroboMind demiming.py
# Copyright (c) 2011-2013, author Leon Bonde Larsen <leon@bondelarsen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
import rospy
import smach
import smach_ros
import actionlib
from rsd_smach.states import get_next
from position_action_server import *
from position_action_server.msg import *

def build_follow_route(parent, wpt, reverse):
    behaviour = smach.StateMachine(outcomes=['success','preempted','aborted'])
    # Next go to point
    behaviour.userdata.next_x = 0.0
    behaviour.userdata.next_y = 0.0
    with behaviour :
        smach.StateMachine.add(parent+'/GET_NEXT', get_next.getNextPosition(wpt,reverse), 
                               transitions={'succeeded':parent+'/GO_TO_POINT'})
        smach.StateMachine.add(parent+'/GO_TO_POINT', 
                               smach_ros.SimpleActionState('/fmExecutors/position_planner',positionAction, goal_slots=['x','y','reverse']),
                               transitions={'succeeded':parent+'/GET_NEXT','preempted':'preempted','aborted':'aborted'},
                               remapping={'x':'next_x','y':'next_y','reverse':'reverse'})        

    return behaviour

##############################################################

import rospy
import smach
import smach_ros
import actionlib
from generic_smach.states import wait_state
from position_action_server import *
from position_action_server.msg import *
from std_msgs.msg import Bool

class SafeWaypointNavigation():
    def __init__(self, parent, wpt, reverse):
        self.parent = parent
        
        self.navigation_sm = smach.Concurrence (outcomes = ['proximityAlert','preempted'], 
                                                default_outcome = 'preempted',
                                                outcome_map = {'preempted':{self.parent+'/FOLLOW_ROUTE':'preempted',self.parent+'/PROXIMITY_MONITOR':'preempted'}, 
                                                               'proximityAlert':{self.parent+'/PROXIMITY_MONITOR':'invalid'},
                                                               'preempted':{self.parent+'/PROXIMITY_MONITOR':'valid'}},
                                                child_termination_cb = self.onPreempt)
        with self.navigation_sm:
            smach.Concurrence.add(self.parent+'/FOLLOW_ROUTE', build_follow_route(self.parent, wpt, reverse))
            smach.Concurrence.add(self.parent+'/PROXIMITY_MONITOR', smach_ros.MonitorState("/fmKnowledge/proximity_ok",Bool, self.proximity_monitor_cb))    
            
                    
        self.safety_sm = smach.StateMachine(outcomes=['success', 'preempted', 'aborted'])
        with self.safety_sm:
            smach.StateMachine.add(self.parent+'/NAVIGATION', self.navigation_sm, transitions={'proximityAlert':self.parent+'/WAIT','preempted':'preempted'})
            smach.StateMachine.add(self.parent+'/WAIT', wait_state.WaitState(5), transitions={'succeeded':self.parent+'/CHECK', 'preempted':'preempted'})
            smach.StateMachine.add(self.parent+'/CHECK', smach_ros.MonitorState("/fmKnowledge/proximity_ok",Bool, self.proximity_monitor_cb,1),
                                    transitions={'invalid':self.parent+'/WAIT', 'valid':self.parent+'/NAVIGATION', 'preempted':'preempted'})
    
    def onPreempt(self,outcome_map):
        """
            Preempts all other states on child termination. 
            TODO: Find a way to avoid this being a global function...
        """
        return True

    def proximity_monitor_cb(self,userdata, msg):
        return(msg.data)

################################################

import rospy
import smach
import smach_ros
import actionlib
import threading
from wii_interface import wii_interface 
#from gamepad_interface import gamepad_interface  
#from rsd_smach.behaviours import safe_wpt_navigation
#from rsd_smach.states import gui_states
from generic_smach.states import joy_states
from nav_msgs.msg import Odometry    
from std_msgs.msg import Float64


class Mission():
    """    
        Top level user interface node implemented as a concurrence between wiimote interface and behaviour updater
    """
    def __init__(self):
        rospy.init_node('mission_control')
        rospy.loginfo(rospy.get_name() + ": Mission control Initialised")

        self.hmi = wii_interface.WiiInterface()
        #self.hmi = gamepad_interface.GamepadInterface()

        self.hmi.register_callback_button_A(self.onButtonA)
        
        self.square_waypoints = [[1,0],[1,-1],[0,-1],[0,0]]
          
    def build(self):
        # Build the autonomous state as concurrence between wiimote and measuring behaviour to allow user preemption
        autonomous = smach.Concurrence(  outcomes = ['exitAutomode'],
                                         default_outcome = 'exitAutomode',
                                         outcome_map = {'exitAutomode':{'HMI':'preempted','SAFETY':'preempted'}},
                                         child_termination_cb = onPreempt)

        with autonomous:
            smach.Concurrence.add('HMI', joy_states.interfaceState(self.hmi))
            smach.Concurrence.add('SAFETY',  SafeWaypointNavigation('NAVIGATE_DISPENSER',self.square_waypoints,reverse=True).safety_sm)

        
        # Build the top level mission control from the remote control state and the autonomous state
        mission_control = smach.StateMachine(outcomes=['preempted'])            
        with mission_control:
            smach.StateMachine.add('REMOTE_CONTROL', joy_states.remoteControlState(self.hmi), transitions={'enterAutomode':'AUTO_MODE','preempted':'preempted'})
            smach.StateMachine.add('AUTO_MODE', autonomous, transitions={'exitAutomode':'REMOTE_CONTROL'})
        return mission_control
                       
    def spin(self):    
        self.sm = self.build()   
   
        self.sis = smach_ros.IntrospectionServer('StateMachineView', self.sm, '/SM_ROOT')           
        self.sis.start() 
        self.sm.execute()
        rospy.spin()

    def quit(self):
        sis.stop()        

    def onButtonA(self):
        rospy.loginfo("A pressed")


def onPreempt(outcome_map):
    """
        Preempts all other states on child termination. 
        TODO: Find a way to avoid this being a global function...
    """
    return True
    
if __name__ == '__main__':
    try:
        node = Mission()
        node.spin()
    except rospy.ROSInterruptException:
        node.quit()
