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
import rospy, smach, smach_ros, actionlib, threading
from rsd_smach.behaviours import safe_wpt_navigation
from rsd_smach.states import publish_deadman
from msgs.msg import StringStamped

class TaskController(smach.State):
    """
    """
    def __init__(self, name, task_list):
        smach.State.__init__(self, outcomes=task_list)
        self.name = name
        self.current_task = 'MANUAL'
        self.r = rospy.Rate(10)
        self.subscriber = rospy.Subscriber('/fmDecisionMaking/task', StringStamped, self.onTaskMessage )
        
    def execute(self, userdata):
        while not rospy.is_shutdown():   
             if self.name in self.current_task :
                  
                 if self.preempt_requested():
                     self.service_preempt()
                     return self.current_task
                  
                 try :
                     self.r.sleep()
                 except rospy.ROSInterruptException:
                     return self.current_task
             else :
                 break
        return self.current_task
    
    def onTaskMessage(self,msg):
        self.current_task = msg.data
    
class Mission():
    """    
        Top level user interface node implemented as a concurrence between wiimote interface and behaviour updater
    """
    def __init__(self):
        rospy.init_node('mission_control')
        rospy.loginfo(rospy.get_name() + ": Mission control Initialised")
        
        self.task_list = [
                            'MANUAL',
                            'WAIT',
           #                 'ABORT',
           #                 'TIP',
                            'NAVIGATE_DISPENSER',
           #                 'NAVIGATE_IN_BOX',
                            'NAVIGATE_LINE'
           #                 'NAVIGATE_STATION_1',
           #                 'NAVIGATE_STATION_2',
           #                 'NAVIGATE_STATION_3',
           #                 'NAVIGATE_RAMP_IN',
           #                 'NAVIGATE_RAMP_OUT',
           #                 'NAVIGATE_FLOOR_IN',
           #                 'NAVIGATE_FLOOR_OUT',
           #                 'NAVIGATE_LOAD_ON_1',
           #                 'NAVIGATE_LOAD_OFF_1',
           #                 'NAVIGATE_LOAD_ON_2',
           #                 'NAVIGATE_LOAD_OFF_2',
           #                 'NAVIGATE_LOAD_ON_3',
           #                 'NAVIGATE_LOAD_OFF_3'
                          ]
        self.task_transitions = {
                            'MANUAL':'MANUAL',
                            'WAIT':'WAIT',
           #                 'ABORT':'ABORT',
           #                 'TIP':'TIP',
                            'NAVIGATE_DISPENSER':'NAVIGATE_DISPENSER',
           #                 'NAVIGATE_IN_BOX':'NAVIGATE_IN_BOX',
                            'NAVIGATE_LINE':'NAVIGATE_LINE'
           #                 'NAVIGATE_STATION_1':'NAVIGATE_STATION_1',
           #                 'NAVIGATE_STATION_2':'NAVIGATE_STATION_2',
           #                 'NAVIGATE_STATION_3':'NAVIGATE_STATION_3',
           #                 'NAVIGATE_RAMP_IN':'NAVIGATE_RAMP_IN',
           #                 'NAVIGATE_RAMP_OUT':'NAVIGATE_RAMP_OUT',
           #                 'NAVIGATE_FLOOR_IN':'NAVIGATE_FLOOR_IN',
           #                 'NAVIGATE_FLOOR_OUT':'NAVIGATE_FLOOR_OUT',
           #                 'NAVIGATE_LOAD_ON_1':'NAVIGATE_LOAD_ON_1',
           #                 'NAVIGATE_LOAD_OFF_1':'NAVIGATE_LOAD_OFF_1',
           #                 'NAVIGATE_LOAD_ON_2':'NAVIGATE_LOAD_ON_2',
           #                 'NAVIGATE_LOAD_OFF_2':'NAVIGATE_LOAD_OFF_2',
           #                 'NAVIGATE_LOAD_ON_3':'NAVIGATE_LOAD_ON_3',
           #                 'NAVIGATE_LOAD_OFF_3':'NAVIGATE_LOAD_OFF_3'
                            }
        
        self.dispenser_waypoints = [[3,0],[3,-3],[0,-3],[0,0]]
        self.line_waypoints = [[0,0],[0,-3],[3,-3],[3,0]]
        
        self.feedback_timer = rospy.Timer(rospy.Duration(2.0), self.onTimer)
        self.feedback_publisher = rospy.Publisher('/fmInformation/state', StringStamped, queue_size=10)
        self.feedback_message = StringStamped()
          
    def build(self):
        autonomous1 = smach.Concurrence(  outcomes = ['exitAutomode'],
                                         default_outcome = 'exitAutomode',
                                         outcome_map = {'exitAutomode':{'NAVIGATE_DISPENSER/SAFETY':'preempted','NAVIGATE_DISPENSER/DEADMAN':'preempted'}},
                                         child_termination_cb = onPreempt)

        with autonomous1:
            smach.Concurrence.add('NAVIGATE_DISPENSER/SAFETY',  safe_wpt_navigation.SafeWaypointNavigation('NAVIGATE_DISPENSER',self.dispenser_waypoints).safety_sm)
            smach.Concurrence.add('NAVIGATE_DISPENSER/DEADMAN', publish_deadman.publishDeadmanState() )
            
        ##################
        
        autonomous2 = smach.Concurrence(  outcomes = ['exitAutomode'],
                                         default_outcome = 'exitAutomode',
                                         outcome_map = {'exitAutomode':{'NAVIGATE_LINE/SAFETY':'preempted','NAVIGATE_LINE/DEADMAN':'preempted'}},
                                         child_termination_cb = onPreempt)

        with autonomous2:
            smach.Concurrence.add('NAVIGATE_LINE/SAFETY',  safe_wpt_navigation.SafeWaypointNavigation('NAVIGATE_LINE',self.line_waypoints).safety_sm)
            smach.Concurrence.add('NAVIGATE_LINE/DEADMAN', publish_deadman.publishDeadmanState() )
            
        ##################
                    
        manual = smach.Concurrence(  outcomes = self.task_list, default_outcome = 'MANUAL',
                                         outcome_map = {'MANUAL':{'MANUAL':'MANUAL'},
                                                        'WAIT':{'MANUAL':'WAIT'},
                                                        'NAVIGATE_DISPENSER':{'MANUAL':'NAVIGATE_DISPENSER'},
                                                        'NAVIGATE_LINE':{'MANUAL':'NAVIGATE_LINE'}
                                                        },
                                         child_termination_cb = onPreempt)
        with manual:
            smach.Concurrence.add('MANUAL', TaskController('MANUAL',self.task_list))
            
        ##################
                    
        wait = smach.Concurrence(  outcomes = self.task_list, default_outcome = 'WAIT',
                                         outcome_map = {'MANUAL':{'WAIT':'MANUAL'},
                                                        'WAIT':{'WAIT':'WAIT'},
                                                        'NAVIGATE_DISPENSER':{'WAIT':'NAVIGATE_DISPENSER'},
                                                        'NAVIGATE_LINE':{'WAIT':'NAVIGATE_LINE'}
                                                        },
                                         child_termination_cb = onPreempt)
        with wait:
            smach.Concurrence.add('WAIT', TaskController('WAIT',self.task_list))
            
        ##################
                    
        navigate_dispenser = smach.Concurrence(  outcomes = self.task_list, default_outcome = 'WAIT',
                                         outcome_map = {'MANUAL':{'NAVIGATE_DISPENSER/CONTROL':'MANUAL'},
                                                        'WAIT':{'NAVIGATE_DISPENSER/CONTROL':'WAIT'},
                                                        'NAVIGATE_DISPENSER':{'NAVIGATE_DISPENSER/CONTROL':'NAVIGATE_DISPENSER'},
                                                        'NAVIGATE_LINE':{'NAVIGATE_DISPENSER/CONTROL':'NAVIGATE_LINE'}},
                                         child_termination_cb = onPreempt)
        with navigate_dispenser:
            smach.Concurrence.add('NAVIGATE_DISPENSER/CONTROL', TaskController('NAVIGATE_DISPENSER',self.task_list))
            smach.Concurrence.add('NAVIGATE_DISPENSER/TASK', autonomous1)
            
        ##################
                    
        navigate_line = smach.Concurrence(  outcomes = self.task_list, default_outcome = 'WAIT',
                                         outcome_map = {'WAIT':{'NAVIGATE_LINE/CONTROL':'WAIT'},
                                                        'NAVIGATE_DISPENSER':{'NAVIGATE_LINE/CONTROL':'NAVIGATE_DISPENSER'},
                                                        'NAVIGATE_LINE':{'NAVIGATE_LINE/CONTROL':'NAVIGATE_LINE'}},
                                         child_termination_cb = onPreempt)
        with navigate_line:
            smach.Concurrence.add('NAVIGATE_LINE/CONTROL', TaskController('NAVIGATE_LINE',self.task_list))
            smach.Concurrence.add('NAVIGATE_LINE/TASK', autonomous2)            
            
        ##################
        
        self.mission_control = smach.StateMachine(outcomes= [])      
              
        with self.mission_control:
            smach.StateMachine.add('MANUAL', manual, transitions=self.task_transitions)
            smach.StateMachine.add('WAIT', wait, transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_DISPENSER', navigate_dispenser, transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_LINE', navigate_line, transitions=self.task_transitions)

        return self.mission_control
                       
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
        
    def onTimer(self, event):
        self.feedback_message.header.stamp = rospy.Time.now()
        self.feedback_message.data = str( self.mission_control.get_active_states() )
        self.feedback_publisher.publish(self.feedback_message)


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
