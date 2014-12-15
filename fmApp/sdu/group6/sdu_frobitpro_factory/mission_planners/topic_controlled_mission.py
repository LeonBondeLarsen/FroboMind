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
from rsd_smach.navigatestates import navigate_states
from rsd_smach.states import task_controller, server_control
from rsd_smach.behaviours import tipper_controller
from msgs.msg import StringStamped
    
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
                            'TIP',
                            'NAVIGATE_DISPENSER',
                            'NAVIGATE_IN_BOX',
                            'NAVIGATE_LINE',
                            'NAVIGATE_STATION_1',
                            'NAVIGATE_STATION_2',
                            'NAVIGATE_STATION_3',
                            'NAVIGATE_RAMP_IN',
                            'NAVIGATE_RAMP_OUT',
                            'NAVIGATE_FLOOR_IN',
                            'NAVIGATE_FLOOR_OUT',
                            'NAVIGATE_LOAD_ON_1',
                            'NAVIGATE_LOAD_OFF_1',
                            'NAVIGATE_LOAD_ON_2',
                            'NAVIGATE_LOAD_OFF_2',
                            'NAVIGATE_LOAD_ON_3',
                            'NAVIGATE_LOAD_OFF_3'
                          ]
        self.task_transitions = {
                            'MANUAL':'MANUAL',
                            'WAIT':'WAIT',
           #                 'ABORT':'ABORT',
                            'TIP':'TIP',
                            'NAVIGATE_DISPENSER':'NAVIGATE_DISPENSER',
                            'NAVIGATE_IN_BOX':'NAVIGATE_IN_BOX',
                            'NAVIGATE_LINE':'NAVIGATE_LINE',
                            'NAVIGATE_STATION_1':'NAVIGATE_STATION_1',
                            'NAVIGATE_STATION_2':'NAVIGATE_STATION_2',
                            'NAVIGATE_STATION_3':'NAVIGATE_STATION_3',
                            'NAVIGATE_RAMP_IN':'NAVIGATE_RAMP_IN',
                            'NAVIGATE_RAMP_OUT':'NAVIGATE_RAMP_OUT',
                            'NAVIGATE_FLOOR_IN':'NAVIGATE_FLOOR_IN',
                            'NAVIGATE_FLOOR_OUT':'NAVIGATE_FLOOR_OUT',
                            'NAVIGATE_LOAD_ON_1':'NAVIGATE_LOAD_ON_1',
                            'NAVIGATE_LOAD_OFF_1':'NAVIGATE_LOAD_OFF_1',
                            'NAVIGATE_LOAD_ON_2':'NAVIGATE_LOAD_ON_2',
                            'NAVIGATE_LOAD_OFF_2':'NAVIGATE_LOAD_OFF_2',
                            'NAVIGATE_LOAD_ON_3':'NAVIGATE_LOAD_ON_3',
                            'NAVIGATE_LOAD_OFF_3':'NAVIGATE_LOAD_OFF_3'
                            }
        
        self.dispenser_waypoints = [[3,0],[3,-3],[0,-3],[0,0]]
        self.line_waypoints = [[0,0],[0,-3],[3,-3],[3,0]]
        
        self.feedback_timer = rospy.Timer(rospy.Duration(2.0), self.onTimer)
        self.feedback_publisher = rospy.Publisher('/fmInformation/state', StringStamped, queue_size=10)
        self.feedback_message = StringStamped()
          
    def build(self):
                    
        manual = smach.Concurrence(  outcomes = self.task_list, default_outcome = 'MANUAL',
                                         outcome_map = {'MANUAL':{'MANUAL':'MANUAL'},
                                                        'WAIT':{'MANUAL':'WAIT'},
                                                        'TIP':{'MANUAL':'TIP'},
                                                        'NAVIGATE_DISPENSER':{'MANUAL':'NAVIGATE_DISPENSER'},
                                                        'NAVIGATE_LINE':{'MANUAL':'NAVIGATE_LINE'},
                                                        'NAVIGATE_IN_BOX':{'MANUAL':'NAVIGATE_IN_BOX'},
                                                        'NAVIGATE_STATION_1':{'MANUAL':'NAVIGATE_STATION_1'},
                                                        'NAVIGATE_STATION_2':{'MANUAL':'NAVIGATE_STATION_2'},
                                                        'NAVIGATE_STATION_3':{'MANUAL':'NAVIGATE_STATION_3'},
                                                        'NAVIGATE_RAMP_IN':{'MANUAL':'NAVIGATE_RAMP_IN'},
                                                        'NAVIGATE_RAMP_OUT':{'MANUAL':'NAVIGATE_RAMP_OUT'},
                                                        'NAVIGATE_FLOOR_IN':{'MANUAL':'NAVIGATE_FLOOR_IN'},
                                                        'NAVIGATE_FLOOR_OUT':{'MANUAL':'NAVIGATE_FLOOR_OUT'},
                                                        'NAVIGATE_LOAD_ON_1':{'MANUAL':'NAVIGATE_LOAD_ON_1'},
                                                        'NAVIGATE_LOAD_ON_2':{'MANUAL':'NAVIGATE_LOAD_ON_2'},
                                                        'NAVIGATE_LOAD_ON_3':{'MANUAL':'NAVIGATE_LOAD_ON_3'},
                                                        'NAVIGATE_LOAD_OFF_1':{'MANUAL':'NAVIGATE_LOAD_OFF_1'},
                                                        'NAVIGATE_LOAD_OFF_2':{'MANUAL':'NAVIGATE_LOAD_OFF_2'},
                                                        'NAVIGATE_LOAD_OFF_3':{'MANUAL':'NAVIGATE_LOAD_OFF_3'}
                                                        },
                                         child_termination_cb = onPreempt)
        with manual:
            smach.Concurrence.add('MANUAL', task_controller.TaskController('MANUAL',self.task_list))
            
        ##################
                    
        wait = smach.Concurrence(  outcomes = self.task_list, default_outcome = 'WAIT',
                                         outcome_map = {'MANUAL':{'WAIT':'MANUAL'},
                                                        'WAIT':{'WAIT':'WAIT'},
                                                        'TIP':{'WAIT':'TIP'},
                                                        'NAVIGATE_DISPENSER':{'WAIT':'NAVIGATE_DISPENSER'},
                                                        'NAVIGATE_LINE':{'WAIT':'NAVIGATE_LINE'},
                                                        'NAVIGATE_IN_BOX':{'WAIT':'NAVIGATE_IN_BOX'},
                                                        'NAVIGATE_STATION_1':{'WAIT':'NAVIGATE_STATION_1'},
                                                        'NAVIGATE_STATION_2':{'WAIT':'NAVIGATE_STATION_2'},
                                                        'NAVIGATE_STATION_3':{'WAIT':'NAVIGATE_STATION_3'},
                                                        'NAVIGATE_RAMP_IN':{'WAIT':'NAVIGATE_RAMP_IN'},
                                                        'NAVIGATE_RAMP_OUT':{'WAIT':'NAVIGATE_RAMP_OUT'},
                                                        'NAVIGATE_FLOOR_IN':{'WAIT':'NAVIGATE_FLOOR_IN'},
                                                        'NAVIGATE_FLOOR_OUT':{'WAIT':'NAVIGATE_FLOOR_OUT'},
                                                        'NAVIGATE_LOAD_ON_1':{'WAIT':'NAVIGATE_LOAD_ON_1'},
                                                        'NAVIGATE_LOAD_ON_2':{'WAIT':'NAVIGATE_LOAD_ON_2'},
                                                        'NAVIGATE_LOAD_ON_3':{'WAIT':'NAVIGATE_LOAD_ON_3'},
                                                        'NAVIGATE_LOAD_OFF_1':{'WAIT':'NAVIGATE_LOAD_OFF_1'},
                                                        'NAVIGATE_LOAD_OFF_2':{'WAIT':'NAVIGATE_LOAD_OFF_2'},
                                                        'NAVIGATE_LOAD_OFF_3':{'WAIT':'NAVIGATE_LOAD_OFF_3'}
                                                        },
                                         child_termination_cb = onPreempt)
        with wait:
            smach.Concurrence.add('WAIT', task_controller.TaskController('WAIT',self.task_list))
            
        ##################
                    
        tip = smach.Concurrence(  outcomes = self.task_list, default_outcome = 'WAIT',
                                         outcome_map = {'MANUAL':{'CONTROL':'MANUAL'},
                                                        'WAIT':{'CONTROL':'WAIT'},
                                                        'WAIT':{'TIP':'WAIT'},
                                                        'TIP':{'CONTROL':'TIP'},
                                                        'NAVIGATE_DISPENSER':{'CONTROL':'NAVIGATE_DISPENSER'},
                                                        'NAVIGATE_LINE':{'CONTROL':'NAVIGATE_LINE'},
                                                        'NAVIGATE_IN_BOX':{'CONTROL':'NAVIGATE_IN_BOX'},
                                                        'NAVIGATE_STATION_1':{'CONTROL':'NAVIGATE_STATION_1'},
                                                        'NAVIGATE_STATION_2':{'CONTROL':'NAVIGATE_STATION_2'},
                                                        'NAVIGATE_STATION_3':{'CONTROL':'NAVIGATE_STATION_3'},
                                                        'NAVIGATE_RAMP_IN':{'CONTROL':'NAVIGATE_RAMP_IN'},
                                                        'NAVIGATE_RAMP_OUT':{'CONTROL':'NAVIGATE_RAMP_OUT'},
                                                        'NAVIGATE_FLOOR_IN':{'CONTROL':'NAVIGATE_FLOOR_IN'},
                                                        'NAVIGATE_FLOOR_OUT':{'CONTROL':'NAVIGATE_FLOOR_OUT'},
                                                        'NAVIGATE_LOAD_ON_1':{'CONTROL':'NAVIGATE_LOAD_ON_1'},
                                                        'NAVIGATE_LOAD_ON_2':{'CONTROL':'NAVIGATE_LOAD_ON_2'},
                                                        'NAVIGATE_LOAD_ON_3':{'CONTROL':'NAVIGATE_LOAD_ON_3'},
                                                        'NAVIGATE_LOAD_OFF_1':{'CONTROL':'NAVIGATE_LOAD_OFF_1'},
                                                        'NAVIGATE_LOAD_OFF_2':{'CONTROL':'NAVIGATE_LOAD_OFF_2'},
                                                        'NAVIGATE_LOAD_OFF_3':{'CONTROL':'NAVIGATE_LOAD_OFF_3'}
                                                        },
                                         child_termination_cb = onPreempt)
        with tip:
            smach.Concurrence.add('TIP', tipper_controller.build())
            smach.Concurrence.add('CONTROL', task_controller.TaskController('TIP',self.task_list))
            
        ##################
        
        self.mission_control = smach.StateMachine(outcomes= ['ended'])      
              
        with self.mission_control:
            smach.StateMachine.add('MANUAL', manual, transitions=self.task_transitions)
            smach.StateMachine.add('WAIT', wait, transitions=self.task_transitions)
            smach.StateMachine.add('TIP', tip, transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_DISPENSER', navigate_states.build_dispenser(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_IN_BOX', navigate_states.build_in_box(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_LINE', navigate_states.build_line(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_STATION_1', navigate_states.build_station_1(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_STATION_2', navigate_states.build_station_2(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_STATION_3', navigate_states.build_station_3(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_RAMP_IN', navigate_states.build_ramp_in(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_RAMP_OUT', navigate_states.build_ramp_out(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_FLOOR_IN', navigate_states.build_floor_in(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_FLOOR_OUT', navigate_states.build_floor_out(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_LOAD_ON_1', navigate_states.build_load_on_1(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_LOAD_ON_2', navigate_states.build_load_on_2(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_LOAD_ON_3', navigate_states.build_load_on_3(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_LOAD_OFF_1', navigate_states.build_load_off_1(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_LOAD_OFF_2', navigate_states.build_load_off_2(self.task_list, onPreempt), transitions=self.task_transitions)
            smach.StateMachine.add('NAVIGATE_LOAD_OFF_3', navigate_states.build_load_off_3(self.task_list, onPreempt), transitions=self.task_transitions)

        self.server_control = smach.Concurrence(outcomes= ['ended'], default_outcome = 'ended', outcome_map = {'ended':{'MISSION_CONTROL':'ended'}})
        
        with self.server_control:
            smach.Concurrence.add('MISSION_CONTROL', self.mission_control)
            smach.Concurrence.add('SERVER_COMMUNICATION', server_control.ServerControl(self.mission_control))
        
        return self.server_control
                       
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
