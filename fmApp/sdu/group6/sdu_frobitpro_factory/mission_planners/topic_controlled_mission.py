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
    def __init__(self, name):
        smach.State.__init__(self, outcomes=['IDLE','JOB1','JOB2'])
        self.name = name
        self.current_task = 'IDLE'
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
        self.task_list = ['IDLE', 'JOB1', 'JOB2']
        self.task_transitions = {'IDLE':'IDLE', 'JOB1':'JOB1', 'JOB2':'JOB2'}
        
        self.job1_waypoints = [[3,0],[3,-3],[0,-3],[0,0]]
        self.job2_waypoints = [[0,0],[0,-3],[3,-3],[3,0]]
          
    def build(self):
        autonomous1 = smach.Concurrence(  outcomes = ['exitAutomode'],
                                         default_outcome = 'exitAutomode',
                                         outcome_map = {'exitAutomode':{'JOB1/SAFETY':'preempted','JOB1/DEADMAN':'preempted'}},
                                         child_termination_cb = onPreempt)

        with autonomous1:
            smach.Concurrence.add('JOB1/SAFETY',  safe_wpt_navigation.SafeWaypointNavigation('JOB1',self.job1_waypoints).safety_sm)
            smach.Concurrence.add('JOB1/DEADMAN', publish_deadman.publishDeadmanState() )
            
        ##################
        
        autonomous2 = smach.Concurrence(  outcomes = ['exitAutomode'],
                                         default_outcome = 'exitAutomode',
                                         outcome_map = {'exitAutomode':{'JOB2/SAFETY':'preempted','JOB2/DEADMAN':'preempted'}},
                                         child_termination_cb = onPreempt)

        with autonomous2:
            smach.Concurrence.add('JOB2/SAFETY',  safe_wpt_navigation.SafeWaypointNavigation('JOB2',self.job2_waypoints).safety_sm)
            smach.Concurrence.add('JOB2/DEADMAN', publish_deadman.publishDeadmanState() )
            
        ##################
                    
        idle = smach.Concurrence(  outcomes = self.task_list, default_outcome = 'IDLE',
                                         outcome_map = {'IDLE':{'IDLE_CONTROL':'IDLE'},'JOB1':{'IDLE_CONTROL':'JOB1'},'JOB2':{'IDLE_CONTROL':'JOB2'}},
                                         child_termination_cb = onPreempt)
        with idle:
            smach.Concurrence.add('IDLE_CONTROL', TaskController('IDLE'))
            
        ##################
                    
        task1 = smach.Concurrence(  outcomes = self.task_list, default_outcome = 'IDLE',
                                         outcome_map = {'IDLE':{'JOB1/CONTROL':'IDLE'},'JOB1':{'JOB1/CONTROL':'JOB1'},'JOB2':{'JOB1/CONTROL':'JOB2'}},
                                         child_termination_cb = onPreempt)
        with task1:
            smach.Concurrence.add('JOB1/CONTROL', TaskController('JOB1'))
            smach.Concurrence.add('JOB1/TASK', autonomous1)
            
        ##################
                    
        task2 = smach.Concurrence(  outcomes = self.task_list, default_outcome = 'IDLE',
                                         outcome_map = {'IDLE':{'JOB2/CONTROL':'IDLE'},'JOB1':{'JOB2/CONTROL':'JOB1'},'JOB2':{'JOB2/CONTROL':'JOB2'}},
                                         child_termination_cb = onPreempt)
        with task2:
            smach.Concurrence.add('JOB2/CONTROL', TaskController('JOB2'))
            smach.Concurrence.add('JOB2/TASK', autonomous2)            
            
        ##################
        
        mission_control = smach.StateMachine(outcomes= [])      
              
        with mission_control:
            smach.StateMachine.add('IDLE', idle, transitions=self.task_transitions)
            smach.StateMachine.add('JOB1', task1, transitions=self.task_transitions)
            smach.StateMachine.add('JOB2', task2, transitions=self.task_transitions)

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
