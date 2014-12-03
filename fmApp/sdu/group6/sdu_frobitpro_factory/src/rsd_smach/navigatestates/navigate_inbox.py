__author__ = 'kristjan'

import rospy
import smach
from rsd_smach.behaviours import safe_wpt_navigation
from rsd_smach.states import publish_deadman
from msgs.msg import StringStamped

def build(task_list, onPreempt):
    waypoints = [[2,0],[2,-2],[0,-2],[0,0]]

    autonomous1 = smach.Concurrence(  outcomes = ['exitAutomode'],
                                    default_outcome = 'exitAutomode',
                                    outcome_map = {'exitAutomode':{'NAVIGATE_IN_BOX/SAFETY':'preempted','NAVIGATE_IN_BOX/DEADMAN':'preempted'}},
                                    child_termination_cb = onPreempt)

    with autonomous1:
        smach.Concurrence.add('NAVIGATE_IN_BOX/SAFETY',  safe_wpt_navigation.SafeWaypointNavigation('NAVIGATE_IN_BOX', waypoints).safety_sm)
        smach.Concurrence.add('NAVIGATE_IN_BOX/DEADMAN', publish_deadman.publishDeadmanState() )
    ##################

    navigate_inbox = smach.Concurrence(  outcomes = task_list, default_outcome = 'WAIT',
                                        outcome_map = {'MANUAL':{'NAVIGATE_IN_BOX/CONTROL':'MANUAL'},
                                                    'WAIT':{'NAVIGATE_IN_BOX/CONTROL':'WAIT'},
                                                    'NAVIGATE_DISPENSER':{'NAVIGATE_IN_BOX/CONTROL':'NAVIGATE_DISPENSER'},
                                                    'NAVIGATE_LINE':{'NAVIGATE_IN_BOX/CONTROL':'NAVIGATE_LINE'}},
                                        child_termination_cb = onPreempt)
    with navigate_inbox:
        smach.Concurrence.add('NAVIGATE_IN_BOX/CONTROL', TaskController('NAVIGATE_IN_BOX', task_list))
        smach.Concurrence.add('NAVIGATE_IN_BOX/TASK', autonomous1)

    return navigate_inbox

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