__author__ = 'kristjan'

import smach
from rsd_smach.behaviours import safe_wpt_navigation
from rsd_smach.states import publish_deadman
from rsd_smach.states import task_controller

def build(task_list, onPreempt):
    waypoints = [[2,0],[2,-2],[0,-2],[0,0]]

    autonomous = smach.Concurrence(  outcomes = ['exitAutomode'],
                                    default_outcome = 'exitAutomode',
                                    outcome_map = {'exitAutomode':{'NAVIGATE_IN_BOX/SAFETY':'preempted','NAVIGATE_IN_BOX/DEADMAN':'preempted'}},
                                    child_termination_cb = onPreempt)

    with autonomous:
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
        smach.Concurrence.add('NAVIGATE_IN_BOX/CONTROL', task_controller.TaskController('NAVIGATE_IN_BOX', task_list))
        smach.Concurrence.add('NAVIGATE_IN_BOX/TASK', autonomous)

    return navigate_inbox