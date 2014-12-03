__author__ = 'kristjan'

import smach
from rsd_smach.behaviours import safe_wpt_navigation
from rsd_smach.states import publish_deadman
from rsd_smach.states import task_controller

def build(task_list, onPreempt):
    waypoints = [[3,0],[3,-3],[0,-3],[0,0]]

    autonomous = smach.Concurrence(  outcomes = ['exitAutomode'],
                                    default_outcome = 'exitAutomode',
                                    outcome_map = {'exitAutomode':{'NAVIGATE_DISPENSER/SAFETY':'preempted','NAVIGATE_DISPENSER/DEADMAN':'preempted'}},
                                    child_termination_cb = onPreempt)

    with autonomous:
        smach.Concurrence.add('NAVIGATE_DISPENSER/SAFETY',  safe_wpt_navigation.SafeWaypointNavigation('NAVIGATE_DISPENSER', waypoints).safety_sm)
        smach.Concurrence.add('NAVIGATE_DISPENSER/DEADMAN', publish_deadman.publishDeadmanState() )
    ##################

    navigate_inbox = smach.Concurrence(  outcomes = task_list, default_outcome = 'WAIT',
                                        outcome_map = {'MANUAL':{'NAVIGATE_DISPENSER/CONTROL':'MANUAL'},
                                                    'WAIT':{'NAVIGATE_DISPENSER/CONTROL':'WAIT'},
                                                    'NAVIGATE_DISPENSER':{'NAVIGATE_DISPENSER/CONTROL':'NAVIGATE_DISPENSER'},
                                                    'NAVIGATE_LINE':{'NAVIGATE_DISPENSER/CONTROL':'NAVIGATE_LINE'}},
                                        child_termination_cb = onPreempt)
    with navigate_inbox:
        smach.Concurrence.add('NAVIGATE_DISPENSER/CONTROL', task_controller.TaskController('NAVIGATE_DISPENSER', task_list))
        smach.Concurrence.add('NAVIGATE_DISPENSER/TASK', autonomous)

    return navigate_inbox