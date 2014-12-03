__author__ = 'kristjan'

import smach
from rsd_smach.behaviours import safe_wpt_navigation
from rsd_smach.states import publish_deadman
from rsd_smach.states import task_controller


def build(task_list, onPreempt):
    waypoints = [[3,0],[3,-3],[0,-3],[0,0]]

    autonomous = smach.Concurrence(  outcomes = ['exitAutomode'],
                                    default_outcome = 'exitAutomode',
                                    outcome_map = {'exitAutomode':{'NAVIGATE_LINE/SAFETY':'preempted','NAVIGATE_LINE/DEADMAN':'preempted'}},
                                    child_termination_cb = onPreempt)

    with autonomous:
        smach.Concurrence.add('NAVIGATE_LINE/SAFETY',  safe_wpt_navigation.SafeWaypointNavigation('NAVIGATE_LINE', waypoints).safety_sm)
        smach.Concurrence.add('NAVIGATE_LINE/DEADMAN', publish_deadman.publishDeadmanState() )
    ##################

    navigate_inbox = smach.Concurrence(  outcomes = task_list, default_outcome = 'WAIT',
                                        outcome_map = {'MANUAL':{'NAVIGATE_LINE/CONTROL':'MANUAL'},
                                                    'WAIT':{'NAVIGATE_LINE/CONTROL':'WAIT'},
                                                    'NAVIGATE_DISPENSER':{'NAVIGATE_LINE/CONTROL':'NAVIGATE_DISPENSER'},
                                                    'NAVIGATE_LINE':{'NAVIGATE_LINE/CONTROL':'NAVIGATE_LINE'}},
                                        child_termination_cb = onPreempt)
    with navigate_inbox:
        smach.Concurrence.add('NAVIGATE_LINE/CONTROL', task_controller.TaskController('NAVIGATE_LINE', task_list))
        smach.Concurrence.add('NAVIGATE_LINE/TASK', autonomous)

    return navigate_inbox