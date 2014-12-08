__author__ = 'kristjan'

import smach
from rsd_smach.behaviours import safe_wpt_navigation
from rsd_smach.states import publish_deadman
from rsd_smach.states import task_controller


def build(task_list, onPreempt, state_name, waypoint_lists):
    autonomous = smach.Concurrence(  outcomes = ['exitAutomode'],
                                    default_outcome = 'exitAutomode',
                                    outcome_map = {'exitAutomode':{state_name + '/SAFETY':'preempted',state_name + '/DEADMAN':'preempted'}},
                                    child_termination_cb = onPreempt)

    with autonomous:
        smach.Concurrence.add(state_name + '/SAFETY',  safe_wpt_navigation.SafeWaypointNavigation(state_name, waypoint_lists, reverse=False).safety_sm)
        smach.Concurrence.add(state_name + '/DEADMAN', publish_deadman.publishDeadmanState() )
    ##################

    navigate_state = smach.Concurrence(  outcomes = task_list, default_outcome = 'WAIT',
                                        outcome_map = {'MANUAL':{state_name + '/CONTROL':'MANUAL'},
                                                    'WAIT':{state_name + '/CONTROL':'WAIT'}},
                                        child_termination_cb = onPreempt)
    with navigate_state:
        smach.Concurrence.add(state_name + '/CONTROL', task_controller.TaskController(state_name, task_list))
        smach.Concurrence.add(state_name + '/TASK', autonomous)

    return navigate_state
