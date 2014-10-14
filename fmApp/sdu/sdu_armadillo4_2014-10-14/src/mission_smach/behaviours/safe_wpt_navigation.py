import rospy
import smach
import smach_ros
import actionlib
from mission_smach.behaviours import follow_route
from mission_smach.behaviours import clear_proximity
from position_action_server import *
from position_action_server.msg import *
from std_msgs.msg import Float64

def build():
        # Build navigation task
        navigation_sm = smach.Concurrence ( outcomes = ['proximityAlert','preempted'], 
                                            default_outcome = 'preempted',
                                            outcome_map = {'preempted':{'follow_route':'preempted'}, 'proximityAlert':{'proximity_monitor':'invalid'}},
                                            output_keys=['next_x','next_y'],
                                            child_termination_cb = onPreempt)

        with navigation_sm:
            smach.Concurrence.add('follow_route', follow_route.build())
            smach.Concurrence.add('proximity_monitor', smach_ros.MonitorState("/fmKnowledge/proximity", Float64, proximity_monitor_cb))

        # Build the safety task        
        behavior = smach.StateMachine(outcomes=['success', 'preempted', 'aborted'])
        with behavior:
            smach.StateMachine.add('NAVIGATION', navigation_sm, transitions={'proximityAlert':'CLEAR_PROXIMITY'})
            smach.StateMachine.add('CLEAR_PROXIMITY', clear_proximity.build() , transitions={'proximityCleared':'NAVIGATION'})
        
        return behavior
    
def onPreempt(outcome_map):
    """
        Preempts all other states on child termination. 
        TODO: Find a way to avoid this being a global function...
    """
    return True

def proximity_monitor_cb(userdata, msg):
    if (msg.data):
        return True
    else:
        return False