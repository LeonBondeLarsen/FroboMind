import rospy
import smach
import smach_ros
import actionlib
from rsd_smach.states import get_next_point
from position_action_server import *
from position_action_server.msg import *

def build(parent, wpt, reverse):
    behaviour = smach.StateMachine(outcomes=['success','preempted','aborted'])
    # Next go to point
    behaviour.userdata.next_x = 0.0
    behaviour.userdata.next_y = 0.0
    with behaviour :
        smach.StateMachine.add(parent+'/GET_NEXT', get_next_point.getNextPosition(wpt,reverse), 
                               transitions={'succeeded':parent+'/GO_TO_POINT'})
        smach.StateMachine.add(parent+'/GO_TO_POINT', 
                               smach_ros.SimpleActionState('/fmExecutors/position_planner',positionAction, goal_slots=['x','y','reverse']),
                               transitions={'succeeded':parent+'/GET_NEXT','preempted':'preempted','aborted':'aborted'},
                               remapping={'x':'next_x','y':'next_y','reverse':'reverse'})        

    return behaviour
