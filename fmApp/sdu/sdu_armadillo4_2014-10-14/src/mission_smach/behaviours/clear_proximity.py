import rospy
import smach
import smach_ros
from action_primitives.msg import *
from position_action_server import *
from position_action_server.msg import *
from std_msgs.msg import Float64

def build():
    clear_proximity_sm = smach.StateMachine(outcomes=['proximityCleared','preempted','aborted'], input_keys=['next_x', 'next_y'])

    with clear_proximity_sm:
         smach.StateMachine.add('proximityAlert', smach_ros.MonitorState("/fmKnowledge/proximity", Float64, proximity_monitor_cb, 1), transitions={'valid':'proximityCleared', 'invalid':'proximityAlert', 'preempted':'preempted'})
        
    return clear_proximity_sm

def proximity_monitor_cb(userdata, msg):
    if (msg.data):
        return True
    else:
        return False
