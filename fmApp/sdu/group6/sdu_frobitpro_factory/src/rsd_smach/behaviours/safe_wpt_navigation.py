import rospy
import smach
import smach_ros
import actionlib
from generic_smach.states import wait_state
from rsd_smach.behaviours import follow_route
from position_action_server import *
from position_action_server.msg import *
from std_msgs.msg import Bool

class SafeWaypointNavigation():
    def __init__(self, parent, wpt):
        self.parent = parent
        
        self.navigation_sm = smach.Concurrence (outcomes = ['proximityAlert','preempted'], 
                                                default_outcome = 'preempted',
                                                outcome_map = {'preempted':{self.parent+'/FOLLOW_ROUTE':'preempted',self.parent+'/PROXIMITY_MONITOR':'preempted'}, 
                                                               'proximityAlert':{self.parent+'/PROXIMITY_MONITOR':'invalid'},
                                                               'preempted':{self.parent+'/PROXIMITY_MONITOR':'valid'}},
                                                child_termination_cb = self.onPreempt)
        with self.navigation_sm:
            smach.Concurrence.add(self.parent+'/FOLLOW_ROUTE', follow_route.build(self.parent, wpt))
            smach.Concurrence.add(self.parent+'/PROXIMITY_MONITOR', smach_ros.MonitorState("/fmKnowledge/proximity_ok",Bool, self.proximity_monitor_cb))    
            
                    
        self.safety_sm = smach.StateMachine(outcomes=['success', 'preempted', 'aborted'])
        with self.safety_sm:
            smach.StateMachine.add(self.parent+'/NAVIGATION', self.navigation_sm, transitions={'proximityAlert':self.parent+'/WAIT','preempted':'preempted'})
            smach.StateMachine.add(self.parent+'/WAIT', wait_state.WaitState(5), transitions={'succeeded':self.parent+'/CHECK', 'preempted':'preempted'})
            smach.StateMachine.add(self.parent+'/CHECK', smach_ros.MonitorState("/fmKnowledge/proximity_ok",Bool, self.proximity_monitor_cb,1),
                                    transitions={'invalid':self.parent+'/WAIT', 'valid':self.parent+'/NAVIGATION', 'preempted':'preempted'})
    
    def onPreempt(self,outcome_map):
        """
            Preempts all other states on child termination. 
            TODO: Find a way to avoid this being a global function...
        """
        return True

    def proximity_monitor_cb(self,userdata, msg):
        return(msg.data)
    