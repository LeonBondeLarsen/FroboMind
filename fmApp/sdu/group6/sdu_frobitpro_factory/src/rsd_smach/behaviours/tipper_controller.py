import rospy
import smach
import smach_ros
import actionlib
from rsd_smach.states import publish_tipper_state
from generic_smach.states import wait_state
from msgs.msg import StringStamped

def build():
	behaviour = smach.StateMachine(outcomes=['ABORT','WAIT'])

	with behaviour:
		smach.StateMachine.add('TIP/PUBLISH', publish_tipper_state.publishTipperState(), transitions={'preempted':'TIP/WAIT'})
		smach.StateMachine.add('TIP/WAIT', wait_state.WaitState(20), transitions={'preempted':'TIP/CONTROL','succeeded':'TIP/CONTROL','aborted':'ABORT'})
		smach.StateMachine.add('TIP/CONTROL', TaskMessageState(), transitions={'preempted':'WAIT','succeeded':'WAIT'})
	return behaviour


class TaskMessageState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','preempted'])
		self.decision_publisher = rospy.Publisher('/fmDecisionMaking/task', StringStamped, queue_size=10)
	
	def execute(self, user_data):
		try:
			task_msg = StringStamped()
			task_msg.data = 'WAIT'
			self.decision_publisher.publish(task_msg)
			return 'succeeded'
		except:
			return 'preempted'
