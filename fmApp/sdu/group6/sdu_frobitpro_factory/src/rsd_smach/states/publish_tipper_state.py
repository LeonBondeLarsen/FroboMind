# rsd_smach/states
import rospy
import smach
import smach_ros
from std_msgs.msg import String

class publishTipperState(smach.State):
	"""
		Implements tipper control
	"""
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted'])
		self.r = rospy.Rate(10)

		self.tipper_topic = rospy.get_param("~tipper_ctrl",'fmSignals/tipper')
		self.tipper_pub = rospy.Publisher(self.tipper_topic, String, queue_size=10)

	def execute(self, userdate):
		if not rospy.is_shutdown():
			self.tipper_pub.publish("TIP_CYCLE")
		return 'preempted'
