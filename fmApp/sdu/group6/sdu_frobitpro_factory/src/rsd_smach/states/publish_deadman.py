import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
    
class publishDeadmanState(smach.State):
    """
        Implements Graphical User Interface
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted'])
        self.r = rospy.Rate(10)
        
        self.deadman_topic = rospy.get_param("~deadman_topic",'fmSignals/deadman')
        self.deadman_pub = rospy.Publisher(self.deadman_topic, Bool, queue_size=10)
        
    def execute(self, userdata):
        while not rospy.is_shutdown():  
            
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            else:       
                self.deadman_pub.publish(True)
                
            try :
                self.r.sleep()
            except rospy.ROSInterruptException:
                return 'preempted'
        return 'preempted'
    

