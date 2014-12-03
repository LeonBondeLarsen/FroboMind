__author__ = 'kristjan'

import rospy
import smach
from msgs.msg import StringStamped

class TaskController(smach.State):
    """
    """
    def __init__(self, name, task_list):
        smach.State.__init__(self, outcomes=task_list)
        self.name = name
        self.current_task = 'MANUAL'
        self.r = rospy.Rate(10)
        self.subscriber = rospy.Subscriber('/fmDecisionMaking/task', StringStamped, self.onTaskMessage )

    def execute(self, userdata):
        while not rospy.is_shutdown():
             if self.name in self.current_task :

                 if self.preempt_requested():
                     self.service_preempt()
                     return self.current_task

                 try :
                     self.r.sleep()
                 except rospy.ROSInterruptException:
                     return self.current_task
             else :
                 break
        return self.current_task

    def onTaskMessage(self,msg):
        self.current_task = msg.data