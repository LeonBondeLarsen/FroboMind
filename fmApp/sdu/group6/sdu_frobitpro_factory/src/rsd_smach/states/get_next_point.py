import rospy
import smach
import smach_ros

class getNextPosition(smach.State):
    """
        Temporary implementation. State giving the next position goal based on hard-coded list
    """
    def __init__(self, wpt):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['next_x','next_y'])
        self.ptr = 0
        self.position_list = wpt

    def execute(self, userdata):
        userdata.next_x = self.position_list[self.ptr][0]
        userdata.next_y = self.position_list[self.ptr][1]
        rospy.loginfo("go to point: %d , %d" % (self.position_list[self.ptr][0], self.position_list[self.ptr][1]))
        self.ptr = self.ptr + 1
        if self.ptr == len(self.position_list) :
            self.ptr = 0
        return 'succeeded'       

