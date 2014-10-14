import rospy
import smach
import smach_ros

class getNextPosition(smach.State):
    """
        Temporary implementation. State giving the next position goal based on hard-coded list
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'done'], input_keys=['waypoints'], output_keys=['next_x','next_y'])
        self.ptr = 0
        self.idle = True
        self.position_list = []

    def execute(self, userdata):
        if self.idle :
            self.idle = False
            self.position_list = userdata.waypoints
            rospy.loginfo(rospy.get_name() + "New waypoint list: " + str(self.position_list))

        if self.ptr == len(self.position_list) :
            self.ptr = 0
            self.idle = True
            return 'done' 
                    
        userdata.next_x = self.position_list[self.ptr][0]
        userdata.next_y = self.position_list[self.ptr][1]
        rospy.loginfo("go to point: %f , %f" % (self.position_list[self.ptr][0], self.position_list[self.ptr][1]))
        self.ptr = self.ptr + 1

    
        return 'succeeded'       
