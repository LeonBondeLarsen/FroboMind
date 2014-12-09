import rospy
import smach
import smach_ros
from std_msgs.msg import String

class getNextPosition(smach.State):
    """
        Temporary implementation. State giving the next position goal based on hard-coded list
    """
    def __init__(self, wptlists, reverse):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['next_x','next_y','reverse'])
        self.ptr = 0
        self.reverse = reverse
        assert len(wptlists) > 0
        self.waypoint_lists = wptlists
        self.position_list = wptlists.values()[0]
        self.position = 'UNKNOWN'
        
        self.position_topic = rospy.get_param("~rsd_area_topic", '/fmKnowledge/rsd_area')
        rospy.Subscriber(self.position_topic, String, self.save_position)

    def execute(self, userdata):
        if self.ptr == 0:
            if self.waypoint_lists.has_key(self.position):
                self.position_list = self.waypoint_lists[self.position]
            else:
                # if the position isn't in the list, we currently just choose an arbitrary list of waypoints
                self.position_list = self.waypoint_lists.values()[0]

        userdata.next_x = self.position_list[self.ptr][0]
        userdata.next_y = self.position_list[self.ptr][1]
        userdata.reverse = self.reverse
        rospy.loginfo("go to point: %d , %d" % (self.position_list[self.ptr][0], self.position_list[self.ptr][1]))
        self.ptr = self.ptr + 1
        if self.ptr == len(self.position_list) :
            self.ptr = 0
        return 'succeeded'
        
    def save_position(self, rsd_area):
        self.position = rsd_area
