#!/usr/bin/env python
__author__ = 'prier'

import roslib
import rospy
import smach
import smach_ros
import time
from rsd_mes_client.msg import mes_mobile_command, mes_mobile_status

command = 0
path = ''

# Initial status of Mobile robot
msg_status = mes_mobile_status()
msg_status.version_id = 1
msg_status.robot_id = 1
msg_status.state = msg_status.STATE_FREE
msg_status.done_pct = 0
msg_status.battery = 0
msg_status.position = 'Station1'
msg_status.status = 'I am mobile robot 1'

pub = rospy.Publisher('/mes/mes_status_topic', mes_mobile_status)

# define state free
class StateFree(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_FREE')
        time.sleep(1)
        msg_status.state = msg_status.STATE_FREE
        pub.publish(msg_status)

        rospy.loginfo('Command was: %s', command)

        if command == mes_mobile_command.COMMAND_WAIT:
            return 'outcome1'
        elif command == mes_mobile_command.COMMAND_NAVIGATE or command == mes_mobile_command.COMMAND_TIP:
            return 'outcome2'
        else:
            return 'outcome1'


# define state working
class StateWorking(smach.State):
    global command, path
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_WORKING')
        time.sleep(1)
        msg_status.state = msg_status.STATE_WORKING
        pub.publish(msg_status)

        rospy.loginfo('Command was: %s', command)

        if command == mes_mobile_command.COMMAND_NAVIGATE:
            if self.counter < 3:
                rospy.loginfo('Navigating to position: %s ', path)
                self.counter += 1
                return 'outcome1'
            else:
                msg_status.position = path
                rospy.loginfo('Position: %s is reached!', path)
                return 'outcome2'

        elif command == mes_mobile_command.COMMAND_TIP:
            if self.counter < 3:
                rospy.loginfo('Tipping off bricks at: %s', msg_status.position)
                self.counter += 1
                return 'outcome1'
            else:
                rospy.loginfo('The bricks are tipped off at: %s', msg_status.position)
                return 'outcome2'
        elif command == mes_mobile_command.COMMAND_WAIT:
            return 'outcome2'


# define state navigate
class StateNavigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_NAVIGATE')
        return 'outcome1'


# define state tip
class StateTip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_TIP')
        return 'outcome1'


# define state error
class StateError(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE_ERROR')
        msg_status.state = msg_status.STATE_ERROR
        pub.publish(msg_status)
        return 'outcome1'


def cmdCallback(cmd):
    global command, path
    command = cmd.command
    path = cmd.path


def main():
    global command, path

    rospy.init_node('mobile_example_node')
    rospy.Subscriber("/mes/mes_command_topic", mes_mobile_command, cmdCallback)

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm_top:
        # Add states to the container

        smach.StateMachine.add('STATE_FREE', StateFree(),
                               transitions={'outcome1':'STATE_FREE', 'outcome2':'STATE_WORKING'})
        smach.StateMachine.add('STATE_WORKING', StateWorking(),
                               transitions={'outcome1':'STATE_WORKING', 'outcome2':'STATE_FREE'})
        smach.StateMachine.add('STATE_ERROR', StateError(),
                               transitions={'outcome1':'STATE_ERROR'})

    # Execute SMACH plan
    outcome = sm_top.execute()

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass