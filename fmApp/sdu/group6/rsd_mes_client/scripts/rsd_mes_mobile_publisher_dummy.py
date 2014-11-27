#!usr/bin/env python
__author__ = 'armienn'

import rospy
from rsd_mes_client.msg import mes_mobile_command, mes_mobile_status

def publish_loop():
    msg_status = mes_mobile_status()
    msg_status.version_id = 1
    msg_status.robot_id = 1
    msg_status.state = msg_status.STATE_FREE
    msg_status.done_pct = 0
    msg_status.battery = 0
    msg_status.position = 'Station1'
    msg_status.status = 'Hi'

    pub = rospy.Publisher('/mes/mes_status_topic', mes_mobile_status)
    rospy.init_node('mobiledummy', anonymous=True)
    while not rospy.is_shutdown():
        pub.publish(msg_status)
        response = raw_input('Input something')
        print response


if __name__ == '__main__':
    try:
        publish_loop()
    except rospy.ROSInterruptException:
        print 'damn'
        pass
