#!/usr/bin/env python

import rospy
import xmlrpclib
import datetime
from rsd_mes_client_node import RSDMesClientNode
from rsd_mes_client.msg import mes_mobile_command, mes_mobile_status

server = xmlrpclib.ServerProxy('http://192.168.1.50:8000', use_datetime=True)
today = datetime.datetime.today()

m_status = {
    'version_id': 1,
    'robot_id': 3,
    'state': 'STATE_FREE',
    'time': str(today),
    'battery': 80,
    'position': "Station3",
    'status': "Human readable status message.."
}

class RSDMesMobileClientNode(RSDMesClientNode):
    def __init__(self):
        RSDMesClientNode.__init__(self)
        return

    def initMsg(self):
        self.msg_command = mes_mobile_command()
        self.msg_status = mes_mobile_status()    
    
    def initNode(self):
        self.rosnode = rospy.init_node('rsd_mes_mobile_client')
    
    def initTopic(self):
        self.mes_command_publisher = rospy.Publisher(self.mes_command_topic, mes_mobile_command, queue_size=1)
        rospy.Subscriber(self.mes_status_topic, mes_mobile_status, self.callbackStatus)

    def callbackStatus(self,status):
        self.msg_status.version_id = status.version_id
        self.msg_status.robot_id = status.robot_id
        self.msg_status.state = status.state
        self.msg_status.done_pct = status.done_pct
        self.msg_status.battery = status.battery
        self.msg_status.position = status.position
        self.msg_status.status = status.status

        self.getMobileStatus()

        mobile_response = (server.mobile_status(m_status))

        self.setMobileCommand(mobile_response)

        self.publishCommand()

    def getMobileStatus(self):

        m_status['version_id'] = self.msg_status.version_id
        m_status['robot_id'] = self.msg_status.robot_id
        state = self.msg_status.state
        if state == self.msg_status.STATE_FREE:
            m_status['state'] = 'STATE_FREE'
        elif state == self.msg_status.STATE_WORKING:
            m_status['state'] = 'STATE_WORKING'
        else:
            m_status['state'] = 'STATE_ERROR'

        #mobile_status['done_pct'] = self.msg_status.done_pct
        m_status['battery'] = self.msg_status.battery
        m_status['position'] = self.msg_status.position
        m_status['status'] = self.msg_status.status

    def setMobileCommand(self, m_response):
        self.msg_command.path = ''
        if m_response['command'] == 'COMMAND_NAVIGATE':
            self.msg_command.command = self.msg_command.COMMAND_NAVIGATE
            self.msg_command.path = m_response['path']
        elif m_response['command'] == 'COMMAND_WAIT':
            self.msg_command.command = self.msg_command.COMMAND_WAIT
        elif m_response['command'] == 'COMMAND_TIP':
            self.msg_command.command = self.msg_command.COMMAND_TIP
        elif m_response['command'] == 'COMMAND_ABORT':
            self.msg_command.command = self.msg_command.COMMAND_ABORT

if __name__ == '__main__':
    try:
        node_class = RSDMesMobileClientNode()

    except rospy.ROSInterruptException:
        pass
    rospy.spin()