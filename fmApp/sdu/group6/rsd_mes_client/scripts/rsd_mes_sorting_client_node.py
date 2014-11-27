#!/usr/bin/env python

import rospy
import xmlrpclib
import datetime
from rsd_mes_client_node import RSDMesClientNode
from rsd_mes_client.msg import mes_sorting_command, mes_sorting_status, lego_brick, mes_order

server = xmlrpclib.ServerProxy('http://192.168.1.50:8000', use_datetime=True)
today = datetime.datetime.today()

c_status = {
    'version_id': 1,
    'robot_id': 3,
    'state': 'STATE_FREE',
    'time': str(today),
    'status': "Human readable status message.."
}

class RSDMesSortingClientNode(RSDMesClientNode):
    def __init__(self):
        RSDMesClientNode.__init__(self)
        return

    def initMsg(self):
        self.msg_command = mes_sorting_command()
        self.msg_status = mes_sorting_status()
    
    def initNode(self):
        self.rosnode = rospy.init_node('rsd_mes_sorting_client')
    
    def initTopic(self):
        self.mes_command_publisher = rospy.Publisher(self.mes_command_topic, mes_sorting_command, queue_size=1)
        rospy.Subscriber(self.mes_status_topic, mes_sorting_status, self.callbackStatus)

    def callbackStatus(self,status):
        self.msg_status.version_id = status.version_id
        self.msg_status.robot_id = status.robot_id
        self.msg_status.state = status.state
        self.msg_status.done_pct = status.done_pct
        self.msg_status.status = status.status

        self.getSortingStatus()

        sorting_response = (server.cell_status(c_status))

        self.setSortingCommand(sorting_response)

        self.publishCommand()

    def getSortingStatus(self):

        c_status['version_id'] = self.msg_status.version_id
        c_status['robot_id'] = self.msg_status.robot_id
        state = self.msg_status.state
        if state == self.msg_status.STATE_FREE:
            c_status['state'] = 'STATE_FREE'
        elif state == self.msg_status.STATE_SORTING:
            c_status['state'] = 'STATE_SORTING'
        elif state == self.msg_status.STATE_OUTOFBRICKS:
            c_status['state'] = 'STATE_OUTOFBRICKS'
        elif state == self.msg_status.STATE_ORDERSORTED:
            c_status['state'] = 'STATE_ORDERSORTED'
        elif state == self.msg_status.STATE_LOADING:
            c_status['state'] = 'STATE_LOADING'
        else:
            c_status['state'] = 'STATE_ERROR'

        #c_status['done_pct'] = self.msg_status.done_pct
        c_status['status'] = self.msg_status.status

    def setSortingCommand(self, c_response):
        self.msg_command.order = mes_order()
        if c_response['command'] == 'COMMAND_WAIT':
            self.msg_command.command = self.msg_command.COMMAND_WAIT
        elif c_response['command'] == 'COMMAND_SORTBRICKS':
            self.msg_command.command = self.msg_command.COMMAND_SORTBRICKS
            self.msg_command.order.order_id = c_response['order']['order_id']
            legos = c_response['order']['bricks']
            self.msg_command.order.bricks = []
            for i in range(len(legos)):
                self.msg_command.order.bricks.append(lego_brick())
                self.msg_command.order.bricks[i].color = legos[i]['color']
                self.msg_command.order.bricks[i].size = legos[i]['size']
                self.msg_command.order.bricks[i].count = legos[i]['count']
        elif c_response['command'] == 'COMMAND_LOADBRICKS':
            self.msg_command.command = self.msg_command.COMMAND_LOADBRICKS
        elif c_response['command'] == 'COMMAND_ABORT':
            self.msg_command.command = self.msg_command.COMMAND_ABORT

if __name__ == '__main__':
    try:
        node_class = RSDMesSortingClientNode()

    except rospy.ROSInterruptException:
        pass
    rospy.spin()