__author__ = 'kristjan'

import rospy
import smach
from sdu_frobitpro_factory.msg import mes_mobile_command, mes_mobile_status
from msgs.msg import StringStamped

class ServerControl(smach.State):
    """
    """
    def __init__(self, mission_control):
        smach.State.__init__(self, outcomes=[])
        
        self.publisher_status = rospy.Publisher('/mes/status', mes_mobile_status, queue_size=1)
        self.publisher_command = rospy.Publisher('/fmDecisionMaking/task', StringStamped, queue_size=1)
        self.subscriber_area = rospy.Subscriber('/fmKnowledge/rsd_area', StringStamped, self.update_area )
        self.subscriber_command = rospy.Subscriber('/mes/command', mes_mobile_command, self.handle_command )
        
        self.r = rospy.Rate(2)
        self.mission_control = mission_control
        self.area = 'UNKNOWN'
        self.mode = 'AUTO'
        

    def execute(self, userdata):
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                break
            
            current_state = self.mission_control.get_active_states()[0]
            if current_state == 'MANUAL':
                self.mode = 'MANUAL'
            else:
                self.mode = 'AUTO'
            
            if self.mode == 'AUTO':
                status_msg = mes_mobile_status()
                status_msg.header.stamp = rospy.Time.now()
                status_msg.version_id = 0
                status_msg.robot_id = 3
                status_msg.done_pct = 0
                status_msg.battery = 0
                status_msg.status = 'Hello from the server communication control module'
                status_msg.position = convert_position(self.area)
                if current_state == 'WAIT':
                    status_msg.state = mes_mobile_status.STATE_FREE
                elif current_state == 'ABORT':
                    status_msg.state = mes_mobile_status.STATE_ERROR
                else:
                    status_msg.state = mes_mobile_status.STATE_WORKING
                
                self.publisher_status.publish(status_msg)
                 
            try :
                self.r.sleep()
            except rospy.ROSInterruptException:
                break
    
    
    def update_area(self,msg):
        print 'updating area with: ', msg.data
        self.area = msg.data

    def handle_command(self,msg):
        if self.mode == 'AUTO':
            cmdmsg = StringStamped()
            if msg.command == mes_mobile_command.COMMAND_NAVIGATE:
                if msg.path == 'Dispenser':
                    cmdmsg.data = 'NAVIGATE_DISPENSER'
                elif msg.path == 'InBox':
                    cmdmsg.data = 'NAVIGATE_IN_BOX'
                elif msg.path == 'RampOut':
                    cmdmsg.data = 'NAVIGATE_RAMP_OUT'
                elif msg.path == 'RampIn':
                    cmdmsg.data = 'NAVIGATE_RAMP_IN'
                elif msg.path == 'FloorOut':
                    cmdmsg.data = 'NAVIGATE_FLOOR_OUT'
                elif msg.path == 'FloorIn':
                    cmdmsg.data = 'NAVIGATE_FLOOR_IN'
                elif msg.path == 'Line':
                    cmdmsg.data = 'NAVIGATE_LINE'
                elif msg.path == 'Station1':
                    cmdmsg.data = 'NAVIGATE_STATION_1'
                elif msg.path == 'Station2':
                    cmdmsg.data = 'NAVIGATE_STATION_2'
                elif msg.path == 'Station3':
                    cmdmsg.data = 'NAVIGATE_STATION_3'
                elif msg.path == 'LoadOff1':
                    cmdmsg.data = 'NAVIGATE_LOAD_OFF_1'
                elif msg.path == 'LoadOff2':
                    cmdmsg.data = 'NAVIGATE_LOAD_OFF_2'
                elif msg.path == 'LoadOff3':
                    cmdmsg.data = 'NAVIGATE_LOAD_OFF_3'
                elif msg.path == 'LoadOn1':
                    cmdmsg.data = 'NAVIGATE_LOAD_ON_1'
                elif msg.path == 'LoadOn2':
                    cmdmsg.data = 'NAVIGATE_LOAD_ON_2'
                elif msg.path == 'LoadOn3':
                    cmdmsg.data = 'NAVIGATE_LOAD_ON_3'
            elif msg.command == mes_mobile_command.COMMAND_TIP:
                cmdmsg.data = 'TIP'
            elif msg.command == mes_mobile_command.COMMAND_WAIT:
                cmdmsg.data = 'WAIT'
            elif msg.command == mes_mobile_command.COMMAND_ABORT:
                cmdmsg.data = 'ABORT'
            self.publisher_command.publish(cmdmsg)
    
def convert_position(pos):
    result = 'Unknown'
    if pos == 'DISPENSER':
        result = 'Dispenser'
    elif pos == 'IN_BOX':
        result = 'InBox'
    elif pos == 'RAMP_OUT':
        result = 'RampOut'
    elif pos == 'RAMP_IN':
        result = 'RampIn'
    elif pos == 'FLOOR_OUT':
        result = 'FloorOut'
    elif pos == 'FLOOR_IN':
        result = 'FloorIn'
    elif pos == 'LINE':
        result = 'Line'
    elif pos == 'STATION_1':
        result = 'Station1'
    elif pos == 'STATION_2':
        result = 'Station2'
    elif pos == 'STATION_3':
        result = 'Station3'
    elif pos == 'LOAD_OFF_1':
        result = 'LoadOff1'
    elif pos == 'LOAD_OFF_2':
        result = 'LoadOff2'
    elif pos == 'LOAD_OFF_3':
        result = 'LoadOff3'
    elif pos == 'LOAD_ON_1':
        result = 'LoadOn1'
    elif pos == 'LOAD_ON_2':
        result = 'LoadOn2'
    elif pos == 'LOAD_ON_3':
        result = 'LoadOn3'
    return result
