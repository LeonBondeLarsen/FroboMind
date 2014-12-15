import os
import rospy
import rospkg
from PyQt4 import QtCore, QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt4.QtGui import QWidget, QPushButton, QLabel
import sys
from std_msgs.msg import String
from msgs.msg import StringStamped
from math import pi, atan2
from geometry_msgs.msg import TwistStamped,PoseStamped
from nav_msgs.msg import Odometry
 
class UIPlugin(Plugin):
    '''
    Initializes a Plugin which displays the UI for the frobit robot
     
    '''
    def __init__(self, context):
     
         
        super(UIPlugin, self).__init__(context)
         
        self.setObjectName('MyPlugin')  
         
        name="RQT Plugin Thread"
        self.t = WorkerThread(name, self,context)
        self.t.start()
   
class WorkerThread(QtCore.QThread):
    '''
    The thread which runs the GUI. All the user interactions with the UI are handled in this thread.
    '''
    def __init__(self, name, receiver,context):
        
        self.output_topic = rospy.get_param("~task_publisher", "/fmDecisionMaking/task")
        self.state_topic = rospy.get_param("~state","/fmInformation/state")
        self.robot_loc_topic = rospy.get_param("~rsd_area_topic","/fmKnowledge/rsd_area")
        self.vel_topic = rospy.get_param("~cmd_vel_topic","/fmCommand/cmd_vel" )
        self.pose_topic = rospy.get_param("odom_topic","/fmKnowledge/odom")
           
        self.output_pub = rospy.Publisher(self.output_topic, StringStamped, queue_size=10)
        self.state_sub = rospy.Subscriber(self.state_topic,StringStamped,self.on_state_topic)
        self.robot_loc_sub = rospy.Subscriber(self.robot_loc_topic,StringStamped,self.on_robotloctopic)
        self.vel_sub = rospy.Subscriber(self.vel_topic,TwistStamped,self.on_veltopic)
        self.pose_sub = rospy.Subscriber(self.pose_topic,PoseStamped,self.on_posetopic)
         
        self.linearVelocity=0
        self.pose_x=0
        self.pose_y=0
        self.yaw=0
        self.robot_loc=0
        self.state_1 = 'STATE_FREE'
        self.state_2 = 'STATE_ERROR'
        self.state_3 = 'STATE_WORKING'
      
        self.mode=0
        self.task_msg = StringStamped()
        QtCore.QThread.__init__(self)
        self.context = context
        self.name = name
        self.receiver = receiver
        self.guiInit(self.context)
         
    def guiInit(self,context):
        '''
        Initializes the GUI and all the different widgets
        '''
 
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'FrobitGUI.ui')
        loadUi(ui_file, self._widget)
     
        self._widget.setObjectName('UIPluginUi')
        
        if context.serial_number() > 1:
               self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
           # Add widget to the user interface
        context.add_widget(self._widget)
         
        # Robot Control Widgets
        self.stopButton = self._widget.stopButton
        self.stopButton.pressed.connect(self.stopButtonClicked)
        self.autoModeButton = self._widget.autoModeButton
        self.autoModeButton.pressed.connect(self.autoModeButtonClicked)
        self.manualModeButton = self._widget.manualModeButton
        self.manualModeButton.pressed.connect(self.manualModeButtonClicked)
        
         
        # Robot Status Indicators
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.velocityLabel = self._widget.speedValueLabel
        self.velocityLabel.setFont(font)
        self.modeLabel = self._widget.modeLabel
        self.modeLabel.setFont(font)
        self.pose_xLabel = self._widget.pose_x_label
        self.pose_xLabel.setFont(font)
        self.pose_yLabel = self._widget.pose_y_label
        self.pose_yLabel.setFont(font) 
        self.yawLabel = self._widget.yawlabel
        self.yawLabel.setFont(font)
        self.robotLocationLabel = self._widget.robotlocationlabel 
        self.robotLocationLabel.setFont(font)
        self.freeState = self._widget.freeState
        self.errorState= self._widget.errorState
        self.workingState = self._widget.workingState
    
    def run(self):
        while not rospy.is_shutdown():
            self.updateGUI()     
            
    def updateGUI(self):
           self.velocityLabel.setText(str(self.linearVelocity))
           self.pose_xLabel.setText(str(self.pose_x))
           self.pose_yLabel.setText(str(self.pose_y))
           self.yawLabel.setText(str(self.yaw))
           self.robotLocationLabel.setText(str(self.robot_loc))
           if(self.mode==0):
               self.modeLabel.setText("Auto")
           else:
               self.modeLabel.setText("Manual")
            
    # Event Handlers
    def clearStates(self):
        
        self.freeState.setStyleSheet("background-color:lightgreen")
        self.errorState.setStyleSheet("background-color:lightgreen")
        self.workingState.setStyleSheet("background-color:lightgreen")
        
    def on_robotloctopic(self,msg):
        self.robot_loc=msg
         
    def on_state_topic(self,msg):
        if(msg==self.state_1):
            self.clearStates()
            self.freeState.setStyleSheet("background-color:red")
        
        elif(msg==self.state_2):
            self.clearStates()
            self.errorState.setStyleSheet("background-color:red")
        
        elif(msg==self.state_3):
            self.clearStates()
            self.workingState.setStyleSheet("background-color:red")
            
                   
         
    def on_posetopic(self,msg):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        self.pose_x=msg.pose.pose.position.x
        self.pose_y=msg.pose.pose.position.y
        self.yaw=yaw
         
    def on_veltopic(self,msg):
        self.linearVelocity=msg.twist.twist.linear.x
 
         
    def stopButtonClicked(self):   
        self.stopButton.setStyleSheet("background-color: blue")
        self.freeState.setStyleSheet("background-color:red")
        self.task_msg.data = 'WAIT'
        self.output_pub.publish(self.task_msg)
             
    def autoModeButtonClicked(self):
        self.mode=0
        self.task_msg.data = 'AUTO'
        self.output_pub.publish(self.task_msg)
         
    def manualModeButtonClicked(self):
        self.mode=1
        self.task_msg.data = 'MANUAL'
        self.output_pub.publish(self.task_msg)
         
         
         
         