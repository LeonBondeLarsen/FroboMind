import os
import rospy
import rospkg
from PyQt4 import QtCore, QtGui, uic
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt4.QtGui import QWidget, QPushButton, QLabel
import sys
from std_msgs.msg import String

class UIPlugin(Plugin):
    '''
    Initializes a Plugin which displays the UI for the frobit robot
    
    '''
    def __init__(self, context):
        rospy.loginfo("New Version")
	
        super(UIPlugin, self).__init__(context)
        
        self.setObjectName('MyPlugin')  
        
        self.threads = []
        name="RQT Plugin Thread"
        t = WorkerThread(name, self,context)
        t.start()
        self.threads.append(t)
  
    def __del__(self):
        for t in self.threads:
            running = t.running()
            t.stop()
            if not t.finished():
                t.wait()
       
          
class WorkerThread(QtCore.QThread):
    '''
    The thread which runs the GUI. All the user interactions with the UI are handled in this thread.
    '''
    def __init__(self, name, receiver,context):
        rospy.loginfo("Worker Thread init() method "+name)
	output_topic = rospy.get_param("~task_publisher", "/fmDecisionMaking/task")
	self.pub = rospy.Publisher(output_topic, StringStamped, queue_size=10)
	self.task_msg = StringStamped()

        QtCore.QThread.__init__(self)
        self.name = name
        self.context = context
        self.receiver = receiver
        self.stopped = False
        self.guiInit(self.context)
        
    def run(self):
#         rospy.loginfo("Worker Thread run() method")

        while not self.stopped:
            self.guiWork()
            rospy.sleep(60)
    
    def stop(self):
        self.stopped = True

    def guiInit(self,context):
#         rospy.loginfo("Worker Thread guiInit() method")

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'FrobitGUI.ui')
        loadUi(ui_file, self._widget)
    
        self._widget.setObjectName('UIPluginUi')
       
        if context.serial_number() > 1:
               self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
           # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Robot Control Widgets
        self._widget.startButton.pressed.connect(self.startButtonClicked)
        self._widget.stopButton.pressed.connect(self.stopButtonClicked)
        self._widget.turnrightButton.pressed.connect(self.turnrightButtonClicked)
        self._widget.turnleftButton.pressed.connect(self.turnleftButtonClicked)
        self._widget.autoModeButton.pressed.connect(self.autoModeButtonClicked)
        self._widget.manualModeButton.pressed.connect(self.manualModeButtonClicked)
        
        # Robot Status Indicators
        self.speedValue = self._widget.speedValueLabel
        self.distanceTravelled = self._widget.distanceTravelledValueLabel
        self.timeElapsed = self._widget.timeElapsedValueLabel
        self.mode = self._widget.modeValueLabel
        
        
    
    def guiWork(self):
        while not rospy.is_shutdown():
            rospy.sleep(10)

       
        
    
    # Event Handlers
    def startButtonClicked(self):
        rospy.loginfo(rospy.get_name() + ": Task 1 selected")
	self.task_msg.data = 'JOB1'
	self.pub.publish(self.task_msg)
        
    def stopButtonClicked(self):   
        print("Stop Button Clicked")
    
    def turnrightButtonClicked(self):
        rospy.loginfo(rospy.get_name() + ": Task 2 selected")
	self.task_msg.data = 'JOB2'
	self.pub.publish(self.task_msg)
    
    def turnleftButtonClicked(self):
        print("Turn Left Button Clicked")
        
    def autoModeButtonClicked(self):
        print("Auto Mode Button Clicked")
        
    def manualModeButtonClicked(self):
	rospy.loginfo(rospy.get_name() + ": Manual mode selected")
	self.task_msg.data = 'IDLE'
	self.pub.publish(self.task_msg)
        
        
        
        
