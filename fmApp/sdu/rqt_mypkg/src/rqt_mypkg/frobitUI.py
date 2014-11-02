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
        rospy.loginfo("New New Version")

        super(UIPlugin, self).__init__(context)
#         self.testDataTopic = rospy.get_param("~testData","/fmTest/testData")
#         rospy.Subscriber(self.testDataTopic,String,self.on_testData)
#         self.frobitUI_data = rospy.get_param("~frobitUIData",'/frobitUI/pub')
#         self.pub = rospy.Publisher(self.frobitUI_data,String)
        self.setObjectName('MyPlugin')  
        
        self.threads = []
        name="RQT Plugin Thread"
        t = WorkerThread(name, self,context)
        t.start()
        self.threads.append(t)
#        
      
    def on_testData(self,msg):
        rospy.loginfo("Message received is "+msg.data)
  
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
        self.frobitUI_data = rospy.get_param("~frobitUIData",'/frobitUI/pub')
        self.pub = rospy.Publisher(self.frobitUI_data,String)
        QtCore.QThread.__init__(self)
        self.name = name
        self.context = context
        self.receiver = receiver
        self.stopped = False
        self.guiInit(self.context)
        
    def run(self):
        rospy.loginfo("Worker Thread run() method")

        while not self.stopped:
            self.guiWork()
            rospy.sleep(60)
    
    def stop(self):
        self.stopped = True

    def guiInit(self,context):
        rospy.loginfo("Worker Thread guiInit() method")

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
        print("New Version")
        while not rospy.is_shutdown():
            self.pub.publish("RSD")        
            rospy.sleep(10)
            PackML()# use smach to implement the state pattern
       
        
    
    # Event Handlers
    def startButtonClicked(self):
        print("Start Button Clicked")  
        self.speedValue.setText("23")        
        
    def stopButtonClicked(self):   
        print("Stop Button Clicked")
    
    def turnrightButtonClicked(self):
        print("Turn Right Button Clicked")
    
    def turnleftButtonClicked(self):
        print("Turn Left Button Clicked")
        
    def autoModeButtonClicked(self):
        print("Auto Mode Button Clicked")
        
    def manualModeButtonClicked(self):
        print("Manual Mode Button Clicked")
        
        
        
        
