import os
import rospy
import rospkg
from PyQt4 import QtCore, QtGui, uic
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt4.QtGui import QWidget, QPushButton, QLabel
from test import MainWindow
import sys

class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')  
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet",
                         help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
          
        if not args.quiet:
               print 'arguments: ', args
               print 'unknowns: ', unknowns
     
           # Create QWidget
        self._widget = QWidget()
           # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'mainwindow.ui')
           # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
    
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
          # Show _widget.windowTitle on left-top of each plugin (when 
         # it's set in _widget). This is useful when you open multiple 
           # plugins at once. Also if you open multiple instances of your 
           # plugin at once, these lines add number to make it easy to 
           # tell from pane to pane.
        if context.serial_number() > 1:
               self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
           # Add widget to the user interface
        context.add_widget(self._widget)
          
#         self.text = QLabel("Data")
        print("Ishan")
        self._widget.pushButton.pressed.connect(self.buttonClicked)
        
    def buttonClicked(self):
        print("Button Clicked")
        self._widget.label.setText("Button Clicked")
#    