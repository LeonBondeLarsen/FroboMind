# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainWindow.ui'
#
# Created: Thu Nov 27 11:30:34 2014
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(573, 425)
        self.centralWidget = QtGui.QWidget(MainWindow)
        self.centralWidget.setObjectName(_fromUtf8("centralWidget"))
        self.commandWait = QtGui.QPushButton(self.centralWidget)
        self.commandWait.setGeometry(QtCore.QRect(290, 270, 271, 27))
        self.commandWait.setObjectName(_fromUtf8("commandWait"))
        self.commandNavigateDispenser = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateDispenser.setGeometry(QtCore.QRect(10, 60, 271, 27))
        self.commandNavigateDispenser.setObjectName(_fromUtf8("commandNavigateDispenser"))
        self.commandNavigateInBox = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateInBox.setGeometry(QtCore.QRect(10, 90, 271, 27))
        self.commandNavigateInBox.setObjectName(_fromUtf8("commandNavigateInBox"))
        self.robotStateFeedback = QtGui.QLabel(self.centralWidget)
        self.robotStateFeedback.setGeometry(QtCore.QRect(20, 0, 541, 51))
        self.robotStateFeedback.setFrameShape(QtGui.QLabel.StyledPanel)
        self.robotStateFeedback.setFrameShadow(QtGui.QLabel.Raised)
        self.robotStateFeedback.setObjectName(_fromUtf8("robotStateFeedback"))
        self.commandNavigateStation2 = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateStation2.setGeometry(QtCore.QRect(10, 150, 271, 27))
        self.commandNavigateStation2.setObjectName(_fromUtf8("commandNavigateStation2"))
        self.commandNavigateStation3 = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateStation3.setGeometry(QtCore.QRect(10, 180, 271, 27))
        self.commandNavigateStation3.setObjectName(_fromUtf8("commandNavigateStation3"))
        self.commandNavigateStation1 = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateStation1.setGeometry(QtCore.QRect(10, 120, 271, 27))
        self.commandNavigateStation1.setObjectName(_fromUtf8("commandNavigateStation1"))
        self.commandNavigateRampIn = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateRampIn.setGeometry(QtCore.QRect(10, 240, 271, 27))
        self.commandNavigateRampIn.setObjectName(_fromUtf8("commandNavigateRampIn"))
        self.commandNavigateFloorOut = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateFloorOut.setGeometry(QtCore.QRect(10, 270, 271, 27))
        self.commandNavigateFloorOut.setObjectName(_fromUtf8("commandNavigateFloorOut"))
        self.commandNavigateRampOut = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateRampOut.setGeometry(QtCore.QRect(10, 210, 271, 27))
        self.commandNavigateRampOut.setObjectName(_fromUtf8("commandNavigateRampOut"))
        self.commandNavigateFloorIn = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateFloorIn.setGeometry(QtCore.QRect(10, 300, 271, 27))
        self.commandNavigateFloorIn.setObjectName(_fromUtf8("commandNavigateFloorIn"))
        self.commandNavigateLine = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateLine.setGeometry(QtCore.QRect(10, 330, 271, 27))
        self.commandNavigateLine.setObjectName(_fromUtf8("commandNavigateLine"))
        self.commandNavigateLoadOn1 = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateLoadOn1.setGeometry(QtCore.QRect(290, 90, 271, 27))
        self.commandNavigateLoadOn1.setObjectName(_fromUtf8("commandNavigateLoadOn1"))
        self.commandNavigateLoadOff1 = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateLoadOff1.setGeometry(QtCore.QRect(290, 60, 271, 27))
        self.commandNavigateLoadOff1.setObjectName(_fromUtf8("commandNavigateLoadOff1"))
        self.commandNavigateLoadOn2 = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateLoadOn2.setGeometry(QtCore.QRect(290, 150, 271, 27))
        self.commandNavigateLoadOn2.setObjectName(_fromUtf8("commandNavigateLoadOn2"))
        self.commandNavigateLoadOff2 = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateLoadOff2.setGeometry(QtCore.QRect(290, 120, 271, 27))
        self.commandNavigateLoadOff2.setObjectName(_fromUtf8("commandNavigateLoadOff2"))
        self.commandNavigateLoadOff3 = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateLoadOff3.setGeometry(QtCore.QRect(290, 180, 271, 27))
        self.commandNavigateLoadOff3.setObjectName(_fromUtf8("commandNavigateLoadOff3"))
        self.commandNavigateLoadOn3 = QtGui.QPushButton(self.centralWidget)
        self.commandNavigateLoadOn3.setGeometry(QtCore.QRect(290, 210, 271, 27))
        self.commandNavigateLoadOn3.setObjectName(_fromUtf8("commandNavigateLoadOn3"))
        self.commandTip = QtGui.QPushButton(self.centralWidget)
        self.commandTip.setGeometry(QtCore.QRect(290, 240, 271, 27))
        self.commandTip.setObjectName(_fromUtf8("commandTip"))
        self.commandAbort = QtGui.QPushButton(self.centralWidget)
        self.commandAbort.setGeometry(QtCore.QRect(290, 300, 271, 27))
        self.commandAbort.setObjectName(_fromUtf8("commandAbort"))
        self.manualMode = QtGui.QPushButton(self.centralWidget)
        self.manualMode.setGeometry(QtCore.QRect(290, 330, 271, 27))
        self.manualMode.setObjectName(_fromUtf8("manualMode"))
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtGui.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 573, 25))
        self.menuBar.setObjectName(_fromUtf8("menuBar"))
        self.menuRSD_User_Interface = QtGui.QMenu(self.menuBar)
        self.menuRSD_User_Interface.setObjectName(_fromUtf8("menuRSD_User_Interface"))
        MainWindow.setMenuBar(self.menuBar)
        self.mainToolBar = QtGui.QToolBar(MainWindow)
        self.mainToolBar.setObjectName(_fromUtf8("mainToolBar"))
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtGui.QStatusBar(MainWindow)
        self.statusBar.setObjectName(_fromUtf8("statusBar"))
        MainWindow.setStatusBar(self.statusBar)
        self.menuBar.addAction(self.menuRSD_User_Interface.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.commandWait.setText(_translate("MainWindow", "COMMAND_WAIT", None))
        self.commandNavigateDispenser.setText(_translate("MainWindow", "COMMAND_NAVIGATE Dispenser", None))
        self.commandNavigateInBox.setText(_translate("MainWindow", "COMMAND_NAVIGATE InBox", None))
        self.commandNavigateStation2.setText(_translate("MainWindow", "COMMAND_NAVIGATE Station2", None))
        self.commandNavigateStation3.setText(_translate("MainWindow", "COMMAND_NAVIGATE Station3", None))
        self.commandNavigateStation1.setText(_translate("MainWindow", "COMMAND_NAVIGATE Station1", None))
        self.commandNavigateRampIn.setText(_translate("MainWindow", "COMMAND_NAVIGATE RampIn", None))
        self.commandNavigateFloorOut.setText(_translate("MainWindow", "COMMAND_NAVIGATE FloorOut", None))
        self.commandNavigateRampOut.setText(_translate("MainWindow", "COMMAND_NAVIGATE RampOut", None))
        self.commandNavigateFloorIn.setText(_translate("MainWindow", "COMMAND_NAVIGATE FloorIn", None))
        self.commandNavigateLine.setText(_translate("MainWindow", "COMMAND_NAVIGATE Line", None))
        self.commandNavigateLoadOn1.setText(_translate("MainWindow", "COMMAND_NAVIGATE LoadOn1", None))
        self.commandNavigateLoadOff1.setText(_translate("MainWindow", "COMMAND_NAVIGATE LoadOff1", None))
        self.commandNavigateLoadOn2.setText(_translate("MainWindow", "COMMAND_NAVIGATE LoadOn2", None))
        self.commandNavigateLoadOff2.setText(_translate("MainWindow", "COMMAND_NAVIGATE LoadOff2", None))
        self.commandNavigateLoadOff3.setText(_translate("MainWindow", "COMMAND_NAVIGATE LoadOff3", None))
        self.commandNavigateLoadOn3.setText(_translate("MainWindow", "COMMAND_NAVIGATE LoadOn3", None))
        self.commandTip.setText(_translate("MainWindow", "COMMAND_TIP", None))
        self.commandAbort.setText(_translate("MainWindow", "COMMAND_ABORT", None))
        self.manualMode.setText(_translate("MainWindow", "Manual mode", None))
        self.menuRSD_User_Interface.setTitle(_translate("MainWindow", "RSD User Interface", None))

