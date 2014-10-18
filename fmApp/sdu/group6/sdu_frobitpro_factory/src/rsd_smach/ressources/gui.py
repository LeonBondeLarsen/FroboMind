# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainWindow.ui'
#
# Created: Sat Oct 18 12:34:26 2014
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
        MainWindow.resize(168, 219)
        self.centralWidget = QtGui.QWidget(MainWindow)
        self.centralWidget.setObjectName(_fromUtf8("centralWidget"))
        self.buttonIdle = QtGui.QPushButton(self.centralWidget)
        self.buttonIdle.setGeometry(QtCore.QRect(30, 20, 111, 27))
        self.buttonIdle.setObjectName(_fromUtf8("buttonIdle"))
        self.buttonExecuteJob1 = QtGui.QPushButton(self.centralWidget)
        self.buttonExecuteJob1.setGeometry(QtCore.QRect(30, 60, 111, 27))
        self.buttonExecuteJob1.setObjectName(_fromUtf8("buttonExecuteJob1"))
        self.buttonExecuteJob2 = QtGui.QPushButton(self.centralWidget)
        self.buttonExecuteJob2.setGeometry(QtCore.QRect(30, 100, 111, 27))
        self.buttonExecuteJob2.setObjectName(_fromUtf8("buttonExecuteJob2"))
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtGui.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 168, 25))
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
        self.buttonIdle.setText(_translate("MainWindow", "Idle", None))
        self.buttonExecuteJob1.setText(_translate("MainWindow", "Execute job 1", None))
        self.buttonExecuteJob2.setText(_translate("MainWindow", "Execute job 2", None))
        self.menuRSD_User_Interface.setTitle(_translate("MainWindow", "RSD User Interface", None))

