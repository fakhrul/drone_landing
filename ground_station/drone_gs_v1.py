# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'drone_gs.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

# reference
# https://pyshine.com/Video-processing-in-Python-with-OpenCV-and-PyQt5-GUI/
# https://www.youtube.com/watch?v=ZyFYBawiA24
# https://realpython.com/intro-to-python-threading/
#     
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QImage
import cv2
import socket
import base64
import numpy as np
import threading
import errno
from time import sleep

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1024, 768)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.lblVideo = QtWidgets.QLabel(self.centralwidget)
        self.lblVideo.setGeometry(QtCore.QRect(0, 0, 640, 480))
        self.lblVideo.setText("")
        self.lblVideo.setPixmap(QtGui.QPixmap("no_camera.jpg"))
        self.lblVideo.setScaledContents(True)
        self.lblVideo.setObjectName("lblVideo")
        self.lvMessage = QtWidgets.QListWidget(self.centralwidget)
        self.lvMessage.setGeometry(QtCore.QRect(639, 0, 381, 711))
        self.lvMessage.setObjectName("lvMessage")
        self.btnTakeOff = QtWidgets.QPushButton(self.centralwidget)
        self.btnTakeOff.setGeometry(QtCore.QRect(208, 490, 100, 50))
        self.btnTakeOff.setObjectName("btnTakeOff")
        self.btnLand = QtWidgets.QPushButton(self.centralwidget)
        self.btnLand.setGeometry(QtCore.QRect(302, 490, 100, 50))
        self.btnLand.setObjectName("btnLand")
        self.btnDisarm = QtWidgets.QPushButton(self.centralwidget)
        self.btnDisarm.setGeometry(QtCore.QRect(114, 490, 100, 50))
        self.btnDisarm.setObjectName("btnDisarm")
        self.btnArm = QtWidgets.QPushButton(self.centralwidget)
        self.btnArm.setGeometry(QtCore.QRect(20, 490, 100, 50))
        self.btnArm.setObjectName("btnArm")
        self.btnForward = QtWidgets.QPushButton(self.centralwidget)
        self.btnForward.setGeometry(QtCore.QRect(390, 550, 70, 70))
        self.btnForward.setObjectName("btnForward")
        self.chkEnableAutoLanding = QtWidgets.QCheckBox(self.centralwidget)
        self.chkEnableAutoLanding.setGeometry(QtCore.QRect(460, 490, 151, 20))
        self.chkEnableAutoLanding.setObjectName("chkEnableAutoLanding")
        self.chkEnableStream = QtWidgets.QCheckBox(self.centralwidget)
        self.chkEnableStream.setGeometry(QtCore.QRect(460, 510, 151, 20))
        self.chkEnableStream.setObjectName("chkEnableStream")
        self.btnThrotleUp = QtWidgets.QPushButton(self.centralwidget)
        self.btnThrotleUp.setGeometry(QtCore.QRect(110, 550, 70, 70))
        self.btnThrotleUp.setObjectName("btnThrotleUp")
        self.btnThrotleDown = QtWidgets.QPushButton(self.centralwidget)
        self.btnThrotleDown.setGeometry(QtCore.QRect(110, 620, 70, 70))
        self.btnThrotleDown.setObjectName("btnThrotleDown")
        self.btnBackward = QtWidgets.QPushButton(self.centralwidget)
        self.btnBackward.setGeometry(QtCore.QRect(390, 620, 70, 70))
        self.btnBackward.setObjectName("btnBackward")
        self.btnMoveLeft = QtWidgets.QPushButton(self.centralwidget)
        self.btnMoveLeft.setGeometry(QtCore.QRect(320, 580, 70, 70))
        self.btnMoveLeft.setObjectName("btnMoveLeft")
        self.btnMoveRight = QtWidgets.QPushButton(self.centralwidget)
        self.btnMoveRight.setGeometry(QtCore.QRect(460, 580, 70, 70))
        self.btnMoveRight.setObjectName("btnMoveRight")
        self.btnYawLeft = QtWidgets.QPushButton(self.centralwidget)
        self.btnYawLeft.setGeometry(QtCore.QRect(40, 580, 70, 70))
        self.btnYawLeft.setObjectName("btnYawLeft")
        self.btnYawRight = QtWidgets.QPushButton(self.centralwidget)
        self.btnYawRight.setGeometry(QtCore.QRect(180, 580, 70, 70))
        self.btnYawRight.setObjectName("btnYawRight")
        self.lblIpAddress = QtWidgets.QLabel(self.centralwidget)
        self.lblIpAddress.setGeometry(QtCore.QRect(10, 700, 55, 16))
        self.lblIpAddress.setObjectName("lblIpAddress")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1024, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        #added code

        self.SERVER_IP = "127.0.0.1"
        self.SERVER_PORT = 9999
        
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.bind(("127.0.0.1", 9998))
        self.client_socket.setblocking(False)

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.setblocking(False)

        self.isShowStream = True
        self.thread = threading.Thread(target=self.loadStreamVideo, args=())
        self.thread.start()

        self.btnArm.clicked.connect(self.arm)
        self.btnDisarm.clicked.connect(self.disarm)
        self.btnTakeOff.clicked.connect(self.takeOff)
        self.btnLand.clicked.connect(self.land)

        self.btnThrotleUp.clicked.connect(self.throtleUp)
        self.btnThrotleDown.clicked.connect(self.throtleDown)
        self.btnYawLeft.clicked.connect(self.yawLeft)
        self.btnYawRight.clicked.connect(self.yawRight)
        
        self.btnForward.clicked.connect(self.forward)
        self.btnBackward.clicked.connect(self.backward)
        self.btnMoveLeft.clicked.connect(self.moveLeft)
        self.btnMoveRight.clicked.connect(self.moveRight)

        self.chkEnableAutoLanding.stateChanged.connect(self.enableAutoLanding)
        self.chkEnableStream.stateChanged.connect(self.enableStream)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.btnTakeOff.setText(_translate("MainWindow", "Take Off"))
        self.btnLand.setText(_translate("MainWindow", "Land"))
        self.btnDisarm.setText(_translate("MainWindow", "Disarm"))
        self.btnArm.setText(_translate("MainWindow", "Arm"))
        self.btnForward.setText(_translate("MainWindow", "Forward"))
        self.chkEnableAutoLanding.setText(_translate("MainWindow", "Enable Auto Landing"))
        self.chkEnableStream.setText(_translate("MainWindow", "Enable Streaming"))
        self.btnThrotleUp.setText(_translate("MainWindow", "Up"))
        self.btnThrotleDown.setText(_translate("MainWindow", "Down"))
        self.btnBackward.setText(_translate("MainWindow", "Backward"))
        self.btnMoveLeft.setText(_translate("MainWindow", "Move Left"))
        self.btnMoveRight.setText(_translate("MainWindow", "Move Right"))
        self.btnYawLeft.setText(_translate("MainWindow", "Yaw Left"))
        self.btnYawRight.setText(_translate("MainWindow", "Yaw Right"))
        self.lblIpAddress.setText(_translate("MainWindow", "127.0.0.1"))
    
    # my code

    def loadLocalVideo(self):
        cap = cv2.VideoCapture(0)
        while cap.isOpened():
            img, self.image = cap.read()
            self.image = cv2.resize(self.image, (640, 480))
            frame = self.image
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            self.lblVideo.setPixmap(QtGui.QPixmap.fromImage(image))
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    def loadStreamVideo(self):
        while self.isShowStream:
            try:
                packet,_ = self.client_socket.recvfrom(65536)
            except socket.error as e:
                err = e.args[0]
                # if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                #     # print('No data available')
                #     continue
                # else:
                #     # a "real" error occurred
                #     print('socket error', e)
                #     break
            else:
                data = base64.b64decode(packet, ' /')
                npdata = np.fromstring(data, dtype=np.uint8)
                frame = cv2.imdecode(npdata,1)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
                self.lblVideo.setPixmap(QtGui.QPixmap.fromImage(image))
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

        print('stream closed')

    def close(self):
        self.isShowStream = False

    def addToList(self, message):
        self.lvMessage.insertItem(0, message)
        if self.lvMessage.count() > 100:
            self.lvMessage.takeItem(self.lvMessage.count() - 1)

    def arm(self):
        self.addToList('arm')
        self.server_socket.sendto(b'r', (self.SERVER_IP, self.SERVER_PORT))
        print('arm sent')

    def disarm(self):
        self.addToList('disarm')
        self.server_socket.sendto(b'i', (self.SERVER_IP, self.SERVER_PORT))
        print('disarm sent')

    def takeOff(self):
        print('takeOff')
        self.server_socket.sendto(b't', (self.SERVER_IP, self.SERVER_PORT))
        print('takeOff sent')

    def land(self):
        print('land')
        self.server_socket.sendto(b'l', (self.SERVER_IP, self.SERVER_PORT))
        print('land sent')

    def throtleUp(self):
        self.server_socket.sendto(b'u', (self.SERVER_IP, self.SERVER_PORT))
    def throtleDown(self):
        self.server_socket.sendto(b'j', (self.SERVER_IP, self.SERVER_PORT))
    def yawLeft(self):
        self.server_socket.sendto(b'h', (self.SERVER_IP, self.SERVER_PORT))
    def yawRight(self):
        self.server_socket.sendto(b'k', (self.SERVER_IP, self.SERVER_PORT))

    def forward(self):
        self.server_socket.sendto(b'w', (self.SERVER_IP, self.SERVER_PORT))
    def backward(self):
        self.server_socket.sendto(b's', (self.SERVER_IP, self.SERVER_PORT))
    def moveLeft(self):
        self.server_socket.sendto(b'a', (self.SERVER_IP, self.SERVER_PORT))
    def moveRight(self):
        self.server_socket.sendto(b'd', (self.SERVER_IP, self.SERVER_PORT))

    def enableStream(self, state):
        if state:
            self.server_socket.sendto(b'1-on', (self.SERVER_IP, self.SERVER_PORT))
        else:
            self.server_socket.sendto(b'1-off', (self.SERVER_IP, self.SERVER_PORT))


    def enableAutoLanding(self, state):
        if state:
            self.server_socket.sendto(b'2-on', (self.SERVER_IP, self.SERVER_PORT))
        else:
            self.server_socket.sendto(b'2-off', (self.SERVER_IP, self.SERVER_PORT))

    # def eventFilter(self, target, event):
    #     print(event.type())       

class MyWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MyWindow, self).__init__(*args, **kwargs)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

    def closeEvent(self, event):
        self.ui.close()
        event.accept()

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    # MainWindow = QtWidgets.QMainWindow()
    MainWindow = MyWindow()
    # ui = Ui_MainWindow()
    # ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

