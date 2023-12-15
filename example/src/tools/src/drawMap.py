#!/usr/bin/python3

import threading
import signal
import sys

from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import Qt, QTimer

import rospy
from nav_msgs.msg import OccupancyGrid

# CONSTANTS
MAP_TOPIC = "map"
WINDOW_WIDTH = 1080
WINDOW_HEIGHT = 720

FREQUENCY = 1
DELAY = 1000 / FREQUENCY

#GLOBAL VARS
mapData: OccupancyGrid
mapDataReady = False
running = True

def stop(arg1, arg2):
    global running
    running = False

def MapTopicCallback(msg: OccupancyGrid):
    global mapData, mapDataReady
    mapData = msg
    mapDataReady = True

def drawMap(label: QtWidgets.QLabel):
    # WE ONLY WANT TO DRAW IN CASE WE HAVE NEW MAP RECEIVED
    global mapData, mapDataReady
    if not mapDataReady: return
    width  = WINDOW_WIDTH / mapData.info.width
    height = WINDOW_HEIGHT / mapData.info.height
    painter = QtGui.QPainter(label.pixmap())
    for i in range(mapData.info.width):
        for j in range(mapData.info.height):
            value = mapData.data[j * mapData.info.width + i]
            color = Qt.black if value > 0 else Qt.white
            painter.fillRect(i * width, j * height, width, height, color)
    label.update()
    mapDataReady = False

if __name__ == "__main__":
    rospy.init_node("MAP_DRAWING_TOOL")
    mapSub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, MapTopicCallback)

    threading.Thread(target = lambda : rospy.spin(), daemon=True)

    signal.signal(signal.SIGINT, stop)

    app = QtWidgets.QApplication(sys.argv)
    window = QtWidgets.QMainWindow()
    
    label = QtWidgets.QLabel()
    canvas = QtGui.QPixmap(WINDOW_WIDTH, WINDOW_HEIGHT)
    canvas.fill(Qt.white)
    label.setPixmap(canvas)
    window.setCentralWidget(label)

    timer = QTimer(window)
    timer.timeout.connect(lambda : drawMap(label))
    timer.start(int(DELAY))

    window.show()
    app.exec_()
        