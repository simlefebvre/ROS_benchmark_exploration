#!/usr/bin/env python

from math import sqrt
from time import sleep

from attr import has
import rospy
from geometry_msgs.msg import Twist,Vector3,Pose
from nav_msgs.msg import Odometry,OccupancyGrid

import time

dist = None

prevX = 0
prevY = 0
prevZ = 0

X = 0
Y = 0
Z = 0

prevMap = []
resolution = 0

somme = 0
surfaceTotal = 0

testInit = True

hasNewSurface = False

def init():
    global dist
    dist = rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/map",OccupancyGrid,callback_map)
    rospy.init_node('distance')


def callback_map(data):
    global prevMap,hasNewSurface,resolution,surfaceTotal,testInit
    if testInit:
        for i in data.data:
            prevMap.append(-1)
        testInit = False
    map = data.data
    mesureSurface(map)
    hasNewSurface = True
    rospy.loginfo(surfaceTotal)

    #prevMap = map
    resolution = data.info.resolution

def callback_odom(data):
    global prevX, prevY, prevZ, X, Y, Z, somme
    prevX = X
    prevY = Y
    prevZ = Z
    

    X = data.pose.pose.position.x
    Y = data.pose.pose.position.y
    Z = data.pose.pose.position.z

    somme = somme+sqrt((X-prevX)**2+(Y-prevY)**2+(Z-prevZ)**2)

def mesureSurface(map : list):
    global surfaceTotal
    surface = 0
    for i in range(len(map)):
            if map[i] != -1:
                surface = surface + resolution**2
    surfaceTotal = surfaceTotal + surface
    

if __name__ == '__main__':
    nbSecFreq = 1
    init()
    with open("/home/simon/Documents/RO51/mini-project/mesure/src/Mesure/Data/distance.csv", "w") as f, open("/home/simon/Documents/RO51/mini-project/mesure/src/Mesure/Data/surface.csv", "w") as f2:
        while not rospy.is_shutdown():
            f.write(f"{rospy.Time.now().to_sec()},{somme}\n")
            if hasNewSurface:
                f2.write(f"{rospy.Time.now().to_sec()},{surfaceTotal}\n")
                hasNewSurface = False
            surfaceTotal = 0
            sleep(nbSecFreq)
