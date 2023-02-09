#!/usr/bin/python

import rospy
import math
from mdek_uwb_driver.msg import Uwb
from tf import TransformListener
from geometry_msgs.msg import PointStamped


rospy.init_node('range_check')

leicaPoint=None
globalFrame=None
fileDict={}
listener=TransformListener()

def leicaCb(msg):
    global leicaPoint
    global globalFrame
    leicaPoint = msg
    # print(leicaPoint)
    globalFrame = msg.header.frame_id

def uwbCb(msg):
    global leicaPoint
    global globalFrame
    global listener
    global fileDict

    if leicaPoint is None:
        return

    for i in range(len(msg.ranges)):
        try:
            anchor = msg.ranges[i].id
            (x,y,z),q = listener.lookupTransform(globalFrame,anchor,rospy.Time(0))
            # print("%s,%f,%f,%f"%(anchor,x,y,z))
            x -= leicaPoint.point.x
            y -= leicaPoint.point.y
            z -= leicaPoint.point.z
            if anchor not in fileDict:
                fileDict[anchor] = open(anchor+".csv","w")
                fileDict[anchor].write("%ground truth, measure\n")
                print("Created log file for "+anchor)
            # print("%s,%f,%f"%(anchor,math.sqrt(x*x+y*y+z*z),msg.ranges[i].distance))
            fileDict[anchor].write("%f,%f\n"%(math.sqrt(x*x+y*y+z*z),msg.ranges[i].distance))
            fileDict[anchor].flush()
        except :
            pass

rospy.sleep(0.5)

psub=rospy.Subscriber("/leica/position_yx",PointStamped,leicaCb,queue_size=1)
usub=rospy.Subscriber("/uwb_driver_node/uwb_remapped",Uwb,uwbCb,queue_size=1)

rospy.loginfo("Device remapper running")
rospy.spin()

for f in fileDict:
    fileDict[f].close()

