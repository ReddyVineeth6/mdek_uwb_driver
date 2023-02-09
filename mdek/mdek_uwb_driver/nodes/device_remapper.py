#!/usr/bin/python

import rospy
from mdek_uwb_driver.msg import Uwb


rospy.init_node('device_remapper')

pub=rospy.Publisher("uwb_out",Uwb,queue_size=1)

id_to_name = {
    '0x9818':'Harry',
    '0x5087':'Fred',
    '0xb08': 'George',
    '0x0b08': 'George',
    '0x1382': 'Hermione',
    '0x0538': 'Dumbledore',
    '0x538': 'Dumbledore',
    '0x5184':'Ron',
    '0x8732': 'Snape',
    '0x821d': 'Ginny',
    '0x1d08': 'Luna',
    '0x53b': 'Minerva',
    '0x053b': 'Minerva',
    '0x1a3b': 'Dobby'
}

def callback(msg):
    global pub
    global id_to_name
    for i in range(len(msg.ranges)):
        try:
            msg.ranges[i].id = id_to_name[msg.ranges[i].id]
        except KeyError:
            pass
    pub.publish(msg)

sub=rospy.Subscriber("uwb_in",Uwb,callback,queue_size=1)
rospy.loginfo("Device remapper running")
rospy.spin()

