#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from subprocess import *
from std_msgs.msg import String

def node_state(message):
    if message.data:
        p = Popen(["rosrun","turtlesim","turtlesim_node"])
    else:
        kill_node("turtlesim")

def kill_node(nodename):
    p2=Popen(["rosnode","list"],stdout=PIPE)
    p2.wait()
    nodelist=p2.communicate()
    nd=nodelist[0]
    nd=nd.split("\n")
    for i in range(len(nd)):
		tmp=nd[i]
		ind=tmp.find(nodename)
		if ind==1:
			call(["rosnode","kill",nd[i]])
			break


rospy.init_node("start_node")
sub = rospy.Subscriber("node_bool", Bool, node_state)
rospy.spin()
