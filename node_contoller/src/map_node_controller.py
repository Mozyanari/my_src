#!/usr/bin/env python
import rospy
import os
import rosnode

from std_msgs.msg import Bool
from subprocess import *
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse


def node_state(message):
    if message.data:
        username = os.environ.get("USER")
        map_path = "/home/" + username + "/maps_temp/map.yaml"
        p = Popen(["rosrun","map_server","map_server","__name:=map_server", map_path])
        return SetBoolResponse(True,"enable")
    else:
        if kill_node("map_server"):
            return SetBoolResponse(True,"disable")
        else :
            return SetBoolResponse(False,"disable")


def kill_node(nodename):
    isNodeAlive=rosnode.rosnode_ping(nodename,max_count=1,verbose=False)
    if isNodeAlive:
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
                return True
    else :
        return False

rospy.init_node("map_node_controller")
service = rospy.Service("map_server_enable",SetBool,node_state)
rospy.spin()
