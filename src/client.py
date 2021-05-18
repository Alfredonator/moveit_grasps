#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from moveit_grasps.srv import *

def add_two_ints_client(name):
    print("waiting for the service to be public")
    rospy.wait_for_service('grasp_pipeline')
    print("service got public, now calling a RPC")
    try:
        grasp_pipeline_fn = rospy.ServiceProxy('grasp_pipeline', Grasp)
        resp1 = grasp_pipeline_fn(name)
        return resp1.grasp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print(add_two_ints_client("mf"))
