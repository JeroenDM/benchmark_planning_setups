#!/usr/bin/env python
"""
TODO move this to a setup_util package ?
"""
from __future__ import print_function

import geometry_msgs.msg
import rospkg
import rospy
import sys

from nexon.interface import Sections
from nexon.io import parse_file
from nexon.util import Plotter


def create_pose_msg(goal):
    xyz, xyzw = goal["xyz"], goal["xyzw"]
    p = geometry_msgs.msg.Pose()
    p.position.x = xyz[0]
    p.position.y = xyz[1]
    p.position.z = xyz[2]
    p.orientation.x = xyzw[0]
    p.orientation.y = xyzw[1]
    p.orientation.z = xyzw[2]
    p.orientation.w = xyzw[3]
    return p


def show_task(plotter, task):
    variables = task[Sections.VARS]
    for v in variables:
        #print(v, variables[v])
        try:
            plotter.plot_axis(create_pose_msg(variables[v]))
        except:
            continue


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: publish_work_poses.py <work_name>.irl")
        exit()
    else:
        work = sys.argv[1]

    print(work)

    rospy.init_node("publish_work_poses")
    rospack = rospkg.RosPack()

    filepath = rospack.get_path("setup_1_support") + \
        "/tasks/" + work

    task = parse_file(filepath)
    show_task(Plotter(wait_time=5.0), task)
