#!/usr/bin/env python

import sys
import rospy
import rospkg
import moveit_commander
import geometry_msgs.msg
import urdfpy

from geometry_msgs.msg import Transform, Vector3, Quaternion, Point, Pose

REL_WORK_PATH = "/urdf/work/"


def numpy_to_pose(arr):
    """ Numpy 4x4 array to geometry_msg.Pose
    code from: https://github.com/eric-wieser/ros_numpy
    """
    from tf import transformations
    assert arr.shape == (4, 4)

    trans = transformations.translation_from_matrix(arr)
    quat = transformations.quaternion_from_matrix(arr)

    return Pose(
        position=Vector3(*trans),
        orientation=Quaternion(*quat)
    )


def remove_all_objects(scene):
    for name in scene.get_known_object_names():
        scene.remove_world_object(name)
    rospy.sleep(0.5)


def parse_file(work_name):
    rospack = rospkg.RosPack()

    filepath = rospack.get_path("setup_1_support")
    filepath += REL_WORK_PATH
    filepath += work_name + ".urdf"
    print(filepath)

    return urdfpy.URDF.load(filepath)


def parse_link(link):
    """ Assume a link has only a single collision object.
        Assume this collision object is a box.
        Assume the link named "work" has no collision objects.
    """
    assert len(link.collisions) == 1
    assert link.collisions[0].geometry.box is not None
    assert link.name != "work"
    return link.collisions[0].geometry.box.size


def publish_parsed_urdf(parsed_urdf, scene):
    links = {}
    for link in parsed_urdf.links:
        if link.name == "work" or link.name == "world":
            continue
        else:
            links[link.name] = {"size": parse_link(link)}
    print(links)
    for joint in parsed_urdf.joints:
        print("==============", joint.name)
        print(joint.origin.shape)
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = joint.parent
        p.pose = numpy_to_pose(joint.origin)
        scene.add_box(joint.child, p, links[joint.child]["size"])


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: publish_work.py <work_name>")
        exit()
    else:
        work_name = sys.argv[1]

    rospy.init_node("publish_work")

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)  # wait for the above things to setup

    remove_all_objects(scene)

    work = parse_file(work_name)
    publish_parsed_urdf(work, scene)

    # scene = moveit_commander.PlanningSceneInterface()
    # robot = moveit_commander.RobotCommander()

    # rospy.sleep(1.0)  # wait for the above things to setup

    # work_pose_1 = geometry_msgs.msg.PoseStamped()
    # work_pose_1.header.frame_id = "/work"
    # work_pose_1.pose.position.x = 0.1
    # work_pose_1.pose.position.y = 0.5
    # work_pose_1.pose.position.z = 0.01
    # work_pose_1.pose.orientation.w = 1.0

    # scene.add_box("l_base", work_pose_1, size=(0.2, 1, 0.02))

    # work_pose_2 = geometry_msgs.msg.PoseStamped()
    # work_pose_2.header.frame_id = "/l_base"
    # work_pose_2.pose.position.x = 0.09
    # work_pose_2.pose.position.y = 0
    # work_pose_2.pose.position.z = 0.11
    # work_pose_2.pose.orientation.w = 1.0

    # scene.add_box("l_side", work_pose_2, size=(0.02, 1, 0.2))

    print("Done!")
