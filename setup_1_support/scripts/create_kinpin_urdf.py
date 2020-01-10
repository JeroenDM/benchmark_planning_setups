#!/usr/bin/env python
"""
Generate an urdf file for the kingpin welding task.
In addition the task description file kinping.irl is generated.

First all data is put into a Python dictionary.
Then this dict is used to create the files based on a template.
"""
import numpy as np
from string import Template

# link names
names = ["u_profile", "obstacle", "table", "side_1", "side_2", "u_profile_2"]

# dimensions
obs_h = 0.01
obs_off = 0.04
L = 0.5  # length of all stuff in y-direction
obs_l = L / 6
u_off = 0.005

wobj_name = "kingpin"
wobj_location = [0.85, 0, 0]


def list_to_string(lst):
    """ Convert [1.0, 2, 0] ==> '1.0 2 0'  """
    return " ".join([str(v) for v in lst])


def create_task(location):
    """ Create dict containing desired tool position and orientation for the task. """
    sizes = [
        [0.27, L, 0.21],
        [0.2, obs_l, obs_h],
        [0.7, 0.7, 0.1],
        [0.2, 0.1, 0.055],
        [0.2, 0.1, 0.055],
    ]

    # path position
    location = np.array(location)
    path_start = np.array([0, -L / 2 + sizes[3][1] + u_off, 0]) + location
    path_stop = np.array([0, L / 2 - sizes[3][1] - u_off, 0]) + location

    task = {}
    task["path_start_xyz"] = list_to_string(path_start)
    task["path_stop_xyz"] = list_to_string(path_stop)

    # tool orientation is hardcoded for now
    task["path_start_xyzw"] = "0.65328148 0.65328148 0.27059805 0.27059805"
    task["path_stop_xyzw"] = "0.65328148 0.65328148 0.27059805 0.27059805"
    return task


def create_urdf_data(location):
    """ Create dict containing the dimensions for the links and joints in the urdf.

    See the global variable 'names' for what dimensions go with wich link.
    """

    sizes = [
        [0.2, L, 0.1],
        [0.2, obs_l, obs_h],
        [0.7, 0.7, 0.01],
        [0.3, 0.1, 0.055],
        [0.3, 0.1, 0.055],
        [0.2, L, 0.1],
    ]
    positions = [
        [sizes[2][0]/2 - sizes[0][0] / 2, 0, sizes[0][2] / 2],
        [0, 0, obs_h / 2],
        [0, 0, -0.005],
        [0, L / 2 - sizes[3][1] / 2, sizes[3][2] / 2],
        [0, -L / 2 + sizes[3][1] / 2, sizes[3][2] / 2],
        [-sizes[2][0]/2 + sizes[5][0] / 2, 0, sizes[5][2] / 2],
    ]

    # move to the absolute location given as input
    positions = [[r + p for r, p in zip(pos, location)] for pos in positions]

    boxes = []
    for name, size, position in zip(names, sizes, positions):
        new_box = {}
        new_box["name"] = name
        new_box["size"] = list_to_string(size)
        new_box["xyz"] = list_to_string(position)
        boxes.append(new_box)

    return boxes


def read_templates():
    with open("templates/box.template.xml") as file:
        box_template = Template(file.read())
    with open("templates/joint.template.xml") as file:
        joint_template = Template(file.read())
    with open("templates/workobject.template.urdf") as file:
        urdf_template = Template(file.read())
    with open("templates/task.template.txt") as file:
        task_template = Template(file.read())
    return box_template, joint_template, urdf_template, task_template


if __name__ == "__main__":
    """ Create 'kingpin.urdf' and 'kingpin.irl'. """

    box_temp, joint_temp, urdf_temp, task_temp = read_templates()

    urdf_data = create_urdf_data(wobj_location)

    content = "\n"
    for box in urdf_data:
        content += box_temp.substitute(box) + "\n"
        content += joint_temp.substitute(box) + "\n"

    with open("{}.urdf".format(wobj_name), "w") as file:
        file.write(
            urdf_temp.substitute({
                "name": wobj_name,
                "content": content
            }))

    # with open("{}.irl".format(wobj_name), "w") as file:
    #     file.write(task_temp.substitute(create_task(wobj_location)))
