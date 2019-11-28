# benchmark_planning_setups
[ROS](https://www.ros.org/) support packages and [MoveIt](https://moveit.ros.org/) configurations for motion planning benchmarking.

## Work objects

By default no work object is loaded in the scene.
You can specify one as a command line argument:
```bash
roslaunch setup_1_support display.launch work:=l_profile
```
for visualization and
```bash
roslaunch setup_1_moveit_config demo.launch work:=l_profile
```
for the full planning context.

There are 4 objects at the moment:
- **nothing**: an empty urdf. (But with a work link, because this is part of the fixed interface.)
- **boxes**: 3 boxes in between which the end-effector can try to move.
- **l_profile**: a simple L-shaped object where you can weld the corner.
- **kingpin**: a slightly more complex part based on a real world example.

The task specification is also specified in the `setup_1_support/tasks/` folder as 'irl' files.
(Industrial robot language, a made up language to specify robot tasks.)
The poses defined in these files are automatically published by the launch files mentioned, using the `setup_1_support/scripts/publish_work_poses.py` script.
