# ME5413_Final_Project

## 1 What we have

- NDT localisation
- Smooth nav. parameter adjustment adapted from KK Sima
- Multi planner and controller switch



## 2 How to use

### 2.1 NDT localisation

First run,
```
roslaunch hdl_localization hdl_localization.launch
```
then run world.launch

```
roslaunch me5413_world world.launch

```

and

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
or play a rosbag

```
rosbag play /xx/xx.bag --clock
```

Note:

- If you would like to disable or adjust the topic name of imu, please directly modify `src/hdl_localization/hdl_localization.launch`.



---
Details please refer to https://me5413-g6-2025.atlassian.net/wiki/spaces/FINAL/pages/12386334/NDT+Localisation.

### 2.2 Nav. param adjustment

please refer to https://me5413-g6-2025.atlassian.net/wiki/spaces/FINAL/pages/15958018/Nav+kk+sima.


### 2.3 Planner and Controller Switch


Step1, launch `world.launch`
```
roslaunch me5413_world world.launch

```

Step2, launch

```
source {YOUR_DIR}/ME5413_Final_Project/src/third_party/ros_motion_planning/devel/setup.bash
roslaunch jackal_navigation navigation.launch
```

Note:

- The operation `source` is to make sure all ros packages realted to planner and controller can be reached.
- All args or parameters can be adjusted are all in package folder `src/jackal_navigation/`.
    - To switch a different planner or controller, please modify arg `global_planner` and `local_planner` in `./launch/navigation.launch`.
    - To adjust the general parameters w.r.t. all planner algo., check `./config/planner`.
    - To adjust the related parameters w.r.t. the specific controller algo., check `./config/controller`.


