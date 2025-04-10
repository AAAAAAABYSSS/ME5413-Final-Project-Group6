# ME5413_Final_Project

## 1 What we have

1. Feature

- Mapping 
    - [x] Cartographer
    - [x] Fast-Livo2
- Localisation
    - [x] AMCL
    - [x] NDT
- Global planner switch
    - [x] dijkstra
    - [x] dstar
    - [x]voronoi
    - [x] theta_star
    - [x] astar  
- Local planner switch
    - [x] TEB
    - [x] PID
    - Note: easily change arg `local_planner` in `src/me5413_navigation/launch/navigation_new.launch`.


2. Structure

## Structure

```shell
src/
├── interactive_tools    # Interactive tools for rviz
├── jackal_description   # Modified Jackal model package
├── me5413_control       # Modified Jackal control package
├── me5413_evaluation    # SLAM evo evaluation and box identification evaluation
├── me5413_mapping       # Mapping package
├── me5413_navigation    # Navigation package
├── me5413_perception    # Perception package
├── me5413_thirdparty    # Third-party packages. We have modified some of them.
└── me5413_world         # Gazebo simulation package
```


## 2 How to use

1. Easily run

```
cd ME5413-Final-Project-Group6
bash scripts/start.sh

```

2. Go back to the original window you enter the above command, choose the mode you want

```
0: Launch world
1: Mapping Mode
2: Navigation Mode
3: Perception Mode

```

### 2.1 Mapping and SLAM



### 2.2 Localisation



### 2.3 Planning and Control

1. To switch global planner, please switch arg `global_planner` in `src/me5413_navigation/launch/launch_new.launch`

2. To switch local planner, please switch arg `local_planner` in `src/me5413_navigation/launch/launch_new.launch`

```
  <include file="$(find me5413_evaluation)/launch/evo_evaluation.launch" />
    <arg name="global_planner" default="dstar" />
    <!-- local planner name -->
    <arg name="local_planner" default="teb" />
  </include>
```
### 2.4 Evaluation




## 3 Acknowledgement

We would like to thank the following open-source projects:

- [ME5413_Final_Project](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project)
- [Fast-Livo2](https://github.com/hku-mars/FAST-LIVO2)
- [ros_motion_planning](https://github.com/ai-winter/ros_motion_planning)
- [Autonomous Robot Navigation](https://github.com/brian00715/Autonomous-Robot-Navigation)

## 4 One more thing

If you have any question, please contact Yvelle@Github.