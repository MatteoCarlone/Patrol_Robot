# Patrol_Robot 

**A ROS-based package that simulate a surveillance robot in a gazebo environment by exploiting Aruco markers detection , MoveIt , SLAM Gmapping algorithm and the Move_Base Navigation package.**

The software architecture at the base of this package is explained in this [respository](https://github.com/MatteoCarlone/Assignment1-ExproLab). 

*Full Code Documentation available*  [**|HERE|**]( https://matteocarlone.github.io/Patrol_Robot/)

---

## Introduction

This project is the second assignment of the Robotics Engineering Course's Experimental Robotics Laboratory at the University of Genoa.
The task entails developing a software architecture and simulation for surveillance robots. 
The project aims to deepen the ROS (Robot-Operating-System) utilization by leveraging:

- [***ARUCO marker detection***](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html), black and white square patterned markers that can be detected in images and video. They are often used for pose estimation, tracking, and other computer vision tasks.

- [***MoveIt***](https://moveit.ros.org/), a software framework for motion planning, manipulation, and visualization of robotic systems. It provides a set of tools and libraries for generating, simulating, and executing complex robot behaviors in a variety of environments. MoveIt is often used in robotics research and development, as well as in industrial automation applications..

- [***SLAM Gmapping Algorithm***](http://wiki.ros.org/gmapping), (Simultaneous Localization and Mapping) is an algorithm that allows a robot to build a map of its environment and determine its own location within that map in real time. The Gmapping algorithm is a specific SLAM algorithm that uses laser rangefinder data to build the map.

- [***Move Base Navigation PkG***](http://wiki.ros.org/move_base), ROS package that provides a simple interface for sending navigation goals to a robot and controlling its movement in the environment. It is part of the ROS Navigation Stack and is used to plan and execute robot trajectories to reach desired goals.

## Project Scenario
The patrolling robot is designed to firstly operate autonomously in a detached room.
**|First Phase|** Upon being deployed, the robot uses its Aruco marker detection capabilities to gather information about the environment and determine the areas it should patrol.
With this information in hand, the robot begins its patrolling routine **|Second Phase|**, moving through the designated areas and using its sensors to detect any potential threats or abnormalities.
The robot is programmed to continue this routine until it is manually deactivated.

### The Robot
After careful consideration, I decided to use the [*Turtlebot3* ( Waffle series )](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robot because it already had all the features I needed.
One of the main reasons I chose this model was because of its laser scanner, which would be crucial for navigation and obstacle avoidance.  
To give the robot even more capabilities, I also decided to mount the real turtlebot arm on it. MoreOver On top of the arm there's an [*Orberc Astra* RGBd camera](https://shop.orbbec3d.com/Astra).  This would allow the robot to perform tasks such as orient the camera by creating complex movement of the arm.
Overall, the combination of the turtlebot3's existing features and the added turtlebot arm  comprehensive of the came made it the perfect choice for the patrolling project.

*the robot turns out to be:*
<p align="center">
<img src="https://user-images.githubusercontent.com/81308076/211048300-5561fde5-e477-427f-b16c-a8e362f97bd8.png"  alt="drawing" width="200"/>
</p>

### The Environment
The Environment is exactly the same of the of this [repo]() in which I tested the saftware architecture at the base of the simulation, with the only different that now is a 3-D space with structural obstacles that the robot will have to avoid and take into account while moving.
In addition to this, there's a separated room in which the robot spawn and start a marker detection to retrive all the necessary environment indormations.
ALl the simulation world description is located in the [world folder]() as the assignment_world.world file.
Since the Mrker detection was quite difficult in the default environment I decided to change it removing some shaders and background artifacts to let the robot better detect.

### Mission Phases 

**First Phase**

During the first mission phase the robot is capable of detecting 7 Aruco markers placed in different positions without moving from its spawn position.
To achieve the result the arm is moved using the MoveIt package to orient the camera toward the markers in a fast and robust manner. The my_moveit node in the src folder 
implements the interface between the program and moving allowing to request a predefined motions of the arm. A MoveIt package has been created for this specific robot and arm, stored
in this [repository](https://github.com/MatteoCarlone/MoveIt_PkG_Turtlebot3_Waffle-Pi-manipulation).
The aruco marker detection is managed by the aruco_detector node of the package [aruco_ros]() it trivially needs to subscribe to the Astra camera topic: /camera/rgb/image_raw  .
from the aruco markers the robot retrive just codes that will then be traslated in informations about the environment thanks to the marker_server node in the src folder.
this passage link this project with the architecture of the first assignment, since all the necessary data to build the ontology of the environment to be patrolled are the ones get from the marker_server
via the RoomInformation service.

**Second Phase**

Once the Ontology has been loaded the patrolling routine starts.

From now on, The motion of the robot follows two main protocols: 
one for the Mapping of the unkonwn environment and one for the actual Navigation from a starting point (Room A) to a goal one (Room B).

- **Mapping**

  I used a *Filtering-Based* approach called **Gmapping (FastSLAM) .**

  This approach uses a particle filter (Rao-Blackwellized particle filters) in which each particle carries an individual map of the environment.

  More info: [rbpf-trans.dvi |uni-freiburg.de|](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf)

  To use the package in ros the robot must provide:
  - odometry data
  - laser range finder data.

  The gmapping package *subscribes to the topics*:

  - `/scan` (where it receives the laser scan to create the map from)
  - `/tf` (transforms necessary to relate frames for base and odometry)
  
  The gmapping package *publishes to the topics*

  - `map`, published as a `nav_msgs/OccupancyGrid`
    The message contains the map data as a *int8[]* data. Data are expressed in
    row-major order. Occupancy probabilities are in the range [0, 100]. Unknown
    is -1.
  - `map_metadat`a, as a `nav_msgs/MapMetaData`
    This topic contains basic information about the characteristics of the
    Occupancy Grid, for example the time at which the map was loaded, the
    map resolution, its width and height, the origin of the map

  I set many parameters when launching the node, but it's higly recommended to change them carefully since the algorithm is really sensitive to parameters.

  - more infos here: [gmapping - ROS Wiki](http://wiki.ros.org/gmapping)
  
- **Navigation**

  I used the **MoveBase** package, of the *ROS
  Navigation stack* .

  It allows for selecting a **global** and a **local planner**:

  - **Global Path Planning**

      I used the **navfn global planner:**

      - uses Dijkstra’s algorithm to find a global path with minimum
      cost between start point and end point.
      
  - **Local Path Planning**

      I used the **dwa local planner** , which works as follows:

      1. Discretely sample in the robot’s control space (dx,dy,dtheta)
        
      2. For each sampled velocity, perform forward simulation from
        the robot’s current state to predict what would happen if the
        sampled velocity were applied for some (short) period of time.
      
      3. Evaluate (score) each trajectory resulting from the forward
        simulation, using a metric that incorporates characteristics such
        as: proximity to obstacles, proximity to the goal, proximity to the
        global path, and speed. Discard illegal trajectories (those that
        collide with obstacles).
        
      4. Pick the highest-scoring trajectory and send the associated
        velocity to the mobile base.
      
      5. Apply the velocity and repeat.

  The DWA planner depends on the local costmap which provides obstacle information. Therefore, tuning the parameters for the local costmap is crucial for optimal behavior of DWA local planner
  All the parameters for both global and local planning are in the [param folder]().
  
## Diagrams

The whole architecture is well described in the following UML Diagrams.

<p align="center">
<img src="https://user-images.githubusercontent.com/81308076/211151491-ae925c43-342f-4479-9b4a-58ec1a3561e3.PNG"  alt="drawing" width="700" />
</p>

`caption` :

	- Time Squence Diagram of the project, with a draft of communication transitions between states in time.
	  To be more precise in this diagram is visible the hierachical sub-state machine structure, with 
	  the moving sub-machine called in two different instances both in the general movement and in the move
	  to recharge dock action. Moreover you can see:
	  1. the first mission phase with aruco detection and arm motion
	  2. the second phase with the Navigation and Mapping


## Launching the Software

This software has been based on ROS Noetic, and it has been developed with this Docker-based
[environment](https://hub.docker.com/repository/docker/carms84/exproblab), which already 
provides all the required dependencies. 

### Installation

Follow these steps to install the software.
 - Clone this repository inside your ROS workspace (which should be sourced in your `.bashrc`).
 - Clone this [MoveIt repository](https://github.com/MatteoCarlone/MoveIt_PkG_Turtlebot3_Waffle-Pi-manipulation) inside your ROS workspace (which should be sourced in your `.bashrc`).
 - Run `chmod +x <file_name>` for each file inside the `scripts` folder.
 - Install `xterm` by entering the command `sudo apt install -y xterm`.
 - Install `smach` by entering the command `sudo apt-get install ros-noetic-smach-ros`
 - Install `aRMOR` directly from [here](https://github.com/EmaroLab/ros_multi_ontology_references.git). or manually download the latest release of each module from the following repositories:
	- [***ARMOR***](https://github.com/EmaroLab/armor)
	+ [***AMOR***](https://github.com/EmaroLab/multi_ontology_reference)
	+ [***armor_msgs***](https://github.com/EmaroLab/armor_msgs)

 - Clone the [`armor_api`](https://github.com/buoncubi/armor_py_api) repository in your workspace

 Note: [Here](http://emarolab.github.io/armor_py_api/) you can find the full documentation of the [`armor_api`](https://github.com/buoncubi/armor_py_api).
 
 - Install MoveIt, Look [Here](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
 - Clone this [repo](https://github.com/CarmineD8/aruco_ros) for aruco detection and add it to your workspace
 - Clone this [repo](https://github.com/CarmineD8/SLAM_packages) for the SLAM gmapping algorithm and add it to your workspace
 - Clone this [repo]( https://github.com/CarmineD8/planning) for the Move Base Navigation pkg and add it to your workspace
 - Cross your fingers and Run `catkin_make` from the root of your ROS workspace.
 
### Launchers

Use the following command to launch the software a spawn:
- The Gazebo environment with the robot in the separate room.
- Rviz already set up to visualize the map created by the Gmapping algorithm, the Move Base global and local cost map, the laser output and the robot model.
- some xterm windows related to all the software components.

```bash
roslaunch patrol_robot demo_assignment.launch
```

Note that the program runs in automatically in a loop and there's no need of the user to start up the finite state machine's transitions.

Check the `roslaunch` outcome to get the path where logs are stored. usually, it is `~/.ros/log/`.
That folder should also contain a link to the `latest` produced log.

*Expected Behaviour:*


<p align="center">
<img src="https://user-images.githubusercontent.com/81308076/211152260-6c02ed5b-c8d0-45b2-a932-35f3cbf5b822.png"  alt="drawing" width="700" />
</p>



### System's limitations
The main limitation of this project relates to the pre-developed software architecture, as it assumed a 2D environment without obstacles and without considering the actual movement of the robot, which could be subject to unforeseen delays and interruptions. Overall this limitation affects only the robot recharge: 
The robot recharge when the ontology knows it's in the starting location so there could be the possibility that the robot start recharging when moving from the starting location to a target one. This behaviour is coherent with the pre-defined architecture, but not consistent with a realistic simulation.
Two other limitations lie in the Marker detection.
1. The environment has been changed for a robust detection of the markers
2. the arm orient the camera toward the target in an hard coded manner to fast up the operation, Initially I did a round detection at various altitudes but for the sake of faster testing I decided to hard code 5 arm positions.

### Possible Improvements

- adjust the recharge routine to make it more realistic 
- detect markers in a more natural environment by tuning the camera and the detection parameters
- orient the camera to the aruco markers by calibrating its position in the environment
- realize my first mapping idea:
  
  I wanted to generate a map of the environment using the KartoSlam algorithm ,  and explore-lite which is a software package that provides tools for exploring and       mapping unknown environments with a mobile robot. It includes algorithms for autonomous exploration.
  here an image of the process:
  
  <p align="center">
	<img src="https://user-images.githubusercontent.com/81308076/211152417-1950f5a9-a732-45f5-903e-53128231d738.png"  alt="drawing" width="700" />
  </p>
  

  Then save the map and load it whenever you launch the program.
  By doing this the robot would need an algorithm to localize itself in the environment ( for instance I used the AMCL algorithm ) while moving with the aformentioned   move base package.
  I actually tried this method but there were many incoherences between the AMCL localization algorithm and the pre-loaded map.

--------

**Authors and Contacts**

Author: *Matteo Carlone s4652067*

to email me:  
<a href="mailto:matteo.carlone99@gmail.com" >
<img align="left" alt="Matte's mail" width="40px" src="https://user-images.githubusercontent.com/81308076/155858753-ef1238f1-5887-4e4d-9ac2-2b0bb82836e2.png" />
</a>
