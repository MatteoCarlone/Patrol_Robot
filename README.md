# Patrol_Robot 

**A ROS-based package that simulate a surveillance robot in a gazebo environment by exploiting Aruco markers detection , MoveIt , SLAM Gmapping algorithm and the Move_Base Navigation package.**

The software architecture at the base of this package is explained in this [respository](https://github.com/MatteoCarlone/Assignment1-ExproLab). 

*Full Code Documentation available*  [**|HERE|**](https://matteocarlone.github.io/Assignment1-ExproLab/)

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

### Mission Phases 

**First Phase**

During the first mission phase the robot is capable of detecting 7 Aruco markers placed in different positions without moving from its spawn position.
To achieve the result the arm is moved using the MoveIt package to orient the camera toward the markers in a fast and robust manner. The my_moveit node in the src folder 
implements the interface between the program and moving allowing to request a predefined motions of the arm. A MoveIt package has been created for this specific robot and arm, stored
in this [repository]().
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

## Launching the Software

### Installation

### Launchers

### System's limitations

### Possible Improvements

--------

**Authors and Contacts**

Author: *Matteo Carlone s4652067*

to email me:  
<a href="mailto:matteo.carlone99@gmail.com" >
<img align="left" alt="Matte's mail" width="40px" src="https://user-images.githubusercontent.com/81308076/155858753-ef1238f1-5887-4e4d-9ac2-2b0bb82836e2.png" />
</a>
