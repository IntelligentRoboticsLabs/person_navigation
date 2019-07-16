# Person Navigation

## Packages
None.

## Summary

- **Maintainer:** Francisco Martín <fmrico@gmail.com>
- **Author:** Francisco Martín <fmrico@gmail.com>
- **License:** BSD
- **ROS Distro:** Kinetic
- **Dependencies:**
  - **ROS Standard:** roscpp, rospy, tf, geometry_msgs, std_msgs, sensor_msgs, nav_msgs, visualization_msgs, move_base_msgs, actionlib, actionlib_msgs, cv_bridge, costmap_2d, image_transport, roslint, naoqi_bridge_msgs
  - **Internals:**
     - bica_core: https://gitlab.com/fmrico/bica.git
     - ir_planning: https://gitlab.com/Intelligent-Robotics/ir_planning.git
     - ROSPLan: https://gitlab.com/Intelligent-Robotics/ROSPlan.git
     - pepper_navigation_bringup: https://gitlab.com/Intelligent-Robotics/pepper_navigation_bringup.git
     - pepper_basic_capabilities: https://gitlab.com/Intelligent-Robotics/pepper_basic_capabilities.git
     - rosweb_clientserver: https://gitlab.com/Intelligent-Robotics/rosweb_clientserver.git
     - topological_navigation: https://gitlab.com/fmrico/topological_navigation.git

## Description

This repository contains a node in charge of the navigation while the robot tries to follow person.
The steps to follow a person are:
- Get the list with the people that are in the room and the program `person_follower_node` only uses the selected goal. To know which is the distance, the list contains the TFs.
- Create a costmap with the person as the goal and the robot as the origin.
- To add the obstacles to the costmap the program uses the laser.
- Calculate a plan to reach the destination. (The robot has a maximum and minimum velocity).

The action `RP_approach_person` goal is to obtain the list that the node `people_detector` publish and select the person that the robot want to follow and publish the TF in "/target". In addiction, `RP_approach_person` processes if the robot continue watching the person or if the person is out of range.

## PDDL Components

- **Types**: person
- **Predicates:**
  - (person_followed ?r - robot ?p - person)
  - (person_at ?p - person ?wp - waypoint)
  - (person_at_room ?p - person ?room - room)
  - (robot_near_person ?r - robot ?p - person)
- **Actions:** guide_navigate, guide_move, guide_cross, follow_person, approach_person

## Nodes
### person_follower_node
The functions of the `person_follower_node` are:
- Receive the last position of the person that the robot is going to follow. (TF: /base_footprint-> /follow_target).
- Calculate the path to going to the person.
- Create a costmap to avoid obstacles.
- Move the robot.

#### Parameters
None.

#### Suscribed Topics
None.

#### Published Topics
- /cmd_vel [geometry_msgs::Twist]

#### Services
None.

## Launch
This package contains 2 launch files:
- **person_navigation_alone.launch** : roslaunch person_navigation person_navigation_alone.launch
- **person_navigation.launch** : roslaunch person_navigation person_navigation.launch
