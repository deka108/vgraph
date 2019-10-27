# Lab 3 - Visibility Graph Path Planning
Lab 3 for [COMSW4733 Computational Aspects of Robotics](https://www.cs.columbia.edu/~allen/F19/) at Columbia University (Instructor: [Prof. Peter Allen](http://www.cs.columbia.edu/~allen/)).

Authors: Deka Auliya, Madhavan Seshadri and Shravan Karthik

----
### Prerequisites:
1. Installation of `indigo-devel` on Ubuntu 14.04

1. Installation of `turtlebot gazebo` packagae

1. Setup a catkin development environment:
    1. Make a new directory using the following command 
    `mkdir -p ~/catkin_ws/src`
    1. Clone the rbx repository in the `catkin_ws/src` folder using the following command:
    `git clone https://github.com/pirobot/rbx1.git`
    1. Checkout the `indigo-devel` branch in the `rbx1` folder.
    1. Switch to the following directory
    `cd ~/catkin_ws/`
    1. Run `catkin_make` command on this directory.
1. Place the `vgraph` directory in `catkin_ws`.
1. Compile the new sources using the following command
```
cd ../
catkin_make
source devel/setup.bash
rospack profile
```

Start the ROS Master and launch RViz using the following command
`roslaunch vgraph launch.launch`


----
### Usage:
1. Switch to the vgraph directory using the following command `cd ~/catkin_ws/src/vgraph/src`
1. Give execute permissions to the file using the command `chmod +x vgraph_with_marker.py`
1. Execute the script: `./vgraph_with_marker.py`

This script will invoke functions from the following scripts: `vgraph.py` and `move_fwd.py`

----
### Method

#### Methods described in `vgraph.py`

1. Class `Node` is the node definition for the custom graph data structure for the purposes of this assignment. 
Node object contains the coordinates (x,y) of points in the defined graph.
1. Class `Edge` is the edge definition for the custom graph data structure for the purposes of this assignment. 
An edge is defined as a directed connection between the `start_node` and `end_node` with the direction pointing 
to the end node. It has an associated weight/cost for traversing along it.
1. Class `Graph` is the graph definition of the map. A graph is defined by a series of nodes and edges connecting them.
    1. Constructor takes in a node as input and defines the  


----
### Video
