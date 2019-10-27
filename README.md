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
    1. Constructor takes in a node as input and initializes a dictionary with empty list values for edges.
    1. `add_edge` creates a directed edge between the start_node and `end_node` by adding an entry 
    into the edge list associated with start_node in the edge dictionary. 
    1. `add_bidirectional_edge` creates a bi-directional association by calling the `add_edge` function with both 
    combinations of nodes as start_node and end_node.
    1. `dijkstra` implements the dijkstra algorithm between the start_node and goal_node parameter passed. 
    The graph association used is as defined in the previous steps. The function returns a list of waypoints/nodes 
    as the response if a valid goal can be found in the graph definition else returns an empty list.
1. Function `grow_obstacles` grows the obstacles by keeping the centre of the robot as a point of reference and 
returns a series of possible boundary points for the new grown obstacle.
1. Function `convex_hull` runs the convex hull algorithm on a list of points and detects the boundary/convex hull 
for them. The ConvexHull class from scipy.spatial has been used in this function for this purpose.
1. Function `point_lie_on_segment` checks if a point, (`point3`) lies on the line connecting points, (`point1` and `point2`).
1. Function `compute_orientation` checks the possible orientation point3 with respect to points, , (`point1` and `point2`).
The possible results are COLINEAR when the points lie on the same line, CLOCKWISE when `point3` lies is clockwise 
direction w.r.t. the other points or ANTICLOCKWISE if otherwise.
1. Function `segment_intersect` checks if the line segments formed by the pair of points (`point1` and `point2`) and 
(`point3` and `point4`) intersects or not. It will return `True` if they intersect and `False` otherwise.
1. Function `get_nodes` is a utility function that generates a list of `Node` objects from a series of points.
1. Function `get_edges` is a utility function that generates a list of line segments between the obstacle nodes.
1. Function `extract_non_intersecting_lines` takes in the `candidate_line_segments` and `obstacle_line_segments` 
as parameters and removes all the line segments in candidate_line_segments that intersect with the obstacles.
1. Function `vgraph` calculates the visibility graph given a list of obstacles points and obstacle segments. 
The algorithm works by generating all possible line segments by enumerating over the list of points and then removing 
the once that intersect with the obstacle line segments.
1. The `main` function does the following series of actions in sequence and returns the dijkstra waypoints for the bot 
to follow
    1. Load the goal point and the positions of obstacles.
    1. Grow the obstacles with the bot dimensions as reference and calculate the convex hull of the same.
    1. Calculate the visibility graph of the grown obstacle boundaries obtained from the previous step.
    1. Create a `Graph` object by defining the edges and nodes of the vgraph.
    1. Run the dijkstra algorithm between the start point (0,0) and the loaded goal.
    1. Return the calculated waypoints.
    
#### Methods described in `move_fwd.py`

#### Methods described in `vgraph_with_marker.py`

----
### Video
