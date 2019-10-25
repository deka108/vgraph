#!/usr/bin/env python
# coding: utf-8

import itertools
import operator

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull

from src.create_map import load_goal
from src.create_map import load_obstacles
from src.create_map import map2img


class Edge(object):
    def __init__(self, start_node, end_node, weight):
        self.start_node = start_node
        self.end_node = end_node
        self.weight = weight
    
    def __repr__(self):
        return "Edge({}:{}, {})".format(self.start_node, self.end_node, self.weight)


class Node:
    def __init__(self, x=None, y=None):
        self.x = x
        self.y = y

    def __repr__(self):
        return "Node(x={}, y={})".format(self.x, self.y)


class Graph:
    def __init__(self, nodes):
        self.nodes = nodes
        self.graph = dict()

        for node in self.nodes:
            self.graph[node] = set([])

    def add_edge(self, start_node, end_node):
        weight = np.sqrt((end_node.x - start_node.x) ** 2 + (end_node.y - start_node.y) ** 2)
        if Edge(start_node, end_node, weight) not in self.graph[start_node]:
            self.graph[start_node].add(Edge(start_node, end_node, weight))

    def add_bidirectional_edge(self, start_node, end_node):
        self.add_edge(start_node, end_node)
        self.add_edge(end_node, start_node)

    def dijkstra(self, start_node, goal_node):
        visited = {i: False for i in self.nodes}
        distance = {i: float("inf") for i in self.nodes}
        parent_node = {i: None for i in self.nodes}

        queue = [(start_node, 0)]
        node = start_node
        is_goal_found = False
        distance[start_node] = 0

        while len(queue) != 0:

            queue.sort(key=operator.itemgetter(1), reverse=True)
            while visited[node]:
                node = queue.pop()[0]

            visited[node] = True
            if node == goal_node:
                print("Goal is found!")
                is_goal_found = True
                break

            edges = self.graph[node]
            for edge in edges:
                distance_end_node = distance[node] + edge.weight
                if distance_end_node < distance[edge.end_node]:
                    distance[edge.end_node] = distance[node] + edge.weight
                    queue.append((edge.end_node, distance_end_node))
                    parent_node[edge.end_node] = node

        path_to_goal = []
        # back track, to the start node
        if is_goal_found:
            path_to_goal.append(goal_node)
            while path_to_goal[0] != start_node:
                path_to_goal.insert(0, parent_node[path_to_goal[0]])

        return path_to_goal


def grow_obstacles(obstacle):
    expanded_obstacle = []
    for point in obstacle:
        expanded_obstacle.append(point)
        expanded_obstacle.append([point[0] - 18, point[1] - 18])
        expanded_obstacle.append([point[0] + 18, point[1] + 18])
        expanded_obstacle.append([point[0] + 18, point[1] - 18])
        expanded_obstacle.append([point[0] - 18, point[1] + 18])
    return np.array(expanded_obstacle)


def convex_hull(obstacles):
    hulls = []
    grown_obstacles = []

    for obstacle in obstacles:
        expanded_obstacle = grow_obstacles(obstacle)

        hull = ConvexHull(expanded_obstacle)

        x = expanded_obstacle[hull.vertices, 0]
        y = expanded_obstacle[hull.vertices, 1]
        points = np.stack((x, y), axis=-1)
        grown_obstacles.append(points)
        hulls.append(hull)

    return hulls, grown_obstacles


def plot_obstacle_growth(grown_obstacles):
    for obstacle in grown_obstacles:
        x = obstacle[:, 0]
        y = obstacle[:, 1]
        x = np.append(x, x[0])
        y = np.append(y, y[0])
        plt.plot(x, y, '--k')


def plot_origin_destination(start, goal):
    plt.plot(goal[0], goal[1], 'ob')
    plt.plot(start[0], start[1], 'or')


def plot_obstacles(obstacles):
    for obstacle in enumerate(obstacles):
        x = [point[0] for point in obstacle]
        y = [point[1] for point in obstacle]
        x.append(obstacle[0][0])
        y.append(obstacle[0][1])

        plt.plot(x, y, 'r-')


class Direction:
    CLOCKWISE = 0
    ANTI_CLOCKWISE = 1
    COLINEAR = 2


def point_lie_on_segment(point1, point2, point3, epsilon=0.):
    if (min(point1.x, point2.x) - epsilon <= point3.x <= max(point1.x, point2.x) + epsilon) \
            and\
            (min(point1.y, point2.y) - epsilon <= point3.y <= max(point1.y, point2.y) + epsilon):
        return True
    return False


def compute_orientation(point1, point2, point3, epsilon=0.):
    cross_product = (point2.y - point1.y) * (point3.x - point2.x) - (point2.x - point1.x) * (point3.y - point2.y)

    if abs(cross_product) <= epsilon:
        return Direction.COLINEAR
    elif cross_product > 0:
        return Direction.CLOCKWISE
    elif cross_product < 0:
        return Direction.ANTI_CLOCKWISE


def segment_intersect(point1, point2, point3, point4, epsilon=0.):
    orientation1 = compute_orientation(point1, point2, point3)
    orientation2 = compute_orientation(point1, point2, point4)
    orientation3 = compute_orientation(point3, point4, point1)
    orientation4 = compute_orientation(point3, point4, point2)

    if orientation1 != orientation2 and orientation3 != orientation4:
        return True

    if orientation1 == Direction.COLINEAR and point_lie_on_segment(point1, point2, point3, epsilon):
        return True

    if orientation2 == Direction.COLINEAR and point_lie_on_segment(point1, point2, point4, epsilon):
        return True

    if orientation3 == Direction.COLINEAR and point_lie_on_segment(point3, point4, point1, epsilon):
        return True

    if orientation4 == Direction.COLINEAR and point_lie_on_segment(point3, point4, point2, epsilon):
        return True

    return False


def get_nodes(obstacles):
    # convert all the obstacles into Point(x, y)
    obstacle_points = []
    for obs in obstacles:
        obs_points = []
        for point in obs:
            obs_points.append(Node(*point))
        obstacle_points.append(obs_points)
    return obstacle_points


def get_edges(obstacle_points):
    """
        computing line segments for each padded obstacle edges
    :param obstacle_points:
    :return:
    """

    obstacle_edges = []

    for obs in obstacle_points:
        obs_point_segments = [(obs[i], obs[i+1]) for i in range(len(obs) - 1)]
        if len(obs) > 2:
            obs_point_segments.append((obs[-1], obs[0]))
        obstacle_edges.append(obs_point_segments)

    return obstacle_edges


def extract_non_intersecting_lines(candidate_segments, obstacle_segments):
    lines_not_intersect = []
    for idx1, can_segment in enumerate(candidate_segments):
        intersect = False

        for obs_segment in obstacle_segments:
            if point_lie_on_segment(obs_segment[0], obs_segment[1], can_segment[0]):
                continue

            if point_lie_on_segment(obs_segment[0], obs_segment[1], can_segment[1]):
                continue

            intersect = segment_intersect(*can_segment, *obs_segment)
            if intersect:
                break

        if not intersect:
            lines_not_intersect.append(can_segment)

    return lines_not_intersect


def vgraph(all_points, obstacle_segments):
    """
    Obtain non intersecting segments from vornoi graph
    :param all_points:
    :param obstacle_segments:
    :return:
    """
    visible_segments = []

    for i, points in enumerate(all_points):
        for point in points:
            other_points = [all_points[j] for j in range(i + 1, len(all_points))]
            other_points = list(itertools.chain.from_iterable(other_points))
            candidate_segments = [(point, other_point) for other_point in other_points]
            free_segments = extract_non_intersecting_lines(candidate_segments, obstacle_segments)
            visible_segments.append(free_segments)
    return visible_segments


def plot_map(start, goal, obstacles):
    img = np.full((600, 1200, 3), 255, np.uint8)

    obs = []
    for ob in obstacles:
        ob = map2img(ob)
        obs.append(ob)
        cv2.fillConvexPoly(img, ob.reshape(-1, 1, 2), (255,255,0))

    # draw start and goal point
    goal_img = tuple(map2img([goal])[0])
    start_img = tuple(map2img([start])[0])
    circ1 = cv2.circle(img, goal_img, 7, (100, 0, 0), -1)
    circ2 = cv2.circle(img, start_img, 7, (0, 0, 100), -1)

    fig = plt.figure(figsize=(20, 20))
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))


def main():

    obstacles = load_obstacles("../data/world_obstacles.txt")

    # Set the origin and destination
    start = [0, 0]
    goal = load_goal("../data/goal.txt")

    hulls, grown_obstacles = convex_hull(obstacles)

    # convert all the start and goal into Point(x, y)
    start_point = Node(*start)
    goal_point = Node(*goal)

    obstacle_points = get_nodes(grown_obstacles)
    obstacle_edges = get_edges(obstacle_points)

    obstacle_segments = list(itertools.chain.from_iterable(obstacle_edges))

    all_points = [[start_point]] + obstacle_points + [[goal_point]]

    all_free_segments = vgraph(all_points, obstacle_segments)

    all_free_segments = all_free_segments + obstacle_edges

    graph = Graph(list(itertools.chain.from_iterable(all_points)))

    for free_segments in all_free_segments:
        for segment in free_segments:
            graph.add_bidirectional_edge(segment[0], segment[1])

    way_points = graph.dijkstra(start_point, goal_point)
    print(way_points)


if __name__ == '__main__':
    main()
