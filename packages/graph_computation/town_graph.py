#!/usr/bin/env python3
from dataclasses import dataclass
from enum import IntEnum

from duckiebot_control import commands

# Values come from the shared command-id table so an edge direction published on /path_commands is
# read as the same command the model was trained on. The planner calls the "just follow the lane"
# id DOWN; it is the model's DEFAULT.
class Direction(IntEnum):
    STRAIGHT = commands.STRAIGHT
    LEFT = commands.LEFT
    RIGHT = commands.RIGHT
    DOWN = commands.DEFAULT


@dataclass(frozen=True)
class Node:
    id: int
    x: int
    y: int

@dataclass
class Edge:
    direction: Direction
    source_id: int
    target_id: int
    weight: float

class Graph():
    def __init__(self):

        self.nodes = [
            Node(1, 0, 1),
            Node(2, 2, 1),
            Node(3, 4, 1),
            Node(4, 1, 2),
            Node(5, 3, 2),
            Node(6, 1, 2),
            Node(7, 3, 2),
            Node(8, 0, 3),
            Node(9, 2,3),
            Node(10, 4, 3),
            Node(11, 2, 4),
            Node(12, 1, 5),
            Node(13, 3, 5),
        ]

        self.edges = [
            Edge (Direction.RIGHT, 1, 4, 1.5),
            Edge (Direction.STRAIGHT, 1, 12, 4.5),
            Edge (Direction.RIGHT, 2, 5, 1.5),
            Edge (Direction.LEFT, 2, 6, 2.5),
            Edge (Direction.STRAIGHT, 2, 11, 3.0),
            Edge (Direction.LEFT, 3, 7, 2.5),
            Edge (Direction.STRAIGHT, 3, 13, 5.5),
            Edge (Direction.RIGHT, 4, 1, 4.5),
            Edge (Direction.STRAIGHT, 4, 5, 2.0),
            Edge (Direction.LEFT, 4, 11, 3.5),
            Edge (Direction.LEFT, 5, 13, 6.0),
            Edge (Direction.LEFT, 6, 2, 7.5),
            Edge (Direction.RIGHT, 6, 12, 4.0),
            Edge (Direction.LEFT, 7, 1, 5.5),
            Edge (Direction.STRAIGHT, 7, 6, 2.0),
            Edge (Direction.RIGHT, 7, 11, 2.5),
            Edge (Direction.STRAIGHT, 8, 2, 7.0),
            Edge (Direction.LEFT, 8, 4, 2.5),
            Edge(Direction.STRAIGHT, 9, 1, 5.0),
            Edge(Direction.LEFT, 9, 5, 2.5),
            Edge(Direction.RIGHT, 9, 6, 1.5),
            Edge(Direction.RIGHT, 10, 7, 1.5),
            Edge(Direction.LEFT, 11, 8, 6.0),
            Edge(Direction.RIGHT, 11, 10, 4.0),
            Edge(Direction.RIGHT, 12, 9, 2.5),
            Edge(Direction.STRAIGHT, 12, 10, 4.5),
            Edge(Direction.STRAIGHT, 13, 8, 5.5),
            Edge(Direction.LEFT, 13, 9, 3.5)
        ]

    def getNode(self, id: int) -> Node:
        for node in self.nodes:
            if node.id == id:
                return node
        return -1
    
    def getEdge(self, source_id, target_id):
        for edge in self.edges:
            if edge.source_id == source_id and edge.target_id == target_id:
                return edge
        return -1

    def getIncidentEdges(self, node: Node) -> list:

        neighboring_edges = []
        for edge in self.edges:
            if edge.source_id == node.id:
                neighboring_edges.append(edge)
        return neighboring_edges
        