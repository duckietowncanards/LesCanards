#!/usr/bin/env python3
import numpy as np
from graph_computation.town_graph import Graph, Node, Edge, Direction
import heapdict
from std_msgs.msg import Int32MultiArray

class A_Star():

    def __init__(self, graph:Graph):
        # Initialize  
        self.graph = graph
        # priority queue with nodes:
        self.next = heapdict.heapdict()
        # already visited nodes
        self.visited = []
        # costs to get to the nodes
        self.costs = {}
        # predecessor of each node
        self.predecessor = {}



    def compute_path(self, starting_node:Node, target_node:Node) -> list:
    
        target_x = target_node.x
        target_y = target_node.y

        # Add starting node to priority queue with costs 0
        self.next[starting_node] = 0
        self.costs[starting_node.id] = 0
        self.predecessor[starting_node.id] = starting_node.id

        # While there are still nodes left to explore
        while list(self.next.items()):

            # Pop item with lowest priority
            self.current = self.next.popitem()
            current_node = self.current[0]
            current_costs = self.costs[current_node.id]

            # this node is now already fully visited
            self.visited.append(current_node)

            # if the current node is the target not we are finished
            if current_node == target_node:
                break
            
            # Look at all neighbors of the current node
            neighboring_edges = self.graph.getIncidentEdges(current_node)
            for edge in neighboring_edges:

                # if the successor is already visited continue
                successor = self.graph.getNode(edge.target_id)
                if successor in self.visited:
                    continue
                
                # compute straight-line distance between node an target node
                direct_distance = np.sqrt((successor.x-target_x)**2 + (successor.y-target_y)**2)
                # compute costs to get from the starting node to the successor node (along the current node)
                new_costs = current_costs + edge.weight

                # If the successor is already in the priority queue and the costs to get to the successor are smaller than the new cost continue
                if successor in self.next and self.costs[successor.id] <= new_costs:
                    continue
                
                # add/update the priority queue and the other parameters
                self.next[successor] = direct_distance + new_costs
                self.predecessor[successor.id] = current_node.id
                self.costs[successor.id] = new_costs
        
        # compute path
        path = []
        curr = target_node.id
        path.append(curr)
        while curr != starting_node.id:
            pred = self.predecessor[curr]
            path.append(pred)
            curr = pred
        
        path.reverse()
        return path


    def compute_commands(self, path:list) -> list:
        directions = []
        for i in range(len(path)-1):
            edge = self.graph.getEdge(path[i], path[i+1])
            directions.append(int(edge.direction))
        return directions


    def compute_a_star_msg(self, start_id: int, target_id: int):
        # priority queue with nodes:
        self.next = heapdict.heapdict()
        # already visited nodes
        self.visited = []
        # costs to get to the nodes
        self.costs = {}
        # predecessor of each node
        self.predecessor = {}
        starting_node = self.graph.getNode(start_id)
        target_node = self.graph.getNode(target_id)

        path = self.compute_path(starting_node, target_node)
        commands = self.compute_commands(path)

        path_msg = Int32MultiArray()
        path_msg.data = path

        commands_msg = Int32MultiArray()
        commands_msg.data = commands

        return path_msg, commands_msg














