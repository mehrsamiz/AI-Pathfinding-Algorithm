import sys
import os
import pygame

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/utils")

from agent import AbstractSearchAgent
from plotting import Plotting
import generator as gn

class BFS_Agent(AbstractSearchAgent):
    def searching(self):
                """
                Breadth-First Search (BFS) algorithm.

                Returns:
                    path (list): The planned path from start to goal.
                    visited (list): List of visited nodes in the order they were expanded.
                """
                # Initialize OPEN with the start state
                self.OPEN.append(self.s_start)
                self.PARENT[self.s_start] = None
                self.CLOSED = set()  # Efficient lookup for closed nodes
                visited = []

                while self.OPEN:
                    # Dequeue the first node from OPEN
                    node = self.OPEN.pop(0)
                    self.CLOSED.add(node)
                    visited.append(node)

                    # Check if the goal has been reached
                    if node == self.s_goal:
                        path = self.extract_path(self.PARENT)
                        return path, visited

                    # Expand neighbors using the inherited get_neighbor method
                    for neighbor in self.get_neighbor(node):
                        if neighbor not in self.CLOSED and neighbor not in self.OPEN:
                            self.OPEN.append(neighbor)
                            self.PARENT[neighbor] = node

                # If the goal is not reachable, return failure
                return None, visited


class DFS_Agent(AbstractSearchAgent):
    def searching(self):
                """
                Depth-First Search algorithm.

                Returns:
                    path (list): The planned path from start to goal.
                    visited (list): All visited nodes for visualization.
                """
                # Initialize
                self.OPEN = [self.s_start]
                self.PARENT[self.s_start] = None
                visited = []

                while self.OPEN:
                    # Pop from stack
                    current = self.OPEN.pop()
                    visited.append(current)
                    self.CLOSED.append(current)

                    # Check if the goal is reached
                    if current == self.s_goal:
                        path = self.extract_path(self.PARENT)
                        return path, visited

                    # Expand neighbors
                    for neighbor in self.get_neighbor(current):
                        if neighbor not in self.CLOSED and neighbor not in self.OPEN:
                            self.OPEN.append(neighbor)
                            self.PARENT[neighbor] = current

                # Goal not reachable
                return None, visited


class AStar_Agent(AbstractSearchAgent):

    def heuristic(self, node):
        """
        Heuristic function: Default is Manhattan distance.
        """
        return abs(node[0] - self.s_goal[0]) + abs(node[1] - self.s_goal[1])

    def searching(self):
        """
        A* search algorithm

        Returns:
        * path (list): The planned path from start to goal
        * visited (list): List of visited nodes
        """

        # Initialize
        self.g[self.s_start] = 0  # Cost from start to the start is 0
        self.OPEN = [self.s_start]
        self.PARENT[self.s_start] = None
        visited = []

        while self.OPEN:
            # Select node with the smallest f(n) = g(n) + h(n)
            current = min(self.OPEN, key=lambda x: self.g[x] + self.heuristic(x))
            self.OPEN.remove(current)
            visited.append(current)

            # Check if goal is reached
            if current == self.s_goal:
                path = self.extract_path(self.PARENT)
                return path, visited

            self.CLOSED.append(current)

            # Expand neighbors
            for neighbor in self.get_neighbor(current):
                if neighbor in self.CLOSED:
                    continue

                tentative_g = self.g[current] + 1  # Assume uniform cost of 1 for each step
                if neighbor not in self.OPEN or tentative_g < self.g.get(neighbor, float('inf')):
                    self.PARENT[neighbor] = current
                    self.g[neighbor] = tentative_g

                    if neighbor not in self.OPEN:
                        self.OPEN.append(neighbor)

        # Goal not reachable
        return None, visited


if __name__ == "__main__":
    s_start = (5, 5) # Starting point
    s_goal = (45, 25) # Goal

    FPS = 60
    generate_mode = False # Turn to True to change the map
    map_name = 'default'

    if generate_mode:
        gn.main(map_name)
    
    else:
        agent = DFS_Agent(s_start, s_goal, map_name) # Choose the agent here
        path, visited = agent.searching()

        # Plotting the path
        plot = Plotting(s_start, s_goal, map_name, FPS)

        plot.animation(path, visited)
