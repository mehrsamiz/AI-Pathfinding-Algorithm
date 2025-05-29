import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/utils")

from agent import AbstractSearchAgent
from plotting import Plotting
import generator as gn

class BFS_Agent(AbstractSearchAgent):
    def searching(self):
        """
        BFS search algorithm
        
        Returns:
        * path (list): The planned path from start to goal

        * visted (list): list of visited nodes
        """
        
        # TODO

class DFS_Agent(AbstractSearchAgent):
    def searching(self):
        """
        DFS search algorithm
        
        Returns:
        * path (list): The planned path from start to goal

        * visted (list): list of visited nodes
        """

        # TODO

class AStar_Agent(AbstractSearchAgent):
    def searching(self):
        """
        DFS search algorithm
        
        Returns:
        * path (list): The planned path from start to goal

        * visted (list): list of visited nodes
        """

        # TODO

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