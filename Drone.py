# simluation of a guide drone to help students navigate a campus in a shortest path manner

import math
from AstarSearch import AstarSearch

class Drone:
    def __init__(self, BUILDING_MAP):
        self.building_map = BUILDING_MAP
        self.astar_search = AstarSearch()

    def find_shortest_path(self, start, goal):
        def neighbors_func(node):
            return self.building_map.get_neighbors(node)

        def cost_func(current, neighbor):
            return self.building_map.get_cost(current, neighbor)

        path = self.astar_search.astar(start, goal, neighbors_func, cost_func)
        return path
    
    def print_path(self, path):
        if path is None:
            print("No path found.")
        else:
            print("Shortest path:", " -> ".join(map(str, path)))