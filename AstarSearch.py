import heapq
import math

# A* Search Algorithm Implementation
class AstarSearch:
    def heuristic(self, start, goal):
        # Using Euclidean distance as heuristic
        return math.dist(start, goal)

    #main A* search function
    def astar(self, start, goal, neighbors_func, cost_func):
        open_set = [] # set of nodes to be evaluated
        heapq.heappush(open_set, (0, start)) 
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in neighbors_func(current):
                tentative_g_score = g_score[current] + cost_func(current, neighbor)

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # Path not found
    
    # Helper function to reconstruct the path from came_from map
    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]  # Return reversed path

