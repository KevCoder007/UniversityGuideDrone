import heapq
import math

class AstarSearch:
    def __init__(self, start, goal, heuristic):
        self.start = start
        self.goal = goal
        self.heuristic = heuristic
        self.buildings_open = set()
        self.buildings_visited = set()
        self.came_from = {}
        self.g_score = {}
        self.f_score = {}

    def reconstruct_path(self, current):
        total_path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self):
        self.open_set.add(self.start)
        self.g_score[self.start] = 0
        self.f_score[self.start] = self.heuristic(self.start, self.goal)

        while self.open_set:
            current = min(self.open_set, key=lambda x: self.f_score.get(x, float('inf')))
            if current == self.goal:
                return self.reconstruct_path(current)

            self.open_set.remove(current)
            self.closed_set.add(current)

            for neighbor in current.get_neighbors():
                if neighbor in self.closed_set:
                    continue

                tentative_g_score = self.g_score.get(current, float('inf')) + current.distance_to(neighbor)

                if neighbor not in self.open_set:
                    self.open_set.add(neighbor)
                elif tentative_g_score >= self.g_score.get(neighbor, float('inf')):
                    continue

                self.came_from[neighbor] = current
                self.g_score[neighbor] = tentative_g_score
                self.f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)

        return self.f_score  # Path not found


