import math
import heapq
import matplotlib.pyplot as plt


class Graph():
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
        self.weights = {}

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x <= self.width and 0 <= y <= self.height
    
    def walkable(self, id):
        return id not in self.walls
    
    def cost(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
        

    def neighbors(self, id):
        (x, y) = id
        results = [(x-1, y), (x-1, y+1), (x, y+1), 
                    (x+1, y+1), (x+1, y), (x+1, y-1),
                    (x, y-1), (x-1, y-1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.walkable, results)
        return results
    
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]


def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far


def main():
    graph = Graph(60, 60)

    
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 55.0  # [m]
    gy = 50.0  # [m]
    
    start = (sx, sy)
    goal = (gx, gy)


    came_from, cost = a_star_search(graph, start, goal)
    path = reconstruct_path(came_from, start, goal)
    for node in path:
        print(node)


if __name__ == '__main__':
    main()