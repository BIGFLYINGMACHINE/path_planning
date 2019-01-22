'''
By  zack zhou
This is the code for this tutorial:https://www.redblobgames.com/pathfinding/a-star/introduction.html
I add some features on the original code,
such as changing from 4 neighbors to 8 neighbors, showing result with good looking pictures.
Feel free to use it on your own project
License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
'''
import math
import heapq
import matplotlib.pyplot as plt


# representing the map, its boundry, walls(obstacles)
class Graph():
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
        self.weights = {}

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x <= self.width and 0 <= y <= self.height
    
    def walkable(self, node):
        id = list(node)
        return id not in self.walls
    
    # the cost moving from current to next node
    def cost(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
        
    # neighbors of node
    def neighbors(self, id):
        (x, y) = id
        results = [(x-1, y), (x-1, y+1), (x, y+1), 
                    (x+1, y+1), (x+1, y), (x+1, y-1),
                    (x, y-1), (x-1, y-1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics, not critical to performance
        # remove those parts where we can't go
        results = filter(self.in_bounds, results)
        results = filter(self.walkable, results)
        return results
    

# python has an impletation for PQ, but it has not empty()
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

# distance from current to goal
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
        count = 0
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
                count += 1
    print('count', count)
    return came_from, cost_so_far


def main():
    graph = Graph(60, 60)

    
    sx = 10.0  
    sy = 10.0  
    gx = 55.0  
    gy = 20.0  
    start = (sx, sy)
    goal = (gx, gy)
    wall_x, wall_y = [], []
    for i in range(55):
        wall_x.append(30)
        wall_y.append(i)
        graph.walls.append([30, i])
    path_x, path_y = [], []
    came_from, cost = a_star_search(graph, start, goal)
    path = reconstruct_path(came_from, start, goal)
    for node in path:
        print(node)
        x, y = node
        path_x.append(x)
        path_y.append(y)
    plt.plot(path_x, path_y, '.k')
    plt.plot(wall_x, wall_y, 'ro')
    plt.plot(sx, sy, "xb")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == '__main__':
    main()
