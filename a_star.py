import math

class Node():
    def __init__(self, x, y):
        self.id = (x, y)
        self.x = x
        self.y = y

    def __sub__(self, other):
        new_x, new_y = self.id - other.id
        return Node(new_x, new_y)
    def module(self):
        return math.sqrt(self.x**2 + self.y**2)

class Graph():
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
        self.weights = {}

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def walkable(self, id):
        return id not in self.walls
    
    def cost(self, from_node, to_node):
        distance = from_node - to_node
        return distance.module()

    def neighbors(self, id):
        (x, y) = id
        results = [(x-1, y), (x-1, y+1), (x, y+1), 
                    (x+1, y+1), (x+1, y), (x+1, y-1),
                    (x, y-1), (x-1, y-1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.walkable, results)
        return results
    

