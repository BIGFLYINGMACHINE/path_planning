"""
version1.1,2018-05-09
《基于智能优化与RRT算法的无人机任务规划方法研究》博士论文
《基于改进人工势场法的路径规划算法研究》硕士论文

"""

import matplotlib.pyplot as plt
import random
import math
import copy

show_animation = True


class Node(object):
   """
   RRT Node
   """

   def __init__(self, x, y):
       self.x = x
       self.y = y
       self.parent = None


class RRT(object):
   """
   Class for RRT Planning
   """

   def __init__(self, start, goal, obstacle_list, rand_area):
       """
       Setting Parameter

       start:Start Position [x,y]
       goal:Goal Position [x,y]
       obstacleList:obstacle Positions [[x,y,size],...]
       randArea:random sampling Area [min,max]

       """
       self.start = Node(start[0], start[1])
       self.end = Node(goal[0], goal[1])
       self.min_rand = rand_area[0]
       self.max_rand = rand_area[1]
       self.expandDis = 1.0
       self.goalSampleRate = 0.05  # 选择终点的概率是0.05
       self.maxIter = 500
       self.obstacleList = obstacle_list
       self.nodeList = [self.start]

   def random_node(self):
       """
       产生随机节点
       :return:
       """
       node_x = random.uniform(self.min_rand, self.max_rand)
       node_y = random.uniform(self.min_rand, self.max_rand)
       node = [node_x, node_y]

       return node

   @staticmethod
   def get_nearest_list_index(node_list, rnd):
       """
       :param node_list:
       :param rnd:
       :return:
       """
       d_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
       min_index = d_list.index(min(d_list))
       return min_index

   @staticmethod
   def collision_check(new_node, obstacle_list):
       a = 1
       for (ox, oy, size) in obstacle_list:
           dx = ox - new_node.x
           dy = oy - new_node.y
           d = math.sqrt(dx * dx + dy * dy)
           if d <= size:
               a = 0  # collision

       return a  # safe

   def planning(self):
       """
       Path planning

       animation: flag for animation on or off
       """

       while True:
           # Random Sampling
           if random.random() > self.goalSampleRate:
               rnd = self.random_node()
           else:
               rnd = [self.end.x, self.end.y]

           # Find nearest node
           min_index = self.get_nearest_list_index(self.nodeList, rnd)
           # print(min_index)

           # expand tree
           nearest_node = self.nodeList[min_index]

           # 返回弧度制
           theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)

           new_node = copy.deepcopy(nearest_node)
           new_node.x += self.expandDis * math.cos(theta)
           new_node.y += self.expandDis * math.sin(theta)
           new_node.parent = min_index

           if not self.collision_check(new_node, self.obstacleList):
               continue

           self.nodeList.append(new_node)

           # check goal
           dx = new_node.x - self.end.x
           dy = new_node.y - self.end.y
           d = math.sqrt(dx * dx + dy * dy)
           if d <= self.expandDis:
               print("Goal!!")
               break

           if True:
               self.draw_graph(rnd)

       path = [[self.end.x, self.end.y]]
       last_index = len(self.nodeList) - 1
       while self.nodeList[last_index].parent is not None:
           node = self.nodeList[last_index]
           path.append([node.x, node.y])
           last_index = node.parent
       path.append([self.start.x, self.start.y])

       return path

   def draw_graph(self, rnd=None):
       """
       Draw Graph
       """
       print('aaa')
       plt.clf()  # 清除上次画的图
       if rnd is not None:
           plt.plot(rnd[0], rnd[1], "^g")
       for node in self.nodeList:
           if node.parent is not None:
               plt.plot([node.x, self.nodeList[node.parent].x], [
                        node.y, self.nodeList[node.parent].y], "-g")

       for (ox, oy, size) in self.obstacleList:
           plt.plot(ox, oy, "sk", ms=10*size)

       plt.plot(self.start.x, self.start.y, "^r")
       plt.plot(self.end.x, self.end.y, "^b")
       plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
       plt.grid(True)
       plt.pause(0.01)

   def draw_static(self, path):
       """
       画出静态图像
       :return:
       """
       plt.clf()  # 清除上次画的图

       for node in self.nodeList:
           if node.parent is not None:
               plt.plot([node.x, self.nodeList[node.parent].x], [
                   node.y, self.nodeList[node.parent].y], "-g")

       for (ox, oy, size) in self.obstacleList:
           plt.plot(ox, oy, "sk", ms=10*size)

       plt.plot(self.start.x, self.start.y, "^r")
       plt.plot(self.end.x, self.end.y, "^b")
       plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])

       plt.plot([data[0] for data in path], [data[1] for data in path], '-r')
       plt.grid(True)
       plt.savefig('RRT.png')
       plt.show()


def main():
   print("start RRT path planning")

   obstacle_list = [
       (5, 1, 1),
       (3, 6, 2),
       (3, 8, 2),
       (1, 1, 2),
       (3, 5, 2),
       (9, 5, 2)]

   # Set Initial parameters
   rrt = RRT(start=[0, 0], goal=[8, 9], rand_area=[-2, 10], obstacle_list=obstacle_list)
   path = rrt.planning()
   print(path)

   # Draw final path
   if show_animation:
       plt.close()
       rrt.draw_static(path)


if __name__ == '__main__':
   main()

import math
from PIL import Image
import numpy as np
import networkx as nx
import copy

STAT_OBSTACLE='#'
STAT_NORMAL='.'

class RoadMap():
   """ 读进一张图片，二值化成为有障碍物的二维网格化地图，并提供相关操作
   """
   def __init__(self,img_file):
       """图片变二维数组"""
       test_map = []
       img = Image.open(img_file)
#        img = img.resize((100,100))  ### resize图片尺寸
       img_gray = img.convert('L')  # 地图灰度化
       img_arr = np.array(img_gray)
       img_binary = np.where(img_arr<127,0,255)
       for x in range(img_binary.shape[0]):
           temp_row = []
           for y in range(img_binary.shape[1]):
               status = STAT_OBSTACLE if img_binary[x,y]==0 else STAT_NORMAL 
               temp_row.append(status)
           test_map.append(temp_row)
           
       self.map = test_map
       self.cols = len(self.map[0])
       self.rows = len(self.map)
       
   def is_valid_xy(self, x,y):
       if x < 0 or x >= self.rows or y < 0 or y >= self.cols:
           return False
       return True

   def not_obstacle(self,x,y):
       return self.map[x][y] != STAT_OBSTACLE
   
   def EuclidenDistance(self, xy1, xy2):
       """两个像素点之间的欧几里得距离"""
       dis = 0
       for (x1, x2) in zip(xy1, xy2):
           dis += (x1 - x2)**2
       return dis**0.5

   def ManhattanDistance(self,xy1,xy2):
       """两个像素点之间的曼哈顿距离"""
       dis = 0
       for x1,x2 in zip(xy1,xy2):
           dis+=abs(x1-x2)
       return dis

   def check_path(self, xy1, xy2):
       """碰撞检测 两点之间的连线是否经过障碍物"""
       steps = max(abs(xy1[0]-xy2[0]), abs(xy1[1]-xy2[1])) # 取横向、纵向较大值，确保经过的每个像素都被检测到
       xs = np.linspace(xy1[0],xy2[0],steps+1)
       ys = np.linspace(xy1[1],xy2[1],steps+1)
       for i in range(1, steps): # 第一个节点和最后一个节点是 xy1，xy2，无需检查
           if not self.not_obstacle(math.ceil(xs[i]), math.ceil(ys[i])):
               return False
       return True

   def plot(self,path):
       out = []
       for x in range(self.rows):
           temp = []
           for y in range(self.cols):
               if self.map[x][y]==STAT_OBSTACLE:
                   temp.append(0)
               elif self.map[x][y]==STAT_NORMAL:
                   temp.append(255)
               elif self.map[x][y]=='*':
                   temp.append(127)
               else:
                   temp.append(255)
           out.append(temp)
       for x,y in path:
           out[x][y] = 127
       out = np.array(out)
       img = Image.fromarray(out)
       img.show()


def path_length(path):
   """计算路径长度"""
   l = 0
   for i in range(len(path)-1):
       x1,y1 = path[i]
       x2,y2 = path[i+1]
       if x1 == x2 or y1 == y2:
           l+=1.0
       else:
           l+=1.4
   return l



class PRM(RoadMap):
   def __init__(self, img_file, **param):
       """ 随机路线图算法(Probabilistic Roadmap, PRM)
       **param: 关键字参数，用以配置规划参数
               start: 起点坐标 (x,y)
               end: 终点左边 (x,y)
               num_sample: 采样点个数，默认100 int
               distance_neighbor: 邻域距离，默认100 float
       """
       RoadMap.__init__(self,img_file)
       
       self.num_sample = param['num_sample'] if 'num_sample' in param else 100
       self.distance_neighbor = param['distance_neighbor'] if 'distance_neighbor' in param else 100
       self.G = nx.Graph() # 无向图，保存构型空间的完整连接属性
       
   def learn(self):
       """PRM算法的学习阶段
           学习阶段只需要运行一次
       """
       # 随机采样节点
       while len(self.G.node)<self.num_sample:
           XY = (np.random.randint(0, self.rows),np.random.randint(0, self.cols)) # 随机取点
           if self.is_valid_xy(XY[0],XY[1]) and self.not_obstacle(XY[0],XY[1]): # 不是障碍物点
               self.G.add_node(XY)
       # 邻域范围内进行碰撞检测，加边
       for node1 in self.G.nodes:
           for node2 in self.G.nodes:
               if node1==node2:
                   continue
               dis = self.EuclidenDistance(node1,node2)
               if dis<self.distance_neighbor and self.check_path(node1,node2):
                   self.G.add_edge(node1,node2,weight=dis) # 边的权重为 欧几里得距离
   
   def find_path(self,startXY=None,endXY=None):
       """ 使用学习得到的无障碍连通图进行寻路
           (为方便测试，默认起点为左上，终点为右下)
       """
       # 寻路时再将起点和终点添加进图中，以便一次学习多次使用 
       temp_G = copy.deepcopy(self.G)
       startXY = tuple(startXY) if startXY else (0,0)
       endXY = tuple(endXY) if endXY else (self.rows-1, self.cols-1)
       temp_G.add_node(startXY)
       temp_G.add_node(endXY)
       for node1 in [startXY, endXY]: # 将起点和目的地连接到图中
           for node2 in temp_G.nodes:
               dis = self.EuclidenDistance(node1,node2)
               if dis<self.distance_neighbor and self.check_path(node1,node2):
                   temp_G.add_edge(node1,node2,weight=dis) # 边的权重为 欧几里得距离
       # 直接调用networkx中求最短路径的方法
       path = nx.shortest_path(temp_G, source=startXY, target=endXY)
       
       return self.construct_path(path)

   def construct_path(self, path):
       """find_path寻路得到的是连通图的节点，扩展为经过的所有像素点"""
       out = []
       for i in range(len(path)-1):
           xy1,xy2=path[i],path[i+1]
           steps = max(abs(xy1[0]-xy2[0]), abs(xy1[1]-xy2[1])) # 取横向、纵向较大值，确保经过的每个像素都被检测到
           xs = np.linspace(xy1[0],xy2[0],steps+1)
           ys = np.linspace(xy1[1],xy2[1],steps+1)
           for j in range(0, steps+1): 
               out.append((math.ceil(xs[j]), math.ceil(ys[j])))
       return out
       
#======= test case ==============
prm = PRM('map2.bmp',num_sample=200,distance_neighbor=200)
prm.learn()
path = prm.find_path()
prm.plot(path)
print('Path length:',path_length(path))