
'''遗传算法'''
import random
import math
import copy
from tkinter import *
import tkinter.font as tkFont
import time, threading

WIDTH = 100
HEIGHT = 100
MIN = 0
MAX = WIDTH * HEIGHT - 1

PATH_COUNT = 100
# 交叉概率
cross_p = 0.6
# 变异概率
variation_p = 0.4
# 变异次数
variation_times = 4

DIS_1 = 1.4
DIS_2 = 1

S = 0
D = 0

best_path = []
best_path_index = 0

res_fit = []

# 路径
paths = []
# 最优路径
# 迭代次数
ITERATION_COUNT = 100
#
direction_arr = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]


def is_valid(point):
    if point[0] < 0 or point[1] < 0 or point[0] >= WIDTH or point[1] >= HEIGHT:
        return False
    return True


# 计算欧式距离
def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


# 标号转坐标
def mark2position(mark):
    return (mark % WIDTH, int(mark / WIDTH))


def position2mark(position):
    return position[1] * WIDTH + position[0]


# 5 6 7
# 3   4
# 0 1 2
def generate_one_path(start, end):
    res = []
    res.append(start)

    s = start
    target_point = mark2position(end)
    dis = distance(mark2position(start), target_point)

    while (s != end):
        pos = mark2position(s)
        r = random.randint(0, 7)
        pos = (pos[0] + direction_arr[r][0], pos[1] + direction_arr[r][1])
        temp_dis = distance(pos, target_point)
        if is_valid(pos) and temp_dis <= dis:
            s = position2mark(pos)
            dis = temp_dis
            res.append(s)
    return res


# 初代
def init(count):
    res = []
    for i in range(0, count):
        res.append(generate_one_path(S, D))
    return res


# 计算一条路径的适应度值
def one_path_fit_val(path):
    sm = 0
    for i in range(1, len(path)):
        w = int(math.fabs(path[i - 1] - path[i]))
        if w == 1 or w == WIDTH:
            sm += DIS_2
        else:
            sm += DIS_1
    return MAX / sm


# 计算适应度值
def fitness():
    res = []
    max_fit = -1
    global best_path
    global best_path_index

    temp_best_path = []

    for i in range(len(paths)):
        f = one_path_fit_val(paths[i])
        res.append(f)
        if f > max_fit:
            max_fit = f
            temp_best_path = paths[i]
            best_path_index = i
    best_path = copy.deepcopy(temp_best_path)
    res_fit.append(max_fit)
    return res


# 累计概率
def cumulative_probability(fits):
    res = []
    sm = sum(fits)
    temp = fits[0] / sm
    res.append(temp)
    for i in range(1, len(fits)):
        res.append(res[i - 1] + fits[i] / sm)
    return res


# 选择 产生下一代
def choose(pArr, count):
    res = []
    for i in range(count):
        p = random.random()
        for j in range(len(pArr)):
            if p <= pArr[j]:
                res.append(paths[j])
                break
    return res


def cross_one_times(path1, path2):
    # 求交集
    temp = list(set(path1[1:-1]).intersection(set(path2[1:-1])))
    sz = len(temp)
    if sz == 0:
        return (path1, path2)
    r = random.random()
    if r > cross_p:
        index = random.randint(0, sz - 1)
        e = temp[index]
        t1 = path1.index(e)
        t2 = path2.index(e)
        p1 = path1[:t1]
        p2 = path2[t2:]
        p3 = path2[:t2]
        p4 = path1[t1:]
        p1.extend(p2)
        p3.extend(p4)
        return (p1, p3)
    else:
        return (path1, path2)


def cross():
    n = len(paths)
    res = []
    for i in range(1, n, 2):
        p = cross_one_times(paths[i], paths[i - 1])
        res.extend(p)

    # 奇数情况
    if len(res) < n:
        res.append(paths[n - 1])
    return res


# 判断三点之间是否联通
def is_valid_3_mark(m1, m2, m3):
    # 重复
    if m1 == m2 or m1 == m3 or m2 == m3:
        return False
    if m2 < MIN or m2 > MAX:
        return False
    # 不联通
    if not (m1 + 1 == m2 or m1 - 1 == m2 or m1 + WIDTH == m2 or m1 - WIDTH == m2
            or m1 + WIDTH + 1 == m2 or m1 + WIDTH - 1 == m2
            or m1 - WIDTH + 1 == m2 or m1 - WIDTH - 1 == m2):
        return False
    # 不联通
    if not (m3 + 1 == m2 or m3 - 1 == m2 or m3 + WIDTH == m2 or m3 - WIDTH == m2
            or m3 + WIDTH + 1 == m2 or m3 + WIDTH - 1 == m2
            or m3 - WIDTH + 1 == m2 or m3 - WIDTH - 1 == m2):
        return False
    return True


def variation_one_times(path):
    r = random.random()
    if r < variation_p:
        return path
    else:
        sz = len(path)
        if sz <= 2:
            return path
        # 变异点
        prob_mark = []
        var_index = random.randint(1, sz - 2)
        pre_mark = path[var_index - 1]
        cnt_mark = path[var_index]
        next_mark = path[var_index + 1]
        # 8中情况
        temp_mark = [cnt_mark + 1, cnt_mark - 1, cnt_mark + WIDTH, cnt_mark - WIDTH, cnt_mark + WIDTH + 1,
                     cnt_mark + WIDTH - 1, cnt_mark - WIDTH - 1, cnt_mark - WIDTH + 1]
        for e in temp_mark:
            if is_valid_3_mark(pre_mark, e, next_mark):
                prob_mark.append(e)

        if len(prob_mark) == 0:
            return path
        changed_mark = prob_mark[random.randint(0, len(prob_mark) - 1)]
        path[var_index] = changed_mark
        return path


def variation():
    res = paths
    for i in range(variation_times):
        temp = []
        for e in res:
            temp.append(variation_one_times(e))
        res = temp
    return res


def output(g, f):
    print("第" + str(g) + "代：最优路径：", end="", file=f)
    print(best_path, end="", file=f)
    print("适应度: ", end="", file=f)
    print(fits[best_path_index], file=f)
    for i, path in enumerate(paths):
        print(str(i + 1) + ". ", end="", file=f)
        print(path, end="", file=f)
        print("适应度值：" + str(fits[i]), file=f)


def mark_screen_position(mark, x_min, y_max):
    temp_p = mark2position(mark)
    x = temp_p[0] - x_min
    y = y_max - temp_p[1]
    return (x, y)


def show(path, title):
    canvas_width = 1000
    point_r = 2
    show_mark_min_width = 10
    temp = []
    for p in path:
        temp.append(p % 100)
    x_min = min(temp)
    x_max = max(temp)
    temp.clear()
    for p in path:
        temp.append(int(p / 100))
    y_min = min(temp)
    y_max = max(temp)
    d = max(x_max - x_min + 1, y_max - y_min + 1)
    grid_width = int(canvas_width / d)
    canvas_width = grid_width * d
    win = Tk()
    win.title(title)
    win.geometry(str(canvas_width) + "x" + str(canvas_width) + "+100+100")
    can = Canvas(win, width=canvas_width, height=canvas_width, bg="white")
    for i in range(0, canvas_width, grid_width):
        can.create_line((0, i), (canvas_width, i))

    for i in range(0, canvas_width, grid_width):
        can.create_line((i, 0), (i, canvas_width))
    ft = tkFont.Font(root=win, family='Fixdsys', size=int(20 / 4), weight=tkFont.BOLD)
    if grid_width >= show_mark_min_width:
        for x in range(0, d):
            for y in range(0, d):
                s = position2mark((x + x_min, y_max - y))
                can.create_text(x * grid_width + grid_width / 2, y * grid_width + grid_width / 2, text=s,
                                font=ft)
    sz = len(path)
    for i in range(0, sz - 1):
        p1 = mark_screen_position(path[i], x_min, y_max)
        p2 = mark_screen_position(path[i + 1], x_min, y_max)
        can.create_line((p1[0] * grid_width + grid_width / 2, p1[1] * grid_width + grid_width / 2),
                        (p2[0] * grid_width + grid_width / 2, p2[1] * grid_width + grid_width / 2), fill="red", width=3)
        if i == 0: {
            can.create_oval(
                (p1[0] * grid_width + grid_width / 2 - point_r, p1[1] * grid_width + grid_width / 2 - point_r,
                 p1[0] * grid_width + grid_width / 2 + point_r, p1[1] * grid_width + grid_width / 2 + point_r),
                fill="blue")
        }
        can.create_oval((p2[0] * grid_width + grid_width / 2 - point_r, p2[1] * grid_width + grid_width / 2 - point_r,
                         p2[0] * grid_width + grid_width / 2 + point_r, p2[1] * grid_width + grid_width / 2 + point_r),
                        fill="blue")
    can.pack()
    win.mainloop()


# run point
random.seed()
S = random.randint(MIN, MAX)
D = random.randint(MIN, MAX)
while (S == D):
    D = random.randint(MIN, MAX)
g = 1
fp = open("1.txt", "w", encoding="utf-8")

# 初代
paths = init(PATH_COUNT)
fits = fitness()  # 适应度计算
output(g, fp)
g = g + 1

origin_best_path = []

for i in range(ITERATION_COUNT):
    pArr = cumulative_probability(fits)  # 累计概率
    paths = choose(pArr, PATH_COUNT - 1)  # 选择
    paths = cross()  # 交叉
    paths = variation()  # 变异
    paths.append(best_path)
    if i == 0:
        origin_best_path = copy.deepcopy(best_path)
    fits = fitness()  # 适应度计算
    output(g, fp)
    g = g + 1
fp.flush()
fp.close()

fp = open("2.txt", "w", encoding="utf-8")
fp.write("最大适应度值列表：\n")
for e in res_fit:
    fp.write(format(e, ".2f"))
    fp.write(" ")
fp.flush()
fp.close()

t1 = threading.Thread(target=show, args=(origin_best_path, "初代最好的路径"))
t2 = threading.Thread(target=show, args=(best_path, "最好的路径"))
t1.start()
t2.start()
t1.join()
t2.join()