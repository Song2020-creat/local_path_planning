import math
import cv2
import numpy as np
#heapq 是一个堆，优先级队列，弹出最小的元素
import heapq

from matplotlib.pyplot import disconnect


class Cell:
    def __init__(self):
        self.parent_i = 0
        self.parent_j = 0
        self.f = float('inf')
        self.g = float('inf')
        self.h = 0        

#check if a cell is valid
def is_valid(y_row,x_col,ROW,COL):
    return (y_row >=0) and (y_row < ROW) and (x_col >= 0) and (x_col < COL)

def is_collision(s_start, s_end,grid):
    """
    check if the line segment (s_start, s_end) is collision.
    :param s_start: start node
    :param s_end: end node
    :return: True: is not in collision / False: collision
    """

    if is_unblocked(grid, s_end[0], s_end[1]):
        if math.hypot((s_start[0] -s_end[0]),(s_start[1] - s_end[1])) > 1.2:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if is_unblocked(grid, s1[0], s1[1]) or is_unblocked(grid, s2[0], s2[1]):
                return True
            else:
                return False
        else:
            return True
    return False


def is_unblocked(grid, row, col):
    if grid[row][col] == 0:
        return True
    else:
        return False


def is_destination(row,col,dest):
    return row == dest[0] and col == dest[1]

# Euclidean distance
def calculate_h_value_1(row,col,dest):
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5
# Diagonal distance
def calculate_h_value_2(row,col,dest):
    dx = abs(row - dest[0])
    dy = abs(col - dest[1])
    return (dx + dy + (math.sqrt(2)-2) * min(dx,dy))
# Manhattan distance
def calculate_h_value_3(row,col,dest):
    dy = abs(row - dest[0])
    dx = abs(col - dest[1])
    return (dx + dy)

def trace_path(cell_details, dest):
    # print("The Path is ")
    path = []
    points = []
    row = dest[0]
    col = dest[1]
    #判断是否到终点了
    while not (cell_details[row][col].parent_i == row and 
               cell_details[row][col].parent_j == col):
        path.append((row,col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col
        
    # path.append((row,col))
    path.reverse()
    for iteam in path:
        points.append({'x_s':iteam[1],'y':iteam[0]})
    return points  
    # for i in path:
    #     print("->", i, end="")
    # print()
        
def A_star_search(grid,src,dest):
    ROW = grid.shape[0]
    COL = grid.shape[1]
    #判断起点和终点的合理性
    # if not is_unblocked(grid,dest[0],dest[1]):
    #     try:
    #         grid[dest[0]-1:dest[0]+1,dest[1]-1:dest[1]+1] = 0
    #     except Exception:
    #         return "destion wrong"
    if not is_unblocked(grid, src[0], src[1]):
        directions = []
        for r in range(1, 2):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    directions.append((dx, dy))
        new_start_array = []
        for dx, dy in directions:
            y_start_new = src[0] + dy
            x_start_new = src[1] + dx
            if is_unblocked(grid, y_start_new, x_start_new):
                new_start_array.append((y_start_new, x_start_new))
        if new_start_array:
            dis = float('inf')
            item = 0
            for index, point in enumerate(new_start_array):
                point_dis = (math.hypot(src[0] - point[0], src[1] - point[1]) +
                             math.hypot(dest[0] - point[0], dest[1] - point[1]))
                if point_dis < dis:
                    dis = point_dis
                    item = index
            src[0] = new_start_array[item][0]
            src[1] = new_start_array[item][1]
        else:
            try:
                grid[src[0] - 1:src[0] + 1, src[1] - 1:src[1] + 1] = 0
            except Exception:
                return "start wrong"
    #判断是否为终点
    if is_destination(src[0],src[1],dest):
        points = []
        return points.append({'x_s':dest[1], 'y':dest[0]})
        # print("We are already at the destination")

    vector_1 = (dest[1] - src[1], dest[0] - src[0])

    #标记位置是否被访问过
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    #存储单元的代价以及前一个单元的位置信息
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]
    
    i = src[0]
    j = src[1]
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0
    #起点的前一个点是本身
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j
    
    #通过堆来找代价最小点
    open_list = []
    heapq.heappush(open_list,(0.0,i,j))
    found_dest = False
    
    while len(open_list) > 0:
        p = heapq.heappop(open_list)
        # print(p)
        i = p[1]
        j = p[2]
        closed_list[i][j] = True
        vector_2 = (dest[1] - p[2], dest[0] - p[1])
        vec_cross = abs(vector_1[0] * vector_2[1] - vector_1[1] * vector_2[0])
        #foe each direction, check the successors
        directions = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,-1),(-1,1)]
        # directions = directions + directions13

        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]
            start = (i,j)
            end = (new_i,new_j)
            #if the successor is valid,unblocked,and not visited
            # if is_valid(new_i, new_j, ROW, COL) and is_unblocked(grid,new_i,new_j) and not closed_list[new_i][new_j]:
            if is_valid(new_i,new_j,ROW,COL) and is_collision(start,end,grid)  and not closed_list[new_i][new_j]:
                if is_destination(new_i,new_j,dest):
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    # print("The destination cell is found")
                    #Trace and print the path form source to destination
                    path_points = trace_path(cell_details,dest)
                    found_dest = True
                    return path_points
                else:
                    g_value = round(math.sqrt(dir[0]**2 + dir[1]**2),4)
                    # print("g_value",g_value)
                    g_new = cell_details[i][j].g + g_value
                    # print("g_new",g_new)
                    h_new = round(calculate_h_value_2(new_i,new_j,dest),4)  + vec_cross*0.02
                    f_new = g_new + h_new
                    
                    #the cell is not in the open list or the new value is smaller
                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                        #add the cell to the open_list
                        heapq.heappush(open_list,(f_new,new_i,new_j))
                        # print("f_new",(f_new,new_i,new_j))

                        #update the cell details
                        cell_details[new_i][new_j].f = f_new
                        cell_details[new_i][new_j].g = g_new
                        cell_details[new_i][new_j].h = h_new
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
    if not found_dest:
        # print("Failed to find the destination cell")
        return "A* failed"

def douglas_peucker(points,epsilon):
    if len(points) < 3:
        return points
    start, end = points[0] , points[-1]
    line_vec = np.subtract(end,start)
    line_vec_norm = line_vec / np.linalg.norm(line_vec)

    d_max = 0
    index = 0
    for i in range(1,len(points)-1):
        vec = np.subtract(points[i],start)
        proj = np.dot(vec,line_vec_norm) * line_vec_norm
        d = np.linalg.norm(vec-proj)
        if d > d_max:
            d_max = d
            index = i
    if d_max > epsilon:
        left = douglas_peucker(points[:index+1],epsilon)
        right = douglas_peucker(points[index:],epsilon)
        return np.vstack((left[:-1],right))
    else:
        return np.array([start,end])

def sliding_window_simplify(points,d_min):
    simplified = [points[0]]
    last_point = points[0]

    for point in points[1:]:
        distance = np.linalg.norm(point - last_point)

        if distance >= d_min:
            simplified.append(point)
            last_point = point

    return np.array(simplified)


if __name__ == "__main__":

    # from curve_fitting import evaluateBezier
    import matplotlib.pyplot as plt
    from scipy import interpolate
    import time

    # 定义地图
    npmap = np.zeros((84, 84), dtype=np.uint8)  # 地图，0是空位，255是障碍

    # 设置起点和终点
    start_point = [12, 15]
    end_point = [23,12]
    # end_point = [10, 15]
    # 设置障碍点
    for i in range(5, 20):
        # npmap[i][20] = 255
        npmap[15][i] = 255
    for i in range(12, 18):
        npmap[18][i] = 255
    for i in range(18, 21):
        npmap[i][10] = 255
    for i in range(18, 21):
        npmap[i][18] = 255

    # 运行算法
    start_time = time.time()
    nodes = A_star_search(npmap, (start_point[0], start_point[1]), end_point)
    # 打印结果
    print(nodes)
    print(round(time.time() - start_time, 5))
    a = [[start_point[1], start_point[0]]]
    # a = []
    for node in nodes:
        list_node = [node['x_s'], node['y']]
        a.append(list_node)
    local_path = np.asarray(a)

    (x, y) = (local_path[:, 0], local_path[:, 1])

    obs_x1 = np.arange(5, 21, 1)
    obs_y1 = np.array([15 for i in range(5, 21)])
    obs_y3 = np.array([18 for i in range(11, 19)])
    obs_x3 = np.arange(11, 19, 1)
    obs_y2 = np.arange(18, 21, 1)
    obs_x2 = np.array([10 for i in range(18, 21)])
    obs_y4 = np.arange(18, 21, 1)
    obs_x4 = np.array([18 for i in range(18, 21)])

    plt.figure()
    plt.plot(x, y,'r', start_point[1], start_point[0], 'bo', end_point[1], end_point[0], 'yo',
             obs_x1, obs_y1, 'g--', obs_x2, obs_y2, 'g--', obs_x3, obs_y3, 'g--', obs_x4, obs_y4, 'g--')
    plt.legend(['Path'], loc='best')
    # plt.axis([min(x_s)-1, max(x_s)+1, min(y)-1, max(y)+1])
    plt.title('Curve fitting')
    plt.show()
    
                        
                    