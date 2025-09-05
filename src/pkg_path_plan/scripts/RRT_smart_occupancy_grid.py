import cv2
import math
import random
import threading
import matplotlib.pyplot as plt
import numpy as np
import time
from scipy.spatial import KDTree
from shapely.geometry import LineString, box
import os, sys
script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)
from curve_spline import CurveSpline

# parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# sys.path.insert(0, parentdir)
# sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# from flog import Logger
class PathVisualizer(threading.Thread):
	def __init__(self,obstacle,goal,shared_path,rand_area,lock):
		super().__init__()
		self.obstacle_list = obstacle
		self.orign = (0,0)
		self.goal = goal
		self.path = shared_path
		self.rand_area = rand_area
		self.lock = lock
		self.daemon = True
	def run(self):
		plt.ion()
		fig, ax = plt.subplots()
		while True:
			ax.clear()
			with self.lock:
				if len(self.obstacle_list) > 0:
					for (ox, oy) in self.obstacle_list:
						# circle = plt.Circle((ox, oy), 3, color="r")
						# ax.add_patch(circle)
						rect = plt.Rectangle((ox - 1.5, oy - 1.5), 3.0, 3.0, color="r")
						ax.add_patch(rect)

					# xs,ys = zip(*self.obstacle_list)
					# ax.scatter(xs,ys,s=5,c='red',label='Obstacles')

				if self.path:
					px = [x for (x, y) in self.path]
					py = [y for (x, y) in self.path]
					ax.plot(px, py, '-.k', linewidth=2,label='Path')

			ax.plot(self.orign[0], self.orign[1], "bs", label='Start')
			ax.plot(self.goal[0], self.goal[1], "gs", label='Goal')

			plt.xlim(self.rand_area[0], self.rand_area[1])
			plt.ylim(self.rand_area[0], self.rand_area[1])
			ax.set_aspect("equal")
			ax.grid(True)
			ax.set_title("Real Time RRT* Path Planning")
			ax.legend()
			plt.pause(0.1)

class Node:
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.parent = None
		self.cost = 0.0

class RRTSmart:
	def __init__(self, goal, occupancy_grid, rand_area,resolution):
		"""
		Args:
			goal:  tuple (x,y,z) E,N
			occupancy_grid: np.asarray
			rand_area: tuple (min,max)
			resolution: float

		"""
		self.start = Node(0, 0)
		self.end = Node(goal[0], goal[1])
		self.point_list = []
		self.min_rand = rand_area[0] + 1
		self.max_rand = rand_area[1] - 1
		self.resolution = resolution
		self.map_size = int(self.max_rand * 2 / self.resolution)
		self.occupancy_grid = occupancy_grid
		if goal[1] > 0 and goal[0] > 0:
			self.ymin = 0.0
			self.ymax = self.max_rand
			self.xmin = 0.0
			self.xmax = self.max_rand
		elif goal[1] > 0 > goal[0]:
			self.ymin = 0.0
			self.ymax = self.max_rand
			self.xmin = self.min_rand
			self.xmax = 0.0
		elif goal[1] < 0 and goal[0] < 0:
			self.ymin = self.min_rand
			self.ymax = 0
			self.xmin = self.min_rand
			self.xmax = 0

		elif goal[1] < 0 < goal[0]:
			self.xmin = 0
			self.xmax = self.max_rand
			self.ymin = self.min_rand
			self.ymax = 0
		if abs(goal[0]) <= 10:
			self.xmin = self.min_rand / 2
			self.xmax = self.max_rand / 2
			if goal[1] > 0:
				self.ymin = 0
				self.ymax = self.max_rand
			elif goal[1] < 0:
				self.ymin = self.min_rand
				self.ymax = 0
		if abs(goal[1]) <= 10:
			self.ymin = self.min_rand / 2
			self.ymax = self.max_rand / 2
			if  goal[0] > 0:
				self.xmin = 0
				self.xmax = self.max_rand
			elif goal[0] < 0:
				self.xmin = self.min_rand
				self.xmax = 0

		self.expand_dis = self.max_rand * 0.15  #树上的节点扩展到新节点的距离，越小树支越密集
		self.radios = 3.0   # 智能采样半径
		self.goal_sample_rate = 20 # 采样到终点的概率
		self.max_iter = 5000 # 最大迭代次数
		self.node_list = []
		self.smooth_path = CurveSpline()
		self.time_thread = 0.5
		self.beacons = []
		self.direct_cost_old = float('inf')

	def grid_map_project(self,node):
		lx = round(node.x / self.resolution)
		ly = round(node.y / self.resolution)
		map_row = int(self.map_size / 2 + lx)
		map_col = int(self.map_size / 2 + ly)
		map_node = (map_row,map_col)
		return map_node

	def smart_sample(self,obs_beacons = None):
		"""
		智能采样，大概率在已有路径附近采样
		"""
		if obs_beacons and random.random() < 0.7:
			base = random.choice(obs_beacons)
			theta = random.uniform(0, 2 * math.pi)
			r = random.uniform(0,self.radios)
			x = base[0] + r * math.cos(theta)
			y = base[1] + r * math.sin(theta)
			x = min(x,41)
			x = max(x,-42)
			y = min(y, 41)
			y = max(y, -42)
			return Node(x,y)
		else:
			return self.get_random_node()

	def is_near_goal(self,new_node,local_goal):
		"""找到一条起点与终点之间的可行路径"""
		dx = new_node.x - local_goal.x
		dy = new_node.y - local_goal.y
		dist = (dx **2 + dy ** 2) ** 0.5

		if dist >= 5.0:
			return False
		new_node_map = self.grid_map_project(new_node)
		local_goal_map = self.grid_map_project(local_goal)
		return self.collision_check_ray(new_node_map, local_goal_map)

	@staticmethod
	def compute_shifted_goal(dist):
			x_local = dist[0] * math.cos(dist[1])
			y_local = dist[0] * math.sin(dist[1])
			x_local = min(x_local, 41)
			x_local = max(x_local, -42)
			y_local = min(y_local, 41)
			y_local = max(y_local, -42)
			return Node(x_local,y_local)

	def plan(self):
		s_time = time.time()
		start_node_map = self.grid_map_project(self.start)
		if not self.start_collision_check(start_node_map):
			return "start wrong"
		end_node_map = self.grid_map_project(self.end)
		if not self.start_collision_check(end_node_map):
			return "end wrong"
		self.node_list.append(self.start)
		self.point_list.append((self.start.x,self.start.y))

		init_path_flag = False
		c_best = float('inf')
		std_c = float('inf')
		c_best_history = []
		n = 0

		local_goal = self.end
		# theta = math.atan2(self.end.y, self.end.x)
		# dist = math.hypot(self.end.x, self.end.y)
		# dist = [dist, dist + 3.0, dist - 3.0]
		# angle = [theta + math.radians(10), theta - math.radians(10)]
		# dist_array = []
		# for i in dist:
		# 	for j in angle:
		# 		dist_array.append((i,j))
		# count = 0

		for num in range(0,self.max_iter,1):
			if (num - n) % 2 == 0:
				rnd_node = self.smart_sample(self.beacons)
			else:
				rnd_node = self.get_random_node()
			if num == self.max_iter - 1:
				print(f"RRT*循环到最大次数了!!!!!!{num}！")
			nearest_ind = self.get_nearest_kdtree_index(self.point_list,rnd_node)
			nearest_node = self.node_list[nearest_ind]  #找到最近的点
			new_node, extend_length = self.steer(nearest_node, rnd_node, self.expand_dis) #最近的点向新点扩展
			if extend_length < 1.0:
				continue
			nearest_node_map = self.grid_map_project(nearest_node)
			new_node_map = self.grid_map_project(new_node)
			if self.collision_check_ray(nearest_node_map,new_node_map):
				new_node.parent = nearest_node
				new_node.cost = nearest_node.cost + extend_length

				if c_best < float('inf'):
					# print("RRT*")
					near_inds = self.find_near_kdtree_nodes(new_node,15)
					new_node = self.choose_parent(new_node, near_inds) #重新计算代价，确认新的节点
					self.rewire(new_node, near_inds)  # 节点重连

				self.node_list.append(new_node)
				self.point_list.append((new_node.x,new_node.y))

				if not init_path_flag and self.is_near_goal(new_node,local_goal):
					self.expand_dis = 4.0
					self.end = local_goal
					self.end.parent = new_node
					init_path_flag = True
					n = num
					total_cost = new_node.cost + self.calc_distance_and_angle(new_node, self.end)[0]
					if total_cost < c_best:
						c_best = total_cost
						self.end.cost = total_cost

				if init_path_flag:
					self.path_optimization(new_node)
					# print("代价",self.direct_cost_old)
					c_best_history.append(self.direct_cost_old)
					if len(c_best_history) > 10:
						std_c = np.std(c_best_history[-10:])
						del c_best_history[0]

					if std_c <= 0.05: #0.3
						# print("标准差小于阈值,退出规划",std_c)
						break

			delta_time = round(time.time() - s_time, 2)
			# if self.time_thread * 0.7 <= delta_time < self.time_thread and c_best == float('inf'):
			# 	if count < len(dist_array):
			# 		local_goal = self.compute_shifted_goal(dist_array[count])
			# 		# self.region_edge_confirm(local_goal)
			# 		count += 1
			# 		s_time = time.time()
			# 	else:
			# 		# print("中继点偏移规划失败")
			# 		return None
			if delta_time >= self.time_thread:
				print("rrt 超时！！！！",delta_time,num)
				break

		path = self.generate_path()
		if path is None or len(path) == 1:
			return None
		# return path
		path = self.reline_path(path)
		path_dense = self.smooth_path.linear_interpolation_2d(path,step=1.0)
		safe_path = self.check_and_repaire_path(path_dense)
		safe_path = self.smooth_path.simple_spline(safe_path)
		if len(safe_path) >= 6:
			(x_s, y_s) = zip(*safe_path)
			try:
				smooth_path = self.smooth_path.BSpline([x_s, y_s], len(safe_path), sm=2.0)
				apoints = []
				for i in range(smooth_path[0].shape[0]):
					apoints.append((round(smooth_path[0][i], 2), round(smooth_path[1][i], 2)))
				return apoints[1:]
			except Exception as e:
				print(f"rrt 路径平滑失败：{e}")
				return safe_path[1:]
		else:
			return safe_path[1:]

	def path_optimization(self,node):
		direct_cost_new = 0.0
		node_end = self.end
		node_s = node
		while node_s.parent:
			node_parent = node_s.parent
			dis = self.calc_distance_and_angle(node_parent,self.end)[0]
			if dis <= 1.2:
				node_end.parent = node_parent
			else:
				node_parent_map = self.grid_map_project(node_parent)
				node_end_map = self.grid_map_project(node_end)
				if self.collision_check_ray(node_parent_map,node_end_map):
					node_end.parent = node_parent
				else:
					direct_cost_new += self.calc_distance_and_angle(node_s,node_end)[0]
					node_end = node_s
			node_s = node_s.parent
		direct_cost_new += self.calc_distance_and_angle(node_s, node_end)[0]
		if direct_cost_new < self.direct_cost_old:
			self.direct_cost_old = direct_cost_new
			self.update_beacons()

	def update_beacons(self):
		node = self.end.parent
		# beacons = []
		self.beacons.clear()
		while node.parent:
			if self.occupancy_grid[int(node.x)][int(node.y)] == 0:
				self.beacons.append((node.x,node.y))
			node = node.parent
		# self.beacons = beacons

	def check_and_repaire_path(self,path):
		if len(path) <= 1:
			return path
		safe_path = [path[0]]
		for i in range(1,len(path)):
			p1 = safe_path[-1]
			p2 = path[i]
			p1_node = Node(p1[0],p1[1])
			p2_node = Node(p2[0],p2[1])
			p1_node_map = self.grid_map_project(p1_node)
			p2_node_map = self.grid_map_project(p2_node)
			flag = self.collision_check_ray(p1_node_map,p2_node_map)
			if not flag:
				fixed = self.try_by_pass(p1,p2)
				if fixed is not None:
					safe_path.extend(fixed[1:])
				else:
					safe_path.append(p2)
					# print("无法修复")
			else:
				safe_path.append(p2)
		return safe_path

	def try_by_pass(self,p1,p2,offset = 3.0):
		"""尝试左右绕过障碍物"""
		p1, p2 = np.array(p1),np.array(p2)
		direction = p2 - p1
		perp = np.array([-direction[1],direction[0]])
		perp = perp / np.linalg.norm(perp)

		for sign in [1,-1]:
			bypass_point = p1 + direction * 0.5 + sign * perp * offset
			bypass_point_node = Node(bypass_point[0],bypass_point[1])


			if self.occupancy_grid[int(bypass_point_node.x)][int(bypass_point_node.y)] == 0:
				return [p1.tolist(), bypass_point.tolist(), p2.tolist()]
		return None


	def steer(self, from_node, to_node, extend_length=float("inf")):
		new_node = Node(from_node.x, from_node.y)
		d, theta = self.calc_distance_and_angle(new_node, to_node)
		length = min(extend_length, d)
		new_node.x += length * math.cos(theta)
		new_node.y += length * math.sin(theta)
		return new_node, length

	def get_random_node(self):
		if random.randint(0, 100) > self.goal_sample_rate:
			rnd = Node(random.uniform(self.xmin, self.xmax),
					   random.uniform(self.ymin, self.ymax))
		else:
			rnd = Node(self.end.x, self.end.y)
		return rnd
	def get_random_node_all_map(self):
		self.expand_dis = self.max_rand * 0.5
		if random.randint(0, 100) > self.goal_sample_rate:
			rnd = Node(random.uniform(self.min_rand, self.max_rand),
					   random.uniform(self.min_rand, self.max_rand))
		else:
			rnd = Node(self.end.x, self.end.y)
		return rnd
	@staticmethod
	def get_nearest_node_index(node_list, rnd_node):
		dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
		return dlist.index(min(dlist))
	@staticmethod
	def get_nearest_kdtree_index(node_array,rnd_node):
		node_array = np.asarray(node_array)
		rnd_node = np.asarray([rnd_node.x, rnd_node.y])
		kdtree = KDTree(node_array)
		distance, index = kdtree.query(rnd_node, k=1)
		return index
	@staticmethod
	def calc_distance_and_angle(from_node, to_node):
		dx = to_node.x - from_node.x
		dy = to_node.y - from_node.y
		return math.hypot(dx, dy), math.atan2(dy, dx)

	def start_collision_check(self,node):
		row = int(node[0])
		col = int(node[1])
		if self.occupancy_grid[row][col] == 0:
			return True
		else:
			try:
				self.occupancy_grid[row-1:row+1,col-1:col+1] = 0
			except Exception as e:
				# print("去除失败: %s",e)
				return False

	def collision_check_ray(self,start_node,end_node):
		"""DDA射线追踪算法"""
		if self.occupancy_grid[int(end_node[0])][int(end_node[1])] > 0:
			return False

		dx = end_node[0]  - start_node[0]
		dy = end_node[1] - start_node[1]
		steps = max(abs(dx),abs(dy))
		if steps == 0:
				return True
		x_inc = dx / steps
		y_inc = dy / steps
		x = start_node[0]
		y = start_node[1]
		for _ in range(int(steps) + 1):
			if self.occupancy_grid[int(x)][int(y)] > 0:
				return False
			x = x + x_inc
			y = y + y_inc
		return True


	def find_near_nodes(self, new_node):
		nnode = len(self.node_list)
		r = 65.0 * math.sqrt((math.log(nnode) / nnode))
		dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in self.node_list]
		near_inds = [i for i, d in enumerate(dlist) if d <= r ** 2]
		return near_inds

	def find_near_kdtree_nodes(self,new_node,max_neighbors=None):
		nnode = len(self.point_list)
		radios = 30.0 * math.sqrt((math.log(nnode) / nnode))
		radios = min(radios,2.0)
		new_node_array = np.asarray([new_node.x, new_node.y])
		node_array = np.asarray(self.point_list)
		kdtree = KDTree(node_array)
		near_inds = kdtree.query_ball_point(new_node_array,radios)
		# self.len.append(len(near_inds))
		if max_neighbors is not None and len(near_inds) > max_neighbors:
			dis,near_inds = kdtree.query(new_node_array,k = max_neighbors)
		return near_inds

	def choose_parent(self, new_node, near_inds):
		if len(near_inds) <= 0:
			return new_node
		else:
			costs = []
			for i in near_inds:
				near_node = self.node_list[i]
				near_node_map = self.grid_map_project(near_node)
				new_node_map = self.grid_map_project(new_node)
				if self.collision_check_ray(near_node_map,new_node_map):
					costs.append(near_node.cost + self.calc_distance_and_angle(near_node, new_node)[0])
				else:
					costs.append(float("inf"))
			min_cost = min(costs)
			min_ind = near_inds[costs.index(min_cost)]
			if min_cost == float("inf"):
				return new_node
			else:
				new_node.cost = min_cost
				new_node.parent = self.node_list[min_ind]
				return new_node

	def rewire(self, new_node, near_inds):
		for i in near_inds:
			near_node = self.node_list[i]
			# edge_node = self.steer(new_node, near_node)
			near_node_map = self.grid_map_project(near_node)
			new_node_map = self.grid_map_project(new_node)
			if not self.collision_check_ray(new_node_map, near_node_map):
				continue
			improved_cost = new_node.cost + self.calc_distance_and_angle(new_node, near_node)[0]
			if near_node.cost > improved_cost:
				near_node.parent = new_node
				near_node.cost = improved_cost

	def reline_path(self, path):
		smoothed = [path[0]]
		i = 0
		while i < len(path) - 1:
			j = len(path) - 1
			while j > i + 1:
				# if self.is_line_collision(path[i],path[j],step=1.0):
				s_node = Node(path[i][0], path[i][1])
				e_node = Node(path[j][0], path[j][1])
				s_node_map = self.grid_map_project(s_node)
				e_node_map = self.grid_map_project(e_node)
				if self.collision_check_ray(s_node_map,e_node_map):
					break
				j -= 1
			smoothed.append(path[j])
			i = j
		return smoothed

	def generate_path(self):
		if self.end.parent is None:
			if self.node_list:
				node = self.node_list[-1]
				distance = self.calc_distance_and_angle(node,self.end)[0]
				if len(self.node_list) <= 3 and 5 < distance:
					return None
				node_map = self.grid_map_project(node)
				end_map = self.grid_map_project(self.end)
				if self.collision_check_ray(node_map,end_map):
					self.end.parent = node
					node = self.end
			else:
				return None
		else:
			node = self.end
		path = []
		while node.parent is not None:
			path.append([node.x, node.y])
			node = node.parent
		path.append([node.x, node.y])
		return path[::-1]

def generate_obstacles(map_size = 70,num_lines=4,mode='mixed'):
	"""
	在栅格地图上生成障碍物，行/列分布
	-map_size: 地图边长
	-num_line: 要生成多少行/列障碍
	-mode: 'row': 行障碍; 'col': 列障碍 'mixed': 各一半
	"""
	obstacles = []
	half_size = int(map_size / 2)
	length = random.randint(20,30)
	orig = random.randint(-half_size,half_size - length)
	length = 20
	orig = 20
	positions = np.arange(orig,orig + length,3)
	if mode in ('row','mixed'):
		row_ys = np.random.choice(range(-half_size,half_size),num_lines,replace=False)
		row_ys = [15]
		for y in row_ys:
			for x in positions:
				obstacles.append((x,y))
	if mode in ('col', 'mixed'):
		col_xs = np.random.choice(range(-half_size, half_size), num_lines, replace=False)
		col_xs = [14.5]
		for x in col_xs:
			for y in positions:
				obstacles.append((x, y))
	return obstacles
def generate_random_goal_on_edge(map_size = 84,edge_width = 7):
	"""
	从边缘为edge_width的地图边界中，随机生成一个终点位置
	"""
	half = map_size / 2
	inner = half - edge_width
	while True:
		x = np.random.uniform(-half,half)
		y = np.random.uniform(-half, half)
		if abs(x) >= inner or abs(y) >= inner:
			return x,y

if __name__ == "__main__":

	start = (0, 0)
	shared_path = []
	goal = (0, 30)
	rand_area = (-42, 42)

	obstacle_list = []
	shared_lock = threading.Lock()
	vis_thread = PathVisualizer(obstacle_list,goal,shared_path,rand_area,shared_lock)
	vis_thread.start()

	# obstacle_list = [
	#     (-12, 8),
	#     (-12, 11),
	#     (-12, 14),
	#     (-12, 17),
	#     (-12, 20),
	#     (-12, 23),
	#     (-12, 26),
	#     (-12, 9),
	#     (-12, 10),
	#     (-12, 12),
	#     (-12, 15),
	#     (-12, 19),
	#     (-12, 18),
	#     (-12, 24),
	#     (14, 0),
	#     (20, -10),
	#     (18, -10),
	#     (16, -10),
	#     (14, -10),
	#     (12, -10),
	#     (10, -10),
	#     (10, -8),
	#     (10, -6),
	#     (10, -4),
	#     (10, -2),
	#     (10, 0),
	#     (10, 2),
	#     (10, 4),
	#     (10, 5),
	#     (10, 6),
	#     (10, 7),
	#     (10, 8),
	#     (10, 11),
	#     (10, 14),
	#     (10, 17),
	#     (10, 20),
	#     (10, 23),
	#     (10, 26),
	#     (15, 20),
	#     (-15, 10),
	#     (16, 10),
	#     (17, 10),
	#     (-10, -10),
	#     (10,  30)
	# ]

	while True:
		start_time = time.time()
		obstacle_list.clear()

		# obstacle_list = generate_obstacles(map_size=70,num_lines=1)
		for _ in range(8):
			ox = random.randint(-35,35)
			oy = random.randint(-35,35)
			obstacle_list.append((ox,oy))
		with shared_lock:
			vis_thread.obstacle_list = obstacle_list
		try:
			obs = obstacle_list.copy()
			his_cloud = np.asarray(obs)
			grid_coords = np.floor_divide(his_cloud[:, :2], 1.0).astype(int)
			grid_coords[:, 0] = 42 + grid_coords[:, 0]
			grid_coords[:, 1] = 42 + grid_coords[:, 1]
		except Exception as e:
			print("建图异常:%s",e)
			continue
		pathmap = np.zeros((84, 84), dtype=np.uint8)
		for point in grid_coords:
			lx = point[0]
			ly = point[1]
			try:
				if (84 - 7) > lx >= 7 <= ly < (84 - 7):
					pathmap[lx, ly] = 100
			except Exception as e:
				print("障碍物越界异常：%s", e)
		try:
			kernalInt = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
			# 边缘膨胀
			pathmap_dilate = cv2.dilate(src=pathmap, kernel=kernalInt, iterations=1,
										borderType=cv2.BORDER_CONSTANT,
										borderValue=0)
			pathmap_dilate[42 - 1:42 + 1,42 - 1:42 + 1] = 0
		except Exception as e:
			print("膨胀异常： %s", e)
			continue
		try:
			display = False
			rrt_star = RRTSmart(goal, pathmap_dilate, rand_area,1.0)
			path = rrt_star.plan()
		except Exception as e:
			print("规划异常: %s",e)
			continue

		print(path)
		if path == "start wrong" or path == "end wrong":
			print("起点/终点周围有障碍物")
			with shared_lock:
				vis_thread.path.clear()
			continue
		elif path is None:
			print("未找到路径")
			with shared_lock:
				vis_thread.path.clear()
			continue
		else:
			delta_time_all = time.time() - start_time
			print("耗时：", delta_time_all)
			# print(path)
			with shared_lock:
				vis_thread.path.clear()
				vis_thread.path = path.copy()
		time.sleep(1.0)
