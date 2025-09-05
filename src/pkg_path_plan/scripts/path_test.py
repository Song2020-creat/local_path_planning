import cv2
import random
import threading
import matplotlib.pyplot as plt
import numpy as np
from duplicity.asyncscheduler import threading


class PathVisualizer(threading.Thread):
	def __init__(self, obstacle, goal, shared_path, tree_path,rand_area, lock):
		super().__init__()
		self.obstacle_list = obstacle
		self.orign = (0, 0)
		self.goal = goal
		self.path = shared_path
		self.tree = tree_path
		self.rand_area = rand_area
		self.lock = lock
		self.daemon = True
		self.event = threading.Event()

	def run(self):
		plt.ion()
		fig, ax = plt.subplots()
		while True:
			if self.event.isSet():
				break
			ax.clear()
			with self.lock:
				if self.obstacle_list:
					for (ox, oy) in self.obstacle_list:
						# circle = plt.Circle((ox, oy), 3, color="r")
						# ax.add_patch(circle)
						rect = plt.Rectangle((oy - 3.0, ox - 3.0), 6.0, 6.0, color="r")
						ax.add_patch(rect)
					# xs,ys = zip(*self.obstacle_list)
					# ax.scatter(ys,xs,s=5,c='red',label='Obstacles')

				if self.path:
					px = [x for (x, y) in self.path]
					py = [y for (x, y) in self.path]
					ax.plot(py, px, 'k', linewidth=1.5, label='Fitting Path')
				# if self.tree:
				# 	px = [x for (x, y) in self.tree]
				# 	py = [y for (x, y) in self.tree]
				# 	ax.plot(py, px, '-.g', linewidth=1, label='Orin Path')
				for node in self.tree:
					if node.parent:
						plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "--g", linewidth=1.0)

			ax.plot(self.orign[0], self.orign[1], "bs", label='Start')
			ax.plot(self.goal[1], self.goal[0], "gs", label='Goal')

			plt.xlim(self.rand_area[0], self.rand_area[1])
			plt.ylim(self.rand_area[0], self.rand_area[1])
			ax.set_aspect("equal")
			ax.grid(True)
			ax.set_title("Real Time Path Planning")
			ax.legend()
			plt.pause(0.1)


def generate_obstacles(map_size=70, num_lines=4, mode='mixed'):
	"""
    在栅格地图上生成障碍物，行/列分布
    -map_size: 地图边长
    -num_line: 要生成多少行/列障碍
    -mode: 'row': 行障碍; 'col': 列障碍 'mixed': 各一半
    """
	obstacles = []
	half_size = int(map_size / 2)
	length = random.randint(20, 30)
	orig = random.randint(-half_size, half_size - length)
	length = 31
	orig = 5
	positions = np.arange(orig, orig + length, 3)
	# positions2 = np.arange(-36, -14, 3)
	if mode in ('row', 'mixed'):
		row_ys = np.random.choice(range(-half_size, half_size), num_lines, replace=False)
		row_ys = [10,25]
		for y in row_ys:
			for x in positions:
				obstacles.append((x, y))
			# for x in positions2:
			# 	obstacles.append((x, y))
	if mode in ('col', 'mixed'):
		col_xs = np.random.choice(range(-half_size, half_size), num_lines, replace=False)
		length = 30
		orig = -20
		positions = np.arange(orig, orig + length, 3)
		# positions2 = np.arange(-35, -25, 3)
		col_xs = [-15,5]
		for x in col_xs:
			for y in positions:
				obstacles.append((x, y))
			# for y in positions2:
			# 	obstacles.append((x, y))

	return obstacles

def generate_random_goal_on_edge(map_size=84, edge_width=7):
	"""
    从边缘为edge_width的地图边界中，随机生成一个终点位置
    """
	half = map_size / 2
	inner = half - edge_width
	while True:
		x = np.random.uniform(-half, half)
		y = np.random.uniform(-half, half)
		if abs(x) >= inner or abs(y) >= inner:
			return x, y


if __name__ == "__main__":
	from AStarPath_WS import A_star_search
	import time
	from curve_spline import CurveSpline
	count = 0
	kernalInt = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
	curve = CurveSpline()
	orin_path = []
	barrier_list = []
	shared_path = []
	goal = (40, 40)
	shared_lock = threading.Lock()
	vis_thread = PathVisualizer(barrier_list.copy(), goal, shared_path, orin_path,
									 (-42, 42), shared_lock)
	vis_thread.start()

	while count < 50:
		pathmap = np.zeros((84, 84), dtype=np.uint8)
		cloud_points = generate_obstacles(map_size=84)
		his_cloud = np.asarray(cloud_points)
		try:
			grid_coords = his_cloud
			grid_coords[:, 0] = 42 + grid_coords[:, 0]
			grid_coords[:, 1] = 42 + grid_coords[:, 1]
		except Exception as e:
			print("建图转换异常： %s", e)
			continue

		# 遍历点云，构图
		for point in grid_coords:
			lx = point[0]
			ly = point[1]
			try:
				if (84 - 7) > lx >= 7 <= ly < (84 - 7):
					pathmap[ly, lx] = 100
			except Exception as e:
				print("障碍物越界异常：%s", e)
		try:
			# 边缘膨胀
			pathmap = cv2.dilate(src=pathmap, kernel=kernalInt, iterations=1,
								 borderType=cv2.BORDER_CONSTANT,
								 borderValue=0)
			pathmap[int(84 / 2) - 1:int(84 / 2) + 1,
			int(84 / 2) - 1:int(84 / 2) + 1] = 0
		except Exception as e:
			print("膨胀越界异常： %s", e)
		apoints_tran = A_star_search(pathmap, [42, 42], [82, 82])

		if apoints_tran == "A* failed":
			print("********************Astar规划失败")
			time.sleep(0.05)
			continue
		else:
			print("A*路径长度为： %d", len(apoints_tran))
			# rate.sleep()
			# 进行Bspline曲线平滑路径
			apoints = []
			local_path_orin = []
			a = []
			for node in apoints_tran:
				list_node = [node['x_s'], node['y']]
				a.append(list_node)
				local_path_orin.append((list_node[0] - 42,list_node[1] - 42))
			path_array = np.asarray(a)
			(x, y) = (path_array[:, 0], path_array[:, 1])
			try:
				fitting_points = curve.BSpline(control_points=[x, y], number=int(len(apoints_tran)),
													sm=0.8)
				for i in range(fitting_points[0].shape[0]):
					apoints.append({'x_s': round(fitting_points[0][i], 4), 'y': round(fitting_points[1][i], 4)})
			except Exception as e:
				print("路径平滑失败，%s", e)
				apoints = apoints_tran
			local_path_fit = []

			if len(apoints) > 0:
				for astarpoint in apoints:
					# 局部路径转为经纬度点
					y_real = (astarpoint['y'] - 42)
					x_real = (astarpoint['x_s'] - 42)
					local_path_fit.append((x_real, y_real))
			with shared_lock:
				vis_thread.obstacle_list = cloud_points.copy()
				vis_thread.goal = (40, 40)
				shared_path.clear()
				orin_path.clear()
				if local_path_fit:
					shared_path.extend(local_path_fit.copy())
					orin_path.extend(local_path_orin.copy())

		count = count + 1
		time.sleep(0.5)
	vis_thread.event.set()
