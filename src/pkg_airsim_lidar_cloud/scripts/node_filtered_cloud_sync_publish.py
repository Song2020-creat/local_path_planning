import time
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
from pkg_netproxy.msg import UavStatus
from std_msgs.msg import Header
from threading import Thread, Lock, Event
import open3d as o3d
from message_filters import ApproximateTimeSynchronizer, Subscriber

def pub_point_cloud2(points2,time_stamp):
	header = Header()
	# header.stamp = rospy.Time.now()
	header.stamp = time_stamp
	header.frame_id = "lidar_link"
	fields = [
		PointField('x', 0,PointField.FLOAT32,1),
		PointField('y', 4, PointField.FLOAT32, 1),
		PointField('z', 8, PointField.FLOAT32, 1),
	]
	return pc2.create_cloud(header,fields,points2.tolist())
def estimate_avg_spacing(pcd,sample_ratio = 0.1):
	"""
	基于随机采样最近点的间距估算
	:param pcd: open3d.geometry.PointCloud
	:param sample_ratio: 采样比例 （0～1）0.02表示抽取2%的点
	:return 平均点的间距（float)
	"""
	points = np.asarray(pcd.points)
	num_point = len(points)
	if num_point < 100:
		return 0.0
	sample_size = max(100,int(num_point * sample_ratio))
	sample_indices = np.random.choice(num_point,sample_size,replace=False)
	pcd_tree = o3d.geometry.KDTreeFlann(pcd)

	distance = []
	for idx in sample_indices:
		[k,nn_idx,_] = pcd_tree.search_knn_vector_3d(points[idx],2)
		nearest_point = points[nn_idx[1]]
		distance.append(np.linalg.norm(points[idx] - nearest_point))
	return float(np.mean(distance))

def point_cloud_filter_lidar(points,attitude):
	"""
	坐标转换+直通滤波
		Args:
			points (np.array): cloud point[[x_s y z] [x_s y z] [x_s y z] .......[]]
			attitude (directory):{'yaw':xxx,'pitch':xxx,'roll':xxx,'height':xxx} radians
		Returns:
			pass_through_pcd_array (np.array)
			[[x_s y z] [x_s y z] [x_s y z] .......[] ] (cloud data in NED framework after filtering )
		"""
	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(points)
	min_bound = (-35, -35, -np.inf)
	max_bound = (35, 35, np.inf)

	box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
	pcd = pcd.crop(box)

	z_yaw = -attitude['yaw']
	y_pitch = attitude['pitch']
	x_roll = attitude['roll']
	euler_angle = [x_roll, y_pitch, z_yaw]
	rotation_matrix = pcd.get_rotation_matrix_from_xyz(euler_angle) #x-y-z 对应 roll-pitch-yaw

	pcd.rotate(rotation_matrix, center=(0, 0, 0))

	h1 = attitude['height']
	height = abs(round(h1, 2))
	min_bound = (-np.inf, -np.inf, -np.inf)
	max_bound = (np.inf, np.inf, height - 2.0)

	box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
	pass_through_pcd = pcd.crop(box)
	dense = estimate_avg_spacing(pass_through_pcd)
	if dense <= 0.01:
		downpcd = pass_through_pcd.voxel_down_sample(voxel_size=dense*12.0)
		point_array = np.asarray(downpcd.points)
	else:
		point_array = np.asarray(pass_through_pcd.points)

	return point_array,dense

class AirsimPointCloudHandler:
	def __init__(self):
		self.pose_lock = Lock()
		self.sync_queue = []
		rospy.init_node('node_airsim_filtered_sync_lidar', anonymous=True)

		self.pc_sub = Subscriber('/topic_airsim_orin_lidar', PointCloud2)
		self.pose_sub = Subscriber('/topic_drone_status',UavStatus)
		try:
			self.ts = ApproximateTimeSynchronizer([self.pc_sub,self.pose_sub],queue_size=5,slop=0.1)
			self.ts.registerCallback(self.sync_callback)
		except Exception as e:
			rospy.logerr("点云滤波同步初始化异常： %s",e)

		self.cloud_pub = rospy.Publisher('/topic_airsim_ned_points', PointCloud2, queue_size=1)
		self.event = Event()
		self.processing_thread = Thread(target=self.process_point_cloud,daemon=True)
		self.processing_thread.start()
		# rospy.loginfo("airsim listener initialized")
		rospy.spin()

	def sync_callback(self,cloud_msg,pose_msg):
		with self.pose_lock:
			self.sync_queue.append((cloud_msg,pose_msg))
			# rospy.loginfo("同步队列长度为： %d",len(self.sync_queue))
			# if len(self.sync_queue) > 5:
			# 	del self.sync_queue[0]

	def process_point_cloud(self):
		rate = rospy.Rate(50.0) #发布；频率50Hz
		while not rospy.is_shutdown():
			if self.event.isSet():
				break
			try:
				start_time = time.time()
				with self.pose_lock:
					if self.sync_queue:
						cloud_msg, pose_msg = self.sync_queue.pop(0)
					else:
						# rospy.loginfo("点云滤波同步队列中没有数据==============")
						rate.sleep()
						continue
				field_names = [f.name for f in cloud_msg.fields]
				if "x" not in field_names or "y" not in field_names or "z" not in field_names:
					rospy.logerr("node filtered Missing x/y/z in PointCloud2")
					rate.sleep()
					continue

				time_stamp = cloud_msg.header.stamp
				point_fields = ["x","y","z"]
				points = pc2.read_points(cloud_msg, field_names = point_fields, skip_nans=True)
				point_array = np.fromiter(points, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
				point_array = np.column_stack((point_array["x"], point_array["y"], point_array["z"]))
				if point_array.shape[0] > 1:
					uav_attitude = {'pitch': pose_msg.pitch, 'roll': pose_msg.roll,
									'yaw': -pose_msg.yaw, 'height': pose_msg.baro_altitude}
					ned_points,dense = point_cloud_filter_lidar(point_array,uav_attitude)
				else:
					ned_points = np.asarray([[0,0,0]])
					dense = 0.0
					rospy.logwarn("No ned point cloud received !!!!!!")
				rospy.loginfo("滤波后点云个数为：%s 密度 %s+++++++++++", ned_points.shape[0],round(dense,4))
				pc2_msg = pub_point_cloud2(ned_points,time_stamp)
				self.cloud_pub.publish(pc2_msg)
				end_time = time.time()
				# rospy.loginfo("点云处理耗时： %.2f", end_time - start_time) #0.04~0.09s
				rate.sleep()
			except Exception as e:
				rospy.logerr(f"点云滤波异常: {e}")

if __name__ == "__main__":
	cloud = None
	try:
		cloud = AirsimPointCloudHandler()
	except rospy.ROSInterruptException:
		cloud.event.set()
		pass


