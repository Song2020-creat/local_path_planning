import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
from pkg_netproxy.msg import UavStatus
from std_msgs.msg import Header
from threading import Thread, Lock, Event
from queue import Queue
import open3d as o3d

def pub_point_cloud2(points2):
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = "lidar_link"
	fields = [
		PointField('x', 0,PointField.FLOAT32,1),
		PointField('y', 4, PointField.FLOAT32, 1),
		PointField('z', 8, PointField.FLOAT32, 1),
	]
	return pc2.create_cloud(header,fields,points2.tolist())

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
	try:
		pcd.points = o3d.utility.Vector3dVector(points)
	except Exception as e:
		rospy.logerr("OPEN3D 异常: %s",e)
		return points
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

	point_array = np.asarray(pass_through_pcd.points)

	return point_array

class AirsimPointCloudHandler:
	def __init__(self):
		self.queue = Queue(maxsize = 2)
		self.pose_data = None
		self.pose_lock = Lock()

		rospy.init_node('node_airsim_filtered_lidar', anonymous=True)
		rospy.Subscriber('/topic_airsim_orin_lidar', PointCloud2, self.cloud_call_back,queue_size = 1)
		rospy.Subscriber('/topic_drone_status',UavStatus,self.telemetry_call_back,queue_size = 1)
		self.cloud_pub = rospy.Publisher('/topic_airsim_ned_points', PointCloud2, queue_size = 5)

		self.event = Event()
		self.processing_thread = Thread(target=self.process_point_cloud,daemon=True)
		self.processing_thread.start()
		# rospy.loginfo("airsim listener initialized")
		rospy.spin()

	def telemetry_call_back(self,msg):
		with self.pose_lock:
			self.pose_data = msg

	def cloud_call_back(self,msg):
		if not self.queue.full():
			self.queue.put(msg)
		else:
			rospy.loginfo("PointCloud queue is full. Skipping frame")

	def process_point_cloud(self):
		rate = rospy.Rate(50.0) #发布频率50Hz
		while not rospy.is_shutdown():
			if self.event.isSet():
				break
			try:
				if not self.queue.empty():
					cloud_msg = self.queue.get(timeout=1)
					field_names = [f.name for f in cloud_msg.fields]
					if "x" not in field_names or "y" not in field_names or "z" not in field_names:
						rospy.logerr("Missing x/y/z in PointCloud2")
						continue
					point_fields = ("x","y","z")
					points = pc2.read_points(cloud_msg, field_names = point_fields, skip_nans=True)
					#结构化数组
					point_array = np.fromiter(points, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])

					point_array = np.column_stack((point_array["x"],point_array["y"],point_array["z"]))
					if point_array.size > 0:
						with self.pose_lock:
							if self.pose_data is not None:
								uav_attitude = {'pitch': self.pose_data.pitch, 'roll': self.pose_data.roll,
												'yaw': -self.pose_data.yaw, 'height': self.pose_data.baro_altitude}
								# uav_attitude = {'pitch': 0.0, 'roll': 0.0,
								# 				'yaw': 0.0, 'height': self.pose_data.baro_altitude}
							else:
								continue
						try:
							ned_points = point_cloud_filter_lidar(point_array,uav_attitude)
						except Exception as e:
							rospy.logerr("点云滤波异常: %s",e)
							ned_points = point_array

						pc2_msg = pub_point_cloud2(ned_points)
						self.cloud_pub.publish(pc2_msg)
						# rospy.loginfo("滤波后点云个数 %d", ned_points.shape[0])
						rate.sleep()

					else:
						rospy.loginfo("No point cloud received !!!!!!")
			except Exception as e:
				rospy.logerr(f"Processing error: {e}")

if __name__ == "__main__":
	cloud = None
	try:
		cloud = AirsimPointCloudHandler()
	except rospy.ROSInterruptException:
		cloud.event.set()
		pass



