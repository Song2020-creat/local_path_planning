import time

import cv2
from queue import Queue
import rospy
import numpy as np
from threading import Thread, Lock, Event
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData

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

class RRTMapGenerator:
	def __init__(self):
		rospy.init_node('node_rrt_map_builder', anonymous=True)
		self.queue = Queue(maxsize=2)
		self.lock = Lock()
		self.time_threshold = rospy.Duration(0.1)
		# self.lidar_range = rospy.get_param('/map_setting/lidar_range', 35)
		self.resolution = rospy.get_param('/map_setting/cell_size', 1)

		self.map_pub = rospy.Publisher('/topic_rrt_map',PointCloud2,queue_size=2)
		rospy.Subscriber('/topic_airsim_ned_points',PointCloud2,self.cloud_callback,queue_size = 1)
		self.event = Event()
		self.map_thread = Thread(target = self.map_generator,daemon=True)
		self.map_thread.start()

	def cloud_callback(self,msg):
		if not self.queue.full():
			self.queue.put(msg)
		# else:
			# rospy.loginfo("PointCloud queue is full. Skipping frame")
	def map_generator(self):
		rate = rospy.Rate(50)
		while not rospy.is_shutdown():
			if self.event.isSet():
				break
			start_time = time.time()
			if not self.queue.empty():
				cloud_msg = self.queue.get(timeout=1)
				field_names = [f.name for f in cloud_msg.fields]
				if "x" not in field_names or "y" not in field_names or "z" not in field_names:
					rospy.logerr("rrt Missing x/y/z in PointCloud2")
					continue
				time_stamp = cloud_msg.header.stamp
				point_fields = ["x", "y", "z"]
				points = pc2.read_points(cloud_msg, field_names=point_fields, skip_nans=True)
				his_cloud = np.fromiter(points, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
				his_cloud = np.column_stack((his_cloud["x"], his_cloud["y"], his_cloud["z"]))

				# rospy.loginfo("建图的原始点云个数： %s",his_cloud.shape)
				if his_cloud.shape[0] > 1:
					try:
						grid_coords = np.floor_divide(his_cloud,self.resolution).astype(int)
						grid_coords = np.unique(grid_coords, axis=0)
					except Exception as e:
						rospy.logerr("RRT建图转换异常： %s",e)
						continue
				else:
					grid_coords = np.asarray([[0,0,0]])

				# rospy.loginfo("建图处理后的点云个数： %s", grid_coords.shape)
				pc2_msg = pub_point_cloud2(grid_coords, time_stamp)
				self.map_pub.publish(pc2_msg)
			end_time = time.time()
			# rospy.loginfo("二维建图耗时： %.2f",end_time-start_time) #0.02~0.03s
			rate.sleep()

if __name__ == '__main__':
	node = None
	try:
		node = RRTMapGenerator()
		rospy.spin()
	except rospy.ROSInterruptException:
		node.event.set()


