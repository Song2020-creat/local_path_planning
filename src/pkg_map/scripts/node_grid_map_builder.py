import time

import cv2
from queue import Queue
import rospy
import numpy as np
from threading import Thread, Lock, Event
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
# from pkg_netproxy.msg import UavStatus
from nav_msgs.msg import OccupancyGrid, MapMetaData

class GridMapGenerator:
    def __init__(self):
        rospy.init_node('node_grid_map_builder', anonymous=True)
        self.queue = Queue(maxsize=2)
        self.lock = Lock()
        self.time_threshold = rospy.Duration(0.1)

        self.lidar_range = rospy.get_param('/map_setting/lidar_range', 35)
        self.dilate = rospy.get_param('/map_setting/dilate', 7)
        self.resolution = rospy.get_param('/map_setting/cell_size', 1)
        self.map_range = int((self.lidar_range + self.dilate))
        self.map_size = int(self.map_range*2/self.resolution)

        self.kernalInt = cv2.getStructuringElement(cv2.MORPH_RECT, (self.dilate, self.dilate))
        self.dilate_map = round(self.dilate / self.resolution)
        self.map_pub = rospy.Publisher('/topic_grid_map',OccupancyGrid,queue_size=2)
        self.orin_map_pub = rospy.Publisher('/topic_orin_grid_map', OccupancyGrid, queue_size=2)

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
            pathmap = np.zeros((self.map_size, self.map_size), dtype=np.uint8)
            pathmap_dilate = np.zeros((self.map_size, self.map_size), dtype=np.uint8)
            start_time = time.time()
            if not self.queue.empty():
                cloud_msg = self.queue.get(timeout=1)
                field_names = [f.name for f in cloud_msg.fields]
                if "x" not in field_names or "y" not in field_names or "z" not in field_names:
                    rospy.logerr("Missing x/y/z in PointCloud2")
                    continue
                time_stamp = cloud_msg.header.stamp
                point_fields = ["x", "y", "z"]
                points = pc2.read_points(cloud_msg, field_names=point_fields, skip_nans=True)
                his_cloud = np.fromiter(points, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
                his_cloud = np.column_stack((his_cloud["x"], his_cloud["y"], his_cloud["z"]))


                if his_cloud.shape[0] > 1:
                    try:
                        grid_coords = np.floor_divide(his_cloud[:,:2],self.resolution).astype(int)
                        grid_coords = np.unique(grid_coords, axis=0)
                        rospy.loginfo("建图的点云个数： %s",grid_coords.shape[0])
                        grid_coords[:,0] = int(self.map_size / 2) + grid_coords[:,0]
                        grid_coords[:, 1] = int(self.map_size / 2) + grid_coords[:,1]
                    except Exception as e:
                        rospy.logerr("建图转换异常： %s",e)
                        continue

                    # 遍历点云，构图
                    for point in grid_coords:
                        lx = point[0]
                        ly = point[1]
                        try:
                            if (self.map_size - self.dilate_map) > lx >= self.dilate_map <= ly < (
                                    self.map_size - self.dilate_map):
                                pathmap[ly, lx] = 100
                        except Exception as e:
                            rospy.logwarn("障碍物越界异常：%s", e)

                    try:
                        # 边缘膨胀
                        pathmap_dilate = cv2.dilate(src=pathmap, kernel=self.kernalInt, iterations=1,
                                             borderType = cv2.BORDER_CONSTANT,
                                             borderValue = 0)
                        pathmap_dilate[int(self.map_size / 2) - 1:int(self.map_size / 2) + 1,
                        int(self.map_size / 2) - 1:int(self.map_size / 2) + 1] = 0
                        pathmap[int(self.map_size / 2) - 1:int(self.map_size / 2) + 1,
                        int(self.map_size / 2) - 1:int(self.map_size / 2) + 1] = 0
                    except Exception as e:
                        rospy.logerr("膨胀越界异常： %s", e)

                self.publish_map(pathmap_dilate,time_stamp,1)
                self.publish_map(pathmap, time_stamp, 0)
                end_time = time.time()
                rospy.loginfo("二维建图耗时： %.2f",end_time-start_time) #0.02~0.03s
            rate.sleep()

    def publish_map(self,grid,time_stamp,flag=1):
        map_msg = OccupancyGrid()
        # map_msg.header.stamp = rospy.Time.now()
        map_msg.header.stamp = time_stamp
        map_msg.header.frame_id = "lidar_link"

        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.map_size
        map_msg.info.height = self.map_size
        origin_x = 0 - (self.map_size / 2) * self.resolution
        origin_y = 0 - (self.map_size / 2) * self.resolution
        map_msg.info.origin.position.x = origin_x
        map_msg.info.origin.position.y = origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = grid.flatten().tolist()
        if flag:
            self.map_pub.publish(map_msg)
        else:
            self.orin_map_pub.publish(map_msg)
        return 1

if __name__ == '__main__':
    node = None
    try:
        node = GridMapGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        node.event.set()


