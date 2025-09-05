import math
import time
import rospy
import numpy as np
from threading import Thread, Lock, Event

from pkg_netproxy.msg import UavStatus
from geometry_msgs.msg import Point32
from pkg_netproxy.msg import PathMsg
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, String
import json
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sys
import os
# import socket
script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)
from AStarPath_WS import A_star_search
from curve_spline import CurveSpline

class AstarPathPlan:
    def __init__(self):
        rospy.init_node('node_Astar_path_plan', anonymous=True)
        # self.unix_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        # self.local_path = rospy.get_param('/net_setting/local_path', "/tmp/local_path.sock")
        self.lock = Lock()
        self.sync_manu = True #手动同步
        if self.sync_manu:
            self.map_buffer = []

            self.pose_buffer = []
            self.time_threshold = rospy.Duration(0.1)

            rospy.Subscriber('/topic_grid_map', OccupancyGrid, self.map_callback,queue_size=3)

            rospy.Subscriber('/topic_drone_status', UavStatus, self.pose_callback,queue_size=3)

        else:
            self.sync_queue = []
            self.map_sub = Subscriber('/topic_grid_map', OccupancyGrid)
            self.pose_sub = Subscriber('/topic_drone_status', UavStatus)
            try:
                self.ts = ApproximateTimeSynchronizer([self.map_sub, self.pose_sub], queue_size=5, slop=0.1)
                self.ts.registerCallback(self.sync_callback)
            except Exception as e:
                rospy.logerr("path_plan同步初始化异常： %s", e)
        self.orin_map_buffer = []
        rospy.Subscriber('/topic_orin_grid_map', OccupancyGrid, self.orin_map_callback, queue_size=3)
        self.curve = CurveSpline()
        self.hb30 = 0
        self.min_dis = 1000
        self.last_local_path = []
        self.last_local_path_xy = []
        self.error_plan = 0
        self.lidar_range = rospy.get_param('/map_setting/lidar_range', 35)
        self.dilate = rospy.get_param('/map_setting/dilate', 7)
        self.resolution = rospy.get_param('/map_setting/cell_size', 1)
        self.barrier_dis = rospy.get_param('/map_setting/barrier_dis', 20.0)
        self.rate = int(rospy.get_param('/map_setting/rate', 30))
        self.show = int(rospy.get_param('/map_setting/image_show', 1))
        self.proto_type = int(rospy.get_param('/net_setting/proto_type', 4))
        self.min_cloud = int(rospy.get_param('/map_setting/minimum_cloud', 4))

        self.map_range = int((self.lidar_range + self.dilate))
        self.map_size = int(self.map_range * 2 / self.resolution)

        if self.proto_type == 1:
            self.path_pub = rospy.Publisher('/topic_Astar_localpath', String, queue_size=1)
        elif self.proto_type == 4:
            self.path_pub_lat = rospy.Publisher('/topic_Astar_localpath', PathMsg, queue_size=1)
        self.path_pub_xy = rospy.Publisher('/topic_local_path_xy', Path, queue_size = 2)
        self.barrier_list = []
        if self.show:
            from path_test import PathVisualizer
            self.shared_path = []
            goal = (40,40)
            self.shared_lock = Lock()
            self.vis_thread = PathVisualizer(self.barrier_list.copy(), goal, self.shared_path,[],(-self.map_range, self.map_range), self.shared_lock)
            self.vis_thread.start()

        self.event = Event()
        self.path_thread = Thread(target=self.local_path_plan, daemon=True)
        self.path_thread.start()

    def sync_callback(self, grid_msg, pose_msg):
        with self.lock:
            self.sync_queue.append((grid_msg, pose_msg))
    def map_callback(self,msg):
        with self.lock:
            self.map_buffer.append(msg)
            if len(self.map_buffer) > 10:
                del self.map_buffer[0]
    def orin_map_callback(self,msg):
        with self.lock:
            self.orin_map_buffer.append(msg)
            if len(self.orin_map_buffer) >= 3:
                del self.orin_map_buffer[0]
    def pose_callback(self,msg):
        with self.lock:
            self.pose_buffer.append(msg)
            if len(self.pose_buffer) > 2:
                del self.pose_buffer[0]

    def try_match_and_fuse(self):
        closest_map = None
        closest_pose = None
        for pose_msg in self.pose_buffer:
            pose_time = pose_msg.header.stamp
            min_time_diff = rospy.Duration(99)
            for map_msg in self.map_buffer:
                map_time = map_msg.header.stamp
                time_diff = abs(pose_time - map_time)
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    closest_map = map_msg
                    closest_pose = pose_msg
            if min_time_diff <= self.time_threshold:
                self.map_buffer.remove(closest_map)
                self.pose_buffer.remove(pose_msg)
                break
        return closest_map,closest_pose

    @staticmethod
    def get_gpsPointFromXY(lat1,lng1,x,y):
        ce = 0.000001
        dy = 0.11132
        dx = dy * math.cos(lat1 * math.pi / 180.0)
        lat = round(lat1 + y * ce / dy,7)
        lng = round(lng1 + x * ce / dx,7)
        return (lat,lng)

    # 计算两个经纬度点之间的距离,给出分量距离
    # disx向东为正，向西为负
    # disy向北为正，向南为负
    @staticmethod
    def get_xydistance(lat1, lng1, lat2, lng2):
        dy = 0.11132
        dx = dy * math.cos(lat1*math.pi/180.0)
        disx = (lng2-lng1)/0.000001*dx
        disy = (lat2-lat1)/0.000001*dy
        return round(disy,2),round(disx,2)

    # 计算全局坐标在算法图的xy值
    def getGlobalXY(self, disx, disy, slop):
        angle = abs(math.atan2(disy, disx) * 180.0 / math.pi)
        if angle <= 45.0 and disx >= self.map_range:
            disx = self.map_range - 1
            if angle <= 2.0:
                disy = 0
            else:
                disy = round(disx / slop, 2)
        elif angle >= 135.0 and disx <= -self.map_range:
            disx = -self.map_range + 1
            if angle >= 178.0:
                disy = 0
            else:
                disy = round(disx / slop, 2)
        elif 45.0 < angle < 135.0 and abs(disy) >= self.map_range:
            if disy > 0:
                disy = self.map_range - 1
            else:
                disy = -self.map_range + 1
            disx = round(slop * disy, 2)
        # 南北
        lx = int(disx / self.resolution)
        # 东西
        ly = int(disy / self.resolution)
        mapy = int(self.map_size / 2 + ly)
        mapx = int(self.map_size / 2 + lx)

        return (mapx, mapy)

    def obstacle_judge(self,map_cloud,line_vec,goal,horizon_dis = 3.0):
        barrier_count = 0
        barrier_count_list = []
        min_barrier_dis = 1000
        obstacle_array = np.argwhere(map_cloud > 0)
        goal_x = (-self.map_size / 2 + goal[0]) * self.resolution
        goal_y = (goal[1] - self.map_size / 2) * self.resolution

        self.barrier_list = []
        if obstacle_array.shape[0] > 0:
            for point in obstacle_array:
                disy = (point[0] - self.map_size / 2) * self.resolution
                disx = (point[1] - self.map_size / 2) * self.resolution

                self.barrier_list.append((disx, disy))
                # 纵向距离
                l_dis = math.sqrt(disy** 2 + disx ** 2)
                # 横向距离
                vec_1 = np.array([[disx, disy]])
                vec_2 = np.array([[goal_x, goal_y]])
                vec_2_unit = vec_2 / np.linalg.norm(vec_2)
                proj_1_to_2 = np.dot(vec_1, vec_2_unit.T) * vec_2_unit
                h_dis = np.linalg.norm(vec_1 - proj_1_to_2)

                if l_dis <= self.barrier_dis and h_dis <= horizon_dis:
                    barrier_count = barrier_count + 1
                    if np.linalg.norm(line_vec) > 0.5:
                        vel_unit = line_vec / np.linalg.norm(line_vec)
                    else:
                        vel_unit = np.asarray([[0, 0]])

                    barrier_uint = vec_1/(np.linalg.norm(vec_1) + 1e-6)
                    angle = np.dot(vel_unit, barrier_uint.T)
                    if angle[0] >= 0:
                        barrier_count_list.append(l_dis)
        else:
            rospy.logwarn("地图上没有障碍物！！！！！！！！！！")
        hb30 = 0
        if barrier_count >= self.min_cloud:
            hb30 = 1
            if len(barrier_count_list) > 0:
                barrier_count_list.sort()
                min_barrier_dis = round(barrier_count_list[0], 2)
        return hb30, min_barrier_dis

    def evaluate_path(self,start_yaw, path):
        if len(path) <= 5:
            return float('-inf')
        path_arr = np.asarray(path)
        heading = self.heading_cost(start_yaw, path)
        clearance = self.obstacle_clearance(path_arr)
        # smooth = self.path_smoothness_cost(path_arr)

        # score = 1.0 / (smooth + 1e-6) + 3.0 * clearance + 5.0 * heading
        #TODO 参数调整
        score = 0.05 * clearance + 0.5 * heading
        # rospy.loginfo("%s, %s",clearance,heading)
        return round(score,2)

    def path_smoothness_cost(self,path):
        angles = []
        for i in range(1,path.shape[0]-1):
            v1 = path[i] - path[i-1]
            v2 = path[i+1] - path[i]
            cos_angle = np.dot(v1,v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
            angle = np.arccos(np.clip(cos_angle, -1.0,1.0))
            angles.append(angle)
        return np.sum(angles)


    def heading_cost(self,start_yaw,path):
        first_vec = np.array(path[3])
        first_angle = np.arctan2(first_vec[1], first_vec[0])
        angle_diff = first_angle - start_yaw
        if angle_diff >= np.pi:
            angle_diff = angle_diff - 2 * np.pi
        elif angle_diff <= -np.pi:
            angle_diff = angle_diff + 2 * np.pi

        heading_error = math.cos(angle_diff)
        return heading_error
    def local_path_plan(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            start_time = time.time()
            if self.event.isSet():
                break
            if self.sync_manu:
                if not self.map_buffer or not self.pose_buffer:
                    rospy.logwarn("A*没有收到地图或者姿态消息")
                    rate.sleep()
                    continue
                with self.lock:
                    grid_map, pose = self.try_match_and_fuse()
                if grid_map is None or pose is None:
                    # rospy.logwarn("无法匹配地图和终点位置,取最新的消息！！！！！")
                    grid_map = self.map_buffer.pop(-1)
                    pose = self.pose_buffer.pop(-1)
            else:
                with self.lock:
                    if self.sync_queue:
                        grid_map, pose = self.sync_queue.pop(0)
                    else:
                        # rospy.logwarn("路径规划同步队列中没有数据==============")
                        # rate.sleep()
                        continue

            # rospy.loginfo("匹配成功++++++++++++++++")
            #grid_map 消息解析
            grid_width = grid_map.info.width
            grid_height = grid_map.info.height
            pathmap = np.array(grid_map.data,dtype=np.int8).reshape((grid_height,grid_width))
            orin_pathmap = None
            if self.orin_map_buffer:
                with self.lock:
                    orin_grid_map = self.orin_map_buffer.pop(-1)
                orin_grid_width = orin_grid_map.info.width
                orin_grid_height = orin_grid_map.info.height
                orin_pathmap = np.array(orin_grid_map.data, dtype=np.int8).reshape((orin_grid_height, orin_grid_width))

            vel_n = pose.velocity_n
            vel_e = pose.velocity_e
            vel_vector = np.asarray([vel_n,vel_e])

            target_position_lat = pose.g_latitude
            target_position_lng = pose.g_longitude
            if target_position_lat == 0.0:
                rospy.logwarn("没有终点信息==============")
                # rate.sleep()
                continue
            current_position_lat = pose.latitude
            current_position_lng = pose.longitude
            goal_x,goal_y = self.get_xydistance(current_position_lat,current_position_lng,
                                                target_position_lat,target_position_lng)
            local_path = []
            local_path_xy = []


            if abs(goal_x) <= 2.0 and abs(goal_y) <= 2.0:
                self.hb30 = 0
                local_path_xy.append([goal_x,goal_y])
                self.publish_path_xy(local_path_xy)

                local_path.append({"lat":round(target_position_lat, 7),"lng":round(target_position_lng, 7)})

                if self.proto_type == 1:
                    self.publish_path_json(local_path)
                elif self.proto_type == 4:
                    self.publish_path_lat(local_path)
                rospy.loginfo("终点为全局点********")
                self.last_local_path = []
                self.last_local_path_xy = []
                time.sleep(0.1)
                continue
            if abs(goal_y) >= 0.5:
                dis_slope = round(goal_x / goal_y, 2)
            else:
                dis_slope = float('inf')

            (mapx, mapy) = self.getGlobalXY(goal_x, goal_y, dis_slope)
            # rospy.loginfo("终点坐标：%s",(mapx, mapy))

            # 是否有障碍判断
            if orin_pathmap is not None:
                self.hb30,self.min_dis = self.obstacle_judge(orin_pathmap,vel_vector,
                                                             goal=(mapx, mapy),horizon_dis=self.dilate/2.0+0.5)
            else:
                # self.hb30, self.min_dis = self.obstacle_judge(pathmap, vel_vector, goal=(mapx, mapy),horizon_dis=1.0)
                rospy.logwarn("原始地图未更新！！！！！！")
                continue
            # if self.hb30:
            #     if self.min_dis > 100:
            #         #前方存在障碍,但当前飞行方向90度范围内无障碍
            #         rospy.loginfo("前方存在障碍，可不避让**************： %f", self.min_dis)
            #     else:
            #         rospy.loginfo("前方存在障碍，需避让**************： %f", self.min_dis)

            apoints_tran = A_star_search(pathmap, [int(self.map_size / 2), int(self.map_size / 2)], [mapy, mapx])

            # if apoints_tran == "destion wrong":
            #     rospy.logerr("终点去除障碍物失败---------")
            #     # rate.sleep()
            #     continue
            if apoints_tran == "start wrong":
                rospy.logerr("起点去除障碍物失败---------")
                # rate.sleep()
                continue
            elif apoints_tran == "A* failed":
                rospy.logerr("********************Astar规划失败")
                if self.obstacle_clearance(self.last_local_path_xy) >= 1.0:
                    local_path = self.last_local_path
                else:
                    local_path = []
                    self.last_local_path = []
                    self.last_local_path_xy = []

                if self.proto_type == 1:
                    self.publish_path_json(local_path)
                elif self.proto_type == 4:
                    self.publish_path_lat(local_path)
                rate.sleep()
                continue
            else:
                # 进行Bspline曲线平滑路径
                apoints = []
                if len(apoints_tran) > 6:
                    a = []
                    for node in apoints_tran:
                        list_node = [node['x_s'], node['y']]
                        a.append(list_node)
                    path_array = np.asarray(a)
                    (x, y) = (path_array[:, 0], path_array[:, 1])
                    try:
                        fitting_points = self.curve.BSpline(control_points=[x, y], number=len(apoints_tran),sm=2.0)
                        for i in range(fitting_points[0].shape[0]):
                            apoints.append({'x_s': round(fitting_points[0][i], 4), 'y': round(fitting_points[1][i], 4)})
                    except Exception as e:
                        rospy.logerr("路径平滑失败，%s", e)
                        apoints = apoints_tran
                else:
                    apoints = apoints_tran

                for astarpoint in apoints:
                    # 局部路径转为经纬度点
                    y_real = (astarpoint['y'] - self.map_size / 2) * self.resolution
                    x_real = (astarpoint['x_s'] - self.map_size / 2 ) * self.resolution
                    local_path_xy.append((x_real,y_real))
                    (local_lat, local_lng) = self.get_gpsPointFromXY(current_position_lat, current_position_lng, y_real,x_real)
                    local_path.append({"lat":local_lat,"lng":local_lng})

                if len(local_path_xy) > 5:
                    vel_vector = np.array([pose.velocity_n,pose.velocity_e])
                    if np.linalg.norm(vel_vector) >= 0.5:
                        vel_angle = np.arctan2(vel_vector[1],vel_vector[0])
                        heading = vel_angle
                    else:
                        heading = pose.yaw
                    score1 = self.evaluate_path(heading,local_path_xy)
                    score2 = self.evaluate_path(heading,self.last_local_path_xy)
                    # rospy.loginfo("%s, %s",score1,score2)
                    if score1 >= score2 - 0.1:
                        self.last_local_path = local_path
                        self.last_local_path_xy = local_path_xy
                    else:
                        local_path = self.last_local_path
                        rospy.loginfo("A*避免频繁大角度转向，仍走之前路径=============")
                else:
                    self.last_local_path = local_path
                    self.last_local_path_xy = local_path_xy

                self.publish_path_xy(local_path_xy)
                if self.proto_type == 1:
                    self.publish_path_json(local_path)
                elif self.proto_type == 4:
                    self.publish_path_lat(local_path)
            if self.show:
                with self.shared_lock:
                    self.vis_thread.obstacle_list = self.barrier_list.copy()
                    self.vis_thread.goal = (goal_x,goal_y)
                    self.shared_path.clear()
                    if local_path_xy:
                        self.shared_path.extend(local_path_xy.copy())
            end_time = time.time()
            # rospy.loginfo("A*二维路径规划耗时： %.2f", end_time - start_time)  #0.06~0.08s
            rate.sleep()

    def obstacle_clearance(self,path):
        min_dist = 1000
        dis_list = []
        if self.barrier_list:
            obstacles = np.asarray(self.barrier_list)
            count = 0
            for p in path:
                dists = np.linalg.norm(obstacles - p, axis=1)
                min_dist = np.min(dists)
                if min_dist < 1.0:
                    count = count + 1
                if count >= 3:
                    return -1000
                dis_list.append(min_dist)

            min_dist = round(sum(dis_list) / len(dis_list),2)
            return min(min_dist,8.0)
        return min_dist

    def publish_path_xy(self,local_path):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "lidar_link"
        if local_path:
            for i,(xn,ye) in enumerate(local_path):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = xn
                pose.pose.position.y = ye
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
        else:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub_xy.publish(path_msg)
        # rospy.loginfo("A*发布规划路径点，共 %d 个点",len(local_path))
        return

    def publish_path_lat(self,local_path):
        path_msg = PathMsg()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "lidar_link"

        path_msg.hb30 = self.hb30
        path_msg.min_dis = self.min_dis
        path_msg.path_points = []
        if local_path:
            path_msg.path_ok = 1
            for point in local_path:
                p = Point32()
                p.x = point["lat"]
                p.y = point["lng"]
                p.z = 0.0
                path_msg.path_points.append(p)
        else:
            path_msg.path_ok = 0
        self.path_pub_lat.publish(path_msg)
        return
    def publish_path_json(self,local_path):
        if local_path:
            path_success = 1
        else:
            path_success = 0
        local_path_lat = {"path":local_path,"time":time.time(),"hb30":self.hb30,"status":path_success,"min_dis":self.min_dis,"isobstacle":self.hb30}
        path_msg = String()
        try:
            json_str = json.dumps(local_path_lat)
            path_msg.data = json_str
            # rospy.loginfo("%s",json_str)
            # self.unix_socket.sendto(json_str.encode("utf-8"), self.local_path)
        except Exception as e:
            rospy.logerr("局部路径发布（经纬度）JSON 编码异常：%s",e)
            return
        self.path_pub.publish(path_msg)
        # rospy.loginfo("发布规划路径点(经纬度），共 %d 个点", len(local_path))

if __name__ == '__main__':
    path_node = None
    try:
        path_node = AstarPathPlan()
        rospy.spin()
    except rospy.ROSInterruptException:
        path_node.event.set()
        # path_node.unix_socket.close()
        if path_node.show:
            path_node.vis_thread.event.set()













