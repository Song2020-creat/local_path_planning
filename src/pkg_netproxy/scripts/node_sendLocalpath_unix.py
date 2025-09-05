#!/usr/bin/env python3
import json
import struct
import time
import zlib

import rospy
import socket
from std_msgs.msg import String
from pkg_netproxy.msg import PathMsg

class UnixLocalPathSender:
    def __init__(self):
        rospy.loginfo("Waiting for /topic_local path topic...")
        try:
            # 等待第一条消息到达，超时5秒
            rospy.wait_for_message('/topic_localpath', String, timeout=5)
            rospy.loginfo("Connected to /topic_localpath topic")
        except rospy.ROSException:
            rospy.logwarn("Timeout waiting for /topic_localpath topic")

        self.local_path = rospy.get_param('/net_setting/local_path', "/tmp/local_path.sock")
        self.proto_type = int(rospy.get_param('/net_setting/proto_type', 4))
        self.method = int(rospy.get_param('/map_setting/method', 1))
        # 创建 Unix socket
        self.unix_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.seq = int(0)
        # 订阅本地路径话题
        if self.method == 1:  #AStar
            if self.proto_type == 1:
                self.subscriber = rospy.Subscriber('/topic_Astar_localpath', String, self.path_callback, queue_size=1)
            elif self.proto_type == 4:
                self.subscriber = rospy.Subscriber('/topic_Astar_localpath', PathMsg, self.path_callback, queue_size=1)
        elif self.method == 2: #RRT Smart
            if self.proto_type == 1:
                self.subscriber = rospy.Subscriber('/topic_rrt_localpath', String, self.path_callback, queue_size=1)
            elif self.proto_type == 4:
                self.subscriber = rospy.Subscriber('/topic_rrt_localpath', PathMsg, self.path_callback, queue_size=1)
    
    def encode_msg(self,data):
        #帧头 2 bytes
        frame_header = b'\xff\xee'
        #消息帧序列 4 bytes
        msg_id = int(self.seq)
        msg_id_bytes = struct.pack("<I",msg_id)
        #消息协议类型 1 bytes
        proto_type = self.proto_type
        proto_type_bytes = struct.pack("<B",proto_type)
        #无人机ID 1 bytes
        uav_id = int(210)
        uav_id_bytes = struct.pack("<B",uav_id)

        #时间戳 8 bytes
        timestamp = time.time()-(rospy.Time.now()-data.header.stamp).to_sec()
        timestamp_bytes = struct.pack("<d",timestamp)
        #与障碍物的距离 4 bytes
        min_dis = data.min_dis
        min_dis_bytes = struct.pack("<f",min_dis)
        #是否有障碍 1 bytes
        hb30 = data.hb30
        hb30_bytes = struct.pack("<b",hb30)
        #是否有路径 1 bytes
        path_ok = data.path_ok
        path_ok_bytes = struct.pack("<b",path_ok)

        #消息主体 8*N
        body = b''
        num_point = 0
        if path_ok:
            for point in data.path_points:
                lat = round(point.x,7)
                lng = round(point.y,7)
                lat = int(lat * 1e7)
                lng = int(lng * 1e7)
                body += struct.pack("<ii",lat,lng)

            num_point = len(data.path_points)
        #路径点个数 2 bytes
        num_point_bytes = struct.pack("<H",num_point)

        #帧长 2 bytes
        frame_len = 10 + 16 + len(body) + 4
        frame_len_bytes = struct.pack("<H",frame_len)

        frame_1 = frame_len_bytes + msg_id_bytes + proto_type_bytes + uav_id_bytes
        frame_2 = timestamp_bytes + hb30_bytes + path_ok_bytes + min_dis_bytes + num_point_bytes + body
        frame_without_header = frame_1 + frame_2
        crc = struct.pack("<I",zlib.crc32(frame_without_header) & 0xFFFFFFFF)
        frame = frame_header + frame_without_header + crc
        return frame


    def path_callback(self, msg):
        try:
            if self.proto_type == 4:
                frame = self.encode_msg(msg)
                self.unix_socket.sendto(frame, self.local_path)
                self.seq = (self.seq + 1) % 65536
            elif self.proto_type == 1:
                json_str = msg.data
                self.unix_socket.sendto(json_str.encode("utf-8"), self.local_path)
            time.sleep(0.02)
        except Exception as e:
            rospy.logerr(f"Error sending Unix local path data: {str(e)}")
    
    def shutdown(self):
        self.unix_socket.close()

if __name__ == '__main__':
    sender = None
    try:
        rospy.init_node('node_sendLocalpath')
        sender = UnixLocalPathSender()
        rospy.spin()
    except rospy.ROSInterruptException:
        # 注册关闭时的清理函数
        rospy.on_shutdown(sender.shutdown)
    
