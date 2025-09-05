import time
import select
import rospy
from pkg_netproxy.msg import UavStatus
import socket
import json
from threading import Thread, Lock, Event
import os

class DroneStatus:
    def __init__(self):
        rospy.init_node('node_drone_status_publish', anonymous=True)
        self.lock = Lock()
        self.waypoints = {}
        self.target_time = None
        self.pub = rospy.Publisher('/topic_drone_status', UavStatus, queue_size=3)
        self.rate = rospy.Rate(50)  #50 Hz

        # self.attitude_path = rospy.get_param('/net_setting/attitude_path', "/tmp/drone_attitude.sock")
        self.target_path = rospy.get_param('/net_setting/target_path', "/tmp/target_position.sock")

        # if os.path.exists(self.attitude_path):
        #     os.remove(self.attitude_path)
        if os.path.exists(self.target_path):
            os.remove(self.target_path)

        # self.sock_att = socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        self.sock_goal = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

        self.sock_goal.bind(self.target_path)
        # self.sock_att.bind(self.attitude_path)
        # self.sock_att.setblocking(False)
        # self.sock_goal.setblocking(False)
        self.sock_goal.settimeout(1.0)

        self.event = Event()
        self.thread = Thread(target=self.receive_loop,daemon=True)
        self.thread.start()

    def receive_loop(self):
        while not rospy.is_shutdown():
            try:
                if self.event.isSet():
                    break
                data,addr = self.sock_goal.recvfrom(2048)
                try:
                    way_point = json.loads(data.decode('utf-8'))
                except json.JSONDecodeError:
                    rospy.logwarn("接收姿态数据解码异常")
                    time.sleep(0.02)
                    continue
                with self.lock:
                    if isinstance(way_point,dict):
                        self.waypoints['latitude'] = way_point.get("lat",0.0)
                        self.waypoints['longitude'] = way_point.get("lng",0.0)
                        self.waypoints['altitude'] = way_point.get("alt", 0.0)
                        self.waypoints['height'] = way_point.get("height", 0.0)
                        self.waypoints['tfheight'] = way_point.get("tfheight", 0.0)
                        self.waypoints['vn'] = way_point.get("vx", 0.0)
                        self.waypoints['ve'] = way_point.get("vy", 0.0)
                        self.waypoints['vz'] = way_point.get("vz", 0.0)
                        self.waypoints['pitch'] = way_point.get("EA_x", 0.0)
                        self.waypoints['roll'] = way_point.get("EA_y", 0.0)
                        self.waypoints['yaw'] = way_point.get("EA_z", 0.0)
                        pos = way_point.get("pos", {"lat": 0.0, "lng": 0.0})
                        self.target_time = way_point.get("time", None)
                        self.waypoints['target_lat'] = pos["lat"]
                        self.waypoints['target_lng'] = pos["lng"]
                    else:
                        rospy.logwarn("接收的姿态信息格式错误")
                        time.sleep(0.02)
                        continue
                # time.sleep(0.1)
            except socket.timeout:
                rospy.loginfo("等待无人机的姿态、位置信息........")
            except Exception as e:
                rospy.logerr(f"接收位置、姿态信息错误！！！！！: {e}")

    def receive_loop_two(self):
        while not rospy.is_shutdown():
            readable, _, _ = select.select([self.sock_att,self.sock_goal], [], [], 0)
            for sock in readable:
                try:
                    if self.event.isSet():
                        break
                    data,addr = sock.recvfrom(1024)
                    try:
                        way_point = json.loads(data.decode('utf-8'))
                    except json.JSONDecodeError:
                        rospy.logwarn("接收数据解码异常")
                        time.sleep(0.02)
                        continue
                    if sock == self.sock_att:
                        with self.lock:
                            if isinstance(way_point,dict):
                                self.waypoints['latitude'] = way_point.get("lat",0.0)
                                self.waypoints['longitude'] = way_point.get("lng",0.0)
                                self.waypoints['altitude'] = way_point.get("alt", 0.0)
                                self.waypoints['height'] = way_point.get("height", 0.0)
                                self.waypoints['tfheight'] = way_point.get("tfheight", 0.0)
                                self.waypoints['vn'] = way_point.get("vx", 0.0)
                                self.waypoints['ve'] = way_point.get("vy", 0.0)
                                self.waypoints['vz'] = way_point.get("vz", 0.0)
                                self.waypoints['pitch'] = way_point.get("EA_x", 0.0)
                                self.waypoints['roll'] = way_point.get("EA_y", 0.0)
                                self.waypoints['yaw'] = way_point.get("EA_z", 0.0)
                            else:
                                rospy.logwarn("接收的姿态信息格式错误")
                                time.sleep(0.02)
                                continue
                    elif sock == self.sock_goal:
                        with self.lock:
                            if isinstance(way_point, dict):
                                pos = way_point.get("pos", {"lat":0.0,"lng":0.0})
                                self.target_time = way_point.get("time", None)
                                self.waypoints['target_lat'] = pos["lat"]
                                self.waypoints['target_lng'] = pos["lng"]
                                # self.waypoints['target_alt'] = pos["alt"]
                            else:
                                rospy.logwarn("接收的终点信息格式错误")
                                time.sleep(0.05)
                                continue
                    # time.sleep(0.1)
                except Exception as e:
                    rospy.logwarn(f"ERROR receiving global point data: {e}")

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.waypoints:
                    msg = UavStatus()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "ned_frame"
                    msg.latitude = self.waypoints.get('latitude',0.0)
                    msg.longitude = self.waypoints.get('longitude',0.0)
                    msg.altitude = self.waypoints.get('altitude',0.0)
                    msg.roll = self.waypoints.get('roll',0.0)
                    msg.pitch = self.waypoints.get('pitch',0.0)
                    msg.yaw = self.waypoints.get('yaw',0.0)
                    msg.baro_altitude = self.waypoints.get('height',0.0)
                    msg.tf_altitude = self.waypoints.get('tfheight',0.0)
                    msg.velocity_n = self.waypoints.get('vn',0.0)
                    msg.velocity_e = self.waypoints.get('ve',0.0)
                    msg.velocity_d = self.waypoints.get('vz',0.0)
                    # if self.target_time is not None:
                    #     if abs(time.time() - self.target_time) > 1.0:
                    #         rospy.logwarn("终点信息超过 1s 没更新了=======")
                    msg.g_latitude = self.waypoints.get('target_lat',0.0)
                    msg.g_longitude = self.waypoints.get('target_lng',0.0)
                    # msg.g_altitude = self.waypoints['target_alt']

                    self.pub.publish(msg)
                    # rospy.loginfo("接收的姿态信息:======")
                    # rospy.loginfo("%.2f,%.2f,%.2f,%.2f",msg.baro_altitude,msg.velocity_n,msg.pitch,msg.yaw)
            self.rate.sleep()

if __name__ == '__main__':
    node = DroneStatus()
    try:
        node.run()
    except rospy.ROSInterruptException:
        node.event.set()
        pass