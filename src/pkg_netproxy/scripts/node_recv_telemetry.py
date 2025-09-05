import time
import rospy
from pkg_netproxy.msg import UavStatus
import socket
import json
from threading import Thread, Lock, Event

class DroneStatus:
    def __init__(self):
        rospy.init_node('node_drone_status_publish', anonymous=True)
        self.lock = Lock()
        self.waypoints = {}

        self.pub = rospy.Publisher('/topic_drone_status', UavStatus, queue_size=3)
        self.rate = rospy.Rate(50)  # 50 Hz

        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.local_ip = rospy.get_param('/net_setting/local_ip', '0.0.0.0')
        self.local_port = rospy.get_param('/net_setting/local_port', 9998)
        self.sock.bind((self.local_ip,self.local_port))
        self.sock.settimeout(0.2) #超时没有收到数据会有提示

        self.event = Event()
        self.thread = Thread(target=self.receive_loop,daemon=True)
        self.thread.start()

    def receive_loop(self):
        while not rospy.is_shutdown():
            if self.event.isSet():
                break
            try:
                data,addr = self.sock.recvfrom(2048)
                try:
                    way_point = json.loads(data.decode())
                except json.JSONDecodeError:
                    rospy.logwarn("接收的姿态数据解码异常")
                    time.sleep(0.05)
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
                        self.waypoints['pitch'] = way_point.get("EA_x", 90.0)
                        self.waypoints['roll'] = way_point.get("EA_y", 90.0)
                        self.waypoints['yaw'] = way_point.get("EA_z", 0.0)
                        pos = way_point.get("pos", {"lat":0.0,"lng":0.0})
                        self.waypoints['target_lat'] = pos["lat"]
                        self.waypoints['target_lng'] = pos["lng"]
                        # self.waypoints['target_alt'] = pos["alt"]
                    else:
                        rospy.logwarn("接收的姿态信息格式错误")
                        time.sleep(0.05)
                        continue
            except socket.timeout:
                rospy.loginfo("超时没有收到姿态信息")
                continue
            except Exception as e:
                rospy.logwarn(f"ERROR receiving global point data: {e}")

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.waypoints:
                    msg = UavStatus()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "ned_frame"
                    msg.latitude = self.waypoints['latitude']
                    msg.longitude = self.waypoints['longitude']
                    msg.altitude = self.waypoints['altitude']
                    msg.roll = self.waypoints['roll']
                    msg.pitch = self.waypoints['pitch']
                    msg.yaw = self.waypoints['yaw']
                    msg.baro_altitude = self.waypoints['height']
                    msg.tf_altitude = self.waypoints['tfheight']
                    msg.velocity_n = self.waypoints['vn']
                    msg.velocity_e = self.waypoints['ve']
                    msg.velocity_d = self.waypoints['vz']
                    msg.g_latitude = self.waypoints['target_lat']
                    msg.g_longitude = self.waypoints['target_lng']
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