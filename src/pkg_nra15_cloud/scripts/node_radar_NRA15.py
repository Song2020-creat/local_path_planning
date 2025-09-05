import rospy
import serial
from sensor_msgs.msg import Range
from std_msgs.msg import Header

class NRA15:
    def __init__(self):
        self.is_open = False
        self.dev = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self._buf = bytearray()

        self.pub_distance = rospy.Publisher('/topic_NRA15_cloud',Range,queue_size=5)

    def start(self,dev,baud_rate):
        if not self.is_open:
            self.dev = dev
            self.baud_rate = baud_rate
            try:
                self.port = serial.Serial(self.dev, self.baud_rate, timeout=1)
                self.is_open = True
                rospy.loginfo("串口打开成功==============")
                rospy.loginfo("串口打开成功==============")
                rospy.loginfo("串口打开成功==============")
            except Exception as e:
                rospy.logerr('NRA15 start 异常==============',e)
                self.is_open = False
            finally:
                return self.is_open
    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.receive_data()
            rate.sleep()

    def receive_data(self):
        chunk = self.port.read(64)
        if chunk:
            self._buf.extend(chunk)
        while self.is_open:
            idx = self._buf.find(b'\xAA\xAA')
            if idx == -1:
                if len(self._buf) > 1:
                    del self._buf[:-1]
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < 14:
                break
            frame = bytes(self._buf[:14])

            if frame[12:14] != b'\x55\x55':
                del self._buf[0]
                continue
            del self._buf[:14]
            self.parse_and_publish(frame)

    def parse_and_publish(self,buf:bytes):
        msg_id = buf[2] | (buf[3] << 8)
        if msg_id == 0x070C:
            pay_load = buf[4:12]
            distance = (pay_load[2] << 8 + pay_load[3]) * 0.01

            rmsg = Range()
            rmsg.header = Header()
            rmsg.header.stamp = rospy.Time.now()
            rmsg.header.frame_id = 'NRA15_link'
            rmsg.radiation_type = Range.ULTRASOUND
            rmsg.field_of_view = 0.0
            rmsg.min_range = 0.0
            rmsg.max_range = 100.0
            rmsg.range = float(distance)

            self.pub_distance.publish(rmsg)

def main():
    rospy.init_node('node_NRA15_range',anonymous=False)
    node = None
    try:
        node = NRA15()
        flag = node.start('/dev/ttyUSB0',15200)
        if flag:
            node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node and node.port.is_open:
            node.port.close()
            node.is_open = False
        rospy.loginfo("NRA15 node stopped")

if __name__ == '__main__':
    main()




