import rospy
import subprocess
import os

class RvizManager:
    def __init__(self,rviz_config):

        self.rviz_config_path = os.path.expanduser(rviz_config)

        if not os.path.exists(self.rviz_config_path):
            rospy.logerr("Rviz 配置文件不存在: {}".format(self.rviz_config_path))
            return

        self.rviz_process = None
        rospy.on_shutdown(self.shutdown_hander)

    def start_rviz(self):
        try:
            self.rviz_process = subprocess.Popen(["rviz", "-d",self.rviz_config_path])
            rospy.loginfo("启动Rviz并成功加载配置: {}".format(self.rviz_config_path))
        except Exception as e:
            rospy.logerr("启动Rviz出错：{}".format(e))

    def shutdown_hander(self):
        if self.rviz_process and self.rviz_process.poll() is None:
            try:
                self.rviz_process.terminate()
                self.rviz_process.wait()
            except Exception as e:
                rospy.logerr("关闭Rviz失败，需手动关闭：%s",e)

if __name__ == "__main__":
    rospy.init_node('node_auto_rviz', anonymous=True)
    rviz_path = "/home/nvidia/airsim_ros/src/airsim.rviz"
    manager = RvizManager(rviz_path)
    manager.start_rviz()
    rospy.spin()
