import rospy
import airsim
import numpy as np
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

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


if __name__ == "__main__":
    rospy.init_node('node_airsim_orin_lidar', anonymous=True)
    airsim_ip = rospy.get_param("/net_setting/airsim_ip","192.168.11.110")
    uav_name = rospy.get_param("/net_setting/uav_name",'UAV210')
    client = airsim.MultirotorClient(ip = airsim_ip)
    client.confirmConnection()
    client.enableApiControl(True,vehicle_name = uav_name)
    # client.armDisarm(True,vehicle_name = uav_name)
    # client.takeoffAsync(vehicle_name = uav_name).join()
    rospy.loginfo("Connect to AirSim")

    cloud_pub = rospy.Publisher('/topic_airsim_orin_lidar',PointCloud2,queue_size=2)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        lidar_data = client.getLidarData(lidar_name='MyLidar1', vehicle_name = uav_name)
        if len(lidar_data.point_cloud) <= 3:
            rospy.logwarn("No orin cloud data received")
            points = np.asarray([[0,0,0]])
            # continue
        else:
            points = np.asarray(lidar_data.point_cloud, dtype = np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
        pc2_msg = pub_point_cloud2(points)
        cloud_pub.publish(pc2_msg)
        rospy.loginfo("原始点云个数 %d", points.shape[0])
        rate.sleep()


    #

