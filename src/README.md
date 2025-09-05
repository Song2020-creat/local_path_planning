==============================说明(2025.09.05 WS)==============================
ROS包功能： AirSim仿真环境中二维平面避障
src目录下功能包：
1. airsim_bringup: 同时启动多个节点 命令：roslaunch airsim_bringup all.launch
2. common_config: 参数配置文件
3. pkg_airsim_lidar_cloud: 发布机体系（前-右-下）、世界坐标系（北-东-地）下的点云数据
4. pkg_map: 发布二维栅格地图
5. pkg_netproxy: 接收位姿和终点信息并发布；订阅局部路径话题并传递
6. pkg_path_plan: 发布二维局部路径

==============================使用方法==============================
1. roslaunch airsim_bringup all.launch
2. sudo python3 Main.py(tcUAV文件夹）


==============================使用前务必确认好两边的通信==============
1. UDP：pkg_netproxy/launch/telemetry.launch <arg name="udp_flag" default="True" />
2. unix(推荐)：pkg_netproxy/launch/telemetry.launch <arg name="udp_flag" default="False" />

