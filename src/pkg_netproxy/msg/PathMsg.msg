# 时间戳
std_msgs/Header header
# 是否有障碍（0=无，1=有）
int32 hb30
# 是否有路径（0=无，1=有）
int32 path_ok
# 与障碍物的最小距离
float32 min_dis
# 路径点数组（经纬度）
geometry_msgs/Point32[] path_points

