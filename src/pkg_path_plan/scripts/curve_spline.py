import math

import dubins
import numpy as np
from scipy import interpolate, linalg
import matplotlib.pyplot as plt

class CurveSpline:
    def __init__(self):
        pass

    def linear_interpolation_2d(self,path,step = 1.0):
        """
        线性插值函数，增加路径数量点
        Args:
            path: list [(x1,y1),(x2,y2)......]
            step: float

        Returns: dense_path list [(x1,y1),(x2,y2)......]

        """
        if len(path) < 2:
            return path
        dense_path = []
        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i+1])
            dist = np.linalg.norm(p2 - p1)
            num_points = max(round(dist/step),1)
            for j in range(num_points):
                t = j / num_points
                new_point = p1 * (1 - t) + p2 * t
                dense_path.append(new_point.tolist())
        dense_path.append(path[-1])
        return dense_path

    def douglas_peucker(self,points, epsilon):
        """
        简化曲线，仅保留弧度大于epsilon部分的点
        Args:
            points: np.array n*2
            epsilon: 弧度系数

        Returns: np.array n * 2

        """
        if len(points) < 3:
            return points
        start, end = points[0], points[-1]
        line_vec = np.subtract(end, start)
        line_vec_norm = line_vec / np.linalg.norm(line_vec)

        d_max = 0
        index = 0
        for i in range(1, len(points) - 1):
            vec = np.subtract(points[i], start)
            proj = np.dot(vec, line_vec_norm) * line_vec_norm
            d = np.linalg.norm(vec - proj)
            if d > d_max:
                d_max = d
                index = i
        if d_max > epsilon:
            left = self.douglas_peucker(points[:index + 1], epsilon)
            right = self.douglas_peucker(points[index:], epsilon)
            return np.vstack((left[:-1], right))
        else:
            return np.array([start, end])

    def BSpline(self,control_points,number,sm=0.3):
        """
        B样条插值 s=0时全部经过路径点，s越大平滑度越高  num越大也可以改善平滑度
        :param control_points:  list,  [[x1,x2,x3,...xn],[y1,y2,y3,...yn]]
        :param number: int,
        :return:  list, [[x1,x2,x3,.....xn],[y1,y2,y3,......yn]]
        """
        tck,u = interpolate.splprep(control_points,k = 3,s = sm)
        u=np.linspace(0,1,num=number,endpoint=True)
        out = interpolate.splev(u,tck)
        #out为列表，out[0]为B样条求得X的取值，数组，out[1]为B样条插值求得Y的取值，数组
        return out
    @staticmethod
    def gaussian_kernel(self,window_size,sigma):
        """生成高斯核"""
        half_size = window_size // 2
        kernel = np.array([np.exp(-0.5 * (i / sigma) ** 2)for i in range(-half_size,half_size+1)])
        return kernel / np.sum(kernel) #归一化

    def gaussian_smooth_path(self,x,y,window_size = 5,sigma = 1.0):
        """
        对路径进行高斯滤波平滑,不能保证一定经过路径点，是一种路径点拟合的方法
        :param x: list
        :param y: list
        :param window_size: int odd number
        :param sigma: float 越大越平滑
        return: list [(x0,y0),(x1,y1),(x2,y2)......]
        """
        kernel = self.gaussian_kernel(window_size,sigma)
        half_size = window_size // 2
        weights = np.linspace(2,1,len(x))
        weights = np.maximum(weights,np.flip(weights))
        smooth_path = []
        for i in range(len(x)):
            x_window = x[max(0,i-half_size):min(len(x),i+half_size+1)]
            y_window = y[max(0, i - half_size):min(len(y), i + half_size + 1)]
            weight_window = weights[max(0,i-half_size):min(len(x),i+half_size+1)]
            kernel_window = kernel[max(0,half_size-i):min(window_size,len(x)-i + half_size)]
            weighted_kernel = kernel_window * weight_window
            weighted_kernel /= np.sum(weighted_kernel)

            s_x = round(np.sum(x_window * weighted_kernel),2)
            s_y = round(np.sum(y_window * weighted_kernel),2)
            #-------强制通过起点和终点----------------
            if i == 0:
                s_x = x[0]
                s_y = y[0]
            if i == len(x) - 1:
                s_x = x[-1]
                s_y = y[-1]
            smooth_path.append((s_x,s_y))
        return smooth_path

    def curve_arc_angle(self,last,current,new):
        vector_1 = np.asarray((last[0]-current[0],last[1]-current[1]))
        vector_2 = np.asarray((new[0]-current[0],new[1]-current[1]))
        result = np.dot(vector_1,vector_2)
        if result > 0:
            vector_1_unit =  vector_1 / np.linalg.norm(vector_1)
            vector_1_new = np.array(current) + 1.5 * vector_1_unit
            vector_2_unit = vector_2 / np.linalg.norm(vector_2)
            vector_2_new = np.array(current) + 1.5 * vector_2_unit
            return tuple(vector_1_new),tuple(vector_2_new)
        else:
            return None

    def simple_spline(self,path):
        """
        简单的路径平滑方式，锐角变为钝角
        Args:
            path: list [(x0,y0),(x1,y1),(x2,y2)......]

        Returns: list [(x0,y0),(x1,y1),(x2,y2)......]

        """
        if len(path) < 3:
            return path
        smooth_path = [path[0]]
        try:
            for i in range(1,len(path)-1):
                curve_points = self.curve_arc_angle(path[i-1],path[i],path[i+1])
                if curve_points is not None:
                    smooth_path.append(curve_points[0])
                    smooth_path.append(curve_points[1])
                else:
                    smooth_path.append(path[i])
        except Exception as e:
            return path
            # print(e)
        if path[-1] in smooth_path:
            return smooth_path
        else:
            smooth_path.append(path[-1])
            return smooth_path

    def dubins_path(self,start, end, min_turn_radius=2.0):
        """
        杜宾曲线：解决在二维平面，具有固定初始和目标方向的移动物体寻找满足转弯半径约束的最短路径问题
        适用于大角度、硬约束的路径，当路径点的夹角较小时，使用贝塞尔或者角度插值会更合适
        path_types = ["LSL","LSR","RSL","RSR","RLR","LRL"]
        :param start: 起点 (x_s, y, stheta)
        :param end: 终点 (x_s, y, gtheta)
        :param min_turn_radius: 最小转弯半径
        :return: np.array n*3 [[x_s, y, rad],[x_s,y,rad],] 每个点都有位置和弧度要求
        """
        sx, sy, stheta = start
        gx, gy, gtheta = end
        theta = gtheta - stheta
        while theta > math.pi:
            theta -= 2 * math.pi
        while theta < -math.pi:
            theta += 2 * math.pi

        if abs(theta) < math.pi / 6:
            return None

        dx = gx - sx
        dy = gy - sy
        D = math.sqrt((dx ** 2 + dy ** 2))
        if D < 2 * min_turn_radius:
            return None

        path = dubins.shortest_path(start, end, min_turn_radius)
        if path:
            path_samples = path.sample_many(2.0)  # 离散化点的间隔
            return np.round(path_samples[0], 2)
        else:
            return None

if __name__ == '__main__':
    curve = CurveSpline()
    target_dis = [(0,0,0),(30,0,np.pi/2),(30,30,np.pi/4),(58.28,58.28,np.pi/4),
                  (72.42,72.42,-np.pi/2),(72.42,0,-np.pi),(50,0,-np.pi)]
    target_dis_array = np.asarray(target_dis)
    new_points = []
    for i in range(0, len(target_dis) - 1):
        po = target_dis_array[i]
        pon = target_dis_array[i + 1]
        start = (po[0], po[1], po[2])
        end = (pon[0], pon[1], pon[2])
        new_point = curve.dubins_path(start, end, 2.0)
        if new_point is not None:
            new_points.extend(new_point[:, 0:2].tolist())
        else:
            new_points.append(target_dis_array[i, 0:2].tolist())
            new_points.append(target_dis_array[i + 1, 0:2].tolist())

    new_points.append(target_dis_array[-1, 0:2].tolist())

    path = []
    for i in new_points:
        if i not in path:
            path.append(i)
    print(len(path))

    path_new = np.asarray(path)
    path_dg = curve.douglas_peucker(path,0.2)
    print(path_dg)
    start_pos = target_dis[0]
    end_pos = target_dis[-1]
    # path_new = curve.dubins_path(start_pos,end_pos,2.0)
    # print(path_new)
    plt.plot(path_new[:,0],path_new[:,1],'r')
    # plt.plot(target_dis_array[:,0],target_dis_array[:,1],'b--')
    plt.plot(path_dg[:, 0], path_dg[:, 1], 'kx')
    plt.plot(start_pos[0],start_pos[1],'go',markersize=10,label='Start')
    plt.plot(end_pos[0],end_pos[1],'ro',markersize=10,label='End')
    plt.legend()
    plt.grid()
    plt.show()
    # dis = np.linalg.norm(path_new[:,0:2],axis=1)
    # print(dis)







