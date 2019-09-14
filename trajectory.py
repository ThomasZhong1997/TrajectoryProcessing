# 一个通用的轨迹数据处理python函数包
import pandas as pd
import numpy as np
import math
from osgeo import ogr
import Measure


# 投影坐标下距离计算函数
def distance(s_x, s_y, e_x, e_y):
    return math.sqrt(math.pow((s_x - e_x), 2) + math.pow((s_y - e_y), 2))


# 地理坐标下距离计算函数
def distance_spatial(s_lon, s_lat, e_lon, e_lat):
    s_rad_lat = s_lat * math.pi / 180.0
    e_rad_lat = e_lat * math.pi / 180.0
    a = s_rad_lat - e_rad_lat
    b = (s_lon - e_lon) * math.pi / 180.0
    s = 2 * math.asin(math.sqrt(math.pow(math.sin(a / 2), 2) + math.cos(s_rad_lat) * math.cos(e_rad_lat)
                                * math.pow(math.sin(b / 2), 2)))
    s = s * 6378137
    s = math.floor(s * 10000) / 10000.0
    return s


# Point类，包含基础的空间地理坐标以及语义信息等
class Point:
    def __init__(self, x, y, semantic_info):
        self.x = x
        self.y = y
        self.semantic_info = []
        for info in semantic_info:
            self.semantic_info.append(info)

    def output_info(self):
        info_str = str(self.semantic_info[0]) + ' ' + str(self.semantic_info[1]) + ' ' \
                   + str(self.x) + ' ' + str(self.y) + ' '
        for i in range(2, len(self.semantic_info)):
            info_str += str(self.semantic_info[i])
            info_str += ' '
        return info_str[:-1]

    # 私有函数，坐标转换函数需要
    def __transform_lat(self):
        lon = self.x - 105.0
        lat = self.y - 35.0
        ret = -100.0 + 2.0 * lon + 3.0 * lat + 0.2 * lat * lat + 0.1 * lon * lat + 0.2 * math.sqrt(math.fabs(lon))
        ret += (20.0 * math.sin(6.0 * lon * math.pi) + 20.0 * math.sin(2.0 * lon * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320.0 * math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
        return ret

    # 私有函数，坐标转换函数需要
    def __transform_lon(self):
        lon = self.x - 105.0
        lat = self.y - 35.0
        ret = 300.0 + lon + 2.0 * lat + 0.1 * lon * lon + 0.1 * lon * lat + 0.1 * math.sqrt(math.fabs(lon))
        ret += (20.0 * math.sin(6.0 * lon * math.pi) + 20.0 * math.sin(2.0 * lon * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lon * math.pi) + 40.0 * math.sin(lon / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lon / 12.0 * math.pi) + 300.0 * math.sin(lon / 30.0 * math.pi)) * 2.0 / 3.0
        return ret

    # 坐标转换函数，GCJ-02坐标系转至WGS-84坐标系
    def gcj02_wgs84(self):
        earth_radius = 6378245.0
        earth_eccentricity = 0.00669342162296594323
        d_longitude = self.__transform_lon()
        d_latitude = self.__transform_lat()
        rad_latitude = self.y / 180.0 * math.pi
        magic = math.sin(rad_latitude)
        magic = 1.0 - earth_eccentricity * magic * magic
        sqrt_magic = math.sqrt(magic)
        d_longitude = (d_longitude * 180.0) / (earth_radius / sqrt_magic * math.cos(rad_latitude) * math.pi)
        d_latitude = (d_latitude * 180.0) / ((earth_radius * (1 - earth_eccentricity)) / (magic * sqrt_magic) * math.pi)
        magic_longitude = self.x + d_longitude
        magic_latitude = self.y + d_latitude
        self.x = self.x * 2 - magic_longitude
        self.y = self.y * 2 - magic_latitude


# 轨迹类，用于存储轨迹的长度以及构成轨迹的点
class Trajectory:
    def __init__(self, t_id, point_list, coordinate):
        self.t_id = id
        self.coordinate = coordinate
        self.point_list = []
        self.dist = 0
        if len(point_list) != 0:
            self.point_list.append(point_list[0])
            for i in range(1, len(point_list)):
                start_point = point_list[i - 1]
                end_point = point_list[i]
                if coordinate == 'geo':
                    self.dist += distance_spatial(start_point.x, start_point.y, end_point.x, end_point.y)
                elif coordinate == 'prj':
                    self.dist += (start_point.x, start_point.y, end_point.x, end_point.y)
                self.point_list.append(point_list[i])


# 用于统一存储一类轨迹的类
# 必选参数：coordinate(string) -> 坐标类型 coordinate == 'geo' 地理坐标系 coordinate == 'prj' 投影坐标系
# 可选参数：filename(string) -> 文件所在位置 filename == '' 构建一个空的Set； filename != '' 从文件中读取轨迹 fields(list) -> 字段列表
class TrajectorySet:
    def __init__(self, coordinate, field_list=['Id', 'Timestamp', 'Longitude', 'Latitude'], filename=''):
        self.coordinate = coordinate
        self.field_list = []
        if filename == '':
            for field in field_list:
                self.field_list.append(field)
            self.trajectory_list = []
        else:
            self.trajectory_list = []
            try:
                input_stream = open(filename, 'r')
                field_line = input_stream.readline()
                if field_line[-1] == '\n':
                    field_line = field_line[:-1]
                field_line = field_line.split(' ')
                for field in field_line:
                    self.field_list.append(field)
                trajectory_count = int(input_stream.readline())
                for i in range(trajectory_count):
                    one_trajectory_points = []
                    point_number = int(input_stream.readline())
                    for j in range(point_number):
                        one_point_line = input_stream.readline()
                        if one_point_line[-1] == '\n':
                            one_point_line = one_point_line[:-1]
                        one_point_info = one_point_line.split(' ')
                        semantic_info = [one_point_info[0], one_point_info[1]]
                        for k in range(4, len(one_point_info)):
                            semantic_info.append(one_point_info[k])
                        new_point = Point(float(one_point_info[2]), float(one_point_info[3]), semantic_info)
                        one_trajectory_points.append(new_point)
                    self.add_trajectory(one_trajectory_points)
                    one_trajectory_points.clear()
            except IOError:
                print('Read File Fail! Please Check File Path & Format')

    def add_trajectory(self, point_list):
        new_trajectory = Trajectory(len(self.trajectory_list), point_list, self.coordinate)
        self.trajectory_list.append(new_trajectory)

    def output(self, output_filename):
        output_stream = open(output_filename, 'w')
        field_str = ''
        for field in self.field_list:
            field_str += str(field)
            field_str += ' '
        field_str = field_str[:-1]
        output_stream.write(field_str + '\n')
        output_stream.write(str(len(self.trajectory_list)) + '\n')
        for trajectory in self.trajectory_list:
            output_stream.write(str(len(trajectory.point_list)) + '\n')
            for one_point in trajectory.point_list:
                data_str = one_point.output_info()
                output_stream.write(data_str + '\n')
        output_stream.close()

    def output_shape_file(self, output_filename):
        driver = ogr.GetDriverByName('ESRI Shapefile')
        output_shape_file = driver.CreateDataSource(output_filename)
        new_layer = output_shape_file.CreateLayer('Trajectory', geom_type=ogr.wkbMultiLineString)

        field_definition = ogr.FieldDefn('ID', ogr.OFTString)
        field_definition.SetWidth(10)
        new_layer.CreateField(field_definition)

        field_definition = ogr.FieldDefn('Length', ogr.OFTReal)
        new_layer.CreateField(field_definition)

        for trajectory in self.trajectory_list:
            if trajectory.dist == 0:
                continue
            feature_definition = new_layer.GetLayerDefn()
            feature = ogr.Feature(feature_definition)
            wkt_string = 'MULTILINESTRING (('
            for one_point in trajectory.point_list:
                wkt_string += (str(one_point.x) + ' ' + str(one_point.y) + ',')
            wkt_string = wkt_string[:-1]
            wkt_string += '))'
            geometry = ogr.CreateGeometryFromWkt(wkt_string)
            feature.SetGeometry(geometry)
            feature.SetField('ID', str(trajectory.point_list[0].semantic_info[0]))
            feature.SetField('Length', trajectory.dist)
            new_layer.CreateFeature(feature)
            feature.Destroy()
        output_shape_file.Destroy()


class Channel:
    def __init__(self, channel_id):
        self.channel_id = channel_id


class FeatureMatrix:
    def __init__(self, trajectory_list, scale, features=[]):
        x_list = []
        y_list = []
        for trajectory in trajectory_list:
            for one_point in trajectory.point_list:
                x_list.append(one_point.x)
                y_list.append(one_point.y)
        top_left_x = min(x_list)
        top_left_y = max(y_list)
        bottom_right_x = max(x_list)
        bottom_right_y = min(y_list)
        if trajectory_list.coordinate == 'geo':
            dist_x = distance_spatial(top_left_x, top_left_y, bottom_right_x, top_left_y)
            dist_y = distance_spatial(top_left_x, top_left_y, top_left_x, bottom_right_y)
        else:
            dist_x = distance(top_left_x, top_left_y, bottom_right_x, top_left_y)
            dist_y = distance(top_left_x, top_left_y, top_left_x, bottom_right_y)
        self.rows = dist_y / scale + 1
        self.cols = dist_x / scale + 1
        density_matrix = np.zeros([self.rows, self.cols], dtype=np.int).tolist()
        for trajectory in trajectory_list:
            for one_point in trajectory.point_list:
                if trajectory.coordinate == 'geo':
                    dist_x = distance_spatial(top_left_x, one_point.y, one_point.x, one_point.y)
                    dist_y = distance_spatial(top_left_x, )


# 经过本步骤的重建后，Point对象的semantic_info成员中的属性存储顺序为[ID, Timestamp, .......(Other Infos)]
# 必选参数列表：filename(string) -> 文件所在位置   file_type(string) -> 文件类型   x_name(string) -> 经度字段名称
# y_name(string) -> 纬度字段名称 id_name(string) -> ID字段名称   ts_name(string) -> 时间戳字段名称
# 可选参数列表：other_info_name(list<string>) -> 除必选字段外的其余语义字段名称   coordinate_type -> 空间坐标类型: geo地理坐标 prj投影坐标
# tolerant_dist(double) -> 轨迹分割最大距离，默认400m   tolerant_time(double) -> 轨迹分割最大时间间隔，默认120秒
def get_trajectory_from_file(filename, file_type, x_name, y_name, id_name, ts_name, other_info_name=[],
                             coordinate_type='geo', tolerant_dist=800, tolerant_time=120, geo_change=True):
    try:
        field_list = [id_name, ts_name, x_name, y_name] + other_info_name
        trajectory_list = TrajectorySet(coordinate_type, field_list)
        if file_type == 'csv':
            ori_data = pd.read_csv(filename, sep=' ', header=0)
            ori_data = ori_data.sort_values(axis=0, by=[id_name, ts_name], ascending=True)
            one_track_point_list = []
            for i in range(1, len(ori_data)):
                s_record = ori_data[i - 1: i]
                e_record = ori_data[i: i + 1]
                semantic_info = [s_record[id_name].values[0], s_record[ts_name].values[0]]
                for info_name in other_info_name:
                    semantic_info.append(s_record[info_name].values[0])
                s_x = s_record[x_name].values[0]
                s_y = s_record[y_name].values[0]
                e_x = e_record[x_name].values[0]
                e_y = e_record[y_name].values[0]
                new_point = Point(s_x, s_y, semantic_info)
                if coordinate_type == 'geo':
                    dist = distance_spatial(s_x, s_y, e_x, e_y)
                elif coordinate_type == 'prj':
                    dist = distance(s_x, s_y, e_x, e_y)
                s_t = s_record[ts_name].values[0]
                e_t = e_record[ts_name].values[0]
                time_interval = e_t - s_t
                s_id = s_record[id_name].values[0]
                e_id = e_record[id_name].values[0]
                if geo_change:
                    new_point.gcj02_wgs84()
                if dist < tolerant_dist and time_interval < tolerant_time and i < len(ori_data) - 1 and s_id == e_id:
                    if time_interval != 0 and dist != 0:
                        one_track_point_list.append(new_point)
                else:
                    if len(one_track_point_list) > 1:
                        trajectory_list.add_trajectory(one_track_point_list)
                    one_track_point_list.clear()
    except IOError:
        print("file error")
    return trajectory_list


# 必选参数列表：trajectory_list(list) -> 轨迹列表   center(Point) -> 中心点   buffer(double) -> 缓冲区大小
# in_output_file(string) -> 进入站点轨迹输出文件   out_output_file(string) -> 离开站点轨迹输出文件
# 可选参数列表：keep_inside -> 是否保留buffer内部轨迹 True 保留 False 不保留
# if keep_inside == False: 进入轨迹的终点全部为center，离开轨迹的起点全部为center
# if keep_inside == True: 在buffer内，距离center最近的轨迹点为分割节点，在buffer外，距离center最远的轨迹点为分割节点
def split_in_out_trajectory_by_circle(trajectory_list, center, buffer,
                                      in_output_file, out_output_file, keep_inside=False):
    in_trajectories = TrajectorySet(trajectory_list.coordinate, trajectory_list.field_list)
    out_trajectories = TrajectorySet(trajectory_list.coordinate, trajectory_list.field_list)
    for trajectory in trajectory_list.trajectory_list:
        # 初始化点存储空间
        in_trajectory_points = []
        out_trajectory_points = []
        # 计算第一个点是否在目标范围内，在范围内则首次为出，否则首次为入
        # in_flag == True 为寻找进入轨迹，反之为寻找离开轨迹
        start_position = trajectory.point_list[0]
        if trajectory.coordinate == 'geo':
            dist = distance_spatial(start_position.x, start_position.y, center.x, center.y)
        else:
            dist = distance(start_position.x, start_position.y, center.x, center.y)
        if dist < buffer:
            in_flag = False
        else:
            in_flag = True
        # 判断用户是否需要保存内部点，使用不同的算法
        if not keep_inside:
            cross_once = False
            # 遍历处理每一条轨迹中的轨迹点
            for trajectory_point in trajectory.point_list:
                # 地理坐标和投影坐标分开计算空间距离
                if trajectory.coordinate == 'geo':
                    dist = distance_spatial(trajectory_point.x, trajectory_point.y, center.x, center.y)
                else:
                    dist = distance(trajectory_point.x, trajectory_point.y, center.x, center.y)
                # 进入轨迹到达buffer后直接截断，开始外部循环
                if in_flag:
                    if dist > buffer:
                        in_trajectory_points.append(trajectory_point)
                    else:
                        in_trajectory_points.append(center)
                        in_trajectories.add_trajectory(in_trajectory_points)
                        in_trajectory_points = []
                        out_trajectory_points.append(center)
                        in_flag = False
                # 轨迹离开buffer后开始向外延伸，若没有回到buffer内部则结束将其加入离开轨迹序列
                # 若回到了buffer内部需要将其按最远距离的点分割成两段，第一段为离开，第二段为进入
                # 添加完成后继续执行第二步，in_flag不变，直至结束
                else:
                    # 离开轨迹提取中，将不在buffer内的轨迹点加入点列表
                    cross_once = True
                    if dist > buffer:
                        out_trajectory_points.append(trajectory_point)
                    else:
                        # 遇到重新进入buffer内的情况需要对轨迹进行分段
                        if len(out_trajectory_points) > 1:
                            max_dist = 0
                            max_dist_index = 0
                            # 找到所有入队轨迹点中的最大距离点
                            for loop in range(len(out_trajectory_points)):
                                if trajectory.coordinate == 'geo':
                                    dist = distance_spatial(out_trajectory_points[loop].x,
                                                            out_trajectory_points[loop].y, center.x, center.y)
                                else:
                                    dist = distance(out_trajectory_points[loop].x, out_trajectory_points[loop].y,
                                                    center.x, center.y)
                                if dist > max_dist:
                                    max_dist = dist
                                    max_dist_index = loop
                            # 分割轨迹
                            for loop in range(max_dist_index, len(out_trajectory_points)):
                                in_trajectory_points.append(out_trajectory_points[loop])
                            in_trajectory_points.append(center)
                            true_out_trajectory_points = []
                            for loop in range(max_dist_index):
                                true_out_trajectory_points.append(out_trajectory_points[loop])
                            # 由分割完成的轨迹点生成轨迹对象
                            in_trajectories.add_trajectory(in_trajectory_points)
                            out_trajectories.add_trajectory(out_trajectory_points)
                            # 清空容器
                            in_trajectory_points.clear()
                            out_trajectory_points.clear()
                            out_trajectory_points.append(center)
                            true_out_trajectory_points.clear()
        else:
            cross_once = False
            for trajectory_point in trajectory.point_list:
                # 地理坐标和投影坐标分开计算空间距离
                if trajectory.coordinate == 'geo':
                    dist = distance_spatial(trajectory_point.x, trajectory_point.y, center.x, center.y)
                else:
                    dist = distance(trajectory_point.x, trajectory_point.y, center.x, center.y)
                if in_flag:
                    if dist >= buffer and not cross_once:
                        in_trajectory_points.append(trajectory_point)
                    else:
                        cross_once = True
                        if dist < buffer:
                            in_trajectory_points.append(trajectory_point)
                        else:
                            min_dist = 9999999999
                            min_dist_index = 0
                            for loop in range(len(in_trajectory_points)):
                                one_point = in_trajectory_points[loop]
                                if trajectory.coordinate == 'geo':
                                    dist = distance_spatial(one_point.x, one_point.y, center.x, center.y)
                                else:
                                    dist = distance(one_point.x, one_point.y, center.x, center.y)
                                if dist < min_dist:
                                    min_dist = dist
                                    min_dist_index = loop
                            true_in_trajectory_points = []
                            for loop in range(min_dist_index):
                                true_in_trajectory_points.append(in_trajectory_points[loop])
                            in_trajectories.add_trajectory(true_in_trajectory_points)
                            true_in_trajectory_points.clear()
                            in_flag = False
                            for loop in range(min_dist_index, len(in_trajectory_points)):
                                out_trajectory_points.append(in_trajectory_points[loop])
                            in_trajectory_points.clear()
                else:
                    if dist <= buffer and not cross_once:
                        out_trajectory_points.append(trajectory_point)
                    else:
                        cross_once = True
                        if dist > buffer:
                            out_trajectory_points.append(trajectory_point)
                        else:
                            max_dist = 0
                            max_dist_index = 0
                            for loop in range(len(out_trajectory_points)):
                                one_point = out_trajectory_points[loop]
                                if trajectory.coordinate == 'geo':
                                    dist = distance_spatial(one_point.x, one_point.y, center.x, center.y)
                                else:
                                    dist = distance(one_point.x, one_point.y, center.x, center.y)
                                if dist > max_dist:
                                    max_dist = dist
                                    max_dist_index = loop
                            true_out_trajectory_points = []
                            for loop in range(max_dist_index):
                                true_out_trajectory_points.append(out_trajectory_points[loop])
                            out_trajectories.add_trajectory(true_out_trajectory_points)
                            true_out_trajectory_points.clear()
                            in_flag = True
                            for loop in range(max_dist_index, len(out_trajectory_points)):
                                in_trajectory_points.append(out_trajectory_points[loop])
                            out_trajectory_points.clear()
        # 如果最后结束不在范围内，则把范围外的点构成一条轨迹
        if cross_once:
            if len(out_trajectory_points) > 1:
                out_trajectories.add_trajectory(out_trajectory_points)
                out_trajectory_points.clear()
            if len(in_trajectory_points) > 1:
                in_trajectories.add_trajectory(in_trajectory_points)
                in_trajectory_points.clear()
    # 统一输出
    in_trajectories.output(in_output_file)
    out_trajectories.output(out_output_file)
    return True


all_trajectory = get_trajectory_from_file('F:/wh_0620.txt', 'csv', 'Longitude', 'Latitude', 'DriverID',
                                          'Timestamp', ['Speed', 'Direction', 'Hdop'], 'geo')
all_trajectory.output('temp_trajectory.txt')
# split_in_out_trajectory_by_circle(result, Point(114.419, 30.609, []), 500, 'in.txt', 'out.txt', True)
# all_trajectory = TrajectorySet('geo', [], 'temp_trajectory.txt')
split_in_out_trajectory_by_circle(all_trajectory, Point(114.418885, 30.608901, []), 750, 'in.txt', 'out.txt', True)
in_trajectory = TrajectorySet('geo', [], 'in.txt')
in_trajectory.output_shape_file('in.shp')
out_trajectory = TrajectorySet('geo', [], 'out.txt')
out_trajectory.output_shape_file('out.shp')
