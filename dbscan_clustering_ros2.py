#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField 
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.distance import cdist
from builtins import int

class ClusterTracker:
    """聚类跟踪器，用于保持聚类的时间一致性（优化少点云无人机的动态匹配）"""
    def __init__(self, max_distance=1.0, min_lifetime=3, max_age=10):
        self.clusters = []  # 存储历史聚类信息
        self.max_distance = max_distance  # 聚类匹配的最大距离
        self.min_lifetime = min_lifetime  # 最小生存帧数
        self.max_age = max_age  # 最大消失帧数（目标短暂丢失不删除）
        self.next_id = 0
        
    def update(self, new_clusters):
        """更新聚类跟踪（优化少点云目标匹配逻辑）"""
        if not self.clusters:
            # 第一帧，直接初始化
            for cluster in new_clusters:
                self.clusters.append({
                    'id': self.next_id,
                    'center': cluster['center'],
                    'bbox': cluster['bbox'],
                    'size': cluster['size'],
                    'age': 0,
                    'matched_count': 1,
                    'history': [cluster]
                })
                self.next_id += 1
            return self.get_stable_clusters()
        
        # 计算距离矩阵进行匹配
        if new_clusters:
            old_centers = np.array([c['center'] for c in self.clusters])
            new_centers = np.array([c['center'] for c in new_clusters])
            distances = cdist(old_centers, new_centers)
            
            # 匈牙利匹配（简化版，适配少点云无人机）
            matched_pairs = []
            for i in range(len(self.clusters)):
                if len(new_clusters) > 0:
                    min_dist_idx = np.argmin(distances[i])
                    min_dist = distances[i][min_dist_idx]
                    # 优化：少点云目标（<5个点）放宽匹配距离（1.2倍阈值）
                    old_cluster_size = self.clusters[i]['size']
                    match_threshold = self.max_distance * 1.2 if old_cluster_size < 5 else self.max_distance
                    if min_dist < match_threshold:
                        matched_pairs.append((i, min_dist_idx))
                        distances[:, min_dist_idx] = np.inf
            
            # 更新匹配的聚类
            matched_new_indices = set()
            for old_idx, new_idx in matched_pairs:
                matched_new_indices.add(new_idx)
                self.clusters[old_idx]['matched_count'] += 1
                self.clusters[old_idx]['age'] = 0
                self.clusters[old_idx]['history'].append(new_clusters[new_idx])
                
                if len(self.clusters[old_idx]['history']) > 5:
                    self.clusters[old_idx]['history'].pop(0)
                
                # 指数移动平均更新
                alpha = 0.3
                old_cluster = self.clusters[old_idx]
                new_cluster = new_clusters[new_idx]
                
                old_cluster['center'] = alpha * new_cluster['center'] + (1 - alpha) * old_cluster['center']
                old_cluster['size'] = int(alpha * new_cluster['size'] + (1 - alpha) * old_cluster['size'])
                
                old_bbox = old_cluster['bbox']
                new_bbox = new_cluster['bbox']
                old_bbox['min'] = alpha * new_bbox['min'] + (1 - alpha) * old_bbox['min']
                old_bbox['max'] = alpha * new_bbox['max'] + (1 - alpha) * old_bbox['max']
                old_bbox['size'] = old_bbox['max'] - old_bbox['min']
            
            # 添加新的未匹配聚类
            for i, cluster in enumerate(new_clusters):
                if i not in matched_new_indices:
                    self.clusters.append({
                        'id': self.next_id,
                        'center': cluster['center'],
                        'bbox': cluster['bbox'],
                        'size': cluster['size'],
                        'age': 0,
                        'matched_count': 1,
                        'history': [cluster]
                    })
                    self.next_id += 1
        
        # 老化计数与过滤
        for cluster in self.clusters:
            if cluster['age'] < self.max_age:
                cluster['age'] += 1
        
        self.clusters = [c for c in self.clusters if c['age'] < self.max_age]
        
        return self.get_stable_clusters()
    
    def get_stable_clusters(self):
        """获取稳定的聚类（满足最小生存帧数要求）"""
        stable_clusters = []
        for cluster in self.clusters:
            if cluster['matched_count'] >= self.min_lifetime:
                stable_clusters.append(cluster)
        return stable_clusters

class PointCloudClusteringNode(Node):
    """ROS 2 点云聚类节点（适配低版本 + 无人机少点云 + 静态背景过滤）"""
    def __init__(self):
        # 初始化 ROS 2 节点
        super().__init__('pointcloud_clustering_node')
        
        # 聚类参数（强化误检过滤，适配无人机少点云）
        self.declare_parameter('eps', 0.4)          # 进一步减小聚类半径，减少噪声聚合
        self.declare_parameter('min_points', 2)     # 保留1个点的最小条件
        self.declare_parameter('max_points', 20)     # 缩小最大点数，排除静态大簇
        self.declare_parameter('min_cluster_size', 2)# 取消最小聚类限制
        self.declare_parameter('voxel_size', 0.01)  # 减小下采样尺寸，保留无人机点云
        self.declare_parameter('z_min', -20.0)        # 无人机最低飞行高度
        self.declare_parameter('z_max', 10.0)       # 无人机最高飞行高度
        self.declare_parameter('radius_filter', 200.0)  # 减少远距离噪声，150米范围
        
        # 获取参数值
        self.eps = self.get_parameter('eps').value
        self.min_points = self.get_parameter('min_points').value
        self.max_points = self.get_parameter('max_points').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.radius_filter = self.get_parameter('radius_filter').value
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # 静态背景过滤相关参数（强化稳定性，减少误检）
        self.background_pcd = None  # 存储静态背景点云
        self.background_update_threshold = 15  # 每15帧更新一次背景，提高稳定性
        self.frame_count = 0  # 帧计数器
        self.background_distance_threshold = 2  # 增大至0.4米，强化静态点过滤
        
        # 聚类跟踪器（优化无人机动态匹配）
        self.tracker = ClusterTracker(
            max_distance=0.4,  # 减小匹配距离，适配无人机低速移动
            min_lifetime=1,    # 保留单帧检测的无人机
            max_age=3          # 短暂丢失不删除跟踪
        )
        
        # 订阅者和发布者
        self.pc_subscriber = self.create_subscription(
            PointCloud2,
            '/lq_lidar_pointcloud',
            self.pointcloud_callback,
            1
        )
        
        self.cluster_publisher = self.create_publisher(
            MarkerArray,
            '/pointcloud_clusters',
            1
        )
        
        self.filtered_pc_publisher = self.create_publisher(
            PointCloud2,
            '/filtered_pointcloud',
            1
        )
        
        # TF 监听器（ROS 2 版本适配）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 日志输出
        self.get_logger().info("无人机少点云检测节点已启动")
        self.get_logger().info(f"聚类参数: eps={self.eps}, min_points={self.min_points}, max_points={self.max_points}")
        self.get_logger().info(f"预处理参数: z_range=[{self.z_min}, {self.z_max}], radius_filter={self.radius_filter}")

    def filter_static_background(self, current_points):
        """强化静态背景过滤：减少误检，优化背景稳定性"""
        # 1. 初始化背景
        if self.background_pcd is None or len(self.background_pcd) == 0:
            self.background_pcd = current_points.copy()
            self.get_logger().debug("初始化静态背景完成")
            return current_points
        
        # 2. 过滤孤立噪声点（当前帧<2个点直接返回空）
        if len(current_points) < 2:
            return np.array([])
        
        # 3. 背景点云下采样，提高计算效率
        background_pcd_o3d = o3d.geometry.PointCloud()
        background_pcd_o3d.points = o3d.utility.Vector3dVector(self.background_pcd)
        background_pcd_o3d_down = background_pcd_o3d.voxel_down_sample(voxel_size=0.5)
        background_points_down = np.asarray(background_pcd_o3d_down.points)
        
        # 4. 筛选动态点（最小距离>0.4米）
        dynamic_points = []
        for point in current_points:
            distances = np.linalg.norm(background_points_down - point, axis=1)
            min_distance = np.min(distances)
            if min_distance > self.background_distance_threshold:
                dynamic_points.append(point)
        
        # 5. 优化背景更新（仅动态点占比<10%时更新）
        self.frame_count += 1
        dynamic_ratio = len(dynamic_points) / len(current_points) if len(current_points) > 0 else 0
        if self.frame_count >= self.background_update_threshold and dynamic_ratio < 0.1:
            if len(current_points) > 0:
                self.background_pcd = np.vstack([self.background_pcd * 0.95, current_points * 0.05])
            self.frame_count = 0
            self.get_logger().debug("静态背景已更新（背景稳定）")
        
        # 6. 返回动态点云
        dynamic_points_np = np.array(dynamic_points) if len(dynamic_points) > 0 else np.array([])
        return dynamic_points_np

    def pointcloud_callback(self, msg):
        """点云回调函数（集成静态背景过滤 + 动态聚类跟踪）"""
        try:
            # 转换 ROS 2 PointCloud2 到 numpy 数组
            points_list = list(point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
            
            if len(points_list) == 0:
                self.get_logger().warn("接收到空点云数据")
                return
            
            # 转换为numpy数组
            points = np.array(points_list, dtype=np.float32)
            
            # 点云处理和聚类
            raw_clusters = self.process_pointcloud(points)
            
            # 获取稳定聚类
            stable_clusters = self.tracker.update(raw_clusters)
            
            # 过滤固定位置聚类（进一步减少误检）
            filtered_stable_clusters = []
            for cluster in stable_clusters:
                cluster_history = [h['center'] for h in cluster['history']]
                if len(cluster_history) < 2:
                    filtered_stable_clusters.append(cluster)
                    continue
                # 计算移动距离，>0.15米视为动态无人机
                last_center = cluster_history[-2]
                current_center = cluster_history[-1]
                move_distance = np.linalg.norm(np.array(current_center) - np.array(last_center))
                if move_distance > 0.15:
                    filtered_stable_clusters.append(cluster)
            
            # 发布结果
            if filtered_stable_clusters:
                self.publish_clusters(filtered_stable_clusters, msg.header)
                self.get_logger().info(f"检测到 {len(filtered_stable_clusters)} 个无人机目标（动态稳定聚类）")
            else:
                self.get_logger().warn("无动态稳定聚类，疑似全为静态背景")
            
        except Exception as e:
            self.get_logger().error(f"点云处理错误: {str(e)}")

    def process_pointcloud(self, points):
        """使用 Open3D 处理点云并进行聚类（集成静态背景过滤）"""
        try:
            # 创建 Open3D 点云对象
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # 1. 体素下采样
            if self.voxel_size > 0:
                pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            
            # 2. 静态背景过滤
            current_points_np = np.asarray(pcd.points)
            dynamic_points_np = self.filter_static_background(current_points_np)
            if len(dynamic_points_np) == 0:
                self.get_logger().warn("无动态点云，已过滤全部静态背景")
                return []
            pcd.points = o3d.utility.Vector3dVector(dynamic_points_np)

            # 3. 径向距离过滤
            center = np.array([0.0, 0.0, 0.0])
            distances = np.linalg.norm(np.asarray(pcd.points) - center, axis=1)
            pcd = pcd.select_by_index(np.where(distances < self.radius_filter)[0])
            
            # 4. Z 轴范围过滤
            points_np = np.asarray(pcd.points)
            z_mask = (points_np[:, 2] >= self.z_min) & (points_np[:, 2] <= self.z_max)
            pcd = pcd.select_by_index(np.where(z_mask)[0])
            
            if len(pcd.points) < self.min_points:
                self.get_logger().warn("过滤后点云过少，无法进行聚类")
                return []
            
            # 5. 移除离群点
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=3.0)
            
            # 6. 发布过滤后的点云
            self.publish_filtered_pointcloud(pcd)
            
            # 7. DBSCAN 聚类
            labels = np.array(pcd.cluster_dbscan(eps=self.eps, min_points=self.min_points))
            
            # 8. 处理聚类结果
            clusters = []
            max_label = labels.max()
            
            if max_label < 0:
                self.get_logger().debug("未检测到任何聚类")
                return clusters
            
            for i in range(max_label + 1):
                cluster_indices = np.where(labels == i)[0]
                
                if len(cluster_indices) > self.max_points:
                    continue
                
                cluster_points = np.asarray(pcd.points)[cluster_indices]
                bbox = self.compute_stable_bounding_box(cluster_points)
                
                clusters.append({
                    'points': cluster_points,
                    'center': np.mean(cluster_points, axis=0),
                    'size': len(cluster_indices),
                    'bbox': bbox
                })
            
            return clusters
            
        except Exception as e:
            self.get_logger().error(f"Open3D 处理错误: {str(e)}")
            return []

    def compute_stable_bounding_box(self, points):
        """计算稳定边界框（适配1~5个点的无人机）"""
        percentile = 1
        
        min_bound = np.percentile(points, percentile, axis=0)
        max_bound = np.percentile(points, 100-percentile, axis=0)
        
        # 增大最小边界框尺寸
        size = max_bound - min_bound
        min_size = 0.5
        for i in range(3):
            if size[i] < min_size:
                center = (min_bound[i] + max_bound[i]) / 2
                min_bound[i] = center - min_size / 2
                max_bound[i] = center + min_size / 2
        
        return {
            'min': min_bound,
            'max': max_bound,
            'size': max_bound - min_bound
        }

    def publish_filtered_pointcloud(self, pcd):
        """发布过滤后的动态点云（低版本兼容）"""
        try:
            points = np.asarray(pcd.points)
            if len(points) == 0:
                return
                
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "livox_frame"
            
            # 构建 PointField
            fields = [
                PointField(
                    name='x',
                    offset=0,
                    datatype=PointField.FLOAT32,
                    count=1
                ),
                PointField(
                    name='y',
                    offset=4,
                    datatype=PointField.FLOAT32,
                    count=1
                ),
                PointField(
                    name='z',
                    offset=8,
                    datatype=PointField.FLOAT32,
                    count=1
                ),
            ]
            
            # 发布消息
            pc_msg = point_cloud2.create_cloud(header, fields, points)
            self.filtered_pc_publisher.publish(pc_msg)
        
        except Exception as e:
            self.get_logger().error(f"发布过滤点云错误: {str(e)}")

    def publish_clusters(self, clusters, header):
        """发布聚类结果（低版本兼容，使用 LINE_LIST 构建立方体线框）"""
        try:
            marker_array = MarkerArray()
            
            for cluster in clusters:
                # 构建立方体线框 Marker（LINE_LIST 类型）
                marker = Marker()
                marker.header = header
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "drone_clusters"
                marker.id = cluster['id']
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                
                # 计算立方体顶点
                center = cluster['center']
                size = cluster['bbox']['size']
                half_x = size[0] / 2.0
                half_y = size[1] / 2.0
                half_z = size[2] / 2.0
                
                # 定义8个顶点
                vertices = [
                    [center[0]-half_x, center[1]-half_y, center[2]-half_z],
                    [center[0]+half_x, center[1]-half_y, center[2]-half_z],
                    [center[0]+half_x, center[1]+half_y, center[2]-half_z],
                    [center[0]-half_x, center[1]+half_y, center[2]-half_z],
                    [center[0]-half_x, center[1]-half_y, center[2]+half_z],
                    [center[0]+half_x, center[1]-half_y, center[2]+half_z],
                    [center[0]+half_x, center[1]+half_y, center[2]+half_z],
                    [center[0]-half_x, center[1]+half_y, center[2]+half_z]
                ]
                
                # 定义12条棱
                lines = [0,1, 1,2, 2,3, 3,0, 4,5, 5,6, 6,7, 7,4, 0,4, 1,5, 2,6, 3,7]
                
                # 填充顶点数据
                marker.points = []
                for v in vertices:
                    p = Point()
                    p.x = v[0]
                    p.y = v[1]
                    p.z = v[2]
                    marker.points.append(p)
                
                # 填充颜色数据
                colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
                color = colors[cluster['id'] % len(colors)]
                line_color = ColorRGBA()
                line_color.r = color[0]
                line_color.g = color[1]
                line_color.b = 0.0
                line_color.a = 0.9
                marker.colors = [line_color for _ in lines]
                
                # 线框参数
                marker.scale.x = 0.1  # 线宽
                marker.pose.orientation.w = 1.0
                marker.lifetime = rclpy.duration.Duration(seconds=3.0).to_msg()
                marker_array.markers.append(marker)
                
                # 文本标记
                text_marker = Marker()
                text_marker.header = header
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "drone_text"
                text_marker.id = cluster['id'] + 10000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                # 文本位置
                text_offset = 2.0
                text_marker.pose.position.x = center[0]
                text_marker.pose.position.y = center[1]
                text_marker.pose.position.z = center[2] + size[2]/2 + text_offset
                text_marker.pose.orientation.w = 1.0
                
                # 文本参数
                text_marker.scale.z = 2.0
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = f"Drone ID: {cluster['id']}\nPts: {cluster['size']}"
                text_marker.lifetime = rclpy.duration.Duration(seconds=3.0).to_msg()
                marker_array.markers.append(text_marker)
            
            # 发布可视化消息
            self.cluster_publisher.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f"发布聚类标记错误: {str(e)}")

def main(args=None):
    """ROS 2 节点入口函数"""
    rclpy.init(args=args)
    node = PointCloudClusteringNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("无人机检测节点已关闭（用户中断）")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
