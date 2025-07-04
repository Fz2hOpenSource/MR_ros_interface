#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
from tf2_ros import TransformListener
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from std_msgs.msg import String
import math
from collections import deque
from explore2.srv import DataCollection, DataCollectionResponse
from tf.transformations import quaternion_from_euler

import time



MAX_DURATION = rospy.Duration(3600).to_sec()
QUEUE_SIZE = 2



class EnhancedFilter:
    def __init__(self, window_size=20, stability_threshold=0.02, max_deviation=0.05,display=False):
        self.window_size = window_size
        self.stability_threshold = stability_threshold  # 判定为稳定状态的最大标准差
        self.max_deviation = max_deviation  # 单值允许的最大偏差
        self.data_history = []
        self.is_stable = False  # 标记当前是否处于稳定状态
        self.bias = 0  # 偏差值
        self.bias_calculated = False  # 是否已计算偏差

        self.display_cnt = 0
        self.is_display = display
        
        # ------------------- 新增时间追踪属性 -------------------
        self.start_time = None       # 首次调用 filter() 的时间（秒）
        self.stable_timestamp = None # 达到稳定状态的时间（秒）
        self.last_update_time = None # 最后一次更新状态的时间（用于计算单次间隔）
        
    def filter(self, value):
        # 添加修正后的值到历史记录
        value = round(value,3)
        self.data_history.append(value)
        self.display()


        # ------------------- 记录首次调用时间 -------------------
        if self.start_time is None:
            self.start_time = time.time()  # 记录首次调用的时间戳

        # 数据不足时返回None
        if len(self.data_history) < self.window_size:
            return None
            
        # 保持历史记录的长度固定
        if len(self.data_history) > self.window_size:
            self.data_history = self.data_history[-self.window_size:]
            
        # 计算统计量
        current_median = np.median(self.data_history)
        current_std = np.std(self.data_history)
        
        # 检测离群值并移除
        if abs(value - current_median) > self.max_deviation:
            if self.is_display:
                print(f"检测到离群值: {value:.3f}, 中位数: {current_median:.3f}")
            # 使用中位数作为替代值
            self.data_history[-1] = current_median
             
        
        # 更新统计量（移除离群值后）
        current_median = np.median(self.data_history)
        current_std = np.std(self.data_history)
        
        # 判断稳定性
        self.is_stable = current_std < self.stability_threshold


        # ------------------- 检测状态变化并记录时间 -------------------
        # 计算偏差（如果尚未计算且数据稳定）
        if not self.bias_calculated and self.is_stable:
            self.bias = - current_median  # 计算偏差
            self.bias_calculated = True
            if self.is_display:
                self.stable_timestamp = time.time()  # 记录稳定时刻的时间戳
                duration = self.stable_timestamp - self.start_time
                print(f"系统在 {duration:.2f} 秒后进入稳定状态")
                print(f"已计算偏差值: {self.bias:.3f}")

        # 根据稳定性选择输出
        if self.is_stable:
            # 稳定状态返回中位数（对离群值更鲁棒）
            if self.bias_calculated:
                return  current_median + self.bias
            else:
                return current_median
        else:
            # 非稳定状态返回加权平均值，最近的值权重更高
            weights = np.linspace(0.1, 1.0, len(self.data_history))
            weighted_avg = np.average(self.data_history, weights=weights)
            return weighted_avg

    def reset(self):
        # 清空历史数据，保留偏差值
        self.data_history = []
        self.is_stable = False
    
    def reset_all(self):
        # 完全重置，包括偏差值
        self.reset()
        self.bias = 0
        self.bias_calculated = False

    

    def display(self):
        if self.is_display:
            if self.display_cnt % self.window_size == 0:
                print('数据:\n',self.data_history)
            self.display_cnt += 1

    def wrap_filter(self, value):
        filtered_value = self.filter(value)
        if self.is_display:
            status = "稳定" if self.is_stable else "不稳定"
            bias_status = "已修正" if self.bias_calculated else "未修正"
            
            if filtered_value is not None:
                print(f"输入: {value:.3f} -> 过滤后: {filtered_value:.3f} ({status}, {bias_status})")
            else:
                print(f"输入: {value:.3f} -> 数据窗口未满，继续收集数据")
        if self.is_stable and self.bias_calculated:
            return filtered_value
        else:
            return None


class MessageCollector:
    def __init__(self):
        rospy.init_node('message_collector')
        # 从参数服务器获取配置
        self.robot_id = rospy.get_param('~robot_id', 0)
        self.topic_config = rospy.get_param('~topic_config', {})
        self.tf_config = rospy.get_param('~tf_config', {})
        self.filter_config = rospy.get_param('~filter_config', {})
        
        # 数据收集标志
        self.collecting_data = False
        self.data_ready = False
        
        # 初始化数据存储
        self.bridge = CvBridge()
        # self.rgb_buffer = deque(maxlen=10)
        # self.depth_buffer = deque(maxlen=20)
        # self.pose_buffer = deque(maxlen=10)
        self.rgb_buffer = [None]
        self.depth_buffer = [None]
        self.pose_buffer = [None]
        self.time_sync_threshold = rospy.Duration(20).to_sec()  # 时间差阈值
        self.max_latency = rospy.Duration(10.0).to_sec()     
        # 最大允许延迟

        self.current_data = {
            'rgb': None,
            'depth': None,
            'pose': None
        }
        
        # 从参数服务器获取滤波器配置
        window_size_x = self.filter_config.get('window_size_x', 10)
        window_size_y = self.filter_config.get('window_size_y', 10)
        window_size_yaw = self.filter_config.get('window_size_yaw', 20)
        display_filter = self.filter_config.get('display_filter', False)
        
        # 位置滤波器
        self.pos_filter = {
            'x': EnhancedFilter(window_size=window_size_x, display=display_filter),
            'y': EnhancedFilter(window_size=window_size_y, display=display_filter),
            'yaw': EnhancedFilter(window_size=window_size_yaw, display=display_filter)
        }
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(60))
        self.tf_listener = TransformListener(self.tf_buffer,tcp_nodelay = True) #tf.TransformListener()
        
        # 订阅传感器数据
        self.subscribe_sensors()
        self.tf_timer = rospy.Timer(rospy.Duration(0.1), self.update_tf)


        # 创建服务
        self.service_name = f'/robot{self.robot_id}/data_collection'
        self.service = rospy.Service(self.service_name, DataCollection, self.data_collection_handler)
        self.now = rospy.Time.now().to_sec()

        
        rospy.loginfo(f"机器人{self.robot_id}的数据收集服务已启动: {self.service_name}")
        rospy.loginfo(f"滤波器配置: x窗口大小={window_size_x}, y窗口大小={window_size_y}, yaw窗口大小={window_size_yaw}")
        rospy.loginfo(f"时间同步阈值: {self.time_sync_threshold}秒")
        rospy.loginfo(f"最大允许延迟: {self.max_latency}秒")
    
    def data_collection_handler(self, req):
        """数据收集服务处理函数"""
        response = DataCollectionResponse()
        self.now = rospy.Time.now().to_sec()
        if req.start_collection:
            rospy.loginfo(f"收到开始收集数据的请求，机器人{self.robot_id}开始收集数据")
            self.collecting_data = True
            self.data_ready = False
            
            # 重置滤波器和数据
            self.reset_filter()
            
            # 等待数据收集完成
            timeout = rospy.Time.now() + rospy.Duration(20.0)  # 设置60秒超时
            rate = rospy.Rate(1) 
            while not rospy.is_shutdown() and rospy.Time.now() < timeout:
                # 尝试同步数据
                synced_data = self.sync_data()
                if synced_data:
                    # 检查同步数据的时效性
                    latest_time = max(synced_data['rgb_time'], synced_data['depth_time'], synced_data['pose_time'])
                    current_time = rospy.Time.now().to_sec()
                    latency = (current_time - latest_time)
                    
                    if latency < self.max_latency:
                        self.current_data = synced_data
                        self.data_ready = True
                        break
                    else:
                        rospy.logwarn(f"{self.robot_id}:同步数据延迟过大: {latency:.3f}秒")
                rate.sleep()
            
            if self.data_ready:
                # 构建姿态消息
                pose_msg = Pose()
                pose_msg.position = Point(*self.current_data['pose']['position'])
                pose_msg.orientation = Quaternion(*self.current_data['pose']['orientation'])
                # 转换图像为ROS消息
                try:
                    rgb_msg = self.current_data['rgb'] # self.bridge.cv2_to_imgmsg(self.current_data['rgb'], "rgb8")
                    # print(f"{} self.current_data['depth']",))
                    
                    depth_msg = self.bridge.cv2_to_imgmsg(self.current_data['depth']) # self.current_data['depth']  #  # self.current_data['depth'] 
                except Exception as e:
                    rospy.logerr(f"图像转换失败: {e}")
                    response.success = False
                    response.message = f"图像转换失败: {e}"
                    return response
                
                # 填充响应
                response.success = True
                response.message = f"机器人{self.robot_id}的数据收集完成"
                response.rgb_image = rgb_msg
                response.depth_image = depth_msg
                response.pose = pose_msg
                
                time_diff = self.current_data['max_time_diff']
                latency = self.current_data['latency']

                rospy.loginfo(f"成功收集机器人{self.robot_id}的数据，时间差: {time_diff:.3f}秒，延迟: {latency:.3f}秒")
            else:
                response.success = False
                response.message = f"机器人{self.robot_id}的数据收集超时或无法同步"
                rospy.logwarn(response.message)
            
            # # 清空缓存:
            # self.rgb_buffer = deque(maxlen=20)
            # self.depth_buffer = deque(maxlen=20)
            # self.pose_buffer = deque(maxlen=10)

            self.rgb_buffer = [None]
            self.depth_buffer = [None]
            self.pose_buffer = [None]
            self.collecting_data = False
            return response
        else:
            response.success = False
            response.message = "未请求数据收集"
            return response

        
        
    def subscribe_sensors(self):
        """订阅传感器数据"""
        # RGB图像
        rgb_topic = self.topic_config.get('rgb', '')
        rgb_type = self.topic_config.get('rgb_type', 'CompressedImage')
        
        if rgb_topic and rgb_type:
            if rgb_type == 'CompressedImage':
                self.rgb_sub = rospy.Subscriber(rgb_topic, CompressedImage, self.rgb_callback, queue_size=QUEUE_SIZE)
            else:
                self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_callback, queue_size=QUEUE_SIZE)
        else:
            rospy.logwarn("未配置RGB图像话题")
        
        # 深度图像
        depth_topic = self.topic_config.get('depth', '')
        depth_type = self.topic_config.get('depth_type', 'Image')
        
        if depth_topic and depth_type:
            if depth_type == 'CompressedImage':
                self.depth_sub = rospy.Subscriber(depth_topic, CompressedImage, self.compressed_depth_callback, queue_size=QUEUE_SIZE,buff_size=2**24,tcp_nodelay=True)
            else:
                self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=QUEUE_SIZE)
        else:
            rospy.logwarn("未配置深度图像话题")


    def rgb_callback(self, msg):
        """RGB图像回调函数"""

        if not self.collecting_data:
            return
        if msg.header.stamp.to_sec() < self.now:
            return 
        self.rgb_buffer[0] = (msg.header.stamp.to_sec(), msg)
        # self.rgb_buffer.append((msg.header.stamp, msg))


    def rgb_process(self, msg):
        # msg = msg[1]
        if isinstance(msg, CompressedImage):
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        else:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if len(cv_image.shape) == 3:  # 确保是彩色图像
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        return cv_image
    

    def depth_callback(self, msg):
        """深度图像回调函数"""
        if not self.collecting_data:
            return
        depth_image = self.bridge.imgmsg_to_cv2(msg)
        # self.depth_buffer = depth_image
        self.depth_buffer[0] = (msg.header.stamp, depth_image)
        # 保持缓冲区按时间排序

    def compressed_depth_callback(self, msg):
        """压缩深度图像回调函数"""

        if not self.collecting_data:
            return
        if msg.header.stamp.to_sec() < self.now:
            return 
        # # 处理压缩深度图像
        depth_data = np.frombuffer(msg.data, np.uint8)
        # print("depth_data:", depth_data[:20])
        # if not np.all(depth_data[:12] == 0):
        #     print("错误：前12位不全为0")

        depth_data = depth_data[12:]  # 跳过头部信息
        


        depth_image = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED)

        if depth_image is not None:
            # 转换为32位浮点深度图
            depth_image = depth_image.astype(np.float32)
            self.depth_buffer[0] =  (msg.header.stamp.to_sec(),depth_image)




    def update_tf(self,event):
        if not self.collecting_data:
            return
        
        """更新TF变换并应用滤波"""
        try:
            target_frame = self.tf_config.get('target_frame', '')
            source_frame = self.tf_config.get('source_frame', '')
            
            if not target_frame or not source_frame:
                rospy.logwarn("未配置TF框架")
                return
            trans = None
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
                transform_time = transform.header.stamp.to_sec()
                trans = (transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z)
                
                if self.now > transform_time:
                    return
                
                # 提取旋转信息 (等效于原来的 rot，四元数格式)
                rot = (transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w)

                # self.tf_listener.waitForTransform(target_frame, source_frame, now, rospy.Duration(1.0))

                # (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, now)
                # transform_time = now
                # transform_time = self.tf_listener.getLatestCommonTime(target_frame, source_frame,now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"获取最新TF失败:{e}")
                # # 如果获取最新失败，尝试获取最近的有效变换
                # transform_time = rospy.Time.now() - rospy.Duration(0.1)
                # (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, transform_time)
            
            if trans is not None:
                # 应用滤波
                filtered_x = self.pos_filter['x'].wrap_filter(trans[0])
                filtered_y = self.pos_filter['y'].wrap_filter(trans[1])
                
                yaw_rad = quaternion_to_yaw_(rot)
                yaw_deg = math.degrees(yaw_rad)
                filtered_yaw_deg = self.pos_filter['yaw'].wrap_filter(yaw_deg)
                
                if filtered_yaw_deg is not None and filtered_x is not None and filtered_y is not None:
                    # if self.robot_id == 1:
                    #     rospy.loginfo(f"x:{filtered_x},y:{filtered_y},angle:{filtered_yaw_deg},time:{transform_time}")
                    # if abs(filtered_yaw_dx:eg) < 0.1:
                    #     filtered_yaw_deg = 0
                    filtered_yaw_rad = math.radians(filtered_yaw_deg)
                    
                    # 创建四元数
                    quat = quaternion_from_euler(0, 0, filtered_yaw_rad)
                    
                    # 更新姿态数据
                    pose_data = {
                        'position': (filtered_x, filtered_y, 0),
                        'orientation': (quat[0], quat[1], quat[2], quat[3])
                    }
                    # 添加到姿态缓冲区a
                    self.pose_buffer[0] = (transform_time,pose_data) 
                    # rospy.loginfo(f"t{self.robot_id}:  {transform_time.to_sec()}")
                    # 保持缓冲区按时间排序
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF获取失败: {e}")
        except Exception as e:
            rospy.logerr(f"TF处理异常: {e}")
    
    def reset_filter(self):
        """重置滤波器"""
        self.pos_filter['x'].reset()
        self.pos_filter['y'].reset()
        self.pos_filter['yaw'].reset()

    def sync_data(self):
        """同步RGB、深度和姿态数据，确保时间差不超过阈值"""
        if self.rgb_buffer[0] is None or self.depth_buffer[0] is None or self.pose_buffer[0] is None:
            return None

        # 获取最新的姿态数据点
        latest_pose_time,latest_pose = self.pose_buffer[0]

        # best_rgb_time = None
        # best_rgb_img = None
        # rgb_diff = MAX_DURATION

        # for rgb_time, rgb_img in self.rgb_buffer:
        #     diff = abs((rgb_time - latest_pose_time).to_sec())
        #     if diff < rgb_diff:
        #         rgb_diff = diff
        #         best_rgb_time = rgb_time
        #         best_rgb_img = rgb_img
        best_rgb_time, best_rgb_img = self.rgb_buffer[0]
        # best_rgb_img =  self.rgb_buffer
        # rgb_diff = abs((best_rgb_time - latest_pose_time).to_sec())
        # print("rgb_diff:",rgb_diff)

        # best_depth_time = None
        # best_depth_img = None
        # depth_diff = MAX_DURATION

        # for depth_time, depth_img in self.depth_buffer:
        #     diff = abs((depth_time - latest_pose_time).to_sec())
        #     if diff < depth_diff:
        #         depth_diff = diff
        #         best_depth_time = depth_time
        #         best_depth_img = depth_img

        
        best_depth_time , best_depth_img =  self.depth_buffer[0]
        # _depth_diff = abs((_best_depth_time - latest_pose_time).to_sec())
        # print("_depth_diff:",_depth_diff,"depth_diff",depth_diff)

        # 计算三个时间点的最大差异
        d2r = best_depth_time - best_rgb_time
        d2p = best_depth_time - latest_pose_time
        r2p = best_rgb_time - latest_pose_time
        rospy.loginfo(f"depth:{best_depth_time}\nrgb:{best_rgb_time}\npose:{latest_pose_time}\nd2r:{d2r}\nd2p:{d2p}\nr2p:{r2p}")
        max_diff =  d2p # max(d2p, depth_diff)
        # 检查时间差是否在可接受范围内
        if max_diff > self.time_sync_threshold:
            rospy.logwarn(f"{self.robot_id}：时间差过大: {max_diff:.3f}秒，阈值: {self.time_sync_threshold}秒")
        #     return None
        
        # 计算数据延迟
        current_time = rospy.Time.now().to_sec()
        latency = (current_time - latest_pose_time)
        
        # 返回同步的数据，包含时间戳和延迟信息
        return {
            'rgb': best_rgb_img, # self.rgb_process(best_rgb_img),
            'rgb_time': best_rgb_time,
            'depth': best_depth_img,
            'depth_time': best_depth_time,
            'pose': latest_pose,
            'pose_time': latest_pose_time,
            'max_time_diff': max_diff,
            'latency': latency
        }

if __name__ == '__main__':
    try:
        collector = MessageCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass