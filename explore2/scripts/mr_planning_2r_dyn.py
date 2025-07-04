#!/usr/bin/env python3
import rospy
import numpy as np
from explore2.srv import DataCollection
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Twist, TransformStamped
from cv_bridge import CvBridge
from std_msgs.msg import Float32 

import threading
import time
import cv2
import tf2_ros
from icecream import ic
ic.disable()


# ===================== NEW DYNAMIC RECONFIGURE IMPORTS =====================
from dynamic_reconfigure.server import Server
from explore2.cfg import MRPlannerConfig  # This will be generated from your .cfg file
# ===================== END OF NEW IMPORTS =================================

def step2mile(step):
    if isinstance(step,list):
        return [step2mile(step_) for step_ in step]
    elif isinstance(step,str):
        step = int(step)
        return round(step*30/100,2)
    return round(step*30/100,2)


class MRPlanningNode:
    def __init__(self):
        rospy.init_node('mr_planning_node')
        
        # 获取参数
        self.debug =  rospy.get_param('~debug',False)
        self.num_agents = rospy.get_param('~num_agents', 2)
        self.agent_ids = list(range(self.num_agents))

        if self.debug:
            print("当前使用测试模式")

        # 多机器人相对位姿 这个列表的数量一定要和机器人num_agents对应上的
        dx_list = rospy.get_param('dx', [0, step2mile(0), step2mile(30)]) # = [0,step2mile(8)]
        self.dx = step2mile(dx_list)

        dy_list = rospy.get_param('dy', [0, step2mile(-30), step2mile(0)]) # = [0,step2mile(15)]
        self.dy = step2mile(dy_list)
        self.set_global_map_tf()


        cmd_vel_table = rospy.get_param('cmd_vel_topic_table', ["/robot1/cmd_vel", '/robot2/cmd_vel','/robot3/cmd_vel'])

        self.cmd_vel_table  = cmd_vel_table

        self.planning_lock = threading.Lock()

        # ===================== NEW DYNAMIC RECONFIGURE SETUP =====================
        # This set will store the agent IDs that are manually disabled

        self.manually_disabled_agents = set() 
        # Set up the dynamic reconfigure server and link it to the callback
        self.dyn_reconfig_server = Server(MRPlannerConfig, self.dynamic_reconfig_callback)
        # ===================== END OF NEW SETUP =================================

        self.last_yaws = {}
        self.data_collection_lock = threading.Lock() 

        # 创建服务客户端
        self.service_clients = {}
        for agent_id in self.agent_ids:
            service_name = f'/robot{agent_id}/data_collection'
            rospy.loginfo(f"等待服务: {service_name}")
            try:
                rospy.wait_for_service(service_name, timeout=10.0)
                self.service_clients[agent_id] = rospy.ServiceProxy(service_name, DataCollection)
                rospy.loginfo(f"成功连接到服务: {service_name}")
            except rospy.ROSException:
                rospy.logerr(f"等待服务 {service_name} 超时，请检查服务是否已启动！")
                rospy.signal_shutdown(f"无法连接到必要服务 {service_name}")
                return
        
        # 创建命令发布器
        self.cmd_publishers = []
        for agent_id in self.agent_ids:
            topic_name = self.cmd_vel_table[agent_id]

            if 'none' in topic_name:
                rospy.loginfo(f"暂时取消了驱动机器人{agent_id}的运动话题")
            self.cmd_publishers.append(rospy.Publisher(topic_name, Twist, queue_size=1))
        
        # 其他初始化
        self.bridge = CvBridge()
        self.initial_stage = True
        self.initial_flag = True

        self.next_actions = []
        self.cnt = 0

        # 自动化规划
        rospy.Subscriber('/request_planning', Float32, self.planning_callback,queue_size=1)
        self.timing_planning = rospy.Publisher('/wait_request', Float32, queue_size=1)
        self.wait_msg = Float32()
        self.wait_msg.data = 5.0  # 默认等待5秒
        
        # 性能统计相关
        self.performance_data = {
            'iteration_count': 0,           # 迭代计数
            'total_processing_time': 0.0,   # 总处理时间
            'min_processing_time': float('inf'),  # 最小处理时间
            'max_processing_time': 0.0,     # 最大处理时间
            'last_10_times': [],            # 最近10次处理时间
            'start_time': 0.0,              # 开始时间戳
            'end_time': 0.0,                # 结束时间戳
        }
        rospy.loginfo("多机器人规划节点已启动，等待规划请求...")
        
    # ===================== NEW DYNAMIC RECONFIGURE CALLBACK =====================
    def dynamic_reconfig_callback(self, config, level):
        """
        This function is called whenever a parameter is changed in the dynamic_reconfigure server.
        """
        rospy.loginfo("Reconfigure Request: Disabled agents string is '{}'".format(config.disabled_agents_str))
        
        disabled_ids = set()
        # Parse the comma-separated string into a set of integers
        if config.disabled_agents_str: # Check if the string is not empty
            parts = config.disabled_agents_str.split(',')
            for part in parts:
                try:
                    # strip() handles whitespace, e.g., "0, 1"
                    disabled_ids.add(int(part.strip()))
                except ValueError:
                    rospy.logwarn(f"Invalid value '{part}' in disabled_agents_str. Ignoring.")
        
        self.manually_disabled_agents = disabled_ids
        rospy.loginfo(f"Manually disabled agents are now: {self.manually_disabled_agents}")
        
        # It's required to return the config object
        return config
    # ===================== END OF NEW CALLBACK =================================

    def planning_callback(self, msg):
        """处理规划请求"""
        if not self.planning_lock.acquire(blocking=False):
            rospy.logwarn("正在处理上一个规划请求，忽略此次新的请求。")
            return
        try:
            # 记录开始时间
            self.performance_data['start_time'] = time.time()

            print("收到规划请求，开始收集数据...")
            
            # 收集所有机器人数据
            collected_data = self.collect_all_robot_data(timeout_per_call=30.0, max_retries=10)
            
            if collected_data is None:
                rospy.logerr("数据收集失败，跳过本次规划, 3s后自动重试")
                self.wait_msg.data = 3.0
                self.timing_planning.publish(self.wait_msg)
                return
            
            print("所有机器人数据收集完成，开始规划...")
            
            # 处理收集到的数据
            processed_data = self.process_collected_data(collected_data)
            
            # 执行规划
            self.execute_planning(processed_data)
            # 进入自动规划
            self.wait_msg.data = 0.2
            self.timing_planning.publish(self.wait_msg)
            print(f"规划执行完成，{self.wait_msg.data}秒后将自动重试")
        # except Exception as e:
        #     rospy.logerr(f"在规划回调中发生严重错误: {e}")
        #     # 即使出错也要尝试触发下一次，但延迟长一点
        #     wait_msg = Float32(data=5.0)
        #     self.timing_planning.publish(wait_msg)
        finally:
            # 6. 更新性能统计并释放锁，让下一次回调可以进入
            self.performance_data['end_time'] = time.time()
            self._update_performance_stats()
            self.planning_lock.release()
            rospy.loginfo("规划锁已释放，等待下一次请求。")

    def _update_performance_stats(self):
        """更新性能统计数据"""
        duration = self.performance_data['end_time'] - self.performance_data['start_time']
        
        # 更新统计数据
        self.performance_data['iteration_count'] += 1
        self.performance_data['total_processing_time'] += duration
        self.performance_data['min_processing_time'] = min(self.performance_data['min_processing_time'], duration)
        self.performance_data['max_processing_time'] = max(self.performance_data['max_processing_time'], duration)
        
        # 更新最近10次处理时间
        self.performance_data['last_10_times'].append(duration)
        if len(self.performance_data['last_10_times']) > 10:
            self.performance_data['last_10_times'].pop(0)
        
        # 计算平均处理时间和FPS
        avg_time = self.performance_data['total_processing_time'] / self.performance_data['iteration_count']
        current_fps = 1.0 / duration if duration > 0 else 0
        avg_fps = 1.0 / avg_time if avg_time > 0 else 0
        
        # 输出性能统计信息
        print(f"===== 性能统计 =====")
        print(f"迭代次数: {self.performance_data['iteration_count']}")
        print(f"当前处理时间: {duration:.4f}秒")
        # print(f"最小处理时间: {self.performance_data['min_processing_time']:.4f}秒")
        # print(f"最大处理时间: {self.performance_data['max_processing_time']:.4f}秒")
        print(f"平均处理时间: {avg_time:.4f}秒")
        print(f"当前FPS: {current_fps:.2f}")
        print(f"平均FPS: {avg_fps:.2f}")
        # print(f"最近10次处理时间: {[round(t, 4) for t in self.performance_data['last_10_times']]}")
        if self.cnt < 100 and avg_fps != 0:
            print(f"当前距离完成100步还需要{(100 - self.cnt) * (1 / avg_fps) /60 :1f}分钟")
        print(f"===================")

    def collect_all_robot_data(self, timeout_per_call=5.0, max_retries=3):
        """收集所有机器人的数据，包含重试和yaw值验证逻辑。"""
        results = {}
        all_agents_set = set(self.agent_ids)
        agents_to_query = all_agents_set.copy()


        #for attempt in range(max_retries):
        attempt = 0
        while True:
            if not agents_to_query:
                break  # 所有数据都已成功并验证通过

            rospy.loginfo(f"数据收集尝试 #{attempt + 1}/{max_retries}. "
                          f"正在为机器人 {list(agents_to_query)} 请求数据...")
            threads = []
            for agent_id in agents_to_query:
                # 为每个待查询的 agent 创建一个线程
                thread = threading.Thread(target=self.collect_robot_data, args=(agent_id, results))
                threads.append(thread)
                thread.start()

            for thread in threads:
                thread.join(timeout_per_call)

            successful_agents = set(results.keys())
            agents_to_query = all_agents_set - successful_agents

            if agents_to_query:
                rospy.logwarn(f"尝试 #{attempt + 1} 后，机器人 {list(agents_to_query)} 未能响应或失败。")
                if attempt < max_retries - 1:
                    rospy.sleep(0.5)

            attempt += 1
        if len(results) != self.num_agents:
            rospy.logerr(f"数据收集最终失败！在 {max_retries} 次尝试后，"
                         f"预期 {self.num_agents} 个，实际收到 {len(results)} 个。"
                         f"失败的机器人: {list(agents_to_query)}")
            return None
        
                
        rospy.loginfo("所有机器人数据均已成功收集。")
        return results

    def collect_robot_data(self, agent_id, results_dict):
        """
        (线程工作函数) 收集单个机器人的数据，并验证yaw值变化。
        """
        try:
            response = self.service_clients[agent_id](True)  # 调用服务
            if response.success:
                results_dict[agent_id] = response
            else:
                rospy.logwarn(f"机器人 {agent_id} 的服务返回失败: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"调用机器人 {agent_id} 的服务时发生异常: {e}")
    
    def process_collected_data(self, collected_data):
        """处理收集到的数据"""
        processed = {
            'rgb': {},
            'depth': {},
            'poses': {},
            'maps': {agent_id : None for agent_id in self.agent_ids}
        }
        
        for agent_id, response in collected_data.items():
            # 处理RGB图像

            cv_image = self.bridge.compressed_imgmsg_to_cv2(response.rgb_image, desired_encoding="rgb8")
            processed['rgb'][agent_id] = cv_image

            
            # 处理深度图像
            # depth_data = response.depth_image.data
            # print("main_depth_data:",depth_data[20:])
            #  depth_data = depth_data[12:]  # 跳过头部信息
            # depth_image = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED)
            # depth_image = depth_image.astype(np.float32)
            depth_image = self.bridge.imgmsg_to_cv2(response.depth_image, desired_encoding="32FC1")
            processed['depth'][agent_id] = depth_image

            
            # 处理位姿数据
            pose = response.pose
            orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            processed['poses'][agent_id] = {
                'position': (pose.position.x + self.dx[agent_id], pose.position.y + self.dy[agent_id], pose.position.z),
                'orientation': orientation
            }
                    
        return processed
    
    def execute_planning(self, ros_obs):
        rospy.loginfo(f"planning {self.cnt}")
        """执行规划算法"""

        if self.check_exploration_complete():
            return [] # 如果已触发关闭，返回一个空动作列表以避免后续代码出错
        if self.initial_stage:
            # 完成集中式规划算法的初始化
            # args = get_args()
            # self.MAE = MultiAgentExploration(args, ros_obs,vh.VisualizationHandler(self.dx,self.dy))
            # self.next_actions = self.loop()
            self.initial_stage = False
        else: 
            # TODO: 自行按照集中式规划算法进行规划           
            # self.MAE.obs, self.MAE.infos = self.MAE.envs.env.step_second_half(ros_obs)
            # self.MAE.record_process()
            # self.MAE.update_info(self.next_actions)
            # self.next_actions = self.loop()
            pass
        self.cnt += 1
    
    def loop(self):
        """规划循环"""
        self.MAE.check_use_global()
        next_actions = self.MAE.get_next_step_action(30)
        if len(self.MAE.stop_exploration_flags) > 0 :
            for stop_agent_id in self.MAE.stop_exploration_flags:
                print(stop_agent_id)

        self.MAE.state_update(next_actions)
        low_level_actions = self.MAE.implement_action(next_actions)
        self.send_action(low_level_actions)
        return next_actions
    
    def send_action(self, actions):
        """多线程发送动作命令到机器人"""
        action = self.MAE.envs.env.step_first_half(action=actions)
        rospy.loginfo(f"动作指令: {action}")
        threads = []

        for i in self.agent_ids:
            if i in self.MAE.stop_exploration_flags or i in self.manually_disabled_agents:
                rospy.loginfo(f"Agent {i}: Skipping action command.")
                continue # 阻止运动
            thread = threading.Thread(target=self._send_agent_action, args=(i, action))
            threads.append(thread)
            thread.start()

        # 等待所有线程完成
        for thread in threads:
            thread.join()

    def _send_agent_action(self, agent_id, action):
        """发送单个机器人的动作命令"""
        if self.debug:
            rospy.loginfo(f"模拟机器人 {agent_id} 移动...")
            return 
        forward_speed = 0.1
        move_duration = 10

        diff = self.MAE.get_angle_diff_once(agent_id)
        if diff is None:
            turn_speed = 0.1
            turn_duration = 5.3 # 30°
        elif action[agent_id] in ['right', 'left']:
            turn_speed = 0.1
            diff = diff if diff <= 90 else 90
            turn_duration = diff / np.rad2deg(turn_speed) 
            print(f"机器人{agent_id}旋转角度: {diff:.2f}度")
        elif action[agent_id] == 'forward':
            diff = diff if diff <= 2 else 0.5
            forward_speed = 0.1
            move_duration = diff / forward_speed + 2
        

        cmd_msg = Twist()
        if action[agent_id] == 'forward':
            cmd_msg.linear.x = forward_speed
            cmd_msg.angular.z = 0.0
            duration = move_duration
        elif action[agent_id] == 'right':
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = -turn_speed
            duration = turn_duration
        elif action[agent_id] == 'left':
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = turn_speed
            duration = abs(turn_duration)

        print(f"机器人{agent_id}执行动作: {action[agent_id]}, 持续时间: {duration:.2f}秒")
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration and not rospy.is_shutdown():
            self.cmd_publishers[agent_id].publish(cmd_msg)
            rospy.sleep(0.02)  # 短暂休眠，避免CPU占用过高 卡卡的
        
        # 发送停止命令
        stop_cmd = Twist()
        self.cmd_publishers[agent_id].publish(stop_cmd)

        self.cmd_publishers[agent_id].publish(stop_cmd)
        print(f"机器人{agent_id}动作完成")

    def set_global_map_tf(self):
        rospy.loginfo("正在发布静态TF变换以连接机器人坐标系...")
        
        # 1. 创建一个静态TF广播器
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # 2. 准备一个列表来存放所有的静态TF消息
        static_transforms = []
        
        # 3. 循环为每个机器人创建并添加TF变换
        #    这将创建 global_map -> robotX/map 的变换
        for agent_id in self.agent_ids:
            # 创建一个TransformStamped消息
            t = TransformStamped()
            
            # 填充header
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "global_map"  # 公共的父坐标系
            # 子坐标系名称，假设与您的主题命名方式一致 (robot1, robot2, ...)
            # 并且每个机器人的局部地图坐标系叫 'map'
            t.child_frame_id = f"robot{agent_id+1}/map"

            # 填充变换的平移部分，使用您提供的dx, dy
            t.transform.translation.x = self.dx[agent_id]
            t.transform.translation.y = self.dy[agent_id]
            t.transform.translation.z = 0.0  # 假设在2D平面上

            # 填充变换的旋转部分
            # 由于只是初始位置不同，地图朝向相同，所以使用单位四元数（无旋转）
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            # 将创建好的变换添加到列表中
            static_transforms.append(t)
            
            rospy.loginfo(f"准备静态TF: '{t.header.frame_id}' -> '{t.child_frame_id}' "
                          f"at (x={t.transform.translation.x:.2f}, y={t.transform.translation.y:.2f})")

        # 4. 一次性发布所有静态TF变换
        self.static_broadcaster.sendTransform(static_transforms)
        rospy.loginfo("静态TF变换发布完成！TF树已连接。")
        # ===================== 新增代码结束 =====================

    def check_exploration_complete(self):
        """
        检查是否所有机器人都已完成探索。
        如果完成，则记录日志并关闭ROS节点。
        """
        # self.MAE 对象在第一次规划后才会创建，所以要先检查它是否存在
        if not hasattr(self, 'MAE'):
            return False

        # 将已停止的机器人列表和所有机器人的ID列表转换为集合，以便于比较
        stopped_agents_set = set(self.MAE.stop_exploration_flags) | set(self.manually_disabled_agents)
        all_agents_set = set(self.agent_ids)

        # 检查两个集合是否相等。这同时满足了“长度相等”和“完全包含”的条件
        if stopped_agents_set == all_agents_set:
            rospy.loginfo("="*40)
            rospy.loginfo("任务完成: 所有机器人都已停止探索！")
            rospy.loginfo(f"已停止的机器人列表: {sorted(list(stopped_agents_set))}")
            rospy.loginfo("将关闭规划节点...")
            rospy.loginfo("="*40)
            
            # 留出几秒钟让用户看到日志
            # rospy.sleep(3)
            
            # 发出关闭信号，这将导致 rospy.spin() 退出
            rospy.signal_shutdown("所有机器人已完成探索，任务结束。")
            return True # 表示已触发关闭
        
        return False # 表示未满足关闭条件
        
if __name__ == '__main__':
    try:
        planner = MRPlanningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点关闭。")
    except Exception as e:
        rospy.logfatal(f"节点遇到未处理的异常并关闭: {e}")