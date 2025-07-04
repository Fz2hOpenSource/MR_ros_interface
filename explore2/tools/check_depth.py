import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import os
from datetime import datetime

class DepthProcessor:
    def __init__(self, output_dir='/home/ling/Pictures/depth_test'):
        """
        初始化节点，设置订阅者和输出目录。

        Args:
            output_dir (str): 保存深度NPY文件的目录路径。
        """
        rospy.init_node('depth_saver_node')
        
        self.subscriber = rospy.Subscriber(
            '/robot2/camera/depth/image_raw/compressedDepth', 
            CompressedImage, 
            self.callback,
            queue_size=1,
            buff_size=2**24
        )
        
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        rospy.loginfo(f"深度图像将保存到: {self.output_dir}")
        
        # 新增：帧计数器
        self.frame_counter = 0
        # 新增：保存频率
        self.save_frequency = 30
        
    def callback(self, msg):
        """
        接收到消息后的回调函数。
        每30帧解码并保存一次深度图像为.npy文件。
        """
        try:
            # 帧计数
            self.frame_counter += 1
            
            # 只处理每save_frequency帧
            if self.frame_counter % self.save_frequency != 0:
                return
            
            depth_array = self.decode_compressed_depth(msg)
            
            if depth_array is None:
                rospy.logwarn("解码失败，无法获取深度数组。")
                return

            valid_depth = depth_array[depth_array > 0]
            if valid_depth.size > 0:
                rospy.loginfo(f"接收到第{self.frame_counter}帧深度数据 | 尺寸: {depth_array.shape} | 类型: {depth_array.dtype} | 范围: {valid_depth.min()}-{valid_depth.max()} mm")
            else:
                rospy.logwarn("警告: 帧内没有检测到有效的深度值。")
                return

            timestamp = msg.header.stamp.to_sec()
            dt_object = datetime.fromtimestamp(timestamp)
            
            filename = os.path.join(
                self.output_dir,
                f"depth_{dt_object.strftime('%Y%m%d_%H%M%S')}_{msg.header.stamp.nsecs:09d}_{msg.header.seq}.npy"
            )
            
            np.save(filename, depth_array)
            rospy.loginfo(f"成功保存深度数组: {os.path.basename(filename)}")
                
        except Exception as e:
            rospy.logerr(f"在回调函数中发生异常: {e}", exc_info=True)
    
    def decode_compressed_depth(self, compressed_msg):
        """
        使用特定方法解析16-bit的压缩深度图像消息。
        """
        if 'compressedDepth' not in compressed_msg.format:
            raise ValueError(f"消息格式 '{compressed_msg.format}' 不是 'compressedDepth'。")
            
        np_arr = np.frombuffer(compressed_msg.data, np.uint8)
        depth_data_raw = np_arr[12:]
        depth_img = cv2.imdecode(depth_data_raw, cv2.IMREAD_UNCHANGED)
        
        return depth_img

if __name__ == '__main__':
    try:
        output_directory = '/home/ling/Pictures/depth_test_mr600_night'

        processor = DepthProcessor(output_dir=output_directory)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROS节点关闭。")
    except Exception as e:
        rospy.logfatal(f"节点启动失败: {e}")