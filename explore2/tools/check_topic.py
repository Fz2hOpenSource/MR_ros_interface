#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import sys

class CompressedImageViewer:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('compressed_image_viewer', anonymous=True)
        
        # 设置图像窗口名称
        self.window_name = "Compressed Image Viewer"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        # 订阅压缩图像话题
        self.image_sub = rospy.Subscriber(
            "/robot3/camera/image_raw/compressed",
            CompressedImage,
            self.callback,
            queue_size=1
        )
        
        # 设置关闭窗口的回调函数
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_AUTOSIZE)
        
        # 注册关闭节点的钩子
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("压缩图像查看器已启动，订阅话题: /robot3/camera/image_raw/compressed")
        
    def callback(self, msg):
        try:
            # 将压缩图像数据转换为numpy数组
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # 解码图像
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # 显示图像
            cv2.imshow(self.window_name, image)
            
            # 按ESC键退出
            if cv2.waitKey(1) == 27:
                rospy.signal_shutdown("用户按下ESC键退出")
                
        except Exception as e:
            rospy.logerr("处理图像时出错: %s", str(e))
    
    def shutdown(self):
        rospy.loginfo("关闭压缩图像查看器...")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        viewer = CompressedImageViewer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
