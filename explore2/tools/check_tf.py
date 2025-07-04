#!/usr/bin/env python3
import rospy
import tf
import math

def yaw_from_quaternion(x, y, z, w):
    """从四元数计算Yaw角(绕Z轴旋转)"""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def tf_listener_node():
    rospy.init_node('tf_yaw_listener_123', anonymous=True)
    
    # 创建TF监听器
    listener = tf.TransformListener()
    
    # 定义目标坐标系和源坐标系
    target_frame = 'robot3/map' # 上级
    source_frame = 'robot3/base_link' # 下级
    
    # 设置循环频率(10Hz)
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            # 获取当前时刻的变换
            now = rospy.Time(0)
            # 等待变换可用
            listener.waitForTransform(target_frame, source_frame, now, rospy.Duration(1.0))
            # 获取变换
            (trans, rot) = listener.lookupTransform(target_frame, source_frame, now)
            
            # 计算Yaw角(弧度)
            yaw_rad = yaw_from_quaternion(rot[0], rot[1], rot[2], rot[3])
            # 转换为角度
            yaw_deg = math.degrees(yaw_rad)
            
            # 打印结果
            rospy.loginfo(f"X:{trans[0]:2f} Y:{trans[1]:2f} Yaw角度: {yaw_deg:.2f}°")
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF异常: {e}")
            rate.sleep()
            continue
            
        rate.sleep()

if __name__ == '__main__':
    try:
        tf_listener_node()
    except rospy.ROSInterruptException:
        pass





