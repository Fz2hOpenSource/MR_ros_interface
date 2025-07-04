#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('pose_normalizer', anonymous=True)

    # 从参数服务器获取TF前缀和全局frame
    tf_prefix = rospy.get_param('~tf_prefix', '')
    global_frame = rospy.get_param('~global_frame', 'map')
    
    # 构建带前缀的frame_id
    if tf_prefix and tf_prefix[-1] != '/':
        tf_prefix += '/'

    base_frame = tf_prefix + 'base_link'
    # 定义我们新创建的、归一化的世界坐标系
    normalized_world_frame = tf_prefix + 'world_start'
    
    rospy.loginfo(f"Pose normalizer for '{tf_prefix}' started.")
    rospy.loginfo(f"Will create a static transform from '{global_frame}' to '{normalized_world_frame}'")
    rospy.loginfo(f"This transform represents the initial pose of '{base_frame}' in '{global_frame}'.")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 循环等待，直到成功获取到机器人相对于全局地图的初始变换
    initial_transform = None
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown() and initial_transform is None:
        try:
            # 查询从全局地图到机器人基座的变换
            initial_transform = tf_buffer.lookup_transform(
                global_frame, 
                base_frame, 
                rospy.Time(0), 
                rospy.Duration(2.0) # 等待2秒
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"Waiting for initial transform '{global_frame}' -> '{base_frame}': {e}")
            rate.sleep()
            continue

    rospy.loginfo("Initial pose acquired. Publishing static transform.")

    # 创建一个静态变换发布器
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    # 这个静态变换就是机器人的初始位姿
    static_transform_stamped = geometry_msgs.msg.TransformStamped()
    
    static_transform_stamped.header.stamp = rospy.Time.now()
    # 这个变换定义了 'world_start' 在 'map' 中的位置
    static_transform_stamped.header.frame_id = global_frame
    static_transform_stamped.child_frame_id = normalized_world_frame
    
    # 直接使用获取到的初始变换
    static_transform_stamped.transform = initial_transform.transform

    # 发布这个静态变换
    broadcaster.sendTransform(static_transform_stamped)

    rospy.loginfo(f"Static transform from '{global_frame}' to '{normalized_world_frame}' is now being published. Normalizer task complete.")
    
    # 节点完成任务后，保持运行以维持静态TF的发布
    rospy.spin()