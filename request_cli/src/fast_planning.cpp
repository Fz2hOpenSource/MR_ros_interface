#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <termios.h>
#include <iostream>
#include <fcntl.h>  // 添加这个头文件以支持fcntl和相关标志

// 函数声明
int kbhit(void);  // 在main函数前声明kbhit函数

// 用于获取键盘输入的函数
int getch() {
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // 保存当前设置
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);         // 设置为非规范模式和不回显
    tcsetattr( STDIN_FILENO, TCSANOW, &newt); // 立即应用新设置

    int ch = getchar();

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt); // 恢复旧设置
    return ch;
}

// 回调函数：处理/wait_request话题消息
void waitRequestCallback(const std_msgs::Bool::ConstPtr& msg, 
                         ros::Publisher* pub, 
                         std_msgs::Bool* request_msg) {
    if (msg->data) { // 如果接收到的消息为true
        std::cout << "Roger that wait for 3 seconds..." << std::endl;

        ros::Duration(3.0).sleep(); // 休眠3秒
        
        request_msg->data = true;
        pub->publish(*request_msg);
        std::cout << "Send Request!" << std::endl;
        
        // 发送后重置为false
        request_msg->data = false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_planning_request");
    ros::NodeHandle nh;
    
    // 创建发布者，发布到/request_planning话题
    ros::Publisher request_pub = nh.advertise<std_msgs::Bool>("/request_planning", 10);
    
    // 创建订阅者，监听/wait_request话题
    std_msgs::Bool request_msg;
    ros::Subscriber wait_sub = nh.subscribe<std_msgs::Bool>(
        "/wait_request", 
        5, 
        boost::bind(waitRequestCallback, _1, &request_pub, &request_msg)
    );
    return 0;
}

