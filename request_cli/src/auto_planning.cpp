#include <ros/ros.h>
#include <std_msgs/Float32.h>  
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

// 回调函数：处理/wait_request话题消息，现在消息类型是Float32
void waitRequestCallback(const std_msgs::Float32::ConstPtr& msg, 
                         ros::Publisher* pub, 
                         std_msgs::Float32* request_msg) {
    if (msg->data > 0) { // 如果接收到的等待时间大于0
        std::cout << "Roger that, waiting for " << msg->data << " seconds..." << std::endl;

        ros::Duration(msg->data).sleep(); // 休眠指定的秒数
        
        request_msg->data = 1.0; // 发送请求信号
        pub->publish(*request_msg);
        std::cout << "Send Request!" << std::endl;
        
        // 发送后重置为0
        request_msg->data = 0.0;
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_planning_request");
    ros::NodeHandle nh;
    
    // 创建发布者，发布到/request_planning话题
    ros::Publisher request_pub = nh.advertise<std_msgs::Float32>("/request_planning", 1);
    
    // 创建订阅者，监听/wait_request话题
    std_msgs::Float32 request_msg;
    request_msg.data = 1.0; // 初始值为0，表示不请求
    ros::Subscriber wait_sub = nh.subscribe<std_msgs::Float32>(
        "/wait_request", 
        5, 
        boost::bind(waitRequestCallback, _1, &request_pub, &request_msg)
    );
    
    ros::Rate rate(10); // 10Hz
    
    std::cout << "按下 'Y' 键发送规划请求至 /request_planning ，按下 'q' 键退出..." << std::endl;
    std::cout << "也可以通过/wait_request话题触发自动规划请求，消息值为等待秒数..." << std::endl;
    
    while (ros::ok()) {
        // 检查键盘输入
        if (kbhit()) {
            char c = getch();
            
            if (c == 'Y' || c == 'y') {
                request_msg.data = 1.0; // 发送请求信号
                request_pub.publish(request_msg);
                std::cout << "已发送规划请求！" << std::endl;
                request_msg.data = 1.0; // 发送后重置为0
            } else if (c == 'q' || c == 'Q') {
                std::cout << "退出程序..." << std::endl;
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}

// 用于检查是否有键盘输入的函数
int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}    
