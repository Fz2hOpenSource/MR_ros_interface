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

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_planning_request");
    ros::NodeHandle nh;
    
    // 创建发布者，发布到/request_planning话题
    ros::Publisher request_pub = nh.advertise<std_msgs::Bool>("/request_planning", 10);
    
    ros::Rate rate(10); // 10Hz
    
    std_msgs::Bool request_msg;
    request_msg.data = false;
    
    std::cout << "按下 'Y' 键发送规划请求，按下 'q' 键退出..." << std::endl;
    
    while (ros::ok()) {
        // 检查键盘输入
        if (kbhit()) {
            char c = getch();
            
            if (c == 'Y' || c == 'y') {
                request_msg.data = true;
                request_pub.publish(request_msg);
                std::cout << "已发送规划请求！" << std::endl;
                request_msg.data = false; // 发送后重置为false
            } else if (c == 'q' || c == 'Q') {
                std::cout << "退出程序..." << std::endl;
                break;
            }
        }
        
        // 发布当前状态
        if(request_msg.data){
            request_pub.publish(request_msg);
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