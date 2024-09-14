#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>                           // 时间相关头文件
using namespace std::chrono_literals;

class TurtleCircle : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    TurtleCircle(std::string name) : Node(name)
    {
        // 创建发布者
        command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleCircle::timer_callback, this));
    }

private:
    // 声明话题发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 创建定时器回调函数
    void timer_callback()
    {
        // 创建消息
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 1.0;
        msg.angular.z = 0.5;
        command_publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TurtleCircle>("Turtle_circle_node");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
