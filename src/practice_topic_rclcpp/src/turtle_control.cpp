#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleConctroller : public rclcpp::Node{
public:
    // 构造函数
    TurtleConctroller():Node("turtle_controller"){      // 这里没有外部传入node名，而是内部写好
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleConctroller::on_pose_received_, this, std::placeholders::_1));
    }
private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;           // 定义接受者变量 智能指针
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;        // 定义速度发布者变量
    double target_x_{1.0};                                                              // 目标点坐标
    double target_y_{1.0};                                                              // 目标点坐标
    double k_{1.0};
    double max_speed{3.0};
private:
    double calculate_dis(const double &x, const double &y){
        return std::sqrt((target_x_ - x)*(target_x_ - x) + (target_y_ - y)*(target_y_ - y));
    }
    
    double calculate_angle(const double &x, const double &y, const double &theta){
        return std::atan2(target_y_ - y, target_x_ - x) - theta;
    }

    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose){
        // TODO: 收到误差值，发布速度指令
        auto msg = geometry_msgs::msg::Twist();
        double current_x = pose->x;
        double current_y = pose->y;
        double current_theta = pose->theta;
        RCLCPP_INFO(this->get_logger(), "当前位置为：(x = %f y = %f)", current_x, current_y);

        double err_dis = calculate_dis(current_x, current_y);
        double err_angle = calculate_angle(current_x, current_y, current_theta);

        if (err_dis > 0.1)
        {
            if(fabs(err_angle) > 0.2){
                msg.angular.z = fabs(err_angle);
            }else{
                msg.linear.x = k_*err_dis;
            }
        }
        
        if(msg.linear.x > max_speed){
            msg.linear.x = max_speed;
        }

        velocity_publisher_->publish(msg);
    }
};

int main(int argc, char const *argv[])
{
    /* code */
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleConctroller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
