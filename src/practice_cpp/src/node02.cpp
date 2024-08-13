
#include "rclcpp/rclcpp.hpp"

/*
    创建一个类节点，名字叫做Node03,继承自Node.
*/
class Node03 : public rclcpp::Node  // 继承自 rclcpp::Node 类，Node03 类将继承 rclcpp::Node 类的所有公有和保护成员
{

public:
    // 构造函数,有一个参数为节点名称
    Node03(std::string name) : Node(name)   // Node03 类的构造函数，它接受一个 std::string 类型的参数，名为 name
    {
        // 打印一句
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.",name.c_str());
    }

private:
   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个node_03的节点*/
    auto node = std::make_shared<Node03>("node_02");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
