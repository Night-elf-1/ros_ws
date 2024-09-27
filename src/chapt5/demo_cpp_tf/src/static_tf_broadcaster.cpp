#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"  // 提供消息接口
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"  // 提供 tf2::Quaternion 类
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // 提供消息类型转换函数
#include "tf2_ros/static_transform_broadcaster.h"  // 提供静态坐标广播器类

class StaticTFBroadcaster : public rclcpp::Node {
 public:
  StaticTFBroadcaster() : Node("tf_broadcaster_node") {
    // 创建静态广播发发布器并发布
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->publish_tf();
  }

  void publish_tf() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();              // 获取时间戳
    transform.header.frame_id = "map";                              // 设置父坐标系
    transform.child_frame_id = "target_point";                      // 设置子坐标系
    transform.transform.translation.x = 5.0;                        // 设置偏移量
    transform.transform.translation.y = 3.0;
    transform.transform.translation.z = 0.0;
    tf2::Quaternion quat;                                           // 创建四元数转换
    quat.setRPY(0, 0, 60 * M_PI / 180);                             // 弧度制欧拉角转四元数 60° --> 弧度
    transform.transform.rotation = tf2::toMsg(quat);                // 转成消息接口类型 这里rotation接收的是四元数格式 x y z w
    broadcaster_->sendTransform(transform);                         // 发送转换关系出去
  }

 private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;      // 声明静态转换发布者 共享指针
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticTFBroadcaster>();              // 创建节点
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}