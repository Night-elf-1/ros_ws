#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class Node04(Node): # 继承自 Node 类, Node04 类将继承 Node 类的所有属性和方法
    """
    创建一个Node04节点，并在初始化时输出一个话
    """
    def __init__(self,name):    # Node04 类的初始化方法（构造函数），当你创建 Node04 类的实例时，会自动调用这个方法
        super().__init__(name)  # super() 函数调用父类 Node 的初始化方法，并传递参数 name
        self.get_logger().info("大家好，我是%s! --> from practice_py" % name)


def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = Node04("node_02")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
