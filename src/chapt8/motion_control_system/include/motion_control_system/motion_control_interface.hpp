#ifndef MOTION_CONTROL_INTERFACE_HPP
#define MOTION_CONTROL_INTERFACE_HPP

// 在motion_control_system命名空间下定义了抽象类MotionController
namespace motion_control_system {

class MotionController {
public:
    virtual void start() = 0;       // 在C++中virtual关键词定义虚函数，而虚函数 = 0 代表纯虚函数 在类中纯在一个或以上的纯虚函数，则类变成抽象类
    virtual void stop() = 0;        // 抽象类只能被继承 不能被实例化
    virtual ~MotionController() {}
};

} // namespace motion_control_system

#endif // MOTION_CONTROL_INTERFACE_HPP