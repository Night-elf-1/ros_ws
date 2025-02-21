#ifndef SPIN_MOTION_CONTROLLER_HPP
#define SPIN_MOTION_CONTROLLER_HPP

#include "motion_control_system/motion_control_interface.hpp"

namespace motion_control_system
{
    class SpinMotionController : public MotionController        // 继承了抽象类
    {
    public:
        void start() override;          // 使用override关键词覆盖基类中的纯虚函数
        void stop() override;
    };

} // namespace motion_control_system

#endif // SPIN_MOTION_CONTROLLER_HPP