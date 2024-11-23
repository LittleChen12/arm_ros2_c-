#include "controler.h"

MotorController::MotorController() : running(false) {}

MotorController::~MotorController() {
    stop();
}

void MotorController::start() {
    running = true;
    controlThread = std::thread(&MotorController::controlMotors, this);
}

void MotorController::stop() {
    running = false;
    if (controlThread.joinable()) {
        controlThread.join();
    }
}

void MotorController::controlMotors() {
    RS485Reader& rs485Reader = RS485Reader::getInstance(); // 获取单例实例
    SerialReader& serialReader = SerialReader::getInstance(); // 获取 SerialReader 单例

    while (running&&start_sys::getInstance().isRunning()) {




        auto jointAngles = serialReader.getJointAngles(); // 获取关节角度数据

        for (const auto& joint : jointAngles) {
            uint8_t jointFlag = joint.first; // 获取关节标志
            float jointAngle = joint.second; // 获取关节角度（弧度）

            // // 打印控制的角度（弧度）
            // std::cout << "关节：" << static_cast<int>(jointFlag)
            //           << ", Angle (radians): " << jointAngle * (180.0/M_PI) << std::endl;

            // 将角度（弧度）转换为脉冲
            int32_t pulseCount = ((jointAngle * 3200.0 * 16 * 50) / (2.0 * M_PI)); // 转换为脉冲

            // 打印控制的角度（弧度）
            std::cout << "关节：" << static_cast<int>(jointFlag)
                      << ", Angle (radians): " << pulseCount << std::endl;

            // 发送控制命令
            rs485Reader.sendMotorControlCommand(jointFlag, FUNCTION_CODE, SPEED, ACCELERATION, pulseCount);
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 10毫秒周期
        }

    }
}
