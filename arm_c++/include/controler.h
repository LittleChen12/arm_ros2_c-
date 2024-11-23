#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "RS485Reader.h"
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include "start.h"
#include "serial_reader.h"
#include <math.h>

// 宏定义
#define FUNCTION_CODE 0xFE // 功能码
#define SPEED 3000          // 速度
#define ACCELERATION 0      // 加速度

class MotorController {
public:
    MotorController();
    ~MotorController(); // 添加析构函数
    void start();
    void stop();

private:
    void controlMotors();

    std::thread controlThread;
    std::atomic<bool> running; // 使用 atomic 变量来安全地控制线程
};

#endif
