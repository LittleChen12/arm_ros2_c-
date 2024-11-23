#include <iostream>
#include <csignal>

#include "serial_reader.h" // 引入串口读取的头文件
#include "gamepad_reader.h"// 手柄的库文件
#include "start.h"
#include "RS485Reader.h"
#include "controler.h"

// 信号处理函数
void signalHandler(int signum) {
    std::cout << "接收到信号: " << signum << ", 正在退出..." << std::endl;
    start_sys::getInstance().setRunning(false); // 设置运行状态为 false
}

int main() {
    // 注册信号处理
    std::signal(SIGINT, signalHandler);
    
    // 获取单例实例 
    start_sys& sys = start_sys::getInstance();

    // 创建ttl/rs485串口线程
    const char* portName = "/dev/ttyUSB0"; // 根据实际情况修改串口名称
    SerialReader& serialReader = SerialReader::getInstance(portName); // 获取 SerialReader 单例
    serialReader.start(); // 启动数据接收

    RS485Reader& rs485Reader = RS485Reader::getInstance("/dev/ttyACM0"); // 根据实际情况修改串口名称
    rs485Reader.start(); // 启动接收线程

    // 创建手柄输入接收线程
    GamepadReader& gamepadReader = GamepadReader::getInstance(); // 获取 GamepadReader 单例
    gamepadReader.start(); // 启动手柄输入接收

    // 创建 MotorController 实例并启动控制线程
    MotorController motorController;
    motorController.start();  


    // 主线程可以进行其他初始化或处理
    std::cout << "串口读取线程已启动..." << std::endl;

    while(sys.isRunning())
    {
        auto buttonStates = gamepadReader.getButtonStates(); // 获取按钮状态
        auto joystickAxes = gamepadReader.getJoystickAxes(); // 获取摇杆数据

        // // 输出按钮状态
        // for (size_t i = 0; i < buttonStates.size(); ++i) {
        //     if (buttonStates[i] == 1) { // 如果按钮被按下
        //         std::cout << "按钮 " << i << " 被按下" << std::endl;
        //     }
        // }

        // // 输出摇杆数据
        // std::cout << "左摇杆 X: " << joystickAxes[0] << ", Y: " << joystickAxes[1] << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 每秒检查一次
    }
    // 等待手柄线程结束
    gamepadReader.stop(); // 停止手柄输入接收

    // 等待串口线程结束
    serialReader.stop(); // 停止数据接收
    rs485Reader.stop();

    //等待控制线程结束
    motorController.stop();

    return 0;
}
