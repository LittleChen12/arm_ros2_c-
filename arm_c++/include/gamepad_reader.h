#ifndef GAMEPAD_READER_H
#define GAMEPAD_READER_H

#include <iostream>
#include <SDL2/SDL.h>
#include <thread>
#include <atomic>
#include <array>
#include <mutex>
#include "start.h"

class GamepadReader {
public:
    // 获取单例实例
    static GamepadReader& getInstance() {
        static GamepadReader instance; // 局部静态变量，确保只创建一次
        return instance;
    }

    // 禁止拷贝构造和赋值操作
    GamepadReader(const GamepadReader&) = delete;
    GamepadReader& operator=(const GamepadReader&) = delete;

    void start();
    void stop();
    const std::array<int, SDL_CONTROLLER_BUTTON_MAX>& getButtonStates() const; // 获取按钮状态
    const std::array<int, 2>& getJoystickAxes() const; // 获取摇杆数据

private:
    GamepadReader(); // 私有构造函数
    ~GamepadReader();

    void receiveInput(); // 手柄输入接收函数

    std::atomic<bool> running; // 控制线程运行状态
    std::thread inputThread; // 输入接收线程
    std::array<int, SDL_CONTROLLER_BUTTON_MAX> buttonStates; // 存储按钮状态
    std::array<int, 2> joystickAxes; // 存储摇杆数据
    mutable std::mutex dataMutex; // 保护数据的互斥量
};

#endif // GAMEPAD_READER_H
