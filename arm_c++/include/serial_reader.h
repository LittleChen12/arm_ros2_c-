#ifndef SERIAL_READER_H
#define SERIAL_READER_H

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>
#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include "start.h"
#include <mutex>

class SerialReader {
public:
    // 获取单例实例
    static SerialReader& getInstance(const char* portName = nullptr) {
        static SerialReader instance(portName); // 局部静态变量，确保只创建一次
        return instance;
    }

    // 禁止拷贝构造和赋值操作
    SerialReader(const SerialReader&) = delete;
    SerialReader& operator=(const SerialReader&) = delete;

    void start();
    void stop();
    std::vector<std::pair<uint8_t, float>> getJointAngles();

private:
    SerialReader(const char* portName); // 私有构造函数
    ~SerialReader();
    
    static void signalHandler(int signum);
    int configureSerialPort(const char* portName);
    void parseJointData(const uint8_t* buffer);
    void receiveData();

    const char* portName;
    int serialPort;
    std::atomic<bool> running; // 控制线程运行状态
    std::thread receiveThread;
    std::vector<std::pair<uint8_t, float>> jointAngles; // 存储关节角度数据
    std::mutex dataMutex; // 保护 jointAngles 的互斥量
};

#endif // SERIAL_READER_H
