#ifndef RS485_READER_H
#define RS485_READER_H

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include "start.h"

class RS485Reader {
public:
    static RS485Reader& getInstance(const char* portName = nullptr) {
        static RS485Reader instance(portName);
        return instance;
    }

    RS485Reader(const RS485Reader&) = delete;
    RS485Reader& operator=(const RS485Reader&) = delete;

    void start();
    void stop();
    void sendData(const std::vector<uint8_t>& data);
    void sendMotorControlCommand(uint8_t slaveAddress, uint8_t functionCode, uint16_t speed, uint8_t acceleration, int32_t pulseCount);
    std::vector<std::pair<uint8_t, float>> getJointAngles();
    uint8_t getCurrentControlStatus() const; // 新增方法获取当前控制状态

private:
    RS485Reader(const char* portName);
    ~RS485Reader();

    void configureSerialPort(const char* portName);
    void receiveData();
    void parseJointData(const uint8_t* buffer, int length);
    uint8_t calculateCRC(const uint8_t* data, size_t length);

    const char* portName;
    int serialPort;
    std::atomic<bool> running;
    std::thread receiveThread;
    std::vector<std::pair<uint8_t, float>> jointAngles;
    std::mutex dataMutex;
    uint8_t currentControlStatus; // 新增成员变量保存当前控制状态
};

#endif // RS485_READER_H
