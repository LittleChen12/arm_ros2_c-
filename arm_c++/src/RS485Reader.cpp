#include "RS485Reader.h"

RS485Reader::RS485Reader(const char* portName) : portName(portName), running(false) {
    configureSerialPort(portName);
}

RS485Reader::~RS485Reader() {
    stop();
    if (serialPort != -1) {
        close(serialPort);
    }
}

void RS485Reader::configureSerialPort(const char* portName) {
    serialPort = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialPort == -1) {
        std::cerr << "无法打开串口: " << portName << std::endl;
        return;
    }

    struct termios options;
    tcgetattr(serialPort, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(serialPort, TCIFLUSH);
    tcsetattr(serialPort, TCSANOW, &options);
}

void RS485Reader::start() {
    running = true;
    receiveThread = std::thread(&RS485Reader::receiveData, this);
}

void RS485Reader::stop() {
    running = false;
    if (receiveThread.joinable()) {
        receiveThread.join();
    }
}

void RS485Reader::sendData(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(dataMutex);
    std::vector<uint8_t> dataWithCRC = data;
    uint8_t crc = calculateCRC(data.data(), data.size());
    dataWithCRC.push_back(crc); // 添加CRC校验位
    write(serialPort, dataWithCRC.data(), dataWithCRC.size());
}

void RS485Reader::sendMotorControlCommand(uint8_t slaveAddress, uint8_t functionCode, uint16_t speed, uint8_t acceleration, int32_t pulseCount) {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    std::vector<uint8_t> command;
    command.push_back(0xFA); // 帧头
    command.push_back(slaveAddress); // 从机地址
    command.push_back(functionCode); // 功能码

    // 添加速度（2字节）
    command.push_back(static_cast<uint8_t>((speed >> 8) & 0xFF)); // 高字节
    command.push_back(static_cast<uint8_t>(speed & 0xFF)); // 低字节

    // 添加加速度（1字节）
    command.push_back(acceleration);

    // 添加绝对脉冲数（4字节）
    command.push_back(static_cast<uint8_t>((pulseCount >> 24) & 0xFF)); // 高字节
    command.push_back(static_cast<uint8_t>((pulseCount >> 16) & 0xFF));
    command.push_back(static_cast<uint8_t>((pulseCount >> 8) & 0xFF));
    command.push_back(static_cast<uint8_t>(pulseCount & 0xFF)); // 低字节
    // 计算并添加CRC
    uint8_t crc = calculateCRC(command.data(), command.size());
    command.push_back(crc); // 添加CRC校验位

    // 发送命令
    write(serialPort, command.data(), command.size());
}

void RS485Reader::receiveData() {
    while (running&&start_sys::getInstance().isRunning()) {
        uint8_t buffer[256];
        int bytesRead = read(serialPort, buffer, sizeof(buffer));
        if (bytesRead > 0) {
            parseJointData(buffer, bytesRead);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void RS485Reader::parseJointData(const uint8_t* buffer, int length) {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    if (length < 5) return; // 至少需要5个字节

    // 检查帧头
    if (buffer[0] != 0xFB) {
        std::cerr << "无效的帧头，丢弃数据" << std::endl;
        return;
    }

    uint8_t slaveAddress = buffer[1]; // 电机通信地址
    uint8_t functionCode = buffer[2]; // 功能码
    uint8_t controlStatus = buffer[3]; // 控制状态
    uint8_t receivedCRC = buffer[4]; // CRC校验位

    // 计算CRC校验值
    uint8_t calculatedCRC = calculateCRC(buffer, length - 1); // 不包括CRC本身

    // 验证CRC校验值
    if (receivedCRC == calculatedCRC) {
        // 存储当前控制状态
        currentControlStatus = controlStatus;
        // 输出电机通信地址和控制状态
        std::cout << "Received data from motor with address: " << static_cast<int>(slaveAddress)
                  << ", Control status: " << static_cast<int>(controlStatus) << std::endl;        
    } else {
        std::cerr << "CRC校验失败，丢弃数据" << std::endl;
    }
}

uint8_t RS485Reader::calculateCRC(const uint8_t* data, size_t length) {
    uint16_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF); // 取低8位
}

std::vector<std::pair<uint8_t, float>> RS485Reader::getJointAngles() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return jointAngles;
}

uint8_t RS485Reader::getCurrentControlStatus() const {
    return currentControlStatus; // 返回当前控制状态
}
