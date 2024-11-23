#include "serial_reader.h"

const size_t JOINT_DATA_SIZE = 5; // 每个关节数据的字节数
const size_t NUM_JOINTS = 6; // 关节数量
bool data_ready = false; // 标志位，表示是否有新数据
// 信号处理函数
void SerialReader::signalHandler(int signum) {
    // 处理信号
    data_ready = true;
}

// 串口配置函数
int SerialReader::configureSerialPort(const char* portName) {
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "无法打开串口" << std::endl;
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; // 设置波特率、数据位
    options.c_iflag = IGNPAR; // 忽略帧错误
    options.c_oflag = 0; // 原样输出
    options.c_lflag = 0; // 不使用本地模式
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    // 设置异步模式
    fcntl(fd, F_SETFL, FASYNC);
    fcntl(fd, F_SETOWN, getpid()); // 设置进程为接收者
    signal(SIGIO, signalHandler); // 注册信号处理函数

    return fd;
}

// 数据解析函数
void SerialReader::parseJointData(const uint8_t* buffer) {
    std::vector<std::pair<uint8_t, float>> angles;

    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        size_t startIndex = i * JOINT_DATA_SIZE;
        uint8_t jointFlag = buffer[startIndex];

        // 将4个字节转换为浮点数
        float jointAngle;
        std::memcpy(&jointAngle, &buffer[startIndex + 1], sizeof(float));

        angles.emplace_back(jointFlag, jointAngle);
    }

    // 使用互斥量保护 jointAngles
    std::lock_guard<std::mutex> lock(dataMutex);
    jointAngles = angles;
}

// 数据接收线程函数
void SerialReader::receiveData() {
    uint8_t receiveBuffer[JOINT_DATA_SIZE * NUM_JOINTS];
    while (running&&start_sys::getInstance().isRunning()) {
        if(data_ready)
        {
            data_ready = false; // 重置标志位
            ssize_t bytesRead = read(serialPort, receiveBuffer, sizeof(receiveBuffer));
            if (bytesRead == sizeof(receiveBuffer)) {
                parseJointData(receiveBuffer);
            }            
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 2毫秒
    }
}

// 构造函数
SerialReader::SerialReader(const char* portName) : portName(portName), running(false) {
    serialPort = configureSerialPort(portName);
}

// 析构函数
SerialReader::~SerialReader() {
    stop(); // 确保在析构时停止线程
}

// 启动接收线程
void SerialReader::start() {
    running = true;
    if (serialPort != -1) {
        receiveThread = std::thread(&SerialReader::receiveData, this);
    }
}

// 停止接收线程
void SerialReader::stop() {
    running = false; // 设置标志位以停止程序
    if (receiveThread.joinable()) {
        receiveThread.join(); // 等待线程结束
    }
    close(serialPort); // 关闭串口
}

// 获取关节角度数据
std::vector<std::pair<uint8_t, float>> SerialReader::getJointAngles() {
    std::lock_guard<std::mutex> lock(dataMutex); // 保护 jointAngles
    return jointAngles;
}
