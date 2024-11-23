#include "gamepad_reader.h"

// 构造函数
GamepadReader::GamepadReader() : running(false) {
    buttonStates.fill(0); // 初始化按钮状态为 0
    joystickAxes.fill(0);  // 初始化摇杆数据为 0
}

// 析构函数
GamepadReader::~GamepadReader() {
    stop(); // 确保在析构时停止线程
}

// 启动手柄输入接收线程
void GamepadReader::start() {
    running = true;
    inputThread = std::thread(&GamepadReader::receiveInput, this);
}

// 停止手柄输入接收线程
void GamepadReader::stop() {
    running = false; // 设置标志位以停止程序
    if (inputThread.joinable()) {
        inputThread.join(); // 等待线程结束
    }
}

// 手柄输入接收函数
void GamepadReader::receiveInput() {
    // 初始化 SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0) {
        std::cerr << "无法初始化 SDL: " << SDL_GetError() << std::endl;
        return;
    }

    // 打开手柄
    SDL_GameController* controller = SDL_GameControllerOpen(0); // 0 表示第一个手柄
    if (!controller) {
        std::cerr << "无法打开手柄: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return;
    }

    // 事件循环
    SDL_Event event;

    while (running && start_sys::getInstance().isRunning()) {
        // 处理事件
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                start_sys::getInstance().setRunning(false); // 处理退出事件
            }
            if (event.type == SDL_CONTROLLERBUTTONDOWN) {
                std::lock_guard<std::mutex> lock(dataMutex);
                buttonStates[event.cbutton.button] = 1; // 按钮按下
                std::cout << "按键按下: " << static_cast<int>(event.cbutton.button) << std::endl;
            }
            if (event.type == SDL_CONTROLLERBUTTONUP) {
                std::lock_guard<std::mutex> lock(dataMutex);
                buttonStates[event.cbutton.button] = 0; // 按钮释放
                std::cout << "按键释放: " << static_cast<int>(event.cbutton.button) << std::endl;
            }
            if (event.type == SDL_CONTROLLERAXISMOTION) {
                std::lock_guard<std::mutex> lock(dataMutex);
                if (event.caxis.axis == SDL_CONTROLLER_AXIS_LEFTX || event.caxis.axis == SDL_CONTROLLER_AXIS_RIGHTX) {
                    joystickAxes[0] = event.caxis.value; // 更新左摇杆或右摇杆的 X 轴
                } else if (event.caxis.axis == SDL_CONTROLLER_AXIS_LEFTY || event.caxis.axis == SDL_CONTROLLER_AXIS_RIGHTY) {
                    joystickAxes[1] = event.caxis.value; // 更新左摇杆或右摇杆的 Y 轴
                }
            }
        }

        // 这里可以添加其他处理逻辑
        SDL_Delay(100); // 每 100 毫秒检查一次
    }

    // 关闭手柄和退出 SDL
    SDL_GameControllerClose(controller);
    SDL_Quit();
}

// 获取按钮状态
const std::array<int, SDL_CONTROLLER_BUTTON_MAX>& GamepadReader::getButtonStates() const {
    std::lock_guard<std::mutex> lock(dataMutex); // 保护 buttonStates
    return buttonStates;
}

// 获取摇杆数据
const std::array<int, 2>& GamepadReader::getJoystickAxes() const {
    std::lock_guard<std::mutex> lock(dataMutex); // 保护 joystickAxes
    return joystickAxes;
}
