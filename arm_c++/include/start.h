#ifndef START_H
#define START_H

#include <atomic>

class start_sys {
public:
    // 获取单例实例
    static start_sys& getInstance() {
        static start_sys instance; // 局部静态变量，确保只创建一次
        return instance;
    }

    // 禁止拷贝构造和赋值操作
    start_sys(const start_sys&) = delete;
    start_sys& operator=(const start_sys&) = delete;

    // 访问运行状态
    bool isRunning() const {
        return running.load();
    }

    void setRunning(bool value) {
        running.store(value);
    }

private:
    start_sys() : running(true) {} // 私有构造函数

    std::atomic<bool> running; // 使用 atomic 以确保线程安全
};

#endif // START_H
