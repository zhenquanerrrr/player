#pragma once
#include <cstdint>
#include "input.h"
#include "display.h"

namespace desk {

// 应用统一接口：六个菜单功能各自实现一个 App。
class App {
public:
    virtual ~App() = default;
    virtual const char* name() const = 0;
    virtual void onEnter() = 0;
    virtual void onExit() = 0;
    // 每帧调用。nowMs 为运行毫秒数。返回 true 表示请求退出本应用（返回菜单）。
    virtual bool loop(Input& input, Display& disp, uint32_t nowMs) = 0;
};

} // namespace desk
