#pragma once
#include <cstdint>

namespace desk {

// 抽象按键：后续接物理按键或触摸。
enum class Key : uint8_t {
    None = 0,
    Up,
    Down,
    Left,
    Right,
    Confirm,
    Back,
};

class Input {
public:
    virtual ~Input() = default;
    // 返回当前按键；无按键返回 Key::None。实现应做去抖。
    virtual Key readKey() = 0;
};

} // namespace desk
