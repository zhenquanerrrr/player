#pragma once
#include <cstdint>
#include "app.h"

namespace desk {

// 贪吃蛇小游戏：逻辑与渲染分离，便于移植到任意屏幕。
class SnakeGame : public App {
public:
    explicit SnakeGame(Display& disp);
    const char* name() const override { return "snake"; }
    void onEnter() override;
    void onExit() override;
    bool loop(Input& input, Display& disp, uint32_t nowMs) override;

private:
    void reset();
    void tick();
    void spawnFood();
    void draw(Display& disp) const;

    static constexpr int CELL = 8;
    static constexpr int MAXLEN = 256;
    static constexpr uint32_t STEP_MS = 180;

    Display* disp_;
    int gw_ = 0, gh_ = 0;
    int sx_[MAXLEN], sy_[MAXLEN];
    int len_ = 0;
    int dx_ = 1, dy_ = 0;
    int fx_ = 0, fy_ = 0;
    int score_ = 0;
    bool dead_ = false;
    uint32_t lastStep_ = 0;
};

} // namespace desk
