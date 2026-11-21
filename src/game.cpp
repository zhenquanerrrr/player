#include "game.h"
#include <cstdio>
#include <cstdlib>

namespace desk {

static constexpr Color16 C_BG    = 0x0000;
static constexpr Color16 C_SNAKE = 0x07E0; // 绿
static constexpr Color16 C_HEAD  = 0x07FF; // 青
static constexpr Color16 C_FOOD  = 0xF800; // 红
static constexpr Color16 C_TEXT  = 0xFFFF;

SnakeGame::SnakeGame(Display& disp) : disp_(&disp) {}

void SnakeGame::onEnter() { reset(); }
void SnakeGame::onExit()  {}

void SnakeGame::reset() {
    gw_ = disp_->width() / CELL;
    gh_ = disp_->height() / CELL;
    if (gw_ < 8) gw_ = 8;
    if (gh_ < 8) gh_ = 8;
    len_ = 3;
    int cx = gw_ / 2, cy = gh_ / 2;
    for (int i = 0; i < len_; ++i) { sx_[i] = cx - i; sy_[i] = cy; }
    dx_ = 1; dy_ = 0;
    score_ = 0;
    dead_ = false;
    spawnFood();
    lastStep_ = 0;
}

void SnakeGame::spawnFood() {
    for (int tries = 0; tries < 512; ++tries) {
        int x = rand() % gw_;
        int y = rand() % gh_;
        bool ok = true;
        for (int i = 0; i < len_; ++i) {
            if (sx_[i] == x && sy_[i] == y) { ok = false; break; }
        }
        if (ok) { fx_ = x; fy_ = y; return; }
    }
    fx_ = 0; fy_ = 0;
}

void SnakeGame::tick() {
    if (dead_) return;
    int nx = sx_[0] + dx_, ny = sy_[0] + dy_;
    if (nx < 0 || ny < 0 || nx >= gw_ || ny >= gh_) { dead_ = true; return; }
    for (int i = 0; i < len_ - 1; ++i) {
        if (sx_[i] == nx && sy_[i] == ny) { dead_ = true; return; }
    }
    for (int i = len_ - 1; i > 0; --i) { sx_[i] = sx_[i - 1]; sy_[i] = sy_[i - 1]; }
    sx_[0] = nx; sy_[0] = ny;
    if (nx == fx_ && ny == fy_) {
        score_ += 10;
        if (len_ < MAXLEN) ++len_;
        spawnFood();
    }
}

void SnakeGame::draw(Display& disp) const {
    disp.clear(C_BG);
    for (int i = 0; i < len_; ++i) {
        Color16 c = (i == 0) ? C_HEAD : C_SNAKE;
        disp.fillRect(sx_[i] * CELL, sy_[i] * CELL, CELL - 1, CELL - 1, c);
    }
    disp.fillRect(fx_ * CELL, fy_ * CELL, CELL - 1, CELL - 1, C_FOOD);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "Score:%d", score_);
    disp.drawText(0, 0, buf, C_TEXT, C_BG);
    if (dead_) disp.drawText(0, disp.height() / 2, "GAME OVER", C_TEXT, C_BG);
}

bool SnakeGame::loop(Input& input, Display& disp, uint32_t nowMs) {
    Key k = input.readKey();
    switch (k) {
        case Key::Up:    if (dy_ == 0) { dx_ = 0; dy_ = -1; } break;
        case Key::Down:  if (dy_ == 0) { dx_ = 0; dy_ =  1; } break;
        case Key::Left:  if (dx_ == 0) { dx_ = -1; dy_ = 0; } break;
        case Key::Right: if (dx_ == 0) { dx_ =  1; dy_ = 0; } break;
        case Key::Back:  return true;   // 退出游戏
        default: break;
    }
    if (!dead_ && (nowMs - lastStep_ >= STEP_MS)) {
        lastStep_ = nowMs;
        tick();
    }
    draw(disp);
    return false;
}

} // namespace desk
