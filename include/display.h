#pragma once
#include <cstdint>

namespace desk {

using Color16 = uint16_t;

// 抽象屏幕接口：由具体驱动（ST7735 / 墨水屏）实现。
// 上层 UI 只依赖本接口，不直接触碰 SPI。
class Display {
public:
    virtual ~Display() = default;
    virtual int width() const = 0;
    virtual int height() const = 0;
    virtual void clear(Color16 color) = 0;
    virtual void drawPixel(int x, int y, Color16 color) = 0;
    virtual void fillRect(int x, int y, int w, int h, Color16 color) = 0;
    // 使用屏幕内置字体渲染 UTF-8 文本
    virtual void drawText(int x, int y, const char* utf8, Color16 fg, Color16 bg) = 0;
    virtual void drawBitmap(int x, int y, int w, int h, const Color16* data) = 0;
};

} // namespace desk
