#pragma once
#include <cstdint>

namespace desk {

// 字形度量提供者：阅读器据此对文本做按像素折行。
// 实现应与 Display::drawText 所用字体保持一致。
class Font {
public:
    virtual ~Font() = default;
    virtual int glyphWidth(uint32_t codepoint) const = 0;
    virtual int lineHeight() const = 0;
};

} // namespace desk
