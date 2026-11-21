#pragma once
#include <cstdint>
#include <cstddef>

namespace desk {

enum class Encoding : uint8_t {
    Ascii  = 0,  // 纯 ASCII（可按 UTF-8 处理）
    Utf8   = 1,
    Gbk    = 2,  // 含 GB2312
    Binary = 3,
};

// 探测编码（用于 txt 开头的采样缓冲）
Encoding detectEncoding(const uint8_t* buf, size_t len);

// 解码一个 UTF-8 码点。
// 返回消耗的字节数 1..4；若 len 不足以构成完整码点返回 0；
// 非法序列吞掉一个字节并置 *out = 0xFFFD。
int utf8Decode(const uint8_t* s, size_t len, uint32_t* out);

// GBK/GB2312 两字节码 -> UCS-2 码点；未收录返回 0xFFFD
uint32_t gbkDecode(uint16_t gbk);

bool isGbkLead(uint8_t b);
bool isGbkTrail(uint8_t b);

} // namespace desk
