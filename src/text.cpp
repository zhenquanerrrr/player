#include "text.h"
#include "gb2312_ucs2.h"

namespace desk {

bool isGbkLead(uint8_t b)  { return b >= 0x81 && b <= 0xFE; }
bool isGbkTrail(uint8_t b) { return (b >= 0x40 && b <= 0x7E) || (b >= 0x80 && b <= 0xFE); }

Encoding detectEncoding(const uint8_t* buf, size_t len) {
    if (len >= 3 && buf[0] == 0xEF && buf[1] == 0xBB && buf[2] == 0xBF)
        return Encoding::Utf8;

    bool allAscii = true;
    bool validUtf8 = true;
    size_t i = 0;
    while (i < len) {
        uint8_t b = buf[i];
        if (b < 0x80) { i += 1; continue; }
        allAscii = false;
        int need;
        if (b >= 0xC2 && b <= 0xDF) need = 2;
        else if (b >= 0xE0 && b <= 0xEF) need = 3;
        else if (b >= 0xF0 && b <= 0xF4) need = 4;
        else { validUtf8 = false; break; }
        if (i + (size_t)need > len) break; // 采样末尾被截断，视为合法前缀
        for (int k = 1; k < need; ++k) {
            if ((buf[i + k] & 0xC0) != 0x80) { validUtf8 = false; break; }
        }
        if (!validUtf8) break;
        i += (size_t)need;
    }
    if (validUtf8) {
        return allAscii ? Encoding::Ascii : Encoding::Utf8;
    }

    // GBK 启发式：所有 >=0x80 的字节必须成对（首 + 尾）
    bool looksGbk = true;
    for (i = 0; i < len; ) {
        uint8_t b = buf[i];
        if (b < 0x80) { i += 1; continue; }
        if (!isGbkLead(b)) { looksGbk = false; break; }
        if (i + 1 >= len) break;           // 采样末尾截断的孤首字节，容忍
        if (!isGbkTrail(buf[i + 1])) { looksGbk = false; break; }
        i += 2;
    }
    if (looksGbk && !allAscii) return Encoding::Gbk;
    return Encoding::Binary;
}

int utf8Decode(const uint8_t* s, size_t len, uint32_t* out) {
    if (len == 0) return 0;
    uint8_t b0 = s[0];
    if (b0 < 0x80) { *out = b0; return 1; }
    int need;
    uint32_t cp;
    if (b0 >= 0xC2 && b0 <= 0xDF)      { need = 2; cp = b0 & 0x1F; }
    else if (b0 >= 0xE0 && b0 <= 0xEF) { need = 3; cp = b0 & 0x0F; }
    else if (b0 >= 0xF0 && b0 <= 0xF4) { need = 4; cp = b0 & 0x07; }
    else { *out = 0xFFFD; return 1; }
    if (len < (size_t)need) return 0; // 需要更多字节
    for (int k = 1; k < need; ++k) {
        uint8_t c = s[k];
        if ((c & 0xC0) != 0x80) { *out = 0xFFFD; return 1; }
        cp = (cp << 6) | (c & 0x3F);
    }
    // 拒绝过长编码与代理区
    if ((need == 3 && cp < 0x800) || (need == 4 && cp < 0x10000)) { *out = 0xFFFD; return 1; }
    if (cp >= 0xD800 && cp <= 0xDFFF) { *out = 0xFFFD; return 1; }
    *out = cp;
    return need;
}

uint32_t gbkDecode(uint16_t gbk) {
    uint8_t hi = (uint8_t)(gbk >> 8);
    uint8_t lo = (uint8_t)(gbk & 0xFF);
    if (hi < 0xA1 || hi > 0xF7 || lo < 0xA1 || lo > 0xFE) return 0xFFFD;
    uint16_t u = gb2312Ucs2[(hi - 0xA1) * 94 + (lo - 0xA1)];
    return u ? (uint32_t)u : 0xFFFD;
}

} // namespace desk
