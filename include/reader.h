#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include "text.h"
#include "font.h"

namespace desk {

// 抽象数据源：读文本字节。read 返回实际读取字节数，0 表示 EOF。
class DataSource {
public:
    virtual ~DataSource() = default;
    virtual int read(uint8_t* dst, size_t max) = 0;
    // 随机访问（SD 文件支持）。阅读器需要可随机访问以便前后翻页。
    virtual bool seek(size_t offset) { (void)offset; return false; }
    virtual size_t tell() const { return 0; }
    virtual size_t size() const { return 0; } // 未知返回 0
};

// 一屏内一个显示行（已按像素宽度折行），cp 为行内码点序列
struct DisplayLine {
    std::vector<uint32_t> cp;
    int pixelWidth = 0;
};

// txt 电子书阅读器：编码探测 + 折行 + 分页 + 前后翻页。
// 依赖 Font 做字形宽度度量；依赖 DataSource::seek 做向前/向后翻页。
class Reader {
public:
    explicit Reader(const Font& font);

    bool open(DataSource* src);

    Encoding encoding() const { return enc_; }

    // 读取下一屏。maxW/maxH 为像素。返回 false 表示已到文末。
    bool nextPage(int maxW, int maxH, std::vector<DisplayLine>& out);
    // 读取上一屏。返回 false 表示已在首页。
    bool prevPage(int maxW, int maxH, std::vector<DisplayLine>& out);

    // 跳转到指定字节偏移（目录跳章）。跳转后 hist_ 清空，下一页从该偏移开始。
    bool seekTo(size_t offset);

    // 当前屏起始字节偏移（用于保存阅读进度 / 定位当前章节）
    size_t position() const { return curStart_; }

    float progress() const;   // 0..1
    bool atEnd() const { return eof_; }

private:
    uint32_t nextCodepoint();            // 解码流中下一个码点；EOF 返回 0
    void     resetTo(size_t offset);     // 定位到绝对偏移并清空缓冲
    bool     appendMore();               // 读入更多字节
    void     scanPage(int maxW, int maxH, std::vector<DisplayLine>& out, size_t& endOffset);

    DataSource* src_ = nullptr;
    const Font* font_ = nullptr;
    Encoding enc_ = Encoding::Binary;

    std::vector<uint8_t> buf_;           // 未消费字节，起点为 filePos_
    size_t filePos_ = 0;                 // buf_[0] 的绝对文件偏移
    bool eof_ = false;

    bool hasPage_ = false;
    size_t curStart_ = 0;                // 当前屏起始偏移
    size_t curEnd_ = 0;                  // 当前屏结束偏移（下一屏起始）
    std::vector<size_t> hist_;           // 之前各屏起始偏移（用于后退）

    size_t fileSize_ = 0;
    bool hasSize_ = false;
};

} // namespace desk
