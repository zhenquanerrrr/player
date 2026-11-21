#include "reader.h"
#include <utility>

namespace desk {

static constexpr size_t CHUNK = 256;

Reader::Reader(const Font& font) : font_(&font) {}

bool Reader::open(DataSource* src) {
    src_ = src;
    enc_ = Encoding::Binary;
    eof_ = false;
    hasPage_ = false;
    hist_.clear();
    curStart_ = curEnd_ = 0;
    filePos_ = 0;
    buf_.clear();

    fileSize_ = src_->size();
    hasSize_ = (fileSize_ > 0);

    if (!src_->seek(0)) return false; // 阅读器需要可随机访问（SD 文件）

    uint8_t sample[4096];
    int n = src_->read(sample, sizeof(sample));
    Encoding e = (n > 0) ? detectEncoding(sample, (size_t)n) : Encoding::Ascii;
    if (e == Encoding::Binary) e = Encoding::Utf8; // 兜底按 UTF-8 尝试
    enc_ = e;

    filePos_ = 0;
    buf_.clear();
    eof_ = false;
    return true;
}

void Reader::resetTo(size_t offset) {
    filePos_ = offset;
    buf_.clear();
    eof_ = false;
}

bool Reader::appendMore() {
    uint8_t tmp[CHUNK];
    int n;
    if (src_->seek(filePos_ + buf_.size()))
        n = src_->read(tmp, CHUNK);
    else
        n = src_->read(tmp, CHUNK); // 顺序流：连续读
    if (n <= 0) return false;
    buf_.insert(buf_.end(), tmp, tmp + n);
    return true;
}

uint32_t Reader::nextCodepoint() {
    if (eof_) return 0;
    size_t need = (enc_ == Encoding::Gbk) ? 2 : 4;
    while (buf_.size() < need) {
        if (!appendMore()) break;
    }
    if (buf_.empty()) { eof_ = true; return 0; }

    uint32_t cp = 0xFFFD;
    int used = 1;
    if (enc_ == Encoding::Utf8 || enc_ == Encoding::Ascii) {
        used = utf8Decode(buf_.data(), buf_.size(), &cp);
        if (used == 0) { used = 1; cp = 0xFFFD; } // 缓冲不足或非法
    } else if (enc_ == Encoding::Gbk) {
        uint8_t b0 = buf_[0];
        if (b0 < 0x80) { cp = b0; used = 1; }
        else if (isGbkLead(b0)) {
            if (buf_.size() >= 2 && isGbkTrail(buf_[1])) {
                cp = gbkDecode((uint16_t)((b0 << 8) | buf_[1]));
                used = 2;
            } else { cp = 0xFFFD; used = 1; }
        } else { cp = 0xFFFD; used = 1; }
    } else { // Binary 按字节透传
        cp = buf_[0]; used = 1;
    }

    buf_.erase(buf_.begin(), buf_.begin() + used);
    filePos_ += (size_t)used;
    return cp;
}

void Reader::scanPage(int maxW, int maxH, std::vector<DisplayLine>& out, size_t& endOffset) {
    out.clear();
    int lineH = font_->lineHeight();
    if (lineH <= 0) lineH = 16;
    int maxLines = maxH / lineH;
    if (maxLines < 1) maxLines = 1;

    DisplayLine cur;
    int curW = 0;
    size_t pendingStart = filePos_; // 当前显示行的首字节偏移

    auto pushLine = [&]() {
        if (!cur.cp.empty()) {
            cur.pixelWidth = curW;
            out.push_back(std::move(cur));
            cur = DisplayLine();
            curW = 0;
        }
    };

    while ((int)out.size() < maxLines) {
        size_t beforeCp = filePos_;
        uint32_t cp = nextCodepoint();
        if (cp == 0) break; // EOF
        if (cp == '\n') { pushLine(); pendingStart = filePos_; continue; }
        if (cp == '\r') continue;
        if (cp == '\t') cp = ' ';
        int w = font_->glyphWidth(cp);
        if (w <= 0) w = font_->glyphWidth(' ');
        if (!cur.cp.empty() && curW + w > maxW) {
            pushLine();            // 折行
            pendingStart = beforeCp; // 新行从当前字符开始
        }
        if (cur.cp.empty()) pendingStart = beforeCp; // 新行首字符
        cur.cp.push_back(cp);
        curW += w;
    }
    if ((int)out.size() < maxLines) {
        pushLine();              // 页面未满：收尾（可能到 EOF）
        endOffset = filePos_;
    } else {
        endOffset = pendingStart; // 页面已满：下一行起点即下一页起点
    }
}

bool Reader::nextPage(int maxW, int maxH, std::vector<DisplayLine>& out) {
    size_t newStart = hasPage_ ? curEnd_ : curStart_;
    resetTo(newStart);
    size_t end = newStart;
    scanPage(maxW, maxH, out, end);
    if (out.empty()) return false; // 无内容（已到文末）
    if (hasPage_) hist_.push_back(curStart_);
    curStart_ = newStart;
    curEnd_ = end;
    hasPage_ = true;
    return true;
}

bool Reader::prevPage(int maxW, int maxH, std::vector<DisplayLine>& out) {
    if (hist_.empty()) return false; // 已在首页
    size_t prevStart = hist_.back();
    hist_.pop_back();
    resetTo(prevStart);
    size_t end = prevStart;
    scanPage(maxW, maxH, out, end);
    curStart_ = prevStart;
    curEnd_ = end; // 确定性：应等于原 curStart_
    hasPage_ = true;
    return true;
}

bool Reader::seekTo(size_t offset) {
    if (!src_) return false;
    if (hasSize_ && offset > fileSize_) offset = fileSize_;
    resetTo(offset);
    hist_.clear();
    curStart_ = curEnd_ = offset;
    hasPage_ = false;   // 下一页从 offset 开始
    return true;
}

float Reader::progress() const {
    if (!hasSize_ || fileSize_ == 0) return 0.0f;
    float p = (float)curEnd_ / (float)fileSize_;
    return p > 1.0f ? 1.0f : p;
}

} // namespace desk
