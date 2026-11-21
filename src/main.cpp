// ============================================================================
//  ESP32 多功能桌面设备 —— 整合版 main.cpp（完整工程见 firmware/）
//  合并自：① 菜单 UI   ② 菜单+墨水屏+BLE   ③ A2DP+VS1053+日历+天气
//  主控：ESP32-WROOM-32E (N16R2, 16MB Flash / 2MB PSRAM)  框架：Arduino
// ----------------------------------------------------------------------------
//  【引脚总表】
//   TFT(ILI9488)  : SCLK=18  MOSI=23  DC=27  CS=5   RST=33   (SPI2_HOST)
//   墨水屏(GDEY042Z98): SCK=14  MOSI=13  MISO=19  CS=16  DC=21  RST=22  BUSY=35
//   TF 卡         : CS=17   (与墨水屏、VS1053 共用 SPI3/VSPI)
//   VS1053        : XCS=4   XDCS=32  DREQ=34  RST=12   (共用 SPI3)
//   按键          : UP=39  OK=36  BACK=2   (均需外部 10K 上拉)
//   功放 SHDN     : IO15（N-MOS 栅极，高=关断；PAM8403 纯硬件音频通路）
// ----------------------------------------------------------------------------
//  【功能分区索引】
//   1  引脚与常量            2  页面状态机 + 6 项菜单
//   3  TFT 驱动(LovyanGFX)   4  墨水屏对象(GxEPD2_3C)
//   5  音频/网络/BLE 对象    6  全局状态变量
//   7  前向声明              8  TFT 辅助（壁纸/描边字/提示/壁纸列表）
//   9  中文字渲染(efontCN)    10 墨水屏功能（日历 + BLE 传图）
//   11 VS1053 + A2DP 音频     12 蓝牙歌词渲染
//   13 TF 音乐 + LRC          14 电子时钟 + WiFi/NTP + 天气
//   15 阅读器(txt)            16 菜单渲染
//   17 按键扫描               18 按键逻辑（页面分发）
//   19 setup / loop
// ----------------------------------------------------------------------------
//  【数据文件】
//   TFT 壁纸  SD:/wallpapers/*.bin  裸 RGB565 480×320×2=307200B 行序无头
//   墨水屏图  BLE 30000B = 400×300 黑白15000 + 红15000
//   中文字体  LovyanGFX 内置 efontCN_*（无需外部文件）
//   电子书    SD:/read/*.txt（UTF-8/GBK 自动识别，支持目录/进度）
//   音乐      SD:/music/*.mp3 + 同名 .lrc
// ============================================================================

#include <Arduino.h>
#include "esp32-hal-psram.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <HTTPClient.h>
#include <LovyanGFX.hpp>
#include <GxEPD2_3C.h>
#include "BluetoothA2DPSink.h"
#include <VS1053.h>
#include "esp_task_wdt.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include <time.h>

// ---- desk_core 可移植模块（阅读 / 小游戏 / 编码）----
#include "font.h"
#include "text.h"
#include "reader.h"
using namespace desk;

// ============================================================
// 1. 引脚与常量
// ============================================================
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_DC   27
#define TFT_CS   5
#define TFT_RST  33

#define EPD_SCK   14
#define EPD_MOSI  13
#define EPD_MISO  19
#define EPD_CS    16
#define EPD_DC    21
#define EPD_RST   22
#define EPD_BUSY  35
#define TF_CS     17

#define VS_XCS    4
#define VS_XDCS   32
#define VS_DREQ   34
#define VS_RST    12
#define PIN_AMP_SHDN 15   // 功放关断：N-MOS 栅极，高=关断

#define PIN_KEY_UP   39   // 仅输入，外部 10K 上拉
#define PIN_KEY_OK   36   // 仅输入，外部 10K 上拉
#define PIN_KEY_BACK 2    // strapping，外部 10K 上拉

static inline void ampOn()  { digitalWrite(PIN_AMP_SHDN, LOW); }   // MOS 截止 → SHDN 高 → 放音
static inline void ampOff() { digitalWrite(PIN_AMP_SHDN, HIGH); }  // MOS 导通 → SHDN 低 → 关断

#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 320
#define WALLPAPER_DIR "/wallpapers"
#define BOOK_DIR      "/read"
#define MUSIC_DIR     "/music"
#define TEXT_MARGIN   10
#define MAX_LINE_WIDTH (SCREEN_WIDTH - TEXT_MARGIN * 2)

// ============================================================
// 2. 页面状态
// ============================================================
enum PageState {
  PAGE_STANDBY,
  PAGE_MAIN, PAGE_EINK_MENU, PAGE_AUDIO_MENU,
  PAGE_CLOCK, PAGE_WALLPAPER_SELECT,
  PAGE_BT_LYRIC, PAGE_TF_LIST, PAGE_TF_PLAYING,
  PAGE_READER, PAGE_GAME, PAGE_EPD_WAIT_BLE, PAGE_BOOK_LIST, PAGE_CHAPTERS
};
PageState nowPage = PAGE_STANDBY;
int main_sel = 0, sub_sel = 0;

const char* main_menu[] = {
  "1.墨水屏显示", "2.歌曲播放", "3.电子时钟",
  "4.更换UI壁纸", "5.阅读", "6.小游戏"
};
#define MAIN_CNT 6
const char* eink_menu[]  = { "显示日历", "蓝牙传图", "返回" };
#define EINK_CNT 3
const char* audio_menu[] = { "蓝牙歌词", "TF卡播放", "返回" };
#define AUDIO_CNT 3

// ============================================================
// 3. TFT (LovyanGFX ILI9488)
// ============================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9488 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;
public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 40000000;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = true;
      cfg.use_lock    = true;
      cfg.dma_channel = 0;   // 关闭 DMA，省内存给蓝牙（TFT 稍慢但可用）
      cfg.pin_sclk = TFT_SCLK;
      cfg.pin_mosi = TFT_MOSI;
      cfg.pin_miso = -1;
      cfg.pin_dc   = TFT_DC;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs       = TFT_CS;
      cfg.pin_rst      = TFT_RST;
      cfg.pin_busy     = -1;
      cfg.panel_width  = 320;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.invert   = false;
      cfg.rgb_order = false;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};
LGFX tft;

// ============================================================
// 4. 墨水屏 (GxEPD2_3C GDEY042Z98, 400x300 三色)
// ============================================================
GxEPD2_3C<GxEPD2_420c_GDEY042Z98, GxEPD2_420c_GDEY042Z98::HEIGHT> epd(
    GxEPD2_420c_GDEY042Z98(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// ============================================================
// 5. 音频 / 网络 / BLE 对象
// ============================================================
VS1053 player(VS_XCS, VS_XDCS, VS_DREQ);
BluetoothA2DPSink a2dp_sink;

WiFiUDP udp;
NTPClient ntp(udp, "ntp.aliyun.com", 8 * 3600);
const char* wifiName = "114514";
const char* wifiPwd  = "1234567890";

// BLE 传图（墨水屏）
#define BLE_SERVICE_UUID "0000ff01-0000-1000-8000-00805f9b34fb"
#define BLE_CHAR_UUID    "0000ff02-0000-1000-8000-00805f9b34fb"
#define IMG_SIZE 30000
BLEServer* pServer = NULL;
BLECharacteristic* pChar = NULL;
bool deviceConnected = false;
uint8_t* bleRxBuffer = NULL;
volatile uint32_t bytesReceived = 0;
volatile bool dataReady = false;

// ============================================================
// 6. 全局状态
// ============================================================
// TFT 壁纸
#define MAX_WALLPAPERS 8
String wallpaperFiles[MAX_WALLPAPERS];
int wallpaperCount = 0, currentWallpaperIdx = 0, selectWallpaperIdx = 0;
File wallFile;
static uint16_t* wallBuf = nullptr;   // PSRAM 壁纸缓存（480*320）
#define MAX_BOOKS 8
String bookFiles[MAX_BOOKS];
int bookCount = 0, bookSel = 0;

// 时间 / 天气
unsigned long ntpEpoch = 0, lastMillis = 0;
bool timeInitialized = false, dataAvailable = false, weatherReady = false;
float temperature = 0; int humidity = 0, weatherCode = 0;
String weatherDesc = "";
static uint32_t lastTimeUpdate = 0;
static String lastTimeStr = "";

// 歌词 / 蓝牙
bool btIsConnect = false, waitingForBT = false, btInited = false;
bool vsInited = false, vsWavHeaderSent = false;
#define PCM_BUF_SIZE 8193    // 必须放内部 RAM：蓝牙回调读写 PSRAM 会触发缓存 bug
uint8_t pcmBuffer[PCM_BUF_SIZE];
volatile uint32_t pcmWritePos = 0, pcmReadPos = 0;
uint8_t wavHeader[44];
char songName[40] = "", singerName[30] = "";
char currLyric[200] = "", lastLyric[200] = "";
char lyricLine1[64] = "", lyricLine2[64] = "", lyricLine3[64] = "";
char lastLine1[64] = "", lastLine2[64] = "";
static char g_lyricLine[3][64];   // 按字体实际宽度换行后的各行
static int g_lyricLines = 0;
char pendingLyric[200] = "", fullArtistRaw[60] = "";
bool hasPendingLyric = false;
volatile bool globalFlagArtistUpdate = false;
bool flagRefreshAll = false, flagRefreshLyric = false;
unsigned long lastLyricShowTime = 0;
const unsigned long lyricShowMinGap = 50;

// 频谱（播放界面：歌词 140,172；频谱/音量区 220~310）
#define VIEW_BOTTOM_Y 310
#define BAR_MAX_HEIGHT 80
#define BAR_WIDTH 20
#define BAR_SPACE 6
#define BAR_START_X 20
#define BAR_NUM 16
uint8_t barH[BAR_NUM] = {0}, barTargetH[BAR_NUM] = {0};
const uint16_t barColors[] = {
  0xF800, // 红
  0xFD20, // 橙
  0xFEA0, // 金黄
  0xFFE0, // 黄
  0x87E0, // 黄绿
  0x07E0, // 绿
  0x07F0, // 青绿
  0x07FF, // 青
  0x041F, // 天蓝
  0x001F, // 蓝
  0x801F, // 紫罗兰
  0xF81F, // 品红
  0xF810, // 玫红
  0xFA10, // 粉
  0x8200, // 棕
  0x8410  // 灰
};
#define COLOR_COUNT (sizeof(barColors)/sizeof(barColors[0]))
volatile uint16_t audioVolPeak = 0;
volatile bool audioCalcLock = false;
#define VOL_MAX_LIMIT 12000
uint32_t volumeOverlayUntil = 0;   // 音量覆盖显示截止时间
int g_audioVol = 100;              // 软件音量 1~100

// 文字尺寸（LovyanGFX 内置 efontCN：中文约 24px，ASCII 约 12px）
#define CHN_CHAR_W 24
#define ENG_CHAR_W 12

// TF 音频
#define MAX_AUDIO_FILES 8
#define MAX_FILENAME 64
char audioFileList[MAX_AUDIO_FILES][MAX_FILENAME];
int audioFileCount = 0, audioFileSel = 0;

// LRC（TF 播放用）
#define MAX_LRC_LINES 20
#define MAX_LRC_TEXT 80
struct LrcLine { uint32_t timeMs; char text[MAX_LRC_TEXT]; };
LrcLine lrcLines[MAX_LRC_LINES];
int lrcCount = 0;
char lrcTitle[40] = "", lrcArtist[40] = "";

// 菜单局部刷新
const char** g_menuItems = NULL;
int g_menuCount = 0, g_startY = 90, g_itemSpacing = 60, g_currentSel = 0;
static const char* g_listNames[8];
static char g_listBuf[8][64];   // 去路径后的文件名缓冲
static int g_listN = 0;

// ============================================================
// 7. 前向声明
// ============================================================
void drawWallpaperRegion(int x, int y, int w, int h);
void drawTFTMessage(const char* msg);
void scanWallpapers();
void loadWallpaperList();
bool loadWallpaper(const char* path);
void drawWallpaperSelect();
void initSD();
void printChineseOutlined(int16_t x, int16_t y, const char* u, uint16_t color);
void printChinese16(int16_t x, int16_t y, const char* u, uint16_t color);
int  centerFor(const lgfx::IFont* f, const char* s);
void drawBigTime(const char* ts, bool clearBg);
void menuRedrawRow(int i, bool sel);
void drawMenuSelect(int oldSel, int newSel);
void drawStandby();
void printChinese(int16_t x, int16_t y, const char* utf8Str, uint16_t color, int16_t gap = CHN_CHAR_W);
int  getTextCenterX(const char* str);

void showCalendarOnEPD();
void showImageOnEPD();
void startBLE();
void stopBLE();
void initVS1053();
void startBluetooth();
void stopBluetooth();
void initWiFiAndGetData();
unsigned long getCurrentEpoch();
void showDateTimePage();
void updateTimeOnly();
void scanAudioFiles();
void playAudioFile(const char* path);
void showVolumeOverlay();
void spectrumTick();
void drawAudioList();
void readerEnter(const char* path);
void scanBooks();
void drawBookList();
void readerRender(bool next);
void gameEnter();
bool gameLoop();
void drawMenu();
void keyLogicTask();
void drawListPage(const char* title, int sel);
void listRedrawRow(int i, bool sel);
bool keyScan(int pin, int buttonIndex, bool edge = true);

// ============================================================
// 8. TFT 辅助
// ============================================================
void drawWallpaperRegion(int x, int y, int w, int h) {
  // 双行缓冲：pushImage 可能异步（DMA），单缓冲会被下一行 memcpy 覆盖导致花屏
  static uint8_t lineBuf[2][SCREEN_WIDTH * 2];
  int bi = 0;
  // 优先用 PSRAM 缓存：先拷到内部 RAM 再 push（DMA 不能直接读 PSRAM）
  if (wallBuf) {
    for (int i = 0; i < h; i++) {
      int cy = y + i;
      if (cy < 0 || cy >= SCREEN_HEIGHT) break;
      memcpy(lineBuf[bi], &wallBuf[cy * SCREEN_WIDTH + x], (size_t)w * 2);
      tft.pushImage(x, cy, w, 1, (uint16_t*)lineBuf[bi]);
      bi ^= 1;
    }
    return;
  }
  if (!wallFile) { tft.fillRect(x, y, w, h, TFT_BLACK); return; }
  for (int i = 0; i < h; i++) {
    int cy = y + i;
    if (cy < 0 || cy >= SCREEN_HEIGHT) break;
    uint32_t offset = ((uint32_t)cy * SCREEN_WIDTH + x) * 2;
    wallFile.seek(offset);
    wallFile.read(lineBuf[bi], w * 2);
    tft.pushImage(x, cy, w, 1, (uint16_t*)lineBuf[bi]);
    bi ^= 1;
  }
}

void drawTFTMessage(const char* msg) {
  tft.fillScreen(TFT_BLACK);
  printChinese(getTextCenterX(msg), 140, msg, TFT_YELLOW);
}

// ---- SD / 壁纸 ----
void initSD() {
  SPI.begin(EPD_SCK, EPD_MISO, EPD_MOSI, TF_CS);
  delay(50);
  SD.begin(TF_CS, SPI);
}

void scanWallpapers() {
  wallpaperCount = 0;
  if (!SD.exists(WALLPAPER_DIR)) return;
  File root = SD.open(WALLPAPER_DIR);
  if (!root) return;
  File e;
  while ((e = root.openNextFile()) && wallpaperCount < MAX_WALLPAPERS) {
    if (e.isDirectory()) { e.close(); continue; }
    String raw = e.name();
    int sl = raw.lastIndexOf('/');
    if (sl >= 0) raw = raw.substring(sl + 1);
    String full = String(WALLPAPER_DIR) + "/" + raw;
    String low = full; low.toLowerCase();
    if (low.endsWith(".bin")) wallpaperFiles[wallpaperCount++] = full;
    e.close();
  }
  root.close();
}

void loadWallpaperList() {
  scanWallpapers();
  if (wallpaperCount > 0 && currentWallpaperIdx < wallpaperCount)
    loadWallpaper(wallpaperFiles[currentWallpaperIdx].c_str());
}

bool loadWallpaper(const char* path) {
  if (wallFile) wallFile.close();
  if (!path || !SD.exists(path)) return false;
  wallFile = SD.open(path, FILE_READ);
  return wallFile ? true : false;
}

static bool loadWallpaperToPSRAM(const char* path) {
  if (!wallBuf) return false;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  size_t n = f.read((uint8_t*)wallBuf, (size_t)SCREEN_WIDTH * SCREEN_HEIGHT * 2);
  f.close();
  return n == (size_t)SCREEN_WIDTH * SCREEN_HEIGHT * 2;
}

void drawWallpaperSelect() {
  g_listN = wallpaperCount > 6 ? 6 : wallpaperCount;
  for (int i = 0; i < g_listN; i++) {
    const char* p = wallpaperFiles[i].c_str();
    const char* s = strrchr(p, '/');
    strncpy(g_listBuf[i], s ? s + 1 : p, 63);
    g_listBuf[i][63] = 0;
    g_listNames[i] = g_listBuf[i];
  }
  if (!g_listN) {
    tft.fillScreen(TFT_BLACK);
    printChineseOutlined(centerFor(&fonts::efontCN_24_b, "文件夹为空"), 140, "文件夹为空", TFT_RED);
    return;
  }
  if (selectWallpaperIdx >= g_listN) selectWallpaperIdx = 0;
  drawListPage("选择壁纸", selectWallpaperIdx);
}

// ============================================================
// 9. 中文文字渲染（直接用 LovyanGFX 内置 efontCN 字体）
// ============================================================
// 说明：不再依赖 SPIFFS 点阵字库 / UTF8ToGB2312.h。
// LovyanGFX 的 efontCN_* 可直接渲染 UTF-8 中文（与参考#1 一致）。
void printChinese(int16_t x, int16_t y, const char* u, uint16_t color, int16_t gap) {
  (void)gap;
  if (!u) return;
  tft.setTextSize(1.0);              // 关键：防止上页残留 setTextSize 导致字体变大
  tft.setFont(&fonts::efontCN_24_b);
  tft.setTextColor(color);          // 透明背景，便于叠在壁纸上
  tft.setCursor(x, y);
  tft.print(u);
}

void printChineseOutlined(int16_t x, int16_t y, const char* u, uint16_t color) {
  for (int dx = -2; dx <= 2; dx += 2)
    for (int dy = -2; dy <= 2; dy += 2) {
      if (!dx && !dy) continue;
      printChinese(x + dx, y + dy, u, TFT_BLACK);
    }
  printChinese(x, y, u, color);
}

// 16px 小字（底部提示用）
void printChinese16(int16_t x, int16_t y, const char* u, uint16_t color) {
  if (!u) return;
  tft.setTextSize(1.0);
  tft.setFont(&fonts::efontCN_16_b);
  tft.setTextColor(color);
  tft.setCursor(x, y);
  tft.print(u);
}

// 按指定字体精确居中
int centerFor(const lgfx::IFont* f, const char* s) {
  tft.setFont(f);
  int x = (SCREEN_WIDTH - tft.textWidth(s)) / 2;
  return x < 0 ? 0 : x;
}

// 通用列表页（壁纸/音频共用，与主菜单同风格，支持局部刷新）
void listRedrawRow(int i, bool sel) {
  if (i < 0 || i >= g_listN) return;
  int y = 72 + i * 38;
  drawWallpaperRegion(0, y - 4, SCREEN_WIDTH, 34);   // 恢复该行壁纸
  uint16_t c = sel ? TFT_RED : TFT_WHITE;
  printChineseOutlined(centerFor(&fonts::efontCN_24_b, g_listNames[i]), y, g_listNames[i], c);
  if (sel) printChineseOutlined(40, y, ">", TFT_RED);
}

void drawListPage(const char* title, int sel) {
  if (wallBuf || wallFile) drawWallpaperRegion(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
  else tft.fillScreen(TFT_BLACK);
  printChineseOutlined(centerFor(&fonts::efontCN_24_b, title), 18, title, TFT_CYAN);
  for (int i = 0; i < g_listN; i++) {
    int y = 72 + i * 38;
    uint16_t c = (i == sel) ? TFT_RED : TFT_WHITE;
    printChineseOutlined(centerFor(&fonts::efontCN_24_b, g_listNames[i]), y, g_listNames[i], c);
    if (i == sel) printChineseOutlined(40, y, ">", TFT_RED);
  }
}

int getUtf8TotalWidth(const char* u) {
  int w = 0;
  const uint8_t* p = (const uint8_t*)u;
  while (*p) {
    if ((*p & 0xE0) == 0xE0) { w += CHN_CHAR_W; p += 3; }
    else { w += ENG_CHAR_W; p++; }
  }
  return w;
}

int getTextCenterX(const char* s) {
  int x = (SCREEN_WIDTH - getUtf8TotalWidth(s)) / 2;
  return x < TEXT_MARGIN ? TEXT_MARGIN : x;
}

// ============================================================
// 10. 墨水屏：日历 + BLE 传图
// ============================================================
static void epdEnd() {
  // 不做 hibernate（其 SPI 事务会触发总线锁断言）；保持唤醒即可
}

void showCalendarOnEPD() {
  stopBluetooth();   // 先停 A2DP，腾出内存
  // 墨水屏日历需要时间：未同步则联网 NTP，取到后自动断开 WiFi
  if (!timeInitialized) {
    tft.fillScreen(TFT_BLACK);
    printChinese(getTextCenterX("正在联网获取时间..."), 140, "正在联网获取时间...", TFT_YELLOW);
    initWiFiAndGetData();
  }
  // 共享 SPI 已初始化，墨水屏引脚相同，不再 SPI.end/begin（避免销毁总线锁）
  delay(50);
  epd.init(115200, true, 20, false);
  epd.setRotation(0);
  epd.setFullWindow();
  unsigned long now = getCurrentEpoch();
  epd.firstPage();
  do {
    epd.fillScreen(GxEPD_WHITE);
    if (now == 0) {
      epd.setTextColor(GxEPD_BLACK);
      epd.setTextSize(2);
      epd.setCursor(10, 10);
      epd.print("Time not synced");
    } else {
      time_t t = (time_t)now;
      struct tm* td = localtime(&t);
      int year = td->tm_year + 1900, month = td->tm_mon + 1, day = td->tm_mday;

      // 头部：年份 + 月/日 + 周数
      epd.setTextColor(GxEPD_RED);
      epd.setTextSize(3);
      epd.setCursor(8, 6);
      char ybuf[8]; snprintf(ybuf, sizeof(ybuf), "%d", year);
      epd.print(ybuf);
      char mdbuf[10]; snprintf(mdbuf, sizeof(mdbuf), "%d/%d", month, day);
      epd.setCursor(175, 6);
      epd.print(mdbuf);
      epd.setTextColor(GxEPD_BLACK);
      epd.setTextSize(2);
      epd.setCursor(332, 12);
      char wkbuf[10]; snprintf(wkbuf, sizeof(wkbuf), "WK%d", (td->tm_yday + 7 - td->tm_wday) / 7);
      epd.print(wkbuf);

      // 周条：两端红、中间黑
      epd.fillRect(6, 42, 52, 24, GxEPD_RED);
      epd.fillRect(58, 42, 284, 24, GxEPD_BLACK);
      epd.fillRect(342, 42, 52, 24, GxEPD_RED);
      const char* wd = "SMTWTFS";
      epd.setTextSize(2);
      epd.setTextColor(GxEPD_WHITE);
      for (int i = 0; i < 7; i++) {
        epd.setCursor(26 + i * 54, 46);
        char c[2] = { wd[i], 0 };
        epd.print(c);
      }

      // 网格
      struct tm fd = {0};
      fd.tm_year = year - 1900; fd.tm_mon = month - 1; fd.tm_mday = 1;
      mktime(&fd);
      int fw = fd.tm_wday;
      int dim;
      if (month == 2) { bool leap = (year%4==0 && year%100!=0) || (year%400==0); dim = leap?29:28; }
      else if (month==4||month==6||month==9||month==11) dim = 30;
      else dim = 31;
      epd.setTextSize(2);
      for (int d = 1; d <= dim; d++) {
        int idx = fw + d - 1;
        int x = 8 + (idx % 7) * 54;
        int y = 78 + (idx / 7) * 36;
        if (d == day) {
          epd.fillRoundRect(x - 2, y - 2, 34, 30, 6, GxEPD_RED);
          epd.setTextColor(GxEPD_WHITE);
        } else {
          epd.setTextColor(GxEPD_BLACK);
        }
        epd.setCursor(x + 8, y + 4);
        char dbuf[4]; snprintf(dbuf, sizeof(dbuf), "%d", d);
        epd.print(dbuf);
      }
    }
  } while (epd.nextPage());
  esp_task_wdt_reset();
  epdEnd();   // 不做 hibernate：避免收尾 SPI 断言重启
  // 不重挂 SD、不动 WDT：SD 在下次 TF/阅读时自行 begin，避免退出时崩溃
}

void showImageOnEPD() {
  if (!bleRxBuffer) return;
  
  // 共享 SPI 已初始化，不再 SPI.end/begin
  delay(50);
  epd.init(115200, true, 20, false);
  epd.setRotation(0);
  epd.setFullWindow();
  delay(500);
  epd.epd2.writeImage(bleRxBuffer, bleRxBuffer + 15000, 0, 0, 400, 300);
  epd.epd2.refresh(false);
  
  esp_task_wdt_reset();
  epdEnd();
  // 不动 SD/WDT，避免退出时崩溃
}

// BLE 回调
class MyServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) { deviceConnected = true; }
  void onDisconnect(BLEServer* s) { deviceConnected = false; BLEDevice::startAdvertising(); }
};
class MyCharCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    if (dataReady || !bleRxBuffer) return;
    std::string v = c->getValue();
    for (size_t i = 0; i < v.length() && bytesReceived < IMG_SIZE; i++)
      bleRxBuffer[bytesReceived++] = v[i];
    if (bytesReceived >= IMG_SIZE) dataReady = true;
  }
};

void startBLE() {
  stopBluetooth();   // BLE 前先停 A2DP，腾出内存和蓝牙栈
  if (!bleRxBuffer) {
    bleRxBuffer = (uint8_t*)heap_caps_malloc(IMG_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!bleRxBuffer) bleRxBuffer = (uint8_t*)malloc(IMG_SIZE);
    if (!bleRxBuffer) { drawTFTMessage("内存不足"); return; }  }
  memset(bleRxBuffer, 0xFF, IMG_SIZE);
  bytesReceived = 0; dataReady = false; deviceConnected = false;
  BLEDevice::init("EPD_Custom");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCb());
  BLEService* svc = pServer->createService(BLE_SERVICE_UUID);
  pChar = svc->createCharacteristic(BLE_CHAR_UUID,
           BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pChar->setCallbacks(new MyCharCb());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  }

void stopBLE() {
  BLEDevice::deinit(true);
  // 不释放 bleRxBuffer：紧接着的 EPD 刷新还要用；下次 startBLE 复用
  bytesReceived = 0; dataReady = false;
}

// EPD 独立启动模式：只跑 EPD（不初始化 TFT/SD/A2DP），刷完重启回主程序
static // ============================================================
// 11. 音频：VS1053 + A2DP
// ============================================================
void generateWavHeader(uint32_t sr, uint16_t ch, uint16_t bits) {
  uint32_t br = sr * ch * bits / 8;
  uint16_t ba = ch * bits / 8;
  memcpy(wavHeader, "RIFF", 4);
  wavHeader[4]=0x00; wavHeader[5]=0xFF; wavHeader[6]=0xFF; wavHeader[7]=0x7F;
  memcpy(wavHeader+8, "WAVE", 4);
  memcpy(wavHeader+12, "fmt ", 4);
  wavHeader[16]=16; wavHeader[17]=0; wavHeader[18]=0; wavHeader[19]=0;
  wavHeader[20]=1; wavHeader[21]=0;
  wavHeader[22]=ch&0xFF; wavHeader[23]=(ch>>8)&0xFF;
  wavHeader[24]=sr&0xFF; wavHeader[25]=(sr>>8)&0xFF; wavHeader[26]=(sr>>16)&0xFF; wavHeader[27]=(sr>>24)&0xFF;
  wavHeader[28]=br&0xFF; wavHeader[29]=(br>>8)&0xFF; wavHeader[30]=(br>>16)&0xFF; wavHeader[31]=(br>>24)&0xFF;
  wavHeader[32]=ba&0xFF; wavHeader[33]=(ba>>8)&0xFF;
  wavHeader[34]=bits&0xFF; wavHeader[35]=(bits>>8)&0xFF;
  memcpy(wavHeader+36, "data", 4);
  wavHeader[40]=0x00; wavHeader[41]=0xFF; wavHeader[42]=0xFF; wavHeader[43]=0x7F;
}

void initVS1053() {
  if (vsInited) return;
  
  pinMode(VS_RST, OUTPUT);
  digitalWrite(VS_RST, LOW); delay(50);
  digitalWrite(VS_RST, HIGH); delay(200);
  
  
  // 共享 SPI 已初始化，不再 SPI.begin
  SPI.setFrequency(4000000);
  SPI.setDataMode(SPI_MODE0);
  
  player.begin();
  
  // ★ 关键：不要调用 switchToMp3Mode()，否则 VS1053 会拒绝解析 WAV 头
  player.setVolume(100);
  vsInited = player.isChipConnected();
  
}

void audioCallback(const uint8_t* data, uint32_t len) {
  if (!data || !len) return;
  static bool firstCb = true;
  if (firstCb) { firstCb = false;  }
  for (uint32_t i = 0; i < len; i++) {
    uint32_t n = (pcmWritePos + 1) % PCM_BUF_SIZE;
    if (n == pcmReadPos) break;
    pcmBuffer[pcmWritePos] = data[i];
    pcmWritePos = n;
  }
  // 蓝牙真实音量：统计本回调的 PCM 峰值
  if (!audioCalcLock) {
    audioCalcLock = true;
    uint16_t v = 0;
    for (uint32_t k = 0; k + 1 < len; k += 4) {
      int16_t s = (int16_t)((data[k+1] << 8) | data[k]);
      uint16_t a = abs(s);
      if (a > v) v = a;
    }
    if (v > VOL_MAX_LIMIT) v = VOL_MAX_LIMIT;
    if (v > audioVolPeak) audioVolPeak = v;
    audioCalcLock = false;
  }
}

void forwardPcmToVS1053() {
  if (!btIsConnect || !vsInited) return;
  if (!vsWavHeaderSent) {
    uint32_t avail = (pcmWritePos - pcmReadPos + PCM_BUF_SIZE) % PCM_BUF_SIZE;
    if (avail < 1024) return;
    generateWavHeader(44100, 2, 16);
    uint32_t i = 0;
    while (i < 44) {
      if (!player.data_request()) { delay(1); continue; }
      uint32_t n = (44 - i) < 32 ? (44 - i) : 32;
      player.playChunk(wavHeader + i, n);
      i += n;
    }
    vsWavHeaderSent = true;
    
  }
  // 连续喂满（512 字节/包），避免欠载产生"电音"
  uint8_t chunk[512];
  while (player.data_request()) {
    uint32_t avail = (pcmWritePos - pcmReadPos + PCM_BUF_SIZE) % PCM_BUF_SIZE;
    if (avail == 0) break;
    uint32_t n = (avail > sizeof(chunk)) ? sizeof(chunk) : avail;
    for (uint32_t i = 0; i < n; i++) {
      chunk[i] = pcmBuffer[pcmReadPos];
      pcmReadPos = (pcmReadPos + 1) % PCM_BUF_SIZE;
    }
    player.playChunk(chunk, n);
  }
}

void btConnectState(esp_a2d_connection_state_t state, void*) {
  
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    btIsConnect = true;
    ampOn();                       // 功放使能（MOS 截止 → SHDN 高）
    
    if (waitingForBT) { waitingForBT = false; flagRefreshAll = true; }
  } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
    btIsConnect = false;
    ampOff();                      // 断开时关断功放，防 pop
     waitingForBT = false; vsWavHeaderSent = false;
    pcmWritePos = 0; pcmReadPos = 0;
    songName[0] = 0; currLyric[0] = 0;
    flagRefreshAll = true;
  }
}

void musicInfoCallback(uint8_t attrId, const uint8_t* txt) {
  if (!txt) return;
  const char* s = (const char*)txt;
  if (!strlen(s)) return;
  if (attrId == 0x02) {  // ARTIST
    strncpy(fullArtistRaw, s, sizeof(fullArtistRaw)-1);
    fullArtistRaw[sizeof(fullArtistRaw)-1] = 0;
    globalFlagArtistUpdate = true;
  } else if (attrId == 0x01) {  // TITLE
    if (!strcmp(s, currLyric)) return;
    strncpy(pendingLyric, s, sizeof(pendingLyric)-1);
    pendingLyric[sizeof(pendingLyric)-1] = 0;
    hasPendingLyric = true;
    }
}

void startBluetooth() {
  if (btInited) return;
  pcmWritePos = 0; pcmReadPos = 0; vsWavHeaderSent = false;
  // ★ 蓝牙音频同样走 VS1053：先确保 VS1053 已初始化（复位+音量）
  vsInited = false;
  initVS1053();
  a2dp_sink.set_stream_reader(audioCallback);
  a2dp_sink.set_on_connection_state_changed(btConnectState);
  // ★ 最开始那段代码的歌词逻辑：TITLE=当前歌词行，ARTIST=「歌名 - 歌手」
  a2dp_sink.set_avrc_metadata_callback(musicInfoCallback);
  a2dp_sink.set_avrc_metadata_attribute_mask(
      ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST |
      ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_TRACK_NUM |
      ESP_AVRC_MD_ATTR_NUM_TRACKS | ESP_AVRC_MD_ATTR_GENRE |
      ESP_AVRC_MD_ATTR_PLAYING_TIME);
  
  
  a2dp_sink.start("ESP32_HiFi");
  
  
  btInited = true;
}

void stopBluetooth() {
  if (!btInited) return;
  ampOff();
  a2dp_sink.end();
  delay(300);
  btInited = false; btIsConnect = false; waitingForBT = false;
  vsWavHeaderSent = false; pcmWritePos = 0; pcmReadPos = 0;
  flagRefreshAll = flagRefreshLyric = false;
  songName[0] = 0; singerName[0] = 0; currLyric[0] = 0; lastLyric[0] = 0;
}

// ============================================================
// 12. 歌词渲染
// ============================================================
void lyricAutoSplit(const char* src, char l1[64], char l2[64]) {
  memset(l1, 0, 64); memset(l2, 0, 64);
  g_lyricLines = 0;
  if (!src || !*src) return;

  // 按 efontCN_24_b 的实际像素宽度换行（最多 3 行）
  tft.setFont(&fonts::efontCN_24_b);
  tft.setTextSize(1.0);

  int n = 0;
  char cur[64]; int cl = 0;
  const uint8_t* p = (const uint8_t*)src;
  while (*p && n < 3) {
    int len = 1;
    if ((*p & 0xF0) == 0xF0) len = 4;
    else if ((*p & 0xE0) == 0xE0) len = 3;
    else if ((*p & 0xC0) == 0xC0) len = 2;
    if (cl + len >= 64) break;

    char ch[5] = {0};
    for (int k = 0; k < len && p[k]; k++) ch[k] = p[k];

    char test[64];
    memcpy(test, cur, cl);
    memcpy(test + cl, ch, len);
    test[cl + len] = 0;

    if (cl > 0 && tft.textWidth(test) > MAX_LINE_WIDTH) {
      cur[cl] = 0;
      strncpy(g_lyricLine[n], cur, 63); g_lyricLine[n][63] = 0; n++;
      cl = 0;
    }
    memcpy(cur + cl, ch, len); cl += len;
    p += len;
  }
  if (cl > 0 && n < 3) {
    cur[cl] = 0;
    strncpy(g_lyricLine[n], cur, 63); g_lyricLine[n][63] = 0; n++;
  }
  g_lyricLines = n;

  if (n > 0) strcpy(l1, g_lyricLine[0]);
  if (n > 1) strcpy(l2, g_lyricLine[1]);
  if (n > 2) strcpy(lyricLine3, g_lyricLine[2]); else lyricLine3[0] = 0;
}

void slashToDash(char* b) { if (b) for (int i = 0; b[i]; i++) if (b[i] == '/') b[i] = '-'; }

void extractSongName(const char* src, char* out, int n) {
  out[0] = 0;
  const char* pos = src ? strstr(src, " - ") : NULL;
  if (pos) {
    int len = pos - src;
    if (len >= n) len = n - 1;
    strncpy(out, src, len); out[len] = 0;
  } else if (src) {
    strncpy(out, src, n - 1); out[n-1] = 0;
  }
}

const char* getSingerFromDash(const char* src) {
  const char* p = src ? strstr(src, " - ") : NULL;
  if (p) { const char* s = p + 3; while (*s == ' ') s++; return s; }
  return src;
}

void refreshAllScreen() {
  if (!btIsConnect) {
    tft.fillScreen(TFT_BLACK);
    printChinese(getTextCenterX("蓝牙未连接"), 100, "蓝牙未连接", TFT_GREEN);
    return;
  }
  if (!strlen(songName) && !strlen(currLyric)) {
    tft.fillScreen(TFT_BLACK);
    printChinese(getTextCenterX("蓝牙已连接"), 100, "蓝牙已连接", TFT_GREEN);
    printChinese(getTextCenterX("播放音乐加载歌词"), 150, "播放音乐加载歌词", TFT_WHITE);
    return;
  }
  tft.fillScreen(TFT_BLACK);
  if (strlen(songName)) printChinese(getTextCenterX(songName), 12, songName, TFT_YELLOW);
  if (strlen(singerName)) printChinese(getTextCenterX(singerName), 42, singerName, TFT_DARKGREY);
  tft.drawLine(30, 66, SCREEN_WIDTH-30, 66, TFT_DARKGREY);
  if (strlen(currLyric)) {
    lyricAutoSplit(currLyric, lyricLine1, lyricLine2);
    int y = 140;
    for (int i = 0; i < g_lyricLines && i < 2; i++) {
      printChinese(getTextCenterX(g_lyricLine[i]), y, g_lyricLine[i], TFT_CYAN);
      y += 34;
    }
  }
  memset(barH, 0, sizeof(barH)); memset(barTargetH, 0, sizeof(barTargetH));
  spectrumTick();
}

void refreshOnlyLyric() {
  if (!btIsConnect) return;
  tft.fillRect(0, 70, SCREEN_WIDTH, 126, TFT_BLACK);
  if (strlen(lastLyric)) {
    printChinese(getTextCenterX(lastLine1), 76, lastLine1, 0x7BEF);
    if (strlen(lastLine2)) printChinese(getTextCenterX(lastLine2), 108, lastLine2, 0x7BEF);
  }
  if (strlen(currLyric)) {
    lyricAutoSplit(currLyric, lyricLine1, lyricLine2);
    int y = 140;
    for (int i = 0; i < g_lyricLines && i < 2; i++) {
      printChinese(getTextCenterX(g_lyricLine[i]), y, g_lyricLine[i], TFT_CYAN);
      y += 34;
    }
  }
}

// 音量覆盖显示（只在按键调音量时出现，显示期间不画频谱）
void showVolumeOverlay() {
  volumeOverlayUntil = millis() + 1200;
  tft.fillRect(0, 220, SCREEN_WIDTH, 100, TFT_BLACK);
  char b[24];
  snprintf(b, sizeof(b), "音量 %d", g_audioVol);
  printChinese(getTextCenterX(b), 238, b, TFT_WHITE);
  int w = g_audioVol * 3;
  tft.fillRect(90, 288, w, 8, TFT_GREEN);
}

// 频谱：蓝牙用真实峰值，TF 用随机数；150ms 换目标、50ms 平滑绘制
// 关键：不做整块清屏，只“增高补色 / 降低擦黑”，避免肉眼可见刷新
void spectrumTick() {
  static uint32_t lastDraw = 0, lastTarget = 0;
  static bool overlayWas = false;
  uint32_t now = millis();
  bool overlay = (int32_t)(now - volumeOverlayUntil) < 0;
  if (overlay) { overlayWas = true; return; }               // 音量覆盖中
  if (overlayWas) {                                        // 音量刚结束：一次性清一次
    overlayWas = false;
    tft.fillRect(0, 220, SCREEN_WIDTH, 100, TFT_BLACK);
    memset(barH, 0, sizeof(barH));
  }
  if (now - lastTarget >= 150) {
    lastTarget = now;
    bool real = (btIsConnect && nowPage == PAGE_BT_LYRIC);
    int peak = real ? (int)audioVolPeak : 0;
    if (real) audioVolPeak = 0;
    int mid = BAR_NUM / 2;
    for (int i = 0; i < BAR_NUM; i++) {
      int base;
      if (real) {
        int h = map(peak, 0, VOL_MAX_LIMIT, 4, BAR_MAX_HEIGHT);
        int env = 100 - abs(i - mid) * 4;          // 包络更平缓
        if (env < 40) env = 40;
        int jit = (int)random(45, 136);            // 每根柱子 0.45~1.35 倍，相邻差异更明显
        base = h * env / 100 * jit / 100 + (int)random(-15, 16);
      } else {
        base = (int)random(4, BAR_MAX_HEIGHT + 1);
      }
      barTargetH[i] = constrain(base, 2, BAR_MAX_HEIGHT);
    }
  }
  if (now - lastDraw < 50) return;
  lastDraw = now;
  tft.startWrite();                     // 批量 SPI，减少每次 fillRect 的开销
  for (int i = 0; i < BAR_NUM; i++) {
    int oldH = barH[i];
    int d = (int)barTargetH[i] - oldH;
    int newH = oldH + d / 3 + (d > 0 ? 1 : (d < 0 ? -1 : 0));
    if (newH < 0) newH = 0;
    if (newH > BAR_MAX_HEIGHT) newH = BAR_MAX_HEIGHT;
    int x = BAR_START_X + i * (BAR_WIDTH + BAR_SPACE);
    int ci = i % (int)COLOR_COUNT;                 // 颜色固定：同一根柱子始终同色
    if (newH > oldH) {
      tft.fillRect(x, VIEW_BOTTOM_Y - newH, BAR_WIDTH, newH - oldH, barColors[ci]);
    } else if (newH < oldH) {
      tft.fillRect(x, VIEW_BOTTOM_Y - oldH, BAR_WIDTH, oldH - newH, TFT_BLACK);
    }
    barH[i] = newH;
  }
  tft.endWrite();
}

void btLyricsTick() {
  if (globalFlagArtistUpdate) {
    globalFlagArtistUpdate = false;
    char song[40];
    extractSongName(fullArtistRaw, song, sizeof(song));
    const char* singer = getSingerFromDash(fullArtistRaw);
    if (strcmp(song, songName)) {
      strncpy(songName, song, sizeof(songName)-1); songName[sizeof(songName)-1] = 0;
      strncpy(singerName, singer, sizeof(singerName)-1); singerName[sizeof(singerName)-1] = 0;
      slashToDash(singerName);
      lastLyric[0] = 0; currLyric[0] = 0; pendingLyric[0] = 0;
      lyricLine1[0] = 0; lyricLine2[0] = 0; lastLine1[0] = 0; lastLine2[0] = 0;
      hasPendingLyric = false; lastLyricShowTime = 0;
      flagRefreshAll = true;
    }
  }
  if (btIsConnect && hasPendingLyric && millis() - lastLyricShowTime >= lyricShowMinGap) {
    strncpy(lastLyric, currLyric, sizeof(lastLyric)-1); lastLyric[sizeof(lastLyric)-1] = 0;
    lyricAutoSplit(lastLyric, lastLine1, lastLine2);
    strncpy(currLyric, pendingLyric, sizeof(currLyric)-1); currLyric[sizeof(currLyric)-1] = 0;
    flagRefreshLyric = true;
    lastLyricShowTime = millis();
    hasPendingLyric = false; pendingLyric[0] = 0;
  }
  if (flagRefreshAll) { refreshAllScreen(); flagRefreshAll = false; }
  if (flagRefreshLyric) { refreshOnlyLyric(); flagRefreshLyric = false; }
  static unsigned long anim = 0;
  if (btIsConnect && millis() - anim >= 200) {
    anim = millis();
    spectrumTick();
  }
}

// ============================================================
// 13. TF 音频 + LRC
// ============================================================
void scanAudioFiles() {
  audioFileCount = 0; audioFileSel = 0;
  if (!SD.begin(TF_CS, SPI)) return;
  File root = SD.open(MUSIC_DIR);
  if (!root) return;
  File e;
  while ((e = root.openNextFile()) && audioFileCount < MAX_AUDIO_FILES) {
    if (e.isDirectory()) { e.close(); continue; }
    const char* n = e.name();
    if (n[0] == '/') n++;
    int l = strlen(n);
    if (l > 4) {
      const char* x = n + l - 4;
      if (x[0]=='.' && (x[1]=='m'||x[1]=='M') && (x[2]=='p'||x[2]=='P') && (x[3]=='3')) {
        snprintf(audioFileList[audioFileCount], MAX_FILENAME, "%s/%s", MUSIC_DIR, n);
        audioFileCount++;
      }
    }
    e.close();
  }
  root.close();
}

bool loadLrcFile(const char* path) {
  lrcCount = 0;
  lrcTitle[0] = 0; lrcArtist[0] = 0;
  if (!SD.exists(path)) return false;
  File f = SD.open(path);
  if (!f) return false;
  while (f.available() && lrcCount < MAX_LRC_LINES) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() < 4 || line[0] != '[') continue;
    int close = line.indexOf(']');
    if (close < 0) continue;
    String tag = line.substring(1, close);
    if (tag.startsWith("ti:")) { strncpy(lrcTitle, tag.substring(3).c_str(), 39); continue; }
    if (tag.startsWith("ar:")) { strncpy(lrcArtist, tag.substring(3).c_str(), 39); continue; }
    if (tag.startsWith("al:") || tag.startsWith("by:") || tag.startsWith("offset:")) continue;
    int colon = tag.indexOf(':');
    if (colon < 0) continue;
    int dot = tag.indexOf('.');
    if (dot < 0) dot = tag.indexOf(':');
    if (dot < 0) continue;
    int mn = tag.substring(0, colon).toInt();
    int sec = tag.substring(colon+1, dot).toInt();
    int ms = tag.substring(dot+1).toInt();
    if (tag.length() - dot - 1 == 2) ms *= 10;
    String txt = line.substring(close+1); txt.trim();
    if (!txt.length()) continue;
    lrcLines[lrcCount].timeMs = mn*60000 + sec*1000 + ms;
    strncpy(lrcLines[lrcCount].text, txt.c_str(), MAX_LRC_TEXT-1);
    lrcLines[lrcCount].text[MAX_LRC_TEXT-1] = 0;
    lrcCount++;
  }
  f.close();
  return lrcCount > 0;
}

int getLrcIndex(uint32_t el) {
  int idx = -1;
  for (int i = 0; i < lrcCount; i++) {
    if (lrcLines[i].timeMs <= el) idx = i;
    else break;
  }
  return idx;
}

void playAudioFile(const char* path) {
  initVS1053();   // 先初始化 VS1053（避免 SD 挂载后再重配 SPI）
  ampOn();        // 功放使能
  char lp[80]; strncpy(lp, path, sizeof(lp)-1); lp[sizeof(lp)-1] = 0;
  char* dot = strrchr(lp, '.');
  if (dot) strcpy(dot, ".lrc");
  if (!SD.begin(TF_CS, SPI)) { ampOff(); return; }
  
  
  bool hasLrc = loadLrcFile(lp);
  if (!hasLrc) {
    char lp2[80]; strncpy(lp2, path, sizeof(lp2)-1); lp2[sizeof(lp2)-1] = 0;
    char* d2 = strrchr(lp2, '.'); if (d2) strcpy(d2, ".LRC");
    
    hasLrc = loadLrcFile(lp2);
  }
  File mp3 = SD.open(path);
  if (!mp3) { ampOff(); return; }

  tft.fillScreen(TFT_BLACK);
  if (strlen(lrcTitle)) printChinese(getTextCenterX(lrcTitle), 12, lrcTitle, TFT_YELLOW);
  if (strlen(lrcArtist)) printChinese(getTextCenterX(lrcArtist), 42, lrcArtist, TFT_DARKGREY);
  tft.drawLine(30, 66, SCREEN_WIDTH-30, 66, TFT_DARKGREY);
  if (!hasLrc) printChinese(getTextCenterX("无歌词文件"), 140, "无歌词文件", TFT_DARKGREY);

  player.setVolume(g_audioVol);   // 软件音量
  memset(barH, 0, sizeof(barH));  // 新曲目从 0 开始
  spectrumTick();                 // 进入时显示频谱
  uint8_t buf[32];
  uint32_t startMs = millis();
  int lastIdx = -2;
  while (mp3.available()) {
    esp_task_wdt_reset();   // 播放阻塞 loop，必须喂狗，否则 60s 后看门狗复位
    if (keyScan(PIN_KEY_UP, 0)) {                 // UP：音量 +
      if (g_audioVol < 100) {
        g_audioVol = (g_audioVol + 5 > 100) ? 100 : g_audioVol + 5;
        player.setVolume(g_audioVol);
        showVolumeOverlay();
      }
    }
    if (keyScan(PIN_KEY_OK, 1)) {                 // OK：音量 -
      if (g_audioVol > 5) {
        g_audioVol -= 5;
        player.setVolume(g_audioVol);
        showVolumeOverlay();
      }
    }
    if (keyScan(PIN_KEY_BACK, 2)) {  break; }
    if (hasLrc) {
      int idx = getLrcIndex(millis() - startMs);
      if (idx != lastIdx) {
        lastIdx = idx;
        tft.fillRect(0, 70, SCREEN_WIDTH, 126, TFT_BLACK);
        if (idx >= 0) {
          char l1[64], l2[64];
          lyricAutoSplit(lrcLines[idx].text, l1, l2);
          printChinese(getTextCenterX(l1), 140, l1, TFT_CYAN);
          if (strlen(l2)) printChinese(getTextCenterX(l2), 172, l2, TFT_CYAN);
        }
      }
    }
    spectrumTick();
    if (!player.data_request()) { delay(1); continue; }
    size_t n = mp3.read(buf, sizeof(buf));
    if (!n) break;
    player.playChunk(buf, n);
  }
  mp3.close();
  while (player.data_request()) delay(1);
  delay(200);
  ampOff();       // 播放结束关断功放
}

void drawAudioList() {
  g_listN = audioFileCount > 6 ? 6 : audioFileCount;
  for (int i = 0; i < g_listN; i++) {
    const char* s = strrchr(audioFileList[i], '/');
    strncpy(g_listBuf[i], s ? s + 1 : audioFileList[i], 63);
    g_listBuf[i][63] = 0;
    g_listNames[i] = g_listBuf[i];
  }
  if (!g_listN) {
    tft.fillScreen(TFT_BLACK);
    printChineseOutlined(centerFor(&fonts::efontCN_24_b, "未找到 MP3"), 140, "未找到 MP3", TFT_RED);
    return;
  }
  if (audioFileSel >= g_listN) audioFileSel = 0;
  drawListPage("选择音频", audioFileSel);
}

void scanBooks() {
  bookCount = 0;
  if (!SD.exists(BOOK_DIR)) return;
  File r = SD.open(BOOK_DIR); if (!r) return;
  File e;
  while ((e = r.openNextFile()) && bookCount < MAX_BOOKS) {
    if (e.isDirectory()) { e.close(); continue; }
    const char* n = e.name(); if (n[0] == '/') n++;
    int l = strlen(n);
    if (l > 4) {
      const char* x = n + l - 4;
      if (x[0]=='.' && (x[1]=='t'||x[1]=='T') && (x[2]=='x'||x[2]=='X') && (x[3]=='t'||x[3]=='T'))
        bookFiles[bookCount++] = String(BOOK_DIR) + "/" + n;
    }
    e.close();
  }
  r.close();
}

void drawBookList() {
  g_listN = bookCount > 6 ? 6 : bookCount;
  for (int i = 0; i < g_listN; i++) {
    const char* p = bookFiles[i].c_str();
    const char* s = strrchr(p, '/');
    strncpy(g_listBuf[i], s ? s + 1 : p, 63);
    g_listBuf[i][63] = 0;
    g_listNames[i] = g_listBuf[i];
  }
  if (!g_listN) {
    tft.fillScreen(TFT_BLACK);
    printChineseOutlined(centerFor(&fonts::efontCN_24_b, "/read 无txt"), 140, "/read 无txt", TFT_RED);
    return;
  }
  if (bookSel >= g_listN) bookSel = 0;
  drawListPage("选择书籍", bookSel);
}

// ============================================================
// 14. 时钟 / 天气
// ============================================================
String getWea(int c) {
  if (c == 0) return "晴";
  if (c >= 1 && c <= 3) return "多云";
  if (c == 45 || c == 48) return "雾";
  if (c >= 51 && c <= 55) return "小雨";
  if (c >= 61 && c <= 65) return "雨";
  if (c >= 80 && c <= 82) return "阵雨";
  return "未知";
}

void analysisWeather(String j) {
  if (j.indexOf("current") < 0) { weatherReady = false; return; }
  int s = j.indexOf("current\":{") + 10;
  int e = j.indexOf("}", s);
  if (e <= s) { weatherReady = false; return; }
  String cur = j.substring(s, e);
  int t1 = cur.indexOf("temperature_2m\":");
  int h1 = cur.indexOf("relative_humidity_2m\":");
  int w1 = cur.indexOf("weather_code\":");
  if (t1 < 0 || h1 < 0 || w1 < 0) { weatherReady = false; return; }
  int t2 = cur.indexOf(",", t1); if (t2 < 0) t2 = cur.length();
  int h2 = cur.indexOf(",", h1); if (h2 < 0) h2 = cur.length();
  int w2 = cur.indexOf(",", w1); if (w2 < 0) w2 = cur.length();
  temperature = cur.substring(t1 + 16, t2).toFloat();
  humidity = cur.substring(h1 + 22, h2).toInt();
  weatherCode = cur.substring(w1 + 13, w2).toInt();
  weatherDesc = getWea(weatherCode);
  weatherReady = true;
}

void getWeather() {
  if (WiFi.status() != WL_CONNECTED) return;
  for (int r = 0; r < 2; r++) {
    if (r) { delay(800); esp_task_wdt_reset(); }
    HTTPClient h;
    h.setTimeout(8000);
    h.begin("http://api.open-meteo.com/v1/forecast?latitude=32.23&longitude=111.68&current=temperature_2m,relative_humidity_2m,weather_code&timezone=Asia%2FShanghai");
    int st = h.GET();
    if (st == 200) { analysisWeather(h.getString()); h.end(); return; }
    h.end();
  }
  weatherReady = false;
}

void initWiFiAndGetData() {
  if (dataAvailable && timeInitialized) return;
  stopBluetooth();   // WiFi/NTP 前先停 A2DP，避免内存不足
  WiFi.mode(WIFI_OFF); delay(100); WiFi.mode(WIFI_STA);
  WiFi.begin(wifiName, wifiPwd);
  int a = 0;
  while (WiFi.status() != WL_CONNECTED && a < 20) { delay(500); esp_task_wdt_reset(); a++; }
  if (WiFi.status() == WL_CONNECTED) {
    ntp.begin();
    unsigned long ep = 0;
    const char* servers[] = {"ntp.aliyun.com", "cn.pool.ntp.org", "pool.ntp.org"};
    for (int s = 0; s < 3 && ep == 0; s++) {
      ntp.setPoolServerName(servers[s]);
      for (int i = 0; i < 3 && ep == 0; i++) {
        ntp.forceUpdate();                 // 必须用 forceUpdate，update() 60s 内不会重发
        if (ntp.isTimeSet()) ep = ntp.getEpochTime();
        if (!ep) delay(300);
        esp_task_wdt_reset();
      }
      
    }
    if (ep > 0 && ntp.isTimeSet()) {
      ntpEpoch = ep; lastMillis = millis(); timeInitialized = true;
      time_t t = (time_t)ep;
      struct tm* td = localtime(&t);
      
    } else {
      
    }
    getWeather();
    dataAvailable = true;
    WiFi.disconnect(false); delay(50);
    WiFi.mode(WIFI_OFF);
  } else {
    weatherReady = false; dataAvailable = true;
  }
}

unsigned long getCurrentEpoch() {
  if (!timeInitialized) return 0;
  return ntpEpoch + (millis() - lastMillis) / 1000;
}

// 大时间：按每个字符的实际宽度排版（冒号不再占整格），局刷只重画变化字符
void drawBigTime(const char* ts, bool partial) {
  tft.setFont(&fonts::Font7);
  tft.setTextSize(1.0);

  int baseW = tft.textWidth(ts);          // 5 个字符的真实总宽（含窄冒号）
  float safeW = SCREEN_WIDTH - 30.0f;
  float scale = (baseW > 0) ? safeW / (float)baseW : 1.0f;
  if (scale > 6.0f) scale = 6.0f;
  if (scale < 1.0f) scale = 1.0f;
  tft.setTextSize(scale);

  int w[5];
  int totalW = 0;
  for (int i = 0; i < 5; i++) {
    char c[2] = { ts[i], 0 };
    w[i] = tft.textWidth(c);
    totalW += w[i];
  }
  int x0 = (SCREEN_WIDTH - totalW) / 2;
  int h = tft.fontHeight();
  int y = 150 - h / 2;

  static char prev[6] = "";
  tft.setTextColor(TFT_SKYBLUE);   // 时钟字体：天蓝色
  if (!partial || strlen(prev) != 5) {
    tft.fillRect(x0, y, totalW, h, TFT_BLACK);
    int x = x0;
    for (int i = 0; i < 5; i++) {
      char c[2] = { ts[i], 0 };
      tft.setCursor(x, y);
      tft.print(c);
      x += w[i];
    }
  } else {
    int x = x0;
    for (int i = 0; i < 5; i++) {
      if (ts[i] != prev[i]) {
        tft.fillRect(x, y, w[i], h, TFT_BLACK);
        char c[2] = { ts[i], 0 };
        tft.setCursor(x, y);
        tft.print(c);
      }
      x += w[i];
    }
  }
  strncpy(prev, ts, 5);
  prev[5] = 0;
  tft.setTextSize(1.0);
}

void showDateTimePage() {
  if (!dataAvailable || !timeInitialized) {
    tft.fillScreen(TFT_BLACK);
    printChinese(getTextCenterX("正在获取数据..."), 120, "正在获取数据...", TFT_YELLOW);
    initWiFiAndGetData();
  }
  tft.fillScreen(TFT_BLACK);   // 纯黑底
  unsigned long now = getCurrentEpoch();
  struct tm* td;
  static struct tm def = {0};
  if (now) { time_t t = (time_t)now; td = localtime(&t); } else td = &def;

  // 大时间 HH:MM（居中偏上）
  char ts[6];
  if (now) snprintf(ts, sizeof(ts), "%02d:%02d", td->tm_hour, td->tm_min);
  else snprintf(ts, sizeof(ts), "--:--");
  drawBigTime(ts, false);

  // 下方同一行：日期 星期 天气 温度（统一 24px 字体、统一间距）
  const char* wk[] = {"周日","周一","周二","周三","周四","周五","周六"};
  char d[72];
  if (now) {
    snprintf(d, sizeof(d), "%d月%d日  %s", td->tm_mon+1, td->tm_mday, wk[td->tm_wday]);
    if (weatherReady) {
      char w[28];
      snprintf(w, sizeof(w), "  %s  %.0fC", weatherDesc.c_str(), temperature);
      strncat(d, w, sizeof(d) - strlen(d) - 1);
    }
  } else {
    snprintf(d, sizeof(d), "--月--日  ---");
  }
  printChinese(getTextCenterX(d), 238, d, TFT_WHITE);

  lastTimeStr = String(ts);
  lastTimeUpdate = millis();
}

void updateTimeOnly() {
  unsigned long now = getCurrentEpoch();
  if (!now) return;
  time_t t = (time_t)now;
  struct tm* td = localtime(&t);
  char ts[6];
  snprintf(ts, sizeof(ts), "%02d:%02d", td->tm_hour, td->tm_min);
  if (lastTimeStr.equals(ts)) return;
  drawBigTime(ts, true);   // 只重画变化的数字
  lastTimeStr = String(ts);
}

// ============================================================
// 15. 阅读器（桥接 desk_core）
// ============================================================
;

class TftFont : public desk::Font {
public:
  int glyphWidth(uint32_t cp) const override { return cp < 0x80 ? ENG_CHAR_W : CHN_CHAR_W; }
  int lineHeight() const override { return 28; }
};
TftFont g_font;

class SdDataSource : public desk::DataSource {
public:
  File f;
  SdDataSource(const char* path) { f = SD.open(path); }
  ~SdDataSource() { if (f) f.close(); }
  int read(uint8_t* d, size_t n) override { return f ? (int)f.read(d, n) : 0; }
  bool seek(size_t o) override { return f ? f.seek(o) : false; }
  size_t tell() const override { return f ? f.position() : 0; }
  size_t size() const override { return f ? f.size() : 0; }
};

;

// 游戏用 3 键映射：UP=左转, OK=右转, BACK=退出
;

// 码点 -> UTF-8
int cpToUtf8(uint32_t cp, char* out) {
  if (cp < 0x80) { out[0] = cp; return 1; }
  if (cp < 0x800) { out[0] = 0xC0 | (cp >> 6); out[1] = 0x80 | (cp & 0x3F); return 2; }
  if (cp < 0x10000) { out[0] = 0xE0 | (cp >> 12); out[1] = 0x80 | ((cp >> 6) & 0x3F); out[2] = 0x80 | (cp & 0x3F); return 3; }
  out[0] = 0xF0 | (cp >> 18); out[1] = 0x80 | ((cp >> 12) & 0x3F);
  out[2] = 0x80 | ((cp >> 6) & 0x3F); out[3] = 0x80 | (cp & 0x3F); return 4;
}

static Reader* g_reader = NULL;
static SdDataSource* g_readerSrc = NULL;
static std::vector<desk::DisplayLine> g_page;
static String g_bookPath;

// 目录
#define MAX_CHAPTERS 128
struct ChapterEntry { uint32_t offset; char title[44]; };
static ChapterEntry g_chapters[MAX_CHAPTERS];
static int g_chapterCount = 0;
static int g_chapterSel = 0;
static bool g_chaptersScanned = false;

// 阅读器/目录按键状态（长按检测）
static uint32_t rUpAt = 0, rOkAt = 0, cOkAt = 0;
static bool rUpWas = false, rOkWas = false, rUpLong = false, rOkLong = false;
static bool cOkWas = false, cOkLong = false, cUpWas = false;
static void readerKeyReset() {
  rUpWas = (digitalRead(PIN_KEY_UP) == LOW);
  rOkWas = (digitalRead(PIN_KEY_OK) == LOW);
  rUpLong = rUpWas; rOkLong = rOkWas;
  rUpAt = rOkAt = millis();
  cOkWas = true; cOkLong = true; cOkAt = millis();
}

// 阅读进度（NVS）
static Preferences g_prefs;
static bool g_prefsOpen = false;

// 每本书一个 NVS 键（路径哈希），不同小说各自保存进度
static String progressKey(const String& path) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < path.length(); i++) { h ^= (uint8_t)path[i]; h *= 16777619u; }
  char k[12];
  snprintf(k, sizeof(k), "p%08x", (unsigned)h);
  return String(k);
}

static void saveProgress() {
  if (!g_reader) return;
  if (!g_prefsOpen) { g_prefs.begin("reader", false); g_prefsOpen = true; }
  g_prefs.putUInt(progressKey(g_bookPath).c_str(), (uint32_t)g_reader->position());
}

static uint32_t loadProgressPos(const String& path) {
  if (!g_prefsOpen) { g_prefs.begin("reader", false); g_prefsOpen = true; }
  return g_prefs.getUInt(progressKey(path).c_str(), 0);
}

static int currentChapterIndex() {
  if (!g_reader || g_chapterCount == 0) return 0;
  uint32_t pos = (uint32_t)g_reader->position();
  int idx = 0;
  for (int i = 0; i < g_chapterCount; i++) {
    if (g_chapters[i].offset <= pos) idx = i;
    else break;
  }
  return idx;
}

static int g_savedChapterIdx = -1;

// 只在“进入新章节”时写一次 NVS，减少 Flash 写入
static void maybeSaveChapter() {
  int idx = currentChapterIndex();
  if (idx != g_savedChapterIdx) {
    g_savedChapterIdx = idx;
    saveProgress();
  }
}

void drawReaderPage() {
  tft.fillScreen(TFT_BLACK);
  int y = 20;
  int lh = g_font.lineHeight();
  for (auto& line : g_page) {
    char buf[256] = {0};
    int p = 0;
    for (size_t i = 0; i < line.cp.size(); i++) {
      char tmp[4];
      int n = cpToUtf8(line.cp[i], tmp);
      if (p + n >= (int)sizeof(buf)) break;
      memcpy(buf + p, tmp, n); p += n;
    }
    buf[p] = 0;
    printChinese(TEXT_MARGIN, y, buf, TFT_WHITE);
    y += lh;
    if (y > SCREEN_HEIGHT - 20) break;
  }
}

void readerRender(bool next) {
  if (!g_reader) return;
  bool ok = next ? g_reader->nextPage(SCREEN_WIDTH - 2*TEXT_MARGIN, SCREEN_HEIGHT - 40, g_page)
                 : g_reader->prevPage(SCREEN_WIDTH - 2*TEXT_MARGIN, SCREEN_HEIGHT - 40, g_page);
  if (!ok) { if (next) printChinese(getTextCenterX("已到文末"), 140, "已到文末", TFT_YELLOW); return; }
  drawReaderPage();
  maybeSaveChapter();   // 每章保存一次（进入新章节时）
}

// 原始行字节 -> UTF-8（GBK 时转换，UTF-8 直接拷贝）
static void lineToUtf8(const uint8_t* p, int n, char* out, int outSize, desk::Encoding enc) {
  int o = 0;
  if (n >= 3 && p[0] == 0xEF && p[1] == 0xBB && p[2] == 0xBF) { p += 3; n -= 3; }  // 去 BOM
  if (enc == desk::Encoding::Gbk) {
    for (int i = 0; i + 1 < n && o + 4 < outSize; ) {
      uint8_t b0 = p[i];
      if (b0 < 0x80) { out[o++] = (char)b0; i++; }
      else {
        uint32_t cp = desk::gbkDecode((uint16_t)((b0 << 8) | p[i+1]));
        o += cpToUtf8(cp, out + o);
        i += 2;
      }
    }
  } else {
    for (int i = 0; i < n && o + 1 < outSize; i++) out[o++] = (char)p[i];
  }
  out[o] = 0;
}

// 判断一行是否是章节标题（UTF-8 / GBK 两种编码都支持）
static bool isChapterLine(const uint8_t* p, int n, desk::Encoding enc) {
  if (n < 4 || n > 200) return false;
  int s = 0;
  while (s < n && (p[s] == ' ' || p[s] == '\t')) s++;
  if (s + 3 <= n && p[s] == 0xEF && p[s+1] == 0xBB && p[s+2] == 0xBF) s += 3;  // 去 BOM
  if (s >= n) return false;
  if (enc == desk::Encoding::Gbk) {
    auto starts = [&](const uint8_t* pat, int len) {
      return n - s >= len && memcmp(p + s, pat, len) == 0;
    };
    static const uint8_t di[]   = {0xB5, 0xDA};              // 第
    static const uint8_t zhang[]= {0xD5, 0xC2};              // 章
    static const uint8_t jie[]  = {0xBD, 0xDA};              // 节
    static const uint8_t juan[] = {0xBE, 0xED};              // 卷
    static const uint8_t hui[]  = {0xBB, 0xD8};              // 回
    static const uint8_t xu[]   = {0xD0, 0xF2, 0xD5, 0xC2};  // 序章
    if (starts(xu, 4)) return true;
    if (starts(di, 2)) {
      for (int i = s + 2; i + 1 < n && i < s + 40; i++) {
        if (memcmp(p + i, zhang, 2) == 0 || memcmp(p + i, jie, 2) == 0 ||
            memcmp(p + i, juan, 2) == 0 || memcmp(p + i, hui, 2) == 0) return true;
      }
    }
    return false;
  }
  auto starts = [&](const char* pat) {
    int len = strlen(pat);
    return n - s >= len && memcmp(p + s, pat, len) == 0;
  };
  if (starts("序章") || starts("楔子") || starts("前言") || starts("后记") || starts("番外") ||
      starts("Chapter") || starts("CHAPTER")) return true;
  if (starts("第")) {
    const char* keys[] = {"章", "节", "卷", "回"};
    for (int i = s + 3; i < n && i < s + 60; i++) {
      for (auto k : keys) {
        int kl = strlen(k);
        if (i + kl <= n && memcmp(p + i, k, kl) == 0) return true;
      }
    }
  }
  return false;
}

void scanChapters() {
  g_chapterCount = 0;
  g_chaptersScanned = true;
  if (!g_reader) return;
  desk::Encoding enc = g_reader->encoding();
  File f = SD.open(g_bookPath.c_str());
  if (!f) return;
  uint8_t buf[512];
  uint8_t line[220];
  int ln = 0;
  uint32_t absOff = 0;
  uint32_t lineOff = 0;
  while (true) {
    int r = f.read(buf, sizeof(buf));
    if (r <= 0) break;
    for (int i = 0; i < r; i++) {
      uint8_t c = buf[i];
      if (c == '\n') {
        if (isChapterLine(line, ln, enc) && g_chapterCount < MAX_CHAPTERS) {
          g_chapters[g_chapterCount].offset = lineOff;
          lineToUtf8(line, ln, g_chapters[g_chapterCount].title,
                     sizeof(g_chapters[g_chapterCount].title), enc);
          g_chapterCount++;
        }
        ln = 0;
        lineOff = absOff + i + 1;
      } else if (c != '\r' && ln < (int)sizeof(line)) {
        line[ln++] = c;
      }
    }
    absOff += r;
    esp_task_wdt_reset();   // 大文件扫描时喂狗
  }
  if (ln > 0 && isChapterLine(line, ln, enc) && g_chapterCount < MAX_CHAPTERS) {
    g_chapters[g_chapterCount].offset = lineOff;
    lineToUtf8(line, ln, g_chapters[g_chapterCount].title,
               sizeof(g_chapters[g_chapterCount].title), enc);
    g_chapterCount++;
  }
  f.close();
  
}

static int g_chapterTop = 0;

static void drawChapterRow(int i) {
  int y = 80 + (i - g_chapterTop) * 40;
  bool sel = (i == g_chapterSel);
  tft.fillRect(0, y - 4, SCREEN_WIDTH, 34, sel ? TFT_BLUE : TFT_BLACK);
  printChinese(16, y, g_chapters[i].title, sel ? TFT_WHITE : 0xC618);
}

void drawChapterList(bool full = true, int oldSel = -1) {
  int top = g_chapterSel - 2;
  if (top < 0) top = 0;
  if (top > g_chapterCount - 5) top = g_chapterCount - 5;
  if (top < 0) top = 0;

  if (full) {
    tft.fillScreen(TFT_BLACK);
    char hdr[40];
    snprintf(hdr, sizeof(hdr), "目录 (%d)", g_chapterCount);
    printChinese(centerFor(&fonts::efontCN_24_b, hdr), 18, hdr, TFT_CYAN);
    if (g_chapterCount == 0) {
      printChinese(getTextCenterX("未识别到章节"), 140, "未识别到章节", TFT_YELLOW);
      printChinese16(getTextCenterX("按BACK返回"), 200, "按BACK返回", TFT_DARKGREY);
      g_chapterTop = 0;
      return;
    }
    printChinese16(getTextCenterX("UP下选 OK上选 长按OK跳转"), 300, "UP下选 OK上选 长按OK跳转", TFT_DARKGREY);
    g_chapterTop = top;
    for (int i = top; i < g_chapterCount && i < top + 5; i++) drawChapterRow(i);
    return;
  }

  if (g_chapterCount == 0) return;
  if (top != g_chapterTop) {           // 窗口滚动：只重画 5 行，不清全屏
    g_chapterTop = top;
    tft.fillRect(0, 72, SCREEN_WIDTH, 5 * 40 + 8, TFT_BLACK);
    for (int i = top; i < g_chapterCount && i < top + 5; i++) drawChapterRow(i);
  } else {                              // 只重画旧/新选中两行
    if (oldSel >= 0 && oldSel != g_chapterSel) drawChapterRow(oldSel);
    drawChapterRow(g_chapterSel);
  }
}

void readerEnter(const char* path) {
  SD.begin(TF_CS, SPI);
  g_bookPath = path;
  g_chaptersScanned = false;
  g_chapterCount = 0;
  g_chapterSel = 0;
  if (g_reader) { delete g_reader; g_reader = NULL; }
  if (g_readerSrc) { delete g_readerSrc; g_readerSrc = NULL; }
  g_readerSrc = new SdDataSource(path);
  g_reader = new Reader(g_font);
  if (!g_reader->open(g_readerSrc)) {
    printChineseOutlined(centerFor(&fonts::efontCN_24_b, "打开失败"), 140, "打开失败", TFT_RED);
    return;
  }
  uint32_t pos = loadProgressPos(g_bookPath);   // 恢复上次阅读位置
  if (pos > 0) g_reader->seekTo(pos);
  g_savedChapterIdx = currentChapterIndex();    // 避免打开时立刻写 NVS
  tft.fillScreen(TFT_BLACK);
  readerKeyReset();
  readerRender(true);
}

// 游戏
// 游戏：猜数字（回合制，按键才刷新，不要求快速刷新）
static int gnum_secret = 50, gnum_guess = 50, gnum_tries = 0;
static bool gnum_won = false;
static bool gnum_showResult = false;   // 仅在长按 BACK 比较后显示结果

void drawGuessGame() {
  tft.fillScreen(TFT_BLACK);
  printChinese(centerFor(&fonts::efontCN_24_b, "猜数字 1-99"), 18, "猜数字 1-99", TFT_CYAN);
  char b[8]; snprintf(b, sizeof(b), "%d", gnum_guess);
  tft.setFont(&fonts::Font7);
  tft.setTextSize(2);                       // 原来是 4，缩小一半，大小更合适
  tft.setTextColor(TFT_YELLOW);
  int x = (SCREEN_WIDTH - tft.textWidth(b)) / 2;
  tft.setCursor(x, 80); tft.print(b);
  tft.setTextSize(1.0);
  const char* msg = "";
  uint16_t mc = TFT_WHITE;
  if (gnum_showResult) {
    if (gnum_won) { msg = "猜对了！长按BACK再来"; mc = TFT_GREEN; }
    else if (gnum_guess > gnum_secret) { msg = "大了"; mc = TFT_RED; }
    else { msg = "小了"; mc = TFT_BLUE; }
  }
  if (msg[0]) printChinese(centerFor(&fonts::efontCN_24_b, msg), 200, msg, mc);
  char t[32]; snprintf(t, sizeof(t), "次数: %d", gnum_tries);
  printChinese16(centerFor(&fonts::efontCN_16_b, t), 255, t, TFT_DARKGREY);
  printChinese16(centerFor(&fonts::efontCN_16_b, "UP+1  OK-1  长按BACK比较"), 285,
                 "UP+1  OK-1  长按BACK比较", TFT_DARKGREY);
}

static bool gUpWas = false, gOkWas = false, gOkLong = false;
static bool gBackWas = false, gBackLong = false;
static uint32_t gOkAt = 0, gBackAt = 0;

void gameEnter() {
  srand(millis());
  gnum_secret = 1 + (rand() % 99);
  gnum_guess = 50; gnum_tries = 0; gnum_won = false;
  gnum_showResult = false;
  gUpWas = (digitalRead(PIN_KEY_UP) == LOW);
  gOkWas = (digitalRead(PIN_KEY_OK) == LOW);
  gOkLong = gOkWas; gOkAt = millis();
  gBackWas = (digitalRead(PIN_KEY_BACK) == LOW);
  gBackLong = gBackWas; gBackAt = millis();
  drawGuessGame();
}
bool gameLoop() {
  bool up = (digitalRead(PIN_KEY_UP) == LOW);
  bool ok = (digitalRead(PIN_KEY_OK) == LOW);
  bool back = (digitalRead(PIN_KEY_BACK) == LOW);

  if (!up && gUpWas) {                     // 短按 UP 释放：+1
    gnum_guess = (gnum_guess % 99) + 1;
    gnum_won = false;
    gnum_showResult = false;               // 调数字时隐藏上次结果
    drawGuessGame();
  }
  gUpWas = up;

  if (ok && !gOkWas) { gOkAt = millis(); gOkLong = false; }
  if (ok && !gOkLong && millis() - gOkAt >= 700) gOkLong = true;
  if (!ok && gOkWas && !gOkLong) {         // 短按 OK 释放：-1
    gnum_guess = ((gnum_guess - 2 + 99) % 99) + 1;
    gnum_won = false;
    gnum_showResult = false;               // 调数字时隐藏上次结果
    drawGuessGame();
  }
  gOkWas = ok;

  if (back && !gBackWas) { gBackAt = millis(); gBackLong = false; }
  if (back && !gBackLong && millis() - gBackAt >= 700) {   // 长按 BACK：比较大小
    gBackLong = true;
    if (gnum_won) { gameEnter(); }
    else {
      gnum_tries++;
      if (gnum_guess == gnum_secret) gnum_won = true;
      gnum_showResult = true;              // 只有长按比较才显示结果
      drawGuessGame();
    }
  }
  if (!back && gBackWas && !gBackLong) return true;        // 短按 BACK：退出
  gBackWas = back;

  return false;
}

// ============================================================
// 16. 菜单渲染
// ============================================================
void drawMenu() {
  const char* title = NULL;
  const char** items = NULL;
  int cnt = 0, sel = 0;
  if (nowPage == PAGE_MAIN) {
    title = "系统主菜单"; items = main_menu; cnt = MAIN_CNT; sel = main_sel;
  } else if (nowPage == PAGE_EINK_MENU) {
    title = "墨水屏设置"; items = eink_menu; cnt = EINK_CNT; sel = sub_sel;
  } else if (nowPage == PAGE_AUDIO_MENU) {
    title = "歌曲播放"; items = audio_menu; cnt = AUDIO_CNT; sel = sub_sel;
  } else return;

  g_menuItems = items; g_menuCount = cnt;
  g_currentSel = sel; g_startY = 72; g_itemSpacing = 38;   // 6 项放得下

  if (wallBuf || wallFile) drawWallpaperRegion(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
  else tft.fillScreen(TFT_BLACK);

  // 标题：青色描边，居中
  printChineseOutlined(centerFor(&fonts::efontCN_24_b, title), 18, title, TFT_CYAN);

  // 菜单项：选中红色、未选中白色，描边、居中；选中的左侧画 ">"
  for (int i = 0; i < cnt; i++) {
    int y = g_startY + i * g_itemSpacing;
    uint16_t c = (i == sel) ? TFT_RED : TFT_WHITE;
    printChineseOutlined(centerFor(&fonts::efontCN_24_b, items[i]), y, items[i], c);
    if (i == sel) printChineseOutlined(40, y, ">", TFT_RED);
  }
}

// 只重画一行（局部刷新，避免整屏刷）
void menuRedrawRow(int i, bool sel) {
  if (!g_menuItems || i < 0 || i >= g_menuCount) return;
  int y = g_startY + i * g_itemSpacing;
  drawWallpaperRegion(0, y - 4, SCREEN_WIDTH, 34);   // 恢复该行壁纸
  uint16_t c = sel ? TFT_RED : TFT_WHITE;
  printChineseOutlined(centerFor(&fonts::efontCN_24_b, g_menuItems[i]), y, g_menuItems[i], c);
  if (sel) printChineseOutlined(40, y, ">", TFT_RED);
}

void drawMenuSelect(int oldSel, int newSel) {
  menuRedrawRow(oldSel, false);
  menuRedrawRow(newSel, true);
  g_currentSel = newSel;
}

// 待机页：只铺壁纸，按 OK 进菜单（无文字提示）
void drawStandby() {
  if (wallBuf) { drawWallpaperRegion(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT); return; }
  // 无 PSRAM 缓存时：每次重新打开壁纸文件，避免旧句柄失效导致崩溃
  if (wallFile) wallFile.close();
  if (wallpaperCount > 0) loadWallpaper(wallpaperFiles[currentWallpaperIdx].c_str());
  if (wallFile) drawWallpaperRegion(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
  else tft.fillScreen(TFT_BLACK);
}

// ============================================================
// 17. 按键
// ============================================================
static unsigned long lastDebounce[3] = {0};
static bool pressed[3] = {false, false, false};

bool keyScan(int pin, int idx, bool edge) {
  bool now = (digitalRead(pin) == LOW);      // 按下 = LOW
  bool hit = false;
  if (now && !pressed[idx] && (millis() - lastDebounce[idx] > 50)) {
    lastDebounce[idx] = millis();
    hit = true;
  }
  pressed[idx] = now;                        // 记录上一状态，松开后自动复位
  return hit;
}

// ============================================================
// 18. 按键逻辑
// ============================================================
void keyLogicTask() {
  // 待机：只显示壁纸，按 OK 进菜单
  if (nowPage == PAGE_STANDBY) {
    if (keyScan(PIN_KEY_OK, 1)) { nowPage = PAGE_MAIN; main_sel = 0; drawMenu(); }
    return;
  }
  // 阅读器：短按 UP 下一页 / OK 上一页，长按 UP 进入目录
  if (nowPage == PAGE_READER) {
    bool up = (digitalRead(PIN_KEY_UP) == LOW);
    bool ok = (digitalRead(PIN_KEY_OK) == LOW);
    if (up && !rUpWas) { rUpAt = millis(); rUpLong = false; }
    if (up && !rUpLong && millis() - rUpAt >= 700) {
      rUpLong = true;
      if (!g_chaptersScanned) { drawTFTMessage("扫描目录..."); scanChapters(); }
      g_chapterSel = currentChapterIndex();   // 从当前章节开始，而不是第 0 章
      cOkWas = true; cOkLong = true; cOkAt = millis();
      cUpWas = true;                       // UP 仍按住，避免进入目录瞬间误触发
      drawChapterList();
      nowPage = PAGE_CHAPTERS;
      rUpWas = up; rOkWas = ok;
      return;
    }
    if (!up && rUpWas && !rUpLong) readerRender(true);   // 短按 UP：下一页
    rUpWas = up;
    if (ok && !rOkWas) { rOkAt = millis(); rOkLong = false; }
    if (ok && !rOkLong && millis() - rOkAt >= 700) rOkLong = true;
    if (!ok && rOkWas && !rOkLong) readerRender(false);  // 短按 OK：上一页
    rOkWas = ok;
    if (keyScan(PIN_KEY_BACK, 2)) { saveProgress(); nowPage = PAGE_MAIN; main_sel = 4; drawMenu(); }
    return;
  }
  // 章节目录：UP 选择，长按 OK 跳转，BACK 返回
  if (nowPage == PAGE_CHAPTERS) {
    bool tup = (digitalRead(PIN_KEY_UP) == LOW);
    if (tup && !cUpWas) {
      if (g_chapterCount > 0) {
        int old = g_chapterSel;
        g_chapterSel = (g_chapterSel + 1) % g_chapterCount;
        drawChapterList(false, old);   // 差量刷新，不清整页
      }
    }
    cUpWas = tup;
    bool cok = (digitalRead(PIN_KEY_OK) == LOW);
    if (cok && !cOkWas) { cOkAt = millis(); cOkLong = false; }
    if (cok && !cOkLong && millis() - cOkAt >= 700) {
      cOkLong = true;
      if (g_chapterCount > 0 && g_reader) {
        g_reader->seekTo(g_chapters[g_chapterSel].offset);
        readerRender(true);
        readerKeyReset();
        nowPage = PAGE_READER;
      }
    }
    if (!cok && cOkWas && !cOkLong && g_chapterCount > 0) {   // 短按 OK：向上选择
      int old = g_chapterSel;
      g_chapterSel = (g_chapterSel - 1 + g_chapterCount) % g_chapterCount;
      drawChapterList(false, old);
    }
    cOkWas = cok;
    if (keyScan(PIN_KEY_BACK, 2)) {
      readerKeyReset();
      nowPage = PAGE_READER;
      drawReaderPage();
    }
    return;
  }
  // 书籍选择
  if (nowPage == PAGE_BOOK_LIST) {
    if (keyScan(PIN_KEY_UP, 0) && g_listN > 0) {
      int old = bookSel;
      bookSel = (bookSel + 1) % g_listN;
      listRedrawRow(old, false);
      listRedrawRow(bookSel, true);
    }
    if (keyScan(PIN_KEY_OK, 1) && bookCount > 0) {
      readerEnter(bookFiles[bookSel].c_str());
      nowPage = PAGE_READER;
    }
    if (keyScan(PIN_KEY_BACK, 2)) { nowPage = PAGE_MAIN; main_sel = 4; drawMenu(); }
    return;
  }
  // 游戏
  if (nowPage == PAGE_GAME) {
    if (gameLoop()) { nowPage = PAGE_MAIN; main_sel = 5; drawMenu(); }
    return;
  }
  // 时钟
  if (nowPage == PAGE_CLOCK) {
    if (keyScan(PIN_KEY_BACK, 2)) { nowPage = PAGE_MAIN; main_sel = 2; drawMenu(); return; }
    if (keyScan(PIN_KEY_OK, 1)) {
      dataAvailable = false; weatherReady = false;
      tft.fillScreen(TFT_BLACK);
      printChinese(getTextCenterX("正在刷新..."), 120, "正在刷新...", TFT_YELLOW);
      initWiFiAndGetData(); showDateTimePage();
    }
    return;
  }
  // TFT 壁纸选择
  if (nowPage == PAGE_WALLPAPER_SELECT) {
    if (keyScan(PIN_KEY_UP, 0) && g_listN > 0) {
      int old = selectWallpaperIdx;
      selectWallpaperIdx = (selectWallpaperIdx + 1) % g_listN;
      listRedrawRow(old, false);
      listRedrawRow(selectWallpaperIdx, true);
    }
    if (keyScan(PIN_KEY_OK, 1) && wallpaperCount > 0) {
      currentWallpaperIdx = selectWallpaperIdx;
      // 优先更新 PSRAM 缓存（壁纸绘制不再依赖 SD 文件句柄）
      if (wallBuf) loadWallpaperToPSRAM(wallpaperFiles[currentWallpaperIdx].c_str());
      else loadWallpaper(wallpaperFiles[currentWallpaperIdx].c_str());
      
      nowPage = PAGE_MAIN; main_sel = 3; drawMenu();
    }
    if (keyScan(PIN_KEY_BACK, 2)) { nowPage = PAGE_MAIN; main_sel = 3; drawMenu(); }
    return;
  }
  // 蓝牙歌词
  if (nowPage == PAGE_BT_LYRIC) {
    if (keyScan(PIN_KEY_UP, 0)) {                 // UP：音量 +
      if (g_audioVol < 100) {
        g_audioVol = (g_audioVol + 5 > 100) ? 100 : g_audioVol + 5;
        player.setVolume(g_audioVol);
        showVolumeOverlay();
      }
    }
    if (keyScan(PIN_KEY_OK, 1)) {                 // OK：音量 -
      if (g_audioVol > 5) {
        g_audioVol -= 5;
        player.setVolume(g_audioVol);
        showVolumeOverlay();
      }
    }
    if (keyScan(PIN_KEY_BACK, 2)) {
      stopBluetooth();
      nowPage = PAGE_AUDIO_MENU; sub_sel = 0; drawMenu();
    }
    return;
  }
  // BLE 传图
  if (nowPage == PAGE_EPD_WAIT_BLE) {
    if (dataReady) {
      dataReady = false;
      drawTFTMessage("收到图片，刷新中...");
      showImageOnEPD();          // BLE 保持开启（与可运行的传图代码一致）
      bytesReceived = 0;
      if (bleRxBuffer) memset(bleRxBuffer, 0xFF, IMG_SIZE);
      tft.fillScreen(TFT_BLACK);
      printChinese(getTextCenterX("已刷新，可继续传图"), 140, "已刷新，可继续传图", TFT_GREEN);
    }
    if (keyScan(PIN_KEY_BACK, 2)) {
      stopBLE();
      nowPage = PAGE_EINK_MENU; sub_sel = 1; drawMenu();
    }
    return;
  }
  // TF 音频列表
  if (nowPage == PAGE_TF_LIST) {
    if (keyScan(PIN_KEY_UP, 0) && g_listN > 0) {
      int old = audioFileSel;
      audioFileSel = (audioFileSel + 1) % g_listN;
      listRedrawRow(old, false);
      listRedrawRow(audioFileSel, true);
    }
    if (keyScan(PIN_KEY_OK, 1) && audioFileCount > 0) {
      playAudioFile(audioFileList[audioFileSel]);
      drawAudioList();
    }
    if (keyScan(PIN_KEY_BACK, 2)) { nowPage = PAGE_AUDIO_MENU; sub_sel = 1; drawMenu(); }
    return;
  }
  // 菜单（主/子）——只重画变化的两行，避免整屏刷
  if (keyScan(PIN_KEY_UP, 0)) {
    
    int old = g_currentSel;
    if (nowPage == PAGE_MAIN) main_sel = (main_sel + 1) % MAIN_CNT;
    else if (nowPage == PAGE_EINK_MENU) sub_sel = (sub_sel + 1) % EINK_CNT;
    else if (nowPage == PAGE_AUDIO_MENU) sub_sel = (sub_sel + 1) % AUDIO_CNT;
    int neu = (nowPage == PAGE_MAIN) ? main_sel : sub_sel;
    drawMenuSelect(old, neu);
    return;
  }
  if (keyScan(PIN_KEY_OK, 1)) {
    
    if (nowPage == PAGE_MAIN) {
      sub_sel = 0;
      switch (main_sel) {
        case 0: nowPage = PAGE_EINK_MENU; drawMenu(); break;
        case 1: nowPage = PAGE_AUDIO_MENU; drawMenu(); break;
        case 2: nowPage = PAGE_CLOCK; showDateTimePage(); break;
        case 3: loadWallpaperList(); selectWallpaperIdx = currentWallpaperIdx; nowPage = PAGE_WALLPAPER_SELECT; drawWallpaperSelect(); break;
        case 4: scanBooks(); bookSel = 0; nowPage = PAGE_BOOK_LIST; drawBookList(); break;
        case 5: gameEnter(); nowPage = PAGE_GAME; break;
      }
    } else if (nowPage == PAGE_EINK_MENU) {
      if (sub_sel == 0) {
        tft.fillScreen(TFT_BLACK);
        printChinese(getTextCenterX("正在刷新墨水屏..."), 150, "正在刷新墨水屏...", TFT_YELLOW);
        showCalendarOnEPD();
        drawMenu();
      } else if (sub_sel == 1) {
        nowPage = PAGE_EPD_WAIT_BLE;
        drawTFTMessage("蓝牙已开启，等待传图...");
        startBLE();
      } else {
        nowPage = PAGE_MAIN; main_sel = 0; drawMenu();
      }
    } else if (nowPage == PAGE_AUDIO_MENU) {
      if (sub_sel == 0) {
        nowPage = PAGE_BT_LYRIC; waitingForBT = true; flagRefreshAll = true;
        tft.fillScreen(TFT_BLACK);
        printChinese(getTextCenterX("蓝牙已开启，请连接..."), 140, "蓝牙已开启，请连接...", TFT_YELLOW);
        startBluetooth();
      } else if (sub_sel == 1) {
        tft.fillScreen(TFT_BLACK);
        printChinese(getTextCenterX("正在扫描TF卡..."), 150, "正在扫描TF卡...", TFT_YELLOW);
        scanAudioFiles();
        nowPage = PAGE_TF_LIST;
        drawAudioList();
      } else {
        nowPage = PAGE_MAIN; main_sel = 1; drawMenu();
      }
    }
    return;
  }
  if (keyScan(PIN_KEY_BACK, 2)) {
    
    if (nowPage == PAGE_EINK_MENU) { nowPage = PAGE_MAIN; main_sel = 0; drawMenu(); }
    else if (nowPage == PAGE_AUDIO_MENU) { nowPage = PAGE_MAIN; main_sel = 1; drawMenu(); }
  }
}

// ============================================================
// 19. setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(800);

  

  // UP=IO39 / OK=IO36 仅输入，需外部 10K 上拉；BACK=IO2 内部上拉 + 外部 10K
  pinMode(PIN_KEY_UP, INPUT);          // IO39 仅输入，外部 10K
  pinMode(PIN_KEY_OK, INPUT);          // IO36 仅输入，外部 10K
  pinMode(PIN_KEY_BACK, INPUT_PULLUP); // IO2 有内部上拉，外部 10K
  pinMode(PIN_AMP_SHDN, OUTPUT);
  digitalWrite(PIN_AMP_SHDN, HIGH);    // 上电默认关断功放（无 pop）
  delay(50);
  

  // 1) 先挂 SD（共享 SPI 只在这里 begin 一次）
  
  initSD();
  scanWallpapers();
  if (wallpaperCount > 0) loadWallpaper(wallpaperFiles[currentWallpaperIdx].c_str());
  

  // 蓝牙启动前，把壁纸整张读进 PSRAM（之后画壁纸不再碰 SD，避开 PSRAM+BT+SD 崩溃）
  wallBuf = (uint16_t*)ps_malloc((size_t)SCREEN_WIDTH * SCREEN_HEIGHT * 2);
  if (wallBuf && wallpaperCount > 0)
    loadWallpaperToPSRAM(wallpaperFiles[currentWallpaperIdx].c_str());
  

  // 2) 蓝牙改为按需启动（进「蓝牙歌词」时再 startBluetooth），开机先不占 ~93KB
  

  // 3) TFT
  
  tft.init();
  tft.setRotation(7);   // 在 rotation=5 的基础上再转 180°（5→7）
  tft.fillScreen(TFT_BLACK);
  

  // 4) 墨水屏（复用已初始化的 SPI 总线）
  // 墨水屏不在启动时初始化：省下约 30KB 缓冲给蓝牙；用到时再 init
  

  
  esp_task_wdt_init(60, false);
  esp_task_wdt_add(NULL);

  
  
  nowPage = PAGE_STANDBY;
  main_sel = 0;
  drawStandby();
  
     // VS1053 延迟到放音时再初始化，避免开机卡住
}

void loop() {
  esp_task_wdt_reset();


  forwardPcmToVS1053();
  keyLogicTask();

  if (nowPage == PAGE_BT_LYRIC && btIsConnect) btLyricsTick();
  if (nowPage == PAGE_CLOCK && millis() - lastTimeUpdate >= 1000) {
    lastTimeUpdate = millis();
    updateTimeOnly();
  }
  delay(1);
}
