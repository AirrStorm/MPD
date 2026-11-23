// ESP32 MP3 player with OLED UI  -  v31 (Portal FX + phone RECEIVER mode)
//
// v31: SCREEN SAVER. After 5 minutes (default) without a key press or encoder turn the OLED turns off (saves battery); any
//   key or the encoder turns it back on. The key that wakes the screen does nothing else. Music keeps playing.
//   The time is set in the output menu (hold ENTER): SCREEN item -> ENTER, UP/DOWN = +/- 1 minute (0 = never),
//   ENTER = save. It is remembered across power cycles. The current song, playlist and shuffle are also remembered
//   and restored at power-on (saved about 1.5 s after a song starts).
//
// v30 FINAL: speaker removed, 5 new animations (see below).
//
// v30: final hardware. The speaker (MAX98357A amp) is gone, so the SPEAKER output mode, the amp pins (14 / 15),
//   the amp shutdown control and the mono mix for the amp were removed. The output menu is now:
//   EARPIECE, BLUETOOTH, RECEIVER / SD CARD, CANCEL. Pins were checked against the hardware test sketches
//   (SD, DAC, encoder, buttons): all unchanged.
//   - 5 new song-change animations (equalizer, spinning record, sonar, warp stars, heartbeat) on top of the 5
//     Portal scenes; one plays on about half of the song changes.
//   - While Bluetooth streams, animation frames are drawn every 40 ms instead of 30 ms (a little more CPU for the
//     audio side).
//
// v29: choppy Bluetooth speaker audio = Bluetooth short of heap// v29: choppy Bluetooth speaker audio = Bluetooth short of heap (speaker asked ~167x/s instead of ~345x/s, heap ~14 KB)
//   - BT output: PCM buffer 10 -> 8 KB, audio task stack 6 -> 4.5 KB, loop() stack 8 -> 6 KB (all measured with room to spare).
//   - New "BT OUT" status line every 5 s (requests/s, underruns, lowest buffer fill, heap).
//
// v28: restart speed, Bluetooth scanner, random resets, receiver diagnostics
//   - "SD card mode" (earpiece) restart played the FULL splash animation (~8 s): the restart request was saved as 0,
//     the same value as a normal power-on. A separate "quick restart" bit (4) is now saved, so the restart shows
//     only "SD card mode" and skips the splash.
//   - Boot does fewer slow SD passes (every openNextFile() opens the file): names + song count in ONE pass, and
//     the playlist list (names + counts) is cached in flash for quick restarts. A power-on boot always rescans.
//   - Bluetooth speaker scanner: the A2DP source / sink objects are now created only when needed (only one role
//     exists per boot), "Rescan" really restarts discovery, and discovery restarts by itself after 25 s without a
//     device. Serial prints "SCAN: ..." and every device that is found.
//   - Random resets: Serial prints the reset reason at boot (BROWNOUT = power problem, PANIC = crash, ...),
//     a STAT line every 5 s shows heap, lowest heap ever and the free stack of each task. i2s task stack 4->6 KB,
//     audio task stack 6->8 KB when Bluetooth is not the output.
//   - Receiver: a second RX line shows how many bytes per second the phone really delivers (need ~176400 at
//     44.1 kHz), the longest gap between Bluetooth packets and the lowest buffer fill. Title polling 1.5 s -> 3 s.
//
// v27: receiver-mode fixes
//   - CHOPPY / speeding up / slowing down: initAudio() reset the I2S sample rate to 44100 Hz AFTER the phone had
//     already announced its real rate. A phone streaming at 48000 Hz was therefore played at 44100 Hz (slow,
//     buffer overflowing, chunks dropped = choppy). The rate announced by the phone is now remembered (rxRate)
//     and used to start I2S.
//   - Receiver buffer: up to 32 KB, prefill up to 16 KB (more cushion for Bluetooth jitter). If the buffer gets
//     more than 80% full, a few frames are discarded inside the I2S task instead of making the Bluetooth
//     task wait (waiting there stalls the Bluetooth stack and causes more stutter).
//   - Song name did not follow the phone: the title is now re-requested from the phone every 1.5 s
//     (and 0.5 s after UP / DOWN) in addition to the phone's own track-change notification.
//   - SD card mode boots faster: the song fingerprints are built from the name cache (one SD folder
//     pass less), the restart delay is shorter, and Serial prints how long each boot step took.
//
// v26: SD card <-> RECEIVER mode now RESTARTS the ESP32 (about 2 s, no long splash). Live switching left the
//   SD player stuck on "Loading..." and the receiver audio choppy, because the Bluetooth stack needs a clean
//   heap. The song / playlist / shuffle are saved before the restart, so SD mode comes back on the same song.
//   Receiver mode now boots WITHOUT the SD card, builds its audio buffer only after the phone has connected
//   and started playing, waits for room instead of dropping audio, and prints a status line to Serial.
//
// v25: RECEIVER MODE. The ESP32 becomes a Bluetooth speaker-sink called "Music Playing Doohickey".
//   Pair your phone with it (it appears as "Music Playing Doohickey"), then whatever the phone plays comes out of the earpiece (DAC).
//   In the output menu:
//     RECEIVER MODE  = switch the input from the SD card to the phone
//     SD CARD MODE   = (shown while in receiver mode) go back to the SD card, same song as before
//   In receiver mode UP / DOWN / ENTER remote-control the phone (next / previous / play-pause) and
//   the title and artist from the phone are shown. The output stays on the earpiece.
//   Receiver -> Bluetooth speaker is NOT possible (the ESP32 can only be an A2DP sink OR source at a
//   time), so BLUETOOTH MODE asks you to go back to SD card mode first.
//
// Pins: UP=25, DOWN=26, ENTER=27 | SD CS=5 | OLED SDA=21, SCL=22
//       I2S DAC (UDA1334A): BCLK=16, WS/LRC=17, DIN=4
//       Rotary encoder: CLK=32, DT=33, SW=13
//
// Controls
//   Player  : UP = next song, DOWN = previous song, ENTER = pause/play,
//             hold ENTER (0.8 s) = open the output menu
//             encoder turn = volume, encoder click = shuffle on/off,
//             HOLD the encoder click (0.8 s) = open the PLAYLIST menu
//   Playlist menu : UP/DOWN = move, ENTER = play that playlist, hold ENTER = cancel
//   Output menu   : UP/DOWN = move, ENTER = select, hold ENTER = cancel
//             (Earpiece = wired headphones, Bluetooth = scanner, Receiver / SD card = switch input)
//   Scanner : hold ENTER = open the output menu

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"
#include "BluetoothA2DPSource.h"
#include "BluetoothA2DPSink.h"
#include "esp_bt.h"
#include "esp_system.h"
#include <Preferences.h>
#include <ESP32Encoder.h>

using namespace audio_tools;

// ---------------- OLED ----------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- Pins ----------------
#define PIN_UP    25
#define PIN_DOWN  26
#define PIN_ENTER 27
const int SD_CS_PIN = 5;

// ---------------- PCM buffer (decoder -> Bluetooth) ----------------
#define TEST_TONE 0   // 0 = normal MP3 playback, 1 = tone only, 2 = tone to Bluetooth while the MP3 pipeline runs
int16_t toneTable[100];
volatile int32_t lastFrameCount = 0;

#define PCM_MIN_SIZE     (6 * 1024)
#define PCM_MAX_SIZE     (8 * 1024)    // Bluetooth output: smaller buffer = more free heap for the BT stack (was 10 KB)
#define PCM_HEAP_RESERVE (13 * 1024)
volatile size_t pcmPrefill = 8 * 1024;
volatile size_t pcmSize    = 0;
volatile uint32_t underruns = 0;
volatile uint32_t minFill   = 0xFFFFFFFF;
volatile uint32_t maxCopyMs = 0;
StreamBufferHandle_t pcmBuf = nullptr;
volatile uint32_t pcmWritten = 0;
volatile uint32_t pcmDropped = 0;

class PcmSink : public Print {
 public:
  size_t write(uint8_t b) override { return write(&b, 1); }
  size_t write(const uint8_t *data, size_t len) override {
    if (!pcmBuf || len == 0) return len;
    size_t sent = xStreamBufferSend(pcmBuf, data, len, pdMS_TO_TICKS(250));
    pcmWritten += sent;
    if (sent < len) pcmDropped += (len - sent);
    return len;
  }
  int availableForWrite() override {
    return pcmBuf ? (int)xStreamBufferSpacesAvailable(pcmBuf) : 0;
  }
};

// ---------------- Audio pipeline ----------------
PcmSink pcmSink;
MP3DecoderHelix decoder;
EncodedAudioStream audioStream(&pcmSink, &decoder);
File fileSource;
StreamCopy copier(audioStream, fileSource, 1024);

// ---------------- Playlist ----------------
int trackCount = 0;
int playCount  = 0;
volatile int  currentSongIndex = 0;
volatile bool paused = false;
volatile bool audioActive = false;

char currentTitle[96] = "";
SemaphoreHandle_t titleMutex;

// ---------------- M3U playlists ----------------
#define PLAYLIST_IGNORE_TEXT "_spotdown.org"
#define MAX_PLAYLISTS 12

struct PlaylistInfo {
  char     file[44];
  uint16_t count;
};
PlaylistInfo playlists[MAX_PLAYLISTS];
int  playlistCount = 0;
bool playlistsEnabled = false;

uint32_t *songHash = nullptr;
uint16_t *plSongs  = nullptr;

bool          plActive = false;
char          plActiveFile[44] = "";
char          plActiveName[32] = "All songs";
volatile int  pendingPlaylist = 0;
char          plNotice[24] = "";
volatile unsigned long plNoticeUntil = 0;
int           plMenuSel = 0;

// Volume 0..100 %
volatile int      volumePercent = 90;
volatile uint32_t volGain       = 207;

// Rotary encoder pins
#define PIN_ENC_CLK   32
#define PIN_ENC_DT    33
#define PIN_ENC_SW    13

// I2S DAC (UDA1334A)
#define PIN_I2S_BCLK  16
#define PIN_I2S_WS    17
#define PIN_I2S_DATA  4

#define SWITCH_MODE_BY_REBOOT 0

#define PCM_HP_MAX_SIZE     (32 * 1024)
#define PCM_HP_HEAP_RESERVE (30 * 1024)
// Receiver mode: the Bluetooth stack is running, but there is no MP3 decoder / audio task
#define PCM_RX_MAX_SIZE     (32 * 1024)
#define PCM_RX_HEAP_RESERVE (20 * 1024)

I2SStream i2sOutput;
I2SConfig i2sCfg;
volatile uint32_t srcRate     = 44100;
volatile uint8_t  srcChannels = 2;
volatile bool audioTaskStop = false, audioTaskDone = true;
volatile bool i2sTaskStop   = false, i2sTaskDone   = true;
TaskHandle_t audioTaskHandle = nullptr, i2sTaskHandle = nullptr;
bool decoderOn = false, i2sOn = false;
#define VOL_PER_COUNT 2
ESP32Encoder volumeEncoder;

uint16_t     *playOrder = nullptr;
volatile bool shuffleOn = false;
uint32_t      shuffleSeed = 0;
int           shuffleFirst = -1;

#define PREFS_NS "player"
Preferences   prefs;
char          currentFileName[96] = "";
volatile bool savePending = false;
unsigned long saveDueAt   = 0;

enum Cmd : int8_t { CMD_NEXT, CMD_PREV, CMD_TOGGLE_PAUSE, CMD_STOP, CMD_TOGGLE_SHUFFLE, CMD_SET_PLAYLIST };
QueueHandle_t cmdQueue;

// ---------------- App state ----------------
enum AppState { STATE_BT_SCANNER, STATE_CONNECTING, STATE_MAIN_PLAYER, STATE_MODE_MENU, STATE_PLAYLIST_MENU };
volatile AppState currentAppState = STATE_MAIN_PLAYER;

enum OutMode { OUT_HEADPHONES, OUT_BLUETOOTH };
volatile OutMode outMode = OUT_HEADPHONES;

// Where the music comes from: the SD card, or a phone streaming to us (receiver mode)
enum InSrc { SRC_SD, SRC_PHONE };
volatile InSrc inSrc = SRC_SD;
#define RX_NAME    "Music Playing Doohickey"   // name shown in the phone's Bluetooth settings
#define MODE_ITEMS 5                           // earpiece, bluetooth, receiver/sd, screen timeout, cancel
volatile bool rxConnected  = false;
volatile bool rxTitleDirty = false;
char rxTitle[64]  = "";
char rxArtist[48] = "";
OutMode lastI2SOut = OUT_HEADPHONES;           // wired output used for receiver mode
volatile unsigned long rxStartedAt   = 0;      // when the phone started streaming (0 = not yet)
volatile unsigned long rxConnectedAt = 0;      // when the phone connected (0 = not yet)
volatile int16_t       rxPeak        = 0;      // loudest sample seen since the last status line
volatile uint32_t      rxRate        = 44100;  // sample rate the phone announced (44100 / 48000)
volatile unsigned long rxMetaDue     = 0;      // when to ask the phone for the title again
bool rxInitTried = false, rxVolPending = false;
volatile uint32_t rxBytes = 0, rxCalls = 0, rxMaxGapMs = 0, rxMinFill = 0xFFFFFFFF, rxDiscards = 0;   // debug
volatile unsigned long rxLastCb = 0;
bool quickBoot = false;                        // restart requested by the menu (not a power-on)

AppState      menuReturnState = STATE_MAIN_PLAYER;
int           menuSelection   = 0;
char          menuNotice[24]  = "";
unsigned long menuNoticeUntil = 0;

// ---------------- Icons ----------------
const unsigned char PROGMEM bitmap_selector[]       = { 0x80, 0xC0, 0xE0, 0xF0, 0xE0, 0xC0, 0x80 };
const unsigned char PROGMEM bitmap_unselected[]     = { 0xFC, 0x84, 0x84, 0x84, 0x84, 0x84, 0xFC };
const unsigned char PROGMEM bitmap_icon_headphone[] = { 0x3C, 0x42, 0x81, 0xBD, 0xBD, 0x81, 0x42 };
const unsigned char PROGMEM bitmap_icon_back[]      = { 0x10, 0x30, 0x7F, 0xFF, 0x7F, 0x30, 0x10 };
const unsigned char PROGMEM shuffle_icon[]          = { 0x83, 0x00, 0x45, 0x00, 0x29, 0x00, 0x11, 0x80, 0x09, 0x00, 0x25, 0x00, 0x43, 0x80, 0x81, 0x00 };
const unsigned char PROGMEM bt_icon[]               = { 0x10, 0x18, 0x54, 0x38, 0x10, 0x38, 0x54, 0x18, 0x10, 0x00, 0x00, 0x00 };
const unsigned char PROGMEM bitmap_icon_playlist[]  = { 0xFE, 0x00, 0xFE, 0x00, 0xFE, 0x00, 0xFE };

const unsigned char PROGMEM bitmap_menu_unselected[] = { 0xF8, 0x88, 0x88, 0x88, 0x88, 0x88, 0xF8 };
const unsigned char PROGMEM bitmap_icon_earpiece[]   = { 0x3C, 0x42, 0x81, 0xBD, 0xBD, 0x81, 0x42 };
const unsigned char PROGMEM bitmap_icon_bluetooth[]  = { 0x20, 0x50, 0x28, 0x10, 0x28, 0x50, 0x20 };
const unsigned char PROGMEM bitmap_icon_cancel[]     = { 0x41, 0x22, 0x14, 0x08, 0x14, 0x22, 0x41 };
const unsigned char PROGMEM bitmap_icon_screen[]     = { 0x7E, 0x42, 0x42, 0x42, 0x7E, 0x18, 0x3C };
const unsigned char PROGMEM bitmap_icon_phone[]      = { 0x78, 0x48, 0x48, 0x48, 0x48, 0x78, 0x30 };
const unsigned char PROGMEM bitmap_icon_sd[]         = { 0x7C, 0x44, 0x54, 0x44, 0x44, 0x44, 0x7C };

// ---------------- Bluetooth device list ----------------
struct BTDeviceItem {
  String name;
  esp_bd_addr_t address;
};
#define MAX_DISCOVERED_DEVICES 10
BTDeviceItem discoveredDevices[MAX_DISCOVERED_DEVICES];
int deviceCount = 0;
int btMenuSelection = 0;
SemaphoreHandle_t listMutex;

String selectedDeviceName = "";
esp_bd_addr_t targetAddress;
volatile bool hasTarget = false;

volatile bool connectedFlag    = false;
volatile bool connectFailedFlag = false;
volatile bool mediaStartedFlag = false;
unsigned long connectedAt      = 0;
bool audioReady                = false;
volatile bool disconnectedFlag = false;

unsigned long connectStartTime = 0;
volatile unsigned long scanStartedAt = 0;      // when Bluetooth discovery was (re)started
const unsigned long CONNECT_TIMEOUT = 30000;

float waveOffset = 0.0;

// ---------------- Buttons ----------------
struct Button {
  uint8_t pin;
  bool down = false, longFired = false;
  bool pressEvt = false, shortEvt = false, longEvt = false;
  bool mute = false;                      // set when this press only woke the screen: ignore it until released
  unsigned long downAt = 0, lastChange = 0;
  void begin(uint8_t p) { pin = p; pinMode(p, INPUT_PULLUP); }
  void update() {
    pressEvt = shortEvt = longEvt = false;
    bool raw = (digitalRead(pin) == LOW);
    unsigned long now = millis();
    if (raw != down && now - lastChange > 40) {
      lastChange = now;
      down = raw;
      if (down) { pressEvt = true; downAt = now; longFired = false; }
      else if (!longFired) shortEvt = true;
    }
    if (down && !longFired && now - downAt > 800) { longFired = true; longEvt = true; }
    if (mute) {
      pressEvt = shortEvt = longEvt = false;
      if (!down) mute = false;
    }
  }
};
Button btnUp, btnDown, btnEnter, btnEnc;

// Screen saver: the OLED turns off after 5 minutes without a key press or encoder turn (saves battery);
// any key or the encoder turns it back on. Music keeps playing while the screen is off.
int  screenTimeoutMin = 5;          // 0 = never; editable in the output menu, remembered across power cycles
bool menuEditTimeout  = false;      // true while the timeout is being edited in the menu
bool screenOff = false;
unsigned long lastActivity = 0;

void saveScreenTimeout() {
  if (prefs.begin(PREFS_NS, false)) { prefs.putInt("scr", screenTimeoutMin); prefs.end(); }
}

void sendCmd(Cmd c) {
  int8_t v = (int8_t)c;
  xQueueSend(cmdQueue, &v, 0);
}

// Only one A2DP role exists per boot, so each object is created on first use (not as a global).
// (Defined after the enums on purpose: Arduino puts auto-generated prototypes before the first function.)
static BluetoothA2DPSource &btSourceObj() { static BluetoothA2DPSource *p = new BluetoothA2DPSource(); return *p; }
static BluetoothA2DPSink   &rxSinkObj()   { static BluetoothA2DPSink   *p = new BluetoothA2DPSink();   return *p; }
#define a2dp_source btSourceObj()
#define a2dp_sink   rxSinkObj()          // receiver mode (phone -> ESP32)

// Forward declarations for Portal FX & UI routines
void portalOut();
void portalIn();
void showFrame();
bool fxActive();
void startRipple(int x, int y);
void rippleDraw();
void songChangeCheck(const char *title);
bool songAnimDraw();
void drawBallStrip(int cy, uint32_t ms);
void splashStart();
void splashStage(const char *status, int pct);
void splashFinish();
void drawBTScannerMenu();
void drawConnectingScreen(String deviceName, float progress);
void drawMainPlayerUI();
void drawModeMenu();
void drawPlaylistMenu();

// =====================================================================
//  Bluetooth callbacks (core 0)
// =====================================================================
bool onDeviceFound(const char *name, esp_bd_addr_t addr, int rssi) {
  if (name == nullptr || name[0] == '\0') return false;

  bool connectNow = false;
  xSemaphoreTake(listMutex, portMAX_DELAY);

  if (hasTarget && memcmp(addr, targetAddress, sizeof(esp_bd_addr_t)) == 0) {
    connectNow = true;
  } else {
    bool exists = false;
    for (int i = 0; i < deviceCount; i++) {
      if (memcmp(discoveredDevices[i].address, addr, sizeof(esp_bd_addr_t)) == 0) { exists = true; break; }
    }
    if (!exists && deviceCount < MAX_DISCOVERED_DEVICES) {
      discoveredDevices[deviceCount].name = String(name);
      memcpy(discoveredDevices[deviceCount].address, addr, sizeof(esp_bd_addr_t));
      deviceCount++;
      Serial.printf("Found BT device: %s (RSSI %d)\n", name, rssi);
    }
  }

  xSemaphoreGive(listMutex);
  return connectNow;
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
  if (outMode != OUT_BLUETOOTH) return;
  switch (state) {
    case ESP_A2D_CONNECTION_STATE_CONNECTING:
      break;
    case ESP_A2D_CONNECTION_STATE_CONNECTED:
      connectedAt = millis();
      mediaStartedFlag = false;
      connectedFlag = true;
      break;
    case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
      {
        AppState scr = currentAppState;
        if (scr == STATE_MODE_MENU) scr = menuReturnState;
        else if (scr == STATE_PLAYLIST_MENU) scr = STATE_MAIN_PLAYER;
        if (scr == STATE_MAIN_PLAYER) disconnectedFlag = true;
        else if (scr == STATE_CONNECTING) connectFailedFlag = true;
      }
      break;
    default:
      break;
  }
}

volatile uint32_t cbCalls = 0;

int32_t get_data_frames(Frame *frame, int32_t frame_count) {
  cbCalls++;
  lastFrameCount = frame_count;
#if TEST_TONE
  static int tonePos = 0;
  for (int i = 0; i < frame_count; i++) {
    int16_t v = toneTable[tonePos];
    frame[i].channel1 = v;
    frame[i].channel2 = v;
    if (++tonePos >= 100) tonePos = 0;
  }
  return frame_count;
#endif
  size_t bytes = frame_count * sizeof(Frame);
  size_t got = 0;
  static bool buffering = true;
  if (pcmBuf && audioActive && !paused) {
    size_t avail = xStreamBufferBytesAvailable(pcmBuf);
    if (buffering && avail >= pcmPrefill) buffering = false;
    if (!buffering && avail < minFill) minFill = avail;
    if (!buffering) {
      size_t take = (avail < bytes ? avail : bytes) & ~(size_t)3;
      if (take) got = xStreamBufferReceive(pcmBuf, (void *)frame, take, 0);
      if (got < bytes) { buffering = true; underruns++; }
    }
  }
  uint32_t g = volGain;
  if (g < 256 && got > 0) {
    int16_t *smp = (int16_t *)frame;
    size_t cnt = got / 2;
    for (size_t i = 0; i < cnt; i++) smp[i] = (int16_t)(((int32_t)smp[i] * (int32_t)g) >> 8);
  }
  if (got < bytes) memset(((uint8_t *)frame) + got, 0, bytes - got);
  return frame_count;
}

void audio_state_changed(esp_a2d_audio_state_t state, void *ptr) {
  if (state == ESP_A2D_AUDIO_STATE_STARTED) mediaStartedFlag = true;
}

void startBluetooth() {
  a2dp_source.set_ssid_callback(onDeviceFound);
  a2dp_source.set_on_connection_state_changed(connection_state_changed);
  a2dp_source.set_auto_reconnect(false);
  a2dp_source.set_volume(127);
  a2dp_source.set_on_audio_state_changed(audio_state_changed);
  a2dp_source.set_data_callback_in_frames(get_data_frames);
  Serial.printf("Bluetooth source: starting discovery (free heap %u, largest %u)\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
  scanStartedAt = millis();
  a2dp_source.start();
  Serial.printf("Bluetooth source: started (free heap %u, largest %u)\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
}

// =====================================================================
//  Receiver mode: Bluetooth sink callbacks (core 0)
// =====================================================================
// Raw PCM from the phone (16-bit stereo) goes straight into the same pcmBuf the MP3 decoder normally fills.
void rxStreamData(const uint8_t *data, uint32_t len) {
  if (!pcmBuf || !audioActive) return;
  len &= ~3u;                                    // whole stereo frames only
  if (len == 0) return;
  {
    unsigned long nowCb = millis();
    if (rxLastCb && nowCb - rxLastCb > rxMaxGapMs) rxMaxGapMs = nowCb - rxLastCb;
    rxLastCb = nowCb;
    rxBytes += len;
    rxCalls++;
  }
  if (paused) paused = false;                    // audio is arriving, so the phone is playing

  int16_t pk = 0;                                // loudest sample in this chunk (status line)
  for (uint32_t i = 0; i + 1 < len; i += 16) {
    int16_t v = (int16_t)(data[i] | (data[i + 1] << 8));
    if (v < 0) v = (v == -32768) ? 32767 : -v;
    if (v > pk) pk = v;
  }
  if (pk > rxPeak) rxPeak = pk;

  // Wait for room (up to 100 ms) instead of dropping audio: like the library's own I2S output, this
  // holds the phone back through Bluetooth flow control. Chunks stay whole, so left/right never swap.
  uint32_t waited = 0;
  while (pcmBuf && xStreamBufferSpacesAvailable(pcmBuf) < len) {
    if (!audioActive || waited >= 100) { pcmDropped += len; return; }
    vTaskDelay(1);
    waited += portTICK_PERIOD_MS;
  }
  if (pcmBuf) pcmWritten += xStreamBufferSend(pcmBuf, data, len, 0);
}

void rxSampleRate(uint16_t rate) {               // phones use 44.1 or 48 kHz
  if (rate >= 8000 && rate <= 48000) { rxRate = rate; srcRate = rate; srcChannels = 2; }
}

void rxConnState(esp_a2d_connection_state_t state, void *ptr) {
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    rxConnected = true;
    rxConnectedAt = millis() | 1;
    rxVolPending = true;
    rxMetaDue = millis() + 800;
  } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
    rxConnected = false;
    paused = true;
    rxTitle[0] = '\0';
    rxArtist[0] = '\0';
    rxTitleDirty = true;
  }
}

void rxAudioState(esp_a2d_audio_state_t state, void *ptr) {
  if (state == ESP_A2D_AUDIO_STATE_STARTED) {
    paused = false;
    if (!rxStartedAt) rxStartedAt = millis() | 1;
  } else {
    paused = true;
  }
}

void rxMetadata(uint8_t id, const uint8_t *text) {
  if (!text) return;
  if (id == ESP_AVRC_MD_ATTR_TITLE)       strlcpy(rxTitle,  (const char *)text, sizeof(rxTitle));
  else if (id == ESP_AVRC_MD_ATTR_ARTIST) strlcpy(rxArtist, (const char *)text, sizeof(rxArtist));
  else return;
  rxTitleDirty = true;
}

void startSink() {
  a2dp_sink.set_stream_reader(rxStreamData, false);   // raw PCM to us, no built-in I2S output
  a2dp_sink.set_sample_rate_callback(rxSampleRate);
  a2dp_sink.set_on_connection_state_changed(rxConnState);
  a2dp_sink.set_on_audio_state_changed(rxAudioState);
  a2dp_sink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
  a2dp_sink.set_avrc_metadata_callback(rxMetadata);
  a2dp_sink.start(RX_NAME, true);                // visible to phones, auto-reconnects to the last phone
}

// =====================================================================
//  Helpers
// =====================================================================
String formatString(String text, int maxLen) {
  if ((int)text.length() > maxLen) return text.substring(0, maxLen - 3) + "...";
  return text;
}

String baseOf(const char *full) {
  String s(full);
  int slash = s.lastIndexOf('/');
  if (slash >= 0) s = s.substring(slash + 1);
  return s;
}

String cleanName(String s) {
  String lower = s;
  lower.toLowerCase();
  if (lower.endsWith(".mp3")) s = s.substring(0, s.length() - 4);
  return s;
}

bool isPlayable(const String &base) {
  if (base.length() == 0 || base.startsWith(".")) return false;
  String lower = base;
  lower.toLowerCase();
  return lower.endsWith(".mp3");
}

int countSongs() {
  int count = 0;
  File root = SD.open("/");
  if (!root) return 0;
  File e = root.openNextFile();
  while (e) {
    if (!e.isDirectory() && isPlayable(baseOf(e.name()))) count++;
    e.close();
    e = root.openNextFile();
  }
  root.close();
  return count;
}

// Song name cache: getSongName() used to walk the whole SD root folder on every
// track change (slow with many songs). Names are now read once at boot.
#define NAME_CACHE_MAX_BYTES (16 * 1024)
static char     *nameCachePool  = nullptr;
static uint16_t *nameCacheOff   = nullptr;
static int       nameCacheCount = 0;

#define NAME_CACHE_MAX_ENTRIES 700
// expected > 0 : the number of songs is already known.
// expected <= 0: unknown, the songs are counted while the names are cached (ONE folder pass).
void buildNameCache(int expected) {
  nameCacheCount = 0;
  int cap = (expected > 0) ? expected : NAME_CACHE_MAX_ENTRIES;
  char *pool = (char *)malloc(NAME_CACHE_MAX_BYTES);
  uint16_t *off = (uint16_t *)malloc(cap * sizeof(uint16_t));
  File root = SD.open("/");
  if (!pool || !off || !root) { free(pool); free(off); if (root) root.close(); return; }
  size_t pos = 0;
  int cnt = 0;
  bool ok = true;
  File e = root.openNextFile();
  while (e) {
    if (!e.isDirectory()) {
      String b = baseOf(e.name());
      if (isPlayable(b)) {
        size_t len = b.length() + 1;
        if (cnt >= cap || pos + len > NAME_CACHE_MAX_BYTES) { ok = false; e.close(); break; }
        memcpy(pool + pos, b.c_str(), len);
        off[cnt++] = (uint16_t)pos;
        pos += len;
      }
    }
    e.close();
    e = root.openNextFile();
  }
  root.close();
  if (!ok || cnt == 0 || (expected > 0 && cnt != expected)) { free(pool); free(off); return; }   // too big: fall back to scanning
  char *shrunk = (char *)realloc(pool, pos);
  uint16_t *offShrunk = (uint16_t *)realloc(off, cnt * sizeof(uint16_t));
  nameCachePool  = shrunk ? shrunk : pool;
  nameCacheOff   = offShrunk ? offShrunk : off;
  nameCacheCount = cnt;
}

// Bluetooth needs a lot of free heap (v16: ~12 KB free = half-rate, choppy audio),
// so the cache is released while Bluetooth is the output and rebuilt afterwards.
void freeNameCache() {
  nameCacheCount = 0;
  char *p = nameCachePool;
  uint16_t *o = nameCacheOff;
  nameCachePool = nullptr;
  nameCacheOff  = nullptr;
  free(p);
  free(o);
}

bool getSongName(int n, String &outName) {
  if (nameCachePool && n >= 0 && n < nameCacheCount) {
    outName = String(nameCachePool + nameCacheOff[n]);
    return true;
  }
  // No-RAM fallback: keep the folder open so "next song" continues where the last
  // lookup stopped instead of re-scanning from the start.
  static File dir;
  static int  nextIdx = 0;
  if (!dir || n < nextIdx) {
    if (dir) dir.close();
    dir = SD.open("/");
    nextIdx = 0;
    if (!dir) return false;
  }
  File e = dir.openNextFile();
  while (e) {
    if (!e.isDirectory()) {
      String b = baseOf(e.name());
      if (isPlayable(b)) {
        if (nextIdx == n) { outName = b; nextIdx++; e.close(); return true; }
        nextIdx++;
      }
    }
    e.close();
    e = dir.openNextFile();
  }
  dir.close();
  nextIdx = 0;
  return false;
}

void setTitle(const String &name) {
  String t = cleanName(name);
  if (t.length() > 95) t = t.substring(0, 95);
  xSemaphoreTake(titleMutex, portMAX_DELAY);
  strncpy(currentTitle, t.c_str(), sizeof(currentTitle) - 1);
  currentTitle[sizeof(currentTitle) - 1] = '\0';
  xSemaphoreGive(titleMutex);
}

// =====================================================================
//  Playlist files (M3U / M3U8)
// =====================================================================
static int ciCompare(const char *a, const char *b) {
  while (*a && *b) {
    char x = tolower((unsigned char)*a), y = tolower((unsigned char)*b);
    if (x != y) return (int)x - (int)y;
    a++; b++;
  }
  return (int)tolower((unsigned char)*a) - (int)tolower((unsigned char)*b);
}

static uint32_t nameKey(const char *s) {
  char buf[200];
  size_t n = 0;
  for (; s[n] && n < sizeof(buf) - 1; n++) {
    char c = s[n];
    if (c >= 'A' && c <= 'Z') c += 32;
    buf[n] = c;
  }
  buf[n] = '\0';
  if (n >= 4 && strcmp(buf + n - 4, ".mp3") == 0) { n -= 4; buf[n] = '\0'; }
  const char *ig = PLAYLIST_IGNORE_TEXT;
  size_t il = strlen(ig);
  if (il > 0) {
    char *p;
    while ((p = strstr(buf, ig)) != nullptr) memmove(p, p + il, strlen(p + il) + 1);
  }
  size_t l = strlen(buf);
  while (l > 0 && buf[l - 1] == ' ') buf[--l] = '\0';
  uint32_t h = 2166136261u;
  for (size_t i = 0; buf[i]; i++) { h ^= (uint8_t)buf[i]; h *= 16777619u; }
  return h;
}

bool buildSongHashes() {
  if (trackCount == 0) return false;
  songHash = (uint32_t *)malloc(trackCount * sizeof(uint32_t));
  if (!songHash) return false;
  if (nameCachePool && nameCacheCount == trackCount) {            // names are already in RAM: no SD access needed
    for (int k = 0; k < trackCount; k++) songHash[k] = nameKey(nameCachePool + nameCacheOff[k]);
    return true;
  }
  int i = 0;
  File root = SD.open("/");
  if (!root) { free(songHash); songHash = nullptr; return false; }
  File e = root.openNextFile();
  while (e && i < trackCount) {
    if (!e.isDirectory()) {
      String b = baseOf(e.name());
      if (isPlayable(b)) songHash[i++] = nameKey(b.c_str());
    }
    e.close();
    e = root.openNextFile();
  }
  if (e) e.close();
  root.close();
  while (i < trackCount) songHash[i++] = 0;
  return true;
}

static int hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static int playlistLineToSong(char *s) {
  if ((uint8_t)s[0] == 0xEF && (uint8_t)s[1] == 0xBB && (uint8_t)s[2] == 0xBF) s += 3;
  while (*s == ' ' || *s == '\t') s++;
  size_t l = strlen(s);
  while (l > 0 && (s[l - 1] == ' ' || s[l - 1] == '\t')) s[--l] = '\0';
  if (l == 0 || s[0] == '#') return -2;

  char *base = s;
  for (char *p = s; *p; p++) if (*p == '/' || *p == '\\') base = p + 1;
  char *w = base;
  for (char *r = base; *r; r++) {
    if (*r == '%' && hexv(r[1]) >= 0 && hexv(r[2]) >= 0) {
      *w++ = (char)(hexv(r[1]) * 16 + hexv(r[2]));
      r += 2;
    } else {
      *w++ = *r;
    }
  }
  *w = '\0';
  if (*base == '\0') return -2;

  uint32_t key = nameKey(base);
  for (int i = 0; i < trackCount; i++) if (songHash[i] == key) return i;
  return -1;
}

int parsePlaylist(const char *file, uint16_t *out, int cap, bool verbose) {
  if (!songHash || trackCount == 0) return -2;
  String path = "/" + String(file);
  File f = SD.open(path.c_str(), FILE_READ);
  if (!f) return -1;

  const size_t PL_LINE_MAX = 320, PL_CHUNK = 256;
  char *line = (char *)malloc(PL_LINE_MAX + PL_CHUNK);
  if (!line) { f.close(); return -2; }
  char *chunk = line + PL_LINE_MAX;

  size_t len = 0;
  int matched = 0;

  auto handle = [&]() {
    line[len] = '\0';
    int r = playlistLineToSong(line);
    len = 0;
    if (r == -2) return;
    if (r >= 0) {
      if (matched < cap) {
        if (out) out[matched] = (uint16_t)r;
        matched++;
      }
    }
  };

  while (true) {
    int got = f.read((uint8_t *)chunk, PL_CHUNK);
    if (got <= 0) break;
    for (int k = 0; k < got; k++) {
      char c = chunk[k];
      if (c == '\n' || c == '\r') { if (len) handle(); }
      else if (len < PL_LINE_MAX - 1) line[len++] = c;
    }
  }
  if (len) handle();

  free(line);
  f.close();
  return matched;
}

void fileToName(const char *file, char *out, size_t cap) {
  strncpy(out, file, cap - 1);
  out[cap - 1] = '\0';
  char *dot = strrchr(out, '.');
  if (dot) *dot = '\0';
}

void plDisplayName(int idx, char *out, size_t cap) {
  if (idx <= 0 || idx >= playlistCount) { strncpy(out, "All songs", cap - 1); out[cap - 1] = '\0'; }
  else fileToName(playlists[idx].file, out, cap);
}

int activePlaylistIndex() {
  if (!plActive) return 0;
  for (int i = 1; i < playlistCount; i++) if (strcmp(playlists[i].file, plActiveFile) == 0) return i;
  return 0;
}

// Playlist list (names + song counts) cached in flash, used on quick restarts so the card is not scanned again.
// Layout: [0..1] song count, [2] number of playlists (without "All songs"), [3] unused, then 46 bytes per playlist.
#define PLCACHE_KEY "plcache"
static bool loadPlaylistCache() {
  uint8_t buf[4 + MAX_PLAYLISTS * 46];
  if (!prefs.begin(PREFS_NS, true)) return false;
  size_t len = prefs.getBytes(PLCACHE_KEY, buf, sizeof(buf));
  prefs.end();
  if (len < 4) return false;
  int tc = buf[0] | (buf[1] << 8);
  int m = buf[2];
  if (tc != trackCount || m > MAX_PLAYLISTS - 1 || len != (size_t)(4 + 46 * m)) return false;
  for (int i = 0; i < m; i++) {
    memcpy(playlists[i + 1].file, buf + 4 + 46 * i, 44);
    playlists[i + 1].file[43] = '\0';
    playlists[i + 1].count = (uint16_t)(buf[4 + 46 * i + 44] | (buf[4 + 46 * i + 45] << 8));
  }
  playlistCount = m + 1;
  return true;
}

static void savePlaylistCache() {
  uint8_t buf[4 + MAX_PLAYLISTS * 46];
  int m = playlistCount - 1;
  if (m < 0) return;
  buf[0] = trackCount & 255; buf[1] = (trackCount >> 8) & 255; buf[2] = (uint8_t)m; buf[3] = 0;
  for (int i = 0; i < m; i++) {
    memcpy(buf + 4 + 46 * i, playlists[i + 1].file, 44);
    buf[4 + 46 * i + 44] = playlists[i + 1].count & 255;
    buf[4 + 46 * i + 45] = (playlists[i + 1].count >> 8) & 255;
  }
  if (prefs.begin(PREFS_NS, false)) { prefs.putBytes(PLCACHE_KEY, buf, 4 + 46 * m); prefs.end(); }
}

void scanPlaylists() {
  playlists[0].file[0] = '\0';
  playlists[0].count = trackCount;
  playlistCount = 1;
  if (!playlistsEnabled) return;
  if (quickBoot && loadPlaylistCache()) { Serial.println("Playlists: loaded from cache"); return; }

  File root = SD.open("/");
  if (!root) return;
  File e = root.openNextFile();
  while (e && playlistCount < MAX_PLAYLISTS) {
    if (!e.isDirectory()) {
      String b = baseOf(e.name());
      String lower = b;
      lower.toLowerCase();
      if (!b.startsWith(".") && (lower.endsWith(".m3u") || lower.endsWith(".m3u8"))) {
        if (b.length() < sizeof(playlists[0].file)) {
          strcpy(playlists[playlistCount].file, b.c_str());
          int c = parsePlaylist(b.c_str(), nullptr, trackCount, false);
          playlists[playlistCount].count = (c > 0) ? (uint16_t)c : 0;
          playlistCount++;
        }
      }
    }
    e.close();
    e = root.openNextFile();
  }
  if (e) e.close();
  root.close();

  for (int i = 1; i < playlistCount; i++) {
    for (int j = i + 1; j < playlistCount; j++) {
      if (ciCompare(playlists[j].file, playlists[i].file) < 0) {
        PlaylistInfo t = playlists[i]; playlists[i] = playlists[j]; playlists[j] = t;
      }
    }
  }
  savePlaylistCache();
}

void setPlNotice(const char *msg) {
  xSemaphoreTake(titleMutex, portMAX_DELAY);
  strncpy(plNotice, msg, sizeof(plNotice) - 1);
  plNotice[sizeof(plNotice) - 1] = '\0';
  xSemaphoreGive(titleMutex);
  plNoticeUntil = millis() + 3000;
}

// =====================================================================
//  Audio task (core 1)
// =====================================================================
int songAt(int pos) { return playOrder ? (int)playOrder[pos] : pos; }

int currentSongNumber() {
  if (playCount == 0) return 0;
  return songAt(currentSongIndex % playCount);
}

uint32_t xorshift32(uint32_t &st) { st ^= st << 13; st ^= st >> 17; st ^= st << 5; return st; }

static inline int baseAt(int i) { return plActive ? (int)plSongs[i] : i; }

void buildOrder(bool shuffled, uint32_t seed, int firstSong) {
  int n = playCount;
  if (!playOrder || n == 0) return;
  for (int i = 0; i < n; i++) playOrder[i] = (uint16_t)baseAt(i);
  if (!shuffled) return;
  uint32_t st = seed ? seed : 1;
  for (int i = n - 1; i > 0; i--) {
    int j = (int)(xorshift32(st) % (uint32_t)(i + 1));
    uint16_t t = playOrder[i]; playOrder[i] = playOrder[j]; playOrder[j] = t;
  }
  if (firstSong >= 0) {
    for (int i = 0; i < n; i++) {
      if (playOrder[i] == firstSong) { uint16_t t = playOrder[0]; playOrder[0] = playOrder[i]; playOrder[i] = t; break; }
    }
  }
}

int posInOrder(int song) {
  if (!playOrder) return song;
  for (int i = 0; i < playCount; i++) if (playOrder[i] == song) return i;
  return 0;
}

void toggleShuffle() {
  int n = playCount;
  if (n == 0 || !playOrder) return;
  int song = currentSongNumber();
  if (!shuffleOn) {
    shuffleSeed  = esp_random() | 1;
    shuffleFirst = song;
    buildOrder(true, shuffleSeed, song);
    shuffleOn = true;
  } else {
    buildOrder(false, 0, -1);
    shuffleOn = false;
    shuffleFirst = -1;
  }
  currentSongIndex = posInOrder(song);
  savePending = true;
  saveDueAt = millis() + 500;
}

void saveState() {
  if (!prefs.begin(PREFS_NS, false)) return;
  prefs.putInt("song", currentSongNumber());
  prefs.putString("name", currentFileName);
  prefs.putBool("shuf", shuffleOn);
  prefs.putUInt("seed", shuffleSeed);
  prefs.putInt("first", shuffleFirst);
  prefs.putString("plfile", plActive ? plActiveFile : "");
  prefs.end();
}

int findSongByName(const char *name) {
  File root = SD.open("/");
  if (!root) return -1;
  int i = 0, found = -1;
  File e = root.openNextFile();
  while (e) {
    if (!e.isDirectory()) {
      String b = baseOf(e.name());
      if (isPlayable(b)) {
        if (strncmp(b.c_str(), name, sizeof(currentFileName) - 1) == 0) { found = i; e.close(); break; }
        i++;
      }
    }
    e.close();
    e = root.openNextFile();
  }
  root.close();
  return found;
}

void loadState() {
  int n = trackCount;
  if (n == 0) return;
  if (!nameCachePool) buildNameCache(n);
  playCount = n;
  playOrder = (uint16_t *)malloc(n * sizeof(uint16_t));
  if (!playOrder) return;

  plSongs = (uint16_t *)malloc(n * sizeof(uint16_t));
  if (plSongs && buildSongHashes()) playlistsEnabled = true;

  int song = 0, first = -1;
  bool shuf = false;
  uint32_t seed = 0;
  char name[96] = "";
  char plf[44] = "";
  if (prefs.begin(PREFS_NS, true)) {
    song  = prefs.getInt("song", 0);
    prefs.getString("name", name, sizeof(name));
    shuf  = prefs.getBool("shuf", false);
    seed  = prefs.getUInt("seed", 0);
    first = prefs.getInt("first", -1);
    prefs.getString("plfile", plf, sizeof(plf));
    prefs.end();
  }

  plActive = false;
  plActiveFile[0] = '\0';
  strcpy(plActiveName, "All songs");
  if (playlistsEnabled && plf[0]) {
    int got = parsePlaylist(plf, plSongs, n, false);
    if (got > 0) {
      plActive = true;
      playCount = got;
      strncpy(plActiveFile, plf, sizeof(plActiveFile) - 1);
      plActiveFile[sizeof(plActiveFile) - 1] = '\0';
      fileToName(plActiveFile, plActiveName, sizeof(plActiveName));
    }
  }

  if (name[0]) {
    String nm;
    bool same = (song >= 0 && song < n && getSongName(song, nm) && strncmp(nm.c_str(), name, sizeof(name) - 1) == 0);
    if (!same) {
      int f = findSongByName(name);
      song = (f >= 0) ? f : 0;
    }
  }
  if (song < 0 || song >= n) song = 0;

  shuffleOn    = shuf && seed != 0;
  shuffleSeed  = seed;
  shuffleFirst = first;
  buildOrder(shuffleOn, seed, first);
  currentSongIndex = posInOrder(song);
}

bool fileOpen = false;

void closeCurrentFile() {
  if (fileOpen) { fileSource.close(); fileOpen = false; }
}

void changeSong(int dir) {
  int n = playCount;
  if (n == 0) return;
  closeCurrentFile();
  currentSongIndex = (currentSongIndex + dir + n) % n;
  if (pcmBuf) xStreamBufferReset(pcmBuf);
  paused = false;
}

void applyPlaylist(int idx) {
  if (!playlistsEnabled || idx < 0 || idx >= playlistCount) return;

  if (idx == 0) {
    plActive = false;
    plActiveFile[0] = '\0';
    playCount = trackCount;
    xSemaphoreTake(titleMutex, portMAX_DELAY);
    strcpy(plActiveName, "All songs");
    xSemaphoreGive(titleMutex);
  } else {
    if (playlists[idx].count == 0) { setPlNotice("No songs matched"); return; }
    int got = parsePlaylist(playlists[idx].file, plSongs, trackCount, false);
    if (got <= 0) { setPlNotice("No songs matched"); return; }
    plActive = true;
    playCount = got;
    strncpy(plActiveFile, playlists[idx].file, sizeof(plActiveFile) - 1);
    plActiveFile[sizeof(plActiveFile) - 1] = '\0';
    xSemaphoreTake(titleMutex, portMAX_DELAY);
    fileToName(plActiveFile, plActiveName, sizeof(plActiveName));
    xSemaphoreGive(titleMutex);
  }

  closeCurrentFile();
  if (pcmBuf) xStreamBufferReset(pcmBuf);
  paused = false;

  if (shuffleOn) {
    shuffleSeed = esp_random() | 1;
    shuffleFirst = -1;
    buildOrder(true, shuffleSeed, -1);
  } else {
    buildOrder(false, 0, -1);
  }
  currentSongIndex = 0;
  savePending = true;
  saveDueAt = millis() + 500;
}

void audioTask(void *param) {
  int failCount = 0;

  for (;;) {
    if (audioTaskStop) {
      closeCurrentFile();
      audioTaskDone = true;
      vTaskDelete(NULL);
    }

    int8_t cmd;
    while (xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) {
      switch (cmd) {
        case CMD_NEXT:         changeSong(+1); break;
        case CMD_PREV:         changeSong(-1); break;
        case CMD_TOGGLE_PAUSE: paused = !paused; break;
        case CMD_TOGGLE_SHUFFLE: toggleShuffle(); break;
        case CMD_SET_PLAYLIST: applyPlaylist(pendingPlaylist); break;
        case CMD_STOP:
          closeCurrentFile();
          if (pcmBuf) xStreamBufferReset(pcmBuf);
          paused = false;
          setTitle("");
          break;
      }
    }

    int n = playCount;
    if (!audioActive || n == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (savePending && (int32_t)(millis() - saveDueAt) >= 0) {
      savePending = false;
      saveState();
    }

    if (paused) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (!fileOpen) {
      String name;
      bool ok = getSongName(songAt(currentSongIndex), name);
      if (ok) {
        String path = "/" + name;
        fileSource = SD.open(path.c_str(), FILE_READ);
        ok = (bool)fileSource;
        if (ok) {
          fileOpen = true;
          failCount = 0;
          setTitle(name);
          strncpy(currentFileName, name.c_str(), sizeof(currentFileName) - 1);
          currentFileName[sizeof(currentFileName) - 1] = '\0';
          savePending = true;
          saveDueAt = millis() + 1500;
        }
      }
      if (!ok) {
        currentSongIndex = (currentSongIndex + 1) % n;
        if (++failCount >= n) { paused = true; failCount = 0; }
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      continue;
    }

    if (xStreamBufferSpacesAvailable(pcmBuf) < 2048) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    uint32_t t0 = millis();
    copier.copy();
    uint32_t dt = millis() - t0;
    if (dt > maxCopyMs) maxCopyMs = dt;
    if (outMode != OUT_BLUETOOTH) {
      AudioInfo ai = audioStream.audioInfo();
      if (ai.sample_rate >= 8000 && ai.sample_rate <= 48000 && ai.channels >= 1 && ai.channels <= 2) {
        srcRate = ai.sample_rate;
        srcChannels = ai.channels;
      }
    }

    if (!fileSource.available()) {
      closeCurrentFile();
      currentSongIndex = (currentSongIndex + 1) % n;
    }

    if (outMode != OUT_BLUETOOTH) vTaskDelay(1);   // let the UI loop (same core) draw while the buffer refills
  }
}

void goBackToScanner() {
  audioActive = false;
  sendCmd(CMD_STOP);

  uint8_t tmp[256];
  unsigned long t = millis();
  while (millis() - t < 300) {
    while (pcmBuf && xStreamBufferReceive(pcmBuf, tmp, sizeof(tmp), 0) > 0) {}
    delay(5);
  }

  xSemaphoreTake(listMutex, portMAX_DELAY);
  hasTarget = false;
  deviceCount = 0;
  btMenuSelection = 0;
  xSemaphoreGive(listMutex);
  currentAppState = STATE_BT_SCANNER;
  a2dp_source.end();
  delay(300);
  startBluetooth();
}

// =====================================================================
//  Audio pipeline init / teardown, I2S feeder
// =====================================================================
static inline void applyGain(int16_t *s, size_t n) {
  uint32_t g = volGain;
  if (g >= 256) return;
  for (size_t i = 0; i < n; i++) s[i] = (int16_t)(((int32_t)s[i] * (int32_t)g) >> 8);
}

void i2sTask(void *param) {
  uint8_t in[256];
  uint8_t out[512];
  bool buffering = true;
  uint32_t curRate = srcRate;
  size_t prefill = pcmSize / 2;
  size_t prefillCap = (inSrc == SRC_PHONE) ? 16 * 1024 : 8 * 1024;   // a bit more cushion for Bluetooth jitter
  if (prefill > prefillCap) prefill = prefillCap;

  while (!i2sTaskStop) {
    uint32_t want = srcRate;
    if (want != curRate) {
      curRate = want;
      i2sOutput.end();
      i2sCfg.sample_rate = curRate;
      i2sOutput.begin(i2sCfg);
    }

    bool mono = (srcChannels == 1);
    size_t outLen = 0;
    if (pcmBuf && audioActive && !paused) {
      size_t avail = xStreamBufferBytesAvailable(pcmBuf);
      if (buffering && avail >= prefill) buffering = false;
      if (!buffering && inSrc == SRC_PHONE && avail < rxMinFill) rxMinFill = avail;
      if (!buffering && inSrc == SRC_PHONE && avail > (pcmSize / 5) * 4) {
        // buffer over 80% full (phone clock a little faster than ours): drop 8 frames now and then, inaudible,
        // instead of blocking the Bluetooth task
        uint8_t junk[32];
        size_t d = xStreamBufferReceive(pcmBuf, junk, sizeof(junk), 0);
        avail = (avail > d) ? avail - d : 0;
        rxDiscards++;
      }
      if (!buffering) {
        size_t maxIn = mono ? sizeof(in) : sizeof(out);
        size_t take = (avail < maxIn ? avail : maxIn) & ~(size_t)3;
        if (take == 0) {
          buffering = true;
          underruns++;
        } else if (mono) {
          size_t got = xStreamBufferReceive(pcmBuf, in, take, 0);
          int16_t *s = (int16_t *)in;
          int16_t *d = (int16_t *)out;
          for (size_t i = 0; i < got / 2; i++) { d[2 * i] = s[i]; d[2 * i + 1] = s[i]; }
          outLen = got * 2;
        } else {
          outLen = xStreamBufferReceive(pcmBuf, out, take, 0);
        }
      }
    }
    if (outLen == 0) {
      memset(out, 0, 256);
      outLen = 256;
    } else {
      applyGain((int16_t *)out, outLen / 2);
    }
    i2sOutput.write(out, outLen);
  }
  i2sTaskDone = true;
  vTaskDelete(NULL);
}

bool initAudio(OutMode mode) {
  audioTaskStop = false;
  i2sTaskStop   = false;

  if (mode != OUT_BLUETOOTH) {
    i2sCfg = i2sOutput.defaultConfig(TX_MODE);
    i2sCfg.sample_rate     = (inSrc == SRC_PHONE) ? rxRate : 44100;   // receiver: the phone's rate, not a fixed 44.1 kHz
    i2sCfg.bits_per_sample = 16;
    i2sCfg.channels        = 2;
    i2sCfg.pin_bck         = PIN_I2S_BCLK;
    i2sCfg.pin_ws          = PIN_I2S_WS;
    i2sCfg.pin_data        = PIN_I2S_DATA;
    if (!i2sOutput.begin(i2sCfg)) return false;
    i2sOn = true;
    srcRate = (inSrc == SRC_PHONE) ? rxRate : 44100;
    srcChannels = 2;
  }

  if (inSrc == SRC_SD) {      // receiver mode: no MP3 decoder / audio task, the phone is the source
    if (!audioStream.begin()) return false;
    decoderOn = true;
    copier.begin(audioStream, fileSource);
    audioTaskDone = false;
    if (xTaskCreatePinnedToCore(audioTask, "audioTask", (mode == OUT_BLUETOOTH) ? 4608 : 8192, NULL, 3, &audioTaskHandle, 1) != pdPASS) {
      audioTaskDone = true;
      return false;
    }
  }

  size_t maxSize = (inSrc == SRC_PHONE) ? PCM_RX_MAX_SIZE     : ((mode != OUT_BLUETOOTH) ? PCM_HP_MAX_SIZE     : PCM_MAX_SIZE);
  size_t reserve = (inSrc == SRC_PHONE) ? PCM_RX_HEAP_RESERVE : ((mode != OUT_BLUETOOTH) ? PCM_HP_HEAP_RESERVE : PCM_HEAP_RESERVE);
  size_t largest = ESP.getMaxAllocHeap();
  size_t size = (largest > reserve + PCM_MIN_SIZE) ? (largest - reserve) : PCM_MIN_SIZE;
  if (size > maxSize) size = maxSize;
  size &= ~(size_t)3;
  pcmBuf = xStreamBufferCreate(size, 1);
  if (!pcmBuf) return false;
  pcmSize = size;
  pcmPrefill = size / 2;

  if (mode != OUT_BLUETOOTH) {
    i2sTaskDone = false;
    if (xTaskCreatePinnedToCore(i2sTask, "i2sTask", 6144, NULL, 4, &i2sTaskHandle, (inSrc == SRC_PHONE) ? 1 : 0) != pdPASS) {
      i2sTaskDone = true;
      return false;
    }
  }

  Serial.printf("Audio ready (%s, %s): PCM buffer %u bytes, free heap %u\n",
                inSrc == SRC_PHONE ? "receiver" : "SD", mode == OUT_BLUETOOTH ? "bluetooth" : "headphones",
                (unsigned)pcmSize, (unsigned)ESP.getFreeHeap());
  audioReady = true;
  return true;
}

void stopAudio() {
  audioActive = false;
  audioTaskStop = true;
  i2sTaskStop = true;

  uint8_t tmp[256];
  unsigned long t = millis();
  while ((!audioTaskDone || !i2sTaskDone) && millis() - t < 1500) {
    while (pcmBuf && xStreamBufferReceive(pcmBuf, tmp, sizeof(tmp), 0) > 0) {}
    delay(5);
  }

  if (decoderOn) { audioStream.end(); decoderOn = false; }
  if (pcmBuf) {
    StreamBufferHandle_t b = pcmBuf;
    pcmBuf = nullptr;
    delay(20);
    vStreamBufferDelete(b);
  }
  if (i2sOn) { i2sOutput.end(); i2sOn = false; }
  audioReady = false;
  paused = false;
  setTitle("");
  delay(100);
}

// =====================================================================
//  Output modes
// =====================================================================
void showBusy(const char *msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor((128 - (int)strlen(msg) * 6) / 2, 28);
  display.print(msg);
  showFrame();
}

void startI2SMode(OutMode mode) {
  outMode = mode;
  if (mode != OUT_BLUETOOTH) lastI2SOut = mode;
  paused = false;
  if (initAudio(mode)) {
    audioActive = true;
    currentAppState = STATE_MAIN_PLAYER;
  } else {
    stopAudio();
    menuReturnState = STATE_MAIN_PLAYER;
    snprintf(menuNotice, sizeof(menuNotice), "Audio init failed");
    menuNoticeUntil = millis() + 4000;
    menuSelection = 1;
    currentAppState = STATE_MODE_MENU;
  }
}

void switchToI2S(OutMode target) {
  showBusy("Switching...");
  portalOut();
  outMode = target;
  audioActive = false;
  connectedFlag = false;
  connectFailedFlag = false;
  disconnectedFlag = false;
  mediaStartedFlag = false;
  hasTarget = false;
  a2dp_source.end();
  delay(300);
  stopAudio();
  if (!nameCachePool) buildNameCache(trackCount);   // plenty of heap again without Bluetooth
  startI2SMode(target);
  portalIn();
}

void switchToBluetooth() {
  showBusy("Starting BT");
  portalOut();
  stopAudio();
  connectedFlag = false;
  connectFailedFlag = false;
  disconnectedFlag = false;
  mediaStartedFlag = false;
  xSemaphoreTake(listMutex, portMAX_DELAY);
  hasTarget = false;
  deviceCount = 0;
  btMenuSelection = 0;
  xSemaphoreGive(listMutex);
  freeNameCache();   // give the RAM back to the Bluetooth stack
  outMode = OUT_BLUETOOTH;
  currentAppState = STATE_BT_SCANNER;
  startBluetooth();
  portalIn();
}

// Title / artist from the phone -> player screen (the OLED font is ASCII only)
void composeRxTitle() {
  char a[48], b[64], t[100];
  strlcpy(a, rxArtist, sizeof(a));
  strlcpy(b, rxTitle, sizeof(b));
  char *bufs[2] = { a, b };
  for (int k = 0; k < 2; k++) {
    char *src = bufs[k], *dst = bufs[k];
    for (; *src; src++) {
      unsigned char c = (unsigned char)*src;
      if (c >= 0xC0) *dst++ = '?';              // start of a non-ASCII character
      else if (c >= 0x80) continue;             // its continuation bytes
      else if (c >= 32 && c != 127) *dst++ = (char)c;
    }
    *dst = '\0';
  }
  if (a[0] && b[0]) snprintf(t, sizeof(t), "%s - %s", a, b);
  else snprintf(t, sizeof(t), "%s", b[0] ? b : a);
  setTitle(String(t));
}

// Receiver mode: start the Bluetooth sink. The audio pipeline is built later, from loop(), once the phone
// has connected and started streaming (the Bluetooth stack allocates its own buffers first, v7 lesson).
void startReceiver(OutMode out) {
  inSrc = SRC_PHONE;
  outMode = out;
  lastI2SOut = out;
  rxConnected = false;
  rxTitle[0] = '\0';
  rxArtist[0] = '\0';
  rxStartedAt = 0;
  rxConnectedAt = 0;
  rxInitTried = false;
  rxVolPending = false;
  paused = true;                                 // until the phone starts streaming
  setTitle("");
  currentAppState = STATE_MAIN_PLAYER;
  startSink();
  Serial.printf("Receiver mode: waiting for a phone to pair with \"%s\" (free heap %u)\n", RX_NAME, (unsigned)ESP.getFreeHeap());
}

// SD card <-> receiver mode restarts the ESP32 so the Bluetooth stack gets a clean heap.
// bm bit 0 = receiver mode (bit 2 = quick restart, added below). setup() reads it once on the next boot.
void rebootInto(int bm) {
  audioActive = false;
  if (inSrc == SRC_SD && playCount > 0) saveState();   // song, playlist and shuffle for the way back
  showBusy("Switching...");
  if (prefs.begin(PREFS_NS, false)) { prefs.putInt("bm", bm | 4); prefs.end(); }   // bit 2 = quick restart (no splash)
  delay(40);
  ESP.restart();
}

void openModeMenu(AppState from) {
  portalOut();
  menuReturnState = from;
  menuEditTimeout = false;
  menuSelection   = (outMode == OUT_BLUETOOTH) ? 1 : 0;
  menuNoticeUntil = 0;
  currentAppState = STATE_MODE_MENU;
  portalIn();
}

void openPlaylistMenu() {
  if (!playlistsEnabled || trackCount == 0) {
    setPlNotice("Playlists off");
    return;
  }
  portalOut();
  plMenuSel = activePlaylistIndex();
  menuNoticeUntil = 0;
  currentAppState = STATE_PLAYLIST_MENU;
  portalIn();
}

static const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "POWER-ON";
    case ESP_RST_EXT:      return "EXTERNAL PIN";
    case ESP_RST_SW:       return "SOFTWARE RESTART";
    case ESP_RST_PANIC:    return "PANIC (crash)";
    case ESP_RST_INT_WDT:  return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK WATCHDOG";
    case ESP_RST_WDT:      return "OTHER WATCHDOG";
    case ESP_RST_DEEPSLEEP:return "DEEP SLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT (power dipped)";
    default:               return "OTHER";
  }
}

// =====================================================================
//  Setup
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.printf("\n=== MPD v31 boot | reset reason: %s | free heap %u ===\n", resetReasonName(esp_reset_reason()), (unsigned)ESP.getFreeHeap());
  setCpuFrequencyMhz(240);
  for (int i = 0; i < 100; i++) toneTable[i] = (int16_t)(sinf(2.0f * 3.14159265f * i / 100.0f) * 8000.0f);
  listMutex  = xSemaphoreCreateMutex();
  titleMutex = xSemaphoreCreateMutex();
  cmdQueue   = xQueueCreate(8, sizeof(int8_t));

  btnUp.begin(PIN_UP);
  btnDown.begin(PIN_DOWN);
  btnEnter.begin(PIN_ENTER);
  btnEnc.begin(PIN_ENC_SW);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  volumeEncoder.attachHalfQuad(PIN_ENC_CLK, PIN_ENC_DT);
  volumeEncoder.setCount(volumePercent / VOL_PER_COUNT);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for (;;);
  }
  display.setTextWrap(false);
  
  // One-shot boot request left by the menu (SD card <-> receiver): bit 0 = receiver mode
  int bm = 0;
  if (prefs.begin(PREFS_NS, false)) {
    bm = prefs.getInt("bm", 0);
    if (bm) prefs.putInt("bm", 0);
    screenTimeoutMin = prefs.getInt("scr", 5);
    if (screenTimeoutMin < 0 || screenTimeoutMin > 30) screenTimeoutMin = 5;
    prefs.end();
  }
  bool rxBoot = (bm & 1) != 0;
  bool quick  = (bm & 4) != 0;          // restart requested by the menu
  quickBoot = quick;
  OutMode bootOut = OUT_HEADPHONES;

  if (quick) showBusy(rxBoot ? "Receiver mode" : "SD card mode");   // quick restart: no long splash
  else       splashStart();                                          // Portal FX boot animation

  if (rxBoot) {
    // Receiver mode needs no SD card: leave all the RAM to the Bluetooth stack
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    startReceiver(bootOut);
  } else {
    splashStage("MOUNTING STORAGE", 30);
    uint32_t tBoot = millis();
    if (!SD.begin(SD_CS_PIN, SPI, 10000000) && !SD.begin(SD_CS_PIN)) {
      display.clearDisplay();
      display.setCursor(0, 24);
      display.print(F("SD card failed!"));
      showFrame();
      delay(2000);
    } else {
      uint32_t t1 = millis();
      buildNameCache(0);                                         // ONE folder pass: song names + count
      trackCount = nameCachePool ? nameCacheCount : countSongs();
      uint32_t t2 = millis();
      loadState();
      uint32_t t3 = millis();
      scanPlaylists();
      uint32_t t4 = millis();
      Serial.printf("Boot timing: SD mount %u ms | count songs %u ms | load state (names, fingerprints, playlist) %u ms | scan playlists %u ms\n",
                    (unsigned)(t1 - tBoot), (unsigned)(t2 - t1), (unsigned)(t3 - t2), (unsigned)(t4 - t3));
    }

    splashStage("CALIBRATING EMITTERS", 70);
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    startI2SMode(bootOut);
  }

  splashFinish(); // Finishes boot animation and transitions smoothly to main screen
}

// =====================================================================
//  Loop (core 1, UI only)
// =====================================================================
void loop() {
  btnUp.update();
  btnDown.update();
  btnEnter.update();
  btnEnc.update();

  {
    long c = volumeEncoder.getCount();
    const long maxC = 100 / VOL_PER_COUNT;
    if (c < 0) { volumeEncoder.setCount(0); c = 0; }
    else if (c > maxC) { volumeEncoder.setCount(maxC); c = maxC; }
    int v = (int)(c * VOL_PER_COUNT);
    if (v != volumePercent) {
      volumePercent = v;
      volGain = ((uint32_t)v * (uint32_t)v * 256u) / 10000u;
    }
  }

  {   // screen saver
    static long lastEncSeen = 0;
    long encNow = volumeEncoder.getCount();
    bool encMoved = (encNow != lastEncSeen);
    lastEncSeen = encNow;
    bool keyPressed = btnUp.pressEvt || btnDown.pressEvt || btnEnter.pressEvt || btnEnc.pressEvt;
    if (screenOff) {
      if (keyPressed || encMoved) {
        display.ssd1306_command(SSD1306_DISPLAYON);
        screenOff = false;
        lastActivity = millis();
        // the key that woke the screen does nothing else, not even when it is released
        Button *all[4] = { &btnUp, &btnDown, &btnEnter, &btnEnc };
        for (int i = 0; i < 4; i++) {
          if (all[i]->down) all[i]->mute = true;
          all[i]->pressEvt = all[i]->shortEvt = all[i]->longEvt = false;
        }
      }
    } else if (keyPressed || encMoved) {
      lastActivity = millis();
    } else if (screenTimeoutMin > 0 && millis() - lastActivity > (unsigned long)screenTimeoutMin * 60000UL) {
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      screenOff = true;
      Serial.printf("Screen off (%d min without a key press)\n", screenTimeoutMin);
    }
  }

  {
    static unsigned long lastStat = 0;
    if (millis() - lastStat > 5000) {
      lastStat = millis();
      Serial.printf("STAT heap %u (largest %u, lowest ever %u) | free stack bytes: loop %u audio %u i2s %u\n",
                    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap(), (unsigned)esp_get_minimum_free_heap_size(),
                    (unsigned)uxTaskGetStackHighWaterMark(NULL),
                    (!audioTaskDone && audioTaskHandle) ? (unsigned)uxTaskGetStackHighWaterMark(audioTaskHandle) : 0u,
                    (!i2sTaskDone && i2sTaskHandle) ? (unsigned)uxTaskGetStackHighWaterMark(i2sTaskHandle) : 0u);
    }
  }

  if (outMode == OUT_BLUETOOTH && audioReady) {
    static unsigned long lastBt = 0;
    static uint32_t lastCalls = 0;
    if (millis() - lastBt > 5000) {
      lastBt = millis();
      uint32_t c = cbCalls;
      Serial.printf("BT OUT: speaker asked %u times/s (need ~345 at 44.1k) | frames/call %d | underruns %u | lowest PCM fill %u of %u | slowest SD+decode %u ms | heap %u\n",
                    (unsigned)((c - lastCalls) / 5), (int)lastFrameCount, (unsigned)underruns,
                    (unsigned)(minFill == 0xFFFFFFFF ? 0 : minFill), (unsigned)pcmSize, (unsigned)maxCopyMs, (unsigned)ESP.getFreeHeap());
      lastCalls = c;
    }
  }

  if (rxTitleDirty) {
    rxTitleDirty = false;
    if (inSrc == SRC_PHONE) composeRxTitle();
  }

  if (inSrc == SRC_PHONE) {
    // Build the audio pipeline once the phone is connected and streaming (or 5 s after it connected).
    if (!audioReady && !rxInitTried) {
      bool go = (rxStartedAt && millis() - rxStartedAt > 400) || (rxConnectedAt && millis() - rxConnectedAt > 5000);
      if (go) {
        rxInitTried = true;
        if (initAudio(outMode)) {
          audioActive = true;
        } else {
          stopAudio();
          setPlNotice("Audio init failed");
        }
      }
    }
    // Ask the phone for the current title/artist regularly, so the screen follows track changes
    if (rxConnected && (int32_t)(millis() - rxMetaDue) >= 0) {
      rxMetaDue = millis() + 8000;   // was 3 s: every poll is extra Bluetooth (AVRC) traffic; the phone also announces track changes itself
      esp_avrc_ct_send_metadata_cmd(1, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
    }
    // Full volume on the phone side (the phone's own slider still works through Bluetooth)
    if (rxVolPending && rxConnectedAt && millis() - rxConnectedAt > 1500) {
      rxVolPending = false;
      a2dp_sink.set_volume(127);
    }
    // Status line for debugging choppy audio / low volume
    static unsigned long lastRxLog = 0;
    if (millis() - lastRxLog > 3000) {
      lastRxLog = millis();
      Serial.printf("RX: conn %d paused %d | heap %u (largest %u) | buf %u/%u | underruns %u dropped %u | rate %u | peak %d\n",
                    (int)rxConnected, (int)paused, (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap(),
                    pcmBuf ? (unsigned)xStreamBufferBytesAvailable(pcmBuf) : 0u, (unsigned)pcmSize,
                    (unsigned)underruns, (unsigned)pcmDropped, (unsigned)srcRate, (int)rxPeak);
      {
        static uint32_t lastB = 0, lastC = 0;
        uint32_t b = rxBytes, c = rxCalls;
        Serial.printf("   RX flow: %u B/s in (need ~%u) | %u packets/s | longest gap %u ms | lowest buffer fill %u | discards %u\n",
                      (unsigned)((b - lastB) / 3), (unsigned)(srcRate * 4), (unsigned)((c - lastC) / 3), (unsigned)rxMaxGapMs,
                      (unsigned)(rxMinFill == 0xFFFFFFFF ? 0 : rxMinFill), (unsigned)rxDiscards);
        lastB = b; lastC = c; rxMaxGapMs = 0; rxMinFill = 0xFFFFFFFF;
      }
      rxPeak = 0;
    }
  }

  if (disconnectedFlag) {
    disconnectedFlag = false;
    if (outMode == OUT_BLUETOOTH) goBackToScanner();
  }

  if (connectFailedFlag) {
    connectFailedFlag = false;
    connectedFlag = false;
    if (outMode == OUT_BLUETOOTH) goBackToScanner();
  }

  if (connectedFlag) {
    if (currentAppState != STATE_CONNECTING) {
      connectedFlag = false;
    } else if (mediaStartedFlag || millis() - connectedAt > 5000) {
      connectedFlag = false;
      if (!audioReady && !initAudio(OUT_BLUETOOTH)) {
        stopAudio();
        goBackToScanner();
      } else {
        paused = false;
        audioActive = true;
        portalOut();
        currentAppState = STATE_MAIN_PLAYER;
        portalIn();
      }
    }
  }

  switch (currentAppState) {

    case STATE_BT_SCANNER: {
      int totalItems = deviceCount + 1;
      {
        static unsigned long lastScanLog = 0;
        if (millis() - lastScanLog > 4000) {
          lastScanLog = millis();
          Serial.printf("SCAN: %d device(s) | heap %u (largest %u)\n", deviceCount, (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
        }
        if (deviceCount == 0 && scanStartedAt && millis() - scanStartedAt > 25000) {
          Serial.println("SCAN: nothing found for 25 s, restarting Bluetooth discovery");
          goBackToScanner();
          break;
        }
      }
      if (btnEnter.longEvt) { openModeMenu(STATE_BT_SCANNER); break; }
      if (btnUp.pressEvt)   { startRipple(15, 16); btMenuSelection = (btMenuSelection - 1 + totalItems) % totalItems; }
      if (btnDown.pressEvt) { startRipple(15, 36); btMenuSelection = (btMenuSelection + 1 + totalItems) % totalItems; }
      if (btnEnter.shortEvt) {
        startRipple(64, 32);
        if (btMenuSelection >= deviceCount) {
          Serial.println("SCAN: rescan requested");
          goBackToScanner();                 // really restart discovery (clearing the list alone did not)
          break;
        } else {
          xSemaphoreTake(listMutex, portMAX_DELAY);
          selectedDeviceName = discoveredDevices[btMenuSelection].name;
          memcpy(targetAddress, discoveredDevices[btMenuSelection].address, sizeof(esp_bd_addr_t));
          hasTarget = true;
          xSemaphoreGive(listMutex);
          connectStartTime = millis();
          portalOut();
          currentAppState = STATE_CONNECTING;
          portalIn();
        }
      }
      static unsigned long lastDraw = 0;
      if (millis() - lastDraw > 100) { lastDraw = millis(); drawBTScannerMenu(); }
      break;
    }

    case STATE_CONNECTING: {
      unsigned long elapsed = millis() - connectStartTime;
      if (elapsed > CONNECT_TIMEOUT || btnEnter.longEvt || btnEnter.shortEvt) {
        hasTarget = false;
        portalOut();
        currentAppState = STATE_BT_SCANNER;
        portalIn();
        break;
      }
      float progress = 0.9f * (1.0f - expf(-(float)elapsed / 8000.0f));
      static unsigned long lastDraw = 0;
      if (millis() - lastDraw > 50) { lastDraw = millis(); drawConnectingScreen(selectedDeviceName, progress); }
      break;
    }

    case STATE_MAIN_PLAYER: {
      if (inSrc == SRC_PHONE) {          // receiver mode: the buttons remote-control the phone
        if (btnUp.pressEvt)    { startRipple(28, 42); if (rxConnected) { a2dp_sink.next(); rxMetaDue = millis() + 500; } }
        if (btnDown.pressEvt)  { startRipple(47, 42); if (rxConnected) { a2dp_sink.previous(); rxMetaDue = millis() + 500; } }
        if (btnEnter.shortEvt) { startRipple(63, 42); if (rxConnected) { if (paused) a2dp_sink.play(); else a2dp_sink.pause(); } }
      } else {
        if (btnUp.pressEvt)    { startRipple(28, 42); sendCmd(CMD_NEXT); }
        if (btnDown.pressEvt)  { startRipple(47, 42); sendCmd(CMD_PREV); }
        if (btnEnter.shortEvt) { startRipple(63, 42); sendCmd(CMD_TOGGLE_PAUSE); }
        if (btnEnc.shortEvt)   { startRipple(81, 42); sendCmd(CMD_TOGGLE_SHUFFLE); }
        if (btnEnc.longEvt)    { openPlaylistMenu(); break; }
      }
      if (btnEnter.longEvt)  { openModeMenu(STATE_MAIN_PLAYER); break; }

      static unsigned long lastDraw = 0;
      unsigned long interval = fxActive() ? ((outMode == OUT_BLUETOOTH) ? 40 : 30) : 120;   // Bluetooth: give the CPU a bit more room
      if (millis() - lastDraw > interval) {
        lastDraw = millis();
        if (!paused && !fxActive()) { waveOffset += 0.3; if (waveOffset > 6.28) waveOffset -= 6.28; }
        drawMainPlayerUI();
      }
      break;
    }

    case STATE_MODE_MENU: {
      if (btnUp.pressEvt) {
        startRipple(15, 20);
        if (menuEditTimeout) { if (screenTimeoutMin < 30) screenTimeoutMin++; }          // +1 minute
        else menuSelection = (menuSelection + MODE_ITEMS - 1) % MODE_ITEMS;
      }
      if (btnDown.pressEvt) {
        startRipple(15, 40);
        if (menuEditTimeout) { if (screenTimeoutMin > 0) screenTimeoutMin--; }           // -1 minute (0 = never)
        else menuSelection = (menuSelection + 1) % MODE_ITEMS;
      }
      if (btnEnter.longEvt) {
        if (menuEditTimeout) { menuEditTimeout = false; saveScreenTimeout(); }
        portalOut(); currentAppState = menuReturnState; portalIn(); break;
      }
      if (btnEnter.shortEvt && menuEditTimeout) {            // confirm the new timeout
        menuEditTimeout = false;
        saveScreenTimeout();
        lastActivity = millis();
        startRipple(64, 32);
      } else if (btnEnter.shortEvt && menuSelection == 3) { // start editing the timeout
        menuEditTimeout = true;
        startRipple(64, 32);
      } else if (btnEnter.shortEvt) {
        startRipple(64, 32);
        portalOut();
        switch (menuSelection) {
          case 0:
            if (outMode == OUT_HEADPHONES) currentAppState = menuReturnState;
            else switchToI2S(OUT_HEADPHONES);
            break;
          case 1:
            if (inSrc == SRC_PHONE) {        // a receiver can't relay to a Bluetooth speaker (one A2DP role at a time)
              snprintf(menuNotice, sizeof(menuNotice), "Needs SD card mode");
              menuNoticeUntil = millis() + 2500;
            }
            else if (outMode != OUT_BLUETOOTH) switchToBluetooth();
            else if (menuReturnState == STATE_MAIN_PLAYER) goBackToScanner();
            else currentAppState = menuReturnState;
            break;
          case 2:
            if (inSrc == SRC_PHONE) rebootInto(0);          // back to the SD card (restarts)
            else rebootInto(1);                             // receiver mode (restarts)
            break;
          case 4:
            currentAppState = menuReturnState;
            break;
        }
        portalIn();
        break;
      }
      static unsigned long lastDraw = 0;
      if (millis() - lastDraw > 100) { lastDraw = millis(); drawModeMenu(); }
      break;
    }

    case STATE_PLAYLIST_MENU: {
      if (btnUp.pressEvt)   { startRipple(15, 20); plMenuSel = (plMenuSel + playlistCount - 1) % playlistCount; }
      if (btnDown.pressEvt) { startRipple(15, 40); plMenuSel = (plMenuSel + 1) % playlistCount; }
      if (btnEnter.longEvt || btnEnc.longEvt) { portalOut(); currentAppState = STATE_MAIN_PLAYER; portalIn(); break; }
      if (btnEnter.shortEvt) {
        startRipple(64, 32);
        portalOut();
        if (plMenuSel == activePlaylistIndex()) {
          currentAppState = STATE_MAIN_PLAYER;
        } else if (plMenuSel > 0 && playlists[plMenuSel].count == 0) {
          snprintf(menuNotice, sizeof(menuNotice), "No songs matched");
          menuNoticeUntil = millis() + 3000;
        } else {
          pendingPlaylist = plMenuSel;
          sendCmd(CMD_SET_PLAYLIST);
          currentAppState = STATE_MAIN_PLAYER;
        }
        portalIn();
        break;
      }
      static unsigned long lastDraw = 0;
      if (millis() - lastDraw > 100) { lastDraw = millis(); drawPlaylistMenu(); }
      break;
    }
  }

  delay(2);
}

// =====================================================================
//  Screens
// =====================================================================
void drawBTScannerMenu() {
  String names[MAX_DISCOVERED_DEVICES];
  int count;
  xSemaphoreTake(listMutex, portMAX_DELAY);
  count = deviceCount;
  for (int i = 0; i < count; i++) names[i] = discoveredDevices[i].name;
  xSemaphoreGive(listMutex);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.print(F(">>> BT PROTOCOL <<<"));
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  int totalItems = count + 1;
  if (btMenuSelection >= totalItems) btMenuSelection = totalItems - 1;
  int visible = 4;
  int startIdx = 0;
  if (btMenuSelection >= visible) startIdx = btMenuSelection - visible + 1;

  for (int i = 0; i < visible; i++) {
    int itemIndex = startIdx + i;
    if (itemIndex >= totalItems) break;
    int yPos = 12 + (i * 10);
    bool sel = (btMenuSelection == itemIndex);
    uint16_t fg = sel ? SSD1306_BLACK : SSD1306_WHITE;

    if (sel) display.fillRect(0, yPos - 1, 128, 9, SSD1306_WHITE);
    display.setTextColor(fg);
    display.drawBitmap(2, yPos, sel ? bitmap_selector : bitmap_unselected, 6, 7, fg);

    if (itemIndex == count) {
      display.drawBitmap(11, yPos, bitmap_icon_back, 7, 7, fg);
      display.setCursor(22, yPos);
      display.print(F("[ Rescan ]"));
    } else {
      display.drawBitmap(11, yPos, bitmap_icon_headphone, 7, 7, fg);
      display.setCursor(22, yPos);
      display.print(formatString(names[itemIndex], 16));
    }
  }

  display.setTextColor(SSD1306_WHITE);
  if (count == 0) {
    drawBallStrip(42, millis());
  }

  display.drawLine(0, 52, 127, 52, SSD1306_WHITE);
  display.setCursor(2, 55);
  display.print(F("NAV ENT:GO HOLD:MENU"));
  rippleDraw();
  showFrame();
}

void drawConnectingScreen(String deviceName, float progress) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(18, 2);
  display.print(F(":: BLUETOOTH LINK ::"));
  display.drawLine(0, 12, 127, 12, SSD1306_WHITE);

  String linkText = "LINK: " + formatString(deviceName, 12);
  int centerX = (128 - (int)linkText.length() * 6) / 2;
  if (centerX < 0) centerX = 0;
  display.setCursor(centerX, 17);
  display.print(linkText);

  display.drawRect(14, 30, 100, 10, SSD1306_WHITE);
  display.fillRect(16, 32, (int)(96 * progress), 6, SSD1306_WHITE);

  String subText = "HANDSHAKE ACTIVE";
  display.setCursor((128 - (int)subText.length() * 6) / 2, 45);
  display.print(subText);
  rippleDraw();
  showFrame();
}

void drawVolumeIndicator() {
  int vol = volumePercent;
  if (vol < 0) vol = 0;
  if (vol > 100) vol = 100;
  char tag[16];
  snprintf(tag, sizeof(tag), "[Vol: %d]", vol);
  display.setCursor(128 - (int)strlen(tag) * 6, 54);
  display.print(tag);
}

void drawMainPlayerUI() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  char title[96];
  xSemaphoreTake(titleMutex, portMAX_DELAY);
  strncpy(title, currentTitle, sizeof(title) - 1);
  title[sizeof(title) - 1] = '\0';
  xSemaphoreGive(titleMutex);
  if (inSrc == SRC_PHONE) {
    if (!rxConnected) strcpy(title, "Waiting for phone...");
    else if (title[0] == '\0') strcpy(title, "Phone audio");
  }
  else if (trackCount == 0) strcpy(title, "No MP3 files");
  else if (title[0] == '\0') strcpy(title, "Loading...");

  songChangeCheck(title);

  int titleW = (int)strlen(title) * 6;
  const bool shufShown = shuffleOn && inSrc == SRC_SD;
  const int titleAreaW = shufShown ? 101 : 112;
  if (titleW <= titleAreaW) {
    display.setCursor(0, 2);
    display.print(title);
  } else {
    int cycle = titleW + 30;
    int off = (millis() / 40) % cycle;
    display.setCursor(-off, 2);
    display.print(title);
    display.setCursor(cycle - off, 2);
    display.print(title);
    display.fillRect(titleAreaW, 0, 128 - titleAreaW, 13, SSD1306_BLACK);
  }
  if (outMode == OUT_BLUETOOTH)     display.drawBitmap(118, 0, bt_icon, 8, 12, SSD1306_WHITE);
  else                              display.drawBitmap(118, 2, bitmap_icon_headphone, 7, 7, SSD1306_WHITE);
  if (shufShown) display.drawBitmap(105, 2, shuffle_icon, 9, 8, SSD1306_WHITE);

  if (!songAnimDraw()) {
    int waveCenterY = 25;
    for (int x = 0; x < 128; x++) {
      int y = waveCenterY + (int)(sin((x * 0.1) + waveOffset) * 4) + (int)(cos((x * 0.05) - waveOffset) * 2);
      display.drawPixel(x, y, SSD1306_WHITE);
    }
  }

  display.fillRect(24, 38, 2, 8, SSD1306_WHITE);
  display.fillTriangle(30, 38, 26, 42, 30, 46, SSD1306_WHITE);
  display.fillTriangle(35, 38, 31, 42, 35, 46, SSD1306_WHITE);
  display.fillTriangle(50, 38, 44, 42, 50, 46, SSD1306_WHITE);
  if (!paused) {
    display.fillRect(60, 38, 3, 8, SSD1306_WHITE);
    display.fillRect(65, 38, 3, 8, SSD1306_WHITE);
  } else {
    display.fillTriangle(60, 38, 66, 42, 60, 46, SSD1306_WHITE);
  }
  display.fillTriangle(78, 38, 84, 42, 78, 46, SSD1306_WHITE);
  display.fillTriangle(93, 38, 97, 42, 93, 46, SSD1306_WHITE);
  display.fillTriangle(98, 38, 102, 42, 98, 46, SSD1306_WHITE);
  display.fillRect(103, 38, 2, 8, SSD1306_WHITE);

  int vol = volumePercent;
  if (vol < 0) vol = 0;
  if (vol > 100) vol = 100;
  char vtag[16];
  snprintf(vtag, sizeof(vtag), "[Vol: %d]", vol);
  int vx = 128 - (int)strlen(vtag) * 6;

  char pl[32];
  xSemaphoreTake(titleMutex, portMAX_DELAY);
  strncpy(pl, (inSrc == SRC_PHONE) ? (rxConnected ? "Phone" : "Pair: " RX_NAME) : ((millis() < plNoticeUntil) ? plNotice : plActiveName), sizeof(pl) - 1);
  pl[sizeof(pl) - 1] = '\0';
  xSemaphoreGive(titleMutex);

  const int areaX = 11;
  const int areaW = vx - 4 - areaX;
  int plW = (int)strlen(pl) * 6;
  if (plW <= areaW) {
    display.setCursor(areaX, 54);
    display.print(pl);
  } else {
    int cycle = plW + 30;
    int off = (millis() / 50) % cycle;
    display.setCursor(areaX - off, 54);
    display.print(pl);
    display.setCursor(areaX + cycle - off, 54);
    display.print(pl);
    display.fillRect(0, 52, areaX, 12, SSD1306_BLACK);
    display.fillRect(areaX + areaW, 52, 128 - (areaX + areaW), 12, SSD1306_BLACK);
  }
  display.drawBitmap(0, 54, bitmap_icon_playlist, 7, 7, SSD1306_WHITE);
  drawVolumeIndicator();

  rippleDraw();
  showFrame();
}

void drawModeMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(13, 0);
  display.print(F(":: SELECT MODE ::"));
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  const char *itemNames[MODE_ITEMS] = { "EARPIECE MODE", "BLUETOOTH MODE", "", "", "CANCEL" };
  itemNames[2] = (inSrc == SRC_PHONE) ? "SD CARD MODE" : "RECEIVER MODE";
  char scrBuf[18];
  if (screenTimeoutMin == 0) {
    if (menuEditTimeout) snprintf(scrBuf, sizeof(scrBuf), "SCREEN:<NEVER>");
    else                 snprintf(scrBuf, sizeof(scrBuf), "SCREEN: NEVER");
  } else {
    if (menuEditTimeout) snprintf(scrBuf, sizeof(scrBuf), "SCREEN:<%d MIN>", screenTimeoutMin);
    else                 snprintf(scrBuf, sizeof(scrBuf), "SCREEN: %d MIN", screenTimeoutMin);
  }
  itemNames[3] = scrBuf;

  const int visible = 4;
  const int startY = 13;
  const int itemHeight = 10;
  int start = 0;
  if (menuSelection >= visible) start = menuSelection - visible + 1;

  for (int i = 0; i < visible; i++) {
    int idx = start + i;
    if (idx >= MODE_ITEMS) break;
    int yPos = startY + (i * itemHeight);
    bool sel = (menuSelection == idx);
    uint16_t fg = sel ? SSD1306_BLACK : SSD1306_WHITE;

    if (sel) display.fillRect(0, yPos - 1, 128, 10, SSD1306_WHITE);
    display.setTextColor(fg);

    display.drawBitmap(2, yPos, sel ? bitmap_selector : bitmap_menu_unselected, 8, 7, fg);

    const unsigned char *icon = bitmap_icon_cancel;
    if (idx == 0) icon = bitmap_icon_earpiece;
    else if (idx == 1) icon = bitmap_icon_bluetooth;
    else if (idx == 2) icon = (inSrc == SRC_PHONE) ? bitmap_icon_sd : bitmap_icon_phone;
    else if (idx == 3) icon = bitmap_icon_screen;
    display.drawBitmap(13, yPos, icon, 8, 7, fg);

    display.setCursor(24, yPos);
    display.print(itemNames[idx]);

    bool active = (idx == 0 && outMode == OUT_HEADPHONES) || (idx == 1 && outMode == OUT_BLUETOOTH);
    if (active) display.fillCircle(120, yPos + 3, 2, fg);
  }

  display.setTextColor(SSD1306_WHITE);
  display.drawLine(0, 53, 127, 53, SSD1306_WHITE);
  display.setCursor(2, 55);
  if (millis() < menuNoticeUntil) display.print(menuNotice);
  else display.print(menuEditTimeout ? F("UP/DN:MIN  ENT:SAVE") : F("UP/DN:NAV  ENT:SELECT"));
  rippleDraw();
  showFrame();
}

void drawPlaylistMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(19, 0);
  display.print(F(":: PLAYLISTS ::"));
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  const int visible = 4;
  const int startY = 13;
  const int itemHeight = 10;
  int start = 0;
  if (plMenuSel >= visible) start = plMenuSel - visible + 1;
  int activeIdx = activePlaylistIndex();

  for (int i = 0; i < visible; i++) {
    int idx = start + i;
    if (idx >= playlistCount) break;
    int yPos = startY + (i * itemHeight);
    bool sel = (idx == plMenuSel);
    uint16_t fg = sel ? SSD1306_BLACK : SSD1306_WHITE;

    if (sel) display.fillRect(0, yPos - 1, 128, 10, SSD1306_WHITE);
    display.setTextColor(fg);

    if (sel) display.drawBitmap(2, yPos, bitmap_selector, 8, 7, fg);
    else if (idx == activeIdx) display.fillRect(3, yPos + 1, 5, 5, fg);
    else display.drawBitmap(2, yPos, bitmap_menu_unselected, 8, 7, fg);

    char cnt[8];
    snprintf(cnt, sizeof(cnt), "%d", (int)playlists[idx].count);
    int cw = (int)strlen(cnt) * 6;

    char name[32];
    plDisplayName(idx, name, sizeof(name));
    int maxChars = (126 - cw - 13 - 4) / 6;
    if ((int)strlen(name) > maxChars && maxChars >= 3) {
      name[maxChars - 2] = '\0';
      strcat(name, "..");
    }
    display.setCursor(13, yPos);
    display.print(name);
    display.setCursor(127 - cw, yPos);
    display.print(cnt);
  }

  display.setTextColor(SSD1306_WHITE);
  display.drawLine(0, 53, 127, 53, SSD1306_WHITE);
  display.setCursor(2, 55);
  if (millis() < menuNoticeUntil) display.print(menuNotice);
  else if (playlistCount <= 1) display.print(F("No .m3u files found"));
  else display.print(F("ENT:PLAY  HOLD:BACK"));
  rippleDraw();
  showFrame();
}

// =====================================================================
//  PORTAL FX (v24)
// =====================================================================
static const unsigned char PROGMEM fx_heart[] = { 0x6C, 0xFE, 0xFE, 0x7C, 0x38, 0x10 };

static void fxOvalFill(int cx, int cy, int rx, int ry, uint16_t color) {
  if (rx < 1 || ry < 1) return;
  for (int dy = -ry; dy <= ry; dy++) {
    float t = (float)dy / (float)ry;
    int w = (int)(rx * sqrtf(1.0f - t * t) + 0.5f);
    display.drawFastHLine(cx - w, cy + dy, 2 * w + 1, color);
  }
}

static void fxOvalRing(int cx, int cy, int rx, int ry, uint16_t color) {
  if (rx < 1 || ry < 1) return;
  int px = cx, py = cy - ry;
  for (int i = 1; i <= 28; i++) {
    float a = -1.5708f + i * 6.2832f / 28.0f;
    int x = cx + (int)lroundf(rx * cosf(a));
    int y = cy + (int)lroundf(ry * sinf(a));
    display.drawLine(px, py, x, y, color);
    px = x; py = y;
  }
}

static void fxIris(int cx, int cy, int r, float open, float rot) {
  display.drawCircle(cx, cy, r, SSD1306_WHITE);
  float ri = r * open;
  for (int k = 0; k < 6; k++) {
    float a0 = rot + k * 1.0472f;
    float a1 = a0 + 1.0472f;
    int x0 = cx + (int)lroundf(ri * cosf(a0)), y0 = cy + (int)lroundf(ri * sinf(a0));
    int x1 = cx + (int)lroundf(ri * cosf(a1)), y1 = cy + (int)lroundf(ri * sinf(a1));
    display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
    float ao = a0 + 1.9f;
    int xo = cx + (int)lroundf(r * cosf(ao)), yo = cy + (int)lroundf(r * sinf(ao));
    display.drawLine(x0, y0, xo, yo, SSD1306_WHITE);
  }
}

static void fxPortal(int cx, int cy, int rx, int ry, bool solid) {
  if (ry < 1) return;
  if (solid) {
    fxOvalFill(cx, cy, rx, ry, SSD1306_WHITE);
    if (ry > 4) fxOvalRing(cx, cy, rx - 1, ry - 2, SSD1306_BLACK);
  } else {
    fxOvalRing(cx, cy, rx, ry, SSD1306_WHITE);
    if (ry > 4 && rx > 2) fxOvalRing(cx, cy, rx - 1, ry - 2, SSD1306_WHITE);
  }
}

static void fxCube(int cx, int cy, int s) {
  display.drawRect(cx - s, cy - s, 2 * s + 1, 2 * s + 1, SSD1306_WHITE);
  display.drawRect(cx - s + 2, cy - s + 2, 2 * s - 3, 2 * s - 3, SSD1306_WHITE);
  display.drawPixel(cx - s + 1, cy - s + 1, SSD1306_WHITE);
  display.drawPixel(cx + s - 1, cy - s + 1, SSD1306_WHITE);
  display.drawPixel(cx - s + 1, cy + s - 1, SSD1306_WHITE);
  display.drawPixel(cx + s - 1, cy + s - 1, SSD1306_WHITE);
  if (s >= 6) display.drawBitmap(cx - 3, cy - 3, fx_heart, 7, 6, SSD1306_WHITE);
}

static void fxBallScene(int xl, int xr, int cy, int ry, int apex, uint32_t ms, uint32_t period, bool leftToRight) {
  fxPortal(xl, cy, 3, ry, false);
  fxPortal(xr, cy, 3, ry, true);
  float t = (float)(ms % period) / (float)period;
  for (int k = 4; k >= 0; k--) {
    float tt = t - 0.035f * k;
    if (tt < 0) continue;
    float p = leftToRight ? tt : 1.0f - tt;
    int bx = xl + 4 + (int)((xr - xl - 8) * p);
    int by = cy - (int)(4.0f * apex * tt * (1.0f - tt));
    if (k == 0) display.fillCircle(bx, by, 2, SSD1306_WHITE);
    else if (k < 3) display.drawPixel(bx, by, SSD1306_WHITE);
    else display.drawPixel(bx, by + 1, SSD1306_WHITE);
  }
}

void drawBallStrip(int cy, uint32_t ms) {
  fxBallScene(14, 114, cy, 7, 5, ms, 1700, true);
}

static float fxOverlayR = 0.0f;

static void fxDrawOvalOverlay(float r) {
  int rx = (int)(r * 96.0f), ry = (int)(r * 48.0f);
  if (rx < 1 || ry < 1) return;
  fxOvalFill(64, 32, rx + 2, ry + 2, SSD1306_BLACK);
  fxOvalFill(64, 32, rx, ry, SSD1306_WHITE);
  if (r < 0.95f) {
    fxOvalRing(64, 32, (int)(rx * 0.72f), (int)(ry * 0.72f), SSD1306_BLACK);
    fxOvalRing(64, 32, (int)(rx * 0.42f), (int)(ry * 0.42f), SSD1306_BLACK);
  }
}

void showFrame() {
  if (screenOff) return;     // panel is off: skip the I2C transfer
  if (fxOverlayR > 0.01f) fxDrawOvalOverlay(fxOverlayR);
  display.display();
}

static void fxRenderCurrentScreen() {
  switch (currentAppState) {
    case STATE_BT_SCANNER:   drawBTScannerMenu(); break;
    case STATE_CONNECTING: {
      float p = 0.9f * (1.0f - expf(-(float)(millis() - connectStartTime) / 8000.0f));
      drawConnectingScreen(selectedDeviceName, p);
      break;
    }
    case STATE_MAIN_PLAYER:  drawMainPlayerUI(); break;
    case STATE_MODE_MENU:    drawModeMenu(); break;
    case STATE_PLAYLIST_MENU: drawPlaylistMenu(); break;
  }
}

void portalOut() {
  for (int i = 1; i <= 6; i++) {
    fxDrawOvalOverlay(i / 6.0f);
    display.display();
  }
}

void portalIn() {
  for (int i = 5; i >= 0; i--) {
    fxOverlayR = i / 6.0f;
    fxRenderCurrentScreen();
  }
  fxOverlayR = 0.0f;
}

#define FX_SCENE_MS  4200   // how long each loading scene stays on screen (was 2600)
#define FX_SCENES       5
#define SPLASH_SPEED    0.70f  // 1.0 = old speed, lower = slower splash animations
#define SPLASH_INTRO_MS 3800   // APERTURE intro length (was 2300)
#define SPLASH_STAGE_MS 520    // minimum time each loading status line stays up (was 380)

static const char *const fxTicker[FX_SCENES] = {
  "THINKING WITH PORTALS", "COMPANION CUBE ONLINE", "ARE YOU STILL THERE?", "THE CAKE IS A LIE", "FOR SCIENCE."
};

struct SplashMsg { char text[24]; int pct; int creep; };
#define SPLASH_Q 12
static SplashMsg splashQ[SPLASH_Q];
static volatile int  splashQHead = 0, splashQTail = 0;
static volatile bool splashRun = false, splashFinishReq = false, splashAbort = false, splashDone = true;
static char  splashText[24] = "";
static volatile int splashTarget = 0, splashCreep = 8;
static float splashShown = 0.0f;

static void fxSceneBall(uint32_t ms) {
  fxBallScene(9, 119, 17, 11, 12, ms, 2100, false);
}

static void fxSceneCube(uint32_t ms) {
  int bob = (int)(sinf(ms / 330.0f) * 2.5f);
  fxCube(64, 16 + bob, 9);
  int w = 9 - bob;
  display.drawFastHLine(64 - w, 31, 2 * w + 1, SSD1306_WHITE);
  display.drawFastHLine(64 - w + 2, 32, 2 * w - 3, SSD1306_WHITE);
}

static void fxSceneTurret(uint32_t ms) {
  int cx = 64, cy = 13;
  fxOvalRing(cx, cy, 5, 9, SSD1306_WHITE);
  display.fillCircle(cx, cy - 1, 2, SSD1306_WHITE);
  display.drawPixel(cx, cy - 1, SSD1306_BLACK);
  display.drawLine(cx - 5, cy - 3, cx - 13, cy - 8, SSD1306_WHITE);
  display.drawLine(cx + 5, cy - 3, cx + 13, cy - 8, SSD1306_WHITE);
  display.drawLine(cx - 3, cy + 8, cx - 7, cy + 15, SSD1306_WHITE);
  display.drawLine(cx + 3, cy + 8, cx + 7, cy + 15, SSD1306_WHITE);
  display.drawFastVLine(cx, cy + 9, 6, SSD1306_WHITE);
  float a = sinf(ms / 600.0f) * 0.8f;
  for (int r = 8; r < 70; r += 3) {
    int x = cx + (int)(r * sinf(a));
    int y = cy - 1 + (int)(r * cosf(a) * 0.5f);
    if (y >= 0 && y < 34 && x >= 0 && x < 128) display.drawPixel(x, y, SSD1306_WHITE);
  }
}

static void fxSceneCake(uint32_t ms) {
  display.drawRect(46, 24, 36, 9, SSD1306_WHITE);
  display.drawRect(52, 17, 24, 7, SSD1306_WHITE);
  display.drawRect(58, 11, 12, 6, SSD1306_WHITE);
  for (int x = 48; x < 80; x += 4) display.drawPixel(x, 26, SSD1306_WHITE);
  display.drawFastVLine(64, 6, 5, SSD1306_WHITE);
  int f = (ms / 90) % 3;
  display.drawPixel(64, 4, SSD1306_WHITE);
  display.drawPixel(64, 3, SSD1306_WHITE);
  if (f == 1) display.drawPixel(65, 3, SSD1306_WHITE);
  if (f == 2) display.drawPixel(63, 3, SSD1306_WHITE);
  if ((ms / 200) % 2) { display.drawPixel(40, 12, SSD1306_WHITE); display.drawPixel(90, 18, SSD1306_WHITE); }
  else                { display.drawPixel(88, 10, SSD1306_WHITE); display.drawPixel(38, 20, SSD1306_WHITE); }
}

static void fxSceneIris(uint32_t ms) {
  fxIris(64, 16, 15, 0.15f + 0.55f * (0.5f + 0.5f * sinf(ms / 450.0f)), ms / 800.0f);
}

static void fxDrawIntro(uint32_t el) {
  el = (uint32_t)(el * SPLASH_SPEED);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  float open = 0.1f + 0.5f * (el > 1200 ? 1.0f : el / 1200.0f);
  fxIris(64, 12, 10, open, el * 0.0045f);
  if (el > 1100) {
    int pct = (el - 1100) / 8;
    if (pct > 100) pct = 100;
    int ry = (8 * pct) / 100;
    fxPortal(7, 32, 3, ry, false);
    fxPortal(121, 32, 3, ry, true);
  }
  if (el > 1300) {
    int n = (el - 1300) / 70;
    if (n > 8) n = 8;
    char buf[9];
    memcpy(buf, "APERTURE", 8);
    buf[n] = '\0';
    display.setTextSize(2);
    display.setCursor(16, 24);
    display.print(buf);
    if (el > 1900) {
      display.setTextSize(1);
      display.setCursor(25, 42);
      display.print(F("S C I E N C E"));
    }
  }
  display.setTextSize(1);
}

static void fxDrawLoading(uint32_t el) {
  float tgt = (float)splashTarget;
  if (splashShown < tgt) { splashShown += (tgt - splashShown) * 0.07f + 0.15f; if (splashShown > tgt) splashShown = tgt; }
  else if (splashShown < tgt + splashCreep && splashShown < 99.0f) splashShown += 0.05f;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  int scene = (el / FX_SCENE_MS) % FX_SCENES;
  uint32_t local = el % FX_SCENE_MS;
  uint32_t sm = (uint32_t)(el * SPLASH_SPEED);   // slowed clock for the scene animations
  switch (scene) {
    case 0: fxSceneBall(sm);   break;
    case 1: fxSceneCube(sm);   break;
    case 2: fxSceneTurret(sm); break;
    case 3: fxSceneCake(sm);   break;
    default: fxSceneIris(sm);  break;
  }
  if (el >= FX_SCENE_MS && local < 350) {
    int edge = (int)(128L * local / 350);
    display.fillRect(edge, 0, 128 - edge, 34, SSD1306_BLACK);
    display.drawFastVLine(edge, 0, 34, SSD1306_WHITE);
  }

  const char *q = fxTicker[scene];
  int len = (int)strlen(q);
  int shown = (int)(local / 75);
  if (shown > len) shown = len;
  char buf[24];
  memcpy(buf, q, shown);
  buf[shown] = '\0';
  int qx = (128 - len * 6) / 2;
  display.setCursor(qx, 37);
  display.print(buf);
  if (shown < len || (local / 300) % 2 == 0) display.fillRect(qx + shown * 6, 37, 5, 7, SSD1306_WHITE);

  int pct = (int)splashShown;
  if (pct > 100) pct = 100;
  display.drawRect(14, 47, 100, 5, SSD1306_WHITE);
  display.fillRect(14, 47, pct, 5, SSD1306_WHITE);
  int sl = (int)strlen(splashText);
  display.setCursor((128 - sl * 6) / 2, 55);
  display.print(splashText);
}

static void splashTask(void *arg) {
  uint32_t t0 = millis(), el;
  while (!splashAbort && (el = millis() - t0) < SPLASH_INTRO_MS) {
    fxDrawIntro(el);
    display.display();
    vTaskDelay(1);
  }
  if (!splashAbort) {
    for (int i = 1; i <= 6; i++) { fxDrawIntro(SPLASH_INTRO_MS); fxDrawOvalOverlay(i / 6.0f); display.display(); }
  }
  uint32_t L0 = millis();
  uint32_t stageAt = L0 - SPLASH_STAGE_MS;
  for (int i = 5; i >= 0 && !splashAbort; i--) {
    fxDrawLoading(0);
    fxDrawOvalOverlay(i / 6.0f);
    display.display();
  }
  while (!splashAbort) {
    uint32_t now = millis();
    if (splashQHead != splashQTail && now - stageAt >= SPLASH_STAGE_MS) {
      int tl = splashQTail;
      strncpy(splashText, splashQ[tl].text, sizeof(splashText) - 1);
      splashText[sizeof(splashText) - 1] = '\0';
      splashTarget = splashQ[tl].pct;
      splashCreep  = splashQ[tl].creep;
      splashQTail  = (tl + 1) % SPLASH_Q;
      stageAt = now;
    }
    fxDrawLoading(now - L0);
    display.display();
    if (splashFinishReq && splashQHead == splashQTail && now - stageAt >= 700) break;
    vTaskDelay(1);
  }
  if (!splashAbort) {
    for (int i = 1; i <= 6; i++) { fxDrawLoading(millis() - L0); fxDrawOvalOverlay(i / 6.0f); display.display(); }
  }
  splashDone = true;
  vTaskDelete(NULL);
}

void splashStart() {
  splashQHead = splashQTail = 0;
  splashAbort = false;
  splashFinishReq = false;
  splashDone = false;
  splashShown = 0.0f;
  splashTarget = 0;
  splashCreep = 8;
  splashText[0] = '\0';
  if (xTaskCreatePinnedToCore(splashTask, "splash", 4096, NULL, 2, NULL, 0) == pdPASS) {
    splashRun = true;
  } else {
    splashRun = false;
    splashDone = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(30, 28);
    display.print(F("STARTING..."));
    display.display();
  }
}

void splashStageCreep(const char *status, int pct, int creep) {
  if (!splashRun) return;
  int h = splashQHead;
  int next = (h + 1) % SPLASH_Q;
  if (next == splashQTail) return;
  strncpy(splashQ[h].text, status, sizeof(splashQ[h].text) - 1);
  splashQ[h].text[sizeof(splashQ[h].text) - 1] = '\0';
  splashQ[h].pct = pct;
  splashQ[h].creep = creep;
  splashQHead = next;
}

void splashStage(const char *status, int pct) {
  splashStageCreep(status, pct, 8);
}

void splashFinish() {
  if (!splashRun) return;
  splashStageCreep("ALL SYSTEMS NOMINAL", 100, 0);
  splashFinishReq = true;
  uint32_t t = millis();
  while (!splashDone && millis() - t < 20000) delay(10);
  splashRun = false;
}

#define SONGANIM_MS 1500
#define SONGANIM_KINDS 10   // 0-4 Portal scenes, 5-9 music scenes (equalizer, record, sonar, warp, heartbeat)
static int songAnimKind = -1;
static unsigned long songAnimStart = 0;
static bool rippleOn = false;
static int rippleX = 0, rippleY = 0;
static unsigned long rippleStart = 0;

bool fxActive() {
  if (songAnimKind >= 0 && millis() - songAnimStart < SONGANIM_MS) return true;
  if (rippleOn && millis() - rippleStart < 450) return true;
  return false;
}

void startRipple(int x, int y) {
  rippleX = x;
  rippleY = y;
  rippleStart = millis();
  rippleOn = true;
}

void rippleDraw() {
  if (!rippleOn) return;
  uint32_t age = millis() - rippleStart;
  if (age > 450) { rippleOn = false; return; }
  int r = 2 + (int)(age / 40);
  display.drawCircle(rippleX, rippleY, r, SSD1306_WHITE);
  if (age > 120 && r > 4) display.drawCircle(rippleX, rippleY, r - 4, SSD1306_WHITE);
}

void songChangeCheck(const char *title) {
  static char last[96] = "";
  if (title[0] == '\0') return;
  if (strcmp(title, last) == 0) return;
  bool first = (last[0] == '\0');
  strncpy(last, title, sizeof(last) - 1);
  last[sizeof(last) - 1] = '\0';
  if (!first && (esp_random() % 100) < 50) {
    songAnimKind = (int)(esp_random() % SONGANIM_KINDS);
    songAnimStart = millis();
  }
}

bool songAnimDraw() {
  if (songAnimKind < 0) return false;
  uint32_t age = millis() - songAnimStart;
  if (age >= SONGANIM_MS) { songAnimKind = -1; return false; }
  float t = (float)age / (float)SONGANIM_MS;

  switch (songAnimKind) {
    case 0: {
      float open = (t < 0.15f) ? t / 0.15f : ((t > 0.85f) ? (1.0f - t) / 0.15f : 1.0f);
      int r = (int)(9 * open);
      fxPortal(10, 25, 3, r, false);
      fxPortal(118, 25, 3, r, true);
      if (t > 0.15f && t < 0.85f) {
        float p = (t - 0.15f) / 0.7f;
        for (int k = 3; k >= 0; k--) {
          float pp = p - 0.04f * k;
          if (pp < 0) continue;
          int bx = 14 + (int)(100 * pp);
          int by = 25 - (int)(32.0f * pp * (1.0f - pp));
          if (k == 0) display.fillCircle(bx, by, 2, SSD1306_WHITE);
          else display.drawPixel(bx, by, SSD1306_WHITE);
        }
      }
      break;
    }
    case 1: {
      int cy;
      if (t < 0.30f)      { float q = t / 0.30f; cy = 21 + (int)(8 * q * q); }
      else if (t < 0.55f) { float q = (t - 0.30f) / 0.25f; cy = 29 - (int)(7.0f * sinf(q * 3.14159f)); }
      else                cy = 29;
      bool visible = !(t > 0.85f && ((age / 60) % 2));
      if (visible) fxCube(64, cy, 6);
      if (t > 0.28f && t < 0.45f) {
        display.drawFastHLine(46, 36, 6, SSD1306_WHITE);
        display.drawFastHLine(76, 36, 6, SSD1306_WHITE);
      }
      break;
    }
    case 2: {
      fxOvalRing(12, 25, 3, 6, SSD1306_WHITE);
      display.fillCircle(12, 24, 1, SSD1306_WHITE);
      display.drawLine(10, 31, 7, 35, SSD1306_WHITE);
      display.drawLine(14, 31, 17, 35, SSD1306_WHITE);
      float ext = (t < 0.35f) ? t / 0.35f : 1.0f;
      int xe = 17 + (int)(95 * ext);
      bool blink = (t > 0.7f) && ((age / 80) % 2);
      if (!blink) {
        for (int x = 17; x < xe; x += 3) display.drawPixel(x, 24, SSD1306_WHITE);
        if (ext >= 1.0f) {
          display.drawLine(xe - 2, 22, xe + 2, 26, SSD1306_WHITE);
          display.drawLine(xe - 2, 26, xe + 2, 22, SSD1306_WHITE);
        }
      }
      break;
    }
    case 3: {
      fxIris(64, 25, 11, 0.1f + 0.8f * sinf(t * 3.14159f), t * 6.0f);
      break;
    }
    case 4: {
      fxCube(64, 30, 5);
      for (int k = 0; k < 3; k++) {
        float fr = fmodf(t * 2.0f + k * 0.33f, 1.0f);
        if (fr < 0.9f) display.drawBitmap(64 + (k - 1) * 14 - 3, 24 - (int)(10 * fr), fx_heart, 7, 6, SSD1306_WHITE);
      }
      break;
    }
    case 5: {   // equalizer bars bouncing, fading in and out
      float env = sinf(t * 3.14159f);
      for (int i = 0; i < 16; i++) {
        float ph = i * 0.9f + age * 0.012f;
        int h = 2 + (int)((5.0f + 9.0f * fabsf(sinf(ph))) * env);
        display.fillRect(8 + i * 7, 38 - h, 5, h, SSD1306_WHITE);
      }
      break;
    }
    case 6: {   // spinning record with a tone arm that drops on it
      const int cx = 54, cy = 26;
      display.drawCircle(cx, cy, 11, SSD1306_WHITE);
      display.drawCircle(cx, cy, 7, SSD1306_WHITE);
      display.fillCircle(cx, cy, 2, SSD1306_WHITE);
      float a = t * 14.0f;
      display.drawLine(cx, cy, cx + (int)(9 * cosf(a)), cy + (int)(9 * sinf(a)), SSD1306_WHITE);
      float drop = (t < 0.2f) ? 1.0f - t / 0.2f : 0.0f;
      int hx = 72 + (int)(8 * drop);
      display.drawLine(96, 15, hx, 26 - (int)(5 * drop), SSD1306_WHITE);
      display.fillCircle(96, 15, 2, SSD1306_WHITE);
      break;
    }
    case 7: {   // sonar pulses
      for (int k = 0; k < 4; k++) {
        float fr = fmodf(t * 2.2f + k * 0.25f, 1.0f);
        if (fr < 0.92f) display.drawCircle(64, 26, 2 + (int)(10 * fr), SSD1306_WHITE);
      }
      display.fillCircle(64, 26, 1, SSD1306_WHITE);
      break;
    }
    case 8: {   // warp-speed stars
      for (int i = 0; i < 18; i++) {
        float a = i * 0.349f + 0.3f * (i & 1);
        float fr = fmodf(t * 1.6f + i * 0.057f, 1.0f);
        float r = 62.0f * fr * fr;
        float r0 = r * 0.82f;
        int x1 = 64 + (int)(r * 2.0f * cosf(a)),  y1 = 26 + (int)(r * 0.2f * sinf(a));
        int x0 = 64 + (int)(r0 * 2.0f * cosf(a)), y0 = 26 + (int)(r0 * 0.2f * sinf(a));
        if (x1 < 0 || x1 > 127 || y1 < 14 || y1 > 38) continue;
        display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
      }
      break;
    }
    default: {  // heartbeat line scrolling past
      static const int8_t ecg[24] = { 0, 0, 0, 0, 0, 0, -2, -3, -2, 0, 0, 0, 2, -12, 10, 2, 0, 0, 0, -2, -3, -2, 0, 0 };
      bool visible = !(t > 0.85f && ((age / 60) % 2));
      if (visible) {
        int prev = 26;
        for (int x = 0; x < 128; x++) {
          int y = 26 + ecg[(x + (int)(age / 8)) % 24];
          if (x > 0) display.drawLine(x - 1, prev, x, y, SSD1306_WHITE);
          prev = y;
        }
      }
      break;
    }
  }
  return true;
}

// Keep this at the END of the file: the macro defines a function, and if it sits near the top the Arduino IDE
// inserts its auto-generated prototypes before the enum types and the build fails.
#if defined(SET_LOOP_TASK_STACK_SIZE)
SET_LOOP_TASK_STACK_SIZE(6 * 1024);   // default is 8 KB; the saved RAM goes back to the Bluetooth stack
#endif