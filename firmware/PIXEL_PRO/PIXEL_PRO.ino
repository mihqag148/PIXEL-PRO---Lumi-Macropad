#include <Arduino.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include <Arduino_TFT.h>
#include <AnimatedGIF.h>
#include <LittleFS.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <JPEGDEC.h>
#include <Adafruit_NeoPixel.h>
#include <mbedtls/base64.h>
#include <esp_system.h>
#include "USB.h"
#include "USBHID.h"
#include "USBHIDKeyboard.h"
#include "USBHIDConsumerControl.h"

#ifndef PIXEL_DIAG_TOUCH_OFF
#define PIXEL_DIAG_TOUCH_OFF 0
#endif

#ifndef PIXEL_DIAG_SAFE_PAR8
#define PIXEL_DIAG_SAFE_PAR8 0
#endif

#ifndef PIXEL_DIAG_DISPLAY_ONLY_IDLE
#define PIXEL_DIAG_DISPLAY_ONLY_IDLE 0
#endif

#ifndef PIXEL_DIAG_POST_INIT_STAGE
#define PIXEL_DIAG_POST_INIT_STAGE 0
#endif

#include "PixelStablePAR8.h"

#if PIXEL_DIAG_SAFE_PAR8
#include "PixelSafePAR8.h"
#endif

#if ARDUINO_USB_CDC_ON_BOOT
#error PIXEL PRO composite firmware requires USB CDC On Boot disabled
#else
USBCDC USBSerial;
#endif

static constexpr char FW_VERSION[] = "1.10.28";
static constexpr uint16_t USB_VID_PIXEL = 0x303A;
static constexpr uint16_t USB_PID_PIXEL = 0x80C2;
static constexpr uint8_t KEY_COUNT = 8;
static constexpr uint8_t PROFILE_COUNT = 20;
static constexpr uint8_t LAYER_COUNT = 4;
static constexpr uint8_t MACRO_COUNT = 20;
static constexpr uint8_t ACTION_COUNT = 32;
static constexpr uint8_t MACRO_MAX_LEN = 80;
static constexpr uint32_t DEBOUNCE_MS = 8;

// PIXEL PRO display: MCUFRIEND-style 3.5" 480x320 shield, HX8357-B controller, i8080 8-bit.
// The physical shield follows the common UNO/Mega2560 8-bit shield signal
// layout; HARDWARE.md maps those shield pins to these ESP32-S2 pins.
static constexpr uint16_t TFT_WIDTH = 480;
static constexpr uint16_t TFT_HEIGHT = 320;
static constexpr uint16_t GIF_LANDSCAPE_WIDTH = 480;
static constexpr uint16_t GIF_LANDSCAPE_HEIGHT = 320;
static constexpr uint16_t GIF_NATIVE_WIDTH = 320;
static constexpr uint16_t GIF_NATIVE_HEIGHT = 480;
static constexpr uint8_t DISPLAY_REFRESH_CAP_HZ = 60;
static constexpr uint8_t GIF_MAX_FPS = 60;
static constexpr uint16_t GIF_MIN_FRAME_MS = 17;
static constexpr uint32_t GIF_UPLOAD_LIMIT_BYTES = 2000UL * 1024UL;
static constexpr uint32_t JPEG_UPLOAD_LIMIT_BYTES = 2UL * 1024UL * 1024UL;
static constexpr uint32_t PACKED_UPLOAD_LIMIT_BYTES = 2000UL * 1024UL;

// PIXEL PRO main-menu artwork. Each keymap profile owns one 2x4 menu,
// matching the eight physical keys. Backgrounds stay JPEG; icons use a compact
// PXI1 RGB565 payload with a transparent color key so aspect-ratio padding
// reveals the wallpaper instead of a black rectangle.
static constexpr uint8_t MENU_SLOT_COUNT = 8;
static constexpr uint8_t MENU_ICON_WIDTH = 96;
static constexpr uint8_t MENU_ICON_HEIGHT = 96;
static constexpr uint32_t MENU_ICON_MAX_BYTES = 24UL * 1024UL;
static constexpr uint16_t MENU_ICON_TRANSPARENT = 0xF81F;
static constexpr uint32_t MENU_ICON_ASSET_BYTES =
    8UL + MENU_ICON_WIDTH * MENU_ICON_HEIGHT * 2UL;
static constexpr uint32_t MENU_BACKGROUND_LIMIT_BYTES = 96UL * 1024UL;
static constexpr uint8_t MENU_LABEL_MAX_LEN = 16;
static constexpr uint8_t MENU_STORAGE_VERSION = 4;
static constexpr uint8_t MENU_STATUS_HEIGHT = 0;
static constexpr char MENU_CONFIG_FILE_PATH[] = "/menu_cfg.bin";
static constexpr uint32_t MENU_BATCH_TIMEOUT_MS = 20000UL;

// PIXEL PRO has no firmware-resident Main Menu background or screensaver.
// User-uploaded assets are the only persistent visual media.

// Legacy raw-frame constants are kept only so older app builds can still
// upload their previous 240x160 RGB332 format. New app builds upload the
// original full-resolution GIF file instead.
static constexpr uint16_t GIF_WIDTH = 240;
static constexpr uint16_t GIF_HEIGHT = 160;
static constexpr uint8_t GIF_MAX_FRAMES = 32;

static constexpr char GIF_PATH[] = "/screensaver.gif";
static constexpr char GIF_TMP_PATH[] = "/screensaver.tmp";
static constexpr char JPEG_PATH[] = "/screensaver.jpg";
static constexpr char JPEG_TMP_PATH[] = "/screensaver_jpg.tmp";
static constexpr char PACKED_PATH[] = "/screensaver.pxq";
static constexpr char PACKED_TMP_PATH[] = "/screensaver_pxq.tmp";
static constexpr char SAVER_THUMB_PATH[] = "/screensaver_thumb.jpg";
static constexpr char SAVER_THUMB_TMP_PATH[] = "/screensaver_thumb.tmp";
static constexpr uint32_t SAVER_THUMB_LIMIT_BYTES = 96UL * 1024UL;

// LCD_RD is not driven by the MCU. Arduino_GFX does not require RD for
// write-only parallel displays; wire the shield's LCD_RD directly to 3V3.
// IMPORTANT: this shield also requires its 3V3 POWER pin tied to the S2 Mini
// 3V3 rail. LCD_RD may share that same 3V3 rail. The shield 5V pin remains
// connected to 5V/VBUS. Leaving shield 3V3 floating was measured at ~2.55 V
// and produced a very dim panel even though GRAM writes still worked.
// This frees D12 for the microSD bus.
static constexpr int8_t TFT_RD = GFX_NOT_DEFINED;
static constexpr int8_t TFT_WR = 13;
static constexpr int8_t TFT_DC = 14;
static constexpr int8_t TFT_CS = 16;
// LCD_RST is tied to the WEMOS S2 Mini EN pin so the panel resets whenever
// the ESP32-S2 resets. No dedicated GPIO is consumed by LCD reset.
static constexpr int8_t TFT_RST = GFX_NOT_DEFINED;
static constexpr int8_t TFT_D0 = 33;
static constexpr int8_t TFT_D1 = 34;
static constexpr int8_t TFT_D2 = 35;
static constexpr int8_t TFT_D3 = 36;
static constexpr int8_t TFT_D4 = 37;
static constexpr int8_t TFT_D5 = 38;
static constexpr int8_t TFT_D6 = 39;
static constexpr int8_t TFT_D7 = 40;

// D15 is also connected to the WEMOS S2 Mini onboard LED through a 2 kOhm
// resistor. It is intentionally used for the push-pull WS2812/SK6812 data
// stream, which frees clean D17/D18 pins for the module I2C bus.
static constexpr uint8_t RGB_PIN = 15;
static constexpr uint8_t RGB_LED_COUNT = 8;
static constexpr uint8_t RGB_STORAGE_VERSION = 1;

// Magnetic expansion-module bus.
// D18/D17 are a dedicated I2C pair feeding a PCA9546A 4-channel switch.
// Channels 0..2 are physical module ports 1..3; channel 3 is spare.
// Every module can therefore use the same I2C address while PIXEL PRO knows
// its physical connector from the selected PCA9546A channel.
static constexpr uint8_t MODULE_SDA_PIN = 18;
static constexpr uint8_t MODULE_SCL_PIN = 17;
static constexpr uint8_t MODULE_MUX_ADDRESS = 0x70;
static constexpr uint8_t MODULE_DEVICE_ADDRESS = 0x42;
static constexpr uint8_t MODULE_PORT_COUNT = 3;
static constexpr uint8_t MODULE_FRAME_SIZE = 16;
static constexpr uint8_t MODULE_PROTOCOL_VERSION = 1;
static constexpr uint8_t MODULE_FRAME_MAGIC = 0xA5;
static constexpr uint8_t MODULE_MISS_LIMIT = 3;
static constexpr uint8_t MODULE_MAX_TX_BYTES = 24;
static constexpr uint32_t MODULE_I2C_HZ = 400000UL;
static constexpr uint32_t MODULE_POLL_INTERVAL_MS = 2;
// Physical LED order requested by PIXEL PRO layout:
// LED1=K1, LED2=K2, LED3=K3, LED4=K4,
// LED5=K8, LED6=K7, LED7=K6, LED8=K5.
static constexpr uint8_t KEY_TO_LED[KEY_COUNT] = {0, 1, 2, 3, 7, 6, 5, 4};

static constexpr uint8_t BIND_DISABLED = 0;
static constexpr uint8_t BIND_KEYBOARD = 1;
static constexpr uint8_t BIND_CONSUMER = 2;
static constexpr uint8_t BIND_LAYER = 3;
static constexpr uint8_t BIND_MACRO = 4;
static constexpr uint8_t BIND_TRANSPARENT = 5;
static constexpr uint8_t BIND_ACTION = 6;

static constexpr uint8_t LAYER_MO = 1;
static constexpr uint8_t LAYER_TG = 2;
static constexpr uint8_t LAYER_TO = 3;
static constexpr uint8_t KEYMAP_STORAGE_VERSION = 3;

// Final PIXEL PRO input wiring.
static constexpr uint8_t MATRIX_ROW_COUNT = 2;
static constexpr uint8_t MATRIX_COL_COUNT = 4;
static constexpr uint8_t MATRIX_ROW_PINS[MATRIX_ROW_COUNT] = {1, 2};
static constexpr uint8_t MATRIX_COL_PINS[MATRIX_COL_COUNT] = {3, 4, 5, 6};

// Low-profile horizontal roller encoder, EVQWGD001-style (the part used by
// PIXEL PRO). Electrically it is still a 2-bit quadrature rotary encoder with
// a momentary push switch, but its footprint/pinout differs from an EC11.
// A/B use internal pull-ups; encoder common and one switch contact go to GND.
// The fourth encoder-side pad on genuine EVQWGD001 parts is NC.
static constexpr uint8_t ROLLER_A_PIN = 7;
static constexpr uint8_t ROLLER_B_PIN = 8;
static constexpr uint8_t ROLLER_SW_PIN = 21;

// TFT-shield microSD. Keeping SD_SCK/DO/DI/SS on consecutive D9..D12 makes
// hand-wiring to the WEMOS S2 Mini simple and leaves the display/data groups
// physically tidy. ESP32-S2 routes hardware SPI through the GPIO matrix.
static constexpr uint8_t SD_SCK_PIN = 9;
static constexpr uint8_t SD_MISO_PIN = 10;  // shield SD_DO
static constexpr uint8_t SD_MOSI_PIN = 11;  // shield SD_DI
static constexpr uint8_t SD_CS_PIN = 12;    // shield SD_SS
static constexpr uint32_t SD_SPI_HZ = 20000000UL;

static constexpr int8_t ROLLER_TRANSITIONS_PER_DETENT = 4;
static constexpr uint16_t ROLLER_CW_CONSUMER = 0x00E9;    // Volume increment
static constexpr uint16_t ROLLER_CCW_CONSUMER = 0x00EA;   // Volume decrement
static constexpr uint16_t ROLLER_SW_CONSUMER = 0x00E2;    // Mute

// 4-wire resistive touch shares four MCUFRIEND shield signals.
// YP=A1/LCD_WR, XM=A2/LCD_RS, XP=D6/LCD_D6, YM=D7/LCD_D7.
// D13/D14 are ADC-capable on ESP32-S2. LCD_CS is held high while the
// shared pins are temporarily reconfigured for a touch sample.
static constexpr int8_t TOUCH_YP_PIN = TFT_WR;  // D13
static constexpr int8_t TOUCH_XM_PIN = TFT_DC;  // D14
static constexpr int8_t TOUCH_XP_PIN = TFT_D6;  // D39
static constexpr int8_t TOUCH_YM_PIN = TFT_D7;  // D40
// Touch calibration copied from the shop's known-working MCUFRIEND sketch.
// Run the ESP32-S2 ADC itself at 10-bit so readings are in the exact 0..1023
// domain used by TouchScreen.h and the supplied calibration values.
static constexpr uint16_t TOUCH_ADC_MAX = 1023;
static constexpr uint16_t TOUCH_PRESSURE_MIN = 20;
static constexpr uint16_t TOUCH_PRESSURE_MAX = 5000;
static constexpr uint16_t TOUCH_RXPLATE_OHMS = 300;
// rawX is TouchScreen.h tp.x (sampled on YP); rawY is tp.y (sampled on XM).
// Keep the calibrated edge pairs with their real raw axes.
static constexpr uint16_t TOUCH_X_MIN_DEFAULT = 136;  // tp.x TS_RT
static constexpr uint16_t TOUCH_X_MAX_DEFAULT = 907;  // tp.x TS_LEFT
static constexpr uint16_t TOUCH_Y_MIN_DEFAULT = 139;  // tp.y TS_BOT
static constexpr uint16_t TOUCH_Y_MAX_DEFAULT = 942;  // tp.y TS_TOP
static constexpr uint16_t TOUCH_CAL_MIN_SPAN = 400;
static constexpr uint32_t TOUCH_POLL_MS = 20;
static constexpr uint8_t TOUCH_PRESS_CONFIRM_COUNT = 3;
static constexpr uint8_t TOUCH_RELEASE_MISS_COUNT = 4;
static constexpr uint16_t TOUCH_SAMPLE_STABILITY_MAX = 100;
static constexpr uint16_t TOUCH_CONFIRM_MOVE_MAX = 70;
static constexpr uint16_t TOUCH_CONTACT_RAIL_MARGIN = 24;
static constexpr uint16_t TOUCH_CONTACT_DELTA_MIN = 10;
static constexpr uint16_t TOUCH_CONTACT_STABILITY_MAX = 90;
// Touch acquisition follows MCUFRIEND/TouchScreen semantics in 1.10.25.
static constexpr uint16_t TOUCH_CAL_SAMPLE_STABILITY_MAX = 180;
static constexpr uint16_t TOUCH_CAL_RAW_MARGIN = 36;
static constexpr uint32_t TOUCH_TAP_MIN_MS = 45;
static constexpr uint32_t TOUCH_TAP_MAX_MS = 1600;
static constexpr uint8_t TOUCH_CAL_VERSION = 8;
static constexpr uint8_t TOUCH_FLAG_SWAP_XY = 0x01;
static constexpr uint8_t TOUCH_FLAG_INVERT_X = 0x02;
static constexpr uint8_t TOUCH_FLAG_INVERT_Y = 0x04;
// HX8357-B native memory is 320x480 and is rotated to 480x320 landscape.
// MCUFRIEND Touch_shield_new Orientation=1 maps:
//   screen X = map(tp.y, TS_TOP, TS_BOT, 0, 480)
//   screen Y = map(tp.x, TS_RT, TS_LEFT, 0, 320)
// With the calibrated ranges above that is swap X/Y + invert screen X only.
static constexpr uint8_t TOUCH_DEFAULT_FLAGS =
    TOUCH_FLAG_SWAP_XY |
    TOUCH_FLAG_INVERT_X;

struct __attribute__((packed)) ModuleFrame {
  uint8_t magic;
  uint8_t version;
  uint8_t type;
  uint8_t sequence;
  uint16_t buttons;
  int8_t encoder1;
  int8_t encoder2;
  uint16_t slider1;
  uint16_t slider2;
  uint16_t moduleId;
  uint8_t flags;
  uint8_t crc;
};

static_assert(
    sizeof(ModuleFrame) == MODULE_FRAME_SIZE,
    "ModuleFrame must stay 16 bytes");

struct ModulePortState {
  bool connected;
  bool hasFrame;
  uint8_t missCount;
  uint8_t lastSequence;
  ModuleFrame frame;
};

struct __attribute__((packed)) KeyBinding {
  uint8_t type;
  uint8_t keyCode;
  uint8_t modifiers;
  uint16_t consumerCode;
};

struct KeyState {
  bool rawPressed;
  bool stablePressed;
  uint32_t changedAt;
};

struct __attribute__((packed)) TouchCalibration {
  uint16_t xMin;
  uint16_t xMax;
  uint16_t yMin;
  uint16_t yMax;
  uint8_t flags;
};

struct TouchAffineCalibration {
  float ax;
  float bx;
  float cx;
  float ay;
  float by;
  float cy;
};

struct __attribute__((packed)) MainMenuConfig {
  uint8_t version;
  uint8_t actions[PROFILE_COUNT][MENU_SLOT_COUNT];
  char labels[PROFILE_COUNT][MENU_SLOT_COUNT][MENU_LABEL_MAX_LEN + 1];
};

struct __attribute__((packed)) LegacyMainMenuConfigV3 {
  uint8_t version;
  uint8_t actions[PROFILE_COUNT][12];
  char labels[PROFILE_COUNT][12][MENU_LABEL_MAX_LEN + 1];
};


// Controller ID was measured on the user's actual shield over LCD_RD:
 // REG 0xBF returned 00 01 62 83 57 FF, i.e. MCUFRIEND ID 0x8357.
 // Use Arduino_GFX's native HX8357-B driver instead of a guessed custom init.


class PixelHX8357BMcufriend : public Arduino_TFT {
 public:
  PixelHX8357BMcufriend(
      Arduino_DataBus *bus,
      int8_t rst = GFX_NOT_DEFINED,
      uint8_t rotation = 0)
      : Arduino_TFT(
            bus,
            rst,
            rotation,
            false,
            320,
            480,
            0,
            0,
            0,
            0) {}

  bool begin(
      int32_t speed =
          GFX_NOT_DEFINED) override {
    // Keep Arduino_GFX bus/display setup, but leave final rotation and
    // REV_SCREEN polarity to initDisplay(). That preserves the exact
    // BASELINE_V3 order proven on the real shield:
    // DISPON -> MADCTL landscape -> INVON.
    return Arduino_TFT::begin(
        speed);
  }

  void writeAddrWindow(
      int16_t x,
      int16_t y,
      uint16_t w,
      uint16_t h) override {
    if ((x != _currentX) ||
        (w != _currentW)) {
      _currentX = x;
      _currentW = w;
      x += _xStart;

      _bus->writeC8D16D16(
          0x2A,
          static_cast<uint16_t>(x),
          static_cast<uint16_t>(
              x + w - 1));
    }

    if ((y != _currentY) ||
        (h != _currentH)) {
      _currentY = y;
      _currentH = h;
      y += _yStart;

      _bus->writeC8D16D16(
          0x2B,
          static_cast<uint16_t>(y),
          static_cast<uint16_t>(
              y + h - 1));
    }

    _bus->writeCommand(0x2C);
  }

  void setRotation(
      uint8_t rotation) override {
    Arduino_TFT::setRotation(
        rotation);

    uint8_t madctl = 0x48;

    switch (_rotation) {
      case 1:
        madctl = 0x28;
        break;

      case 2:
        madctl = 0x98;
        break;

      case 3:
        madctl = 0xF8;
        break;

      default:
        madctl = 0x48;
        break;
    }

    _bus->beginWrite();
    _bus->writeCommand(0x36);
    _bus->write(madctl);
    _bus->endWrite();
  }

  void invertDisplay(
      bool invert) override {
    // MCUFRIEND: _lcd_rev = REV_SCREEN ^ invert.
    // ID 0x8357 has REV_SCREEN set, so invert=false => command 0x21.
    _bus->sendCommand(
        invert
            ? 0x20
            : 0x21);
  }

  void displayOn() override {
    _bus->sendCommand(0x29);
  }

  void displayOff() override {
    _bus->sendCommand(0x28);
  }

 protected:
  void tftInit() override {
    // Mirror MCUFRIEND_kbv's actual ID 0x8357 path instead of Arduino_GFX's
    // HX8357B power/timing table. MCUFRIEND intentionally uses only the
    // generic reset_off/wake_on sequence for 0x8357; forcing the native
    // Arduino_HX8357B voltage/timing registers caused the user's panel to
    // remain dark and flicker.
    //
    // The shield's LCD_RST is tied to S2 Mini EN, so a hardware reset already
    // occurred when the ESP32-S2 booted. MCUFRIEND also writes B0=0 during
    // reset/readID, so reproduce that harmless unlock before reset_off.
    _bus->beginWrite();
    _bus->writeCommand(0xB0);
    _bus->write(0x00);
    _bus->write(0x00);
    _bus->endWrite();

    _bus->sendCommand(0x01);
    delay(150);

    _bus->sendCommand(0x28);

    _bus->beginWrite();
    _bus->writeCommand(0x3A);
    _bus->write(0x55);
    _bus->endWrite();

    delay(1);

    _bus->sendCommand(0x11);
    delay(150);


    _bus->sendCommand(0x29);
    delay(50);

    // Stop here intentionally. The measured ID 0x8357 MCUFRIEND path does
    // not program the Adafruit HX8357-B POWER/VCOM/PANEL/GAMMA table.
    // PIXEL PRO diagnostics showed no visual benefit from forcing that table,
    // and production has since shown washed-out / low-contrast colour.
    // initDisplay() now applies only landscape MADCTL + REV_SCREEN INVON after
    // this proven wake sequence.
  }
};

USBHID HID;
USBHIDKeyboard Keyboard;
USBHIDConsumerControl ConsumerControl;
Preferences preferences;

#if PIXEL_DIAG_SAFE_PAR8
Arduino_DataBus *tftBus =
    new PixelSafePAR8(
        TFT_DC,
        TFT_CS,
        TFT_WR,
        TFT_RD,
        TFT_D0,
        TFT_D1,
        TFT_D2,
        TFT_D3,
        TFT_D4,
        TFT_D5,
        TFT_D6,
        TFT_D7);
#else
Arduino_DataBus *tftBus =
    new PixelStablePAR8(
        TFT_DC,
        TFT_CS,
        TFT_WR,
        TFT_RD,
        TFT_D0,
        TFT_D1,
        TFT_D2,
        TFT_D3,
        TFT_D4,
        TFT_D5,
        TFT_D6,
        TFT_D7);
#endif

Arduino_GFX *tft =
    new PixelHX8357BMcufriend(
        tftBus,
        TFT_RST,
        1);

static KeyState keyState[KEY_COUNT] = {};
static KeyState rollerSwitchState = {};
static uint8_t rollerLastAB = 0;
static int8_t rollerTransitionAccumulator = 0;

static TouchCalibration touchCalibration = {};
static TouchAffineCalibration touchAffine = {};
static bool touchAffineValid = false;
static bool touchCalibrationRequired = false;
static bool touchRawPressed = false;
static bool touchStablePressed = false;
static uint8_t touchPressConfirmations = 0;
static uint8_t touchReleaseMisses = 0;
static uint16_t touchCandidateRawX = 0;
static uint16_t touchCandidateRawY = 0;
static uint32_t touchPressStartedAt = 0;
static int8_t touchPendingSlot = -1;
static bool touchWakeOnly = false;
static int8_t touchHeldFallbackSlot = -1;
static int8_t touchFeedbackSlot = -1;
static bool touchCalibrationMode = false;
static bool touchPixelTestMode = false;
static uint8_t touchCalibrationPoint = 0;
static bool touchCalibrationPointCaptured = false;
static uint16_t touchCalibrationRawX[4] = {};
static uint16_t touchCalibrationRawY[4] = {};
static uint32_t touchChangedAt = 0;
static uint32_t touchLastPollAt = 0;
static uint16_t touchRawX = 0;
static uint16_t touchRawY = 0;
static uint16_t touchPressure = 0;
static int16_t touchX = -1;
static int16_t touchY = -1;

static KeyBinding keymap[PROFILE_COUNT][LAYER_COUNT][KEY_COUNT] = {};
static KeyBinding activeBindings[KEY_COUNT] = {};
static String macros[MACRO_COUNT];
static MainMenuConfig mainMenuConfig = {};
static File menuUploadFile;
static uint8_t menuUploadKind = 0;  // 1=background, 2=icon
static uint8_t menuUploadProfile = 0;
static uint8_t menuUploadSlot = 0;
static uint32_t menuUploadExpectedBytes = 0;
static uint32_t menuUploadReceivedBytes = 0;
static bool menuBatchActive = false;
static bool menuBatchDirty = false;
static uint8_t menuBatchProfile = 0;
static uint32_t menuBatchLastActivityAt = 0;
static bool menuRenderPending = false;
static uint8_t menuRenderPendingProfile = 0;
static uint32_t menuRenderNotBeforeAt = 0;
static constexpr uint32_t MENU_RENDER_DEFER_MS = 120UL;
static uint8_t menuLastContentProfile = 0;
static uint8_t menuRenderedProfile = 0;
static uint32_t menuCompositeMask = 0;

RTC_DATA_ATTR static uint32_t bootSequence = 0;
static esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;

static int16_t menuCpuLoad = -1;
static int16_t menuCpuTemp = -1;
static int16_t menuGpuLoad = -1;
static int16_t menuGpuTemp = -1;
static uint8_t menuMonth = 0;
static uint8_t menuDay = 0;
static uint8_t menuHour = 0;
static uint8_t menuMinute = 0;
static bool menuPcStatusValid = false;
static uint8_t menuHostOs = 0;  // 0=unknown, 1=Windows, 2=macOS, 3=Linux

static uint8_t rgbProfiles[PROFILE_COUNT][KEY_COUNT][3] = {};
// PIXEL effects: 0 rainbow, 1 purple ping-pong, 2 orange blink,
 // 3 static, 4 fade, 5 chase, 6 breathe, 7 color shift, 8 rain, 9 wave.
static uint8_t rgbEffects[PROFILE_COUNT] = {};
static bool rgbEnabled = true;
static uint8_t rgbBrightnessPercent = 25;
static uint8_t rgbSpeedPercent = 50;
static uint32_t rgbLastFrameAt = 0;
static uint16_t rgbAnimationStep = 0;
static Adafruit_NeoPixel rgbStrip(
    RGB_LED_COUNT,
    RGB_PIN,
    NEO_GRB + NEO_KHZ800);

static uint8_t pressedMask = 0;
static uint16_t activeConsumerCode = 0;
static uint8_t activeProfile = 0;
static uint8_t baseLayer = 0;
static int8_t momentaryLayer = -1;
static uint8_t toggledLayerMask = 0;
static String cdcLine;

static ModulePortState modulePorts[MODULE_PORT_COUNT] = {};
static bool moduleMuxReady = false;
static uint8_t moduleNextPort = 0;
static uint32_t moduleLastPollAt = 0;

enum SaverPixelFormat : uint8_t {
  SAVER_NONE = 0,
  SAVER_RGB332 = 1,
  SAVER_RGB565 = 2,
  SAVER_GIF = 3,
  SAVER_JPEG = 4,
  SAVER_PACKED = 5,
};

enum GifScaleMode : uint8_t {
  GIF_SCALE_FILL = 0,
  GIF_SCALE_FIT = 1,
  GIF_SCALE_STRETCH = 2,
  GIF_SCALE_TILE = 3,
  GIF_SCALE_CENTER = 4,
  GIF_SCALE_SPAN = 5,
};


enum RawMediaKind : uint8_t {
  RAW_MEDIA_NONE = 0,
  RAW_MEDIA_MENU_BG = 1,
  RAW_MEDIA_MENU_ICON = 2,
  RAW_MEDIA_GIF = 3,
  RAW_MEDIA_JPEG = 4,
  RAW_MEDIA_PACKED = 5,
  RAW_MEDIA_THUMB = 6,
};

static constexpr uint32_t RAW_MEDIA_ACK_BYTES = 512UL;
static constexpr uint32_t RAW_MEDIA_TIMEOUT_MS = 15000UL;
static constexpr size_t RAW_MEDIA_RX_BUFFER_BYTES = 8192U;

static bool displayReady = false;
static uint16_t *renderBuffer = nullptr;
static uint16_t displayVividLine[TFT_WIDTH] = {};


static uint8_t *saverData = nullptr;
static size_t saverDataBytes = 0;
static size_t saverFrameBytes = 0;
static size_t saverBytesReceived = 0;
static uint8_t saverFrameCount = 0;
static uint16_t saverWidth = 0;
static uint16_t saverHeight = 0;
static SaverPixelFormat saverFormat = SAVER_NONE;
static uint16_t saverDurations[GIF_MAX_FRAMES] = {};
static bool saverUploading = false;
static bool saverReady = false;
static bool saverActive = false;
static uint8_t saverFrameIndex = 0;
static uint32_t saverFrameStartedAt = 0;
static uint32_t lastUserActivityAt = 0;
static uint32_t saverDelayMs = 60000;

static bool littleFsReady = false;
static bool sdReady = false;
static uint8_t sdCardType = CARD_NONE;
static File gifUploadFile;
static uint32_t gifUploadExpectedBytes = 0;
static uint16_t gifUploadWidth = 0;
static uint16_t gifUploadHeight = 0;

static File jpegUploadFile;
static uint32_t jpegUploadExpectedBytes = 0;
static uint16_t jpegUploadWidth = 0;
static uint16_t jpegUploadHeight = 0;
static JPEGDEC jpegDecoder;
static File jpegPlaybackFile;

static File packedUploadFile;
static File packedPlaybackFile;
static uint32_t packedUploadExpectedBytes = 0;

static File saverThumbUploadFile;
static uint32_t saverThumbExpectedBytes = 0;
static uint32_t saverThumbReceivedBytes = 0;
static RawMediaKind rawMediaKind = RAW_MEDIA_NONE;
static uint32_t rawMediaExpectedBytes = 0;
static uint32_t rawMediaReceivedBytes = 0;
static uint32_t rawMediaExpectedCrc = 0;
static uint32_t rawMediaRunningCrc = 0xFFFFFFFFUL;
static uint32_t rawMediaNextAckAt = 0;
static uint32_t rawMediaLastActivityAt = 0;
static uint8_t rawMediaProfile = 0;
static uint8_t rawMediaSlot = 0;

static uint16_t packedStorageWidth = 0;
static uint16_t packedStorageHeight = 0;
static uint16_t packedFrameCount = 0;
static uint16_t packedFps = 0;
static uint16_t packedFrameIndex = 0;
static uint16_t packedPaletteCount = 0;
static uint8_t packedColorMode = 0;
static uint32_t packedDurationMs = 0;
static uint32_t packedFramesOffset = 0;
static uint32_t packedNextFrameAt = 0;
static uint16_t packedPalette565[256] = {};
static uint16_t *packedLineBuffer = nullptr;
static uint16_t packedLineFallback[TFT_WIDTH] = {};

static AnimatedGIF gifDecoder;
static File gifPlaybackFile;
static bool gifDecoderOpen = false;
static bool gifAtEnd = false;
static uint16_t gifCanvasWidth = 0;
static uint16_t gifCanvasHeight = 0;
static uint32_t gifNextFrameAt = 0;
static GifScaleMode gifScaleMode = GIF_SCALE_CENTER;
static GifScaleMode gifUploadScaleMode = GIF_SCALE_CENTER;
static bool bootloaderArmed = false;
static uint32_t bootloaderArmUntil = 0;
static float gifScaleX = 1.0f;
static float gifScaleY = 1.0f;
static float gifOffsetX = 0.0f;
static float gifOffsetY = 0.0f;

static void stopSaver();
static void clearSaverBuffer();
static void closeJpegUploadFile();
static void closePackedFiles();
static void startAutomaticTouchCalibration();
static void cancelAutomaticTouchCalibration();

static void cdcPrintln(const String &line) {
  USBSerial.println(line);
}

static bool mountPersistentStorage(
    bool allowFormat = true) {
  LittleFS.end();

  // The custom partition is named "spiffs" for Arduino compatibility,
  // while the filesystem stored inside it is LittleFS. Name the partition
  // explicitly so a core/library default can never select the wrong data
  // partition.
  if (LittleFS.begin(
          false,
          "/littlefs",
          10,
          "spiffs")) {
    return true;
  }

  if (!allowFormat) {
    return false;
  }

  return LittleFS.begin(
      true,
      "/littlefs",
      10,
      "spiffs");
}

static const char *sdCardTypeName(
    uint8_t type) {
  switch (type) {
    case CARD_MMC:
      return "MMC";
    case CARD_SD:
      return "SDSC";
    case CARD_SDHC:
      return "SDHC";
    case CARD_NONE:
      return "NONE";
    default:
      return "UNKNOWN";
  }
}

static void unmountSdCard() {
  if (sdReady) {
    SD.end();
  }

  SPI.end();

  sdReady = false;
  sdCardType = CARD_NONE;

  // Leave CS high between mount attempts so the card stays deselected.
  pinMode(
      SD_CS_PIN,
      OUTPUT);
  digitalWrite(
      SD_CS_PIN,
      HIGH);
}

static bool mountSdCard() {
  unmountSdCard();

  pinMode(
      SD_CS_PIN,
      OUTPUT);
  digitalWrite(
      SD_CS_PIN,
      HIGH);

  SPI.begin(
      SD_SCK_PIN,
      SD_MISO_PIN,
      SD_MOSI_PIN,
      SD_CS_PIN);

  if (!SD.begin(
          SD_CS_PIN,
          SPI,
          SD_SPI_HZ)) {
    SPI.end();
    return false;
  }

  sdCardType =
      SD.cardType();

  if (sdCardType ==
      CARD_NONE) {
    SD.end();
    SPI.end();
    sdCardType =
        CARD_NONE;
    return false;
  }

  sdReady = true;
  return true;
}

static void sendSdInfo() {
  if (!sdReady) {
    cdcPrintln(
        "SDINFO|ABSENT");
    return;
  }

  char out[160];

  snprintf(
      out,
      sizeof(out),
      "SDINFO|READY|TYPE=%s|CARD=%llu|TOTAL=%llu|USED=%llu|HZ=%lu",
      sdCardTypeName(sdCardType),
      static_cast<unsigned long long>(
          SD.cardSize()),
      static_cast<unsigned long long>(
          SD.totalBytes()),
      static_cast<unsigned long long>(
          SD.usedBytes()),
      static_cast<unsigned long>(
          SD_SPI_HZ));

  cdcPrintln(
      out);
}

static bool runSdSelfTest() {
  if (!sdReady &&
      !mountSdCard()) {
    return false;
  }

  static constexpr char TEST_PATH[] =
      "/pixelpro_sd_test.tmp";

  static constexpr char TEST_PAYLOAD[] =
      "PIXELPRO-SD-1.9.3";

  SD.remove(
      TEST_PATH);

  File file =
      SD.open(
          TEST_PATH,
          FILE_WRITE);

  if (!file) {
    return false;
  }

  const size_t expected =
      strlen(
          TEST_PAYLOAD);

  const size_t written =
      file.write(
          reinterpret_cast<const uint8_t *>(
              TEST_PAYLOAD),
          expected);

  file.flush();
  file.close();

  if (written !=
      expected) {
    SD.remove(
        TEST_PATH);
    return false;
  }

  file =
      SD.open(
          TEST_PATH,
          FILE_READ);

  if (!file) {
    SD.remove(
        TEST_PATH);
    return false;
  }

  char buffer[
      sizeof(TEST_PAYLOAD)] = {};

  const size_t read =
      file.readBytes(
          buffer,
          expected);

  file.close();
  SD.remove(
      TEST_PATH);

  return read == expected &&
         memcmp(
             buffer,
             TEST_PAYLOAD,
             expected) == 0;
}

static KeyBinding disabledBinding() {
  KeyBinding binding = {};
  binding.type = BIND_DISABLED;
  return binding;
}

static KeyBinding transparentBinding() {
  KeyBinding binding = {};
  binding.type = BIND_TRANSPARENT;
  return binding;
}

static void setDefaultKeymap() {
  for (uint8_t profile = 0; profile < PROFILE_COUNT; ++profile) {
    for (uint8_t layer = 0; layer < LAYER_COUNT; ++layer) {
      for (uint8_t i = 0; i < KEY_COUNT; ++i) {
        keymap[profile][layer][i] =
            layer == 0 ? disabledBinding() : transparentBinding();
      }
    }

    for (uint8_t i = 0; i < KEY_COUNT; ++i) {
      keymap[profile][0][i].type = BIND_KEYBOARD;
      keymap[profile][0][i].keyCode = static_cast<uint8_t>(HID_KEY_A + i);
      keymap[profile][0][i].modifiers = 0;
      keymap[profile][0][i].consumerCode = 0;
    }
  }
}

static bool bindingIsValid(const KeyBinding &binding) {
  switch (binding.type) {
    case BIND_DISABLED:
    case BIND_TRANSPARENT:
      return binding.keyCode == 0 &&
             binding.modifiers == 0 &&
             binding.consumerCode == 0;

    case BIND_KEYBOARD:
      return (binding.keyCode != 0 || binding.modifiers != 0) &&
             (binding.modifiers & 0xF0) == 0 &&
             binding.consumerCode == 0;

    case BIND_CONSUMER:
      return binding.keyCode == 0 &&
             binding.modifiers == 0 &&
             binding.consumerCode != 0;

    case BIND_LAYER:
      return binding.keyCode < LAYER_COUNT &&
             (binding.modifiers == LAYER_MO ||
              binding.modifiers == LAYER_TG ||
              binding.modifiers == LAYER_TO) &&
             binding.consumerCode == 0;

    case BIND_MACRO:
      return binding.keyCode < MACRO_COUNT &&
             binding.modifiers == 0 &&
             binding.consumerCode == 0;

    case BIND_ACTION:
      return binding.keyCode >= 1 &&
             binding.keyCode <= ACTION_COUNT &&
             binding.modifiers == 0 &&
             binding.consumerCode == 0;

    default:
      return false;
  }
}

static void saveKeymap() {
  preferences.putUChar("mapver", KEYMAP_STORAGE_VERSION);
  preferences.putBytes("keymap", keymap, sizeof(keymap));
}

static void loadKeymap() {
  setDefaultKeymap();

  if (preferences.getUChar("mapver", 0) != KEYMAP_STORAGE_VERSION ||
      preferences.getBytesLength("keymap") != sizeof(keymap)) {
    saveKeymap();
    return;
  }

  static KeyBinding stored[PROFILE_COUNT][LAYER_COUNT][KEY_COUNT] = {};
  size_t read = preferences.getBytes("keymap", stored, sizeof(stored));

  if (read != sizeof(stored)) {
    saveKeymap();
    return;
  }

  for (uint8_t profile = 0; profile < PROFILE_COUNT; ++profile) {
    for (uint8_t layer = 0; layer < LAYER_COUNT; ++layer) {
      for (uint8_t i = 0; i < KEY_COUNT; ++i) {
        if (!bindingIsValid(stored[profile][layer][i])) {
          saveKeymap();
          return;
        }
      }
    }
  }

  memcpy(keymap, stored, sizeof(keymap));
}

static void loadMacros() {
  for (uint8_t i = 0; i < MACRO_COUNT; ++i) {
    char key[5];
    snprintf(key, sizeof(key), "m%u", i);
    macros[i] = preferences.getString(key, "");
    if (macros[i].length() > MACRO_MAX_LEN) {
      macros[i].remove(MACRO_MAX_LEN);
    }
  }
}

static void saveMacro(uint8_t index) {
  if (index >= MACRO_COUNT) {
    return;
  }

  char key[5];
  snprintf(key, sizeof(key), "m%u", index);
  preferences.putString(key, macros[index]);
}

static void setDefaultTouchCalibration() {
  touchCalibration.xMin = TOUCH_X_MIN_DEFAULT;
  touchCalibration.xMax = TOUCH_X_MAX_DEFAULT;
  touchCalibration.yMin = TOUCH_Y_MIN_DEFAULT;
  touchCalibration.yMax = TOUCH_Y_MAX_DEFAULT;
  touchCalibration.flags = TOUCH_DEFAULT_FLAGS;

  touchAffine = {};
  touchAffineValid = false;
}

static bool touchCalibrationIsValid(
    const TouchCalibration &calibration) {
  if (calibration.xMax <= calibration.xMin ||
      calibration.yMax <= calibration.yMin) {
    return false;
  }

  if (calibration.xMax - calibration.xMin < TOUCH_CAL_MIN_SPAN ||
      calibration.yMax - calibration.yMin < TOUCH_CAL_MIN_SPAN) {
    return false;
  }

  if (calibration.xMax > TOUCH_ADC_MAX ||
      calibration.yMax > TOUCH_ADC_MAX ||
      calibration.flags >
          (TOUCH_FLAG_SWAP_XY |
           TOUCH_FLAG_INVERT_X |
           TOUCH_FLAG_INVERT_Y)) {
    return false;
  }

  return true;
}

static bool touchAffineIsValid(
    const TouchAffineCalibration &affine) {
  const float values[6] = {
      affine.ax,
      affine.bx,
      affine.cx,
      affine.ay,
      affine.by,
      affine.cy};

  for (float value : values) {
    if (!isfinite(value)) {
      return false;
    }
  }

  // Touch-panel scale is normally well below 1 px/raw-count. Keep generous
  // limits while rejecting corrupted NVS values.
  if (fabsf(affine.ax) > 10.0f ||
      fabsf(affine.bx) > 10.0f ||
      fabsf(affine.ay) > 10.0f ||
      fabsf(affine.by) > 10.0f ||
      fabsf(affine.cx) > 10000.0f ||
      fabsf(affine.cy) > 10000.0f) {
    return false;
  }

  return fabsf(affine.ax) +
             fabsf(affine.bx) >
         0.02f &&
         fabsf(affine.ay) +
             fabsf(affine.by) >
         0.02f;
}

static void saveTouchCalibration() {
  preferences.putUChar(
      "tcver",
      TOUCH_CAL_VERSION);

  preferences.putUShort(
      "tcxmin",
      touchCalibration.xMin);
  preferences.putUShort(
      "tcxmax",
      touchCalibration.xMax);
  preferences.putUShort(
      "tcymin",
      touchCalibration.yMin);
  preferences.putUShort(
      "tcymax",
      touchCalibration.yMax);
  preferences.putUChar(
      "tcflags",
      touchCalibration.flags);

  preferences.putBool(
      "tcaff",
      touchAffineValid);

  preferences.putFloat(
      "tcax",
      touchAffine.ax);
  preferences.putFloat(
      "tcbx",
      touchAffine.bx);
  preferences.putFloat(
      "tccx",
      touchAffine.cx);
  preferences.putFloat(
      "tcay",
      touchAffine.ay);
  preferences.putFloat(
      "tcby",
      touchAffine.by);
  preferences.putFloat(
      "tccy",
      touchAffine.cy);
}

static void loadTouchCalibration() {
  setDefaultTouchCalibration();

  const uint8_t storedVersion =
      preferences.getUChar(
          "tcver",
          0);

  if (storedVersion !=
      TOUCH_CAL_VERSION) {
    touchCalibrationRequired = true;
    saveTouchCalibration();
    return;
  }

  TouchCalibration stored = {};
  stored.xMin =
      preferences.getUShort(
          "tcxmin",
          TOUCH_X_MIN_DEFAULT);
  stored.xMax =
      preferences.getUShort(
          "tcxmax",
          TOUCH_X_MAX_DEFAULT);
  stored.yMin =
      preferences.getUShort(
          "tcymin",
          TOUCH_Y_MIN_DEFAULT);
  stored.yMax =
      preferences.getUShort(
          "tcymax",
          TOUCH_Y_MAX_DEFAULT);
  stored.flags =
      preferences.getUChar(
          "tcflags",
          TOUCH_DEFAULT_FLAGS);

  if (touchCalibrationIsValid(
          stored)) {
    touchCalibration =
        stored;
  }

  TouchAffineCalibration storedAffine = {};
  storedAffine.ax =
      preferences.getFloat(
          "tcax",
          0.0f);
  storedAffine.bx =
      preferences.getFloat(
          "tcbx",
          0.0f);
  storedAffine.cx =
      preferences.getFloat(
          "tccx",
          0.0f);
  storedAffine.ay =
      preferences.getFloat(
          "tcay",
          0.0f);
  storedAffine.by =
      preferences.getFloat(
          "tcby",
          0.0f);
  storedAffine.cy =
      preferences.getFloat(
          "tccy",
          0.0f);

  const bool storedAffineValid =
      preferences.getBool(
          "tcaff",
          false);

  if (storedAffineValid &&
      touchAffineIsValid(
          storedAffine)) {
    touchAffine =
        storedAffine;

    touchAffineValid =
        true;

    touchCalibrationRequired =
        false;
  } else {
    touchAffine = {};
    touchAffineValid = false;
    touchCalibrationRequired = true;
  }
}

static void setDefaultRgbProfiles() {
  for (uint8_t profile = 0; profile < PROFILE_COUNT; ++profile) {
    rgbEffects[profile] = 3;

    for (uint8_t key = 0; key < KEY_COUNT; ++key) {
      rgbProfiles[profile][key][0] =
          static_cast<uint8_t>(255 - key * 16);
      rgbProfiles[profile][key][1] =
          static_cast<uint8_t>(96 + key * 18);
      uint16_t blue =
          static_cast<uint16_t>(profile) * 5U;
      rgbProfiles[profile][key][2] =
          static_cast<uint8_t>(
              blue > 120U
                  ? 120U
                  : blue);
    }
  }
}

static void saveRgbProfiles() {
  preferences.putUChar("rgbver", RGB_STORAGE_VERSION);
  preferences.putBytes(
      "rgbkeys",
      rgbProfiles,
      sizeof(rgbProfiles));
  preferences.putBytes(
      "rgbfx",
      rgbEffects,
      sizeof(rgbEffects));
  preferences.putBool("rgben", rgbEnabled);
  preferences.putUChar(
      "rgbbr",
      rgbBrightnessPercent);
  preferences.putUChar(
      "rgbspd",
      rgbSpeedPercent);
}

static void loadRgbProfiles() {
  setDefaultRgbProfiles();

  if (preferences.getUChar("rgbver", 0) == RGB_STORAGE_VERSION &&
      preferences.getBytesLength("rgbkeys") == sizeof(rgbProfiles)) {
    size_t read =
        preferences.getBytes(
            "rgbkeys",
            rgbProfiles,
            sizeof(rgbProfiles));

    if (read != sizeof(rgbProfiles)) {
      setDefaultRgbProfiles();
    }
  }

  rgbEnabled =
      preferences.getBool(
          "rgben",
          true);

  if (preferences.getBytesLength("rgbfx") == sizeof(rgbEffects)) {
    preferences.getBytes(
        "rgbfx",
        rgbEffects,
        sizeof(rgbEffects));

    for (uint8_t profile = 0; profile < PROFILE_COUNT; ++profile) {
      if (rgbEffects[profile] > 9) {
        rgbEffects[profile] = 3;
      }
    }
  }

  rgbBrightnessPercent =
      static_cast<uint8_t>(
          constrain(
              preferences.getUChar(
                  "rgbbr",
                  25),
              0,
              100));

  rgbSpeedPercent =
      static_cast<uint8_t>(
          constrain(
              preferences.getUChar(
                  "rgbspd",
                  50),
              10,
              100));
}

static uint16_t rgbFrameIntervalMs() {
  return static_cast<uint16_t>(
      map(
          rgbSpeedPercent,
          10,
          100,
          180,
          24));
}

static void applyRgbBrightness() {
  uint8_t brightness =
      rgbEnabled
          ? static_cast<uint8_t>(
                map(
                    rgbBrightnessPercent,
                    0,
                    100,
                    0,
                    255))
          : 0;

  rgbStrip.setBrightness(brightness);
}

static void renderRgbStatic() {
  applyRgbBrightness();

  for (uint8_t key = 0; key < KEY_COUNT; ++key) {
    uint8_t led =
        KEY_TO_LED[key];

    rgbStrip.setPixelColor(
        led,
        rgbProfiles[activeProfile][key][0],
        rgbProfiles[activeProfile][key][1],
        rgbProfiles[activeProfile][key][2]);
  }

  rgbStrip.show();
}

static uint8_t triangle8(
    uint16_t value) {
  uint8_t phase =
      static_cast<uint8_t>(
          value & 0xFFU);

  return phase < 128
      ? static_cast<uint8_t>(
            phase * 2U)
      : static_cast<uint8_t>(
            (255U - phase) *
            2U);
}

static uint32_t scaledProfileColor(
    uint8_t key,
    uint8_t scale) {
  uint16_t r =
      static_cast<uint16_t>(
          rgbProfiles[activeProfile][key][0]) *
      scale /
      255U;

  uint16_t g =
      static_cast<uint16_t>(
          rgbProfiles[activeProfile][key][1]) *
      scale /
      255U;

  uint16_t b =
      static_cast<uint16_t>(
          rgbProfiles[activeProfile][key][2]) *
      scale /
      255U;

  return rgbStrip.Color(
      static_cast<uint8_t>(r),
      static_cast<uint8_t>(g),
      static_cast<uint8_t>(b));
}

static void pollRgbEffect(bool force = false) {
  uint8_t effect =
      rgbEffects[activeProfile];

  if (effect == 3) {
    if (force) {
      renderRgbStatic();
    }
    return;
  }

  uint32_t now =
      millis();

  uint16_t interval =
      rgbFrameIntervalMs();

  if (!force &&
      static_cast<uint32_t>(
          now - rgbLastFrameAt) <
          interval) {
    return;
  }

  rgbLastFrameAt =
      now;

  applyRgbBrightness();

  if (effect == 0) {
    // Rainbow: spatial rainbow flowing through logical K1..K8.
    for (uint8_t key = 0; key < KEY_COUNT; ++key) {
      uint16_t hue =
          static_cast<uint16_t>(
              rgbAnimationStep * 512U +
              key *
                  (65535U /
                   KEY_COUNT));

      rgbStrip.setPixelColor(
          KEY_TO_LED[key],
          rgbStrip.ColorHSV(
              hue,
              255,
              255));
    }
  } else if (effect == 1) {
    // Purple Ping-Pong.
    uint8_t phase =
        static_cast<uint8_t>(
            rgbAnimationStep %
            14U);

    uint8_t position =
        phase < 8
            ? phase
            : static_cast<uint8_t>(
                  14U - phase);

    for (uint8_t key = 0; key < KEY_COUNT; ++key) {
      uint8_t distance =
          key > position
              ? key - position
              : position - key;

      uint8_t level =
          distance == 0
              ? 255
              : distance == 1
                  ? 72
                  : 12;

      rgbStrip.setPixelColor(
          KEY_TO_LED[key],
          rgbStrip.Color(
              static_cast<uint8_t>(
                  190U *
                  level /
                  255U),
              static_cast<uint8_t>(
                  40U *
                  level /
                  255U),
              static_cast<uint8_t>(
                  255U *
                  level /
                  255U)));
    }
  } else if (effect == 2) {
    // Orange Blink.
    bool on =
        (rgbAnimationStep &
         1U) == 0;

    uint32_t color =
        on
            ? rgbStrip.Color(
                  255,
                  90,
                  0)
            : rgbStrip.Color(
                  0,
                  0,
                  0);

    for (uint8_t key = 0; key < KEY_COUNT; ++key) {
      rgbStrip.setPixelColor(
          KEY_TO_LED[key],
          color);
    }
  } else if (effect == 4) {
    // Fade: crossfade each saved key color into the next key color.
    uint8_t mix =
        static_cast<uint8_t>(
            rgbAnimationStep &
            0xFFU);

    for (uint8_t key = 0; key < KEY_COUNT; ++key) {
      uint8_t next =
          static_cast<uint8_t>(
              (key + 1U) %
              KEY_COUNT);

      uint16_t inv =
          255U - mix;

      uint8_t r =
          static_cast<uint8_t>(
              (rgbProfiles[activeProfile][key][0] *
                   inv +
               rgbProfiles[activeProfile][next][0] *
                   mix) /
              255U);

      uint8_t g =
          static_cast<uint8_t>(
              (rgbProfiles[activeProfile][key][1] *
                   inv +
               rgbProfiles[activeProfile][next][1] *
                   mix) /
              255U);

      uint8_t b =
          static_cast<uint8_t>(
              (rgbProfiles[activeProfile][key][2] *
                   inv +
               rgbProfiles[activeProfile][next][2] *
                   mix) /
              255U);

      rgbStrip.setPixelColor(
          KEY_TO_LED[key],
          rgbStrip.Color(
              r,
              g,
              b));
    }
  } else if (effect == 5) {
    // Chase: selected per-key colors chase around K1..K8 with a short tail.
    uint8_t head =
        static_cast<uint8_t>(
            rgbAnimationStep %
            KEY_COUNT);

    for (uint8_t key = 0; key < KEY_COUNT; ++key) {
      uint8_t distance =
          static_cast<uint8_t>(
              (head +
               KEY_COUNT -
               key) %
              KEY_COUNT);

      uint8_t level =
          distance == 0
              ? 255
              : distance == 1
                  ? 110
                  : distance == 2
                      ? 42
                      : 6;

      rgbStrip.setPixelColor(
          KEY_TO_LED[key],
          scaledProfileColor(
              key,
              level));
    }
  } else if (effect == 6) {
    // Breathe: all saved per-key colors breathe together.
    uint8_t level =
        static_cast<uint8_t>(
            24U +
            (static_cast<uint16_t>(
                 triangle8(
                     rgbAnimationStep *
                     3U)) *
             231U /
             255U));

    for (uint8_t key = 0; key < KEY_COUNT; ++key) {
      rgbStrip.setPixelColor(
          KEY_TO_LED[key],
          scaledProfileColor(
              key,
              level));
    }
  } else if (effect == 7) {
    // Color Shift: one hue slowly shifts across all keys.
    uint16_t hue =
        static_cast<uint16_t>(
            rgbAnimationStep *
            420U);

    for (uint8_t key = 0; key < KEY_COUNT; ++key) {
      rgbStrip.setPixelColor(
          KEY_TO_LED[key],
          rgbStrip.ColorHSV(
              hue,
              255,
              255));
    }
  } else if (effect == 8) {
    // Rain: deterministic blue/cyan drops with fading trails.
    uint8_t drop =
        static_cast<uint8_t>(
            (rgbAnimationStep *
                 5U +
             (rgbAnimationStep >>
              2U) *
                 3U) %
            KEY_COUNT);

    uint8_t second =
        static_cast<uint8_t>(
            (drop + 3U) %
            KEY_COUNT);

    for (uint8_t key = 0; key < KEY_COUNT; ++key) {
      uint8_t level =
          key == drop
              ? 255
              : key == second
                  ? 150
                  : static_cast<uint8_t>(
                        12U +
                        ((key * 17U +
                          rgbAnimationStep * 11U) %
                         24U));

      rgbStrip.setPixelColor(
          KEY_TO_LED[key],
          rgbStrip.Color(
              0,
              static_cast<uint8_t>(
                  level *
                  3U /
                  5U),
              level));
    }
  } else {
    // Wave: brightness wave travels through the saved per-key colors.
    for (uint8_t key = 0; key < KEY_COUNT; ++key) {
      uint8_t level =
          static_cast<uint8_t>(
              18U +
              (static_cast<uint16_t>(
                   triangle8(
                       rgbAnimationStep *
                           4U +
                       key *
                           28U)) *
               237U /
               255U));

      rgbStrip.setPixelColor(
          KEY_TO_LED[key],
          scaledProfileColor(
              key,
              level));
    }
  }

  rgbStrip.show();
  rgbAnimationStep++;
}

static void applyRgbProfile() {
  rgbAnimationStep = 0;
  rgbLastFrameAt = 0;
  pollRgbEffect(true);
}

static bool parseRgbHex(
    const String &token,
    uint8_t &r,
    uint8_t &g,
    uint8_t &b) {
  if (token.length() != 6) {
    return false;
  }

  char buffer[7] = {};
  token.toCharArray(
      buffer,
      sizeof(buffer));

  char *end = nullptr;
  unsigned long value =
      strtoul(
          buffer,
          &end,
          16);

  if (end == buffer ||
      *end != '\0' ||
      value > 0xFFFFFFUL) {
    return false;
  }

  r = static_cast<uint8_t>(
      (value >> 16) & 0xFF);
  g = static_cast<uint8_t>(
      (value >> 8) & 0xFF);
  b = static_cast<uint8_t>(
      value & 0xFF);

  return true;
}

static String serializeRgbProfile(
    uint8_t profile) {
  String out = "RGB_PROFILE|";
  out += String(profile);
  out += '|';

  char color[7];

  for (uint8_t key = 0; key < KEY_COUNT; ++key) {
    if (key) {
      out += ',';
    }

    snprintf(
        color,
        sizeof(color),
        "%02X%02X%02X",
        rgbProfiles[profile][key][0],
        rgbProfiles[profile][key][1],
        rgbProfiles[profile][key][2]);

    out += color;
  }

  return out;
}

static bool parseRgbProfileCsv(
    const String &csv,
    uint8_t colors[KEY_COUNT][3]) {
  int start = 0;

  for (uint8_t key = 0; key < KEY_COUNT; ++key) {
    int comma =
        csv.indexOf(
            ',',
            start);

    bool last =
        key ==
        KEY_COUNT - 1;

    if ((!last && comma < 0) ||
        (last && comma >= 0)) {
      return false;
    }

    String token =
        last
            ? csv.substring(start)
            : csv.substring(start, comma);

    if (!parseRgbHex(
            token,
            colors[key][0],
            colors[key][1],
            colors[key][2])) {
      return false;
    }

    start =
        comma + 1;
  }

  return true;
}

static uint8_t currentLayer() {
  if (momentaryLayer >= 0 && momentaryLayer < LAYER_COUNT) {
    return static_cast<uint8_t>(momentaryLayer);
  }

  for (int8_t layer = LAYER_COUNT - 1; layer >= 0; --layer) {
    if ((toggledLayerMask & static_cast<uint8_t>(1U << layer)) != 0) {
      return static_cast<uint8_t>(layer);
    }
  }

  return baseLayer;
}


static KeyBinding resolveBinding(
    uint8_t layer,
    uint8_t keyIndex);

static bool saveMainMenuConfig();

static uint8_t effectiveMainMenuAction(
    uint8_t profile,
    uint8_t slot) {
  if (profile >= PROFILE_COUNT ||
      slot >= MENU_SLOT_COUNT) {
    return 0;
  }

  const uint8_t stored =
      mainMenuConfig
          .actions[profile][slot];

  if (stored > 0 &&
      stored <= ACTION_COUNT) {
    return stored;
  }

  // If the dedicated menu map was erased but the keymap survived, use the
  // layer-0 Action binding for the same physical key. This keeps the screen
  // and touch action functional without inventing a factory visual.
  const KeyBinding &binding =
      keymap[profile][0][slot];

  if (binding.type == BIND_ACTION &&
      binding.keyCode >= 1 &&
      binding.keyCode <= ACTION_COUNT) {
    return binding.keyCode;
  }

  return 0;
}

static bool recoverMainMenuActionsFromKeymap() {
  bool changed = false;

  for (uint8_t profile = 0;
       profile < PROFILE_COUNT;
       ++profile) {
    for (uint8_t slot = 0;
         slot < MENU_SLOT_COUNT;
         ++slot) {
      if (mainMenuConfig.actions[profile][slot] != 0) {
        continue;
      }

      const KeyBinding &binding =
          keymap[profile][0][slot];

      if (binding.type != BIND_ACTION ||
          binding.keyCode < 1 ||
          binding.keyCode > ACTION_COUNT) {
        continue;
      }

      mainMenuConfig.actions[profile][slot] =
          binding.keyCode;

      if (mainMenuConfig.labels[profile][slot][0] == '\0') {
        snprintf(
            mainMenuConfig.labels[profile][slot],
            MENU_LABEL_MAX_LEN + 1,
            "A%02u",
            static_cast<unsigned>(
                binding.keyCode));
      }

      changed = true;
    }
  }

  if (!changed) {
    return false;
  }

  return saveMainMenuConfig();
}


static void setDefaultMainMenuConfig() {
  memset(&mainMenuConfig, 0, sizeof(mainMenuConfig));
  mainMenuConfig.version = MENU_STORAGE_VERSION;
}

static bool mainMenuConfigIsValid(
    MainMenuConfig &config) {
  if (config.version !=
      MENU_STORAGE_VERSION) {
    return false;
  }

  for (uint8_t profile = 0;
       profile < PROFILE_COUNT;
       ++profile) {
    for (uint8_t slot = 0;
         slot < MENU_SLOT_COUNT;
         ++slot) {
      if (config.actions[profile][slot] >
          ACTION_COUNT) {
        return false;
      }

      config.labels[profile][slot]
          [MENU_LABEL_MAX_LEN] = '\0';
    }
  }

  return true;
}

static bool saveMainMenuConfigFile() {
  if (!littleFsReady) {
    return false;
  }

  File file =
      LittleFS.open(
          MENU_CONFIG_FILE_PATH,
          "w");

  if (!file) {
    return false;
  }

  size_t written =
      file.write(
          reinterpret_cast<
              const uint8_t *>(
              &mainMenuConfig),
          sizeof(mainMenuConfig));

  file.flush();
  file.close();

  return written ==
             sizeof(mainMenuConfig) &&
         LittleFS.exists(
             MENU_CONFIG_FILE_PATH);
}

static bool loadMainMenuConfigFile() {
  if (!littleFsReady ||
      !LittleFS.exists(
          MENU_CONFIG_FILE_PATH)) {
    return false;
  }

  File file =
      LittleFS.open(
          MENU_CONFIG_FILE_PATH,
          "r");

  if (!file ||
      file.size() !=
          sizeof(MainMenuConfig)) {
    if (file) {
      file.close();
    }

    return false;
  }

  MainMenuConfig stored = {};

  size_t read =
      file.read(
          reinterpret_cast<uint8_t *>(
              &stored),
          sizeof(stored));

  file.close();

  if (read != sizeof(stored) ||
      !mainMenuConfigIsValid(
          stored)) {
    return false;
  }

  memcpy(
      &mainMenuConfig,
      &stored,
      sizeof(mainMenuConfig));

  preferences.putUChar(
      "menuver",
      MENU_STORAGE_VERSION);

  preferences.putBytes(
      "menucfg",
      &mainMenuConfig,
      sizeof(mainMenuConfig));

  return true;
}

static bool saveMainMenuConfig() {
  mainMenuConfig.version =
      MENU_STORAGE_VERSION;

  size_t versionWritten =
      preferences.putUChar(
          "menuver",
          MENU_STORAGE_VERSION);

  size_t configWritten =
      preferences.putBytes(
          "menucfg",
          &mainMenuConfig,
          sizeof(mainMenuConfig));

  bool nvsSaved =
      versionWritten == sizeof(uint8_t) &&
      configWritten == sizeof(mainMenuConfig) &&
      preferences.getBytesLength(
          "menucfg") ==
          sizeof(mainMenuConfig);

  if (littleFsReady) {
    return saveMainMenuConfigFile() &&
           nvsSaved;
  }

  return nvsSaved;
}

static void loadMainMenuConfig() {
  setDefaultMainMenuConfig();

  uint8_t storedVersion =
      preferences.getUChar("menuver", 0);

  size_t storedBytes =
      preferences.getBytesLength("menucfg");

  if (storedVersion == 3 &&
      storedBytes == sizeof(LegacyMainMenuConfigV3)) {
    LegacyMainMenuConfigV3 legacy = {};

    if (preferences.getBytes(
            "menucfg",
            &legacy,
            sizeof(legacy)) ==
            sizeof(legacy) &&
        legacy.version == 3) {
      for (uint8_t profile = 0;
           profile < PROFILE_COUNT;
           ++profile) {
        for (uint8_t slot = 0;
             slot < MENU_SLOT_COUNT;
             ++slot) {
          mainMenuConfig.actions[profile][slot] =
              legacy.actions[profile][slot];

          memcpy(
              mainMenuConfig.labels[profile][slot],
              legacy.labels[profile][slot],
              MENU_LABEL_MAX_LEN + 1);

          mainMenuConfig.labels[profile][slot]
              [MENU_LABEL_MAX_LEN] = '\0';
        }
      }

      saveMainMenuConfig();
      return;
    }
  }

  if (storedVersion != MENU_STORAGE_VERSION ||
      storedBytes != sizeof(mainMenuConfig)) {
    saveMainMenuConfig();
    return;
  }

  MainMenuConfig stored = {};
  if (preferences.getBytes(
          "menucfg",
          &stored,
          sizeof(stored)) != sizeof(stored) ||
      stored.version != MENU_STORAGE_VERSION) {
    saveMainMenuConfig();
    return;
  }

  for (uint8_t profile = 0;
       profile < PROFILE_COUNT;
       ++profile) {
    for (uint8_t slot = 0;
         slot < MENU_SLOT_COUNT;
         ++slot) {
      if (stored.actions[profile][slot] > ACTION_COUNT) {
        saveMainMenuConfig();
        return;
      }

      stored.labels[profile][slot]
          [MENU_LABEL_MAX_LEN] = '\0';
    }
  }

  memcpy(
      &mainMenuConfig,
      &stored,
      sizeof(mainMenuConfig));
}

static void loadActiveProfileState() {
  activeProfile =
      preferences.getUChar(
          "activeprof",
          0);

  baseLayer =
      preferences.getUChar(
          "activelayer",
          0);

  if (activeProfile >= PROFILE_COUNT) {
    activeProfile = 0;
  }

  if (baseLayer >= LAYER_COUNT) {
    baseLayer = 0;
  }
}

static bool saveActiveProfileState() {
  size_t profileWritten =
      preferences.putUChar(
          "activeprof",
          activeProfile);

  size_t layerWritten =
      preferences.putUChar(
          "activelayer",
          baseLayer);

  return profileWritten == sizeof(uint8_t) &&
         layerWritten == sizeof(uint8_t);
}


static void menuBackgroundPath(
    uint8_t profile,
    bool temporary,
    char *out,
    size_t outSize) {
  snprintf(
      out,
      outSize,
      temporary ? "/mb%u.tmp" : "/mb%u.jpg",
      static_cast<unsigned>(profile));
}


static void menuCompositeFlagPath(
    uint8_t profile,
    char *out,
    size_t outSize) {
  snprintf(
      out,
      outSize,
      "/mc%u.flag",
      static_cast<unsigned>(
          profile));
}


static bool mainMenuCompositeEnabled(
    uint8_t profile) {
  if (profile >= PROFILE_COUNT ||
      !littleFsReady) {
    return false;
  }

  char backgroundPath[24] = {};
  menuBackgroundPath(
      profile,
      false,
      backgroundPath,
      sizeof(backgroundPath));

  if (!LittleFS.exists(
          backgroundPath)) {
    return false;
  }

  char flagPath[24] = {};
  menuCompositeFlagPath(
      profile,
      flagPath,
      sizeof(flagPath));

  const bool persistedWithAsset =
      LittleFS.exists(
          flagPath);

  const bool persistedInNvs =
      (menuCompositeMask &
       (1UL << profile)) != 0;

  return persistedWithAsset ||
         persistedInNvs;
}

static bool setMainMenuCompositeEnabled(
    uint8_t profile,
    bool enabled) {
  if (profile >= PROFILE_COUNT) {
    return false;
  }

  const uint32_t bit =
      1UL << profile;

  if (enabled) {
    menuCompositeMask |= bit;
  } else {
    menuCompositeMask &= ~bit;
  }

  bool fsOk = true;

  if (littleFsReady) {
    char flagPath[24] = {};
    menuCompositeFlagPath(
        profile,
        flagPath,
        sizeof(flagPath));

    if (enabled) {
      File flag =
          LittleFS.open(
              flagPath,
              "w");

      if (!flag) {
        fsOk = false;
      } else {
        const uint8_t marker[4] = {
            'P',
            'X',
            'M',
            '1'};

        fsOk =
            flag.write(
                marker,
                sizeof(marker)) ==
            sizeof(marker);

        flag.close();
      }
    } else {
      LittleFS.remove(
          flagPath);
    }
  }

  const bool nvsOk =
      preferences.putUInt(
          "menucomp",
          menuCompositeMask) ==
      sizeof(uint32_t);

  return fsOk &&
         nvsOk;
}

static void menuIconPath(
    uint8_t profile,
    uint8_t slot,
    bool temporary,
    char *out,
    size_t outSize) {
  snprintf(
      out,
      outSize,
      temporary ? "/mi%u_%u.tmp" : "/mi%u_%u.pxi",
      static_cast<unsigned>(profile),
      static_cast<unsigned>(slot));
}

static void menuBackgroundBackupPath(
    uint8_t profile,
    char *out,
    size_t outSize) {
  snprintf(
      out,
      outSize,
      "/mb%u.bak",
      static_cast<unsigned>(profile));
}

static void menuIconBackupPath(
    uint8_t profile,
    uint8_t slot,
    char *out,
    size_t outSize) {
  snprintf(
      out,
      outSize,
      "/mi%u_%u.bak",
      static_cast<unsigned>(profile),
      static_cast<unsigned>(slot));
}


static void menuLegacyIconJpegPath(
    uint8_t profile,
    uint8_t slot,
    char *out,
    size_t outSize) {
  snprintf(
      out,
      outSize,
      "/mi%u_%u.jpg",
      static_cast<unsigned>(profile),
      static_cast<unsigned>(slot));
}


static uint8_t vividClamp8(
    int value) {
  if (value < 0) {
    return 0;
  }
  if (value > 255) {
    return 255;
  }
  return static_cast<uint8_t>(value);
}

static uint16_t vividRgb565(
    uint16_t color) {
  int r =
      static_cast<int>((color >> 11) & 0x1F) *
      255 / 31;
  int g =
      static_cast<int>((color >> 5) & 0x3F) *
      255 / 63;
  int b =
      static_cast<int>(color & 0x1F) *
      255 / 31;

  const int luma =
      (r * 77 +
       g * 150 +
       b * 29) >>
      8;

  r = luma + (r - luma) * 118 / 100;
  g = luma + (g - luma) * 118 / 100;
  b = luma + (b - luma) * 118 / 100;

  r = 128 + (r - 128) * 112 / 100;
  g = 128 + (g - 128) * 112 / 100;
  b = 128 + (b - 128) * 112 / 100;

  const uint8_t rr = vividClamp8(r);
  const uint8_t gg = vividClamp8(g);
  const uint8_t bb = vividClamp8(b);

  return static_cast<uint16_t>(
      ((static_cast<uint16_t>(rr) * 31U / 255U) << 11) |
      ((static_cast<uint16_t>(gg) * 63U / 255U) << 5) |
      (static_cast<uint16_t>(bb) * 31U / 255U));
}

static void drawVividRgb565Row(
    int16_t x,
    int16_t y,
    const uint16_t *pixels,
    uint16_t width) {
  if (pixels == nullptr ||
      width == 0 ||
      width > TFT_WIDTH) {
    return;
  }

  for (uint16_t i = 0;
       i < width;
       ++i) {
    displayVividLine[i] =
        vividRgb565(
            pixels[i]);
  }

  tft->draw16bitRGBBitmap(
      x,
      y,
      displayVividLine,
      width,
      1);
}

static bool profileHasMainMenuContent(
    uint8_t profile) {
  if (profile >= PROFILE_COUNT) {
    return false;
  }

  for (uint8_t slot = 0;
       slot < MENU_SLOT_COUNT;
       ++slot) {
    if (effectiveMainMenuAction(
            profile,
            slot) > 0) {
      return true;
    }
  }

  if (!littleFsReady) {
    return false;
  }

  char bgPath[24] = {};
  menuBackgroundPath(
      profile,
      false,
      bgPath,
      sizeof(bgPath));

  if (LittleFS.exists(bgPath)) {
    return true;
  }

  for (uint8_t slot = 0;
       slot < MENU_SLOT_COUNT;
       ++slot) {
    char iconPath[24] = {};
    menuIconPath(
        profile,
        slot,
        false,
        iconPath,
        sizeof(iconPath));

    if (LittleFS.exists(iconPath)) {
      return true;
    }

    char legacyPath[24] = {};
    menuLegacyIconJpegPath(
        profile,
        slot,
        legacyPath,
        sizeof(legacyPath));

    if (LittleFS.exists(legacyPath)) {
      return true;
    }
  }

  return false;
}

static void rememberMainMenuContentProfile(
    uint8_t profile) {
  if (profile >= PROFILE_COUNT) {
    return;
  }

  menuLastContentProfile = profile;
  preferences.putUChar(
      "menulast",
      profile);
}

static uint8_t resolveMainMenuRenderProfile() {
  if (activeProfile < PROFILE_COUNT &&
      profileHasMainMenuContent(
          activeProfile)) {
    return activeProfile;
  }

  if (menuLastContentProfile < PROFILE_COUNT &&
      profileHasMainMenuContent(
          menuLastContentProfile)) {
    return menuLastContentProfile;
  }

  for (uint8_t profile = 0;
       profile < PROFILE_COUNT;
       ++profile) {
    if (profileHasMainMenuContent(
            profile)) {
      return profile;
    }
  }

  return activeProfile < PROFILE_COUNT
      ? activeProfile
      : 0;
}

static int mainMenuJpegDraw(JPEGDRAW *draw) {
  if (draw == nullptr ||
      draw->pPixels == nullptr ||
      !displayReady) {
    return 0;
  }

  int width =
      draw->iWidthUsed > 0
          ? draw->iWidthUsed
          : draw->iWidth;

  if (draw->x < 0 ||
      draw->y < 0 ||
      draw->x + width > TFT_WIDTH ||
      draw->y + draw->iHeight > TFT_HEIGHT) {
    return 0;
  }

  for (int row = 0;
       row < draw->iHeight;
       ++row) {
    drawVividRgb565Row(
        draw->x,
        draw->y + row,
        draw->pPixels +
            static_cast<size_t>(row) *
                draw->iWidth,
        static_cast<uint16_t>(
            width));

    if ((row & 0x03) == 0x03) {
      delay(0);
    }
  }

  return 1;
}

static bool renderMainMenuBackground(
    uint8_t profile) {
  if (profile >= PROFILE_COUNT) {
    profile = 0;
  }

  char path[24] = {};
  menuBackgroundPath(
      profile,
      false,
      path,
      sizeof(path));

  if (!littleFsReady ||
      !LittleFS.exists(path)) {
    return false;
  }

  File file =
      LittleFS.open(
          path,
          "r");

  if (!file) {
    return false;
  }

  JPEGDEC decoder;

  const bool valid =
      decoder.open(
          file,
          mainMenuJpegDraw) &&
      decoder.getWidth() ==
          TFT_WIDTH &&
      decoder.getHeight() ==
          TFT_HEIGHT;

  if (!valid) {
    decoder.close();
    file.close();
    return false;
  }

  tft->fillScreen(
      RGB565_BLACK);

  const int result =
      decoder.decode(
          0,
          0,
          0);

  decoder.close();
  file.close();

  return result != 0;
}

static bool renderMainMenuIcon(
    uint8_t profile,
    uint8_t slot,
    int x,
    int y) {
  if (!littleFsReady ||
      profile >= PROFILE_COUNT ||
      slot >= MENU_SLOT_COUNT) {
    return false;
  }

  char path[24] = {};
  menuIconPath(
      profile,
      slot,
      false,
      path,
      sizeof(path));

  if (LittleFS.exists(path)) {
    File file =
        LittleFS.open(
            path,
            "r");

    if (file) {
      uint8_t header[8] = {};

      bool valid =
          file.size() == MENU_ICON_ASSET_BYTES &&
          file.read(
              header,
              sizeof(header)) ==
              sizeof(header) &&
          header[0] == 'P' &&
          header[1] == 'X' &&
          header[2] == 'I' &&
          header[3] == '1';

      uint16_t width =
          static_cast<uint16_t>(
              header[4] |
              (static_cast<uint16_t>(
                   header[5]) <<
               8));

      uint16_t height =
          static_cast<uint16_t>(
              header[6] |
              (static_cast<uint16_t>(
                   header[7]) <<
               8));

      valid =
          valid &&
          width == MENU_ICON_WIDTH &&
          height == MENU_ICON_HEIGHT;

      if (valid) {
        uint16_t rowPixels[MENU_ICON_WIDTH] = {};

        for (uint16_t row = 0;
             row < MENU_ICON_HEIGHT;
             ++row) {
          size_t bytesRead =
              file.read(
                  reinterpret_cast<uint8_t *>(
                      rowPixels),
                  sizeof(rowPixels));

          if (bytesRead !=
              sizeof(rowPixels)) {
            valid = false;
            break;
          }

          for (uint16_t col = 0;
               col < MENU_ICON_WIDTH;
               ++col) {
            if (rowPixels[col] !=
                MENU_ICON_TRANSPARENT) {
              rowPixels[col] =
                  vividRgb565(
                      rowPixels[col]);
            }
          }

          int runStart = -1;

          for (int col = 0;
               col <= MENU_ICON_WIDTH;
               ++col) {
            bool transparent =
                col == MENU_ICON_WIDTH ||
                rowPixels[col] ==
                    MENU_ICON_TRANSPARENT;

            if (!transparent &&
                runStart < 0) {
              runStart = col;
            }

            if (transparent &&
                runStart >= 0) {
              tft->draw16bitRGBBitmap(
                  x + runStart,
                  y + row,
                  rowPixels + runStart,
                  col - runStart,
                  1);

              runStart = -1;
            }
          }
        }
      }

      file.close();

      if (valid) {
        return true;
      }
    }
  }

  // Preserve icons already uploaded by firmware 1.8.2 until the new app
  // replaces them with transparent PXI1 assets.
  char legacyPath[24] = {};
  menuLegacyIconJpegPath(
      profile,
      slot,
      legacyPath,
      sizeof(legacyPath));

  if (!LittleFS.exists(legacyPath)) {
    return false;
  }

  File legacy =
      LittleFS.open(
          legacyPath,
          "r");

  if (!legacy) {
    return false;
  }

  JPEGDEC decoder;
  bool valid =
      decoder.open(
          legacy,
          mainMenuJpegDraw) &&
      decoder.getWidth() == MENU_ICON_WIDTH &&
      decoder.getHeight() == MENU_ICON_HEIGHT;

  if (!valid) {
    decoder.close();
    legacy.close();
    return false;
  }

  int result =
      decoder.decode(
          x,
          y,
          0);

  decoder.close();
  legacy.close();

  return result != 0;
}

static void drawDockWindowsStart(
    int x,
    int y) {
  const uint16_t blue =
      0x051F;

  tft->fillRect(
      x + 3,
      y + 3,
      18,
      18,
      blue);

  tft->fillRect(
      x + 27,
      y + 3,
      18,
      18,
      blue);

  tft->fillRect(
      x + 3,
      y + 27,
      18,
      18,
      blue);

  tft->fillRect(
      x + 27,
      y + 27,
      18,
      18,
      blue);
}

static void drawDockMacLaunchpad(
    int x,
    int y) {
  static const uint16_t colors[9] = {
      0x4CFF,
      0x92BF,
      0xFAEC,
      0x2E6F,
      0xFDA6,
      0x5E7F,
      0xFB95,
      0x7BFF,
      0x65EC
  };

  for (int row = 0;
       row < 3;
       ++row) {
    for (int col = 0;
         col < 3;
         ++col) {
      tft->fillRoundRect(
          x + 4 +
              col * 15,
          y + 4 +
              row * 15,
          11,
          11,
          3,
          colors[
              row * 3 +
              col]);
    }
  }
}

static void drawDockLinuxApps(
    int x,
    int y) {
  const uint16_t white =
      0xFFFF;

  for (int row = 0;
       row < 3;
       ++row) {
    for (int col = 0;
         col < 3;
         ++col) {
      tft->fillCircle(
          x + 8 +
              col * 16,
          y + 8 +
              row * 16,
          4,
          white);
    }
  }
}

static void drawDockWindowsFolder(
    int x,
    int y) {
  const uint16_t yellow =
      0xFE66;

  const uint16_t gold =
      0xFDC0;

  const uint16_t blue =
      0x4C7F;

  tft->fillRoundRect(
      x + 3,
      y + 17,
      45,
      29,
      5,
      yellow);

  tft->fillRoundRect(
      x + 7,
      y + 10,
      22,
      13,
      4,
      gold);

  tft->fillRoundRect(
      x + 28,
      y + 14,
      17,
      7,
      3,
      blue);
}

static void drawDockMacFinder(
    int x,
    int y) {
  const uint16_t lightBlue =
      0x5D7F;

  const uint16_t deepBlue =
      0x2C9F;

  const uint16_t ink =
      0x09AA;

  tft->fillRoundRect(
      x + 1,
      y + 1,
      48,
      48,
      9,
      lightBlue);

  tft->fillRect(
      x + 25,
      y + 1,
      24,
      48,
      deepBlue);

  tft->drawLine(
      x + 25,
      y + 5,
      x + 25,
      y + 44,
      ink);

  tft->fillCircle(
      x + 16,
      y + 22,
      2,
      ink);

  tft->fillCircle(
      x + 34,
      y + 22,
      2,
      ink);

  tft->drawLine(
      x + 14,
      y + 32,
      x + 22,
      y + 36,
      ink);

  tft->drawLine(
      x + 22,
      y + 36,
      x + 34,
      y + 31,
      ink);
}

static void drawDockLinuxFolder(
    int x,
    int y) {
  const uint16_t blue =
      0x4C7F;

  const uint16_t darkBlue =
      0x337A;

  tft->fillRoundRect(
      x + 3,
      y + 17,
      45,
      29,
      5,
      blue);

  tft->fillRoundRect(
      x + 7,
      y + 10,
      22,
      13,
      4,
      darkBlue);
}

static void drawDockBrowser(
    int x,
    int y,
    uint8_t hostOs) {
  if (hostOs == 2) {
    const uint16_t blue =
        0x35FF;

    const uint16_t white =
        0xFFFF;

    const uint16_t red =
        0xF249;

    tft->fillCircle(
        x + 25,
        y + 25,
        23,
        blue);

    tft->drawCircle(
        x + 25,
        y + 25,
        20,
        white);

    tft->drawCircle(
        x + 25,
        y + 25,
        16,
        white);

    tft->drawLine(
        x + 20,
        y + 36,
        x + 30,
        y + 13,
        red);

    tft->drawLine(
        x + 30,
        y + 13,
        x + 27,
        y + 27,
        white);

    tft->fillCircle(
        x + 25,
        y + 25,
        2,
        white);

    return;
  }

  if (hostOs == 3) {
    const uint16_t purple =
        0x632D;

    const uint16_t orange =
        0xFB84;

    const uint16_t blue =
        0x3B9A;

    tft->fillCircle(
        x + 25,
        y + 25,
        23,
        purple);

    tft->fillCircle(
        x + 29,
        y + 22,
        18,
        orange);

    tft->fillCircle(
        x + 25,
        y + 27,
        13,
        blue);

    tft->drawLine(
        x + 12,
        y + 12,
        x + 21,
        y + 7,
        orange);

    return;
  }

  const uint16_t teal =
      0x16D3;

  const uint16_t blue =
      0x04FF;

  const uint16_t deepBlue =
      0x02D2;

  tft->fillCircle(
      x + 25,
      y + 25,
      23,
      teal);

  tft->fillCircle(
      x + 29,
      y + 30,
      18,
      blue);

  tft->fillCircle(
      x + 30,
      y + 25,
      10,
      deepBlue);

  tft->drawLine(
      x + 7,
      y + 28,
      x + 43,
      y + 28,
      0xFFFF);
}

static void drawDockSettings(
    int x,
    int y) {
  const int cx =
      x + 25;

  const int cy =
      y + 25;

  const uint16_t silver =
      0x9CF3;

  const uint16_t white =
      0xFFFF;

  tft->fillCircle(
      cx,
      cy,
      17,
      silver);

  tft->drawCircle(
      cx,
      cy,
      18,
      white);

  tft->fillCircle(
      cx,
      cy,
      7,
      0x3186);

  for (int i = 0;
       i < 8;
       ++i) {
    int dx = 0;
    int dy = 0;

    switch (i) {
      case 0:
        dx = 0;
        dy = -23;
        break;

      case 1:
        dx = 16;
        dy = -16;
        break;

      case 2:
        dx = 23;
        dy = 0;
        break;

      case 3:
        dx = 16;
        dy = 16;
        break;

      case 4:
        dx = 0;
        dy = 23;
        break;

      case 5:
        dx = -16;
        dy = 16;
        break;

      case 6:
        dx = -23;
        dy = 0;
        break;

      default:
        dx = -16;
        dy = -16;
        break;
    }

    tft->drawLine(
        cx +
            dx * 13 /
                23,
        cy +
            dy * 13 /
                23,
        cx + dx,
        cy + dy,
        silver);
  }
}

static void renderMainMenuStatusBar() {
  // 1.10.3: the previous four OS-style dock glyphs were decorative only and
  // could not be pressed. Keep this hook as a no-op for existing callers so
  // every visible Main Menu item is now one of the eight real touch slots.
  return;

  if (!displayReady ||
      saverActive) {
    return;
  }

  const int tileY =
      TFT_HEIGHT -
      50;

  const int firstX =
      24;

  const int step =
      120;

  uint8_t hostOs =
      menuHostOs >= 1 &&
      menuHostOs <= 3
          ? menuHostOs
          : 1;

  if (hostOs == 2) {
    // macOS: Launchpad, Finder, Safari, Settings.
    drawDockMacLaunchpad(
        firstX,
        tileY);

    drawDockMacFinder(
        firstX + step,
        tileY);
  } else if (hostOs == 3) {
    // Linux/GNOME: Applications, Files, Browser, Settings.
    drawDockLinuxApps(
        firstX,
        tileY);

    drawDockLinuxFolder(
        firstX + step,
        tileY);
  } else {
    // Windows: Start, Explorer, Edge, Settings.
    drawDockWindowsStart(
        firstX,
        tileY);

    drawDockWindowsFolder(
        firstX + step,
        tileY);
  }

  drawDockBrowser(
      firstX + step * 2,
      tileY,
      hostOs);

  drawDockSettings(
      firstX + step * 3,
      tileY);
}

static const char *fallbackConsumerName(
    uint16_t usage) {
  switch (usage) {
    case 0x00E9:
      return "Volume +";
    case 0x00EA:
      return "Volume -";
    case 0x00E2:
      return "Mute";
    case 0x00CD:
      return "Play/Pause";
    case 0x00B5:
      return "Next";
    case 0x00B6:
      return "Previous";
    case 0x00B7:
      return "Stop";
    default:
      return nullptr;
  }
}

static void fallbackKeyboardName(
    uint8_t keyCode,
    char *out,
    size_t outSize) {
  if (keyCode >= 0x04 &&
      keyCode <= 0x1D) {
    snprintf(
        out,
        outSize,
        "%c",
        static_cast<char>(
            'A' +
            keyCode -
            0x04));
    return;
  }

  if (keyCode >= 0x1E &&
      keyCode <= 0x26) {
    snprintf(
        out,
        outSize,
        "%u",
        static_cast<unsigned>(
            keyCode -
            0x1D));
    return;
  }

  if (keyCode == 0x27) {
    snprintf(
        out,
        outSize,
        "0");
    return;
  }

  if (keyCode >= 0x3A &&
      keyCode <= 0x45) {
    snprintf(
        out,
        outSize,
        "F%u",
        static_cast<unsigned>(
            keyCode -
            0x39));
    return;
  }

  const char *name = nullptr;

  switch (keyCode) {
    case 0x28:
      name = "Enter";
      break;
    case 0x29:
      name = "Esc";
      break;
    case 0x2A:
      name = "Backspace";
      break;
    case 0x2B:
      name = "Tab";
      break;
    case 0x2C:
      name = "Space";
      break;
    case 0x4A:
      name = "Home";
      break;
    case 0x4B:
      name = "Page Up";
      break;
    case 0x4D:
      name = "End";
      break;
    case 0x4E:
      name = "Page Down";
      break;
    case 0x4F:
      name = "Right";
      break;
    case 0x50:
      name = "Left";
      break;
    case 0x51:
      name = "Down";
      break;
    case 0x52:
      name = "Up";
      break;
    case 0x49:
      name = "Insert";
      break;
    case 0x4C:
      name = "Delete";
      break;
    default:
      break;
  }

  if (name != nullptr) {
    snprintf(
        out,
        outSize,
        "%s",
        name);
    return;
  }

  snprintf(
      out,
      outSize,
      "Key %02X",
      static_cast<unsigned>(
          keyCode));
}

static void fallbackBindingLabel(
    uint8_t slot,
    char *out,
    size_t outSize) {
  if (out == nullptr ||
      outSize == 0 ||
      slot >= KEY_COUNT) {
    return;
  }

  const KeyBinding binding =
      resolveBinding(
          currentLayer(),
          slot);

  out[0] = '\0';

  switch (binding.type) {
    case BIND_KEYBOARD: {
      char keyName[18] = {};
      fallbackKeyboardName(
          binding.keyCode,
          keyName,
          sizeof(keyName));

      char mods[12] = {};

      if ((binding.modifiers & 0x01U) != 0) {
        strncat(
            mods,
            "Ctrl+",
            sizeof(mods) -
                strlen(mods) -
                1);
      }

      if ((binding.modifiers & 0x02U) != 0) {
        strncat(
            mods,
            "Shift+",
            sizeof(mods) -
                strlen(mods) -
                1);
      }

      if ((binding.modifiers & 0x04U) != 0) {
        strncat(
            mods,
            "Alt+",
            sizeof(mods) -
                strlen(mods) -
                1);
      }

      if ((binding.modifiers & 0x08U) != 0) {
        strncat(
            mods,
            "Win+",
            sizeof(mods) -
                strlen(mods) -
                1);
      }

      snprintf(
          out,
          outSize,
          "%s%s",
          mods,
          keyName);
      break;
    }

    case BIND_CONSUMER: {
      const char *name =
          fallbackConsumerName(
              binding.consumerCode);

      if (name != nullptr) {
        snprintf(
            out,
            outSize,
            "%s",
            name);
      } else {
        snprintf(
            out,
            outSize,
            "Media %04X",
            static_cast<unsigned>(
                binding.consumerCode));
      }
      break;
    }

    case BIND_LAYER: {
      const char *mode =
          binding.modifiers ==
                  LAYER_MO
              ? "Hold"
              : binding.modifiers ==
                        LAYER_TG
                    ? "Toggle"
                    : "Layer";

      snprintf(
          out,
          outSize,
          "%s L%u",
          mode,
          static_cast<unsigned>(
              binding.keyCode));
      break;
    }

    case BIND_MACRO:
      snprintf(
          out,
          outSize,
          "Macro %u",
          static_cast<unsigned>(
              binding.keyCode + 1));
      break;

    case BIND_ACTION: {
      const char *label =
          mainMenuConfig
              .labels[activeProfile][slot];

      if (label[0] != '\0') {
        snprintf(
            out,
            outSize,
            "%s",
            label);
      } else {
        snprintf(
            out,
            outSize,
            "Action %u",
            static_cast<unsigned>(
                binding.keyCode));
      }
      break;
    }

    case BIND_TRANSPARENT:
      snprintf(
          out,
          outSize,
          "Transparent");
      break;

    case BIND_DISABLED:
    default:
      snprintf(
          out,
          outSize,
          "Not assigned");
      break;
  }
}

static void drawMainMenuEmptyState(
    uint8_t profile,
    bool imageError) {
  if (!displayReady) {
    return;
  }

  // Never leave the panel looking dead. This is a diagnostic/status screen,
  // not a default Main Menu asset and it contains no K1-K8 fallback keys.
  tft->fillScreen(
      0x1082);

  tft->fillRoundRect(
      28,
      44,
      TFT_WIDTH - 56,
      TFT_HEIGHT - 88,
      18,
      0x18E3);

  tft->drawRoundRect(
      28,
      44,
      TFT_WIDTH - 56,
      TFT_HEIGHT - 88,
      18,
      0x632C);

  tft->setTextColor(
      0xFFFF);

  tft->setTextSize(3);
  tft->setCursor(
      134,
      78);
  tft->print(
      "PIXEL PRO");

  tft->setTextSize(2);
  tft->setCursor(
      imageError
          ? 112
          : 126,
      132);

  tft->print(
      imageError
          ? "MAIN MENU IMAGE ERROR"
          : "MAIN MENU EMPTY");

  tft->setTextSize(1);
  tft->setTextColor(
      0xC618);

  tft->setCursor(
      112,
      181);

  tft->print(
      "Open LumiPad > Main Menu > Save");

  char line[48] = {};
  snprintf(
      line,
      sizeof(line),
      "Profile %02u  FW %s",
      static_cast<unsigned>(
          profile + 1),
      FW_VERSION);

  tft->setCursor(
      170,
      211);

  tft->print(
      line);
}

static void renderMainMenu() {
  if (!displayReady ||
      saverActive) {
    return;
  }

  const uint8_t profile =
      resolveMainMenuRenderProfile();

  menuRenderedProfile = profile;

  if (profileHasMainMenuContent(
          profile)) {
    rememberMainMenuContentProfile(
        profile);
  }

  const bool backgroundDrawn =
      renderMainMenuBackground(
          profile);

  if (mainMenuCompositeEnabled(
          profile)) {
    if (!backgroundDrawn) {
      drawMainMenuEmptyState(
          profile,
          true);
    }

    renderMainMenuStatusBar();
    return;
  }

  if (!backgroundDrawn) {
    tft->fillScreen(
        0x1082);
  }

  const int statusY =
      TFT_HEIGHT -
      MENU_STATUS_HEIGHT;

  const int marginX = 10;
  const int marginY = 6;
  const int gapX = 6;
  const int gapY = 4;

  const int cellW =
      (TFT_WIDTH -
       marginX * 2 -
       gapX * 3) /
      4;

  const int cellH =
      (statusY -
       marginY * 2 -
       gapY) /
      2;

  bool hasVisibleMenuItem = false;

  for (uint8_t slot = 0;
       slot < MENU_SLOT_COUNT;
       ++slot) {
    int col =
        slot % 4;

    int row =
        slot / 4;

    int x =
        marginX +
        col *
            (cellW + gapX);

    int y =
        marginY +
        row *
            (cellH + gapY);

    int iconX =
        x +
        (cellW -
         MENU_ICON_WIDTH) /
            2;

    int iconY =
        y +
        (cellH -
         MENU_ICON_HEIGHT) /
            2;

    bool drewIcon =
        renderMainMenuIcon(
            profile,
            slot,
            iconX,
            iconY);

    uint8_t action =
        effectiveMainMenuAction(
            profile,
            slot);

    if (drewIcon ||
        action > 0) {
      hasVisibleMenuItem = true;
    }

    // Match the requested eezBotFun behavior: custom icons stand alone.
    // The action name is shown only as a fallback when no icon is assigned.
    if (!drewIcon &&
        action > 0) {
      const char *label =
          mainMenuConfig
              .labels[profile][slot];

      char fallback[8] = {};

      if (label[0] == '\0') {
        snprintf(
            fallback,
            sizeof(fallback),
            "A%02u",
            static_cast<unsigned>(
                action));

        label =
            fallback;
      }

      size_t len =
          strnlen(
              label,
              MENU_LABEL_MAX_LEN);

      int textWidth =
          static_cast<int>(len) *
          6;

      int textX =
          x +
          max(
              2,
              (cellW -
               textWidth) /
                  2);

      int textY =
          y +
          (cellH - 8) /
              2;

      tft->setTextSize(1);
      tft->setTextColor(
          0x0000);

      tft->setCursor(
          textX + 1,
          textY + 1);

      tft->print(
          label);

      tft->setTextColor(
          0xFFFF);

      tft->setCursor(
          textX,
          textY);

      tft->print(
          label);
    }
  }

  if (!backgroundDrawn &&
      !hasVisibleMenuItem) {
    drawMainMenuEmptyState(
        profile,
        false);
  }

  renderMainMenuStatusBar();
}

static const char *resetReasonName(
    esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return "POWERON";
    case ESP_RST_EXT:
      return "EXT";
    case ESP_RST_SW:
      return "SW";
    case ESP_RST_PANIC:
      return "PANIC";
    case ESP_RST_INT_WDT:
      return "INT_WDT";
    case ESP_RST_TASK_WDT:
      return "TASK_WDT";
    case ESP_RST_WDT:
      return "WDT";
    case ESP_RST_DEEPSLEEP:
      return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT";
    case ESP_RST_SDIO:
      return "SDIO";
    case ESP_RST_UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

static void scheduleMainMenuRender(
    uint8_t profile,
    uint32_t delayMs = MENU_RENDER_DEFER_MS) {
  if (profile >= PROFILE_COUNT ||
      profile != activeProfile ||
      saverActive ||
      !displayReady) {
    return;
  }

  menuRenderPending = true;
  menuRenderPendingProfile = profile;
  menuRenderNotBeforeAt =
      millis() +
      delayMs;
}

static void requestMainMenuRender(
    uint8_t profile) {
  if (profile >= PROFILE_COUNT ||
      profile != activeProfile ||
      saverActive ||
      !displayReady) {
    return;
  }

  if (menuBatchActive &&
      profile == menuBatchProfile) {
    menuBatchDirty = true;
    menuBatchLastActivityAt =
        millis();
    return;
  }

  scheduleMainMenuRender(
      profile);
}

static void beginMainMenuBatch(
    uint8_t profile) {
  menuBatchActive = true;
  menuBatchDirty = false;
  menuBatchProfile =
      profile < PROFILE_COUNT
          ? profile
          : 0;
  menuBatchLastActivityAt =
      millis();
}

static void endMainMenuBatch(
    uint8_t profile) {
  if (!menuBatchActive) {
    scheduleMainMenuRender(
        profile);
    return;
  }

  const uint8_t batchProfile =
      menuBatchProfile;

  const bool render =
      menuBatchDirty &&
      batchProfile == profile;

  menuBatchActive = false;
  menuBatchDirty = false;
  menuBatchLastActivityAt = 0;

  if (render) {
    scheduleMainMenuRender(
        batchProfile);
  }
}

static void pollMainMenuBatchTimeout() {
  if (!menuBatchActive) {
    return;
  }

  const uint32_t now =
      millis();

  if (static_cast<uint32_t>(
          now -
          menuBatchLastActivityAt) <
      MENU_BATCH_TIMEOUT_MS) {
    return;
  }

  const uint8_t profile =
      menuBatchProfile;

  const bool render =
      menuBatchDirty;

  menuBatchActive = false;
  menuBatchDirty = false;
  menuBatchLastActivityAt = 0;

  if (render) {
    scheduleMainMenuRender(
        profile);
  }
}

static void pollDeferredMainMenuRender() {
  if (!menuRenderPending ||
      rawMediaKind != RAW_MEDIA_NONE ||
      menuBatchActive ||
      saverActive ||
      !displayReady) {
    return;
  }

  const uint32_t now =
      millis();

  if (static_cast<int32_t>(
          now -
          menuRenderNotBeforeAt) <
      0) {
    return;
  }

  if (USBSerial.available() > 0 ||
      cdcLine.length() > 0) {
    menuRenderNotBeforeAt =
        now + 25;
    return;
  }

  const uint8_t profile =
      menuRenderPendingProfile;

  menuRenderPending = false;
  menuRenderNotBeforeAt = 0;

  if (profile != activeProfile) {
    return;
  }

  renderMainMenu();

  // Give TinyUSB a scheduling point after the full 480x320 JPEG render.
  delay(0);
}

static void closeMenuUpload() {
  if (menuUploadFile) {
    menuUploadFile.close();
  }

  menuUploadKind = 0;
  menuUploadExpectedBytes = 0;
  menuUploadReceivedBytes = 0;
}

static bool replaceLittleFsFileAtomically(
    const char *tempPath,
    const char *finalPath,
    const char *backupPath) {
  if (!littleFsReady ||
      tempPath == nullptr ||
      finalPath == nullptr ||
      backupPath == nullptr ||
      !LittleFS.exists(tempPath)) {
    return false;
  }

  LittleFS.remove(backupPath);

  const bool hadFinal =
      LittleFS.exists(finalPath);

  if (hadFinal &&
      !LittleFS.rename(
          finalPath,
          backupPath)) {
    return false;
  }

  if (!LittleFS.rename(
          tempPath,
          finalPath)) {
    if (hadFinal) {
      (void)LittleFS.rename(
          backupPath,
          finalPath);
    }

    return false;
  }

  if (hadFinal) {
    LittleFS.remove(backupPath);
  }

  return true;
}

static void recoverMainMenuAssets() {
  if (!littleFsReady) {
    return;
  }

  for (uint8_t profile = 0;
       profile < PROFILE_COUNT;
       ++profile) {
    char bgFinal[24] = {};
    char bgTemp[24] = {};
    char bgBackup[24] = {};

    menuBackgroundPath(
        profile,
        false,
        bgFinal,
        sizeof(bgFinal));

    menuBackgroundPath(
        profile,
        true,
        bgTemp,
        sizeof(bgTemp));

    menuBackgroundBackupPath(
        profile,
        bgBackup,
        sizeof(bgBackup));

    if (!LittleFS.exists(bgFinal) &&
        LittleFS.exists(bgBackup)) {
      (void)LittleFS.rename(
          bgBackup,
          bgFinal);
    } else if (LittleFS.exists(bgFinal)) {
      LittleFS.remove(bgBackup);
    }

    // A .tmp is never authoritative. It belongs to an interrupted upload.
    LittleFS.remove(bgTemp);

    for (uint8_t slot = 0;
         slot < MENU_SLOT_COUNT;
         ++slot) {
      char iconFinal[24] = {};
      char iconTemp[24] = {};
      char iconBackup[24] = {};

      menuIconPath(
          profile,
          slot,
          false,
          iconFinal,
          sizeof(iconFinal));

      menuIconPath(
          profile,
          slot,
          true,
          iconTemp,
          sizeof(iconTemp));

      menuIconBackupPath(
          profile,
          slot,
          iconBackup,
          sizeof(iconBackup));

      if (!LittleFS.exists(iconFinal) &&
          LittleFS.exists(iconBackup)) {
        (void)LittleFS.rename(
            iconBackup,
            iconFinal);
      } else if (LittleFS.exists(iconFinal)) {
        LittleFS.remove(iconBackup);
      }

      LittleFS.remove(iconTemp);
    }
  }

  for (uint8_t profile = 0;
       profile < PROFILE_COUNT;
       ++profile) {
    char bgPath[24] = {};
    menuBackgroundPath(
        profile,
        false,
        bgPath,
        sizeof(bgPath));

    char flagPath[24] = {};
    menuCompositeFlagPath(
        profile,
        flagPath,
        sizeof(flagPath));

    if (!LittleFS.exists(
            bgPath)) {
      LittleFS.remove(
          flagPath);

      menuCompositeMask &=
          ~(1UL << profile);

      continue;
    }

    if ((menuCompositeMask &
         (1UL << profile)) != 0 &&
        !LittleFS.exists(
            flagPath)) {
      File flag =
          LittleFS.open(
              flagPath,
              "w");

      if (flag) {
        const uint8_t marker[4] = {
            'P',
            'X',
            'M',
            '1'};

        (void)flag.write(
            marker,
            sizeof(marker));

        flag.close();
      }
    }
  }

  (void)preferences.putUInt(
      "menucomp",
      menuCompositeMask);
}

static bool beginMenuBackgroundUpload(
    uint8_t profile,
    uint32_t expectedBytes) {
  if (!littleFsReady ||
      profile >= PROFILE_COUNT ||
      expectedBytes < 4 ||
      expectedBytes > MENU_BACKGROUND_LIMIT_BYTES) {
    return false;
  }

  closeMenuUpload();

  (void)setMainMenuCompositeEnabled(
      profile,
      false);

  char finalPath[24] = {};
  char tempPath[24] = {};
  char backupPath[24] = {};

  menuBackgroundPath(
      profile,
      false,
      finalPath,
      sizeof(finalPath));

  menuBackgroundPath(
      profile,
      true,
      tempPath,
      sizeof(tempPath));

  menuBackgroundBackupPath(
      profile,
      backupPath,
      sizeof(backupPath));

  // Recover any interrupted previous replacement before starting another one.
  if (!LittleFS.exists(finalPath) &&
      LittleFS.exists(backupPath)) {
    (void)LittleFS.rename(
        backupPath,
        finalPath);
  } else if (LittleFS.exists(finalPath)) {
    LittleFS.remove(backupPath);
  }

  // Never delete the current final asset at BEGIN. It remains the device-owned
  // fallback until the new upload has been fully written and validated.
  LittleFS.remove(tempPath);

  size_t total =
      LittleFS.totalBytes();

  size_t used =
      LittleFS.usedBytes();

  size_t freeBytes =
      total > used
          ? total - used
          : 0;

  if (expectedBytes + 4096 >
      freeBytes) {
    return false;
  }

  menuUploadFile =
      LittleFS.open(
          tempPath,
          "w");

  if (!menuUploadFile) {
    return false;
  }

  menuUploadKind = 1;
  menuUploadProfile = profile;
  menuUploadExpectedBytes = expectedBytes;
  menuUploadReceivedBytes = 0;

  return true;
}

static bool beginMenuIconUpload(
    uint8_t profile,
    uint8_t slot,
    uint32_t expectedBytes) {
  if (!littleFsReady ||
      profile >= PROFILE_COUNT ||
      slot >= MENU_SLOT_COUNT ||
      expectedBytes != MENU_ICON_ASSET_BYTES ||
      expectedBytes > MENU_ICON_MAX_BYTES) {
    return false;
  }

  closeMenuUpload();

  char finalPath[24] = {};
  char tempPath[24] = {};
  char backupPath[24] = {};

  menuIconPath(
      profile,
      slot,
      false,
      finalPath,
      sizeof(finalPath));

  menuIconPath(
      profile,
      slot,
      true,
      tempPath,
      sizeof(tempPath));

  menuIconBackupPath(
      profile,
      slot,
      backupPath,
      sizeof(backupPath));

  if (!LittleFS.exists(finalPath) &&
      LittleFS.exists(backupPath)) {
    (void)LittleFS.rename(
        backupPath,
        finalPath);
  } else if (LittleFS.exists(finalPath)) {
    LittleFS.remove(backupPath);
  }

  LittleFS.remove(tempPath);

  size_t total =
      LittleFS.totalBytes();

  size_t used =
      LittleFS.usedBytes();

  size_t freeBytes =
      total > used
          ? total - used
          : 0;

  if (expectedBytes + 1024 >
      freeBytes) {
    return false;
  }

  menuUploadFile =
      LittleFS.open(
          tempPath,
          "w");

  if (!menuUploadFile) {
    return false;
  }

  menuUploadKind = 2;
  menuUploadProfile = profile;
  menuUploadSlot = slot;
  menuUploadExpectedBytes = expectedBytes;
  menuUploadReceivedBytes = 0;

  return true;
}

static bool writeMenuAssetChunk(
    uint32_t offset,
    const String &encoded) {
  if (menuUploadKind == 0 ||
      !menuUploadFile) {
    return false;
  }

  uint8_t decoded[1100] = {};
  size_t decodedLength = 0;

  int result =
      mbedtls_base64_decode(
          decoded,
          sizeof(decoded),
          &decodedLength,
          reinterpret_cast<const unsigned char *>(encoded.c_str()),
          encoded.length());

  if (result != 0 ||
      decodedLength == 0) {
    return false;
  }

  // ACK-per-chunk transfers must tolerate an ACK being lost after the data
  // was already committed. Accept one exact replay without writing twice.
  if (offset < menuUploadReceivedBytes) {
    return offset + decodedLength ==
           menuUploadReceivedBytes;
  }

  if (offset != menuUploadReceivedBytes ||
      menuUploadReceivedBytes + decodedLength >
          menuUploadExpectedBytes) {
    return false;
  }

  if (menuUploadFile.write(
          decoded,
          decodedLength) !=
      decodedLength) {
    return false;
  }

  menuUploadReceivedBytes +=
      decodedLength;

  if (menuBatchActive) {
    menuBatchLastActivityAt =
        millis();
  }

  return true;
}

static bool finishMenuBackgroundUpload() {
  if (menuUploadKind != 1 ||
      !menuUploadFile ||
      menuUploadProfile >= PROFILE_COUNT ||
      menuUploadReceivedBytes !=
          menuUploadExpectedBytes) {
    closeMenuUpload();
    return false;
  }

  const uint8_t profile =
      menuUploadProfile;

  char finalPath[24] = {};
  char tempPath[24] = {};
  char backupPath[24] = {};

  menuBackgroundPath(
      profile,
      false,
      finalPath,
      sizeof(finalPath));

  menuBackgroundPath(
      profile,
      true,
      tempPath,
      sizeof(tempPath));

  menuBackgroundBackupPath(
      profile,
      backupPath,
      sizeof(backupPath));

  // Do not instantiate JPEGDEC or touch the TFT in the final raw-data
  // callback. That path runs inline with native USB receive and was the last
  // operation before the observed CDC disconnect around the final chunk.
  menuUploadFile.flush();
  menuUploadFile.close();

  File file =
      LittleFS.open(
          tempPath,
          "r");

  if (!file) {
    closeMenuUpload();
    return false;
  }

  const size_t fileSize =
      file.size();

  bool valid =
      fileSize ==
          menuUploadExpectedBytes &&
      fileSize >= 4;

  uint8_t first[2] = {};
  uint8_t last[2] = {};

  if (valid) {
    valid =
        file.read(
            first,
            sizeof(first)) ==
            sizeof(first) &&
        first[0] == 0xFF &&
        first[1] == 0xD8;
  }

  if (valid) {
    valid =
        file.seek(
            fileSize - 2) &&
        file.read(
            last,
            sizeof(last)) ==
            sizeof(last) &&
        last[0] == 0xFF &&
        last[1] == 0xD9;
  }

  file.close();

  if (!valid) {
    LittleFS.remove(
        tempPath);

    closeMenuUpload();
    return false;
  }

  const bool committed =
      replaceLittleFsFileAtomically(
          tempPath,
          finalPath,
          backupPath);

  if (!committed) {
    LittleFS.remove(
        tempPath);

    closeMenuUpload();
    return false;
  }

  // Only commit the file here. The app sends MENUMODE / MENUCFG / MENUSHOW
  // after MEDIA_RAW_DONE, so preferences and rendering happen outside the
  // time-critical USB finalizer.
  closeMenuUpload();
  return true;
}

static bool finishMenuIconUpload() {
  if (menuUploadKind != 2 ||
      !menuUploadFile ||
      menuUploadProfile >= PROFILE_COUNT ||
      menuUploadReceivedBytes !=
          menuUploadExpectedBytes) {
    closeMenuUpload();
    return false;
  }

  uint8_t profile =
      menuUploadProfile;

  uint8_t slot =
      menuUploadSlot;

  menuUploadFile.flush();
  menuUploadFile.close();

  char finalPath[24] = {};
  char tempPath[24] = {};
  char backupPath[24] = {};

  menuIconPath(
      profile,
      slot,
      false,
      finalPath,
      sizeof(finalPath));

  menuIconPath(
      profile,
      slot,
      true,
      tempPath,
      sizeof(tempPath));

  menuIconBackupPath(
      profile,
      slot,
      backupPath,
      sizeof(backupPath));

  File verify =
      LittleFS.open(
          tempPath,
          "r");

  bool valid = false;

  if (verify) {
    uint8_t header[8] = {};

    valid =
        verify.size() ==
            MENU_ICON_ASSET_BYTES &&
        verify.read(
            header,
            sizeof(header)) ==
            sizeof(header) &&
        header[0] == 'P' &&
        header[1] == 'X' &&
        header[2] == 'I' &&
        header[3] == '1';

    uint16_t width =
        static_cast<uint16_t>(
            header[4] |
            (static_cast<uint16_t>(
                 header[5]) <<
             8));

    uint16_t height =
        static_cast<uint16_t>(
            header[6] |
            (static_cast<uint16_t>(
                 header[7]) <<
             8));

    valid =
        valid &&
        width == MENU_ICON_WIDTH &&
        height == MENU_ICON_HEIGHT;

    verify.close();
  }

  if (!valid) {
    LittleFS.remove(tempPath);
    closeMenuUpload();
    return false;
  }

  bool committed =
      replaceLittleFsFileAtomically(
          tempPath,
          finalPath,
          backupPath);

  if (!committed) {
    LittleFS.remove(tempPath);
    closeMenuUpload();
    return false;
  }

  char legacyJpegPath[24] = {};

  menuLegacyIconJpegPath(
      profile,
      slot,
      legacyJpegPath,
      sizeof(legacyJpegPath));

  LittleFS.remove(
      legacyJpegPath);

  char legacyBinPath[24] = {};
  snprintf(
      legacyBinPath,
      sizeof(legacyBinPath),
      "/mi%u_%u.bin",
      static_cast<unsigned>(profile),
      static_cast<unsigned>(slot));

  LittleFS.remove(
      legacyBinPath);

  closeMenuUpload();

  rememberMainMenuContentProfile(
      profile);

  requestMainMenuRender(
      profile);

  return true;
}

static void clearMainMenuBackground(
    uint8_t profile) {
  if (!littleFsReady ||
      profile >= PROFILE_COUNT) {
    return;
  }

  char finalPath[24] = {};
  char tempPath[24] = {};
  char backupPath[24] = {};

  menuBackgroundPath(
      profile,
      false,
      finalPath,
      sizeof(finalPath));

  menuBackgroundPath(
      profile,
      true,
      tempPath,
      sizeof(tempPath));

  menuBackgroundBackupPath(
      profile,
      backupPath,
      sizeof(backupPath));

  LittleFS.remove(tempPath);
  LittleFS.remove(backupPath);
  LittleFS.remove(finalPath);

  (void)setMainMenuCompositeEnabled(
      profile,
      false);
}

static void clearMainMenuIcon(
    uint8_t profile,
    uint8_t slot) {
  if (!littleFsReady ||
      profile >= PROFILE_COUNT ||
      slot >= MENU_SLOT_COUNT) {
    return;
  }

  char finalPath[24] = {};
  char tempPath[24] = {};
  char backupPath[24] = {};

  menuIconPath(
      profile,
      slot,
      false,
      finalPath,
      sizeof(finalPath));

  menuIconPath(
      profile,
      slot,
      true,
      tempPath,
      sizeof(tempPath));

  menuIconBackupPath(
      profile,
      slot,
      backupPath,
      sizeof(backupPath));

  char legacyBinPath[24] = {};
  snprintf(
      legacyBinPath,
      sizeof(legacyBinPath),
      "/mi%u_%u.bin",
      static_cast<unsigned>(profile),
      static_cast<unsigned>(slot));

  char legacyJpegPath[24] = {};
  menuLegacyIconJpegPath(
      profile,
      slot,
      legacyJpegPath,
      sizeof(legacyJpegPath));

  LittleFS.remove(tempPath);
  LittleFS.remove(backupPath);
  LittleFS.remove(finalPath);
  LittleFS.remove(legacyBinPath);
  LittleFS.remove(legacyJpegPath);
}

static KeyBinding resolveBinding(uint8_t layer, uint8_t keyIndex) {
  int8_t scan = static_cast<int8_t>(layer);

  while (scan >= 0) {
    const KeyBinding &binding = keymap[activeProfile][scan][keyIndex];

    if (binding.type != BIND_TRANSPARENT) {
      return binding;
    }

    --scan;
  }

  return disabledBinding();
}

static String serializeBinding(const KeyBinding &binding) {
  char out[28];

  switch (binding.type) {
    case BIND_DISABLED:
      return String("D:0:0");

    case BIND_TRANSPARENT:
      return String("T:0:0");

    case BIND_CONSUMER:
      snprintf(
          out,
          sizeof(out),
          "C:%u:0",
          static_cast<unsigned>(binding.consumerCode));
      return String(out);

    case BIND_LAYER:
      snprintf(
          out,
          sizeof(out),
          "L:%u:%u",
          static_cast<unsigned>(binding.keyCode),
          static_cast<unsigned>(binding.modifiers));
      return String(out);

    case BIND_MACRO:
      snprintf(
          out,
          sizeof(out),
          "M:%u:0",
          static_cast<unsigned>(binding.keyCode));
      return String(out);

    case BIND_ACTION:
      snprintf(
          out,
          sizeof(out),
          "A:%u:0",
          static_cast<unsigned>(binding.keyCode));
      return String(out);

    case BIND_KEYBOARD:
    default:
      snprintf(
          out,
          sizeof(out),
          "K:%u:%u",
          static_cast<unsigned>(binding.keyCode),
          static_cast<unsigned>(binding.modifiers));
      return String(out);
  }
}

static String serializeKeymap(uint8_t profile, uint8_t layer) {
  String out = "KEYMAP|";
  out += String(profile);
  out += '|';
  out += String(layer);
  out += '|';

  for (uint8_t i = 0; i < KEY_COUNT; ++i) {
    if (i) {
      out += ',';
    }

    out += serializeBinding(keymap[profile][layer][i]);
  }

  return out;
}

static bool parseUnsigned(
    const String &text,
    uint16_t maxValue,
    uint16_t &value) {
  if (text.length() == 0) {
    return false;
  }

  for (size_t i = 0; i < text.length(); ++i) {
    if (!isDigit(text[i])) {
      return false;
    }
  }

  unsigned long parsed = text.toInt();
  if (parsed > maxValue) {
    return false;
  }

  value = static_cast<uint16_t>(parsed);
  return true;
}

static bool parseSigned(
    const String &text,
    int16_t minValue,
    int16_t maxValue,
    int16_t &value) {
  if (text.length() == 0) {
    return false;
  }

  size_t start = 0;
  if (text[0] == '-') {
    start = 1;
  }

  if (start >= text.length()) {
    return false;
  }

  for (size_t i = start; i < text.length(); ++i) {
    if (!isDigit(text[i])) {
      return false;
    }
  }

  long parsed =
      strtol(
          text.c_str(),
          nullptr,
          10);

  if (parsed < minValue ||
      parsed > maxValue) {
    return false;
  }

  value =
      static_cast<int16_t>(
          parsed);

  return true;
}

static bool parseUnsignedLong(
    const String &text,
    uint32_t maxValue,
    uint32_t &value) {
  if (text.length() == 0) {
    return false;
  }

  for (size_t i = 0; i < text.length(); ++i) {
    if (!isDigit(text[i])) {
      return false;
    }
  }

  unsigned long parsed = strtoul(text.c_str(), nullptr, 10);

  if (parsed > maxValue) {
    return false;
  }

  value = static_cast<uint32_t>(parsed);
  return true;
}


static bool parseHexU32(
    const String &text,
    uint32_t &value) {
  if (text.length() != 8) {
    return false;
  }

  uint32_t parsed = 0;

  for (size_t i = 0;
       i < text.length();
       ++i) {
    char ch = text[i];
    uint8_t nibble = 0;

    if (ch >= '0' &&
        ch <= '9') {
      nibble =
          static_cast<uint8_t>(
              ch - '0');
    } else if (ch >= 'A' &&
               ch <= 'F') {
      nibble =
          static_cast<uint8_t>(
              ch - 'A' + 10);
    } else if (ch >= 'a' &&
               ch <= 'f') {
      nibble =
          static_cast<uint8_t>(
              ch - 'a' + 10);
    } else {
      return false;
    }

    parsed =
        (parsed << 4) |
        nibble;
  }

  value = parsed;
  return true;
}

static uint32_t rawCrc32Update(
    uint32_t crc,
    const uint8_t *data,
    size_t length) {
  if (data == nullptr) {
    return crc;
  }

  for (size_t i = 0;
       i < length;
       ++i) {
    crc ^= data[i];

    for (uint8_t bit = 0;
         bit < 8;
         ++bit) {
      crc =
          (crc & 1U) != 0
              ? (crc >> 1) ^
                    0xEDB88320UL
              : crc >> 1;
    }
  }

  return crc;
}

static bool parseBindingToken(String token, KeyBinding &binding) {
  token.trim();
  token.toUpperCase();

  int first = token.indexOf(':');
  int second = first >= 0 ? token.indexOf(':', first + 1) : -1;

  if (first != 1 || second < 0) {
    return false;
  }

  char type = token[0];
  String codePart = token.substring(first + 1, second);
  String modsPart = token.substring(second + 1);

  uint16_t code = 0;
  uint16_t mods = 0;

  if (!parseUnsigned(codePart, 0xFFFF, code) ||
      !parseUnsigned(modsPart, 0xFF, mods)) {
    return false;
  }

  binding = {};

  if (type == 'D') {
    binding.type = BIND_DISABLED;
    return code == 0 && mods == 0;
  }

  if (type == 'T') {
    binding.type = BIND_TRANSPARENT;
    return code == 0 && mods == 0;
  }

  if (type == 'K') {
    if ((code == 0 && mods == 0) || code > 0xFF || mods > 0x0F) {
      return false;
    }

    binding.type = BIND_KEYBOARD;
    binding.keyCode = static_cast<uint8_t>(code);
    binding.modifiers = static_cast<uint8_t>(mods);
    return true;
  }

  if (type == 'C') {
    if (code == 0 || mods != 0) {
      return false;
    }

    binding.type = BIND_CONSUMER;
    binding.consumerCode = code;
    return true;
  }

  if (type == 'L') {
    if (code >= LAYER_COUNT ||
        (mods != LAYER_MO && mods != LAYER_TG && mods != LAYER_TO)) {
      return false;
    }

    binding.type = BIND_LAYER;
    binding.keyCode = static_cast<uint8_t>(code);
    binding.modifiers = static_cast<uint8_t>(mods);
    return true;
  }

  if (type == 'M') {
    if (code >= MACRO_COUNT || mods != 0) {
      return false;
    }

    binding.type = BIND_MACRO;
    binding.keyCode = static_cast<uint8_t>(code);
    return true;
  }

  if (type == 'A') {
    if (code < 1 || code > ACTION_COUNT || mods != 0) {
      return false;
    }

    binding.type = BIND_ACTION;
    binding.keyCode = static_cast<uint8_t>(code);
    return true;
  }

  return false;
}

static bool parseKeymapPayload(
    const String &payload,
    KeyBinding (&parsed)[KEY_COUNT]) {
  int start = 0;

  for (uint8_t i = 0; i < KEY_COUNT; ++i) {
    int comma = payload.indexOf(',', start);
    bool last = i == KEY_COUNT - 1;

    if ((!last && comma < 0) || (last && comma >= 0)) {
      return false;
    }

    String token =
        last ? payload.substring(start) : payload.substring(start, comma);

    if (!parseBindingToken(token, parsed[i])) {
      return false;
    }

    start = comma + 1;
  }

  return true;
}

static String hexEncode(const String &input) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  String out;
  out.reserve(input.length() * 2);

  for (size_t i = 0; i < input.length(); ++i) {
    uint8_t value = static_cast<uint8_t>(input[i]);
    out += HEX_DIGITS[value >> 4];
    out += HEX_DIGITS[value & 0x0F];
  }

  return out;
}

static int8_t hexNibble(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  return -1;
}

static bool hexDecode(const String &hex, String &out) {
  if ((hex.length() % 2) != 0 ||
      hex.length() > static_cast<size_t>(MACRO_MAX_LEN * 2)) {
    return false;
  }

  out = "";
  out.reserve(hex.length() / 2);

  for (size_t i = 0; i < hex.length(); i += 2) {
    int8_t hi = hexNibble(hex[i]);
    int8_t lo = hexNibble(hex[i + 1]);

    if (hi < 0 || lo < 0) {
      return false;
    }

    char value = static_cast<char>((hi << 4) | lo);

    if (value < 0x20 || value > 0x7E) {
      return false;
    }

    out += value;
  }

  return true;
}



static void *gifAlloc(uint32_t size) {
  void *ptr = nullptr;

  if (ESP.getPsramSize() > 0) {
    ptr = ps_malloc(size);
  }

  if (ptr == nullptr) {
    ptr = malloc(size);
  }

  return ptr;
}

static void gifFree(void *ptr) {
  if (ptr != nullptr) {
    free(ptr);
  }
}

static void *gifOpenFile(
    const char *filename,
    int32_t *fileSize) {
  (void)filename;

  gifPlaybackFile = LittleFS.open(GIF_PATH, "r");

  if (!gifPlaybackFile) {
    return nullptr;
  }

  *fileSize =
      static_cast<int32_t>(gifPlaybackFile.size());

  return &gifPlaybackFile;
}

static void gifCloseFile(void *handle) {
  File *file = static_cast<File *>(handle);

  if (file != nullptr) {
    file->close();
  }
}

static int32_t gifReadFile(
    GIFFILE *file,
    uint8_t *buffer,
    int32_t length) {
  if (file == nullptr ||
      file->fHandle == nullptr ||
      buffer == nullptr ||
      length <= 0) {
    return 0;
  }

  File *source =
      static_cast<File *>(file->fHandle);

  int32_t remaining =
      file->iSize - file->iPos;

  if (remaining <= 0) {
    return 0;
  }

  int32_t toRead =
      length < remaining ? length : remaining;

  int32_t read =
      static_cast<int32_t>(
          source->read(
              buffer,
              static_cast<size_t>(toRead)));

  file->iPos =
      static_cast<int32_t>(
          source->position());

  return read;
}

static int32_t gifSeekFile(
    GIFFILE *file,
    int32_t position) {
  if (file == nullptr ||
      file->fHandle == nullptr ||
      position < 0) {
    return -1;
  }

  File *source =
      static_cast<File *>(file->fHandle);

  if (!source->seek(
          static_cast<uint32_t>(position))) {
    return -1;
  }

  file->iPos =
      static_cast<int32_t>(
          source->position());

  return file->iPos;
}

static GifScaleMode parseGifScaleMode(String value) {
  value.trim();
  value.toUpperCase();

  if (value == "FIT") return GIF_SCALE_FIT;
  if (value == "STRETCH") return GIF_SCALE_STRETCH;
  if (value == "TILE") return GIF_SCALE_TILE;
  if (value == "CENTER") return GIF_SCALE_CENTER;
  if (value == "SPAN") return GIF_SCALE_SPAN;
  return GIF_SCALE_FILL;
}

static void configureGifTransform() {
  float sourceW =
      static_cast<float>(gifCanvasWidth);

  float sourceH =
      static_cast<float>(gifCanvasHeight);

  float fit =
      min(
          TFT_WIDTH / sourceW,
          TFT_HEIGHT / sourceH);

  float fill =
      max(
          TFT_WIDTH / sourceW,
          TFT_HEIGHT / sourceH);

  gifScaleX = 1.0f;
  gifScaleY = 1.0f;

  switch (gifScaleMode) {
    case GIF_SCALE_FIT:
      gifScaleX = gifScaleY = fit;
      break;

    case GIF_SCALE_STRETCH:
      gifScaleX = TFT_WIDTH / sourceW;
      gifScaleY = TFT_HEIGHT / sourceH;
      break;

    case GIF_SCALE_CENTER:
      // Original-size mode: never upscale small GIFs. Only shrink if the
      // source is larger than the 480x320 panel, preserving aspect ratio.
      gifScaleX = gifScaleY = min(1.0f, fit);
      break;

    case GIF_SCALE_SPAN:
      gifScaleX = gifScaleY = fill * 1.08f;
      break;

    case GIF_SCALE_TILE:
      // Animated GIF tiling is intentionally lightweight: use Fit instead
      // of expanding the compressed file into several raw copies.
      gifScaleX = gifScaleY = fit;
      break;

    case GIF_SCALE_FILL:
    default:
      gifScaleX = gifScaleY = fill;
      break;
  }

  float drawW = sourceW * gifScaleX;
  float drawH = sourceH * gifScaleY;

  gifOffsetX =
      (TFT_WIDTH - drawW) * 0.5f;

  gifOffsetY =
      (TFT_HEIGHT - drawH) * 0.5f;
}

static void gifDraw(GIFDRAW *draw) {
  if (draw == nullptr ||
      renderBuffer == nullptr ||
      draw->pPixels == nullptr) {
    return;
  }

  const uint16_t *pixels =
      reinterpret_cast<const uint16_t *>(
          draw->pPixels);

  int sourceY =
      draw->iY + draw->y;

  int sourceX =
      draw->iX;

  int width = draw->iWidth;

  for (int x = 0; x < width; ++x) {
    int sx = sourceX + x;
    int sy = sourceY;

    if (sx < 0 ||
        sy < 0 ||
        sx >= gifCanvasWidth ||
        sy >= gifCanvasHeight) {
      continue;
    }

    float rx = static_cast<float>(sx);
    float ry = static_cast<float>(sy);

    int dx0 =
        static_cast<int>(
            floorf(
                gifOffsetX +
                rx * gifScaleX));

    int dx1 =
        static_cast<int>(
            ceilf(
                gifOffsetX +
                (rx + 1.0f) * gifScaleX)) - 1;

    int dy0 =
        static_cast<int>(
            floorf(
                gifOffsetY +
                ry * gifScaleY));

    int dy1 =
        static_cast<int>(
            ceilf(
                gifOffsetY +
                (ry + 1.0f) * gifScaleY)) - 1;

    if (dx1 < 0 ||
        dy1 < 0 ||
        dx0 >= TFT_WIDTH ||
        dy0 >= TFT_HEIGHT) {
      continue;
    }

    dx0 = dx0 < 0 ? 0 : dx0;
    dy0 = dy0 < 0 ? 0 : dy0;
    dx1 =
        dx1 >= TFT_WIDTH
            ? TFT_WIDTH - 1
            : dx1;
    dy1 =
        dy1 >= TFT_HEIGHT
            ? TFT_HEIGHT - 1
            : dy1;

    uint16_t color =
        vividRgb565(
            pixels[x]);

    for (int dy = dy0; dy <= dy1; ++dy) {
      uint16_t *row =
          renderBuffer +
          static_cast<size_t>(dy) * TFT_WIDTH;

      for (int dx = dx0; dx <= dx1; ++dx) {
        row[dx] = color;
      }
    }
  }
}

static void closeGifDecoder() {
  if (gifDecoder.getFrameBuf() != nullptr) {
    gifDecoder.freeFrameBuf(gifFree);
  }

  if (gifDecoderOpen) {
    gifDecoder.close();
  }

  if (gifPlaybackFile) {
    gifPlaybackFile.close();
  }

  gifDecoderOpen = false;
  gifAtEnd = false;
  gifCanvasWidth = 0;
  gifCanvasHeight = 0;
  gifNextFrameAt = 0;
}

static bool gifDimensionsSupported(
    uint16_t width,
    uint16_t height) {
  return
      width >= 1 &&
      height >= 1 &&
      width <= 1024 &&
      height <= 1024;
}

static bool readGifHeader(
    const char *path,
    uint16_t &width,
    uint16_t &height,
    size_t &fileSize) {
  if (!littleFsReady) {
    return false;
  }

  File file = LittleFS.open(path, "r");

  if (!file) {
    return false;
  }

  fileSize = file.size();

  uint8_t header[10] = {};

  size_t read =
      file.read(
          header,
          sizeof(header));

  file.close();

  if (read != sizeof(header) ||
      header[0] != 'G' ||
      header[1] != 'I' ||
      header[2] != 'F') {
    return false;
  }

  width =
      static_cast<uint16_t>(
          header[6] |
          (header[7] << 8));

  height =
      static_cast<uint16_t>(
          header[8] |
          (header[9] << 8));

  return gifDimensionsSupported(
      width,
      height);
}

static bool openGifDecoder() {
  closeGifDecoder();

  if (!littleFsReady ||
      !LittleFS.exists(GIF_PATH) ||
      renderBuffer == nullptr) {
    return false;
  }

  gifDecoder.begin(
      GIF_PALETTE_RGB565_LE);

  if (!gifDecoder.open(
          GIF_PATH,
          gifOpenFile,
          gifCloseFile,
          gifReadFile,
          gifSeekFile,
          gifDraw)) {
    closeGifDecoder();
    return false;
  }

  gifCanvasWidth =
      static_cast<uint16_t>(
          gifDecoder.getCanvasWidth());

  gifCanvasHeight =
      static_cast<uint16_t>(
          gifDecoder.getCanvasHeight());

  if (!gifDimensionsSupported(
          gifCanvasWidth,
          gifCanvasHeight)) {
    closeGifDecoder();
    return false;
  }

  if (gifDecoder.allocFrameBuf(gifAlloc) !=
      GIF_SUCCESS) {
    closeGifDecoder();
    return false;
  }

  if (gifDecoder.setDrawType(
          GIF_DRAW_COOKED) != GIF_SUCCESS) {
    closeGifDecoder();
    return false;
  }

  configureGifTransform();

  memset(
      renderBuffer,
      0,
      static_cast<size_t>(TFT_WIDTH) *
          TFT_HEIGHT *
          sizeof(uint16_t));

  gifDecoderOpen = true;
  gifAtEnd = false;
  gifNextFrameAt = 0;

  return true;
}

static bool decodeNextGifFrame() {
  if (!gifDecoderOpen ||
      renderBuffer == nullptr ||
      !displayReady) {
    return false;
  }

  if (gifAtEnd) {
    gifDecoder.reset();
    gifAtEnd = false;

    memset(
        renderBuffer,
        0,
        static_cast<size_t>(TFT_WIDTH) *
            TFT_HEIGHT *
            sizeof(uint16_t));
  }

  const uint32_t frameStartedAt =
      millis();

  int delayMs = 0;

  int hasMore =
      gifDecoder.playFrame(
          false,
          &delayMs,
          nullptr);

  tft->draw16bitRGBBitmap(
      0,
      0,
      renderBuffer,
      TFT_WIDTH,
      TFT_HEIGHT);

  gifAtEnd = hasMore == 0;

  uint32_t holdMs =
      delayMs < GIF_MIN_FRAME_MS
          ? GIF_MIN_FRAME_MS
          : static_cast<uint32_t>(delayMs);

  gifNextFrameAt =
      frameStartedAt + holdMs;

  return true;
}

static void closeGifUploadFile() {
  if (gifUploadFile) {
    gifUploadFile.close();
  }
}

static bool beginGifUpload(
    uint32_t expectedBytes,
    uint16_t width,
    uint16_t height,
    GifScaleMode scaleMode) {
  if (!littleFsReady ||
      expectedBytes < 10 ||
      !gifDimensionsSupported(
          width,
          height)) {
    return false;
  }

  // Drop any previous raw/GIF/JPEG media before allocating the new
  // compressed upload. This also frees legacy PSRAM frame buffers.
  clearSaverBuffer();

  size_t total =
      LittleFS.totalBytes();

  size_t used =
      LittleFS.usedBytes();

  size_t freeBytes =
      total > used
          ? total - used
          : 0;

  if (expectedBytes + 4096 > freeBytes) {
    return false;
  }

  gifUploadFile =
      LittleFS.open(
          GIF_TMP_PATH,
          "w");

  if (!gifUploadFile) {
    return false;
  }

  gifUploadExpectedBytes =
      expectedBytes;

  gifUploadWidth = width;
  gifUploadHeight = height;

  // PIXEL PRO no longer accepts host-side Fill/Fit for uploaded GIFs.
  // Small GIFs stay pixel-sized; only oversized canvases are reduced to fit.
  (void)scaleMode;
  gifUploadScaleMode = GIF_SCALE_CENTER;

  saverBytesReceived = 0;
  saverDataBytes = expectedBytes;
  saverWidth = width;
  saverHeight = height;
  saverFormat = SAVER_GIF;
  saverUploading = true;
  saverReady = false;
  saverActive = false;

  return true;
}

static bool writeGifUploadChunk(
    uint32_t offset,
    const String &encoded) {
  if (!saverUploading ||
      saverFormat != SAVER_GIF ||
      !gifUploadFile) {
    return false;
  }

  uint8_t decoded[1100] = {};
  size_t decodedLength = 0;

  int result =
      mbedtls_base64_decode(
          decoded,
          sizeof(decoded),
          &decodedLength,
          reinterpret_cast<
              const unsigned char *>(
              encoded.c_str()),
          encoded.length());

  if (result != 0 ||
      decodedLength == 0) {
    return false;
  }

  if (offset < saverBytesReceived) {
    return offset + decodedLength ==
           saverBytesReceived;
  }

  if (offset != saverBytesReceived ||
      saverBytesReceived +
              decodedLength >
          gifUploadExpectedBytes) {
    return false;
  }

  size_t written =
      gifUploadFile.write(
          decoded,
          decodedLength);

  if (written != decodedLength) {
    return false;
  }

  saverBytesReceived +=
      decodedLength;

  return true;
}

static bool finishGifUpload() {
  if (!saverUploading ||
      saverFormat != SAVER_GIF ||
      saverBytesReceived !=
          gifUploadExpectedBytes) {
    closeGifUploadFile();
    return false;
  }

  gifUploadFile.flush();
  closeGifUploadFile();

  uint16_t actualWidth = 0;
  uint16_t actualHeight = 0;
  size_t actualSize = 0;

  if (!readGifHeader(
          GIF_TMP_PATH,
          actualWidth,
          actualHeight,
          actualSize) ||
      actualSize !=
          gifUploadExpectedBytes ||
      actualWidth != gifUploadWidth ||
      actualHeight != gifUploadHeight) {
    LittleFS.remove(GIF_TMP_PATH);
    saverUploading = false;
    saverReady = false;
    return false;
  }

  LittleFS.remove(GIF_PATH);

  if (!LittleFS.rename(
          GIF_TMP_PATH,
          GIF_PATH)) {
    LittleFS.remove(GIF_TMP_PATH);
    saverUploading = false;
    saverReady = false;
    return false;
  }

  LittleFS.remove(
      PACKED_PATH);
  LittleFS.remove(
      PACKED_TMP_PATH);

  saverUploading = false;
  saverReady = true;
  saverActive = false;
  saverFormat = SAVER_GIF;
  saverWidth = actualWidth;
  saverHeight = actualHeight;
  saverDataBytes = actualSize;
  saverBytesReceived = actualSize;
  gifScaleMode = gifUploadScaleMode;
  preferences.putUChar(
      "gscale",
      static_cast<uint8_t>(gifScaleMode));
  preferences.putUChar(
      "gscalev",
      2);

  clearSaverIdentity();

  return true;
}

static void loadPersistedGif() {
  if (!littleFsReady ||
      !LittleFS.exists(GIF_PATH)) {
    return;
  }

  uint16_t width = 0;
  uint16_t height = 0;
  size_t fileSize = 0;

  if (!readGifHeader(
          GIF_PATH,
          width,
          height,
          fileSize)) {
    LittleFS.remove(GIF_PATH);
    return;
  }

  saverFormat = SAVER_GIF;

  // Always migrate persisted media to Center/no-upscale. Older builds could
  // leave Fill/Fit in NVS and make a small GIF look huge after reboot.
  gifScaleMode = GIF_SCALE_CENTER;
  preferences.putUChar(
      "gscale",
      static_cast<uint8_t>(
          GIF_SCALE_CENTER));
  preferences.putUChar(
      "gscalev",
      3);

  saverWidth = width;
  saverHeight = height;
  saverDataBytes = fileSize;
  saverBytesReceived = fileSize;
  saverUploading = false;
  saverReady = true;
  saverActive = false;
}

static void closePackedFiles() {
  if (packedUploadFile) {
    packedUploadFile.close();
  }

  if (packedPlaybackFile) {
    packedPlaybackFile.close();
  }
}

static void closeSaverThumbUpload() {
  if (saverThumbUploadFile) {
    saverThumbUploadFile.close();
  }
}

static void clearSaverIdentity() {
  closeSaverThumbUpload();

  if (littleFsReady) {
    LittleFS.remove(
        SAVER_THUMB_TMP_PATH);

    LittleFS.remove(
        SAVER_THUMB_PATH);
  }

  preferences.remove(
      "sav_name");

  preferences.remove(
      "sav_kind");

  saverThumbExpectedBytes = 0;
  saverThumbReceivedBytes = 0;
}

static bool beginSaverThumbUpload(
    uint32_t byteCount) {
  if (!littleFsReady ||
      byteCount == 0 ||
      byteCount >
          SAVER_THUMB_LIMIT_BYTES) {
    return false;
  }

  closeSaverThumbUpload();

  LittleFS.remove(
      SAVER_THUMB_TMP_PATH);

  saverThumbUploadFile =
      LittleFS.open(
          SAVER_THUMB_TMP_PATH,
          "w");

  if (!saverThumbUploadFile) {
    return false;
  }

  saverThumbExpectedBytes =
      byteCount;

  saverThumbReceivedBytes =
      0;

  return true;
}

static bool writeSaverThumbChunk(
    uint32_t offset,
    const String &encoded) {
  if (!saverThumbUploadFile) {
    return false;
  }

  uint8_t decoded[1100] = {};
  size_t decodedLength = 0;

  int result =
      mbedtls_base64_decode(
          decoded,
          sizeof(decoded),
          &decodedLength,
          reinterpret_cast<
              const unsigned char *>(
              encoded.c_str()),
          encoded.length());

  if (result != 0 ||
      decodedLength == 0) {
    return false;
  }

  if (offset < saverThumbReceivedBytes) {
    return offset + decodedLength ==
           saverThumbReceivedBytes;
  }

  if (offset != saverThumbReceivedBytes ||
      saverThumbReceivedBytes +
              decodedLength >
          saverThumbExpectedBytes) {
    return false;
  }

  size_t written =
      saverThumbUploadFile.write(
          decoded,
          decodedLength);

  if (written != decodedLength) {
    return false;
  }

  saverThumbReceivedBytes +=
      decodedLength;

  return true;
}

static bool finishSaverThumbUpload() {
  if (!saverThumbUploadFile ||
      saverThumbReceivedBytes !=
          saverThumbExpectedBytes) {
    closeSaverThumbUpload();
    return false;
  }

  saverThumbUploadFile.flush();
  closeSaverThumbUpload();

  File file =
      LittleFS.open(
          SAVER_THUMB_TMP_PATH,
          "r");

  size_t actual =
      file
          ? file.size()
          : 0;

  if (file) {
    file.close();
  }

  if (actual !=
      saverThumbExpectedBytes) {
    LittleFS.remove(
        SAVER_THUMB_TMP_PATH);
    return false;
  }

  LittleFS.remove(
      SAVER_THUMB_PATH);

  if (!LittleFS.rename(
          SAVER_THUMB_TMP_PATH,
          SAVER_THUMB_PATH)) {
    LittleFS.remove(
        SAVER_THUMB_TMP_PATH);
    return false;
  }

  return true;
}

static bool storeSaverMetadata(
    const String &kind,
    const String &encodedName) {
  uint8_t decoded[128] = {};
  size_t decodedLength = 0;

  int result =
      mbedtls_base64_decode(
          decoded,
          sizeof(decoded) - 1,
          &decodedLength,
          reinterpret_cast<
              const unsigned char *>(
              encodedName.c_str()),
          encodedName.length());

  if (result != 0 ||
      decodedLength == 0) {
    return false;
  }

  decoded[decodedLength] = 0;

  preferences.putString(
      "sav_name",
      reinterpret_cast<
          const char *>(decoded));

  preferences.putString(
      "sav_kind",
      kind);

  return true;
}

static bool readPackedU16(
    File &file,
    uint16_t &value) {
  uint8_t bytes[2] = {};

  if (file.read(
          bytes,
          sizeof(bytes)) !=
      sizeof(bytes)) {
    return false;
  }

  value =
      static_cast<uint16_t>(
          bytes[0]) |
      (static_cast<uint16_t>(
           bytes[1]) <<
       8);

  return true;
}

static bool readPackedU32(
    File &file,
    uint32_t &value) {
  uint8_t bytes[4] = {};

  if (file.read(
          bytes,
          sizeof(bytes)) !=
      sizeof(bytes)) {
    return false;
  }

  value =
      static_cast<uint32_t>(
          bytes[0]) |
      (static_cast<uint32_t>(
           bytes[1]) <<
       8) |
      (static_cast<uint32_t>(
           bytes[2]) <<
       16) |
      (static_cast<uint32_t>(
           bytes[3]) <<
       24);

  return true;
}

static uint16_t rgb888To565(
    uint8_t r,
    uint8_t g,
    uint8_t b) {
  return static_cast<uint16_t>(
      ((r & 0xF8) << 8) |
      ((g & 0xFC) << 3) |
      (b >> 3));
}

static uint16_t expectedPackedPaletteCount(
    uint8_t mode) {
  switch (mode) {
    case 2:
      return 256;
    case 3:
      return 16;
    case 4:
      return 4;
    case 5:
      return 2;
    default:
      return 0;
  }
}

static bool readPackedHeader(
    File &file,
    bool loadPalette) {
  if (!file ||
      !file.seek(0)) {
    return false;
  }

  uint8_t magic[4] = {};

  if (file.read(
          magic,
          sizeof(magic)) !=
      sizeof(magic) ||
      magic[0] != 'P' ||
      magic[1] != 'X' ||
      magic[2] != 'Q' ||
      magic[3] != '1') {
    return false;
  }

  int modeRead =
      file.read();

  int flagsRead =
      file.read();

  if (modeRead < 0 ||
      flagsRead < 0) {
    return false;
  }

  uint8_t mode =
      static_cast<uint8_t>(
          modeRead);

  uint8_t flags =
      static_cast<uint8_t>(
          flagsRead);

  uint16_t storageWidth = 0;
  uint16_t storageHeight = 0;
  uint16_t displayWidth = 0;
  uint16_t displayHeight = 0;
  uint16_t frameCount = 0;
  uint16_t fps = 0;
  uint32_t durationMs = 0;
  uint16_t paletteCount = 0;
  uint16_t reserved = 0;

  if (!readPackedU16(
          file,
          storageWidth) ||
      !readPackedU16(
          file,
          storageHeight) ||
      !readPackedU16(
          file,
          displayWidth) ||
      !readPackedU16(
          file,
          displayHeight) ||
      !readPackedU16(
          file,
          frameCount) ||
      !readPackedU16(
          file,
          fps) ||
      !readPackedU32(
          file,
          durationMs) ||
      !readPackedU16(
          file,
          paletteCount) ||
      !readPackedU16(
          file,
          reserved)) {
    return false;
  }

  (void)reserved;

  bool supportedStorage =
      (storageWidth == 480 &&
       storageHeight == 320) ||
      (storageWidth == 360 &&
       storageHeight == 240) ||
      (storageWidth == 240 &&
       storageHeight == 160);

  if (mode > 5 ||
      (flags & 0x03) != 0x03 ||
      !supportedStorage ||
      displayWidth != TFT_WIDTH ||
      displayHeight != TFT_HEIGHT ||
      frameCount == 0 ||
      fps < 15 ||
      fps > 60 ||
      durationMs == 0 ||
      paletteCount !=
          expectedPackedPaletteCount(
              mode)) {
    return false;
  }

  if (file.size() == 0 ||
      file.size() >
          PACKED_UPLOAD_LIMIT_BYTES) {
    return false;
  }

  memset(
      packedPalette565,
      0,
      sizeof(packedPalette565));

  for (uint16_t index = 0;
       index < paletteCount;
       ++index) {
    int r = file.read();
    int g = file.read();
    int b = file.read();

    if (r < 0 ||
        g < 0 ||
        b < 0) {
      return false;
    }

    if (loadPalette) {
      packedPalette565[index] =
          rgb888To565(
              static_cast<uint8_t>(r),
              static_cast<uint8_t>(g),
              static_cast<uint8_t>(b));
    }
  }

  packedColorMode =
      mode;

  packedStorageWidth =
      storageWidth;

  packedStorageHeight =
      storageHeight;

  packedFrameCount =
      frameCount;

  packedFps =
      fps;

  packedDurationMs =
      durationMs;

  packedPaletteCount =
      paletteCount;

  packedFramesOffset =
      static_cast<uint32_t>(
          file.position());

  return true;
}

static bool inspectPackedFile(
    const char *path,
    size_t &fileSize) {
  if (!littleFsReady ||
      !LittleFS.exists(
          path)) {
    return false;
  }

  File file =
      LittleFS.open(
          path,
          "r");

  if (!file) {
    return false;
  }

  fileSize =
      file.size();

  bool ok =
      readPackedHeader(
          file,
          false);

  file.close();

  return ok;
}

static bool beginPackedUpload(
    uint32_t expectedBytes) {
  if (!littleFsReady ||
      expectedBytes < 26 ||
      expectedBytes >
          PACKED_UPLOAD_LIMIT_BYTES) {
    return false;
  }

  clearSaverBuffer();

  size_t total =
      LittleFS.totalBytes();

  size_t used =
      LittleFS.usedBytes();

  size_t freeBytes =
      total > used
          ? total - used
          : 0;

  if (expectedBytes + 4096 >
      freeBytes) {
    return false;
  }

  packedUploadFile =
      LittleFS.open(
          PACKED_TMP_PATH,
          "w");

  if (!packedUploadFile) {
    return false;
  }

  packedUploadExpectedBytes =
      expectedBytes;

  saverBytesReceived =
      0;

  saverDataBytes =
      expectedBytes;

  saverFormat =
      SAVER_PACKED;

  saverUploading =
      true;

  saverReady =
      false;

  saverActive =
      false;

  return true;
}

static bool writePackedUploadChunk(
    uint32_t offset,
    const String &encoded) {
  if (!saverUploading ||
      saverFormat != SAVER_PACKED ||
      !packedUploadFile) {
    return false;
  }

  uint8_t decoded[1100] = {};
  size_t decodedLength = 0;

  int result =
      mbedtls_base64_decode(
          decoded,
          sizeof(decoded),
          &decodedLength,
          reinterpret_cast<
              const unsigned char *>(
              encoded.c_str()),
          encoded.length());

  if (result != 0 ||
      decodedLength == 0) {
    return false;
  }

  // Upload is ACKed chunk-by-chunk. If the ACK for the immediately previous
  // chunk is lost on USB CDC, the app retries that same offset. Treat that
  // exact last-chunk replay as already committed instead of rejecting the
  // entire ~2 MB transfer.
  if (offset <
      saverBytesReceived) {
    return offset +
               decodedLength ==
           saverBytesReceived;
  }

  if (offset !=
          saverBytesReceived ||
      saverBytesReceived +
              decodedLength >
          packedUploadExpectedBytes) {
    return false;
  }

  size_t written =
      packedUploadFile.write(
          decoded,
          decodedLength);

  if (written !=
      decodedLength) {
    return false;
  }

  saverBytesReceived +=
      decodedLength;

  return true;
}

static bool finishPackedUpload() {
  if (!saverUploading ||
      saverFormat != SAVER_PACKED ||
      saverBytesReceived !=
          packedUploadExpectedBytes) {
    closePackedFiles();
    return false;
  }

  packedUploadFile.flush();
  packedUploadFile.close();

  size_t actualSize = 0;

  if (!inspectPackedFile(
          PACKED_TMP_PATH,
          actualSize) ||
      actualSize !=
          packedUploadExpectedBytes) {
    LittleFS.remove(
        PACKED_TMP_PATH);

    saverUploading =
        false;

    saverReady =
        false;

    return false;
  }

  LittleFS.remove(
      PACKED_PATH);

  if (!LittleFS.rename(
          PACKED_TMP_PATH,
          PACKED_PATH)) {
    LittleFS.remove(
        PACKED_TMP_PATH);

    saverUploading =
        false;

    saverReady =
        false;

    return false;
  }

  LittleFS.remove(
      GIF_PATH);

  LittleFS.remove(
      GIF_TMP_PATH);

  LittleFS.remove(
      JPEG_PATH);

  LittleFS.remove(
      JPEG_TMP_PATH);

  saverUploading =
      false;

  saverReady =
      true;

  saverActive =
      false;

  saverFormat =
      SAVER_PACKED;

  saverWidth =
      packedStorageWidth;

  saverHeight =
      packedStorageHeight;

  saverDataBytes =
      actualSize;

  saverBytesReceived =
      actualSize;

  clearSaverIdentity();

  return true;
}

static bool loadPersistedPacked() {
  if (!littleFsReady ||
      !LittleFS.exists(
          PACKED_PATH)) {
    return false;
  }

  size_t fileSize = 0;

  if (!inspectPackedFile(
          PACKED_PATH,
          fileSize)) {
    LittleFS.remove(
        PACKED_PATH);

    return false;
  }

  saverFormat =
      SAVER_PACKED;

  saverWidth =
      packedStorageWidth;

  saverHeight =
      packedStorageHeight;

  saverDataBytes =
      fileSize;

  saverBytesReceived =
      fileSize;

  saverUploading =
      false;

  saverReady =
      true;

  saverActive =
      false;

  return true;
}

static bool readPackedSingleCode(
    File &file,
    uint8_t mode,
    uint16_t &color) {
  if (mode == 0) {
    int r = file.read();
    int g = file.read();
    int b = file.read();

    if (r < 0 ||
        g < 0 ||
        b < 0) {
      return false;
    }

    color =
        rgb888To565(
            static_cast<uint8_t>(r),
            static_cast<uint8_t>(g),
            static_cast<uint8_t>(b));

    return true;
  }

  if (mode == 1) {
    return readPackedU16(
        file,
        color);
  }

  int index =
      file.read();

  if (index < 0 ||
      static_cast<uint16_t>(
          index) >=
          packedPaletteCount) {
    return false;
  }

  color =
      packedPalette565[
          static_cast<uint8_t>(
              index)];

  return true;
}

static void setPackedScaledPixel(
    uint16_t sourceY,
    uint16_t sourceX,
    uint16_t color) {
  if (renderBuffer == nullptr ||
      packedStorageWidth == 0 ||
      packedStorageHeight == 0) {
    return;
  }

  color =
      vividRgb565(
          color);

  uint16_t dx0 =
      static_cast<uint16_t>(
          (static_cast<uint32_t>(sourceX) *
           TFT_WIDTH) /
          packedStorageWidth);

  uint16_t dx1 =
      static_cast<uint16_t>(
          (static_cast<uint32_t>(sourceX + 1U) *
           TFT_WIDTH) /
          packedStorageWidth);

  uint16_t dy0 =
      static_cast<uint16_t>(
          (static_cast<uint32_t>(sourceY) *
           TFT_HEIGHT) /
          packedStorageHeight);

  uint16_t dy1 =
      static_cast<uint16_t>(
          (static_cast<uint32_t>(sourceY + 1U) *
           TFT_HEIGHT) /
          packedStorageHeight);

  if (dx1 <= dx0) {
    uint32_t next =
        static_cast<uint32_t>(dx0) + 1U;
    dx1 =
        static_cast<uint16_t>(
            next > TFT_WIDTH
                ? TFT_WIDTH
                : next);
  }

  if (dy1 <= dy0) {
    uint32_t next =
        static_cast<uint32_t>(dy0) + 1U;
    dy1 =
        static_cast<uint16_t>(
            next > TFT_HEIGHT
                ? TFT_HEIGHT
                : next);
  }

  for (uint16_t y = dy0;
       y < dy1 && y < TFT_HEIGHT;
       ++y) {
    uint16_t *row =
        renderBuffer +
        static_cast<size_t>(y) *
            TFT_WIDTH;

    for (uint16_t x = dx0;
         x < dx1 && x < TFT_WIDTH;
         ++x) {
      row[x] = color;
    }
  }
}

static bool decodePackedSpan(
    File &file,
    uint16_t sourceY,
    uint16_t sourceX,
    uint16_t sourceCount) {
  if (renderBuffer == nullptr ||
      sourceY >= packedStorageHeight ||
      sourceX >= packedStorageWidth ||
      sourceCount == 0 ||
      static_cast<uint32_t>(sourceX) +
              sourceCount >
          packedStorageWidth) {
    return false;
  }

  uint16_t produced = 0;

  while (produced < sourceCount) {
    int controlRead = file.read();

    if (controlRead < 0) {
      return false;
    }

    uint8_t control =
        static_cast<uint8_t>(controlRead);

    uint16_t packetCount =
        static_cast<uint16_t>(
            (control & 0x7F) + 1U);

    if (produced + packetCount >
        sourceCount) {
      return false;
    }

    bool repeat =
        (control & 0x80) != 0;

    if (repeat) {
      uint16_t color = 0;

      if (!readPackedSingleCode(
              file,
              packedColorMode,
              color)) {
        return false;
      }

      for (uint16_t i = 0;
           i < packetCount;
           ++i) {
        setPackedScaledPixel(
            sourceY,
            static_cast<uint16_t>(
                sourceX + produced + i),
            color);
      }

      produced += packetCount;
      continue;
    }

    if (packedColorMode <= 2) {
      for (uint16_t i = 0;
           i < packetCount;
           ++i) {
        uint16_t color = 0;

        if (!readPackedSingleCode(
                file,
                packedColorMode,
                color)) {
          return false;
        }

        setPackedScaledPixel(
            sourceY,
            static_cast<uint16_t>(
                sourceX + produced + i),
            color);
      }

      produced += packetCount;
      continue;
    }

    uint8_t bits =
        packedColorMode == 3
            ? 4
            : packedColorMode == 4
                ? 2
                : 1;

    size_t packedBytes =
        (static_cast<size_t>(packetCount) *
             bits +
         7U) /
        8U;

    uint8_t packed[64] = {};

    if (packedBytes > sizeof(packed) ||
        file.read(
            packed,
            packedBytes) !=
            packedBytes) {
      return false;
    }

    uint8_t mask =
        static_cast<uint8_t>(
            (1U << bits) - 1U);

    for (uint16_t i = 0;
         i < packetCount;
         ++i) {
      uint16_t bitPosition =
          static_cast<uint16_t>(
              i * bits);

      uint16_t byteIndex =
          bitPosition / 8U;

      uint8_t shift =
          static_cast<uint8_t>(
              bitPosition % 8U);

      uint8_t paletteIndex =
          static_cast<uint8_t>(
              (packed[byteIndex] >>
               shift) &
              mask);

      if (paletteIndex >=
          packedPaletteCount) {
        return false;
      }

      setPackedScaledPixel(
          sourceY,
          static_cast<uint16_t>(
              sourceX + produced + i),
          packedPalette565[
              paletteIndex]);
    }

    produced += packetCount;
  }

  return true;
}

static bool openPackedPlayback() {
  closePackedFiles();

  if (!littleFsReady ||
      !LittleFS.exists(
          PACKED_PATH) ||
      renderBuffer == nullptr) {
    return false;
  }

  packedPlaybackFile =
      LittleFS.open(
          PACKED_PATH,
          "r");

  if (!packedPlaybackFile) {
    return false;
  }

  if (!readPackedHeader(
          packedPlaybackFile,
          true)) {
    packedPlaybackFile.close();
    return false;
  }

  packedFrameIndex = 0;

  memset(
      renderBuffer,
      0,
      static_cast<size_t>(TFT_WIDTH) *
          TFT_HEIGHT *
          sizeof(uint16_t));

  packedNextFrameAt =
      millis();

  return true;
}

static bool decodeNextPackedFrame() {
  if (!packedPlaybackFile ||
      packedFrameCount == 0 ||
      renderBuffer == nullptr ||
      !displayReady) {
    return false;
  }

  if (packedFrameIndex >=
      packedFrameCount) {
    if (!packedPlaybackFile.seek(
            packedFramesOffset)) {
      return false;
    }

    packedFrameIndex = 0;

    memset(
        renderBuffer,
        0,
        static_cast<size_t>(TFT_WIDTH) *
            TFT_HEIGHT *
            sizeof(uint16_t));
  }

  // Use an absolute deadline. The old code added the hold time after
  // decoding and TFT writes, so every frame also paid the render cost.
  const uint32_t frameStartedAt =
      millis();

  uint16_t durationMs = 0;
  uint16_t spanCount = 0;

  if (!readPackedU16(
          packedPlaybackFile,
          durationMs) ||
      !readPackedU16(
          packedPlaybackFile,
          spanCount)) {
    return false;
  }

  // Assemble the complete delta frame in PSRAM first.
  for (uint16_t span = 0;
       span < spanCount;
       ++span) {
    uint16_t y = 0;
    uint16_t x = 0;
    uint16_t count = 0;

    if (!readPackedU16(
            packedPlaybackFile,
            y) ||
        !readPackedU16(
            packedPlaybackFile,
            x) ||
        !readPackedU16(
            packedPlaybackFile,
            count) ||
        !decodePackedSpan(
            packedPlaybackFile,
            y,
            x,
            count)) {
      return false;
    }
  }

  // Present one completed frame instead of exposing hundreds of small span
  // writes while the image is still being assembled.
  tft->draw16bitRGBBitmap(
      0,
      0,
      renderBuffer,
      TFT_WIDTH,
      TFT_HEIGHT);

  packedFrameIndex++;

  const uint32_t holdMs =
      durationMs == 0
          ? 1U
          : static_cast<uint32_t>(
                durationMs);

  packedNextFrameAt =
      frameStartedAt + holdMs;

  return true;
}

static void closeJpegUploadFile() {
  if (jpegUploadFile) {
    jpegUploadFile.close();
  }
}

static int jpegDraw(JPEGDRAW *draw) {
  if (draw == nullptr ||
      draw->pPixels == nullptr ||
      !displayReady) {
    return 0;
  }

  int x = draw->x;
  int y = draw->y;
  int sourceStride = draw->iWidth;
  int width =
      draw->iWidthUsed > 0
          ? draw->iWidthUsed
          : draw->iWidth;
  int height = draw->iHeight;

  if (x < 0 ||
      y < 0 ||
      x + width > TFT_WIDTH ||
      y + height > TFT_HEIGHT ||
      sourceStride < width) {
    return 0;
  }

  // iWidthUsed can be smaller than the MCU row stride on odd image widths.
  // Draw row-by-row so edge padding never writes outside the centered image.
  for (int row = 0; row < height; ++row) {
    drawVividRgb565Row(
        static_cast<int16_t>(
            x),
        static_cast<int16_t>(
            y + row),
        draw->pPixels +
            static_cast<size_t>(row) *
            sourceStride,
        static_cast<uint16_t>(
            width));
  }

  return 1;
}

static bool inspectJpeg(
    const char *path,
    uint16_t &width,
    uint16_t &height,
    size_t &fileSize) {
  if (!littleFsReady) {
    return false;
  }

  File file =
      LittleFS.open(
          path,
          "r");

  if (!file) {
    return false;
  }

  fileSize =
      file.size();

  JPEGDEC decoder;

  if (!decoder.open(
          file,
          jpegDraw)) {
    file.close();
    return false;
  }

  int decodedWidth =
      decoder.getWidth();

  int decodedHeight =
      decoder.getHeight();

  decoder.close();
  file.close();

  if (decodedWidth < 1 ||
      decodedHeight < 1 ||
      decodedWidth > TFT_WIDTH ||
      decodedHeight > TFT_HEIGHT) {
    return false;
  }

  width =
      static_cast<uint16_t>(
          decodedWidth);

  height =
      static_cast<uint16_t>(
          decodedHeight);

  return true;
}

static bool renderJpeg() {
  if (!littleFsReady ||
      !LittleFS.exists(JPEG_PATH) ||
      !displayReady) {
    return false;
  }

  jpegPlaybackFile =
      LittleFS.open(
          JPEG_PATH,
          "r");

  if (!jpegPlaybackFile) {
    return false;
  }

  if (!jpegDecoder.open(
          jpegPlaybackFile,
          jpegDraw)) {
    jpegPlaybackFile.close();
    return false;
  }

  int width =
      jpegDecoder.getWidth();

  int height =
      jpegDecoder.getHeight();

  if (width < 1 ||
      height < 1 ||
      width > TFT_WIDTH ||
      height > TFT_HEIGHT) {
    jpegDecoder.close();
    jpegPlaybackFile.close();
    return false;
  }

  int offsetX =
      (TFT_WIDTH - width) / 2;

  int offsetY =
      (TFT_HEIGHT - height) / 2;

  tft->fillScreen(
      RGB565_BLACK);

  int result =
      jpegDecoder.decode(
          offsetX,
          offsetY,
          0);

  jpegDecoder.close();
  jpegPlaybackFile.close();

  return result != 0;
}

static bool beginJpegUpload(
    uint32_t expectedBytes,
    uint16_t width,
    uint16_t height) {
  if (!littleFsReady ||
      expectedBytes < 4 ||
      expectedBytes > JPEG_UPLOAD_LIMIT_BYTES ||
      width < 1 ||
      height < 1 ||
      width > TFT_WIDTH ||
      height > TFT_HEIGHT) {
    return false;
  }

  clearSaverBuffer();

  size_t total =
      LittleFS.totalBytes();

  size_t used =
      LittleFS.usedBytes();

  size_t freeBytes =
      total > used
          ? total - used
          : 0;

  if (expectedBytes + 4096 > freeBytes) {
    return false;
  }

  jpegUploadFile =
      LittleFS.open(
          JPEG_TMP_PATH,
          "w");

  if (!jpegUploadFile) {
    return false;
  }

  jpegUploadExpectedBytes =
      expectedBytes;
  jpegUploadWidth =
      width;
  jpegUploadHeight =
      height;

  saverBytesReceived = 0;
  saverDataBytes =
      expectedBytes;
  saverWidth =
      width;
  saverHeight =
      height;
  saverFormat =
      SAVER_JPEG;
  saverUploading = true;
  saverReady = false;
  saverActive = false;

  return true;
}

static bool writeJpegUploadChunk(
    uint32_t offset,
    const String &encoded) {
  if (!saverUploading ||
      saverFormat != SAVER_JPEG ||
      !jpegUploadFile) {
    return false;
  }

  uint8_t decoded[1100] = {};
  size_t decodedLength = 0;

  int result =
      mbedtls_base64_decode(
          decoded,
          sizeof(decoded),
          &decodedLength,
          reinterpret_cast<
              const unsigned char *>(
              encoded.c_str()),
          encoded.length());

  if (result != 0 ||
      decodedLength == 0) {
    return false;
  }

  if (offset < saverBytesReceived) {
    return offset + decodedLength ==
           saverBytesReceived;
  }

  if (offset != saverBytesReceived ||
      saverBytesReceived +
              decodedLength >
          jpegUploadExpectedBytes) {
    return false;
  }

  size_t written =
      jpegUploadFile.write(
          decoded,
          decodedLength);

  if (written != decodedLength) {
    return false;
  }

  saverBytesReceived +=
      decodedLength;

  return true;
}

static bool finishJpegUpload() {
  if (!saverUploading ||
      saverFormat != SAVER_JPEG ||
      saverBytesReceived !=
          jpegUploadExpectedBytes) {
    closeJpegUploadFile();
    return false;
  }

  jpegUploadFile.flush();
  closeJpegUploadFile();

  uint16_t actualWidth = 0;
  uint16_t actualHeight = 0;
  size_t actualSize = 0;

  if (!inspectJpeg(
          JPEG_TMP_PATH,
          actualWidth,
          actualHeight,
          actualSize) ||
      actualSize !=
          jpegUploadExpectedBytes ||
      actualWidth !=
          jpegUploadWidth ||
      actualHeight !=
          jpegUploadHeight) {
    LittleFS.remove(
        JPEG_TMP_PATH);
    saverUploading = false;
    saverReady = false;
    return false;
  }

  LittleFS.remove(
      JPEG_PATH);

  if (!LittleFS.rename(
          JPEG_TMP_PATH,
          JPEG_PATH)) {
    LittleFS.remove(
        JPEG_TMP_PATH);
    saverUploading = false;
    saverReady = false;
    return false;
  }

  LittleFS.remove(
      GIF_PATH);
  LittleFS.remove(
      GIF_TMP_PATH);
  LittleFS.remove(
      PACKED_PATH);
  LittleFS.remove(
      PACKED_TMP_PATH);

  saverUploading = false;
  saverReady = true;
  saverActive = false;
  saverFormat =
      SAVER_JPEG;
  saverWidth =
      actualWidth;
  saverHeight =
      actualHeight;
  saverDataBytes =
      actualSize;
  saverBytesReceived =
      actualSize;

  preferences.putUShort(
      "jpgw",
      actualWidth);
  preferences.putUShort(
      "jpgh",
      actualHeight);

  clearSaverIdentity();

  return true;
}

static bool loadPersistedJpeg() {
  if (!littleFsReady ||
      !LittleFS.exists(
          JPEG_PATH)) {
    return false;
  }

  uint16_t width = 0;
  uint16_t height = 0;
  size_t fileSize = 0;

  if (!inspectJpeg(
          JPEG_PATH,
          width,
          height,
          fileSize)) {
    LittleFS.remove(
        JPEG_PATH);
    return false;
  }

  saverFormat =
      SAVER_JPEG;
  saverWidth =
      width;
  saverHeight =
      height;
  saverDataBytes =
      fileSize;
  saverBytesReceived =
      fileSize;
  saverUploading = false;
  saverReady = true;
  saverActive = false;

  return true;
}

static void loadPersistedMedia() {
  saverReady = false;

  if (loadPersistedPacked()) {
    return;
  }

  if (loadPersistedJpeg()) {
    return;
  }

  loadPersistedGif();

}

static void clearSaverBuffer() {
  closeGifDecoder();
  closeGifUploadFile();
  closeJpegUploadFile();
  closePackedFiles();

  if (jpegPlaybackFile) {
    jpegPlaybackFile.close();
  }

  if (littleFsReady) {
    LittleFS.remove(GIF_TMP_PATH);
    LittleFS.remove(GIF_PATH);
    LittleFS.remove(JPEG_TMP_PATH);
    LittleFS.remove(JPEG_PATH);
    LittleFS.remove(PACKED_TMP_PATH);
    LittleFS.remove(PACKED_PATH);
    LittleFS.remove(SAVER_THUMB_TMP_PATH);
    LittleFS.remove(SAVER_THUMB_PATH);
  }

  preferences.remove("sav_name");
  preferences.remove("sav_kind");

  if (saverData != nullptr) {
    free(saverData);
    saverData = nullptr;
  }

  saverDataBytes = 0;
  saverFrameBytes = 0;
  saverBytesReceived = 0;
  saverFrameCount = 0;
  saverWidth = 0;
  saverHeight = 0;
  saverFormat = SAVER_NONE;
  saverUploading = false;
  saverReady = false;
  saverActive = false;
  saverFrameIndex = 0;
  saverFrameStartedAt = 0;

  gifUploadExpectedBytes = 0;
  gifUploadWidth = 0;
  gifUploadHeight = 0;

  jpegUploadExpectedBytes = 0;
  jpegUploadWidth = 0;
  jpegUploadHeight = 0;

  packedUploadExpectedBytes = 0;
  packedStorageWidth = 0;
  packedStorageHeight = 0;
  packedFrameCount = 0;
  packedFps = 0;
  packedFrameIndex = 0;
  packedPaletteCount = 0;
  packedColorMode = 0;
  packedDurationMs = 0;
  packedFramesOffset = 0;
  packedNextFrameAt = 0;
  memset(
      packedPalette565,
      0,
      sizeof(packedPalette565));

  memset(saverDurations, 0, sizeof(saverDurations));

}

static void initDisplay() {
  // LCD_RST is tied directly to EN. BASELINE_V3 proved this panel is reliable
  // when it is allowed to settle for 1 second after the board reset before
  // the first HX8357-B command.
  delay(1000);

  displayReady = tft->begin();

  if (!displayReady) {
    return;
  }

  // Match the exact final order from the proven BASELINE_V3:
  // display-on happens in tftInit(), then landscape MADCTL, then REV_SCREEN
  // polarity (INVON / 0x21).
  tft->setRotation(1);
  tft->invertDisplay(false);
  delay(20);

  // PixelHX8357BMcufriend intentionally mirrors MCUFRIEND_kbv's minimal
  // ID 0x8357 initialization. Production also uses PixelStablePAR8 so D33-D40
  // settle before every WR edge; this avoids the washed-out/flickering output
  // seen with the stock zero-wait Arduino_ESP32PAR8 path.

  tft->fillScreen(RGB565_BLACK);

  if (ESP.getPsramSize() > 0) {
    renderBuffer = static_cast<uint16_t *>(
        ps_malloc(static_cast<size_t>(TFT_WIDTH) * TFT_HEIGHT * 2));

    packedLineBuffer = static_cast<uint16_t *>(
        ps_malloc(static_cast<size_t>(TFT_WIDTH) * sizeof(uint16_t)));
  }

  // Persistent media stays in flash/LittleFS. PSRAM is used only as a
  // transient decode/render buffer; fall back to internal SRAM if PSRAM
  // allocation is unavailable.
  if (packedLineBuffer == nullptr) {
    packedLineBuffer = packedLineFallback;
  }
}

static uint16_t rgb332To565(uint8_t value) {
  uint8_t r3 = (value >> 5) & 0x07;
  uint8_t g3 = (value >> 2) & 0x07;
  uint8_t b2 = value & 0x03;

  uint16_t r5 = static_cast<uint16_t>((r3 * 31 + 3) / 7);
  uint16_t g6 = static_cast<uint16_t>((g3 * 63 + 3) / 7);
  uint16_t b5 = static_cast<uint16_t>((b2 * 31 + 1) / 3);

  return vividRgb565(
      static_cast<uint16_t>(
          (r5 << 11) |
          (g6 << 5) |
          b5));
}

static void renderSaverFrame(uint8_t index) {
  if (!displayReady ||
      !saverReady ||
      saverData == nullptr ||
      index >= saverFrameCount) {
    return;
  }

  uint8_t *frame =
      saverData + static_cast<size_t>(index) * saverFrameBytes;

  if (saverFormat == SAVER_RGB565) {
    tft->draw16bitRGBBitmap(
        0,
        0,
        reinterpret_cast<uint16_t *>(frame),
        TFT_WIDTH,
        TFT_HEIGHT);
    return;
  }

  if (saverFormat != SAVER_RGB332 ||
      renderBuffer == nullptr) {
    return;
  }

  for (uint16_t y = 0; y < GIF_HEIGHT; ++y) {
    const uint8_t *src =
        frame + static_cast<size_t>(y) * GIF_WIDTH;

    uint16_t *row0 =
        renderBuffer +
        static_cast<size_t>(y * 2) * TFT_WIDTH;

    uint16_t *row1 = row0 + TFT_WIDTH;

    for (uint16_t x = 0; x < GIF_WIDTH; ++x) {
      uint16_t color = rgb332To565(src[x]);
      uint16_t dx = x * 2;

      row0[dx] = color;
      row0[dx + 1] = color;
      row1[dx] = color;
      row1[dx + 1] = color;
    }
  }

  tft->draw16bitRGBBitmap(
      0,
      0,
      renderBuffer,
      TFT_WIDTH,
      TFT_HEIGHT);
}

static void startSaverNow() {
  // Calibration owns the display until all four targets have been captured
  // and the final finger release is observed.
  if (touchCalibrationMode ||
      touchPixelTestMode) {
    return;
  }

  if (!saverReady || !displayReady) {
    return;
  }

  if (saverFormat == SAVER_PACKED) {
    if (!openPackedPlayback()) {
      saverReady = false;
      return;
    }

    saverActive = true;
    saverFrameStartedAt = millis();

    tft->fillScreen(
        RGB565_BLACK);

    if (!decodeNextPackedFrame()) {
      stopSaver();
    }

    return;
  }

  if (saverFormat == SAVER_GIF) {
    if (!openGifDecoder()) {
      saverReady = false;
      return;
    }

    saverActive = true;
    saverFrameIndex = 0;
    saverFrameStartedAt = millis();

    if (!decodeNextGifFrame()) {
      stopSaver();
    }

    return;
  }

  if (saverFormat == SAVER_JPEG) {
    saverActive = true;
    saverFrameIndex = 0;
    saverFrameStartedAt = millis();

    if (!renderJpeg()) {
      saverReady = false;
      stopSaver();
    }

    return;
  }

  if (saverData == nullptr) {
    return;
  }

  saverActive = true;
  saverFrameIndex = 0;
  saverFrameStartedAt = millis();
  renderSaverFrame(0);
}

static void stopSaver() {
  bool wasActive = saverActive;

  saverActive = false;
  saverFrameIndex = 0;
  saverFrameStartedAt = 0;

  closeGifDecoder();

  if (packedPlaybackFile) {
    packedPlaybackFile.close();
  }

  if (wasActive &&
      displayReady &&
      !touchCalibrationMode &&
      !touchPixelTestMode) {
    renderMainMenu();
  }
}

static void pollSaver() {
  uint32_t now = millis();

  // Never allow inactivity timing or playback to take ownership of the TFT
  // while touch calibration is active.
  if (touchCalibrationMode ||
      touchPixelTestMode) {
    return;
  }

  if (!saverActive) {
    if (saverReady &&
        saverDelayMs > 0 &&
        pressedMask == 0 &&
        static_cast<uint32_t>(now - lastUserActivityAt) >= saverDelayMs) {
      startSaverNow();
    }

    return;
  }

  if (!saverReady) {
    return;
  }

  if (saverFormat == SAVER_PACKED) {
    if (static_cast<int32_t>(
            now - packedNextFrameAt) >= 0) {
      if (!decodeNextPackedFrame()) {
        stopSaver();
      }
    }

    return;
  }

  if (saverFormat == SAVER_GIF) {
    if (static_cast<int32_t>(
            now - gifNextFrameAt) >= 0) {
      if (!decodeNextGifFrame()) {
        stopSaver();
      }
    }

    return;
  }

  if (saverFormat == SAVER_JPEG ||
      saverFrameCount <= 1 ||
      saverFormat == SAVER_RGB565) {
    return;
  }

  uint16_t duration =
      saverDurations[saverFrameIndex] < GIF_MIN_FRAME_MS
          ? GIF_MIN_FRAME_MS
          : saverDurations[saverFrameIndex];

  if (static_cast<uint32_t>(now - saverFrameStartedAt) < duration) {
    return;
  }

  saverFrameIndex =
      static_cast<uint8_t>((saverFrameIndex + 1) % saverFrameCount);

  saverFrameStartedAt = now;
  renderSaverFrame(saverFrameIndex);
}

static bool parseSaverDurations(
    const String &csv,
    uint8_t frameCount) {
  int start = 0;

  for (uint8_t i = 0; i < frameCount; ++i) {
    int comma = csv.indexOf(',', start);
    bool last = i == frameCount - 1;

    if ((!last && comma < 0) ||
        (last && comma >= 0)) {
      return false;
    }

    String token =
        last ? csv.substring(start) : csv.substring(start, comma);

    uint16_t duration = 0;

    if (!parseUnsigned(token, 5000, duration)) {
      return false;
    }

    saverDurations[i] =
        duration < GIF_MIN_FRAME_MS
            ? GIF_MIN_FRAME_MS
            : duration;

    start = comma + 1;
  }

  return true;
}

static bool beginSaverUpload(
    uint8_t frameCount,
    uint16_t width,
    uint16_t height,
    SaverPixelFormat format,
    const String &durationCsv) {
  clearSaverBuffer();

  bool staticImage =
      format == SAVER_RGB565 &&
      frameCount == 1 &&
      width == TFT_WIDTH &&
      height == TFT_HEIGHT;

  bool animated =
      format == SAVER_RGB332 &&
      frameCount >= 1 &&
      frameCount <= GIF_MAX_FRAMES &&
      width == GIF_WIDTH &&
      height == GIF_HEIGHT;

  if (!staticImage && !animated) {
    return false;
  }

  if (!parseSaverDurations(durationCsv, frameCount)) {
    clearSaverBuffer();
    return false;
  }

  saverFrameCount = frameCount;
  saverWidth = width;
  saverHeight = height;
  saverFormat = format;

  saverFrameBytes =
      static_cast<size_t>(width) *
      height *
      (format == SAVER_RGB565 ? 2 : 1);

  saverDataBytes =
      saverFrameBytes * frameCount;

  if (saverDataBytes == 0 ||
      saverDataBytes > ESP.getFreePsram()) {
    clearSaverBuffer();
    return false;
  }

  saverData =
      static_cast<uint8_t *>(ps_malloc(saverDataBytes));

  if (saverData == nullptr) {
    clearSaverBuffer();
    return false;
  }

  memset(saverData, 0, saverDataBytes);
  saverBytesReceived = 0;
  saverUploading = true;
  saverReady = false;
  saverActive = false;

  return true;
}

static bool writeSaverChunk(
    uint8_t frameIndex,
    size_t offset,
    const String &encoded) {
  if (!saverUploading ||
      saverData == nullptr ||
      frameIndex >= saverFrameCount ||
      offset >= saverFrameBytes) {
    return false;
  }

  size_t expectedOffset =
      static_cast<size_t>(frameIndex) * saverFrameBytes +
      offset;

  if (expectedOffset != saverBytesReceived) {
    return false;
  }

  uint8_t decoded[320] = {};
  size_t decodedLength = 0;

  int result =
      mbedtls_base64_decode(
          decoded,
          sizeof(decoded),
          &decodedLength,
          reinterpret_cast<const unsigned char *>(encoded.c_str()),
          encoded.length());

  if (result != 0 ||
      decodedLength == 0 ||
      offset + decodedLength > saverFrameBytes ||
      saverBytesReceived + decodedLength > saverDataBytes) {
    return false;
  }

  memcpy(
      saverData + saverBytesReceived,
      decoded,
      decodedLength);

  saverBytesReceived += decodedLength;
  return true;
}

static bool finishSaverUpload() {
  if (!saverUploading ||
      saverData == nullptr ||
      saverBytesReceived != saverDataBytes) {
    return false;
  }

  saverUploading = false;
  saverReady = true;
  saverActive = false;
  saverFrameIndex = 0;
  saverFrameStartedAt = 0;

  return true;
}

static String deviceHello() {
  char out[448];
  snprintf(
      out,
      sizeof(out),
      "PIXELPRO|1|FW=%s|MCU=ESP32S2|KEYS=8|PROFILES=20|LAYERS=4|MACROS=20|ACTIONS=32|DISPLAY=HX8357B-MCUFRIEND,480x320,i8080-8|CAPS=HID,CDC,KEYMAP,LAYERS,HOST_MACRO,HOST_ACTION,MEM,PANEL,SAVER,MEDIA,DIRECT_GIF,DIRECT_JPEG,PXQ,RLE,DELTA,RAW_MEDIA_V1,RGB_PER_KEY,RGB_EFFECTS,MAIN_MENU,MAIN_MENU_ICONS,PCMON,MATRIX_2X4,ENCODER,ROLLER_EVQWGD001,TOUCH_RESISTIVE,SD_SPI,MODULE_I2C,PCA9546A,3PORT,ROM_BOOT|VID=%04X|PID=%04X",
      FW_VERSION,
      USB_VID_PIXEL,
      USB_PID_PIXEL);
  return String(out);
}

static void sendMemoryInfo() {
  const uint32_t flashTotal = ESP.getFlashChipSize();
  const uint32_t fileSystemUsed =
      littleFsReady
          ? static_cast<uint32_t>(LittleFS.usedBytes())
          : 0;
  const uint32_t sketchUsed = ESP.getSketchSize();
  const uint32_t flashUsed =
      sketchUsed + fileSystemUsed > flashTotal
          ? flashTotal
          : sketchUsed + fileSystemUsed;

  const uint32_t sramTotal = ESP.getHeapSize();
  const uint32_t sramFree = ESP.getFreeHeap();
  const uint32_t sramUsed =
      sramTotal > sramFree ? sramTotal - sramFree : 0;

  const uint32_t psramTotal = ESP.getPsramSize();
  const uint32_t psramFree = ESP.getFreePsram();
  const uint32_t psramUsed =
      psramTotal > psramFree ? psramTotal - psramFree : 0;

  char out[160];
  snprintf(
      out,
      sizeof(out),
      "MEM|%lu|%lu|%lu|%lu|%lu|%lu",
      static_cast<unsigned long>(flashUsed),
      static_cast<unsigned long>(flashTotal),
      static_cast<unsigned long>(sramUsed),
      static_cast<unsigned long>(sramTotal),
      static_cast<unsigned long>(psramUsed),
      static_cast<unsigned long>(psramTotal));
  cdcPrintln(out);
}

static void sendKeyState() {
  char out[48];
  snprintf(
      out,
      sizeof(out),
      "KEYS|%02X|P=%u|L=%u",
      pressedMask,
      static_cast<unsigned>(activeProfile),
      static_cast<unsigned>(currentLayer()));
  cdcPrintln(out);
}

static void sendMappedReports() {
  if (!HID.ready()) {
    return;
  }

  KeyReport report = {};
  uint8_t slot = 0;
  uint16_t consumer = 0;

  for (uint8_t i = 0; i < KEY_COUNT; ++i) {
    if ((pressedMask & static_cast<uint8_t>(1U << i)) == 0) {
      continue;
    }

    const KeyBinding &binding = activeBindings[i];

    if (binding.type == BIND_KEYBOARD) {
      report.modifiers |= binding.modifiers;

      if (binding.keyCode != 0 && slot < 6) {
        report.keys[slot++] = binding.keyCode;
      }
    } else if (binding.type == BIND_CONSUMER && consumer == 0) {
      consumer = binding.consumerCode;
    }
  }

  Keyboard.sendReport(&report);

  if (consumer != activeConsumerCode) {
    if (activeConsumerCode != 0) {
      ConsumerControl.release();
    }

    if (consumer != 0) {
      ConsumerControl.press(consumer);
    }

    activeConsumerCode = consumer;
  }
}

static void executeMacro(uint8_t index) {
  if (index >= MACRO_COUNT ||
      macros[index].length() == 0 ||
      !HID.ready()) {
    return;
  }

  Keyboard.print(macros[index]);
  delay(2);
  sendMappedReports();
}

static void applyLayerPress(const KeyBinding &binding) {
  uint8_t target = binding.keyCode;

  if (target >= LAYER_COUNT) {
    return;
  }

  if (binding.modifiers == LAYER_MO) {
    momentaryLayer = target;
  } else if (binding.modifiers == LAYER_TG) {
    toggledLayerMask ^= static_cast<uint8_t>(1U << target);
  } else if (binding.modifiers == LAYER_TO) {
    baseLayer = target;
    momentaryLayer = -1;
    toggledLayerMask = 0;
  }
}

static void applyLayerRelease(const KeyBinding &binding) {
  if (binding.type == BIND_LAYER &&
      binding.modifiers == LAYER_MO &&
      momentaryLayer == binding.keyCode) {
    momentaryLayer = -1;
  }
}


static const char *rawMediaKindName(
    RawMediaKind kind) {
  switch (kind) {
    case RAW_MEDIA_MENU_BG:
      return "MENUBG";
    case RAW_MEDIA_MENU_ICON:
      return "ICON";
    case RAW_MEDIA_GIF:
      return "GIF";
    case RAW_MEDIA_JPEG:
      return "JPG";
    case RAW_MEDIA_PACKED:
      return "PX";
    case RAW_MEDIA_THUMB:
      return "THUMB";
    case RAW_MEDIA_NONE:
    default:
      return "NONE";
  }
}

static void resetRawMediaState() {
  rawMediaKind = RAW_MEDIA_NONE;
  rawMediaExpectedBytes = 0;
  rawMediaReceivedBytes = 0;
  rawMediaExpectedCrc = 0;
  rawMediaRunningCrc = 0xFFFFFFFFUL;
  rawMediaNextAckAt = 0;
  rawMediaLastActivityAt = 0;
  rawMediaProfile = 0;
  rawMediaSlot = 0;
}

static void abortRawMediaTransfer(
    const char *reason) {
  const RawMediaKind failed =
      rawMediaKind;

  switch (failed) {
    case RAW_MEDIA_MENU_BG:
    case RAW_MEDIA_MENU_ICON:
      closeMenuUpload();
      if (littleFsReady) {
        if (failed ==
            RAW_MEDIA_MENU_BG) {
          char path[24] = {};
          menuBackgroundPath(
              rawMediaProfile,
              true,
              path,
              sizeof(path));
          LittleFS.remove(path);
        } else {
          char path[24] = {};
          menuIconPath(
              rawMediaProfile,
              rawMediaSlot,
              true,
              path,
              sizeof(path));
          LittleFS.remove(path);
        }
      }
      break;

    case RAW_MEDIA_GIF:
      closeGifUploadFile();
      if (littleFsReady) {
        LittleFS.remove(
            GIF_TMP_PATH);
      }
      saverUploading = false;
      saverReady = false;
      break;

    case RAW_MEDIA_JPEG:
      closeJpegUploadFile();
      if (littleFsReady) {
        LittleFS.remove(
            JPEG_TMP_PATH);
      }
      saverUploading = false;
      saverReady = false;
      break;

    case RAW_MEDIA_PACKED:
      closePackedFiles();
      if (littleFsReady) {
        LittleFS.remove(
            PACKED_TMP_PATH);
      }
      saverUploading = false;
      saverReady = false;
      break;

    case RAW_MEDIA_THUMB:
      closeSaverThumbUpload();
      if (littleFsReady) {
        LittleFS.remove(
            SAVER_THUMB_TMP_PATH);
      }
      break;

    case RAW_MEDIA_NONE:
    default:
      break;
  }

  resetRawMediaState();

  if (reason != nullptr &&
      reason[0] != '\0') {
    cdcPrintln(
        String("ERR|MEDIA_RAW|") +
        reason);
  }
}

static bool finishRawMediaTransfer() {
  if (rawMediaKind ==
          RAW_MEDIA_NONE ||
      rawMediaReceivedBytes !=
          rawMediaExpectedBytes) {
    return false;
  }

  const uint32_t actualCrc =
      rawMediaRunningCrc ^
      0xFFFFFFFFUL;

  if (actualCrc !=
      rawMediaExpectedCrc) {
    char error[80] = {};
    snprintf(
        error,
        sizeof(error),
        "CRC|EXPECTED=%08lX|ACTUAL=%08lX",
        static_cast<unsigned long>(
            rawMediaExpectedCrc),
        static_cast<unsigned long>(
            actualCrc));

    abortRawMediaTransfer(
        error);
    return false;
  }

  const RawMediaKind completed =
      rawMediaKind;

  const uint32_t completedBytes =
      rawMediaExpectedBytes;

  const uint32_t completedCrc =
      actualCrc;

  bool ok = false;

  switch (completed) {
    case RAW_MEDIA_MENU_BG:
      ok =
          finishMenuBackgroundUpload();
      break;

    case RAW_MEDIA_MENU_ICON:
      ok =
          finishMenuIconUpload();
      break;

    case RAW_MEDIA_GIF:
      ok =
          finishGifUpload();
      break;

    case RAW_MEDIA_JPEG:
      ok =
          finishJpegUpload();
      break;

    case RAW_MEDIA_PACKED:
      ok =
          finishPackedUpload();
      break;

    case RAW_MEDIA_THUMB:
      ok =
          finishSaverThumbUpload();
      break;

    case RAW_MEDIA_NONE:
    default:
      ok = false;
      break;
  }

  const char *kindName =
      rawMediaKindName(
          completed);

  resetRawMediaState();

  if (!ok) {
    cdcPrintln(
        String("ERR|MEDIA_RAW|FINALIZE|") +
        kindName);
    return false;
  }

  lastUserActivityAt =
      millis();

  char out[96] = {};
  snprintf(
      out,
      sizeof(out),
      "OK|MEDIA_RAW_DONE|%s|%lu|%08lX",
      kindName,
      static_cast<unsigned long>(
          completedBytes),
      static_cast<unsigned long>(
          completedCrc));

  cdcPrintln(out);
  return true;
}

static bool writeRawMediaBytes(
    const uint8_t *data,
    size_t length) {
  if (rawMediaKind ==
          RAW_MEDIA_NONE ||
      data == nullptr ||
      length == 0 ||
      rawMediaReceivedBytes +
              length >
          rawMediaExpectedBytes) {
    return false;
  }

  size_t written = 0;

  switch (rawMediaKind) {
    case RAW_MEDIA_MENU_BG:
    case RAW_MEDIA_MENU_ICON:
      if (!menuUploadFile) {
        return false;
      }
      written =
          menuUploadFile.write(
              data,
              length);
      if (written ==
          length) {
        menuUploadReceivedBytes +=
            static_cast<uint32_t>(
                length);
      }
      break;

    case RAW_MEDIA_GIF:
      if (!gifUploadFile) {
        return false;
      }
      written =
          gifUploadFile.write(
              data,
              length);
      if (written ==
          length) {
        saverBytesReceived +=
            length;
      }
      break;

    case RAW_MEDIA_JPEG:
      if (!jpegUploadFile) {
        return false;
      }
      written =
          jpegUploadFile.write(
              data,
              length);
      if (written ==
          length) {
        saverBytesReceived +=
            length;
      }
      break;

    case RAW_MEDIA_PACKED:
      if (!packedUploadFile) {
        return false;
      }
      written =
          packedUploadFile.write(
              data,
              length);
      if (written ==
          length) {
        saverBytesReceived +=
            length;
      }
      break;

    case RAW_MEDIA_THUMB:
      if (!saverThumbUploadFile) {
        return false;
      }
      written =
          saverThumbUploadFile.write(
              data,
              length);
      if (written ==
          length) {
        saverThumbReceivedBytes +=
            static_cast<uint32_t>(
                length);
      }
      break;

    case RAW_MEDIA_NONE:
    default:
      return false;
  }

  if (written != length) {
    return false;
  }

  rawMediaRunningCrc =
      rawCrc32Update(
          rawMediaRunningCrc,
          data,
          length);

  rawMediaReceivedBytes +=
      static_cast<uint32_t>(
          length);

  rawMediaLastActivityAt =
      millis();

  if (rawMediaReceivedBytes ==
      rawMediaExpectedBytes) {
    return finishRawMediaTransfer();
  }

  if (rawMediaReceivedBytes ==
      rawMediaNextAckAt) {
    char out[48] = {};
    snprintf(
        out,
        sizeof(out),
        "OK|MEDIA_RAW_DATA|%lu",
        static_cast<unsigned long>(
            rawMediaReceivedBytes));

    cdcPrintln(out);

    rawMediaNextAckAt =
        min(
            rawMediaExpectedBytes,
            rawMediaNextAckAt +
                RAW_MEDIA_ACK_BYTES);
  }

  return true;
}

static bool beginRawMediaTransfer(
    const String &command) {
  if (rawMediaKind !=
      RAW_MEDIA_NONE) {
    cdcPrintln(
        "ERR|MEDIA_RAW|BUSY");
    return false;
  }

  String parts[8];
  uint8_t partCount = 0;
  int start = 0;

  while (start <=
             static_cast<int>(
                 command.length()) &&
         partCount <
             static_cast<uint8_t>(
                 sizeof(parts) /
                 sizeof(parts[0]))) {
    int sep =
        command.indexOf(
            '|',
            start);

    if (sep < 0) {
      parts[partCount++] =
          command.substring(
              start);
      break;
    }

    parts[partCount++] =
        command.substring(
            start,
            sep);

    start =
        sep + 1;
  }

  if (partCount < 4 ||
      !parts[0].equalsIgnoreCase(
          "MEDIA_RAW_BEGIN")) {
    cdcPrintln(
        "ERR|MEDIA_RAW|BAD_BEGIN");
    return false;
  }

  parts[1].toUpperCase();

  RawMediaKind kind =
      RAW_MEDIA_NONE;

  uint32_t expectedBytes = 0;
  uint32_t expectedCrc = 0;
  bool started = false;

  if (parts[1] == "MENUBG" &&
      partCount == 5) {
    uint16_t profile = 0;

    if (parseUnsigned(
            parts[2],
            PROFILE_COUNT - 1,
            profile) &&
        parseUnsignedLong(
            parts[3],
            MENU_BACKGROUND_LIMIT_BYTES,
            expectedBytes) &&
        parseHexU32(
            parts[4],
            expectedCrc)) {
      started =
          beginMenuBackgroundUpload(
              static_cast<uint8_t>(
                  profile),
              expectedBytes);

      if (started) {
        kind =
            RAW_MEDIA_MENU_BG;
        rawMediaProfile =
            static_cast<uint8_t>(
                profile);
      }
    }
  } else if (
      parts[1] == "ICON" &&
      partCount == 6) {
    uint16_t profile = 0;
    uint16_t slot = 0;

    if (parseUnsigned(
            parts[2],
            PROFILE_COUNT - 1,
            profile) &&
        parseUnsigned(
            parts[3],
            MENU_SLOT_COUNT - 1,
            slot) &&
        parseUnsignedLong(
            parts[4],
            MENU_ICON_MAX_BYTES,
            expectedBytes) &&
        parseHexU32(
            parts[5],
            expectedCrc)) {
      started =
          beginMenuIconUpload(
              static_cast<uint8_t>(
                  profile),
              static_cast<uint8_t>(
                  slot),
              expectedBytes);

      if (started) {
        kind =
            RAW_MEDIA_MENU_ICON;
        rawMediaProfile =
            static_cast<uint8_t>(
                profile);
        rawMediaSlot =
            static_cast<uint8_t>(
                slot);
      }
    }
  } else if (
      parts[1] == "GIF" &&
      partCount == 6) {
    uint16_t width = 0;
    uint16_t height = 0;

    if (parseUnsignedLong(
            parts[2],
            GIF_UPLOAD_LIMIT_BYTES,
            expectedBytes) &&
        parseUnsigned(
            parts[3],
            1024,
            width) &&
        parseUnsigned(
            parts[4],
            1024,
            height) &&
        parseHexU32(
            parts[5],
            expectedCrc)) {
      started =
          beginGifUpload(
              expectedBytes,
              width,
              height,
              GIF_SCALE_CENTER);

      if (started) {
        kind =
            RAW_MEDIA_GIF;
      }
    }
  } else if (
      parts[1] == "JPG" &&
      partCount == 6) {
    uint16_t width = 0;
    uint16_t height = 0;

    if (parseUnsignedLong(
            parts[2],
            JPEG_UPLOAD_LIMIT_BYTES,
            expectedBytes) &&
        parseUnsigned(
            parts[3],
            TFT_WIDTH,
            width) &&
        parseUnsigned(
            parts[4],
            TFT_HEIGHT,
            height) &&
        parseHexU32(
            parts[5],
            expectedCrc)) {
      started =
          beginJpegUpload(
              expectedBytes,
              width,
              height);

      if (started) {
        kind =
            RAW_MEDIA_JPEG;
      }
    }
  } else if (
      parts[1] == "PX" &&
      partCount == 4) {
    if (parseUnsignedLong(
            parts[2],
            PACKED_UPLOAD_LIMIT_BYTES,
            expectedBytes) &&
        parseHexU32(
            parts[3],
            expectedCrc)) {
      started =
          beginPackedUpload(
              expectedBytes);

      if (started) {
        kind =
            RAW_MEDIA_PACKED;
      }
    }
  } else if (
      parts[1] == "THUMB" &&
      partCount == 4) {
    if (parseUnsignedLong(
            parts[2],
            SAVER_THUMB_LIMIT_BYTES,
            expectedBytes) &&
        parseHexU32(
            parts[3],
            expectedCrc)) {
      started =
          beginSaverThumbUpload(
              expectedBytes);

      if (started) {
        kind =
            RAW_MEDIA_THUMB;
      }
    }
  }

  if (!started ||
      kind ==
          RAW_MEDIA_NONE ||
      expectedBytes == 0) {
    cdcPrintln(
        "ERR|MEDIA_RAW|BEGIN");
    return false;
  }

  rawMediaKind = kind;
  rawMediaExpectedBytes =
      expectedBytes;
  rawMediaReceivedBytes = 0;
  rawMediaExpectedCrc =
      expectedCrc;
  rawMediaRunningCrc =
      0xFFFFFFFFUL;
  rawMediaNextAckAt =
      min(
          expectedBytes,
          RAW_MEDIA_ACK_BYTES);
  rawMediaLastActivityAt =
      millis();

  char out[80] = {};
  snprintf(
      out,
      sizeof(out),
      "OK|MEDIA_RAW_BEGIN|%s|%lu",
      rawMediaKindName(kind),
      static_cast<unsigned long>(
          expectedBytes));

  cdcPrintln(out);
  return true;
}

static void pollRawMediaTransfer() {
  if (rawMediaKind ==
      RAW_MEDIA_NONE) {
    return;
  }

  if (static_cast<uint32_t>(
          millis() -
          rawMediaLastActivityAt) >
      RAW_MEDIA_TIMEOUT_MS) {
    abortRawMediaTransfer(
        "TIMEOUT");
    return;
  }

  // Keep each loop iteration short. Large reads followed by repeated
  // LittleFS writes can starve TinyUSB/WDT on ESP32-S2 and make a reconnect
  // upload turn into a reset loop.
  static uint8_t buffer[512];

  if (USBSerial.available() <= 0) {
    return;
  }

  const uint32_t remaining =
      rawMediaExpectedBytes -
      rawMediaReceivedBytes;

  if (remaining == 0) {
    (void)finishRawMediaTransfer();
    return;
  }

  const int available =
      USBSerial.available();

  size_t want =
      static_cast<size_t>(
          remaining <
                  static_cast<uint32_t>(
                      sizeof(buffer))
              ? remaining
              : static_cast<uint32_t>(
                    sizeof(buffer)));

  want =
      min(
          want,
          static_cast<size_t>(
              available));

  const size_t got =
      USBSerial.read(
          buffer,
          want);

  if (got == 0) {
    return;
  }

  if (!writeRawMediaBytes(
          buffer,
          got)) {
    if (rawMediaKind !=
        RAW_MEDIA_NONE) {
      abortRawMediaTransfer(
          "WRITE");
    }
    return;
  }

  // Let the USB task and watchdog run before accepting the next 512 bytes.
  delay(1);
}

static void handleCommand(String command) {
  command.trim();

  String upper = command;
  upper.toUpperCase();


  if (upper.startsWith(
          "MEDIA_RAW_BEGIN|")) {
    (void)beginRawMediaTransfer(
        command);
    return;
  }

  if (upper == "MEDIA_RAW_CAPS") {
    cdcPrintln(
        "MEDIA_RAW_CAPS|V=1|ACK=512|CRC=CRC32|KINDS=MENUBG,ICON,GIF,JPG,PX,THUMB");
    return;
  }

  if (upper == "HELLO" || upper == "GET_INFO") {
    cdcPrintln(deviceHello());
    return;
  }

  if (upper == "GET_KEYS") {
    sendKeyState();
    return;
  }

  if (upper == "SDINFO" ||
      upper == "GET_SD") {
    sendSdInfo();
    return;
  }

  if (upper == "SDREMOUNT") {
    if (mountSdCard()) {
      cdcPrintln(
          "OK|SD_MOUNTED");
    } else {
      cdcPrintln(
          "ERR|SD_NOT_FOUND");
    }

    return;
  }

  if (upper == "SDTEST") {
    if (runSdSelfTest()) {
      cdcPrintln(
          "SDTEST|OK");
    } else {
      cdcPrintln(
          "SDTEST|FAIL");
    }

    return;
  }

  if (upper == "GET_MODULES") {
    sendModuleSummary();
    return;
  }

  if (upper == "MODULE_SCAN") {
    moduleMuxReady =
        moduleMuxDisable();

    for (uint8_t port = 0;
         port < MODULE_PORT_COUNT;
         ++port) {
      pollModulePort(
          port);
    }

    sendModuleSummary();
    return;
  }

  if (upper.startsWith("MODULE_TX|")) {
    const int p1 =
        command.indexOf('|');

    const int p2 =
        command.indexOf(
            '|',
            p1 + 1);

    uint16_t portNumber = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        !parseUnsigned(
            command.substring(
                p1 + 1,
                p2),
            MODULE_PORT_COUNT,
            portNumber) ||
        portNumber < 1) {
      cdcPrintln(
          "ERR|BAD_MODULE_PORT");
      return;
    }

    uint8_t payload[
        MODULE_MAX_TX_BYTES] = {};

    uint8_t payloadLength = 0;

    if (!parseModuleHex(
            command.substring(
                p2 + 1),
            payload,
            payloadLength)) {
      cdcPrintln(
          "ERR|BAD_MODULE_HEX");
      return;
    }

    const bool ok =
        moduleSendRaw(
            static_cast<uint8_t>(
                portNumber - 1),
            payload,
            payloadLength);

    cdcPrintln(
        ok
            ? "OK|MODULE_TX"
            : "ERR|MODULE_TX");
    return;
  }

  if (upper == "TOUCH_TEST_START") {
    touchCalibrationMode = false;
    touchPixelTestMode = true;
    touchCalibrationPoint = 0;
    touchCalibrationPointCaptured = false;
    touchRawPressed = false;
    touchStablePressed = false;
    touchPressConfirmations = 0;
    touchReleaseMisses = 0;
    touchCandidateRawX = 0;
    touchCandidateRawY = 0;
    touchPressStartedAt = 0;
    touchPendingSlot = -1;
    touchWakeOnly = false;
    touchHeldFallbackSlot = -1;
    touchChangedAt = millis();
    lastUserActivityAt =
        touchChangedAt;

    stopSaver();
    drawTouchPixelTestScreen();

    cdcPrintln(
        "OK|TOUCH_TEST_START|480|320");
    return;
  }

  if (upper == "TOUCH_TEST_STOP") {
    touchPixelTestMode = false;
    touchRawPressed = false;
    touchStablePressed = false;
    touchPressConfirmations = 0;
    touchReleaseMisses = 0;
    touchCandidateRawX = 0;
    touchCandidateRawY = 0;
    touchPressStartedAt = 0;
    touchPendingSlot = -1;
    touchChangedAt = millis();
    lastUserActivityAt =
        touchChangedAt;

    renderMainMenu();

    cdcPrintln(
        "OK|TOUCH_TEST_STOP");
    return;
  }

  if (upper == "GET_TOUCH_TEST_STATE") {
    char out[80] = {};
    snprintf(
        out,
        sizeof(out),
        "TOUCH_TEST_STATE|ACTIVE=%u|W=%u|H=%u",
        touchPixelTestMode ? 1U : 0U,
        static_cast<unsigned>(
            TFT_WIDTH),
        static_cast<unsigned>(
            TFT_HEIGHT));
    cdcPrintln(out);
    return;
  }

  if (upper == "TOUCH_CAL_START") {
    startAutomaticTouchCalibration();
    cdcPrintln(
        "OK|TOUCH_CAL_START");
    return;
  }

  if (upper == "TOUCH_CAL_CANCEL") {
    cancelAutomaticTouchCalibration();
    cdcPrintln(
        "OK|TOUCH_CAL_CANCEL");
    return;
  }

  if (upper == "GET_TOUCH_CAL_STATE") {
    char out[64] = {};
    snprintf(
        out,
        sizeof(out),
        "TOUCH_CAL_STATE|ACTIVE=%u|POINT=%u",
        touchCalibrationMode ? 1U : 0U,
        static_cast<unsigned>(
            touchCalibrationPoint));
    cdcPrintln(
        out);
    return;
  }

  if (upper == "GET_TOUCH_RAW") {
    uint16_t rawX = 0;
    uint16_t rawY = 0;
    uint16_t pressure = 0;

    const bool pressed =
        readTouchPointMcufriend(
            rawX,
            rawY,
            pressure);

    int16_t x = -1;
    int16_t y = -1;

    if (pressed) {
      mapTouchDefaultPixels(
          rawX,
          rawY,
          x,
          y);
    }

    char out[128] = {};
    snprintf(
        out,
        sizeof(out),
        "TOUCHRAW|DOWN=%u|RAWX=%u|RAWY=%u|P=%u|X=%d|Y=%d",
        pressed ? 1U : 0U,
        static_cast<unsigned>(rawX),
        static_cast<unsigned>(rawY),
        static_cast<unsigned>(pressure),
        static_cast<int>(x),
        static_cast<int>(y));

    cdcPrintln(out);
    return;
  }

  if (upper == "GET_TOUCH_CAL") {
    char out[96];
    snprintf(
        out,
        sizeof(out),
        "TOUCH_CAL|%u|%u|%u|%u|%u",
        static_cast<unsigned>(touchCalibration.xMin),
        static_cast<unsigned>(touchCalibration.xMax),
        static_cast<unsigned>(touchCalibration.yMin),
        static_cast<unsigned>(touchCalibration.yMax),
        static_cast<unsigned>(touchCalibration.flags));
    cdcPrintln(out);
    return;
  }

  if (upper == "RESET_TOUCH_CAL") {
    setDefaultTouchCalibration();
    touchCalibrationRequired = true;
    saveTouchCalibration();
    cdcPrintln("OK|TOUCH_CAL_RESET");
    return;
  }

  if (upper.startsWith("SET_TOUCH_CAL|")) {
    int p1 = command.indexOf('|');
    int p2 = command.indexOf('|', p1 + 1);
    int p3 = command.indexOf('|', p2 + 1);
    int p4 = command.indexOf('|', p3 + 1);
    int p5 = command.indexOf('|', p4 + 1);

    uint16_t xMin = 0;
    uint16_t xMax = 0;
    uint16_t yMin = 0;
    uint16_t yMax = 0;
    uint16_t flags = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        p3 < 0 ||
        p4 < 0 ||
        p5 < 0 ||
        !parseUnsigned(
            command.substring(p1 + 1, p2),
            TOUCH_ADC_MAX,
            xMin) ||
        !parseUnsigned(
            command.substring(p2 + 1, p3),
            TOUCH_ADC_MAX,
            xMax) ||
        !parseUnsigned(
            command.substring(p3 + 1, p4),
            TOUCH_ADC_MAX,
            yMin) ||
        !parseUnsigned(
            command.substring(p4 + 1, p5),
            TOUCH_ADC_MAX,
            yMax) ||
        !parseUnsigned(
            command.substring(p5 + 1),
            TOUCH_FLAG_SWAP_XY |
                TOUCH_FLAG_INVERT_X |
                TOUCH_FLAG_INVERT_Y,
            flags)) {
      cdcPrintln("ERR|BAD_TOUCH_CAL");
      return;
    }

    TouchCalibration candidate = {
        xMin,
        xMax,
        yMin,
        yMax,
        static_cast<uint8_t>(flags)};

    if (!touchCalibrationIsValid(candidate)) {
      cdcPrintln("ERR|BAD_TOUCH_CAL");
      return;
    }

    touchCalibration = candidate;
    touchAffine = {};
    touchAffineValid = false;
    touchCalibrationRequired = false;
    saveTouchCalibration();
    cdcPrintln("OK|TOUCH_CAL");
    return;
  }

  if (upper == "GET_LAYER") {
    char out[48];
    snprintf(
        out,
        sizeof(out),
        "LAYER|PROFILE=%u|ACTIVE=%u|BASE=%u|TOGGLE=%u",
        static_cast<unsigned>(activeProfile),
        static_cast<unsigned>(currentLayer()),
        static_cast<unsigned>(baseLayer),
        static_cast<unsigned>(toggledLayerMask));
    cdcPrintln(out);
    return;
  }

  if (upper == "MEM" || upper == "GET_MEMORY") {
    sendMemoryInfo();
    return;
  }

  if (upper == "PANEL") {
    cdcPrintln("PANEL|HX8357B-MCUFRIEND|60|0|60|VIVID=1|TOUCHTEST=480x320");
    return;
  }

  if (upper.startsWith("HOSTOS|")) {
    String host =
        upper.substring(7);

    host.trim();

    if (host == "WIN" ||
        host == "WINDOWS") {
      menuHostOs = 1;
    } else if (
        host == "MAC" ||
        host == "MACOS" ||
        host == "OSX") {
      menuHostOs = 2;
    } else if (
        host == "LINUX") {
      menuHostOs = 3;
    } else {
      cdcPrintln("ERR|HOSTOS");
      return;
    }

    preferences.putUChar(
        "host_os",
        menuHostOs);

    if (!saverActive) {
      renderMainMenuStatusBar();
    }

    cdcPrintln("OK|HOSTOS");
    return;
  }

  if (upper == "PCCLEAR") {
    menuCpuLoad = -1;
    menuCpuTemp = -1;
    menuGpuLoad = -1;
    menuGpuTemp = -1;
    menuMonth = 0;
    menuDay = 0;
    menuHour = 0;
    menuMinute = 0;
    menuPcStatusValid = false;

    if (!saverActive) {
      renderMainMenuStatusBar();
    }

    return;
  }

  if (upper.startsWith("PCMON|")) {
    int p1 = command.indexOf('|');
    int p2 = command.indexOf('|', p1 + 1);
    int p3 = command.indexOf('|', p2 + 1);
    int p4 = command.indexOf('|', p3 + 1);
    int p5 = command.indexOf('|', p4 + 1);
    int p6 = command.indexOf('|', p5 + 1);
    int p7 = command.indexOf('|', p6 + 1);
    int p8 = command.indexOf('|', p7 + 1);

    int16_t cpuLoad = -1;
    int16_t cpuTemp = -1;
    int16_t gpuLoad = -1;
    int16_t gpuTemp = -1;

    uint16_t month = 0;
    uint16_t day = 0;
    uint16_t hour = 0;
    uint16_t minute = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        p3 < 0 ||
        p4 < 0 ||
        p5 < 0 ||
        p6 < 0 ||
        p7 < 0 ||
        p8 < 0 ||
        !parseSigned(
            command.substring(p1 + 1, p2),
            -1,
            100,
            cpuLoad) ||
        !parseSigned(
            command.substring(p2 + 1, p3),
            -1,
            150,
            cpuTemp) ||
        !parseSigned(
            command.substring(p3 + 1, p4),
            -1,
            100,
            gpuLoad) ||
        !parseSigned(
            command.substring(p4 + 1, p5),
            -1,
            150,
            gpuTemp) ||
        !parseUnsigned(
            command.substring(p5 + 1, p6),
            12,
            month) ||
        month < 1 ||
        !parseUnsigned(
            command.substring(p6 + 1, p7),
            31,
            day) ||
        day < 1 ||
        !parseUnsigned(
            command.substring(p7 + 1, p8),
            23,
            hour) ||
        !parseUnsigned(
            command.substring(p8 + 1),
            59,
            minute)) {
      return;
    }

    menuCpuLoad = cpuLoad;
    menuCpuTemp = cpuTemp;
    menuGpuLoad = gpuLoad;
    menuGpuTemp = gpuTemp;
    menuMonth = static_cast<uint8_t>(month);
    menuDay = static_cast<uint8_t>(day);
    menuHour = static_cast<uint8_t>(hour);
    menuMinute = static_cast<uint8_t>(minute);
    menuPcStatusValid = true;

    if (!saverActive) {
      renderMainMenuStatusBar();
    }

    return;
  }

  if (upper.startsWith("MENUBATCHBEGIN|")) {
    int sep =
        command.indexOf('|');

    uint16_t profile = 0;

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(
                sep + 1),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln(
          "ERR|MENUBATCHBEGIN");
      return;
    }

    closeMenuUpload();
    beginMainMenuBatch(
        static_cast<uint8_t>(
            profile));

    char out[48] = {};
    snprintf(
        out,
        sizeof(out),
        "OK|MENUBATCHBEGIN|%u",
        static_cast<unsigned>(
            profile));
    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("MENUBATCHEND|")) {
    int sep =
        command.indexOf('|');

    uint16_t profile = 0;

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(
                sep + 1),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln(
          "ERR|MENUBATCHEND");
      return;
    }

    closeMenuUpload();
    endMainMenuBatch(
        static_cast<uint8_t>(
            profile));

    char out[48] = {};
    snprintf(
        out,
        sizeof(out),
        "OK|MENUBATCHEND|%u",
        static_cast<unsigned>(
            profile));
    cdcPrintln(out);
    return;
  }

  if (upper == "RESETINFO") {
    char out[96] = {};
    snprintf(
        out,
        sizeof(out),
        "RESETINFO|REASON=%s|CODE=%u|BOOT=%lu",
        resetReasonName(
            bootResetReason),
        static_cast<unsigned>(
            bootResetReason),
        static_cast<unsigned long>(
            bootSequence));
    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("GET_MENUCFG|")) {
    uint16_t profile = 0;
    int sep = command.indexOf('|');

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(sep + 1),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln("ERR|BAD_MENU_PROFILE");
      return;
    }

    String out = "MENUCFG|";
    out += String(profile);
    out += '|';

    for (uint8_t slot = 0; slot < MENU_SLOT_COUNT; ++slot) {
      if (slot) out += ',';
      out += String(mainMenuConfig.actions[profile][slot]);
    }

    out += '|';

    for (uint8_t slot = 0; slot < MENU_SLOT_COUNT; ++slot) {
      if (slot) out += ',';
      out += String(mainMenuConfig.labels[profile][slot]);
    }

    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("MENUCFG|")) {
    int p1 = command.indexOf('|');
    int p2 = command.indexOf('|', p1 + 1);
    int p3 = command.indexOf('|', p2 + 1);

    uint16_t profile = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        p3 < 0 ||
        !parseUnsigned(
            command.substring(p1 + 1, p2),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln("ERR|BAD_MENUCFG");
      return;
    }

    String actionCsv =
        command.substring(
            p2 + 1,
            p3);

    int actionStart = 0;

    for (uint8_t slot = 0; slot < MENU_SLOT_COUNT; ++slot) {
      int comma =
          actionCsv.indexOf(
              ',',
              actionStart);

      bool last =
          slot ==
          MENU_SLOT_COUNT - 1;

      if ((!last && comma < 0) ||
          (last && comma >= 0)) {
        cdcPrintln("ERR|BAD_MENU_ACTIONS");
        return;
      }

      String token =
          last
              ? actionCsv.substring(actionStart)
              : actionCsv.substring(actionStart, comma);

      uint16_t action = 0;
      if (!parseUnsigned(
              token,
              ACTION_COUNT,
              action)) {
        cdcPrintln("ERR|BAD_MENU_ACTION");
        return;
      }

      mainMenuConfig.actions[profile][slot] =
          static_cast<uint8_t>(action);

      actionStart =
          comma + 1;
    }

    String labelCsv =
        command.substring(
            p3 + 1);

    int labelStart = 0;

    for (uint8_t slot = 0; slot < MENU_SLOT_COUNT; ++slot) {
      int comma =
          labelCsv.indexOf(
              ',',
              labelStart);

      bool last =
          slot ==
          MENU_SLOT_COUNT - 1;

      if ((!last && comma < 0) ||
          (last && comma >= 0)) {
        cdcPrintln("ERR|BAD_MENU_LABELS");
        return;
      }

      String label =
          last
              ? labelCsv.substring(labelStart)
              : labelCsv.substring(labelStart, comma);

      label.trim();

      if (label.length() > MENU_LABEL_MAX_LEN) {
        label =
            label.substring(
                0,
                MENU_LABEL_MAX_LEN);
      }

      memset(
          mainMenuConfig.labels[profile][slot],
          0,
          MENU_LABEL_MAX_LEN + 1);

      label.toCharArray(
          mainMenuConfig.labels[profile][slot],
          MENU_LABEL_MAX_LEN + 1);

      labelStart =
          comma + 1;
    }

    if (!saveMainMenuConfig()) {
      cdcPrintln("ERR|MENUCFG_SAVE");
      return;
    }

    bool hasAction = false;
    for (uint8_t slot = 0;
         slot < MENU_SLOT_COUNT;
         ++slot) {
      if (mainMenuConfig.actions[profile][slot] > 0) {
        hasAction = true;
        break;
      }
    }

    if (hasAction) {
      rememberMainMenuContentProfile(
          static_cast<uint8_t>(
              profile));
    }

    requestMainMenuRender(
        static_cast<uint8_t>(
            profile));

    cdcPrintln("OK|MENUCFG");
    return;
  }

  if (upper.startsWith("MENUICONSTATE|")) {
    uint16_t profile = 0;
    int sep = command.indexOf('|');

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(sep + 1),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln("ERR|MENUICONSTATE");
      return;
    }

    uint8_t mask = 0;

    for (uint8_t slot = 0;
         slot < MENU_SLOT_COUNT;
         ++slot) {
      char path[24] = {};
      menuIconPath(
          static_cast<uint8_t>(profile),
          slot,
          false,
          path,
          sizeof(path));

      if (littleFsReady &&
          LittleFS.exists(path)) {
        File icon =
            LittleFS.open(path, "r");

        if (icon &&
            icon.size() ==
                MENU_ICON_ASSET_BYTES) {
          mask |=
              static_cast<uint8_t>(
                  1U << slot);
        }

        if (icon) {
          icon.close();
        }
      }
    }

    char out[64] = {};
    snprintf(
        out,
        sizeof(out),
        "MENUICONSTATE|PROFILE=%u|MASK=%02X",
        static_cast<unsigned>(profile),
        static_cast<unsigned>(mask));

    cdcPrintln(out);
    return;
  }

  if (upper == "MENURENDERSTATE") {
    const uint8_t profile =
        resolveMainMenuRenderProfile();

    char bgPath[24] = {};
    menuBackgroundPath(
        profile,
        false,
        bgPath,
        sizeof(bgPath));

    const bool bg =
        littleFsReady &&
        LittleFS.exists(
            bgPath);

    const bool composite =
        mainMenuCompositeEnabled(
            profile);

    char out[140] = {};
    snprintf(
        out,
        sizeof(out),
        "MENURENDERSTATE|PROFILE=%u|ACTIVE=%u|BG=%u|COMPOSITE=%u|FS=%u|FW=%s",
        static_cast<unsigned>(
            profile),
        static_cast<unsigned>(
            activeProfile),
        bg ? 1U : 0U,
        composite ? 1U : 0U,
        littleFsReady ? 1U : 0U,
        FW_VERSION);

    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("MENUMODESTATE|")) {
    int sep =
        command.indexOf('|');

    uint16_t profile = 0;

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(
                sep + 1),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln(
          "ERR|MENUMODESTATE");
      return;
    }

    const bool composite =
        mainMenuCompositeEnabled(
            static_cast<uint8_t>(
                profile));

    char out[72] = {};
    snprintf(
        out,
        sizeof(out),
        "MENUMODESTATE|PROFILE=%u|MODE=%s",
        static_cast<unsigned>(
            profile),
        composite
            ? "COMPOSITE"
            : "LEGACY");

    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("MENUMODE|")) {
    int p1 =
        command.indexOf('|');

    int p2 =
        command.indexOf(
            '|',
            p1 + 1);

    uint16_t profile = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        !parseUnsigned(
            command.substring(
                p1 + 1,
                p2),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln(
          "ERR|MENUMODE");
      return;
    }

    String mode =
        upper.substring(
            p2 + 1);

    const bool composite =
        mode ==
        "COMPOSITE";

    const bool legacy =
        mode ==
        "LEGACY";

    if ((!composite &&
         !legacy) ||
        !setMainMenuCompositeEnabled(
            static_cast<uint8_t>(
                profile),
            composite)) {
      cdcPrintln(
          "ERR|MENUMODE");
      return;
    }

    requestMainMenuRender(
        static_cast<uint8_t>(
            profile));

    char out[64] = {};
    snprintf(
        out,
        sizeof(out),
        "OK|MENUMODE|%u|%s",
        static_cast<unsigned>(
            profile),
        composite
            ? "COMPOSITE"
            : "LEGACY");

    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("MENUBGSTATE|")) {
    int sep = command.indexOf('|');
    uint16_t profile = 0;

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(sep + 1),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln("ERR|MENUBGSTATE");
      return;
    }

    char path[24] = {};
    menuBackgroundPath(
        static_cast<uint8_t>(profile),
        false,
        path,
        sizeof(path));

    bool custom =
        littleFsReady &&
        LittleFS.exists(path);

    size_t bytes = 0;

    if (custom) {
      File file =
          LittleFS.open(
              path,
              "r");

      if (file) {
        bytes =
            file.size();
        file.close();
      } else {
        custom = false;
      }
    }

    char out[180] = {};

    snprintf(
        out,
        sizeof(out),
        "MENUBGSTATE|PROFILE=%u|STATE=%s|ASSET=%s|BYTES=%lu|W=%u|H=%u",
        static_cast<unsigned>(profile),
        custom
            ? "CUSTOM"
            : "EMPTY",
        custom
            ? "USER_JPEG"
            : "NONE",
        static_cast<unsigned long>(bytes),
        static_cast<unsigned>(
            custom
                ? TFT_WIDTH
                : 0),
        static_cast<unsigned>(
            custom
                ? TFT_HEIGHT
                : 0));

    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("MENUBGBEGIN|")) {
    int p1 = command.indexOf('|');
    int p2 = command.indexOf('|', p1 + 1);
    uint16_t profile = 0;
    uint32_t bytes = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        !parseUnsigned(
            command.substring(p1 + 1, p2),
            PROFILE_COUNT - 1,
            profile) ||
        !parseUnsignedLong(
            command.substring(p2 + 1),
            MENU_BACKGROUND_LIMIT_BYTES,
            bytes) ||
        !beginMenuBackgroundUpload(
            static_cast<uint8_t>(profile),
            bytes)) {
      cdcPrintln("ERR|MENUBGBEGIN");
      return;
    }

    cdcPrintln("OK|MENUBGBEGIN");
    return;
  }

  if (upper.startsWith("MENUBGDATA|")) {
    int p1 = command.indexOf('|');
    int p2 = command.indexOf('|', p1 + 1);
    uint32_t offset = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        !parseUnsignedLong(
            command.substring(p1 + 1, p2),
            menuUploadExpectedBytes,
            offset) ||
        !writeMenuAssetChunk(
            offset,
            command.substring(p2 + 1))) {
      cdcPrintln("ERR|MENUBGDATA");
      return;
    }

    char out[40];
    snprintf(
        out,
        sizeof(out),
        "OK|MENUBGDATA|%lu",
        static_cast<unsigned long>(menuUploadReceivedBytes));
    cdcPrintln(out);
    return;
  }

  if (upper == "MENUBGEND") {
    if (!finishMenuBackgroundUpload()) {
      cdcPrintln("ERR|MENUBGEND");
      return;
    }

    cdcPrintln("OK|MENUBGEND");
    return;
  }

  if (upper.startsWith("MENUBGCLEAR|")) {
    int sep = command.indexOf('|');
    uint16_t profile = 0;

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(sep + 1),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln("ERR|MENUBGCLEAR");
      return;
    }

    closeMenuUpload();
    clearMainMenuBackground(
        static_cast<uint8_t>(profile));

    requestMainMenuRender(
        static_cast<uint8_t>(
            profile));

    cdcPrintln("OK|MENUBGCLEAR");
    return;
  }

  if (upper.startsWith("MENUICONBEGIN|")) {
    int p1 = command.indexOf('|');
    int p2 = command.indexOf('|', p1 + 1);
    int p3 = command.indexOf('|', p2 + 1);
    uint16_t profile = 0;
    uint16_t slot = 0;
    uint32_t bytes = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        p3 < 0 ||
        !parseUnsigned(
            command.substring(p1 + 1, p2),
            PROFILE_COUNT - 1,
            profile) ||
        !parseUnsigned(
            command.substring(p2 + 1, p3),
            MENU_SLOT_COUNT - 1,
            slot) ||
        !parseUnsignedLong(
            command.substring(p3 + 1),
            MENU_ICON_MAX_BYTES,
            bytes) ||
        !beginMenuIconUpload(
            static_cast<uint8_t>(profile),
            static_cast<uint8_t>(slot),
            bytes)) {
      cdcPrintln("ERR|MENUICONBEGIN");
      return;
    }

    cdcPrintln("OK|MENUICONBEGIN");
    return;
  }

  if (upper.startsWith("MENUICONDATA|")) {
    int p1 = command.indexOf('|');
    int p2 = command.indexOf('|', p1 + 1);
    uint32_t offset = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        !parseUnsignedLong(
            command.substring(p1 + 1, p2),
            menuUploadExpectedBytes,
            offset) ||
        !writeMenuAssetChunk(
            offset,
            command.substring(p2 + 1))) {
      cdcPrintln("ERR|MENUICONDATA");
      return;
    }

    char out[40];
    snprintf(
        out,
        sizeof(out),
        "OK|MENUICONDATA|%lu",
        static_cast<unsigned long>(menuUploadReceivedBytes));
    cdcPrintln(out);
    return;
  }

  if (upper == "MENUICONEND") {
    if (!finishMenuIconUpload()) {
      cdcPrintln("ERR|MENUICONEND");
      return;
    }

    cdcPrintln("OK|MENUICONEND");
    return;
  }

  if (upper.startsWith("MENUICONCLEAR|")) {
    int p1 = command.indexOf('|');
    int p2 = command.indexOf('|', p1 + 1);
    uint16_t profile = 0;
    uint16_t slot = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        !parseUnsigned(
            command.substring(p1 + 1, p2),
            PROFILE_COUNT - 1,
            profile) ||
        !parseUnsigned(
            command.substring(p2 + 1),
            MENU_SLOT_COUNT - 1,
            slot)) {
      cdcPrintln("ERR|MENUICONCLEAR");
      return;
    }

    clearMainMenuIcon(
        static_cast<uint8_t>(profile),
        static_cast<uint8_t>(slot));

    requestMainMenuRender(
        static_cast<uint8_t>(
            profile));

    cdcPrintln("OK|MENUICONCLEAR");
    return;
  }

  if (upper == "MENUSHOW") {
    menuBatchActive = false;
    menuBatchDirty = false;
    menuBatchLastActivityAt = 0;
    stopSaver();

    scheduleMainMenuRender(
        activeProfile,
        MENU_RENDER_DEFER_MS);

    cdcPrintln("OK|MENUSHOW");
    return;
  }

  if (upper == "FSINFO") {
    size_t total =
        littleFsReady
            ? LittleFS.totalBytes()
            : 0;

    size_t used =
        littleFsReady
            ? LittleFS.usedBytes()
            : 0;

    size_t freeBytes =
        total > used
            ? total - used
            : 0;

    uint8_t iconMask = 0;

    if (littleFsReady) {
      for (uint8_t slot = 0;
           slot < MENU_SLOT_COUNT;
           ++slot) {
        char path[24] = {};
        menuIconPath(
            activeProfile,
            slot,
            false,
            path,
            sizeof(path));

        if (LittleFS.exists(path)) {
          File icon =
              LittleFS.open(path, "r");

          if (icon &&
              icon.size() ==
                  MENU_ICON_ASSET_BYTES) {
            iconMask |=
                static_cast<uint8_t>(
                    1U << slot);
          }

          if (icon) {
            icon.close();
          }
        }
      }
    }

    char bgPath[24] = {};
    menuBackgroundPath(
        activeProfile,
        false,
        bgPath,
        sizeof(bgPath));

    const bool customBackground =
        littleFsReady &&
        LittleFS.exists(bgPath);

    char out[220] = {};
    snprintf(
        out,
        sizeof(out),
        "FSINFO|READY=%u|TOTAL=%lu|USED=%lu|FREE=%lu|PROFILE=%u|BG=%s|ICONS=%02X|MODE=%s|SAVER=%s|FW=%s",
        littleFsReady ? 1U : 0U,
        static_cast<unsigned long>(total),
        static_cast<unsigned long>(used),
        static_cast<unsigned long>(freeBytes),
        static_cast<unsigned>(activeProfile),
        customBackground ? "CUSTOM" : "EMPTY",
        static_cast<unsigned>(iconMask),
        mainMenuCompositeEnabled(
            activeProfile)
            ? "COMPOSITE"
            : "LEGACY",
        saverReady ? "CUSTOM" : "EMPTY",
        FW_VERSION);

    cdcPrintln(out);
    return;
  }

  if (upper == "FSREPAIR") {
    if (littleFsReady) {
      cdcPrintln("OK|FSREPAIR|ALREADY_READY");
      return;
    }

    littleFsReady =
        mountPersistentStorage(true);

    if (!littleFsReady) {
      cdcPrintln("ERR|FSREPAIR");
      return;
    }

    recoverMainMenuAssets();
    loadPersistedMedia();

    if (displayReady &&
        !saverActive) {
      renderMainMenu();
    }

    cdcPrintln("OK|FSREPAIR|READY");
    return;
  }

  if (upper == "SAVERINFO") {
    size_t total =
        littleFsReady
            ? LittleFS.totalBytes()
            : 0;

    size_t used =
        littleFsReady
            ? LittleFS.usedBytes()
            : 0;

    size_t freeBytes =
        total > used
            ? total - used
            : 0;

    char out[128];
    snprintf(
        out,
        sizeof(out),
        "SAVERINFO|TOTAL=%lu|USED=%lu|FREE=%lu|FLASH=%lu",
        static_cast<unsigned long>(total),
        static_cast<unsigned long>(used),
        static_cast<unsigned long>(freeBytes),
        static_cast<unsigned long>(ESP.getFlashChipSize()));

    cdcPrintln(out);
    return;
  }

  if (upper == "SAVERSTATE") {
    if (saverUploading) {
      cdcPrintln("SAVERSTATE|UPLOADING");
    } else if (saverReady) {
      cdcPrintln("SAVERSTATE|READY");
    } else {
      cdcPrintln("SAVERSTATE|EMPTY");
    }
    return;
  }

  if (upper == "SAVMEDIA") {
    if (!saverReady) {
      cdcPrintln(
          "SAVMEDIA|STATE=EMPTY");
      return;
    }


    String kind =
        preferences.getString(
            "sav_kind",
            "");

    if (kind.length() == 0) {
      kind =
          saverFormat == SAVER_PACKED
              ? "GIF"
              : saverFormat == SAVER_GIF
                  ? "GIF"
                  : saverFormat == SAVER_JPEG
                      ? "IMAGE"
                      : "MEDIA";
    }

    String name =
        preferences.getString(
            "sav_name",
            "");

    unsigned char encodedName[192] = {};
    size_t encodedNameLength = 0;

    if (name.length() > 0) {
      mbedtls_base64_encode(
          encodedName,
          sizeof(encodedName) - 1,
          &encodedNameLength,
          reinterpret_cast<
              const unsigned char *>(
              name.c_str()),
          name.length());

      encodedName[
          encodedNameLength] = 0;
    }

    size_t thumbBytes = 0;

    if (littleFsReady &&
        LittleFS.exists(
            SAVER_THUMB_PATH)) {
      File thumb =
          LittleFS.open(
              SAVER_THUMB_PATH,
              "r");

      if (thumb) {
        thumbBytes =
            thumb.size();

        thumb.close();
      }
    }

    char out[420] = {};

    snprintf(
        out,
        sizeof(out),
        "SAVMEDIA|STATE=READY|KIND=%s|FORMAT=%u|DEFAULT=0|NAME=%s|BYTES=%lu|W=%u|H=%u|FPS=%u|DUR=%lu|THUMB=%lu",
        kind.c_str(),
        static_cast<unsigned>(
            saverFormat),
        reinterpret_cast<
            const char *>(
            encodedName),
        static_cast<unsigned long>(
            saverDataBytes),
        static_cast<unsigned>(
            saverWidth),
        static_cast<unsigned>(
            saverHeight),
        static_cast<unsigned>(
            saverFormat == SAVER_PACKED
                ? packedFps
                : 0),
        static_cast<unsigned long>(
            saverFormat == SAVER_PACKED
                ? packedDurationMs
                : 0),
        static_cast<unsigned long>(
            thumbBytes));

    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("SAVMETA|")) {
    int first =
        command.indexOf('|');

    int second =
        command.indexOf(
            '|',
            first + 1);

    if (!saverReady ||
        first < 0 ||
        second < 0 ||
        !storeSaverMetadata(
            command.substring(
                first + 1,
                second),
            command.substring(
                second + 1))) {
      cdcPrintln(
          "ERR|SAVMETA");
      return;
    }

    cdcPrintln(
        "OK|SAVMETA");
    return;
  }

  if (upper.startsWith("SAVTHBEGIN|")) {
    int sep =
        command.indexOf('|');

    uint32_t byteCount = 0;

    if (!saverReady ||
        sep < 0 ||
        !parseUnsignedLong(
            command.substring(
                sep + 1),
            SAVER_THUMB_LIMIT_BYTES,
            byteCount) ||
        !beginSaverThumbUpload(
            byteCount)) {
      cdcPrintln(
          "ERR|SAVTHBEGIN");
      return;
    }

    cdcPrintln(
        "OK|SAVTHBEGIN");
    return;
  }

  if (upper.startsWith("SAVTHDATA|")) {
    int first =
        command.indexOf('|');

    int second =
        command.indexOf(
            '|',
            first + 1);

    uint32_t offset = 0;

    if (first < 0 ||
        second < 0 ||
        !parseUnsignedLong(
            command.substring(
                first + 1,
                second),
            saverThumbExpectedBytes,
            offset) ||
        !writeSaverThumbChunk(
            offset,
            command.substring(
                second + 1))) {
      cdcPrintln(
          "ERR|SAVTHDATA");
      return;
    }

    char out[40] = {};

    snprintf(
        out,
        sizeof(out),
        "OK|SAVTHDATA|%lu",
        static_cast<unsigned long>(
            saverThumbReceivedBytes));

    cdcPrintln(out);
    return;
  }

  if (upper == "SAVTHEND") {
    if (!finishSaverThumbUpload()) {
      cdcPrintln(
          "ERR|SAVTHEND");
      return;
    }

    cdcPrintln(
        "OK|SAVTHEND");
    return;
  }

  if (upper.startsWith("SAVTHREAD|")) {
    int first =
        command.indexOf('|');

    int second =
        command.indexOf(
            '|',
            first + 1);

    uint32_t offset = 0;
    uint16_t count = 0;

    if (!littleFsReady ||
        first < 0 ||
        second < 0 ||
        !parseUnsignedLong(
            command.substring(
                first + 1,
                second),
            SAVER_THUMB_LIMIT_BYTES,
            offset) ||
        !parseUnsigned(
            command.substring(
                second + 1),
            512,
            count) ||
        count == 0 ||
        !LittleFS.exists(
            SAVER_THUMB_PATH)) {
      cdcPrintln(
          "ERR|SAVTHREAD");
      return;
    }

    File thumb =
        LittleFS.open(
            SAVER_THUMB_PATH,
            "r");

    if (!thumb ||
        offset >=
            thumb.size() ||
        !thumb.seek(
            offset)) {
      if (thumb) {
        thumb.close();
      }

      cdcPrintln(
          "ERR|SAVTHREAD");
      return;
    }

    uint8_t raw[512] = {};

    size_t toRead =
        min(
            static_cast<size_t>(
                count),
            static_cast<size_t>(
                thumb.size() -
                offset));

    size_t got =
        thumb.read(
            raw,
            toRead);

    thumb.close();

    if (got == 0) {
      cdcPrintln(
          "ERR|SAVTHREAD");
      return;
    }

    unsigned char encoded[700] = {};
    size_t encodedLength = 0;

    if (mbedtls_base64_encode(
            encoded,
            sizeof(encoded) - 1,
            &encodedLength,
            raw,
            got) != 0) {
      cdcPrintln(
          "ERR|SAVTHREAD");
      return;
    }

    encoded[
        encodedLength] = 0;

    String response =
        "SAVTHDATA|" +
        String(
            static_cast<unsigned long>(
                offset)) +
        "|" +
        reinterpret_cast<
            const char *>(
            encoded);

    cdcPrintln(
        response);
    return;
  }

  if (upper.startsWith("SAVPXBEGIN|")) {
    int sep =
        command.indexOf('|');

    uint32_t byteCount = 0;

    if (sep < 0 ||
        !parseUnsignedLong(
            command.substring(
                sep + 1),
            PACKED_UPLOAD_LIMIT_BYTES,
            byteCount) ||
        byteCount < 26) {
      cdcPrintln(
          "ERR|BAD_SAVPXBEGIN");
      return;
    }

    if (!littleFsReady) {
      cdcPrintln(
          "ERR|FS_NOT_READY");
      return;
    }

    if (!beginPackedUpload(
            byteCount)) {
      size_t total =
          LittleFS.totalBytes();

      size_t used =
          LittleFS.usedBytes();

      size_t freeBytes =
          total > used
              ? total - used
              : 0;

      if (byteCount + 4096 >
          freeBytes) {
        char out[96];

        snprintf(
            out,
            sizeof(out),
            "ERR|NO_SPACE|FREE=%lu|NEED=%lu",
            static_cast<unsigned long>(
                freeBytes),
            static_cast<unsigned long>(
                byteCount + 4096));

        cdcPrintln(out);
      } else {
        cdcPrintln(
            "ERR|SAVPXBEGIN_ALLOC");
      }

      return;
    }

    cdcPrintln(
        "OK|SAVPXBEGIN");
    return;
  }

  if (upper.startsWith("SAVPXDATA|")) {
    int first =
        command.indexOf('|');

    int second =
        command.indexOf(
            '|',
            first + 1);

    if (first < 0 ||
        second < 0) {
      cdcPrintln(
          "ERR|BAD_SAVPXDATA");
      return;
    }

    uint32_t offset = 0;

    if (!parseUnsignedLong(
            command.substring(
                first + 1,
                second),
            packedUploadExpectedBytes,
            offset) ||
        !writePackedUploadChunk(
            offset,
            command.substring(
                second + 1))) {
      cdcPrintln(
          "ERR|SAVPXDATA");
      return;
    }

    char out[40];

    snprintf(
        out,
        sizeof(out),
        "OK|SAVPXDATA|%lu",
        static_cast<unsigned long>(
            saverBytesReceived));

    cdcPrintln(out);
    return;
  }

  if (upper == "SAVPXEND") {
    if (!finishPackedUpload()) {
      // Failed user media must not leave PIXEL PRO without a screensaver.
      clearSaverBuffer();
      cdcPrintln(
          "ERR|SAVPXEND");
      return;
    }

    lastUserActivityAt =
        millis();

    cdcPrintln(
        "OK|SAVER|READY");
    return;
  }

  if (upper.startsWith("SAVJPGBEGIN|")) {
    int first =
        command.indexOf('|');
    int second =
        command.indexOf(
            '|',
            first + 1);
    int third =
        command.indexOf(
            '|',
            second + 1);

    if (first < 0 ||
        second < 0 ||
        third < 0) {
      cdcPrintln(
          "ERR|BAD_SAVJPGBEGIN");
      return;
    }

    uint32_t byteCount = 0;
    uint16_t width = 0;
    uint16_t height = 0;

    if (!parseUnsignedLong(
            command.substring(
                first + 1,
                second),
            JPEG_UPLOAD_LIMIT_BYTES,
            byteCount) ||
        !parseUnsigned(
            command.substring(
                second + 1,
                third),
            TFT_WIDTH,
            width) ||
        !parseUnsigned(
            command.substring(
                third + 1),
            TFT_HEIGHT,
            height) ||
        width == 0 ||
        height == 0) {
      cdcPrintln(
          "ERR|BAD_SAVJPGBEGIN");
      return;
    }

    if (!beginJpegUpload(
            byteCount,
            width,
            height)) {
      cdcPrintln(
          "ERR|SAVJPGBEGIN_ALLOC");
      return;
    }

    cdcPrintln(
        "OK|SAVJPGBEGIN");
    return;
  }

  if (upper.startsWith("SAVJPGDATA|")) {
    int first =
        command.indexOf('|');
    int second =
        command.indexOf(
            '|',
            first + 1);

    if (first < 0 ||
        second < 0) {
      cdcPrintln(
          "ERR|BAD_SAVJPGDATA");
      return;
    }

    uint32_t offset = 0;

    if (!parseUnsignedLong(
            command.substring(
                first + 1,
                second),
            jpegUploadExpectedBytes,
            offset) ||
        !writeJpegUploadChunk(
            offset,
            command.substring(
                second + 1))) {
      cdcPrintln(
          "ERR|SAVJPGDATA");
      return;
    }

    char out[40];
    snprintf(
        out,
        sizeof(out),
        "OK|SAVJPGDATA|%lu",
        static_cast<unsigned long>(
            saverBytesReceived));

    cdcPrintln(out);
    return;
  }

  if (upper == "SAVJPGEND") {
    if (!finishJpegUpload()) {
      // Failed user media must not leave PIXEL PRO without a screensaver.
      clearSaverBuffer();
      cdcPrintln(
          "ERR|SAVJPGEND");
      return;
    }

    lastUserActivityAt =
        millis();

    cdcPrintln(
        "OK|SAVER|READY");
    return;
  }

  if (upper.startsWith("SAVGIFBEGIN|")) {
    int first = command.indexOf('|');
    int second = command.indexOf('|', first + 1);
    int third = command.indexOf('|', second + 1);
    int fourth = command.indexOf('|', third + 1);

    if (first < 0 ||
        second < 0 ||
        third < 0) {
      cdcPrintln("ERR|BAD_SAVGIFBEGIN");
      return;
    }

    uint32_t byteCount = 0;
    uint16_t width = 0;
    uint16_t height = 0;

    String heightPart =
        fourth >= 0
            ? command.substring(third + 1, fourth)
            : command.substring(third + 1);

    String scalePart =
        fourth >= 0
            ? command.substring(fourth + 1)
            : String("FILL");

    GifScaleMode scaleMode =
        parseGifScaleMode(scalePart);

    if (!parseUnsignedLong(
            command.substring(first + 1, second),
            GIF_UPLOAD_LIMIT_BYTES,
            byteCount) ||
        !parseUnsigned(
            command.substring(second + 1, third),
            1024,
            width) ||
        !parseUnsigned(
            heightPart,
            1024,
            height) ||
        !gifDimensionsSupported(
            width,
            height)) {
      cdcPrintln("ERR|BAD_SAVGIFBEGIN");
      return;
    }

    if (!littleFsReady) {
      cdcPrintln("ERR|FS_NOT_READY");
      return;
    }

    // beginGifUpload() deletes the previous user screensaver first, exactly
    // as requested by the product UX. Check capacity only after that cleanup,
    // otherwise replacing an existing large GIF falsely reports NO_SPACE.
    if (!beginGifUpload(
            byteCount,
            width,
            height,
            scaleMode)) {
      size_t total =
          LittleFS.totalBytes();

      size_t used =
          LittleFS.usedBytes();

      size_t freeBytes =
          total > used
              ? total - used
              : 0;

      if (static_cast<size_t>(byteCount) + 4096 > freeBytes) {
        char out[96];
        snprintf(
            out,
            sizeof(out),
            "ERR|NO_SPACE|NEED=%lu|FREE=%lu|TOTAL=%lu",
            static_cast<unsigned long>(byteCount + 4096),
            static_cast<unsigned long>(freeBytes),
            static_cast<unsigned long>(total));
        cdcPrintln(out);
      } else {
        cdcPrintln("ERR|SAVGIFBEGIN_ALLOC");
      }

      return;
    }

    cdcPrintln("OK|SAVGIFBEGIN");
    return;
  }

  if (upper.startsWith("SAVGIFDATA|")) {
    int first = command.indexOf('|');
    int second = command.indexOf('|', first + 1);

    if (first < 0 ||
        second < 0) {
      cdcPrintln("ERR|BAD_SAVGIFDATA");
      return;
    }

    uint32_t offset = 0;

    if (!parseUnsignedLong(
            command.substring(first + 1, second),
            gifUploadExpectedBytes,
            offset) ||
        !writeGifUploadChunk(
            offset,
            command.substring(second + 1))) {
      cdcPrintln("ERR|SAVGIFDATA");
      return;
    }

    char out[40];
    snprintf(
        out,
        sizeof(out),
        "OK|SAVGIFDATA|%lu",
        static_cast<unsigned long>(
            saverBytesReceived));

    cdcPrintln(out);
    return;
  }

  if (upper == "SAVGIFEND") {
    if (!finishGifUpload()) {
      // Failed user media must not leave PIXEL PRO without a screensaver.
      clearSaverBuffer();
      cdcPrintln(
          "ERR|SAVGIFEND");
      return;
    }

    lastUserActivityAt = millis();
    cdcPrintln("OK|SAVER|READY");
    return;
  }

  if (upper == "SAVCLEAR") {
    clearSaverBuffer();
    lastUserActivityAt = millis();

    if (displayReady) {
      renderMainMenu();
    }

    cdcPrintln("OK|SAVCLEAR");
    return;
  }

  if (upper == "SAVSHOW") {
    if (!saverReady) {
      cdcPrintln("ERR|SAVER_EMPTY");
      return;
    }

    startSaverNow();
    cdcPrintln("OK|SAVSHOW");
    return;
  }

  if (upper == "SAVSOURCE|MEDIA") {
    cdcPrintln("OK|SAVSOURCE|MEDIA");
    return;
  }

  if (upper.startsWith("SAVDELAY|")) {
    int sep = command.indexOf('|');
    uint32_t seconds = 0;

    if (sep < 0 ||
        !parseUnsignedLong(
            command.substring(sep + 1),
            86400,
            seconds)) {
      cdcPrintln("ERR|BAD_SAVDELAY");
      return;
    }

    saverDelayMs =
        seconds == 0
            ? 0
            : seconds * 1000UL;

    lastUserActivityAt = millis();
    cdcPrintln("OK|SAVDELAY");
    return;
  }

  if (upper.startsWith("SAVBEGIN|")) {
    int first = command.indexOf('|');
    int second = command.indexOf('|', first + 1);
    int third = command.indexOf('|', second + 1);
    int fourth = command.indexOf('|', third + 1);
    int fifth = command.indexOf('|', fourth + 1);

    if (first < 0 || second < 0 || third < 0 ||
        fourth < 0 || fifth < 0) {
      cdcPrintln("ERR|BAD_SAVBEGIN");
      return;
    }

    uint16_t frames = 0;
    uint16_t width = 0;
    uint16_t height = 0;

    if (!parseUnsigned(
            command.substring(first + 1, second),
            GIF_MAX_FRAMES,
            frames) ||
        frames == 0 ||
        !parseUnsigned(
            command.substring(second + 1, third),
            TFT_WIDTH,
            width) ||
        !parseUnsigned(
            command.substring(third + 1, fourth),
            TFT_HEIGHT,
            height)) {
      cdcPrintln("ERR|BAD_SAVBEGIN");
      return;
    }

    String formatText =
        command.substring(fourth + 1, fifth);
    formatText.toUpperCase();

    SaverPixelFormat format =
        formatText == "RGB565"
            ? SAVER_RGB565
            : formatText == "RGB332"
                ? SAVER_RGB332
                : SAVER_NONE;

    if (format == SAVER_NONE ||
        !beginSaverUpload(
            static_cast<uint8_t>(frames),
            width,
            height,
            format,
            command.substring(fifth + 1))) {
      cdcPrintln("ERR|SAVBEGIN_ALLOC");
      return;
    }

    cdcPrintln("OK|SAVBEGIN");
    return;
  }

  if (upper.startsWith("SAVDATA|")) {
    int first = command.indexOf('|');
    int second = command.indexOf('|', first + 1);
    int third = command.indexOf('|', second + 1);

    if (first < 0 || second < 0 || third < 0) {
      cdcPrintln("ERR|BAD_SAVDATA");
      return;
    }

    uint16_t frame = 0;
    uint32_t offset = 0;

    if (!parseUnsigned(
            command.substring(first + 1, second),
            GIF_MAX_FRAMES - 1,
            frame) ||
        !parseUnsignedLong(
            command.substring(second + 1, third),
            static_cast<uint32_t>(
                TFT_WIDTH * TFT_HEIGHT * 2),
            offset)) {
      cdcPrintln("ERR|BAD_SAVDATA");
      return;
    }

    if (!writeSaverChunk(
            static_cast<uint8_t>(frame),
            offset,
            command.substring(third + 1))) {
      cdcPrintln("ERR|SAVDATA_WRITE");
      return;
    }

    if (saverFrameBytes > 0 &&
        saverBytesReceived > 0 &&
        (saverBytesReceived % saverFrameBytes) == 0) {
      char out[32];
      snprintf(
          out,
          sizeof(out),
          "OK|SAVFRAME|%u",
          static_cast<unsigned>(frame));
      cdcPrintln(out);
    }

    return;
  }

  if (upper == "SAVEND") {
    if (!finishSaverUpload()) {
      // Failed user media must not leave PIXEL PRO without a screensaver.
      clearSaverBuffer();
      cdcPrintln(
          "ERR|SAVEND_INCOMPLETE");
      return;
    }

    lastUserActivityAt = millis();
    cdcPrintln("OK|SAVER|READY");
    return;
  }

  if (upper.startsWith("GET_KEYMAP")) {
    uint8_t profile = activeProfile;
    uint8_t layer = 0;

    int first = command.indexOf('|');
    if (first >= 0) {
      int second = command.indexOf('|', first + 1);

      uint16_t parsedProfile = 0;
      if (!parseUnsigned(
              second >= 0
                  ? command.substring(first + 1, second)
                  : command.substring(first + 1),
              PROFILE_COUNT - 1,
              parsedProfile)) {
        cdcPrintln("ERR|BAD_PROFILE");
        return;
      }

      profile = static_cast<uint8_t>(parsedProfile);

      if (second >= 0) {
        uint16_t parsedLayer = 0;
        if (!parseUnsigned(
                command.substring(second + 1),
                LAYER_COUNT - 1,
                parsedLayer)) {
          cdcPrintln("ERR|BAD_LAYER");
          return;
        }

        layer = static_cast<uint8_t>(parsedLayer);
      }
    }

    cdcPrintln(serializeKeymap(profile, layer));
    return;
  }

  if (upper.startsWith("SET_KEYMAP|")) {
    int first = command.indexOf('|');
    int second = command.indexOf('|', first + 1);
    int third = second >= 0 ? command.indexOf('|', second + 1) : -1;

    if (first < 0 || second < 0 || third < 0) {
      cdcPrintln("ERR|BAD_KEYMAP");
      return;
    }

    uint16_t parsedProfile = 0;
    uint16_t parsedLayer = 0;

    if (!parseUnsigned(
            command.substring(first + 1, second),
            PROFILE_COUNT - 1,
            parsedProfile)) {
      cdcPrintln("ERR|BAD_PROFILE");
      return;
    }

    if (!parseUnsigned(
            command.substring(second + 1, third),
            LAYER_COUNT - 1,
            parsedLayer)) {
      cdcPrintln("ERR|BAD_LAYER");
      return;
    }

    uint8_t profile = static_cast<uint8_t>(parsedProfile);
    uint8_t layer = static_cast<uint8_t>(parsedLayer);
    String payload = command.substring(third + 1);

    KeyBinding parsed[KEY_COUNT] = {};

    if (!parseKeymapPayload(payload, parsed)) {
      cdcPrintln("ERR|BAD_KEYMAP");
      return;
    }

    memcpy(keymap[profile][layer], parsed, sizeof(parsed));
    saveKeymap();
    sendMappedReports();

    char out[40];
    snprintf(
        out,
        sizeof(out),
        "OK|KEYMAP|%u|%u",
        static_cast<unsigned>(profile),
        static_cast<unsigned>(layer));
    cdcPrintln(out);
    return;
  }

  if (upper == "RESET_KEYMAP") {
    setDefaultKeymap();
    saveKeymap();
    activeProfile = 0;
    baseLayer = 0;
    (void)saveActiveProfileState();
    momentaryLayer = -1;
    toggledLayerMask = 0;
    sendMappedReports();
    cdcPrintln("OK|KEYMAP_RESET");
    return;
  }

  if (upper.startsWith("GET_MACRO|")) {
    uint16_t index = 0;
    int sep = command.indexOf('|');

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(sep + 1),
            MACRO_COUNT - 1,
            index)) {
      cdcPrintln("ERR|BAD_MACRO");
      return;
    }

    String out = "MACRO|";
    out += String(index);
    out += '|';
    out += hexEncode(macros[index]);
    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("SET_MACRO|")) {
    int first = command.indexOf('|');
    int second = command.indexOf('|', first + 1);

    if (first < 0 || second < 0) {
      cdcPrintln("ERR|BAD_MACRO");
      return;
    }

    uint16_t index = 0;
    if (!parseUnsigned(
            command.substring(first + 1, second),
            MACRO_COUNT - 1,
            index)) {
      cdcPrintln("ERR|BAD_MACRO");
      return;
    }

    String decoded;
    if (!hexDecode(command.substring(second + 1), decoded)) {
      cdcPrintln("ERR|BAD_MACRO_DATA");
      return;
    }

    macros[index] = decoded;
    saveMacro(static_cast<uint8_t>(index));

    String out = "OK|MACRO|";
    out += String(index);
    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("GET_RGB_PROFILE|")) {
    int sep =
        command.indexOf('|');

    uint16_t profile = 0;

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(
                sep + 1),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln(
          "ERR|BAD_RGB_PROFILE");
      return;
    }

    cdcPrintln(
        serializeRgbProfile(
            static_cast<uint8_t>(
                profile)));
    return;
  }

  if (upper.startsWith("RGB_KEY|")) {
    int p1 =
        command.indexOf('|');
    int p2 =
        command.indexOf('|', p1 + 1);
    int p3 =
        command.indexOf('|', p2 + 1);
    int p4 =
        command.indexOf('|', p3 + 1);
    int p5 =
        command.indexOf('|', p4 + 1);

    uint16_t profile = 0;
    uint16_t key = 0;
    uint16_t r = 0;
    uint16_t g = 0;
    uint16_t b = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        p3 < 0 ||
        p4 < 0 ||
        p5 < 0 ||
        !parseUnsigned(
            command.substring(
                p1 + 1,
                p2),
            PROFILE_COUNT - 1,
            profile) ||
        !parseUnsigned(
            command.substring(
                p2 + 1,
                p3),
            KEY_COUNT - 1,
            key) ||
        !parseUnsigned(
            command.substring(
                p3 + 1,
                p4),
            255,
            r) ||
        !parseUnsigned(
            command.substring(
                p4 + 1,
                p5),
            255,
            g) ||
        !parseUnsigned(
            command.substring(
                p5 + 1),
            255,
            b)) {
      cdcPrintln(
          "ERR|BAD_RGB_KEY");
      return;
    }

    rgbProfiles[profile][key][0] =
        static_cast<uint8_t>(r);
    rgbProfiles[profile][key][1] =
        static_cast<uint8_t>(g);
    rgbProfiles[profile][key][2] =
        static_cast<uint8_t>(b);

    saveRgbProfiles();

    if (profile ==
        activeProfile) {
      applyRgbProfile();
    }

    cdcPrintln(
        "OK|RGB_KEY");
    return;
  }

  if (upper.startsWith("RGB_ALL|")) {
    int p1 =
        command.indexOf('|');
    int p2 =
        command.indexOf('|', p1 + 1);
    int p3 =
        command.indexOf('|', p2 + 1);
    int p4 =
        command.indexOf('|', p3 + 1);

    uint16_t profile = 0;
    uint16_t r = 0;
    uint16_t g = 0;
    uint16_t b = 0;

    if (p1 < 0 ||
        p2 < 0 ||
        p3 < 0 ||
        p4 < 0 ||
        !parseUnsigned(
            command.substring(
                p1 + 1,
                p2),
            PROFILE_COUNT - 1,
            profile) ||
        !parseUnsigned(
            command.substring(
                p2 + 1,
                p3),
            255,
            r) ||
        !parseUnsigned(
            command.substring(
                p3 + 1,
                p4),
            255,
            g) ||
        !parseUnsigned(
            command.substring(
                p4 + 1),
            255,
            b)) {
      cdcPrintln(
          "ERR|BAD_RGB_ALL");
      return;
    }

    for (uint8_t key = 0;
         key < KEY_COUNT;
         ++key) {
      rgbProfiles[profile][key][0] =
          static_cast<uint8_t>(r);
      rgbProfiles[profile][key][1] =
          static_cast<uint8_t>(g);
      rgbProfiles[profile][key][2] =
          static_cast<uint8_t>(b);
    }

    saveRgbProfiles();

    if (profile ==
        activeProfile) {
      applyRgbProfile();
    }

    cdcPrintln(
        "OK|RGB_ALL");
    return;
  }

  if (upper.startsWith("RGB_PROFILE_SET|")) {
    int first =
        command.indexOf('|');
    int second =
        command.indexOf(
            '|',
            first + 1);

    uint16_t profile = 0;

    if (first < 0 ||
        second < 0 ||
        !parseUnsigned(
            command.substring(
                first + 1,
                second),
            PROFILE_COUNT - 1,
            profile)) {
      cdcPrintln(
          "ERR|BAD_RGB_PROFILE");
      return;
    }

    uint8_t colors[KEY_COUNT][3] = {};

    if (!parseRgbProfileCsv(
            command.substring(
                second + 1),
            colors)) {
      cdcPrintln(
          "ERR|BAD_RGB_PROFILE");
      return;
    }

    memcpy(
        rgbProfiles[profile],
        colors,
        sizeof(colors));

    saveRgbProfiles();

    if (profile ==
        activeProfile) {
      applyRgbProfile();
    }

    cdcPrintln(
        "OK|RGB_PROFILE");
    return;
  }

  if (upper.startsWith("RGB_EFFECT|")) {
    int first =
        command.indexOf('|');
    int second =
        command.indexOf(
            '|',
            first + 1);

    uint16_t profile = 0;
    uint16_t effect = 0;

    if (first < 0 ||
        second < 0 ||
        !parseUnsigned(
            command.substring(
                first + 1,
                second),
            PROFILE_COUNT - 1,
            profile) ||
        !parseUnsigned(
            command.substring(
                second + 1),
            9,
            effect)) {
      cdcPrintln(
          "ERR|BAD_RGB_EFFECT");
      return;
    }

    rgbEffects[profile] =
        static_cast<uint8_t>(
            effect);

    saveRgbProfiles();

    if (profile ==
        activeProfile) {
      applyRgbProfile();
    }

    cdcPrintln(
        "OK|RGB_EFFECT");
    return;
  }

  if (upper.startsWith("RGB_SPEED|")) {
    int sep =
        command.indexOf('|');

    uint16_t speed = 0;

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(
                sep + 1),
            100,
            speed) ||
        speed < 10) {
      cdcPrintln(
          "ERR|BAD_RGB_SPEED");
      return;
    }

    rgbSpeedPercent =
        static_cast<uint8_t>(
            speed);

    saveRgbProfiles();
    rgbLastFrameAt = 0;

    cdcPrintln(
        "OK|RGB_SPEED");
    return;
  }

  if (upper.startsWith("RGB_ENABLE|")) {
    int sep =
        command.indexOf('|');

    uint16_t enabled = 0;

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(
                sep + 1),
            1,
            enabled)) {
      cdcPrintln(
          "ERR|BAD_RGB_ENABLE");
      return;
    }

    rgbEnabled =
        enabled != 0;

    saveRgbProfiles();
    applyRgbProfile();

    cdcPrintln(
        "OK|RGB_ENABLE");
    return;
  }

  if (upper.startsWith("RGB_BRIGHTNESS|")) {
    int sep =
        command.indexOf('|');

    uint16_t brightness = 0;

    if (sep < 0 ||
        !parseUnsigned(
            command.substring(
                sep + 1),
            100,
            brightness)) {
      cdcPrintln(
          "ERR|BAD_RGB_BRIGHTNESS");
      return;
    }

    rgbBrightnessPercent =
        static_cast<uint8_t>(
            brightness);

    saveRgbProfiles();
    applyRgbProfile();

    cdcPrintln(
        "OK|RGB_BRIGHTNESS");
    return;
  }

  if (upper == "GET_PROFILE") {
    char out[48];
    snprintf(
        out,
        sizeof(out),
        "PROFILE|ACTIVE=%u|LAYER=%u",
        static_cast<unsigned>(activeProfile),
        static_cast<unsigned>(baseLayer));
    cdcPrintln(out);
    return;
  }

  if (upper.startsWith("SET_PROFILE|")) {
    int first = command.indexOf('|');
    int second = command.indexOf('|', first + 1);

    if (first < 0 || second < 0) {
      cdcPrintln("ERR|BAD_PROFILE");
      return;
    }

    uint16_t profile = 0;
    uint16_t layer = 0;

    if (!parseUnsigned(
            command.substring(first + 1, second),
            PROFILE_COUNT - 1,
            profile) ||
        !parseUnsigned(
            command.substring(second + 1),
            LAYER_COUNT - 1,
            layer)) {
      cdcPrintln("ERR|BAD_PROFILE");
      return;
    }

    bool profileChanged =
        activeProfile != static_cast<uint8_t>(profile);

    activeProfile = static_cast<uint8_t>(profile);
    baseLayer = static_cast<uint8_t>(layer);

    if (!saveActiveProfileState()) {
      cdcPrintln("ERR|PROFILE_SAVE");
      return;
    }

    momentaryLayer = -1;
    toggledLayerMask = 0;
    sendMappedReports();
    applyRgbProfile();

    if (profileChanged ||
        !saverActive) {
      requestMainMenuRender(
          activeProfile);
    }

    char out[40];
    snprintf(
        out,
        sizeof(out),
        "OK|PROFILE|%u|%u",
        static_cast<unsigned>(activeProfile),
        static_cast<unsigned>(baseLayer));
    cdcPrintln(out);
    return;
  }

  if (upper == "PING") {
    cdcPrintln("PONG|PIXELPRO");
    return;
  }

  if (upper == "ARM_BOOTLOADER") {
    // Normal CDC sessions have the Arduino-ESP32 DTR/RTS reboot hook disabled.
    // Only this explicit firmware-update command arms it, and only briefly.
    bootloaderArmed = true;
    bootloaderArmUntil =
        millis() + 5000UL;
    USBSerial.enableReboot(true);
    cdcPrintln("OK|BOOTLOADER_ARMED");
    USBSerial.flush();
    return;
  }

  if (upper == "REBOOT") {
    cdcPrintln("OK|REBOOT");
    USBSerial.flush();
    delay(50);
    ESP.restart();
    return;
  }

  if (upper.length()) {
    cdcPrintln("ERR|UNKNOWN_COMMAND");
  }
}

static void pollCdc() {
  if (rawMediaKind !=
      RAW_MEDIA_NONE) {
    pollRawMediaTransfer();

    if (rawMediaKind !=
        RAW_MEDIA_NONE) {
      return;
    }
  }

  while (USBSerial.available()) {
    if (rawMediaKind !=
        RAW_MEDIA_NONE) {
      pollRawMediaTransfer();

      if (rawMediaKind !=
          RAW_MEDIA_NONE) {
        return;
      }

      continue;
    }

    char ch =
        static_cast<char>(
            USBSerial.read());

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      if (cdcLine.length()) {
        handleCommand(cdcLine);
        cdcLine = "";
      }
      continue;
    }

    if (cdcLine.length() < 2048) {
      cdcLine += ch;
    } else {
      cdcLine = "";
      cdcPrintln(
          "ERR|LINE_TOO_LONG");
    }
  }
}


static uint8_t moduleCrc8(
    const uint8_t *data,
    size_t length) {
  uint8_t crc = 0;

  for (size_t i = 0;
       i < length;
       ++i) {
    crc ^= data[i];

    for (uint8_t bit = 0;
         bit < 8;
         ++bit) {
      crc =
          (crc & 0x80U) != 0
              ? static_cast<uint8_t>(
                    (crc << 1) ^ 0x07U)
              : static_cast<uint8_t>(
                    crc << 1);
    }
  }

  return crc;
}

static bool moduleMuxSelect(
    uint8_t port) {
  if (port >= MODULE_PORT_COUNT) {
    return false;
  }

  Wire.beginTransmission(
      MODULE_MUX_ADDRESS);

  Wire.write(
      static_cast<uint8_t>(
          1U << port));

  return Wire.endTransmission(true) == 0;
}

static bool moduleMuxDisable() {
  Wire.beginTransmission(
      MODULE_MUX_ADDRESS);

  Wire.write(
      static_cast<uint8_t>(0));

  return Wire.endTransmission(true) == 0;
}

static bool moduleProbeSelectedPort() {
  Wire.beginTransmission(
      MODULE_DEVICE_ADDRESS);

  return Wire.endTransmission(true) == 0;
}

static bool moduleReadSelectedFrame(
    ModuleFrame &frame) {
  uint8_t *bytes =
      reinterpret_cast<uint8_t *>(
          &frame);

  const uint8_t received =
      Wire.requestFrom(
          MODULE_DEVICE_ADDRESS,
          MODULE_FRAME_SIZE);

  if (received !=
      MODULE_FRAME_SIZE) {
    while (Wire.available()) {
      (void)Wire.read();
    }

    return false;
  }

  for (uint8_t i = 0;
       i < MODULE_FRAME_SIZE;
       ++i) {
    if (!Wire.available()) {
      return false;
    }

    bytes[i] =
        static_cast<uint8_t>(
            Wire.read());
  }

  if (frame.magic !=
          MODULE_FRAME_MAGIC ||
      frame.version !=
          MODULE_PROTOCOL_VERSION) {
    return false;
  }

  const uint8_t expected =
      moduleCrc8(
          bytes,
          MODULE_FRAME_SIZE - 1);

  return frame.crc == expected;
}

static void emitModuleConnected(
    uint8_t port) {
  char out[48];

  snprintf(
      out,
      sizeof(out),
      "MODULE|PORT=%u|CONNECTED",
      static_cast<unsigned>(
          port + 1));

  cdcPrintln(
      out);
}

static void emitModuleDisconnected(
    uint8_t port) {
  char out[52];

  snprintf(
      out,
      sizeof(out),
      "MODULE|PORT=%u|DISCONNECTED",
      static_cast<unsigned>(
          port + 1));

  cdcPrintln(
      out);
}

static void emitModuleFrame(
    uint8_t port,
    const ModuleFrame &frame) {
  char out[196];

  snprintf(
      out,
      sizeof(out),
      "MODULE_DATA|PORT=%u|TYPE=%u|ID=%u|SEQ=%u|BTN=%04X|E1=%d|E2=%d|S1=%u|S2=%u|FLAGS=%02X",
      static_cast<unsigned>(
          port + 1),
      static_cast<unsigned>(
          frame.type),
      static_cast<unsigned>(
          frame.moduleId),
      static_cast<unsigned>(
          frame.sequence),
      static_cast<unsigned>(
          frame.buttons),
      static_cast<int>(
          frame.encoder1),
      static_cast<int>(
          frame.encoder2),
      static_cast<unsigned>(
          frame.slider1),
      static_cast<unsigned>(
          frame.slider2),
      static_cast<unsigned>(
          frame.flags));

  cdcPrintln(
      out);
}

static void clearModulePort(
    uint8_t port,
    bool emitDisconnect) {
  if (port >= MODULE_PORT_COUNT) {
    return;
  }

  ModulePortState &state =
      modulePorts[port];

  const bool wasConnected =
      state.connected;

  state = {};

  if (emitDisconnect &&
      wasConnected) {
    emitModuleDisconnected(
        port);
  }
}

static void pollModulePort(
    uint8_t port) {
  if (port >= MODULE_PORT_COUNT) {
    return;
  }

  ModulePortState &state =
      modulePorts[port];

  if (!moduleMuxReady) {
    clearModulePort(
        port,
        true);
    return;
  }

  if (!moduleMuxSelect(port)) {
    moduleMuxReady = false;
    clearModulePort(
        port,
        true);
    return;
  }

  const bool present =
      moduleProbeSelectedPort();

  if (!present) {
    (void)moduleMuxDisable();

    if (state.missCount <
        0xFF) {
      ++state.missCount;
    }

    if (state.connected &&
        state.missCount >=
            MODULE_MISS_LIMIT) {
      clearModulePort(
          port,
          true);
    }

    return;
  }

  state.missCount = 0;

  if (!state.connected) {
    state.connected = true;
    state.hasFrame = false;
    emitModuleConnected(
        port);
  }

  ModuleFrame frame = {};

  const bool validFrame =
      moduleReadSelectedFrame(
          frame);

  (void)moduleMuxDisable();

  if (!validFrame) {
    return;
  }

  const bool newFrame =
      !state.hasFrame ||
      frame.sequence !=
          state.lastSequence;

  state.frame = frame;
  state.lastSequence =
      frame.sequence;
  state.hasFrame = true;

  if (newFrame) {
    emitModuleFrame(
        port,
        frame);
  }
}

static void initModuleBus() {
  Wire.begin(
      MODULE_SDA_PIN,
      MODULE_SCL_PIN,
      MODULE_I2C_HZ);

  Wire.setTimeOut(5);

  moduleMuxReady =
      moduleMuxDisable();

  moduleNextPort = 0;
  moduleLastPollAt =
      millis();

  for (uint8_t port = 0;
       port < MODULE_PORT_COUNT;
       ++port) {
    modulePorts[port] = {};
  }
}

static void pollModuleBus() {
  const uint32_t now =
      millis();

  if (now -
          moduleLastPollAt <
      MODULE_POLL_INTERVAL_MS) {
    return;
  }

  moduleLastPollAt = now;

  if (!moduleMuxReady) {
    static uint32_t lastMuxRetryAt = 0;

    if (now -
            lastMuxRetryAt >=
        1000UL) {
      lastMuxRetryAt = now;
      moduleMuxReady =
          moduleMuxDisable();
    }

    return;
  }

  pollModulePort(
      moduleNextPort);

  moduleNextPort =
      static_cast<uint8_t>(
          (moduleNextPort + 1U) %
          MODULE_PORT_COUNT);
}

static void sendModuleSummary() {
  char out[196];

  snprintf(
      out,
      sizeof(out),
      "MODULES|MUX=%u|PORTS=%u",
      moduleMuxReady ? 1U : 0U,
      static_cast<unsigned>(
          MODULE_PORT_COUNT));

  cdcPrintln(
      out);

  for (uint8_t port = 0;
       port < MODULE_PORT_COUNT;
       ++port) {
    const ModulePortState &state =
        modulePorts[port];

    if (!state.connected) {
      snprintf(
          out,
          sizeof(out),
          "MODULE_PORT|%u|CONNECTED=0",
          static_cast<unsigned>(
              port + 1));

      cdcPrintln(
          out);
      continue;
    }

    if (!state.hasFrame) {
      snprintf(
          out,
          sizeof(out),
          "MODULE_PORT|%u|CONNECTED=1|FRAME=0",
          static_cast<unsigned>(
              port + 1));

      cdcPrintln(
          out);
      continue;
    }

    const ModuleFrame &frame =
        state.frame;

    snprintf(
        out,
        sizeof(out),
        "MODULE_PORT|%u|CONNECTED=1|FRAME=1|TYPE=%u|ID=%u|SEQ=%u|BTN=%04X|E1=%d|E2=%d|S1=%u|S2=%u|FLAGS=%02X",
        static_cast<unsigned>(
            port + 1),
        static_cast<unsigned>(
            frame.type),
        static_cast<unsigned>(
            frame.moduleId),
        static_cast<unsigned>(
            frame.sequence),
        static_cast<unsigned>(
            frame.buttons),
        static_cast<int>(
            frame.encoder1),
        static_cast<int>(
            frame.encoder2),
        static_cast<unsigned>(
            frame.slider1),
        static_cast<unsigned>(
            frame.slider2),
        static_cast<unsigned>(
            frame.flags));

    cdcPrintln(
        out);
  }
}

static bool moduleSendRaw(
    uint8_t port,
    const uint8_t *data,
    uint8_t length) {
  if (!moduleMuxReady ||
      port >= MODULE_PORT_COUNT ||
      data == nullptr ||
      length == 0 ||
      length > MODULE_MAX_TX_BYTES) {
    return false;
  }

  if (!moduleMuxSelect(port)) {
    moduleMuxReady = false;
    return false;
  }

  Wire.beginTransmission(
      MODULE_DEVICE_ADDRESS);

  const size_t written =
      Wire.write(
          data,
          length);

  const uint8_t result =
      Wire.endTransmission(true);

  (void)moduleMuxDisable();

  return written == length &&
         result == 0;
}

static bool parseModuleHex(
    const String &hex,
    uint8_t *data,
    uint8_t &length) {
  if (data == nullptr ||
      hex.length() == 0 ||
      (hex.length() % 2U) != 0 ||
      hex.length() >
          static_cast<size_t>(
              MODULE_MAX_TX_BYTES * 2U)) {
    return false;
  }

  length =
      static_cast<uint8_t>(
          hex.length() / 2U);

  for (uint8_t i = 0;
       i < length;
       ++i) {
    const int8_t hi =
        hexNibble(
            hex[i * 2U]);

    const int8_t lo =
        hexNibble(
            hex[i * 2U + 1U]);

    if (hi < 0 ||
        lo < 0) {
      return false;
    }

    data[i] =
        static_cast<uint8_t>(
            (hi << 4) | lo);
  }

  return true;
}

static void emitKeyEvent(uint8_t index, bool pressed) {
  if (pressed) {
    lastUserActivityAt = millis();
    stopSaver();

    uint8_t layer = currentLayer();
    KeyBinding resolved = resolveBinding(layer, index);
    activeBindings[index] = resolved;
    pressedMask |= static_cast<uint8_t>(1U << index);

    if (resolved.type == BIND_LAYER) {
      applyLayerPress(resolved);
    } else if (resolved.type == BIND_MACRO) {
      char macroOut[64];
      snprintf(
          macroOut,
          sizeof(macroOut),
          "MACRO|%u|KEY=%u|P=%u|L=%u",
          static_cast<unsigned>(resolved.keyCode + 1),
          static_cast<unsigned>(index + 1),
          static_cast<unsigned>(activeProfile),
          static_cast<unsigned>(layer));
      cdcPrintln(macroOut);
    } else if (resolved.type == BIND_ACTION) {
      char actionOut[64];
      snprintf(
          actionOut,
          sizeof(actionOut),
          "ACTION|%u|KEY=%u|P=%u|L=%u",
          static_cast<unsigned>(resolved.keyCode),
          static_cast<unsigned>(index + 1),
          static_cast<unsigned>(activeProfile),
          static_cast<unsigned>(layer));
      cdcPrintln(actionOut);
    }
  } else {
    applyLayerRelease(activeBindings[index]);
    pressedMask &= static_cast<uint8_t>(~(1U << index));
    activeBindings[index] = disabledBinding();
  }

  sendMappedReports();

  char out[48];
  snprintf(
      out,
      sizeof(out),
      "KEY|%u|%s|P=%u|L=%u",
      index + 1,
      pressed ? "DOWN" : "UP",
      static_cast<unsigned>(activeProfile),
      static_cast<unsigned>(currentLayer()));
  cdcPrintln(out);
}

static uint8_t readMatrixMask() {
  uint8_t mask = 0;

  for (uint8_t row = 0;
       row < MATRIX_ROW_COUNT;
       ++row) {
    const uint8_t rowPin =
        MATRIX_ROW_PINS[row];

    pinMode(
        rowPin,
        OUTPUT);
    digitalWrite(
        rowPin,
        LOW);

    // 2 us is ample for the short hand-wired matrix and keeps scan latency low.
    delayMicroseconds(2);

    for (uint8_t col = 0;
         col < MATRIX_COL_COUNT;
         ++col) {
      if (digitalRead(
              MATRIX_COL_PINS[col]) == LOW) {
        const uint8_t index =
            row * MATRIX_COL_COUNT +
            col;

        mask |=
            static_cast<uint8_t>(
                1U << index);
      }
    }

    // Unselected rows are high impedance. With diode cathodes toward ROW this
    // avoids cross-row current while the column pull-ups remain enabled.
    pinMode(
        rowPin,
        INPUT);
  }

  return mask;
}

static void initKeys() {
  for (uint8_t row = 0;
       row < MATRIX_ROW_COUNT;
       ++row) {
    pinMode(
        MATRIX_ROW_PINS[row],
        INPUT);
  }

  for (uint8_t col = 0;
       col < MATRIX_COL_COUNT;
       ++col) {
    pinMode(
        MATRIX_COL_PINS[col],
        INPUT_PULLUP);
  }

  delayMicroseconds(20);

  const uint8_t mask =
      readMatrixMask();

  pressedMask = 0;

  for (uint8_t i = 0;
       i < KEY_COUNT;
       ++i) {
    const bool pressed =
        (mask &
         static_cast<uint8_t>(
             1U << i)) != 0;

    keyState[i].rawPressed = pressed;
    keyState[i].stablePressed = pressed;
    keyState[i].changedAt = millis();
    activeBindings[i] = disabledBinding();

    if (pressed) {
      pressedMask |=
          static_cast<uint8_t>(
              1U << i);
    }
  }
}

static void sendConsumerTap(
    uint16_t usage) {
  if (!HID.ready() ||
      usage == 0) {
    return;
  }

  // Preserve a consumer key that may already be held by a physical key.
  const uint16_t heldUsage =
      activeConsumerCode;

  if (heldUsage != 0) {
    ConsumerControl.release();
  }

  ConsumerControl.press(
      usage);
  delay(2);
  ConsumerControl.release();

  if (heldUsage != 0) {
    ConsumerControl.press(
        heldUsage);
  }
}

static void emitRollerStep(
    bool clockwise) {
  lastUserActivityAt = millis();
  stopSaver();

  sendConsumerTap(
      clockwise
          ? ROLLER_CW_CONSUMER
          : ROLLER_CCW_CONSUMER);

  cdcPrintln(
      clockwise
          ? "ENCODER|CW"
          : "ENCODER|CCW");
}

static void emitRollerSwitchPress() {
  lastUserActivityAt = millis();
  stopSaver();

  sendConsumerTap(
      ROLLER_SW_CONSUMER);

  cdcPrintln(
      "ENCODER|PRESS");
}

static void initRoller() {
  pinMode(
      ROLLER_A_PIN,
      INPUT_PULLUP);
  pinMode(
      ROLLER_B_PIN,
      INPUT_PULLUP);
  pinMode(
      ROLLER_SW_PIN,
      INPUT_PULLUP);

  rollerLastAB =
      static_cast<uint8_t>(
          (digitalRead(ROLLER_A_PIN) == HIGH
               ? 2U
               : 0U) |
          (digitalRead(ROLLER_B_PIN) == HIGH
               ? 1U
               : 0U));

  rollerTransitionAccumulator = 0;

  const bool switchPressed =
      digitalRead(
          ROLLER_SW_PIN) == LOW;

  rollerSwitchState.rawPressed =
      switchPressed;
  rollerSwitchState.stablePressed =
      switchPressed;
  rollerSwitchState.changedAt =
      millis();
}

static void pollRoller() {
  // Gray-code transition table. Invalid two-bit jumps are ignored, which
  // removes most mechanical bounce without delaying the main loop.
  static const int8_t TRANSITION[16] = {
      0, -1, 1, 0,
      1, 0, 0, -1,
      -1, 0, 0, 1,
      0, 1, -1, 0};

  const uint8_t currentAB =
      static_cast<uint8_t>(
          (digitalRead(ROLLER_A_PIN) == HIGH
               ? 2U
               : 0U) |
          (digitalRead(ROLLER_B_PIN) == HIGH
               ? 1U
               : 0U));

  const uint8_t transitionIndex =
      static_cast<uint8_t>(
          (rollerLastAB << 2) |
          currentAB);

  rollerLastAB =
      currentAB;

  rollerTransitionAccumulator +=
      TRANSITION[transitionIndex];

  if (rollerTransitionAccumulator >=
      ROLLER_TRANSITIONS_PER_DETENT) {
    rollerTransitionAccumulator = 0;
    emitRollerStep(true);
  } else if (rollerTransitionAccumulator <=
             -ROLLER_TRANSITIONS_PER_DETENT) {
    rollerTransitionAccumulator = 0;
    emitRollerStep(false);
  }

  const uint32_t now =
      millis();

  const bool switchPressed =
      digitalRead(
          ROLLER_SW_PIN) == LOW;

  if (switchPressed !=
      rollerSwitchState.rawPressed) {
    rollerSwitchState.rawPressed =
        switchPressed;
    rollerSwitchState.changedAt =
        now;
  }

  if (switchPressed !=
          rollerSwitchState.stablePressed &&
      now -
              rollerSwitchState.changedAt >=
          DEBOUNCE_MS) {
    rollerSwitchState.stablePressed =
        switchPressed;

    if (switchPressed) {
      emitRollerSwitchPress();
    }
  }
}

static void restoreTouchSharedPins() {
  // Return every shared touch/LCD line to the output state expected by the
  // 8080 bus. CS stays high while idle; Arduino_GFX asserts it as required.
  pinMode(
      TOUCH_YP_PIN,
      OUTPUT);
  digitalWrite(
      TOUCH_YP_PIN,
      HIGH);

  pinMode(
      TOUCH_XM_PIN,
      OUTPUT);
  digitalWrite(
      TOUCH_XM_PIN,
      HIGH);

  pinMode(
      TOUCH_XP_PIN,
      OUTPUT);
  digitalWrite(
      TOUCH_XP_PIN,
      LOW);

  pinMode(
      TOUCH_YM_PIN,
      OUTPUT);
  digitalWrite(
      TOUCH_YM_PIN,
      LOW);

  pinMode(
      TFT_CS,
      OUTPUT);
  digitalWrite(
      TFT_CS,
      HIGH);

  // Give the 8080 bus a few microseconds to settle after the ADC pins have
  // been switched back to push-pull outputs. This prevents the next LCD write
  // from seeing a transient WR/DC/data state.
  delayMicroseconds(4);
}

static uint16_t readTouchAdc10(
    int8_t pin) {
  // ADC is configured to 10-bit in initTouch(). Discard the first sample
  // after re-wiring the resistive network, then average three stable samples.
  (void)analogRead(pin);

  uint32_t sum = 0;

  for (uint8_t i = 0;
       i < 3;
       ++i) {
    sum +=
        static_cast<uint16_t>(
            analogRead(pin));
    delayMicroseconds(8);
  }

  return static_cast<uint16_t>(
      sum / 3U);
}

static void sampleTouchCoordinates(
    uint16_t &rawX,
    uint16_t &rawY) {
  // Deselect LCD while the shared resistive-touch lines are sampled.
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

  // TouchScreen.h X read:
  // YP/YM Hi-Z, XP=HIGH, XM=LOW, sample YP.
  digitalWrite(TOUCH_YP_PIN, LOW);
  digitalWrite(TOUCH_YM_PIN, LOW);
  pinMode(TOUCH_YP_PIN, INPUT);
  pinMode(TOUCH_YM_PIN, INPUT);
  pinMode(TOUCH_XP_PIN, OUTPUT);
  pinMode(TOUCH_XM_PIN, OUTPUT);
  digitalWrite(TOUCH_XP_PIN, HIGH);
  digitalWrite(TOUCH_XM_PIN, LOW);
  delayMicroseconds(28);

  rawX =
      static_cast<uint16_t>(
          TOUCH_ADC_MAX -
          readTouchAdc10(
              TOUCH_YP_PIN));

  // TouchScreen.h Y read:
  // XP/XM Hi-Z, YP=HIGH, YM=LOW, sample XM.
  digitalWrite(TOUCH_XP_PIN, LOW);
  digitalWrite(TOUCH_XM_PIN, LOW);
  pinMode(TOUCH_XP_PIN, INPUT);
  pinMode(TOUCH_XM_PIN, INPUT);
  pinMode(TOUCH_YP_PIN, OUTPUT);
  pinMode(TOUCH_YM_PIN, OUTPUT);
  digitalWrite(TOUCH_YP_PIN, HIGH);
  digitalWrite(TOUCH_YM_PIN, LOW);
  delayMicroseconds(28);

  rawY =
      static_cast<uint16_t>(
          TOUCH_ADC_MAX -
          readTouchAdc10(
              TOUCH_XM_PIN));
}

static uint16_t touchDelta(
    uint16_t a,
    uint16_t b) {
  return a >= b
      ? static_cast<uint16_t>(a - b)
      : static_cast<uint16_t>(b - a);
}

static uint16_t touchMedian3(
    uint16_t a,
    uint16_t b,
    uint16_t c) {
  if (a > b) {
    const uint16_t t = a;
    a = b;
    b = t;
  }

  if (b > c) {
    const uint16_t t = b;
    b = c;
    c = t;
  }

  if (a > b) {
    const uint16_t t = a;
    a = b;
    b = t;
  }

  return b;
}

static void sampleTouchContactNodes(
    uint16_t &z1,
    uint16_t &z2) {
  pinMode(TOUCH_XP_PIN, OUTPUT);
  digitalWrite(TOUCH_XP_PIN, LOW);
  pinMode(TOUCH_YM_PIN, OUTPUT);
  digitalWrite(TOUCH_YM_PIN, HIGH);

  digitalWrite(TOUCH_XM_PIN, LOW);
  digitalWrite(TOUCH_YP_PIN, LOW);
  pinMode(TOUCH_XM_PIN, INPUT);
  pinMode(TOUCH_YP_PIN, INPUT);

  delayMicroseconds(32);

  z1 =
      readTouchAdc10(
          TOUCH_XM_PIN);

  z2 =
      readTouchAdc10(
          TOUCH_YP_PIN);
}

static bool touchContactNodesValid(
    uint16_t z1,
    uint16_t z2) {
  const bool z1Inside =
      z1 >=
          TOUCH_CONTACT_RAIL_MARGIN &&
      z1 <=
          TOUCH_ADC_MAX -
              TOUCH_CONTACT_RAIL_MARGIN;

  const bool z2Inside =
      z2 >=
          TOUCH_CONTACT_RAIL_MARGIN &&
      z2 <=
          TOUCH_ADC_MAX -
              TOUCH_CONTACT_RAIL_MARGIN;

  return z1Inside &&
         z2Inside &&
         touchDelta(
             z1,
             z2) >=
             TOUCH_CONTACT_DELTA_MIN;
}

static bool readTouchRaw(
    uint16_t &rawX,
    uint16_t &rawY,
    uint16_t &pressure) {
  if (!displayReady) {
    return false;
  }

  // Qualify the resistive divider before doing X/Y acquisition. While idle
  // this keeps the shared TFT pins in touch mode for the shortest possible
  // time and stops coordinate noise from being interpreted as a press.
  uint16_t z1a = 0;
  uint16_t z2a = 0;
  uint16_t z1b = 0;
  uint16_t z2b = 0;

  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

  sampleTouchContactNodes(
      z1a,
      z2a);

  delayMicroseconds(70);

  sampleTouchContactNodes(
      z1b,
      z2b);

  const bool contactStable =
      touchContactNodesValid(
          z1a,
          z2a) &&
      touchContactNodesValid(
          z1b,
          z2b) &&
      touchDelta(
          z1a,
          z1b) <=
          TOUCH_CONTACT_STABILITY_MAX &&
      touchDelta(
          z2a,
          z2b) <=
          TOUCH_CONTACT_STABILITY_MAX;

  if (!contactStable) {
    restoreTouchSharedPins();
    pressure = 0;
    rawX = 0;
    rawY = 0;
    return false;
  }

  uint16_t x1 = 0;
  uint16_t y1 = 0;
  uint16_t x2 = 0;
  uint16_t y2 = 0;
  uint16_t x3 = 0;
  uint16_t y3 = 0;

  sampleTouchCoordinates(
      x1,
      y1);

  delayMicroseconds(70);

  sampleTouchCoordinates(
      x2,
      y2);

  delayMicroseconds(70);

  sampleTouchCoordinates(
      x3,
      y3);

  rawX =
      touchMedian3(
          x1,
          x2,
          x3);

  rawY =
      touchMedian3(
          y1,
          y2,
          y3);

  const uint16_t minX =
      min(
          x1,
          min(
              x2,
              x3));

  const uint16_t maxX =
      max(
          x1,
          max(
              x2,
              x3));

  const uint16_t minY =
      min(
          y1,
          min(
              y2,
              y3));

  const uint16_t maxY =
      max(
          y1,
          max(
              y2,
              y3));

  // Verify the divider again after X/Y sampling. A bus transient can satisfy
  // one pressure read, but a real finger remains present for the whole sample.
  uint16_t z1c = 0;
  uint16_t z2c = 0;

  sampleTouchContactNodes(
      z1c,
      z2c);

  const bool contactStillPresent =
      touchContactNodesValid(
          z1c,
          z2c) &&
      touchDelta(
          z1b,
          z1c) <=
          TOUCH_CONTACT_STABILITY_MAX &&
      touchDelta(
          z2b,
          z2c) <=
          TOUCH_CONTACT_STABILITY_MAX;

  pressure = 0;

  if (contactStillPresent) {
    const uint16_t zLow =
        min(
            z1c,
            z2c);

    const uint16_t zDelta =
        touchDelta(
            z1c,
            z2c);

    if (zLow > 0 &&
        zDelta > 0) {
      uint64_t rtouch =
          static_cast<uint64_t>(
              zDelta) *
          rawX *
          TOUCH_RXPLATE_OHMS;

      rtouch /= zLow;
      rtouch /= 1024U;

      pressure =
          static_cast<uint16_t>(
              rtouch > 65535U
                  ? 65535U
                  : rtouch);
    }
  }

  restoreTouchSharedPins();

  const bool axesInsidePanel =
      rawX >= 24 &&
      rawX <= TOUCH_ADC_MAX - 24 &&
      rawY >= 24 &&
      rawY <= TOUCH_ADC_MAX - 24;

  const bool stableCoordinates =
      static_cast<uint16_t>(
          maxX - minX) <=
          TOUCH_SAMPLE_STABILITY_MAX &&
      static_cast<uint16_t>(
          maxY - minY) <=
          TOUCH_SAMPLE_STABILITY_MAX;

  return contactStillPresent &&
         axesInsidePanel &&
         stableCoordinates;
}


static bool readTouchPointMcufriend(
    uint16_t &rawX,
    uint16_t &rawY,
    uint16_t &pressure) {
  if (!displayReady) {
    return false;
  }

  pinMode(
      TFT_CS,
      OUTPUT);
  digitalWrite(
      TFT_CS,
      HIGH);

  uint16_t x1 = 0;
  uint16_t y1 = 0;
  uint16_t x2 = 0;
  uint16_t y2 = 0;
  uint16_t x3 = 0;
  uint16_t y3 = 0;

  sampleTouchCoordinates(
      x1,
      y1);
  sampleTouchCoordinates(
      x2,
      y2);
  sampleTouchCoordinates(
      x3,
      y3);

  rawX =
      touchMedian3(
          x1,
          x2,
          x3);

  rawY =
      touchMedian3(
          y1,
          y2,
          y3);

  // Match TouchScreen.cpp pressure topology: XP=LOW, YM=HIGH, read XM/YP.
  pinMode(
      TOUCH_XP_PIN,
      OUTPUT);
  digitalWrite(
      TOUCH_XP_PIN,
      LOW);

  pinMode(
      TOUCH_YM_PIN,
      OUTPUT);
  digitalWrite(
      TOUCH_YM_PIN,
      HIGH);

  digitalWrite(
      TOUCH_XM_PIN,
      LOW);
  pinMode(
      TOUCH_XM_PIN,
      INPUT);

  digitalWrite(
      TOUCH_YP_PIN,
      LOW);
  pinMode(
      TOUCH_YP_PIN,
      INPUT);

  delayMicroseconds(20);

  const uint16_t z1 =
      readTouchAdc10(
          TOUCH_XM_PIN);

  const uint16_t z2 =
      readTouchAdc10(
          TOUCH_YP_PIN);

  uint32_t rtouch = 0;

  if (z1 > 0 &&
      z2 > z1) {
    // Adafruit TouchScreen.cpp with _rxplate=300:
    // ((z2 / z1) - 1) * x * rxplate / 1024.
    const uint64_t numerator =
        static_cast<uint64_t>(
            z2 - z1) *
        rawX *
        TOUCH_RXPLATE_OHMS;

    rtouch =
        static_cast<uint32_t>(
            numerator /
            z1 /
            1024U);
  }

  pressure =
      static_cast<uint16_t>(
          rtouch > 65535UL
              ? 65535UL
              : rtouch);

  restoreTouchSharedPins();

  const bool coordinatesValid =
      rawX >= 8 &&
      rawX <= TOUCH_ADC_MAX - 8 &&
      rawY >= 8 &&
      rawY <= TOUCH_ADC_MAX - 8;

  // MCUFRIEND examples use this window with a 300-ohm 4-wire panel.
  const bool pressureValid =
      pressure > 200 &&
      pressure < 1000;

  return coordinatesValid &&
         pressureValid;
}

static void mapTouchDefaultPixels(
    uint16_t rawX,
    uint16_t rawY,
    int16_t &screenX,
    int16_t &screenY) {
  // Known-working MCUFRIEND Orientation=1 mapping for this exact 480x320
  // shield: screen X follows tp.y reversed, screen Y follows tp.x.
  long mappedX =
      map(
          static_cast<long>(
              rawY),
          static_cast<long>(
              TOUCH_Y_MAX_DEFAULT),
          static_cast<long>(
              TOUCH_Y_MIN_DEFAULT),
          0L,
          static_cast<long>(
              TFT_WIDTH - 1));

  long mappedY =
      map(
          static_cast<long>(
              rawX),
          static_cast<long>(
              TOUCH_X_MIN_DEFAULT),
          static_cast<long>(
              TOUCH_X_MAX_DEFAULT),
          0L,
          static_cast<long>(
              TFT_HEIGHT - 1));

  screenX =
      static_cast<int16_t>(
          constrain(
              mappedX,
              0L,
              static_cast<long>(
                  TFT_WIDTH - 1)));

  screenY =
      static_cast<int16_t>(
          constrain(
              mappedY,
              0L,
              static_cast<long>(
                  TFT_HEIGHT - 1)));
}

static uint16_t normalizeTouchAxis(
    uint16_t raw,
    uint16_t minimum,
    uint16_t maximum) {
  if (raw <= minimum) {
    return 0;
  }

  if (raw >= maximum) {
    return 65535;
  }

  return static_cast<uint16_t>(
      (static_cast<uint32_t>(
           raw - minimum) *
       65535UL) /
      static_cast<uint32_t>(
          maximum - minimum));
}

static void mapTouchCoordinates(
    uint16_t rawX,
    uint16_t rawY,
    int16_t &screenX,
    int16_t &screenY) {
  if (touchAffineValid) {
    const float mappedX =
        touchAffine.ax *
            static_cast<float>(
                rawX) +
        touchAffine.bx *
            static_cast<float>(
                rawY) +
        touchAffine.cx;

    const float mappedY =
        touchAffine.ay *
            static_cast<float>(
                rawX) +
        touchAffine.by *
            static_cast<float>(
                rawY) +
        touchAffine.cy;

    screenX =
        static_cast<int16_t>(
            constrain(
                static_cast<int>(
                    lroundf(
                        mappedX)),
                0,
                static_cast<int>(
                    TFT_WIDTH - 1)));

    screenY =
        static_cast<int16_t>(
            constrain(
                static_cast<int>(
                    lroundf(
                        mappedY)),
                0,
                static_cast<int>(
                    TFT_HEIGHT - 1)));

    return;
  }

  // Legacy fallback only. New installs/calibration use the full affine
  // transform above, which also corrects panel skew and axis cross-coupling.
  uint16_t nx =
      normalizeTouchAxis(
          rawX,
          touchCalibration.xMin,
          touchCalibration.xMax);

  uint16_t ny =
      normalizeTouchAxis(
          rawY,
          touchCalibration.yMin,
          touchCalibration.yMax);

  if ((touchCalibration.flags &
       TOUCH_FLAG_SWAP_XY) != 0) {
    const uint16_t temp =
        nx;
    nx = ny;
    ny = temp;
  }

  if ((touchCalibration.flags &
       TOUCH_FLAG_INVERT_X) != 0) {
    nx =
        static_cast<uint16_t>(
            65535U - nx);
  }

  if ((touchCalibration.flags &
       TOUCH_FLAG_INVERT_Y) != 0) {
    ny =
        static_cast<uint16_t>(
            65535U - ny);
  }

  screenX =
      static_cast<int16_t>(
          (static_cast<uint32_t>(nx) *
           (TFT_WIDTH - 1U)) /
          65535UL);

  screenY =
      static_cast<int16_t>(
          (static_cast<uint32_t>(ny) *
           (TFT_HEIGHT - 1U)) /
          65535UL);
}

static int8_t mainMenuSlotAt(
    int16_t x,
    int16_t y) {
  const int statusY =
      TFT_HEIGHT -
      MENU_STATUS_HEIGHT;

  const int marginX = 10;
  const int marginY = 6;
  const int gapX = 6;
  const int gapY = 4;

  const int cellW =
      (TFT_WIDTH -
       marginX * 2 -
       gapX * 3) /
      4;

  const int cellH =
      (statusY -
       marginY * 2 -
       gapY) /
      2;

  for (uint8_t slot = 0;
       slot < MENU_SLOT_COUNT;
       ++slot) {
    const int col =
        slot % 4;

    const int row =
        slot / 4;

    const int cellX =
        marginX +
        col *
            (cellW + gapX);

    const int cellY =
        marginY +
        row *
            (cellH + gapY);

    const int left =
        cellX +
        (cellW -
         MENU_ICON_WIDTH) /
            2;

    const int top =
        cellY +
        (cellH -
         MENU_ICON_HEIGHT) /
            2;

    if (x >= left &&
        x < left + MENU_ICON_WIDTH &&
        y >= top &&
        y < top + MENU_ICON_HEIGHT) {
      return static_cast<int8_t>(
          slot);
    }
  }

  return -1;
}

static void drawTouchSlotFeedback(
    int8_t slot) {
  if (!displayReady ||
      slot < 0 ||
      slot >= MENU_SLOT_COUNT) {
    return;
  }

  const int statusY =
      TFT_HEIGHT -
      MENU_STATUS_HEIGHT;

  const int marginX = 10;
  const int marginY = 6;
  const int gapX = 6;
  const int gapY = 4;

  const int cellW =
      (TFT_WIDTH -
       marginX * 2 -
       gapX * 3) /
      4;

  const int cellH =
      (statusY -
       marginY * 2 -
       gapY) /
      2;

  const int col =
      slot % 4;

  const int row =
      slot / 4;

  const int cellX =
      marginX +
      col *
          (cellW + gapX);

  const int cellY =
      marginY +
      row *
          (cellH + gapY);

  const int x =
      cellX +
      (cellW -
       MENU_ICON_WIDTH) /
          2;

  const int y =
      cellY +
      (cellH -
       MENU_ICON_HEIGHT) /
          2;

  tft->drawRect(
      x,
      y,
      MENU_ICON_WIDTH,
      MENU_ICON_HEIGHT,
      0xFFFF);

  tft->drawRect(
      x + 1,
      y + 1,
      MENU_ICON_WIDTH - 2,
      MENU_ICON_HEIGHT - 2,
      0xFFFF);

  touchFeedbackSlot =
      slot;
}

static void drawTouchCalibrationTarget() {
  if (!displayReady ||
      !touchCalibrationMode ||
      touchCalibrationPoint >= 4) {
    return;
  }

  static const int16_t targetX[4] = {
      40,
      TFT_WIDTH - 41,
      TFT_WIDTH - 41,
      40
  };

  static const int16_t targetY[4] = {
      40,
      40,
      TFT_HEIGHT - 41,
      TFT_HEIGHT - 41
  };

  const int16_t x =
      targetX[touchCalibrationPoint];

  const int16_t y =
      targetY[touchCalibrationPoint];

  tft->fillScreen(
      RGB565_BLACK);

  tft->setTextSize(2);
  tft->setTextColor(
      0xFFFF);

  tft->setCursor(
      118,
      12);

  tft->print(
      "TOUCH CALIBRATION");

  char step[32] = {};
  snprintf(
      step,
      sizeof(step),
      "Touch target %u / 4",
      static_cast<unsigned>(
          touchCalibrationPoint + 1));

  tft->setTextSize(1);
  tft->setCursor(
      184,
      38);
  tft->print(
      step);

  tft->drawCircle(
      x,
      y,
      14,
      0xFFFF);
  tft->drawCircle(
      x,
      y,
      15,
      0xFFFF);
  tft->drawLine(
      x - 24,
      y,
      x + 24,
      y,
      0xFFFF);
  tft->drawLine(
      x,
      y - 24,
      x,
      y + 24,
      0xFFFF);
}

static void drawTouchPixelTestScreen() {
  if (!displayReady ||
      !touchPixelTestMode) {
    return;
  }

  tft->fillScreen(
      RGB565_BLACK);

  tft->drawRect(
      0,
      0,
      TFT_WIDTH,
      TFT_HEIGHT,
      0xFFFF);

  for (int x = 80;
       x < TFT_WIDTH;
       x += 80) {
    tft->drawLine(
        x,
        0,
        x,
        TFT_HEIGHT - 1,
        0x2104);
  }

  for (int y = 80;
       y < TFT_HEIGHT;
       y += 80) {
    tft->drawLine(
        0,
        y,
        TFT_WIDTH - 1,
        y,
        0x2104);
  }

  tft->drawLine(
      TFT_WIDTH / 2,
      0,
      TFT_WIDTH / 2,
      TFT_HEIGHT - 1,
      0x39E7);

  tft->drawLine(
      0,
      TFT_HEIGHT / 2,
      TFT_WIDTH - 1,
      TFT_HEIGHT / 2,
      0x39E7);

  tft->setTextSize(2);
  tft->setTextColor(
      0xFFFF);
  tft->setCursor(
      12,
      10);
  tft->print(
      "TOUCH PIXEL TEST 480x320");

  tft->setTextSize(1);
  tft->setCursor(
      12,
      34);
  tft->print(
      "Touch anywhere - green crosshair is mapped pixel");
}

static void drawTouchPixelPoint(
    int16_t x,
    int16_t y,
    uint16_t rawX,
    uint16_t rawY) {
  if (!displayReady ||
      !touchPixelTestMode) {
    return;
  }

  drawTouchPixelTestScreen();

  const int left =
      max(
          0,
          static_cast<int>(x) - 16);

  const int right =
      min(
          static_cast<int>(
              TFT_WIDTH - 1),
          static_cast<int>(x) + 16);

  const int top =
      max(
          0,
          static_cast<int>(y) - 16);

  const int bottom =
      min(
          static_cast<int>(
              TFT_HEIGHT - 1),
          static_cast<int>(y) + 16);

  tft->drawCircle(
      x,
      y,
      9,
      0x07E0);

  tft->drawLine(
      left,
      y,
      right,
      y,
      0x07E0);

  tft->drawLine(
      x,
      top,
      x,
      bottom,
      0x07E0);

  char line[88] = {};
  snprintf(
      line,
      sizeof(line),
      "PIXEL X=%d Y=%d   RAW X=%u Y=%u",
      static_cast<int>(x),
      static_cast<int>(y),
      static_cast<unsigned>(rawX),
      static_cast<unsigned>(rawY));

  tft->fillRect(
      8,
      TFT_HEIGHT - 24,
      TFT_WIDTH - 16,
      18,
      RGB565_BLACK);

  tft->setTextSize(1);
  tft->setTextColor(
      0xFFFF);
  tft->setCursor(
      12,
      TFT_HEIGHT - 20);
  tft->print(
      line);
}

static bool finishAutomaticTouchCalibration() {
  // Targets are TL, TR, BR, BL at (40,40), (439,40), (439,279), (40,279).
  // Model the raw panel as a 2D affine plane instead of assuming that raw X/Y
  // are perfectly orthogonal. This corrects rotation, swapped axes and shear.
  const float r0x =
      static_cast<float>(
          touchCalibrationRawX[0]);
  const float r0y =
      static_cast<float>(
          touchCalibrationRawY[0]);
  const float r1x =
      static_cast<float>(
          touchCalibrationRawX[1]);
  const float r1y =
      static_cast<float>(
          touchCalibrationRawY[1]);
  const float r2x =
      static_cast<float>(
          touchCalibrationRawX[2]);
  const float r2y =
      static_cast<float>(
          touchCalibrationRawY[2]);
  const float r3x =
      static_cast<float>(
          touchCalibrationRawX[3]);
  const float r3y =
      static_cast<float>(
          touchCalibrationRawY[3]);

  const float centerX =
      (r0x + r1x + r2x + r3x) *
      0.25f;
  const float centerY =
      (r0y + r1y + r2y + r3y) *
      0.25f;

  // Raw half-axis corresponding to screen horizontal movement.
  const float ux =
      (r1x + r2x - r0x - r3x) *
      0.25f;
  const float uy =
      (r1y + r2y - r0y - r3y) *
      0.25f;

  // Raw half-axis corresponding to screen vertical movement.
  const float vx =
      (r2x + r3x - r0x - r1x) *
      0.25f;
  const float vy =
      (r2y + r3y - r0y - r1y) *
      0.25f;

  const float determinant =
      ux * vy -
      uy * vx;

  if (!isfinite(
          determinant) ||
      fabsf(
          determinant) <
          1500.0f) {
    return false;
  }

  static constexpr float SCREEN_CENTER_X =
      (40.0f +
       static_cast<float>(
           TFT_WIDTH - 41)) *
      0.5f;

  static constexpr float SCREEN_CENTER_Y =
      (40.0f +
       static_cast<float>(
           TFT_HEIGHT - 41)) *
      0.5f;

  static constexpr float SCREEN_HALF_X =
      (static_cast<float>(
           TFT_WIDTH - 41) -
       40.0f) *
      0.5f;

  static constexpr float SCREEN_HALF_Y =
      (static_cast<float>(
           TFT_HEIGHT - 41) -
       40.0f) *
      0.5f;

  TouchAffineCalibration affine = {};

  affine.ax =
      SCREEN_HALF_X *
      vy /
      determinant;

  affine.bx =
      -SCREEN_HALF_X *
      vx /
      determinant;

  affine.cx =
      SCREEN_CENTER_X -
      affine.ax *
          centerX -
      affine.bx *
          centerY;

  affine.ay =
      -SCREEN_HALF_Y *
      uy /
      determinant;

  affine.by =
      SCREEN_HALF_Y *
      ux /
      determinant;

  affine.cy =
      SCREEN_CENTER_Y -
      affine.ay *
          centerX -
      affine.by *
          centerY;

  if (!touchAffineIsValid(
          affine)) {
    return false;
  }

  static const float targetX[4] = {
      40.0f,
      static_cast<float>(
          TFT_WIDTH - 41),
      static_cast<float>(
          TFT_WIDTH - 41),
      40.0f};

  static const float targetY[4] = {
      40.0f,
      40.0f,
      static_cast<float>(
          TFT_HEIGHT - 41),
      static_cast<float>(
          TFT_HEIGHT - 41)};

  float worstError = 0.0f;

  for (uint8_t i = 0;
       i < 4;
       ++i) {
    const float mappedX =
        affine.ax *
            touchCalibrationRawX[i] +
        affine.bx *
            touchCalibrationRawY[i] +
        affine.cx;

    const float mappedY =
        affine.ay *
            touchCalibrationRawX[i] +
        affine.by *
            touchCalibrationRawY[i] +
        affine.cy;

    const float dx =
        mappedX -
        targetX[i];

    const float dy =
        mappedY -
        targetY[i];

    const float error =
        sqrtf(
            dx * dx +
            dy * dy);

    worstError =
        max(
            worstError,
            error);
  }

  // If the four raw points cannot be represented reasonably by one affine
  // plane, the user probably missed a target or a false press was captured.
  if (!isfinite(
          worstError) ||
      worstError >
          55.0f) {
    return false;
  }

  touchAffine =
      affine;
  touchAffineValid =
      true;
  touchCalibrationRequired =
      false;

  // Keep legacy min/max populated for diagnostics/older commands. They are
  // no longer used for normal mapping when affine calibration is valid.
  uint16_t minX =
      touchCalibrationRawX[0];
  uint16_t maxX =
      touchCalibrationRawX[0];
  uint16_t minY =
      touchCalibrationRawY[0];
  uint16_t maxY =
      touchCalibrationRawY[0];

  for (uint8_t i = 1;
       i < 4;
       ++i) {
    minX =
        min(
            minX,
            touchCalibrationRawX[i]);
    maxX =
        max(
            maxX,
            touchCalibrationRawX[i]);
    minY =
        min(
            minY,
            touchCalibrationRawY[i]);
    maxY =
        max(
            maxY,
            touchCalibrationRawY[i]);
  }

  touchCalibration.xMin =
      minX;
  touchCalibration.xMax =
      maxX;
  touchCalibration.yMin =
      minY;
  touchCalibration.yMax =
      maxY;
  touchCalibration.flags =
      0;

  saveTouchCalibration();

  return true;
}

static void startAutomaticTouchCalibration() {
  touchPixelTestMode = false;
  touchCalibrationMode = true;
  touchCalibrationPoint = 0;
  touchCalibrationPointCaptured = false;

  memset(
      touchCalibrationRawX,
      0,
      sizeof(
          touchCalibrationRawX));

  memset(
      touchCalibrationRawY,
      0,
      sizeof(
          touchCalibrationRawY));

  touchRawPressed = false;
  touchStablePressed = false;
  touchPressConfirmations = 0;
  touchReleaseMisses = 0;
  touchCandidateRawX = 0;
  touchCandidateRawY = 0;
  touchPressStartedAt = 0;
  touchPendingSlot = -1;
  touchChangedAt = millis();
  lastUserActivityAt = touchChangedAt;
  touchWakeOnly = false;
  touchHeldFallbackSlot = -1;
  touchFeedbackSlot = -1;

  stopSaver();
  drawTouchCalibrationTarget();
}

static void cancelAutomaticTouchCalibration() {
  touchCalibrationMode = false;
  touchCalibrationPoint = 0;
  touchCalibrationPointCaptured = false;
  touchRawPressed = false;
  touchStablePressed = false;
  touchPressConfirmations = 0;
  touchReleaseMisses = 0;
  touchCandidateRawX = 0;
  touchCandidateRawY = 0;
  touchPressStartedAt = 0;
  touchPendingSlot = -1;
  touchChangedAt = millis();
  lastUserActivityAt = touchChangedAt;
  renderMainMenu();
}

static bool emitTouchAction(
    uint8_t slot) {
  if (slot >= MENU_SLOT_COUNT) {
    return false;
  }

  const uint8_t profile =
      menuRenderedProfile <
              PROFILE_COUNT
          ? menuRenderedProfile
          : (activeProfile <
                     PROFILE_COUNT
                 ? activeProfile
                 : 0);

  const uint8_t action =
      effectiveMainMenuAction(
          profile,
          slot);

  if (action == 0 ||
      action > ACTION_COUNT) {
    return false;
  }

  char out[80];
  snprintf(
      out,
      sizeof(out),
      "ACTION|%u|KEY=0|P=%u|L=%u|TOUCH=%u",
      static_cast<unsigned>(action),
      static_cast<unsigned>(profile),
      static_cast<unsigned>(currentLayer()),
      static_cast<unsigned>(slot + 1));

  cdcPrintln(out);
  return true;
}

static void pressTouchFallbackKey(
    uint8_t slot) {
  if (slot >= KEY_COUNT ||
      touchHeldFallbackSlot >= 0) {
    return;
  }

  touchHeldFallbackSlot =
      static_cast<int8_t>(
          slot);

  emitKeyEvent(
      slot,
      true);

  char out[64] = {};
  snprintf(
      out,
      sizeof(out),
      "TOUCH|FALLBACK|DOWN|KEY=%u",
      static_cast<unsigned>(
          slot + 1));
  cdcPrintln(out);
}

static void releaseTouchFallbackKey() {
  if (touchHeldFallbackSlot < 0) {
    return;
  }

  const uint8_t slot =
      static_cast<uint8_t>(
          touchHeldFallbackSlot);

  touchHeldFallbackSlot = -1;

  emitKeyEvent(
      slot,
      false);

  char out[64] = {};
  snprintf(
      out,
      sizeof(out),
      "TOUCH|FALLBACK|UP|KEY=%u",
      static_cast<unsigned>(
          slot + 1));
  cdcPrintln(out);
}

static void tapTouchFallbackKey(
    uint8_t slot) {
  pressTouchFallbackKey(
      slot);

  if (touchHeldFallbackSlot >= 0) {
    delay(8);
    releaseTouchFallbackKey();
  }
}

static void initTouch() {
  analogReadResolution(10);
  restoreTouchSharedPins();

  touchRawPressed = false;
  touchStablePressed = false;
  touchPressConfirmations = 0;
  touchReleaseMisses = 0;
  touchCandidateRawX = 0;
  touchCandidateRawY = 0;
  touchPressStartedAt = 0;
  touchPendingSlot = -1;
  touchWakeOnly = false;
  touchHeldFallbackSlot = -1;
  touchFeedbackSlot = -1;
  touchChangedAt = millis();
  touchLastPollAt = 0;
}

static void pollTouch() {
  if (!displayReady) {
    return;
  }

  const uint32_t now =
      millis();

  if (now -
          touchLastPollAt <
      TOUCH_POLL_MS) {
    return;
  }

  touchLastPollAt =
      now;

  uint16_t rawX = 0;
  uint16_t rawY = 0;
  uint16_t pressure = 0;

  const bool pressed =
      readTouchPointMcufriend(
          rawX,
          rawY,
          pressure);

  touchRawPressed =
      pressed;

  if (pressed) {
    touchReleaseMisses = 0;

    if (!touchStablePressed) {
      // A single electrically-plausible sample is not enough. Require three
      // consecutive polls whose median coordinates stay close together.
      // This is ~60 ms at the normal poll rate: fast enough for a tap, but
      // long enough to reject TFT-bus transients and floating ADC ghosts.
      if (touchPressConfirmations == 0) {
        touchPressConfirmations = 1;
      } else if (
          touchDelta(
              rawX,
              touchCandidateRawX) <=
              TOUCH_CONFIRM_MOVE_MAX &&
          touchDelta(
              rawY,
              touchCandidateRawY) <=
              TOUCH_CONFIRM_MOVE_MAX) {
        if (touchPressConfirmations <
            TOUCH_PRESS_CONFIRM_COUNT) {
          touchPressConfirmations++;
        }
      } else {
        touchPressConfirmations = 1;
      }

      touchCandidateRawX = rawX;
      touchCandidateRawY = rawY;

      if (touchPressConfirmations <
          TOUCH_PRESS_CONFIRM_COUNT) {
        return;
      }

      touchPressConfirmations = 0;
    }

    touchRawX = rawX;
    touchRawY = rawY;
    touchPressure = pressure;

    if (touchPixelTestMode) {
      mapTouchDefaultPixels(
          rawX,
          rawY,
          touchX,
          touchY);
    } else if (!touchCalibrationMode) {
      mapTouchCoordinates(
          rawX,
          rawY,
          touchX,
          touchY);
    }

    if (touchStablePressed) {
      if (!touchCalibrationMode &&
          !touchWakeOnly &&
          touchPendingSlot >= 0) {
        const int8_t currentSlot =
            mainMenuSlotAt(
                touchX,
                touchY);

        if (currentSlot !=
            touchPendingSlot) {
          touchPendingSlot = -1;
        }
      }

      return;
    }

    touchStablePressed = true;
    touchChangedAt = now;
    touchPressStartedAt = now;
    touchPendingSlot = -1;

    if (touchPixelTestMode) {
      lastUserActivityAt = now;

      drawTouchPixelPoint(
          touchX,
          touchY,
          touchRawX,
          touchRawY);

      char testOut[112] = {};
      snprintf(
          testOut,
          sizeof(testOut),
          "TOUCH_TEST|RAWX=%u|RAWY=%u|X=%d|Y=%d|W=%u|H=%u",
          static_cast<unsigned>(
              touchRawX),
          static_cast<unsigned>(
              touchRawY),
          static_cast<int>(
              touchX),
          static_cast<int>(
              touchY),
          static_cast<unsigned>(
              TFT_WIDTH),
          static_cast<unsigned>(
              TFT_HEIGHT));

      cdcPrintln(
          testOut);
      return;
    }

    if (touchCalibrationMode) {
      lastUserActivityAt = now;

      if (touchCalibrationPoint < 4 &&
          !touchCalibrationPointCaptured) {
        touchCalibrationRawX[
            touchCalibrationPoint] =
            touchRawX;

        touchCalibrationRawY[
            touchCalibrationPoint] =
            touchRawY;

        touchCalibrationPointCaptured =
            true;

        char pointOut[96] = {};
        snprintf(
            pointOut,
            sizeof(pointOut),
            "TOUCH_CAL_AUTO|CAPTURE=%u|RAWX=%u|RAWY=%u",
            static_cast<unsigned>(
                touchCalibrationPoint + 1),
            static_cast<unsigned>(
                touchRawX),
            static_cast<unsigned>(
                touchRawY));
        cdcPrintln(
            pointOut);
      }

      // Keep showing the current target until this finger is released.
      // Advancement happens only in the UP path below.
      return;
    }

    lastUserActivityAt =
        now;

    const bool wasSaverActive =
        saverActive;

    if (wasSaverActive) {
      touchWakeOnly = true;
      stopSaver();
      return;
    }

    touchWakeOnly = false;

    const int8_t slot =
        mainMenuSlotAt(
            touchX,
            touchY);

    touchPendingSlot =
        slot;

    char out[112] = {};
    snprintf(
        out,
        sizeof(out),
        "TOUCH|DOWN|X=%d|Y=%d|RAWX=%u|RAWY=%u|P=%u|SLOT=%d",
        static_cast<int>(
            touchX),
        static_cast<int>(
            touchY),
        static_cast<unsigned>(
            touchRawX),
        static_cast<unsigned>(
            touchRawY),
        static_cast<unsigned>(
            touchPressure),
        slot >= 0
            ? static_cast<int>(
                  slot + 1)
            : 0);

    cdcPrintln(
        out);

    // Do not dispatch or redraw on DOWN. A ghost sample that slips through
    // the electrical filter must also survive a complete, coherent tap before
    // it can trigger an action.
    return;
  }

  // A failed contact sample also cancels an unconfirmed DOWN candidate.
  // Once DOWN is latched, tolerate a few misses before UP so one noisy ADC
  // read cannot turn a held finger into repeated taps.
  touchPressConfirmations = 0;
  touchCandidateRawX = 0;
  touchCandidateRawY = 0;

  if (!touchStablePressed) {
    touchReleaseMisses = 0;
    return;
  }

  if (touchReleaseMisses <
      TOUCH_RELEASE_MISS_COUNT) {
    touchReleaseMisses++;
  }

  if (touchReleaseMisses <
      TOUCH_RELEASE_MISS_COUNT) {
    return;
  }

  touchReleaseMisses = 0;
  touchStablePressed = false;
  touchChangedAt = now;

  const uint32_t pressDuration =
      now -
      touchPressStartedAt;

  const int8_t releasedSlot =
      mainMenuSlotAt(
          touchX,
          touchY);

  const int8_t pendingSlot =
      touchPendingSlot;

  touchPressStartedAt = 0;
  touchPendingSlot = -1;

  if (touchPixelTestMode) {
    lastUserActivityAt = now;
    return;
  }

  // Calibration advances only after a complete press/release cycle. This
  // guarantees one physical touch can capture exactly one target.
  if (touchCalibrationMode) {
    lastUserActivityAt = now;

    if (!touchCalibrationPointCaptured) {
      return;
    }

    touchCalibrationPointCaptured = false;

    if (touchCalibrationPoint < 4) {
      touchCalibrationPoint++;
    }

    if (touchCalibrationPoint >= 4) {
      const bool ok =
          finishAutomaticTouchCalibration();

      touchCalibrationMode = false;
      touchCalibrationPoint = 0;
      lastUserActivityAt = now;

      if (ok) {
        char done[192] = {};
        snprintf(
            done,
            sizeof(done),
            "TOUCH_CAL_AUTO|DONE|AFFINE|%.6f|%.6f|%.3f|%.6f|%.6f|%.3f",
            static_cast<double>(
                touchAffine.ax),
            static_cast<double>(
                touchAffine.bx),
            static_cast<double>(
                touchAffine.cx),
            static_cast<double>(
                touchAffine.ay),
            static_cast<double>(
                touchAffine.by),
            static_cast<double>(
                touchAffine.cy));
        cdcPrintln(
            done);
      } else {
        cdcPrintln(
            "TOUCH_CAL_AUTO|ERR|BAD_GEOMETRY");
      }

      renderMainMenu();
    } else {
      drawTouchCalibrationTarget();
    }

    return;
  }

  if (!touchWakeOnly) {
    const bool validTap =
        pendingSlot >= 0 &&
        releasedSlot ==
            pendingSlot &&
        pressDuration >=
            TOUCH_TAP_MIN_MS &&
        pressDuration <=
            TOUCH_TAP_MAX_MS;

    if (validTap) {
      const uint8_t touchedSlot =
          static_cast<uint8_t>(
              pendingSlot);

      if (!emitTouchAction(
              touchedSlot)) {
        tapTouchFallbackKey(
            touchedSlot);
      }
    }

    cdcPrintln(
        validTap
            ? "TOUCH|UP|TAP=1"
            : "TOUCH|UP|TAP=0");
  }

  touchWakeOnly = false;
}

// D15 now carries WS2812/SK6812 data. The WEMOS onboard LED remains
// electrically attached through its 2 kOhm resistor, but firmware no longer
// drives it as a separate status indicator.

static void pollKeys() {
  const uint32_t now =
      millis();

  const uint8_t mask =
      readMatrixMask();

  for (uint8_t i = 0;
       i < KEY_COUNT;
       ++i) {
    const bool pressed =
        (mask &
         static_cast<uint8_t>(
             1U << i)) != 0;

    if (pressed !=
        keyState[i].rawPressed) {
      keyState[i].rawPressed =
          pressed;
      keyState[i].changedAt =
          now;
    }

    if (pressed !=
            keyState[i].stablePressed &&
        now -
                keyState[i].changedAt >=
            DEBOUNCE_MS) {
      keyState[i].stablePressed =
          pressed;
      emitKeyEvent(
          i,
          pressed);
    }
  }
}

void setup() {
  bootResetReason =
      esp_reset_reason();
  bootSequence++;

  preferences.begin("pixelpro", false);
  loadKeymap();
  loadMacros();
  loadRgbProfiles();
  loadMainMenuConfig();
  loadActiveProfileState();

  menuLastContentProfile =
      preferences.getUChar(
          "menulast",
          activeProfile);

  if (menuLastContentProfile >=
      PROFILE_COUNT) {
    menuLastContentProfile =
        activeProfile;
  }

  menuRenderedProfile =
      activeProfile;

  menuCompositeMask =
      preferences.getUInt(
          "menucomp",
          0);

  loadTouchCalibration();

  menuHostOs =
      preferences.getUChar(
          "host_os",
          1);

  if (menuHostOs < 1 ||
      menuHostOs > 3) {
    menuHostOs = 1;
  }

  rgbStrip.begin();
  rgbStrip.clear();
  rgbStrip.show();

  initKeys();
  initRoller();
  initDisplay();

#if PIXEL_DIAG_POST_INIT_STAGE
  auto showStage =
      [](uint16_t color,
         const char *label) {
        if (!displayReady) {
          delay(5000);
          return;
        }

        tft->fillScreen(color);
        tft->setTextSize(3);

        const uint16_t textColor =
            color == 0x0000
                ? 0xFFFF
                : 0x0000;

        tft->setTextColor(textColor);
        tft->setCursor(20, 24);
        tft->print(label);
      };

  auto holdStage =
      [](uint32_t durationMs,
         bool pollKeysCdc,
         bool pollRgb,
         bool pollModules,
         bool pollSaverRuntime) {
        const uint32_t start =
            millis();

        while (static_cast<uint32_t>(
                   millis() - start) <
               durationMs) {
          if (pollKeysCdc) {
            pollKeys();
            pollRoller();
            pollCdc();
          }

          if (pollRgb) {
            pollRgbEffect();
          }

          if (pollModules) {
            pollModuleBus();
          }

          if (pollSaverRuntime) {
            pollSaver();
          }

          delay(1);
        }
      };

  // Stage 0: exact display-only state that has already tested bright/stable.
  showStage(0xFFFF, "0 DISPLAY ONLY");
  holdStage(5000, false, false, false, false);

  // Stage 1: add SD/SPI init only.
  mountSdCard();
  showStage(0xFFE0, "1 SD / SPI");
  holdStage(5000, false, false, false, false);

  // Stage 2: add I2C module-bus init only.
  initModuleBus();
  showStage(0x07FF, "2 I2C INIT");
  holdStage(5000, false, false, false, false);

  // Stage 3: mount LittleFS and load saver metadata/media only.
  littleFsReady = mountPersistentStorage(true);
  if (littleFsReady) {
    if (!loadMainMenuConfigFile()) {
      (void)saveMainMenuConfig();
    }

    (void)recoverMainMenuActionsFromKeymap();
    recoverMainMenuAssets();
    loadPersistedMedia();
  } else {
    saverReady = false;
    saverActive = false;
    saverFormat = SAVER_NONE;
  }

  showStage(0x07E0, "3 FLASH / MEDIA");
  holdStage(5000, false, false, false, false);

  // Stage 4: render the real Main Menu once, then do no runtime polling.
  renderMainMenu();
  if (displayReady) {
    tft->fillRect(0, 0, 220, 34, 0xFFFF);
    tft->setTextSize(2);
    tft->setTextColor(0x0000);
    tft->setCursor(8, 8);
    tft->print("4 MAIN MENU");
  }
  holdStage(5000, false, false, false, false);

  // Stage 5: start the same USB composite stack as production.
  uint64_t diagMac = ESP.getEfuseMac();
  char diagSerial[24];
  snprintf(
      diagSerial,
      sizeof(diagSerial),
      "PIXELPRO-%012llX",
      static_cast<unsigned long long>(
          diagMac));

  USB.VID(USB_VID_PIXEL);
  USB.PID(USB_PID_PIXEL);
  USB.productName("PIXEL PRO");
  USB.manufacturerName("Lumi3D");
  USB.serialNumber(diagSerial);
  USB.firmwareVersion(0x01A1);
  USBSerial.enableReboot(false);
  USBSerial.setRxBufferSize(
      RAW_MEDIA_RX_BUFFER_BYTES);
  USBSerial.begin();
  Keyboard.begin();
  ConsumerControl.begin();
  USB.begin();

  delay(500);
  showStage(0xF81F, "5 USB");
  holdStage(5000, false, false, false, false);

  // Stage 6: key/roller/CDC runtime only.
  showStage(0xFFFF, "6 KEYS + CDC");
  holdStage(7000, true, false, false, false);

  // Stage 7: add RGB runtime. D15 must remain physically disconnected.
  applyRgbProfile();
  showStage(0xFD20, "7 RGB POLL");
  holdStage(7000, true, true, false, false);

  // Stage 8: add module polling.
  showStage(0x07FF, "8 MODULE POLL");
  holdStage(7000, true, true, true, false);

  // Stage 9: add saver runtime too. Stay here so any recurring fault remains
  // visible. Touch remains disabled for this diagnostic.
  lastUserActivityAt = millis();
  showStage(0xFFFF, "9 FULL LOOP");

  while (true) {
    pollKeys();
    pollRoller();
    pollCdc();
    pollRgbEffect();
    pollSaver();
    pollModuleBus();
    delay(1);
  }
#endif

#if PIXEL_DIAG_DISPLAY_ONLY_IDLE
  // Diagnostic: stop immediately after the exact production display init.
  // Draw a bright constant frame once, then leave the LCD bus completely idle.
  // No SD, I2C module bus, LittleFS, USB, RGB animation, keys, saver or touch
  // runtime is started after this point.
  if (displayReady) {
    tft->fillScreen(0xFFFF);
  }

  while (true) {
    delay(1000);
  }
#endif
  if (!PIXEL_DIAG_TOUCH_OFF) {
    initTouch();
  }
  mountSdCard();
  initModuleBus();

  littleFsReady = mountPersistentStorage(true);
  if (littleFsReady) {
    if (!loadMainMenuConfigFile()) {
      (void)saveMainMenuConfig();
    }

    (void)recoverMainMenuActionsFromKeymap();
    recoverMainMenuAssets();
    loadPersistedMedia();
  } else {
    saverReady = false;
    saverActive = false;
    saverFormat = SAVER_NONE;
  }

  renderMainMenu();
  lastUserActivityAt = millis();

  uint64_t mac = ESP.getEfuseMac();
  char serial[24];
  snprintf(
      serial,
      sizeof(serial),
      "PIXELPRO-%012llX",
      static_cast<unsigned long long>(mac));

  USB.VID(USB_VID_PIXEL);
  USB.PID(USB_PID_PIXEL);
  USB.productName("PIXEL PRO");
  USB.manufacturerName("Lumi3D");
  USB.serialNumber(serial);
  USB.firmwareVersion(0x01A1);

  // Normal Lumi Macropad CDC traffic must never be interpreted as a request
  // to enter the ESP32-S2 bootloader. Firmware updates use the dedicated ROM
  // BOOT/esptool path instead.
  USBSerial.enableReboot(false);
  USBSerial.setRxBufferSize(
      RAW_MEDIA_RX_BUFFER_BYTES);
  USBSerial.begin();
  Keyboard.begin();
  ConsumerControl.begin();

  USB.begin();

  delay(500);
  applyRgbProfile();
  sendMappedReports();

  char bootLine[112] = {};
  snprintf(
      bootLine,
      sizeof(bootLine),
      "BOOT|PIXELPRO|%s|RESET=%s|CODE=%u|BOOT=%lu",
      FW_VERSION,
      resetReasonName(
          bootResetReason),
      static_cast<unsigned>(
          bootResetReason),
      static_cast<unsigned long>(
          bootSequence));
  cdcPrintln(bootLine);

  if (!PIXEL_DIAG_TOUCH_OFF &&
      touchCalibrationRequired) {
    // Do not force the device into four-point calibration at boot. This panel
    // can still use the known 480x320 legacy mapping immediately, while
    // LumiPad's Touch Test explicitly exercises the raw panel when requested.
    cdcPrintln(
        "TOUCH_CAL_REQUIRED|DEFAULT_480x320|USE_TOUCH_TEST");
  }
}

void loop() {
  pollKeys();
  pollRoller();
  pollCdc();

  // While a file is crossing USB, do not reconfigure the shared TFT/touch
  // pins, decode a saver frame, animate RGB, or poll I2C modules. This keeps
  // the native USB task serviced and prevents upload-triggered reboot loops.
  if (rawMediaKind !=
      RAW_MEDIA_NONE) {
    delay(1);
    return;
  }

  pollMainMenuBatchTimeout();
  pollDeferredMainMenuRender();
  pollRgbEffect();
  pollSaver();
  if (!PIXEL_DIAG_TOUCH_OFF) {
    pollTouch();
  }
  pollModuleBus();

  if (bootloaderArmed &&
      static_cast<int32_t>(
          millis() - bootloaderArmUntil) >= 0) {
    USBSerial.enableReboot(false);
    bootloaderArmed = false;
  }

  delay(1);
}
