/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "oled_sh1106.h"
#include "OLEDDisplayFonts.h"
#include "qrcode.h"
#if __has_include("kirby.h")
#include "kirby.h"
#define WALL_ENV_HAS_KIRBY 1
#else
#define WALL_ENV_HAS_KIRBY 0
#endif

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <algorithm>

#include <esp_timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "drivers/bsec2_app.h"
#include "drivers/i2c_bus.h"

static const char *TAG = "oled";

#ifndef CONFIG_WALL_ENV_OLED_ENABLE
#define CONFIG_WALL_ENV_OLED_ENABLE 0
#endif

#if CONFIG_WALL_ENV_OLED_ENABLE

static constexpr uint8_t kI2cCtrlCmd = 0x00;
static constexpr uint8_t kI2cCtrlData = 0x40;

static constexpr int kWidth = CONFIG_WALL_ENV_OLED_WIDTH;
static constexpr int kHeight = CONFIG_WALL_ENV_OLED_HEIGHT;
static constexpr int kPages = kHeight / 8;
static_assert(kWidth == 128, "Only 128px wide OLED panels are supported right now");
static_assert(kHeight == 64, "Only 64px tall OLED panels are supported right now");

static constexpr uint8_t kColOffset = static_cast<uint8_t>(CONFIG_WALL_ENV_OLED_COL_OFFSET);

static i2c_master_dev_handle_t s_dev = nullptr;
static TaskHandle_t s_task = nullptr;
static bool s_is_dimmed = false;
static int8_t s_time_sync_state = -1; // -1 unknown, 0 unsynced, 1 synced
static bool s_commissioning_active = true;
static int64_t s_runtime_cycle_start_us = -1;
static char s_qr_code[128] = {0};
static char s_manual_code[32] = {0};
static constexpr int64_t kKirbyPhaseUs = 25LL * 1000LL * 1000LL;
static constexpr int64_t kSensorPhaseUs = 5LL * 1000LL * 1000LL;
static constexpr int64_t kRuntimeCycleUs = kKirbyPhaseUs + kSensorPhaseUs;

static uint8_t s_fb[kWidth * kPages];

// 5x7 font, ASCII 0x20..0x7E. Public domain style table (common embedded font).
static const uint8_t kFont5x7[] = {
    0x00,0x00,0x00,0x00,0x00, // ' '
    0x00,0x00,0x5F,0x00,0x00, // '!'
    0x00,0x07,0x00,0x07,0x00, // '"'
    0x14,0x7F,0x14,0x7F,0x14, // '#'
    0x24,0x2A,0x7F,0x2A,0x12, // '$'
    0x23,0x13,0x08,0x64,0x62, // '%'
    0x36,0x49,0x55,0x22,0x50, // '&'
    0x00,0x05,0x03,0x00,0x00, // '''
    0x00,0x1C,0x22,0x41,0x00, // '('
    0x00,0x41,0x22,0x1C,0x00, // ')'
    0x14,0x08,0x3E,0x08,0x14, // '*'
    0x08,0x08,0x3E,0x08,0x08, // '+'
    0x00,0x50,0x30,0x00,0x00, // ','
    0x08,0x08,0x08,0x08,0x08, // '-'
    0x00,0x60,0x60,0x00,0x00, // '.'
    0x20,0x10,0x08,0x04,0x02, // '/'
    0x3E,0x51,0x49,0x45,0x3E, // '0'
    0x00,0x42,0x7F,0x40,0x00, // '1'
    0x42,0x61,0x51,0x49,0x46, // '2'
    0x21,0x41,0x45,0x4B,0x31, // '3'
    0x18,0x14,0x12,0x7F,0x10, // '4'
    0x27,0x45,0x45,0x45,0x39, // '5'
    0x3C,0x4A,0x49,0x49,0x30, // '6'
    0x01,0x71,0x09,0x05,0x03, // '7'
    0x36,0x49,0x49,0x49,0x36, // '8'
    0x06,0x49,0x49,0x29,0x1E, // '9'
    0x00,0x36,0x36,0x00,0x00, // ':'
    0x00,0x56,0x36,0x00,0x00, // ';'
    0x08,0x14,0x22,0x41,0x00, // '<'
    0x14,0x14,0x14,0x14,0x14, // '='
    0x00,0x41,0x22,0x14,0x08, // '>'
    0x02,0x01,0x51,0x09,0x06, // '?'
    0x32,0x49,0x79,0x41,0x3E, // '@'
    0x7E,0x11,0x11,0x11,0x7E, // 'A'
    0x7F,0x49,0x49,0x49,0x36, // 'B'
    0x3E,0x41,0x41,0x41,0x22, // 'C'
    0x7F,0x41,0x41,0x22,0x1C, // 'D'
    0x7F,0x49,0x49,0x49,0x41, // 'E'
    0x7F,0x09,0x09,0x09,0x01, // 'F'
    0x3E,0x41,0x49,0x49,0x7A, // 'G'
    0x7F,0x08,0x08,0x08,0x7F, // 'H'
    0x00,0x41,0x7F,0x41,0x00, // 'I'
    0x20,0x40,0x41,0x3F,0x01, // 'J'
    0x7F,0x08,0x14,0x22,0x41, // 'K'
    0x7F,0x40,0x40,0x40,0x40, // 'L'
    0x7F,0x02,0x0C,0x02,0x7F, // 'M'
    0x7F,0x04,0x08,0x10,0x7F, // 'N'
    0x3E,0x41,0x41,0x41,0x3E, // 'O'
    0x7F,0x09,0x09,0x09,0x06, // 'P'
    0x3E,0x41,0x51,0x21,0x5E, // 'Q'
    0x7F,0x09,0x19,0x29,0x46, // 'R'
    0x46,0x49,0x49,0x49,0x31, // 'S'
    0x01,0x01,0x7F,0x01,0x01, // 'T'
    0x3F,0x40,0x40,0x40,0x3F, // 'U'
    0x1F,0x20,0x40,0x20,0x1F, // 'V'
    0x7F,0x20,0x18,0x20,0x7F, // 'W'
    0x63,0x14,0x08,0x14,0x63, // 'X'
    0x03,0x04,0x78,0x04,0x03, // 'Y'
    0x61,0x51,0x49,0x45,0x43, // 'Z'
    0x00,0x7F,0x41,0x41,0x00, // '['
    0x02,0x04,0x08,0x10,0x20, // '\'
    0x00,0x41,0x41,0x7F,0x00, // ']'
    0x04,0x02,0x01,0x02,0x04, // '^'
    0x40,0x40,0x40,0x40,0x40, // '_'
    0x00,0x01,0x02,0x04,0x00, // '`'
    0x20,0x54,0x54,0x54,0x78, // 'a'
    0x7F,0x48,0x44,0x44,0x38, // 'b'
    0x38,0x44,0x44,0x44,0x20, // 'c'
    0x38,0x44,0x44,0x48,0x7F, // 'd'
    0x38,0x54,0x54,0x54,0x18, // 'e'
    0x08,0x7E,0x09,0x01,0x02, // 'f'
    0x0C,0x52,0x52,0x52,0x3E, // 'g'
    0x7F,0x08,0x04,0x04,0x78, // 'h'
    0x00,0x44,0x7D,0x40,0x00, // 'i'
    0x20,0x40,0x44,0x3D,0x00, // 'j'
    0x7F,0x10,0x28,0x44,0x00, // 'k'
    0x00,0x41,0x7F,0x40,0x00, // 'l'
    0x7C,0x04,0x18,0x04,0x78, // 'm'
    0x7C,0x08,0x04,0x04,0x78, // 'n'
    0x38,0x44,0x44,0x44,0x38, // 'o'
    0x7C,0x14,0x14,0x14,0x08, // 'p'
    0x08,0x14,0x14,0x18,0x7C, // 'q'
    0x7C,0x08,0x04,0x04,0x08, // 'r'
    0x48,0x54,0x54,0x54,0x20, // 's'
    0x04,0x3F,0x44,0x40,0x20, // 't'
    0x3C,0x40,0x40,0x20,0x7C, // 'u'
    0x1C,0x20,0x40,0x20,0x1C, // 'v'
    0x3C,0x40,0x30,0x40,0x3C, // 'w'
    0x44,0x28,0x10,0x28,0x44, // 'x'
    0x0C,0x50,0x50,0x50,0x3C, // 'y'
    0x44,0x64,0x54,0x4C,0x44, // 'z'
    0x00,0x08,0x36,0x41,0x00, // '{'
    0x00,0x00,0x7F,0x00,0x00, // '|'
    0x00,0x41,0x36,0x08,0x00, // '}'
    0x08,0x04,0x08,0x10,0x08, // '~'
};

static esp_err_t tx_bytes(const uint8_t *data, size_t len)
{
    if (!s_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit(s_dev, data, len, 1000);
}

static esp_err_t send_cmd(uint8_t cmd)
{
    uint8_t pkt[2] = {kI2cCtrlCmd, cmd};
    return tx_bytes(pkt, sizeof(pkt));
}

static esp_err_t set_contrast(uint8_t contrast)
{
    esp_err_t err = send_cmd(0x81);
    if (err != ESP_OK) {
        return err;
    }
    return send_cmd(contrast);
}

static esp_err_t send_cmds(const uint8_t *cmds, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        esp_err_t err = send_cmd(cmds[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static void fb_clear()
{
    memset(s_fb, 0, sizeof(s_fb));
}

static void fb_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= kWidth || y < 0 || y >= kHeight) {
        return;
    }
    const int page = y / 8;
    const uint8_t bit = 1U << (y % 8);
    const int idx = (page * kWidth) + x;
    if (on) {
        s_fb[idx] |= bit;
    } else {
        s_fb[idx] &= static_cast<uint8_t>(~bit);
    }
}

static void fb_fill_rect(int x, int y, int w, int h, bool on)
{
    for (int yy = y; yy < (y + h); ++yy) {
        for (int xx = x; xx < (x + w); ++xx) {
            fb_set_pixel(xx, yy, on);
        }
    }
}

static void fb_draw_char_xy(int x, int y, char c, int scale_x, int scale_y)
{
    if (scale_x < 1) {
        scale_x = 1;
    }
    if (scale_y < 1) {
        scale_y = 1;
    }
    if (c < 0x20 || c > 0x7E) {
        c = '?';
    }
    const int glyph_index = (c - 0x20) * 5;

    for (int col = 0; col < 5; col++) {
        const uint8_t bits = kFont5x7[glyph_index + col];
        for (int row = 0; row < 7; row++) {
            const bool on = (bits >> row) & 0x1;
            if (!on) {
                continue;
            }
            for (int dx = 0; dx < scale_x; dx++) {
                for (int dy = 0; dy < scale_y; dy++) {
                    fb_set_pixel(x + (col * scale_x) + dx, y + (row * scale_y) + dy, true);
                }
            }
        }
    }
}

static void fb_draw_char(int x, int y, char c, int scale)
{
    fb_draw_char_xy(x, y, c, scale, scale);
}

static void fb_draw_text(int x, int y, const char *s, int scale)
{
    if (!s) {
        return;
    }
    const int step = (5 + 1) * scale;
    for (size_t i = 0; s[i] != '\0'; i++) {
        fb_draw_char(x + static_cast<int>(i) * step, y, s[i], scale);
    }
}

// ThingPulse-style font format from OLEDDisplayFonts.h:
// [max_w, height, first_char, char_count, jump_table..., glyph_data...]
static int tp_font_height(const uint8_t *font)
{
    return font ? static_cast<int>(font[1]) : 0;
}

static int tp_glyph_width(const uint8_t *font, uint8_t c)
{
    if (!font) {
        return 0;
    }
    const int first = static_cast<int>(font[2]);
    const int count = static_cast<int>(font[3]);
    const int idx = static_cast<int>(c) - first;
    if (idx < 0 || idx >= count) {
        return 0;
    }
    const uint8_t *entry = &font[4 + idx * 4];
    return static_cast<int>(entry[3]);
}

static void tp_draw_char(int x, int y, const uint8_t *font, uint8_t c)
{
    if (!font) {
        return;
    }
    const int height = static_cast<int>(font[1]);
    const int first = static_cast<int>(font[2]);
    const int count = static_cast<int>(font[3]);
    int idx = static_cast<int>(c) - first;
    if (idx < 0 || idx >= count) {
        idx = static_cast<int>('?') - first;
        if (idx < 0 || idx >= count) {
            return;
        }
    }

    const uint8_t *entry = &font[4 + idx * 4];
    const uint16_t offset = static_cast<uint16_t>((entry[0] << 8) | entry[1]);
    const uint8_t len = entry[2];
    if (offset == 0xFFFF || len == 0) {
        return;
    }

    const int raster_height = (height + 7) / 8; // ceil(height / 8)
    if (raster_height <= 0) {
        return;
    }
    const int glyph_base = 4 + count * 4;
    const uint8_t *glyph = &font[glyph_base + offset];

    // Match ThingPulse's drawInternal() packing:
    // byte stream walks x by raster blocks, then y-page within each column.
    for (int i = 0; i < len; i++) {
        const uint8_t current = glyph[i];
        const int col = i / raster_height;
        const int page = i % raster_height;
        for (int bit = 0; bit < 8; bit++) {
            if ((current & (1U << bit)) == 0) {
                continue;
            }
            const int row = page * 8 + bit;
            if (row < height) {
                fb_set_pixel(x + col, y + row, true);
            }
        }
    }
}

static int tp_text_width_px(const uint8_t *font, const char *s)
{
    if (!font || !s) {
        return 0;
    }
    int w = 0;
    for (size_t i = 0; s[i] != '\0'; i++) {
        int gw = tp_glyph_width(font, static_cast<uint8_t>(s[i]));
        if (gw <= 0) {
            gw = tp_glyph_width(font, static_cast<uint8_t>('?'));
        }
        w += gw;
    }
    return w;
}

static void tp_draw_text_centered(int y, const uint8_t *font, const char *s)
{
    if (!font || !s) {
        return;
    }
    const int w = tp_text_width_px(font, s);
    int x = (kWidth - w) / 2;
    for (size_t i = 0; s[i] != '\0'; i++) {
        uint8_t ch = static_cast<uint8_t>(s[i]);
        int gw = tp_glyph_width(font, ch);
        if (gw <= 0) {
            ch = static_cast<uint8_t>('?');
            gw = tp_glyph_width(font, ch);
        }
        tp_draw_char(x, y, font, ch);
        x += gw;
    }
}

static esp_err_t flush_fb()
{
    esp_err_t err;
    for (int page = 0; page < kPages; page++) {
        // Set page + column address to 0
        const uint8_t page_cmds[] = {
            static_cast<uint8_t>(0xB0 | page),
            static_cast<uint8_t>(0x00 | (kColOffset & 0x0F)),         // low col (SH1106 often needs +2)
            static_cast<uint8_t>(0x10 | ((kColOffset >> 4) & 0x0F)),  // high col
        };
        err = send_cmds(page_cmds, sizeof(page_cmds));
        if (err != ESP_OK) {
            return err;
        }

        const uint8_t *row = &s_fb[page * kWidth];
        // Chunk the data to keep packets small.
        for (int off = 0; off < kWidth; off += 16) {
            const int chunk = (kWidth - off) >= 16 ? 16 : (kWidth - off);
            uint8_t pkt[1 + 16];
            pkt[0] = kI2cCtrlData;
            memcpy(&pkt[1], row + off, chunk);
            err = tx_bytes(pkt, 1 + chunk);
            if (err != ESP_OK) {
                return err;
            }
        }
    }

    return ESP_OK;
}

static esp_err_t sh1106_init()
{
    const uint8_t init_cmds[] = {
        // SH1106 init (page addressing + 132-col internal RAM).
        0xAE,       // display off
        0xD5, 0x80, // display clock divide ratio/oscillator
        0xA8, 0x3F, // multiplex ratio (1/64)
        0xD3, 0x00, // display offset
        0x40,       // display start line = 0
        0xAD, 0x8B, // DC-DC control: ON (common on I2C modules)
        0xA1,       // segment remap
        0xC8,       // COM scan direction: remapped mode
        0xDA, 0x12, // COM pins hardware configuration
        0x81, 0x7F, // contrast
        0xD9, 0x22, // pre-charge period
        0xDB, 0x20, // VCOM deselect level
        0x20, 0x02, // memory addressing mode: page addressing
        0xA4,       // entire display ON from RAM
        0xA6,       // normal (not inverted)
        0xAF,       // display on
    };

    // init_cmds includes 2-byte sequences; send one byte at a time for simplicity.
    return send_cmds(init_cmds, sizeof(init_cmds));
}

static int get_timezone_offset_minutes(time_t now, const struct tm &local_tm)
{
    struct tm utc_tm = {};
    if (!gmtime_r(&now, &utc_tm)) {
        return 0;
    }

    // mktime() interprets both structs as local time; the delta yields local UTC offset.
    const time_t local_epoch = mktime(const_cast<struct tm *>(&local_tm));
    const time_t utc_as_local_epoch = mktime(&utc_tm);
    return static_cast<int>(difftime(local_epoch, utc_as_local_epoch) / 60.0);
}

static bool get_sunrise_sunset_minutes(const struct tm &local_tm, int tz_min, int *sunrise_min, int *sunset_min)
{
    if (!sunrise_min || !sunset_min) {
        return false;
    }

    // ZIP 98103 (Seattle) approximate coordinates.
    static constexpr double kLatDeg = 47.67;
    static constexpr double kLonDeg = -122.34;
    static constexpr double kZenithDeg = 90.833; // official sunrise/sunset
    static constexpr double kDegToRad = M_PI / 180.0;
    static constexpr double kRadToDeg = 180.0 / M_PI;

    const int day_of_year = local_tm.tm_yday + 1;
    const double gamma = 2.0 * M_PI / 365.0 * (day_of_year - 1);
    const double eqtime = 229.18 * (0.000075 + 0.001868 * cos(gamma) - 0.032077 * sin(gamma) -
                                    0.014615 * cos(2.0 * gamma) - 0.040849 * sin(2.0 * gamma));
    const double decl = 0.006918 - 0.399912 * cos(gamma) + 0.070257 * sin(gamma) - 0.006758 * cos(2.0 * gamma) +
                        0.000907 * sin(2.0 * gamma) - 0.002697 * cos(3.0 * gamma) + 0.00148 * sin(3.0 * gamma);

    const double lat_rad = kLatDeg * kDegToRad;
    const double zenith_rad = kZenithDeg * kDegToRad;
    const double cos_ha = (cos(zenith_rad) / (cos(lat_rad) * cos(decl))) - tan(lat_rad) * tan(decl);
    if (cos_ha <= -1.0 || cos_ha >= 1.0) {
        return false;
    }

    const double ha_deg = acos(cos_ha) * kRadToDeg;
    const double solar_noon = 720.0 - 4.0 * kLonDeg - eqtime + tz_min;
    int rise = static_cast<int>(std::lround(solar_noon - ha_deg * 4.0));
    int set = static_cast<int>(std::lround(solar_noon + ha_deg * 4.0));

    rise = (rise % 1440 + 1440) % 1440;
    set = (set % 1440 + 1440) % 1440;

    *sunrise_min = rise;
    *sunset_min = set;
    return true;
}

static void maybe_update_day_night_contrast()
{
    const time_t now = time(nullptr);
    if (now <= 0) {
        if (s_time_sync_state != 0) {
            ESP_LOGI(TAG, "Time sync pending (clock not set yet)");
            s_time_sync_state = 0;
        }
        return;
    }

    struct tm local_tm = {};
    if (!localtime_r(&now, &local_tm)) {
        return;
    }
    const bool have_time = (local_tm.tm_year >= (2024 - 1900));
    if (s_time_sync_state != (have_time ? 1 : 0)) {
        if (have_time) {
            ESP_LOGI(TAG, "Time synchronized: %04d-%02d-%02d %02d:%02d:%02d",
                     local_tm.tm_year + 1900, local_tm.tm_mon + 1, local_tm.tm_mday,
                     local_tm.tm_hour, local_tm.tm_min, local_tm.tm_sec);
        } else {
            ESP_LOGI(TAG, "Time sync pending (invalid calendar year: %d)", local_tm.tm_year + 1900);
        }
        s_time_sync_state = have_time ? 1 : 0;
    }

    if (!have_time) {
        // Skip dimming until wall-clock time is plausibly synchronized.
        return;
    }

    int sunrise_min = 0;
    int sunset_min = 0;
    const int tz_min = get_timezone_offset_minutes(now, local_tm);
    if (!get_sunrise_sunset_minutes(local_tm, tz_min, &sunrise_min, &sunset_min)) {
        return;
    }

    const int now_min = local_tm.tm_hour * 60 + local_tm.tm_min;
    const bool should_dim = (now_min < sunrise_min) || (now_min >= sunset_min);
    if (should_dim == s_is_dimmed) {
        return;
    }

    if (set_contrast(should_dim ? 0x00 : 0xFF) == ESP_OK) {
        s_is_dimmed = should_dim;
        ESP_LOGI(TAG, "OLED contrast set to %u (%s mode, sunrise=%02d:%02d sunset=%02d:%02d)",
                 should_dim ? 0 : 255,
                 should_dim ? "night" : "day",
                 sunrise_min / 60, sunrise_min % 60,
                 sunset_min / 60, sunset_min % 60);
    }
}

static void draw_splash()
{
    fb_clear();

    const char *line1 = "WALL-ENV Snsr";
    const char *line2 = "by Waite Design Labs";

    const uint8_t *font1 = ArialMT_Plain_16;
    const uint8_t *font2 = ArialMT_Plain_10;
    const int h1 = tp_font_height(font1);
    const int h2 = tp_font_height(font2);
    const int spacing = 2;
    const int total_h = h1 + spacing + h2;
    const int y0 = (kHeight - total_h) / 2;

    tp_draw_text_centered(y0, font1, line1);
    tp_draw_text_centered(y0 + h1 + spacing, font2, line2);
}

static void draw_runtime_screen()
{
    fb_clear();

    bsec2_app_latest_t latest = {};
    (void) bsec2_app_get_latest(&latest);

    char line0[40];
    char line1[32];
    char line2[32];
    char line3[32];

    time_t now = time(nullptr);
    struct tm local_tm = {};
    bool have_time = false;
    if (now > 0 && localtime_r(&now, &local_tm)) {
        have_time = (local_tm.tm_year >= (2024 - 1900));
    }

    static const char *kWeekday[] = {"Sun", "Mon", "Tue", "Weds", "Thu", "Fri", "Sat"};
    if (now > 0 && local_tm.tm_wday >= 0 && local_tm.tm_wday <= 6) {
        int hour12 = local_tm.tm_hour % 12;
        if (hour12 == 0) {
            hour12 = 12;
        }
        const char *ampm = (local_tm.tm_hour >= 12) ? "PM" : "AM";
        if (have_time) {
            snprintf(line0, sizeof(line0), "%s %d:%02d%s %d/%d/%02d",
                     kWeekday[local_tm.tm_wday], hour12, local_tm.tm_min, ampm,
                     local_tm.tm_mon + 1, local_tm.tm_mday, (local_tm.tm_year + 1900) % 100);
        } else {
            snprintf(line0, sizeof(line0), "*%s %d:%02d%s %d/%d/%02d*",
                     kWeekday[local_tm.tm_wday], hour12, local_tm.tm_min, ampm,
                     local_tm.tm_mon + 1, local_tm.tm_mday, (local_tm.tm_year + 1900) % 100);
        }
    } else {
        snprintf(line0, sizeof(line0), "*Time not synced*");
    }

    if (latest.have_temp && latest.have_humidity) {
        const float temp_f = (latest.temp_c * 9.0f / 5.0f) + 32.0f;
        snprintf(line1, sizeof(line1), "T %.1fF  RH %.1f%%", temp_f, latest.humidity_pct);
    } else {
        snprintf(line1, sizeof(line1), "T --.-F  RH --.-%%");
    }

    if (latest.have_iaq) {
        snprintf(line2, sizeof(line2), "IAQ %.0f (acc %u)", latest.iaq, latest.iaq_accuracy);
    } else {
        snprintf(line2, sizeof(line2), "IAQ --- (acc -)");
    }

    if (latest.have_co2) {
        snprintf(line3, sizeof(line3), "CO2 %.0f ppm", latest.co2_ppm);
    } else {
        snprintf(line3, sizeof(line3), "CO2 --- ppm");
    }

    fb_draw_text(0, 0, line0, 1);
    fb_draw_text(0, 16, line1, 1);
    fb_draw_text(0, 32, line2, 1);
    fb_draw_text(0, 48, line3, 1);
}

static bool try_build_qr(QRCode *qr, uint8_t *buffer, size_t buffer_len, const char *text)
{
    if (!qr || !buffer || !text || text[0] == '\0') {
        return false;
    }

    // Pick the smallest version that can encode the payload to maximize module size on 128x64.
    for (uint8_t version = 1; version <= 10; ++version) {
        const uint16_t needed = qrcode_getBufferSize(version);
        if (needed > buffer_len) {
            continue;
        }
        if (qrcode_initText(qr, buffer, version, ECC_LOW, text) == 0) {
            return true;
        }
    }
    return false;
}

static void draw_commissioning_screen()
{
    fb_clear();

    char qr_text[sizeof(s_qr_code)] = {0};
    strncpy(qr_text, s_qr_code, sizeof(qr_text) - 1);

    if (qr_text[0] == '\0') {
        fb_draw_text(0, 0, "No QR payload", 1);
        return;
    }

    QRCode qr = {};
    uint8_t qr_buf[400] = {0};
    if (!try_build_qr(&qr, qr_buf, sizeof(qr_buf), qr_text)) {
        fb_draw_text(0, 0, "QR encode failed", 1);
        return;
    }

    const int quiet = 2;
    const int total_modules = qr.size + (2 * quiet);
    const int module_px = std::max(1, std::min(kWidth / total_modules, kHeight / total_modules));
    const int total_px = total_modules * module_px;
    const int x0 = (kWidth - total_px) / 2;
    const int y0 = (kHeight - total_px) / 2;

    // White background square so modules render as black for scanner compatibility.
    fb_fill_rect(x0, y0, total_px, total_px, true);

    for (int y = 0; y < qr.size; ++y) {
        for (int x = 0; x < qr.size; ++x) {
            const bool module_is_dark = qrcode_getModule(&qr, static_cast<uint8_t>(x), static_cast<uint8_t>(y));
            const int px = x0 + (x + quiet) * module_px;
            const int py = y0 + (y + quiet) * module_px;
            // module_is_dark => black pixel block, else keep white block.
            if (module_is_dark) {
                fb_fill_rect(px, py, module_px, module_px, false);
            }
        }
    }
}

// XBM-style bitmap layout: row-major, 1bpp, LSB-first inside each byte.
static void fb_draw_bitmap_mono_lsb(int x, int y, int w, int h, const uint8_t *bitmap, size_t bitmap_len)
{
    if (!bitmap || w <= 0 || h <= 0) {
        return;
    }
    const int bytes_per_row = (w + 7) / 8;
    for (int yy = 0; yy < h; ++yy) {
        for (int xx = 0; xx < w; ++xx) {
            const size_t idx = static_cast<size_t>((yy * bytes_per_row) + (xx / 8));
            if (idx >= bitmap_len) {
                continue;
            }
            const uint8_t b = bitmap[idx];
            const bool on = (b & static_cast<uint8_t>(1U << (xx & 0x7))) != 0;
            fb_set_pixel(x + xx, y + yy, on);
        }
    }
}

static void draw_kirby_screen()
{
    fb_clear();
#if WALL_ENV_HAS_KIRBY
    const size_t kirby_len = sizeof(kirby_bits);
    constexpr size_t kirby_expected_len = static_cast<size_t>(((KIRBY_W + 7) / 8) * KIRBY_H);
    (void) kirby_expected_len;
    const int x0 = std::max(0, (kWidth - KIRBY_W) / 2);
    const int y0 = std::max(0, (kHeight - KIRBY_H) / 2);
    fb_draw_bitmap_mono_lsb(x0, y0, KIRBY_W, KIRBY_H, kirby_bits, kirby_len);
#else
    fb_draw_text(0, 24, "Missing kirby.h", 1);
#endif
}

static void oled_task(void *arg)
{
    (void) arg;

    esp_err_t err = i2c_bus_add_device(CONFIG_WALL_ENV_OLED_I2C_ADDR, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED I2C device add failed: %d", err);
        vTaskDelete(nullptr);
        return;
    }

    err = sh1106_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SH1106 init failed: %d", err);
        vTaskDelete(nullptr);
        return;
    }

    draw_splash();
    (void) flush_fb();
    vTaskDelay(pdMS_TO_TICKS(CONFIG_WALL_ENV_OLED_SPLASH_MS));

    // Local timezone for ZIP 98103 (Seattle): PST/PDT with automatic DST.
    setenv("TZ", "PST8PDT,M3.2.0/2,M11.1.0/2", 1);
    tzset();

    while (true) {
        maybe_update_day_night_contrast();
        if (s_commissioning_active) {
            s_runtime_cycle_start_us = -1;
            draw_commissioning_screen();
        } else {
            const int64_t now_us = esp_timer_get_time();
            if (s_runtime_cycle_start_us < 0) {
                s_runtime_cycle_start_us = now_us;
            }
            const int64_t elapsed_us = (now_us - s_runtime_cycle_start_us) % kRuntimeCycleUs;
            if (elapsed_us < kKirbyPhaseUs) {
                draw_kirby_screen();
            } else {
                draw_runtime_screen();
            }
        }
        (void) flush_fb();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t oled_sh1106_start()
{
    if (s_task) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreate(oled_task, "oled", 4096, nullptr, 2, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void oled_sh1106_set_commissioning_codes(const char *qr_code, const char *manual_code)
{
    s_qr_code[0] = '\0';
    s_manual_code[0] = '\0';
    if (qr_code) {
        strncpy(s_qr_code, qr_code, sizeof(s_qr_code) - 1);
    }
    if (manual_code) {
        strncpy(s_manual_code, manual_code, sizeof(s_manual_code) - 1);
    }
}

void oled_sh1106_set_commissioning_active(bool active)
{
    if (s_commissioning_active != active) {
        s_runtime_cycle_start_us = -1;
    }
    s_commissioning_active = active;
}

#else

esp_err_t oled_sh1106_start()
{
    return ESP_OK;
}

void oled_sh1106_set_commissioning_codes(const char *qr_code, const char *manual_code)
{
    (void) qr_code;
    (void) manual_code;
}

void oled_sh1106_set_commissioning_active(bool active)
{
    (void) active;
}

#endif // CONFIG_WALL_ENV_OLED_ENABLE
