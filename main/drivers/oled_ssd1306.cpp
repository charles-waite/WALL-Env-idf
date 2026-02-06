/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "oled_ssd1306.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

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

static void fb_draw_char(int x, int y, char c, int scale)
{
    if (scale < 1) {
        scale = 1;
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
            for (int dx = 0; dx < scale; dx++) {
                for (int dy = 0; dy < scale; dy++) {
                    fb_set_pixel(x + (col * scale) + dx, y + (row * scale) + dy, true);
                }
            }
        }
    }
}

static int text_width_px(const char *s, int scale)
{
    if (!s) {
        return 0;
    }
    const int char_w = (5 + 1) * scale; // 5px glyph + 1px spacing
    return static_cast<int>(strlen(s)) * char_w - (1 * scale);
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

static void fb_draw_text_centered(int y, const char *s, int scale)
{
    const int w = text_width_px(s, scale);
    const int x = (kWidth - w) / 2;
    fb_draw_text(x, y, s, scale);
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

static void draw_splash()
{
    fb_clear();

    const char *line1 = "WALL-ENV Snsr";
    const char *line2 = "by Waite Design Labs";

    const int scale1 = 2; // ~16px tall
    const int scale2 = 1; // ~8px tall
    const int h1 = 7 * scale1;
    const int h2 = 7 * scale2;
    const int spacing = 4;
    const int total_h = h1 + spacing + h2;
    const int y0 = (kHeight - total_h) / 2;

    fb_draw_text_centered(y0, line1, scale1);
    fb_draw_text_centered(y0 + h1 + spacing, line2, scale2);
}

static void draw_runtime_screen()
{
    fb_clear();

    bsec2_app_latest_t latest = {};
    (void) bsec2_app_get_latest(&latest);

    char line0[32];
    char line1[32];
    char line2[32];
    char line3[32];

    if (latest.have_temp && latest.have_humidity) {
        snprintf(line0, sizeof(line0), "T %.1fC  RH %.1f%%", latest.temp_c, latest.humidity_pct);
    } else {
        snprintf(line0, sizeof(line0), "T --.-C  RH --.-%%");
    }

    if (latest.have_pressure) {
        snprintf(line1, sizeof(line1), "P %.1fhPa", latest.pressure_hpa);
    } else {
        snprintf(line1, sizeof(line1), "P ----.-hPa");
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

    while (true) {
        draw_runtime_screen();
        (void) flush_fb();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t oled_ssd1306_start()
{
    if (s_task) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreate(oled_task, "oled", 4096, nullptr, 2, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

#else

esp_err_t oled_ssd1306_start()
{
    return ESP_OK;
}

#endif // CONFIG_WALL_ENV_OLED_ENABLE
