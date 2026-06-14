/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "display.h"

#include <string.h>
#include <time.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"

/* ------------------------------------------------------------------ */
/* Cấu hình phần cứng — override từ Kconfig                           */
/* ------------------------------------------------------------------ */

#define LCD_HOST         SPI2_HOST
#define LCD_H_RES        240
#define LCD_V_RES        320
#define LCD_BITS_PER_PX  16
#define LCD_CLK_HZ       (CONFIG_LCD_SPI_CLOCK_MHZ * 1000 * 1000)
#define LCD_MOSI_PIN     CONFIG_LCD_MOSI_GPIO
#define LCD_SCLK_PIN     CONFIG_LCD_SCLK_GPIO
#define LCD_CS_PIN       CONFIG_LCD_CS_GPIO
#define LCD_DC_PIN       CONFIG_LCD_DC_GPIO
#define LCD_RST_PIN      CONFIG_LCD_RST_GPIO
#define LCD_BL_PIN       CONFIG_LCD_BL_GPIO   /* -1 để bỏ qua */

/* LVGL draw buffer: 20 dòng × 2 buffer */
#define LVGL_DRAW_BUF_LINES  20
#define LVGL_TICK_PERIOD_MS  5
#define LVGL_TASK_STACK_KB   6
#define LVGL_TASK_PRIORITY   2

/* ------------------------------------------------------------------ */
/* Tọa độ layout (tất cả là tọa độ tuyệt đối trên màn hình)          */
/* ------------------------------------------------------------------ */

/* Topbar */
#define TURN_ICON_X       8
#define TURN_ICON_Y       2
#define TURN_ICON_W       46
#define TURN_ICON_H       46

#define DIST_LABEL_X      57
#define DIST_LABEL_W      46
#define DIST_NUM_Y         5
#define DIST_UNIT_Y       28

#define TURN_INFO_X       104
#define TURN_INFO_Y       8
#define TURN_INFO_W       136
#define TURN_INFO_H       40

#define PROGRESS_X        13
#define PROGRESS_Y        50
#define PROGRESS_W        214
#define PROGRESS_H        5

/* Map area */
#define MAP_X             0
#define MAP_Y             55
#define MAP_W             240
#define MAP_H             180
#define MAP_USER_X        (MAP_W / 2)       /* 120 — mobile đảm bảo user luôn ở đây */
#define MAP_USER_Y        (MAP_H * 2 / 3)   /* 120 */

/* Bottom bar — vùng tốc độ (trái) và điểm đến (giữa) */
#define SPEED_ZONE_X       2
#define SPEED_ZONE_W      74
#define SPEED_NUM_Y      244
#define KMH_Y            272

#define DEST_ZONE_X       78
#define DEST_ZONE_W       76
#define DEST_ICON_Y      237
#define DEST_DIST_Y      268
#define DEST_TIME_Y      282

#define WARN_BORDER_X     155
#define WARN_BORDER_Y     236
#define WARN_BORDER_W     80
#define WARN_BORDER_H     80

#define WARN_IMG_X        (WARN_BORDER_X + 16)
#define WARN_IMG_Y        (WARN_BORDER_Y + 6)
#define WARN_IMG_W        48
#define WARN_IMG_H        48

#define WARN_TEXT_X       160
#define WARN_TEXT_Y       289

#define BAT_ICON_X        3
#define BAT_ICON_Y        298

#define CONN_ICON_X       23
#define CONN_ICON_Y       298

#define TIME_LABEL_X      74
#define TIME_LABEL_Y      298

/* ------------------------------------------------------------------ */
/* Lưu trữ dữ liệu bản đồ (copy từ payload thoáng qua)               */
/* ------------------------------------------------------------------ */

#define DISP_MAP_MAX_LINES   NUS_PROTO_MAP_MAX_LINES
#define DISP_MAP_MAX_PTS     NUS_PROTO_MAP_LINE_MAX_POINTS

typedef struct {
    uint8_t line_type;
    uint8_t point_count;
    uint8_t points[DISP_MAP_MAX_PTS * 2];
} disp_map_line_t;

typedef struct {
    uint8_t line_count;
    disp_map_line_t lines[DISP_MAP_MAX_LINES];
} disp_map_data_t;

/* ------------------------------------------------------------------ */
/* Dữ liệu pending — được copy từ BLE callback vào đây dưới mutex     */
/* ------------------------------------------------------------------ */

typedef struct {
    char direction[64];
    char next_direction[64];
    uint32_t distance_m;
    uint32_t destination_distance_m;
    uint32_t remaining_time_minutes;
    uint16_t current_speed_mps;
    uint32_t current_time_epoch_seconds;
    uint16_t route_progress_permille;
} disp_nav_data_t;

typedef struct {
    uint8_t sign_type;
    char text[64];
} disp_sign_data_t;

#define DISP_NAV_IMG_MAX  486

typedef struct {
    uint8_t image_type;
    uint16_t width;
    uint16_t height;
    uint16_t data_len;
    uint8_t data[DISP_NAV_IMG_MAX];
} disp_nav_img_data_t;

/* ------------------------------------------------------------------ */
/* State tĩnh                                                          */
/* ------------------------------------------------------------------ */

static const char *TAG = "DISP";

static SemaphoreHandle_t s_mutex;
static volatile bool s_display_ready;

static bool s_pending_nav;
static bool s_pending_map;
static bool s_pending_sign;
static bool s_pending_nav_img;
static bool s_pending_connection;

static disp_nav_data_t    s_nav;
static disp_map_data_t    s_map;
static disp_sign_data_t   s_sign;
static disp_nav_img_data_t s_nav_img;
static bool               s_connected;

/* LVGL objects */
static lv_obj_t *s_turn_icon_label;
static lv_obj_t *s_dist_label;
static lv_obj_t *s_dist_unit_label;
static lv_obj_t *s_turn_info_label;
static lv_obj_t *s_progress_bar;
static lv_obj_t *s_map_obj;
static lv_obj_t *s_speed_label;
static lv_obj_t *s_dest_dist_label;
static lv_obj_t *s_dest_time_label;
static lv_obj_t *s_warn_text_label;
static lv_obj_t *s_warn_img_label;
static lv_obj_t *s_bat_label;
static lv_obj_t *s_conn_label;
static lv_obj_t *s_time_label;

/* Draw buffer cho LVGL */
static lv_color_t s_buf1[LCD_H_RES * LVGL_DRAW_BUF_LINES];
static lv_color_t s_buf2[LCD_H_RES * LVGL_DRAW_BUF_LINES];

/* esp_lcd panel handle */
static esp_lcd_panel_handle_t s_panel;

/* ------------------------------------------------------------------ */
/* Helper: chuyển hướng → ký tự mũi tên                               */
/* ------------------------------------------------------------------ */

static const char *direction_to_arrow(const char *dir)
{
    if (!dir || dir[0] == '\0') return LV_SYMBOL_UP;
    /* so sánh case-insensitive bằng cách tìm substring */
    const char *d = dir;
    /* left */
    if (strstr(d, "left") || strstr(d, "Left") || strstr(d, "LEFT") ||
        strstr(d, "trai") || strstr(d, "Trai")) {
        return LV_SYMBOL_LEFT;
    }
    /* right */
    if (strstr(d, "right") || strstr(d, "Right") || strstr(d, "RIGHT") ||
        strstr(d, "phai") || strstr(d, "Phai")) {
        return LV_SYMBOL_RIGHT;
    }
    /* u-turn */
    if (strstr(d, "u-turn") || strstr(d, "U-turn") || strstr(d, "uturn") ||
        strstr(d, "quay") || strstr(d, "Quay")) {
        return LV_SYMBOL_LOOP;
    }
    /* default: straight / thẳng */
    return LV_SYMBOL_UP;
}

/* ------------------------------------------------------------------ */
/* Helper: format khoảng cách                                          */
/* ------------------------------------------------------------------ */

static void fmt_distance(char *buf, size_t sz, uint32_t meters)
{
    if (meters >= 1000) {
        snprintf(buf, sz, "%.1f km", meters / 1000.0f);
    } else {
        snprintf(buf, sz, "%" PRIu32 " m", meters);
    }
}

/* Tách số và đơn vị — dùng cho topbar distance 2 dòng */
static void fmt_dist_num(char *buf, size_t sz, uint32_t meters)
{
    if (meters >= 1000)
        snprintf(buf, sz, "%.1f", meters / 1000.0f);
    else
        snprintf(buf, sz, "%" PRIu32, meters);
}

static const char *fmt_dist_unit(uint32_t meters)
{
    return (meters >= 1000) ? "km" : "m";
}

/* Helper: format thời gian còn lại */
static void fmt_duration(char *buf, size_t sz, uint32_t minutes)
{
    if (minutes >= 60) {
        snprintf(buf, sz, "%" PRIu32 "h%02" PRIu32, minutes / 60, minutes % 60);
    } else {
        snprintf(buf, sz, "%" PRIu32 "min", minutes);
    }
}

/* ------------------------------------------------------------------ */
/* Helper: chuyển UTF-8 tiếng Việt → ASCII (bỏ dấu)                   */
/* ------------------------------------------------------------------ */

static const struct {
    uint8_t bytes[3];
    uint8_t len;
    char    ascii;
} s_vn_map[] = {
    /* 2-byte U+00C0-U+00FF */
    {{0xC3,0x80},2,'A'},{{0xC3,0x81},2,'A'},{{0xC3,0x82},2,'A'},{{0xC3,0x83},2,'A'},
    {{0xC3,0x88},2,'E'},{{0xC3,0x89},2,'E'},{{0xC3,0x8A},2,'E'},
    {{0xC3,0x8C},2,'I'},{{0xC3,0x8D},2,'I'},
    {{0xC3,0x92},2,'O'},{{0xC3,0x93},2,'O'},{{0xC3,0x94},2,'O'},{{0xC3,0x95},2,'O'},
    {{0xC3,0x99},2,'U'},{{0xC3,0x9A},2,'U'},{{0xC3,0x9D},2,'Y'},
    {{0xC3,0xA0},2,'a'},{{0xC3,0xA1},2,'a'},{{0xC3,0xA2},2,'a'},{{0xC3,0xA3},2,'a'},
    {{0xC3,0xA8},2,'e'},{{0xC3,0xA9},2,'e'},{{0xC3,0xAA},2,'e'},
    {{0xC3,0xAC},2,'i'},{{0xC3,0xAD},2,'i'},
    {{0xC3,0xB2},2,'o'},{{0xC3,0xB3},2,'o'},{{0xC3,0xB4},2,'o'},{{0xC3,0xB5},2,'o'},
    {{0xC3,0xB9},2,'u'},{{0xC3,0xBA},2,'u'},{{0xC3,0xBD},2,'y'},
    /* 2-byte Latin Ext: Ă ă Đ đ Ơ ơ Ư ư */
    {{0xC4,0x82},2,'A'},{{0xC4,0x83},2,'a'},
    {{0xC4,0x90},2,'D'},{{0xC4,0x91},2,'d'},
    {{0xC6,0xA0},2,'O'},{{0xC6,0xA1},2,'o'},
    {{0xC6,0xAF},2,'U'},{{0xC6,0xB0},2,'u'},
    /* 3-byte U+1EA0-U+1EF9 (dấu thanh tiếng Việt) */
    {{0xE1,0xBA,0xA0},3,'A'},{{0xE1,0xBA,0xA1},3,'a'},
    {{0xE1,0xBA,0xA2},3,'A'},{{0xE1,0xBA,0xA3},3,'a'},
    {{0xE1,0xBA,0xA4},3,'A'},{{0xE1,0xBA,0xA5},3,'a'},
    {{0xE1,0xBA,0xA6},3,'A'},{{0xE1,0xBA,0xA7},3,'a'},
    {{0xE1,0xBA,0xA8},3,'A'},{{0xE1,0xBA,0xA9},3,'a'},
    {{0xE1,0xBA,0xAA},3,'A'},{{0xE1,0xBA,0xAB},3,'a'},
    {{0xE1,0xBA,0xAC},3,'A'},{{0xE1,0xBA,0xAD},3,'a'},
    {{0xE1,0xBA,0xAE},3,'A'},{{0xE1,0xBA,0xAF},3,'a'},
    {{0xE1,0xBA,0xB0},3,'A'},{{0xE1,0xBA,0xB1},3,'a'},
    {{0xE1,0xBA,0xB2},3,'A'},{{0xE1,0xBA,0xB3},3,'a'},
    {{0xE1,0xBA,0xB4},3,'A'},{{0xE1,0xBA,0xB5},3,'a'},
    {{0xE1,0xBA,0xB6},3,'A'},{{0xE1,0xBA,0xB7},3,'a'},
    {{0xE1,0xBA,0xB8},3,'E'},{{0xE1,0xBA,0xB9},3,'e'},
    {{0xE1,0xBA,0xBA},3,'E'},{{0xE1,0xBA,0xBB},3,'e'},
    {{0xE1,0xBA,0xBC},3,'E'},{{0xE1,0xBA,0xBD},3,'e'},
    {{0xE1,0xBA,0xBE},3,'E'},{{0xE1,0xBA,0xBF},3,'e'},
    {{0xE1,0xBB,0x80},3,'E'},{{0xE1,0xBB,0x81},3,'e'},
    {{0xE1,0xBB,0x82},3,'E'},{{0xE1,0xBB,0x83},3,'e'},
    {{0xE1,0xBB,0x84},3,'E'},{{0xE1,0xBB,0x85},3,'e'},
    {{0xE1,0xBB,0x86},3,'E'},{{0xE1,0xBB,0x87},3,'e'},
    {{0xE1,0xBB,0x88},3,'I'},{{0xE1,0xBB,0x89},3,'i'},
    {{0xE1,0xBB,0x8A},3,'I'},{{0xE1,0xBB,0x8B},3,'i'},
    {{0xE1,0xBB,0x8C},3,'O'},{{0xE1,0xBB,0x8D},3,'o'},
    {{0xE1,0xBB,0x8E},3,'O'},{{0xE1,0xBB,0x8F},3,'o'},
    {{0xE1,0xBB,0x90},3,'O'},{{0xE1,0xBB,0x91},3,'o'},
    {{0xE1,0xBB,0x92},3,'O'},{{0xE1,0xBB,0x93},3,'o'},
    {{0xE1,0xBB,0x94},3,'O'},{{0xE1,0xBB,0x95},3,'o'},
    {{0xE1,0xBB,0x96},3,'O'},{{0xE1,0xBB,0x97},3,'o'},
    {{0xE1,0xBB,0x98},3,'O'},{{0xE1,0xBB,0x99},3,'o'},
    {{0xE1,0xBB,0x9A},3,'O'},{{0xE1,0xBB,0x9B},3,'o'},
    {{0xE1,0xBB,0x9C},3,'O'},{{0xE1,0xBB,0x9D},3,'o'},
    {{0xE1,0xBB,0x9E},3,'O'},{{0xE1,0xBB,0x9F},3,'o'},
    {{0xE1,0xBB,0xA0},3,'O'},{{0xE1,0xBB,0xA1},3,'o'},
    {{0xE1,0xBB,0xA2},3,'O'},{{0xE1,0xBB,0xA3},3,'o'},
    {{0xE1,0xBB,0xA4},3,'U'},{{0xE1,0xBB,0xA5},3,'u'},
    {{0xE1,0xBB,0xA6},3,'U'},{{0xE1,0xBB,0xA7},3,'u'},
    {{0xE1,0xBB,0xA8},3,'U'},{{0xE1,0xBB,0xA9},3,'u'},
    {{0xE1,0xBB,0xAA},3,'U'},{{0xE1,0xBB,0xAB},3,'u'},
    {{0xE1,0xBB,0xAC},3,'U'},{{0xE1,0xBB,0xAD},3,'u'},
    {{0xE1,0xBB,0xAE},3,'U'},{{0xE1,0xBB,0xAF},3,'u'},
    {{0xE1,0xBB,0xB0},3,'U'},{{0xE1,0xBB,0xB1},3,'u'},
    {{0xE1,0xBB,0xB2},3,'Y'},{{0xE1,0xBB,0xB3},3,'y'},
    {{0xE1,0xBB,0xB4},3,'Y'},{{0xE1,0xBB,0xB5},3,'y'},
    {{0xE1,0xBB,0xB6},3,'Y'},{{0xE1,0xBB,0xB7},3,'y'},
    {{0xE1,0xBB,0xB8},3,'Y'},{{0xE1,0xBB,0xB9},3,'y'},
};

static void vn_strip(char *dst, size_t dsz, const char *src)
{
    size_t di = 0;
    const uint8_t *s = (const uint8_t *)src;
    while (*s && di < dsz - 1) {
        if (*s < 0x80) { dst[di++] = (char)*s++; continue; }
        uint8_t seq = (*s < 0xE0) ? 2u : (*s < 0xF0) ? 3u : 4u;
        if (seq <= 3) {
            for (size_t k = 0; k < sizeof(s_vn_map)/sizeof(s_vn_map[0]); k++) {
                if (s_vn_map[k].len == seq &&
                    s[0] == s_vn_map[k].bytes[0] && s[1] == s_vn_map[k].bytes[1] &&
                    (seq < 3 || s[2] == s_vn_map[k].bytes[2])) {
                    dst[di++] = s_vn_map[k].ascii;
                    break;
                }
            }
        }
        s += seq;
    }
    dst[di] = '\0';
}

/* ------------------------------------------------------------------ */
/* LVGL flush callback                                                 */
/* ------------------------------------------------------------------ */

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
    esp_lcd_panel_draw_bitmap(panel,
                              area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1,
                              color_map);
    lv_disp_flush_ready(drv);
}

/* ------------------------------------------------------------------ */
/* LVGL tick timer                                                     */
/* ------------------------------------------------------------------ */

static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

/* ------------------------------------------------------------------ */
/* Map widget — draw event callback                                    */
/* ------------------------------------------------------------------ */

static lv_color_t line_type_color(uint8_t line_type)
{
    switch (line_type) {
    case 0x01: return lv_color_hex(0x1A73E8); /* route chính       — xanh dương */
    case 0x02: return lv_color_hex(0xBDBDBD); /* đường xung quanh  — xám nhạt */
    case 0x03: return lv_color_hex(0x616161); /* đã đi             — xám đậm */
    default:   return lv_color_hex(0x1A73E8);
    }
}

static uint8_t line_type_width(uint8_t line_type)
{
    return (line_type == 0x01) ? 3 : 1;
}

static void map_draw_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) {
        return;
    }

    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    lv_coord_t x_ofs = obj->coords.x1;
    lv_coord_t y_ofs = obj->coords.y1;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    disp_map_data_t map_copy = s_map;
    xSemaphoreGive(s_mutex);

    for (uint8_t l = 0; l < map_copy.line_count; l++) {
        const disp_map_line_t *line = &map_copy.lines[l];
        if (line->point_count < 2) continue;

        lv_draw_line_dsc_t dsc;
        lv_draw_line_dsc_init(&dsc);
        dsc.color = line_type_color(line->line_type);
        dsc.width = line_type_width(line->line_type);
        dsc.round_start = 1;
        dsc.round_end   = 1;

        for (uint8_t p = 1; p < line->point_count; p++) {
            lv_point_t p1 = {
                x_ofs + line->points[(p - 1) * 2],
                y_ofs + line->points[(p - 1) * 2 + 1]
            };
            lv_point_t p2 = {
                x_ofs + line->points[p * 2],
                y_ofs + line->points[p * 2 + 1]
            };
            lv_draw_line(draw_ctx, &dsc, &p1, &p2);
        }
    }

    /* User dot — toạ độ cố định, mobile đã tính rotation và lookahead offset */
    lv_draw_rect_dsc_t dot_dsc;
    lv_draw_rect_dsc_init(&dot_dsc);
    dot_dsc.bg_color    = lv_color_hex(0xFFFFFF);
    dot_dsc.border_color = lv_color_hex(0x1A73E8);
    dot_dsc.border_width = 2;
    dot_dsc.radius       = LV_RADIUS_CIRCLE;
    lv_coord_t dot_r = 4;
    lv_area_t dot_area = {
        x_ofs + MAP_USER_X - dot_r,
        y_ofs + MAP_USER_Y - dot_r,
        x_ofs + MAP_USER_X + dot_r,
        y_ofs + MAP_USER_Y + dot_r,
    };
    lv_draw_rect(draw_ctx, &dot_dsc, &dot_area);
}

/* ------------------------------------------------------------------ */
/* Tạo UI                                                             */
/* ------------------------------------------------------------------ */

static lv_obj_t *make_label(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                             const lv_font_t *font, lv_color_t color,
                             const char *text)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_bg_opa(label, LV_OPA_TRANSP, 0);
    lv_label_set_text(label, text);
    return label;
}

static void create_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1C1C1E), 0); /* nền tối */
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    /* --- Topbar background --- */
    // lv_obj_t *topbar = lv_obj_create(scr);
    // lv_obj_set_pos(topbar, 0, 0);
    // lv_obj_set_size(topbar, LCD_H_RES, 59);
    // lv_obj_set_style_bg_opa(topbar, LV_OPA_TRANSP, 0);
    // lv_obj_set_style_border_width(topbar, 0, 0);
    // lv_obj_set_style_radius(topbar, 0, 0);
    // lv_obj_set_style_pad_all(topbar, 0, 0);
    // lv_obj_clear_flag(topbar, LV_OBJ_FLAG_SCROLLABLE);

    /* Turn icon box */
    lv_obj_t *turn_box = lv_obj_create(scr);
    lv_obj_set_pos(turn_box, TURN_ICON_X, TURN_ICON_Y);
    lv_obj_set_size(turn_box, TURN_ICON_W, TURN_ICON_H);
    lv_obj_set_style_bg_color(turn_box, lv_color_hex(0x1A73E8), 0);
    lv_obj_set_style_radius(turn_box, 6, 0);
    lv_obj_set_style_border_width(turn_box, 0, 0);
    lv_obj_set_style_pad_all(turn_box, 0, 0);
    lv_obj_clear_flag(turn_box, LV_OBJ_FLAG_SCROLLABLE);

    s_turn_icon_label = lv_label_create(turn_box);
    lv_obj_set_style_text_font(s_turn_icon_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_turn_icon_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(s_turn_icon_label, LV_OPA_TRANSP, 0);
    lv_label_set_text(s_turn_icon_label, LV_SYMBOL_UP);
    lv_obj_center(s_turn_icon_label);

    /* Distance to next turn — dòng 1: số, dòng 2: đơn vị, đều căn giữa */
    s_dist_label = lv_label_create(scr);
    lv_obj_set_pos(s_dist_label, DIST_LABEL_X, DIST_NUM_Y);
    lv_obj_set_width(s_dist_label, DIST_LABEL_W);
    lv_obj_set_style_text_font(s_dist_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_dist_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(s_dist_label, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_align(s_dist_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_dist_label, "--");

    s_dist_unit_label = lv_label_create(scr);
    lv_obj_set_pos(s_dist_unit_label, DIST_LABEL_X, DIST_UNIT_Y);
    lv_obj_set_width(s_dist_unit_label, DIST_LABEL_W);
    lv_obj_set_style_text_font(s_dist_unit_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_dist_unit_label, lv_color_hex(0xAEAEB2), 0);
    lv_obj_set_style_bg_opa(s_dist_unit_label, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_align(s_dist_unit_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_dist_unit_label, "m");

    /* Turn info (next street / instruction) */
    s_turn_info_label = lv_label_create(scr);
    lv_obj_set_pos(s_turn_info_label, TURN_INFO_X, TURN_INFO_Y);
    lv_obj_set_size(s_turn_info_label, TURN_INFO_W, TURN_INFO_H);
    lv_obj_set_style_text_font(s_turn_info_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_turn_info_label, lv_color_hex(0xC7C7CC), 0);
    lv_obj_set_style_bg_opa(s_turn_info_label, LV_OPA_TRANSP, 0);
    lv_label_set_long_mode(s_turn_info_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_turn_info_label, "");

    /* Progress bar */
    s_progress_bar = lv_bar_create(scr);
    lv_obj_set_pos(s_progress_bar, PROGRESS_X, PROGRESS_Y);
    lv_obj_set_size(s_progress_bar, PROGRESS_W, PROGRESS_H);
    lv_bar_set_range(s_progress_bar, 0, 1000);
    lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x3A3A3C), 0);
    lv_obj_set_style_bg_color(s_progress_bar, lv_color_hex(0x1A73E8),
                               LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_progress_bar, 2, 0);
    lv_obj_set_style_radius(s_progress_bar, 2, LV_PART_INDICATOR);

    /* --- Map area --- */
    s_map_obj = lv_obj_create(scr);
    lv_obj_set_pos(s_map_obj, MAP_X, MAP_Y);
    lv_obj_set_size(s_map_obj, MAP_W, MAP_H);
    lv_obj_set_style_bg_color(s_map_obj, lv_color_hex(0x2D4A22), 0); /* nền xanh lá nhạt */
    lv_obj_set_style_border_width(s_map_obj, 0, 0);
    lv_obj_set_style_radius(s_map_obj, 0, 0);
    lv_obj_set_style_pad_all(s_map_obj, 0, 0);
    lv_obj_clear_flag(s_map_obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_map_obj, map_draw_event_cb, LV_EVENT_DRAW_MAIN, NULL);

    /* --- Bottom bar background --- */
    // lv_obj_t *botbar = lv_obj_create(scr);
    // lv_obj_set_pos(botbar, 0, 233);
    // lv_obj_set_size(botbar, LCD_H_RES, 87);
    // lv_obj_set_style_bg_opa(botbar, LV_OPA_TRANSP, 0);
    // lv_obj_set_style_border_width(botbar, 0, 0);
    // lv_obj_set_style_radius(botbar, 0, 0);
    // lv_obj_set_style_pad_all(botbar, 0, 0);
    // lv_obj_clear_flag(botbar, LV_OBJ_FLAG_SCROLLABLE);

    /* Speed — cả số và "km/h" cùng chiều rộng + căn giữa để thẳng hàng */
    s_speed_label = lv_label_create(scr);
    lv_obj_set_pos(s_speed_label, SPEED_ZONE_X, SPEED_NUM_Y);
    lv_obj_set_width(s_speed_label, SPEED_ZONE_W);
    lv_obj_set_style_text_font(s_speed_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_speed_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(s_speed_label, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_align(s_speed_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_speed_label, "0");

    lv_obj_t *kmh_label = lv_label_create(scr);
    lv_obj_set_pos(kmh_label, SPEED_ZONE_X, KMH_Y);
    lv_obj_set_width(kmh_label, SPEED_ZONE_W);
    lv_obj_set_style_text_font(kmh_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(kmh_label, lv_color_hex(0xAEAEB2), 0);
    lv_obj_set_style_bg_opa(kmh_label, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_align(kmh_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(kmh_label, "km/h");

    /* Destination — icon, khoảng cách, thời gian đều căn giữa trong DEST_ZONE */
    lv_obj_t *dest_icon = lv_obj_create(scr);
    lv_obj_set_pos(dest_icon, DEST_ZONE_X + (DEST_ZONE_W - 28) / 2, DEST_ICON_Y);
    lv_obj_set_size(dest_icon, 28, 28);
    lv_obj_set_style_bg_color(dest_icon, lv_color_hex(0x3A3A3C), 0);
    lv_obj_set_style_radius(dest_icon, 4, 0);
    lv_obj_set_style_border_width(dest_icon, 0, 0);
    lv_obj_set_style_pad_all(dest_icon, 0, 0);
    lv_obj_clear_flag(dest_icon, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *dest_sym = lv_label_create(dest_icon);
    lv_obj_set_style_text_font(dest_sym, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(dest_sym, lv_color_hex(0xFFD60A), 0);
    lv_obj_set_style_bg_opa(dest_sym, LV_OPA_TRANSP, 0);
    lv_label_set_text(dest_sym, LV_SYMBOL_GPS);
    lv_obj_center(dest_sym);

    s_dest_dist_label = lv_label_create(scr);
    lv_obj_set_pos(s_dest_dist_label, DEST_ZONE_X, DEST_DIST_Y);
    lv_obj_set_width(s_dest_dist_label, DEST_ZONE_W);
    lv_obj_set_style_text_font(s_dest_dist_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_dest_dist_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(s_dest_dist_label, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_align(s_dest_dist_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_dest_dist_label, "--");

    s_dest_time_label = lv_label_create(scr);
    lv_obj_set_pos(s_dest_time_label, DEST_ZONE_X, DEST_TIME_Y);
    lv_obj_set_width(s_dest_time_label, DEST_ZONE_W);
    lv_obj_set_style_text_font(s_dest_time_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_dest_time_label, lv_color_hex(0xAEAEB2), 0);
    lv_obj_set_style_bg_opa(s_dest_time_label, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_align(s_dest_time_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_dest_time_label, "--");

    /* Warning border */
    lv_obj_t *warn_border = lv_obj_create(scr);
    lv_obj_set_pos(warn_border, WARN_BORDER_X, WARN_BORDER_Y);
    lv_obj_set_size(warn_border, WARN_BORDER_W, WARN_BORDER_H);
    lv_obj_set_style_bg_color(warn_border, lv_color_hex(0x3A3A3C), 0);
    lv_obj_set_style_border_color(warn_border, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_border_width(warn_border, 2, 0);
    lv_obj_set_style_radius(warn_border, 6, 0);
    lv_obj_set_style_pad_all(warn_border, 0, 0);
    lv_obj_clear_flag(warn_border, LV_OBJ_FLAG_SCROLLABLE);

    /* Warning image placeholder (sign symbol, centered in border) */
    s_warn_img_label = lv_label_create(warn_border);
    lv_obj_set_style_text_font(s_warn_img_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_warn_img_label, lv_color_hex(0xFF9F0A), 0);
    lv_obj_set_style_bg_opa(s_warn_img_label, LV_OPA_TRANSP, 0);
    lv_label_set_text(s_warn_img_label, LV_SYMBOL_WARNING);
    lv_obj_align(s_warn_img_label, LV_ALIGN_TOP_MID, 0, 8);

    /* Warning text */
    s_warn_text_label = lv_label_create(warn_border);
    lv_obj_set_style_text_font(s_warn_text_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_warn_text_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(s_warn_text_label, LV_OPA_TRANSP, 0);
    lv_label_set_long_mode(s_warn_text_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(s_warn_text_label, WARN_BORDER_W - 4);
    lv_label_set_text(s_warn_text_label, "");
    lv_obj_align(s_warn_text_label, LV_ALIGN_BOTTOM_MID, 0, -4);

    /* Status bar: battery */
    s_bat_label = lv_label_create(scr);
    lv_obj_set_pos(s_bat_label, BAT_ICON_X, BAT_ICON_Y);
    lv_obj_set_style_text_font(s_bat_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_bat_label, lv_color_hex(0x30D158), 0);
    lv_obj_set_style_bg_opa(s_bat_label, LV_OPA_TRANSP, 0);
    lv_label_set_text(s_bat_label, LV_SYMBOL_BATTERY_FULL);

    /* Status bar: connection */
    s_conn_label = lv_label_create(scr);
    lv_obj_set_pos(s_conn_label, CONN_ICON_X, CONN_ICON_Y);
    lv_obj_set_style_text_font(s_conn_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_conn_label, lv_color_hex(0x636366), 0);
    lv_obj_set_style_bg_opa(s_conn_label, LV_OPA_TRANSP, 0);
    lv_label_set_text(s_conn_label, LV_SYMBOL_BLUETOOTH);

    /* Status bar: time */
    s_time_label = make_label(scr, TIME_LABEL_X, TIME_LABEL_Y,
                               &lv_font_montserrat_18, lv_color_hex(0xFFFFFF), "--:--");
}

/* ------------------------------------------------------------------ */
/* Áp dụng pending updates vào LVGL widgets (chạy trong LVGL task)   */
/* ------------------------------------------------------------------ */

static void apply_nav_update(void)
{
    disp_nav_data_t nav;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    nav = s_nav;
    s_pending_nav = false;
    xSemaphoreGive(s_mutex);

    /* Turn icon */
    lv_label_set_text(s_turn_icon_label, direction_to_arrow(nav.direction));

    /* Distance to next turn — tách số (dòng trên) và đơn vị (dòng dưới) */
    char buf[32];
    fmt_dist_num(buf, sizeof(buf), nav.distance_m);
    lv_label_set_text(s_dist_label, buf);
    lv_label_set_text(s_dist_unit_label, fmt_dist_unit(nav.distance_m));

    /* Next direction instruction — bỏ dấu tiếng Việt để font hiển thị đúng */
    char ascii_dir[64];
    vn_strip(ascii_dir, sizeof(ascii_dir), nav.next_direction);
    lv_label_set_text(s_turn_info_label, ascii_dir);

    /* Progress bar */
    if (nav.route_progress_permille > 0) {
        lv_bar_set_value(s_progress_bar, nav.route_progress_permille, LV_ANIM_OFF);
    }

    /* Speed: chuyển m/s → km/h */
    uint32_t kmh = ((uint32_t)nav.current_speed_mps * 36) / 10;
    snprintf(buf, sizeof(buf), "%" PRIu32, kmh);
    lv_label_set_text(s_speed_label, buf);

    /* Destination distance */
    fmt_distance(buf, sizeof(buf), nav.destination_distance_m);
    lv_label_set_text(s_dest_dist_label, buf);

    /* Remaining time */
    fmt_duration(buf, sizeof(buf), nav.remaining_time_minutes);
    lv_label_set_text(s_dest_time_label, buf);

    /* Current time */
    if (nav.current_time_epoch_seconds > 0) {
        time_t t = (time_t)nav.current_time_epoch_seconds + 7 * 3600; /* UTC+7 */
        struct tm tm_info;
        gmtime_r(&t, &tm_info);
        snprintf(buf, sizeof(buf), "%02d:%02d", tm_info.tm_hour, tm_info.tm_min);
        lv_label_set_text(s_time_label, buf);
    }
}

static void apply_map_update(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_pending_map = false;
    xSemaphoreGive(s_mutex);

    lv_obj_invalidate(s_map_obj);
}

static void apply_sign_update(void)
{
    disp_sign_data_t sign;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    sign = s_sign;
    s_pending_sign = false;
    xSemaphoreGive(s_mutex);

    char ascii_sign[64];
    vn_strip(ascii_sign, sizeof(ascii_sign), sign.text);
    lv_label_set_text(s_warn_text_label, ascii_sign);
    /* sign_type: tạm dùng WARNING symbol cho tất cả; có thể mở rộng sau */
    lv_label_set_text(s_warn_img_label, LV_SYMBOL_WARNING);
}

static void apply_connection_update(void)
{
    bool connected;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    connected = s_connected;
    s_pending_connection = false;
    xSemaphoreGive(s_mutex);

    if (connected) {
        lv_obj_set_style_text_color(s_conn_label, lv_color_hex(0x0A84FF), 0);
    } else {
        lv_obj_set_style_text_color(s_conn_label, lv_color_hex(0x636366), 0);
    }
}

static void process_pending_updates(void)
{
    if (s_pending_nav)        apply_nav_update();
    if (s_pending_map)        apply_map_update();
    if (s_pending_sign)       apply_sign_update();
    if (s_pending_connection) apply_connection_update();
    /* s_pending_nav_img: xử lý khi cần (ảnh từ điện thoại gửi qua NAV_IMAGE) */
}

/* ------------------------------------------------------------------ */
/* ST7789 init                                                         */
/* ------------------------------------------------------------------ */

static void st7789_hw_init(void)
{
    /* SPI bus */
    spi_bus_config_t buscfg = {
        .mosi_io_num     = LCD_MOSI_PIN,
        .miso_io_num     = -1,
        .sclk_io_num     = LCD_SCLK_PIN,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * LVGL_DRAW_BUF_LINES * LCD_BITS_PER_PX / 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    /* Panel IO */
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num       = LCD_DC_PIN,
        .cs_gpio_num       = LCD_CS_PIN,
        .pclk_hz           = LCD_CLK_HZ,
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
        .spi_mode          = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    /* ST7789 panel */
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST_PIN,
        .rgb_endian     = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = LCD_BITS_PER_PX,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &s_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true)); /* ST7789 thường cần invert */
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, false, false)); // lật màn hình 
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    /* Backlight */
#if CONFIG_LCD_BL_GPIO >= 0
    gpio_config_t bl_cfg = {
        .pin_bit_mask = 1ULL << CONFIG_LCD_BL_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&bl_cfg);
    gpio_set_level(CONFIG_LCD_BL_GPIO, 1);
#endif
}

/* ------------------------------------------------------------------ */
/* LVGL task                                                           */
/* ------------------------------------------------------------------ */

static void lvgl_task(void *arg)
{
    (void)arg;

    st7789_hw_init();

    /* LVGL init */
    lv_init();

    /* Draw buffer */
    static lv_disp_draw_buf_t disp_buf;
    lv_disp_draw_buf_init(&disp_buf, s_buf1, s_buf2, LCD_H_RES * LVGL_DRAW_BUF_LINES);

    /* Display driver */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res   = LCD_H_RES;
    disp_drv.ver_res   = LCD_V_RES;
    disp_drv.flush_cb  = lvgl_flush_cb;
    disp_drv.draw_buf  = &disp_buf;
    disp_drv.user_data = s_panel;
    lv_disp_drv_register(&disp_drv);

    /* LVGL tick timer (esp_timer) */
    esp_timer_handle_t tick_timer;
    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .name     = "lv_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    /* Tạo UI */
    create_ui();

    /* --- TEST: xóa sau khi xác nhận LCD hoạt động --- */
    // lv_obj_t *scr = lv_scr_act();
    // lv_obj_set_style_bg_color(scr, lv_color_hex(0xFF0000), 0);
    // lv_obj_t *test_lbl = lv_label_create(scr);
    // lv_obj_set_style_text_font(test_lbl, &lv_font_montserrat_24, 0);
    // lv_obj_set_style_text_color(test_lbl, lv_color_hex(0xFFFFFF), 0);
    // lv_obj_set_style_bg_opa(test_lbl, LV_OPA_TRANSP, 0);
    // lv_label_set_text(test_lbl, "HELLO");
    // lv_obj_center(test_lbl);
    /* --- END TEST --- */

    s_display_ready = true;
    ESP_LOGI(TAG, "Display ready");

    while (1) {
        process_pending_updates();
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(LVGL_TICK_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void display_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    xTaskCreate(lvgl_task, "lvgl",
                LVGL_TASK_STACK_KB * 1024,
                NULL,
                LVGL_TASK_PRIORITY,
                NULL);
}

void display_update_nav(const nus_proto_nav_instruction_t *nav)
{
    if (!nav || !s_display_ready) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Copy text (không null-terminated trong protocol) */
    uint8_t len = nav->direction.len < sizeof(s_nav.direction) - 1
                      ? nav->direction.len
                      : (uint8_t)(sizeof(s_nav.direction) - 1);
    memcpy(s_nav.direction, nav->direction.data, len);
    s_nav.direction[len] = '\0';

    len = nav->next_direction.len < sizeof(s_nav.next_direction) - 1
              ? nav->next_direction.len
              : (uint8_t)(sizeof(s_nav.next_direction) - 1);
    memcpy(s_nav.next_direction, nav->next_direction.data, len);
    s_nav.next_direction[len] = '\0';

    s_nav.distance_m                = nav->distance_m;
    s_nav.destination_distance_m    = nav->destination_distance_m;
    s_nav.remaining_time_minutes    = nav->remaining_time_minutes;
    s_nav.current_speed_mps         = nav->current_speed_mps;
    s_nav.current_time_epoch_seconds = nav->current_time_epoch_seconds;
    s_nav.route_progress_permille   = nav->route_progress_permille;
    s_pending_nav = true;

    xSemaphoreGive(s_mutex);
}

void display_update_nav_image(const nus_proto_nav_image_t *img)
{
    if (!img || !s_display_ready) return;
    if (img->data_len == 0 || img->data == NULL) return;

    uint16_t copy_len = img->data_len < DISP_NAV_IMG_MAX ? img->data_len : DISP_NAV_IMG_MAX;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_nav_img.image_type = img->image_type;
    s_nav_img.width      = img->width;
    s_nav_img.height     = img->height;
    s_nav_img.data_len   = copy_len;
    memcpy(s_nav_img.data, img->data, copy_len);
    s_pending_nav_img = true;
    xSemaphoreGive(s_mutex);
    /* TODO: render ảnh vào vùng turn icon khi cần */
}

void display_update_map_lines(const nus_proto_map_lines_t *lines)
{
    if (!lines || !s_display_ready) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_map.line_count = lines->line_count < DISP_MAP_MAX_LINES
                           ? lines->line_count
                           : DISP_MAP_MAX_LINES;
    for (uint8_t l = 0; l < s_map.line_count; l++) {
        const nus_proto_map_line_t *src = &lines->lines[l];
        disp_map_line_t *dst = &s_map.lines[l];
        dst->line_type   = src->line_type;
        dst->point_count = src->point_count < DISP_MAP_MAX_PTS
                               ? src->point_count
                               : DISP_MAP_MAX_PTS;
        memcpy(dst->points, src->points, dst->point_count * 2);
    }
    s_pending_map = true;
    xSemaphoreGive(s_mutex);
}

void display_update_traffic_sign(const nus_proto_traffic_sign_t *sign)
{
    if (!sign || !s_display_ready) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_sign.sign_type = sign->sign_type;
    uint16_t len = sign->data_len < sizeof(s_sign.text) - 1
                       ? sign->data_len
                       : (uint16_t)(sizeof(s_sign.text) - 1);
    memcpy(s_sign.text, sign->data, len);
    s_sign.text[len] = '\0';
    s_pending_sign = true;
    xSemaphoreGive(s_mutex);
}

void display_set_connected(bool connected)
{
    if (!s_display_ready) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_connected = connected;
    s_pending_connection = true;
    xSemaphoreGive(s_mutex);
}
