/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "nus_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Khởi tạo ST7789 + LVGL và tạo task LVGL nền. Gọi một lần từ app_main(). */
void display_init(void);

/* Các hàm sau thread-safe — có thể gọi từ BLE callback hoặc bất kỳ task nào.
 * Dữ liệu được copy ngay trong lời gọi, không giữ con trỏ payload. */
void display_update_nav(const nus_proto_nav_instruction_t *nav);
void display_update_nav_image(const nus_proto_nav_image_t *img);
void display_update_map_lines(const nus_proto_map_lines_t *lines);
void display_update_traffic_sign(const nus_proto_traffic_sign_t *sign);
void display_set_connected(bool connected);

#ifdef __cplusplus
}
#endif
