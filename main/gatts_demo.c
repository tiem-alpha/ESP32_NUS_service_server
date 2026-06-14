/*
 * SPDX-FileCopyrightText: 2021-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <inttypes.h>
#include <stdint.h>

#include "display.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nus.h"
#include "nus_protocol.h"

#define APP_TAG "APP"

static nus_protocol_t s_protocol;

/* ------------------------------------------------------------------ */
/* Protocol transport                                                  */
/* ------------------------------------------------------------------ */

static esp_err_t app_proto_send_cb(const uint8_t *data,
                                   uint16_t len,
                                   TickType_t ticks_to_wait,
                                   void *user_ctx)
{
    (void)user_ctx;
    return nus_send(data, len, ticks_to_wait);
}

/* ------------------------------------------------------------------ */
/* Protocol diagnostic callbacks                                       */
/* ------------------------------------------------------------------ */

static void app_proto_frame_cb(const nus_proto_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGD(APP_TAG,
             "Frame cmd=0x%02x type=0x%02x payload=%u crc=0x%04x",
             frame->cmd,
             frame->type,
             frame->payload_len,
             frame->crc);
}

static void app_proto_ack_cb(uint8_t cmd,
                             nus_proto_status_t status,
                             const uint8_t *extra,
                             uint16_t extra_len,
                             void *user_ctx)
{
    (void)extra;
    (void)extra_len;
    (void)user_ctx;
    ESP_LOGD(APP_TAG, "ACK cmd=0x%02x status=0x%02x", cmd, status);
}

static void app_proto_parse_error_cb(nus_proto_parse_error_t error,
                                     const uint8_t *data,
                                     uint16_t len,
                                     void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGW(APP_TAG, "Parse error=%d len=%u", error, len);
    if (data && len > 0) {
        ESP_LOG_BUFFER_HEX(APP_TAG, data, len);
    }
}

static void app_proto_tx_error_cb(esp_err_t error,
                                  uint8_t cmd,
                                  nus_proto_msg_type_t type,
                                  void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGW(APP_TAG,
             "TX failed cmd=0x%02x type=0x%02x: %s",
             cmd,
             type,
             esp_err_to_name(error));
}

/* ------------------------------------------------------------------ */
/* Navigation callbacks → display                                      */
/* ------------------------------------------------------------------ */

static nus_proto_status_t app_proto_nav_instruction_cb(const nus_proto_nav_instruction_t *message,
                                                       void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGI(APP_TAG,
             "NAV dir=%.*s dist=%" PRIu32 "m speed=%u m/s dest=%" PRIu32 "m remain=%" PRIu32 "min",
             (int)message->direction.len,
             message->direction.data,
             message->distance_m,
             message->current_speed_mps,
             message->destination_distance_m,
             message->remaining_time_minutes);

    display_update_nav(message);
    return NUS_PROTO_STATUS_OK;
}

static nus_proto_status_t app_proto_nav_image_cb(const nus_proto_nav_image_t *message,
                                                 void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGI(APP_TAG,
             "NAV image type=%u size=%ux%u data=%u bytes",
             message->image_type,
             message->width,
             message->height,
             message->data_len);

    display_update_nav_image(message);
    return NUS_PROTO_STATUS_OK;
}

static nus_proto_status_t app_proto_traffic_sign_cb(const nus_proto_traffic_sign_t *message,
                                                    void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGI(APP_TAG,
             "Traffic sign type=%u data=%u bytes",
             message->sign_type,
             message->data_len);

    display_update_traffic_sign(message);
    return NUS_PROTO_STATUS_OK;
}

static nus_proto_status_t app_proto_map_lines_cb(const nus_proto_map_lines_t *message,
                                                 void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGD(APP_TAG, "MAP_LINES line_count=%u", message->line_count);

    display_update_map_lines(message);
    return NUS_PROTO_STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* Device info / time / file / OTA callbacks                          */
/* ------------------------------------------------------------------ */

static nus_proto_status_t app_proto_device_info_cb(nus_proto_device_info_t *response,
                                                   void *user_ctx)
{
    (void)user_ctx;
    *response = (nus_proto_device_info_t) {
        .hardware_version = 0x00000001,
        .firmware_version = 0x00010000,
        .product_id       = 0x00000001,
        .model_id         = 0x00000001, /* phone dùng model_id để tra map_w × map_h */
    };
    memcpy(response->manufacturer_id, "TIEM", 4);
    memcpy(response->serial_number,   "SN000001", 8);
    return NUS_PROTO_STATUS_OK;
}

static nus_proto_status_t app_proto_current_time_cb(const nus_proto_current_time_t *message,
                                                    void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGI(APP_TAG, "Current time epoch=%" PRIu32, message->epoch_seconds);
    return NUS_PROTO_STATUS_OK;
}

static nus_proto_status_t app_proto_file_transfer_cb(const nus_proto_file_transfer_t *message,
                                                     void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGI(APP_TAG,
             "File chunk size=%" PRIu32 " offset=%" PRIu32 " data=%u bytes",
             message->file_size,
             message->offset,
             message->data_len);
    return NUS_PROTO_STATUS_OK;
}

static nus_proto_status_t app_proto_ota_cb(const nus_proto_frame_t *frame, void *user_ctx)
{
    (void)frame;
    (void)user_ctx;
    ESP_LOGW(APP_TAG, "OTA not implemented");
    return NUS_PROTO_STATUS_UNSUPPORTED_CMD;
}

static nus_proto_status_t app_proto_unknown_cb(const nus_proto_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGW(APP_TAG,
             "Unknown cmd=0x%02x type=0x%02x",
             frame->cmd,
             frame->type);
    return NUS_PROTO_STATUS_UNSUPPORTED_CMD;
}

/* ------------------------------------------------------------------ */
/* NUS state / RX callbacks                                            */
/* ------------------------------------------------------------------ */

static void app_nus_rx_cb(const uint8_t *data, uint16_t len)
{
    esp_err_t err = nus_protocol_input(&s_protocol, data, len);
    if (err != ESP_OK) {
        ESP_LOGW(APP_TAG, "Protocol input: %s", esp_err_to_name(err));
    }
}

static void app_nus_state_cb(nus_state_event_t event)
{
    switch (event) {
    case NUS_STATE_ADV_STARTED:
        ESP_LOGI(APP_TAG, "Advertising started");
        break;
    case NUS_STATE_ADV_STOPPED:
        ESP_LOGI(APP_TAG, "Advertising stopped");
        break;
    case NUS_STATE_ADV_TIMEOUT:
        ESP_LOGI(APP_TAG, "Advertising timeout");
        break;
    case NUS_STATE_CONNECTED:
        ESP_LOGI(APP_TAG, "Connected");
        display_set_connected(true);
        break;
    case NUS_STATE_DISCONNECTED:
        ESP_LOGI(APP_TAG, "Disconnected");
        display_set_connected(false);
        break;
    case NUS_STATE_NOTIFY_ENABLED:
        ESP_LOGI(APP_TAG, "Notify enabled");
        break;
    case NUS_STATE_NOTIFY_DISABLED:
        ESP_LOGI(APP_TAG, "Notify disabled");
        break;
    case NUS_STATE_IDLE_TIMEOUT:
        ESP_LOGI(APP_TAG, "Idle timeout");
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    /* Khởi tạo display và LVGL task trước khi BLE */
    display_init();

    /* NUS config */
    nus_config_t nus_config = NUS_CONFIG_DEFAULT();
    nus_config.device_name               = "Tiem NUS";
    nus_config.adv_interval_min_ms       = 80;
    nus_config.adv_interval_max_ms       = 120;
    nus_config.adv_timeout_ms            = 0;
    nus_config.conn_interval_min_ms      = 20;
    nus_config.conn_interval_max_ms      = 40;
    nus_config.conn_timeout_ms           = 4000;
    nus_config.idle_timeout_ms           = 0;
    nus_config.auto_start_adv            = true;
    nus_config.restart_adv_after_disconnect = true;

    /* Protocol config */
    nus_protocol_config_t proto_config = NUS_PROTOCOL_CONFIG_DEFAULT(app_proto_send_cb, NULL);
    proto_config.tx_wait_ticks = 0;
    proto_config.auto_ack      = true;

    const nus_protocol_callbacks_t proto_callbacks = {
        .on_frame              = app_proto_frame_cb,
        .on_ack                = app_proto_ack_cb,
        .on_parse_error        = app_proto_parse_error_cb,
        .on_tx_error           = app_proto_tx_error_cb,
        .on_nav_instruction    = app_proto_nav_instruction_cb,
        .on_nav_image          = app_proto_nav_image_cb,
        .on_traffic_sign       = app_proto_traffic_sign_cb,
        .on_device_info_request = app_proto_device_info_cb,
        .on_current_time       = app_proto_current_time_cb,
        .on_file_transfer      = app_proto_file_transfer_cb,
        .on_ota                = app_proto_ota_cb,
        .on_map_lines          = app_proto_map_lines_cb,
        .on_unknown            = app_proto_unknown_cb,
    };
    ESP_ERROR_CHECK(nus_protocol_init(&s_protocol, &proto_config, &proto_callbacks));

    nus_set_state_callback(app_nus_state_cb);
    ESP_ERROR_CHECK(nus_init_with_config(&nus_config, app_nus_rx_cb));
}
