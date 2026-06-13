/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "nus.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NUS_PROTOCOL_SOF 0xAC
#define NUS_PROTOCOL_HEADER_LEN 5
#define NUS_PROTOCOL_CRC_LEN 2
#define NUS_PROTOCOL_FRAME_OVERHEAD (NUS_PROTOCOL_HEADER_LEN + NUS_PROTOCOL_CRC_LEN)
#define NUS_PROTOCOL_MAX_FRAME_LEN NUS_MAX_DATA_LEN
#define NUS_PROTOCOL_MAX_PAYLOAD_LEN (NUS_PROTOCOL_MAX_FRAME_LEN - NUS_PROTOCOL_FRAME_OVERHEAD)
#define NUS_PROTO_MANUFACTURER_ID_LEN 16
#define NUS_PROTO_SERIAL_NUMBER_LEN 32
#define NUS_PROTO_DEVICE_INFO_PAYLOAD_LEN 64

#define NUS_PROTO_TEXT_LITERAL(text_literal) { \
    .data = (text_literal), \
    .len = (uint8_t)(sizeof(text_literal) - 1), \
}

typedef enum {
    NUS_PROTO_TYPE_REQUEST = 0x00,
    NUS_PROTO_TYPE_RESPONSE = 0x01,
    NUS_PROTO_TYPE_EVENT = 0x02,
    NUS_PROTO_TYPE_COMMAND = 0x03,
    NUS_PROTO_TYPE_ACK = 0x04,
} nus_proto_msg_type_t;

typedef enum {
    NUS_PROTO_CMD_NAV_INSTRUCTION = 0x01,
    NUS_PROTO_CMD_NAV_IMAGE = 0x02,
    NUS_PROTO_CMD_TRAFFIC_SIGN = 0x03,
    NUS_PROTO_CMD_DEVICE_INFO = 0x04,
    NUS_PROTO_CMD_CURRENT_TIME = 0x05,
    NUS_PROTO_CMD_FILE_TRANSFER = 0x06,
    NUS_PROTO_CMD_OTA = 0x07,
} nus_proto_cmd_t;

typedef enum {
    NUS_PROTO_STATUS_OK = 0x00,
    NUS_PROTO_STATUS_INVALID_FRAME = 0x01,
    NUS_PROTO_STATUS_INVALID_TYPE = 0x02,
    NUS_PROTO_STATUS_INVALID_PAYLOAD = 0x03,
    NUS_PROTO_STATUS_UNSUPPORTED_CMD = 0x04,
    NUS_PROTO_STATUS_APP_ERROR = 0x05,
    NUS_PROTO_STATUS_TX_FAILED = 0x06,
} nus_proto_status_t;

typedef enum {
    NUS_PROTO_PARSE_ERROR_FRAME_TOO_LONG,
    NUS_PROTO_PARSE_ERROR_BAD_CRC,
    NUS_PROTO_PARSE_ERROR_INVALID_TYPE,
} nus_proto_parse_error_t;

typedef struct {
    /* Views are not null-terminated and are valid only for the callback scope. */
    const char *data;
    uint8_t len;
} nus_proto_text_t;

typedef struct {
    /* Payload points into the parser RX buffer and is valid only for the callback scope. */
    uint8_t cmd;
    nus_proto_msg_type_t type;
    const uint8_t *payload;
    uint16_t payload_len;
    uint16_t crc;
} nus_proto_frame_t;

typedef struct {
    nus_proto_text_t direction;
    uint32_t distance_m;
    nus_proto_text_t next_direction;
    uint32_t destination_distance_m;
    uint32_t remaining_time_minutes;
    uint16_t current_speed_mps;
    uint32_t current_time_epoch_seconds;
} nus_proto_nav_instruction_t;

typedef struct {
    uint8_t image_type;
    uint16_t width;
    uint16_t height;
    const uint8_t *data;
    uint16_t data_len;
} nus_proto_nav_image_t;

typedef struct {
    uint8_t sign_type;
    const uint8_t *data;
    uint16_t data_len;
} nus_proto_traffic_sign_t;

typedef struct {
    uint32_t hardware_version;
    uint32_t firmware_version;
    uint8_t manufacturer_id[NUS_PROTO_MANUFACTURER_ID_LEN];
    uint8_t serial_number[NUS_PROTO_SERIAL_NUMBER_LEN];
    uint32_t product_id;
    uint32_t model_id;
} nus_proto_device_info_t;

typedef struct {
    uint32_t epoch_seconds;
} nus_proto_current_time_t;

typedef struct {
    uint32_t file_size;
    uint32_t offset;
    const uint8_t *data;
    uint16_t data_len;
} nus_proto_file_transfer_t;

typedef esp_err_t (*nus_proto_transport_send_cb_t)(const uint8_t *data,
                                                   uint16_t len,
                                                   TickType_t ticks_to_wait,
                                                   void *user_ctx);

typedef void (*nus_proto_frame_cb_t)(const nus_proto_frame_t *frame, void *user_ctx);
typedef void (*nus_proto_ack_cb_t)(uint8_t cmd,
                                   nus_proto_status_t status,
                                   const uint8_t *extra,
                                   uint16_t extra_len,
                                   void *user_ctx);
typedef void (*nus_proto_parse_error_cb_t)(nus_proto_parse_error_t error,
                                           const uint8_t *data,
                                           uint16_t len,
                                           void *user_ctx);
typedef void (*nus_proto_tx_error_cb_t)(esp_err_t error,
                                        uint8_t cmd,
                                        nus_proto_msg_type_t type,
                                        void *user_ctx);

typedef nus_proto_status_t (*nus_proto_nav_instruction_cb_t)(const nus_proto_nav_instruction_t *message,
                                                             void *user_ctx);
typedef nus_proto_status_t (*nus_proto_nav_image_cb_t)(const nus_proto_nav_image_t *message,
                                                       void *user_ctx);
typedef nus_proto_status_t (*nus_proto_traffic_sign_cb_t)(const nus_proto_traffic_sign_t *message,
                                                          void *user_ctx);
typedef nus_proto_status_t (*nus_proto_device_info_request_cb_t)(nus_proto_device_info_t *response,
                                                                 void *user_ctx);
typedef nus_proto_status_t (*nus_proto_current_time_cb_t)(const nus_proto_current_time_t *message,
                                                         void *user_ctx);
typedef nus_proto_status_t (*nus_proto_file_transfer_cb_t)(const nus_proto_file_transfer_t *message,
                                                           void *user_ctx);
typedef nus_proto_status_t (*nus_proto_ota_cb_t)(const nus_proto_frame_t *frame,
                                                 void *user_ctx);
typedef nus_proto_status_t (*nus_proto_unknown_cb_t)(const nus_proto_frame_t *frame,
                                                     void *user_ctx);

typedef struct {
    nus_proto_frame_cb_t on_frame;
    nus_proto_frame_cb_t on_response;
    nus_proto_ack_cb_t on_ack;
    nus_proto_parse_error_cb_t on_parse_error;
    nus_proto_tx_error_cb_t on_tx_error;

    nus_proto_nav_instruction_cb_t on_nav_instruction;
    nus_proto_nav_image_cb_t on_nav_image;
    nus_proto_traffic_sign_cb_t on_traffic_sign;
    nus_proto_device_info_request_cb_t on_device_info_request;
    nus_proto_current_time_cb_t on_current_time;
    nus_proto_file_transfer_cb_t on_file_transfer;
    nus_proto_ota_cb_t on_ota;
    nus_proto_unknown_cb_t on_unknown;
} nus_protocol_callbacks_t;

typedef struct {
    nus_proto_transport_send_cb_t send_cb;
    void *user_ctx;
    TickType_t tx_wait_ticks;
    bool auto_ack;
} nus_protocol_config_t;

#define NUS_PROTOCOL_CONFIG_DEFAULT(send_fn, ctx) { \
    .send_cb = (send_fn), \
    .user_ctx = (ctx), \
    .tx_wait_ticks = pdMS_TO_TICKS(100), \
    .auto_ack = true, \
}

typedef struct {
    uint8_t rx_buf[NUS_PROTOCOL_MAX_FRAME_LEN];
    uint16_t rx_len;
    uint16_t expected_len;
    nus_protocol_callbacks_t callbacks;
    nus_proto_transport_send_cb_t send_cb;
    void *user_ctx;
    TickType_t tx_wait_ticks;
    bool auto_ack;
} nus_protocol_t;

esp_err_t nus_protocol_init(nus_protocol_t *protocol,
                            const nus_protocol_config_t *config,
                            const nus_protocol_callbacks_t *callbacks);
void nus_protocol_reset(nus_protocol_t *protocol);
esp_err_t nus_protocol_input(nus_protocol_t *protocol, const uint8_t *data, uint16_t len);

uint16_t nus_protocol_crc16_mcrf4xx(const uint8_t *data, uint16_t len);
esp_err_t nus_protocol_pack_frame(uint8_t cmd,
                                  nus_proto_msg_type_t type,
                                  const uint8_t *payload,
                                  uint16_t payload_len,
                                  uint8_t *out,
                                  uint16_t out_size,
                                  uint16_t *written);
esp_err_t nus_protocol_send_frame(nus_protocol_t *protocol,
                                  uint8_t cmd,
                                  nus_proto_msg_type_t type,
                                  const uint8_t *payload,
                                  uint16_t payload_len);
esp_err_t nus_protocol_send_ack(nus_protocol_t *protocol,
                                uint8_t cmd,
                                nus_proto_status_t status);
esp_err_t nus_protocol_send_device_info_response(nus_protocol_t *protocol,
                                                 const nus_proto_device_info_t *info);

esp_err_t nus_protocol_parse_nav_instruction_payload(const uint8_t *payload,
                                                     uint16_t payload_len,
                                                     nus_proto_nav_instruction_t *out);
esp_err_t nus_protocol_parse_nav_image_payload(const uint8_t *payload,
                                               uint16_t payload_len,
                                               nus_proto_nav_image_t *out);
esp_err_t nus_protocol_parse_traffic_sign_payload(const uint8_t *payload,
                                                  uint16_t payload_len,
                                                  nus_proto_traffic_sign_t *out);
esp_err_t nus_protocol_parse_device_info_payload(const uint8_t *payload,
                                                 uint16_t payload_len,
                                                 nus_proto_device_info_t *out);
esp_err_t nus_protocol_parse_current_time_payload(const uint8_t *payload,
                                                  uint16_t payload_len,
                                                  nus_proto_current_time_t *out);
esp_err_t nus_protocol_parse_file_transfer_payload(const uint8_t *payload,
                                                   uint16_t payload_len,
                                                   nus_proto_file_transfer_t *out);

esp_err_t nus_protocol_pack_nav_instruction_payload(const nus_proto_nav_instruction_t *message,
                                                    uint8_t *out,
                                                    uint16_t out_size,
                                                    uint16_t *written);
esp_err_t nus_protocol_pack_nav_image_payload(const nus_proto_nav_image_t *message,
                                              uint8_t *out,
                                              uint16_t out_size,
                                              uint16_t *written);
esp_err_t nus_protocol_pack_traffic_sign_payload(const nus_proto_traffic_sign_t *message,
                                                 uint8_t *out,
                                                 uint16_t out_size,
                                                 uint16_t *written);
esp_err_t nus_protocol_pack_device_info_payload(const nus_proto_device_info_t *info,
                                                uint8_t *out,
                                                uint16_t out_size,
                                                uint16_t *written);
esp_err_t nus_protocol_pack_current_time_payload(const nus_proto_current_time_t *message,
                                                 uint8_t *out,
                                                 uint16_t out_size,
                                                 uint16_t *written);
esp_err_t nus_protocol_pack_file_transfer_payload(const nus_proto_file_transfer_t *message,
                                                  uint8_t *out,
                                                  uint16_t out_size,
                                                  uint16_t *written);

#ifdef __cplusplus
}
#endif
