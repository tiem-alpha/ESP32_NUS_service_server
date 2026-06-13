/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "nus_protocol.h"

#include <stddef.h>
#include <string.h>

#include "esp_log.h"

#define NUS_PROTO_TAG "NUS_PROTO"
#define NUS_PROTO_TYPE_MASK(type) (1U << (uint8_t)(type))

typedef esp_err_t (*nus_proto_dispatch_cb_t)(nus_protocol_t *protocol,
                                             const nus_proto_frame_t *frame);

typedef struct {
    uint8_t cmd;
    uint8_t type_mask;
    nus_proto_dispatch_cb_t dispatch;
} nus_proto_dispatch_entry_t;

static esp_err_t nus_proto_dispatch_nav_instruction(nus_protocol_t *protocol,
                                                    const nus_proto_frame_t *frame);
static esp_err_t nus_proto_dispatch_nav_image(nus_protocol_t *protocol,
                                              const nus_proto_frame_t *frame);
static esp_err_t nus_proto_dispatch_traffic_sign(nus_protocol_t *protocol,
                                                 const nus_proto_frame_t *frame);
static esp_err_t nus_proto_dispatch_device_info(nus_protocol_t *protocol,
                                                const nus_proto_frame_t *frame);
static esp_err_t nus_proto_dispatch_current_time(nus_protocol_t *protocol,
                                                 const nus_proto_frame_t *frame);
static esp_err_t nus_proto_dispatch_file_transfer(nus_protocol_t *protocol,
                                                  const nus_proto_frame_t *frame);
static esp_err_t nus_proto_dispatch_ota(nus_protocol_t *protocol,
                                        const nus_proto_frame_t *frame);

static const nus_proto_dispatch_entry_t s_dispatch_table[] = {
    {
        .cmd = NUS_PROTO_CMD_NAV_INSTRUCTION,
        .type_mask = NUS_PROTO_TYPE_MASK(NUS_PROTO_TYPE_EVENT),
        .dispatch = nus_proto_dispatch_nav_instruction,
    },
    {
        .cmd = NUS_PROTO_CMD_NAV_IMAGE,
        .type_mask = NUS_PROTO_TYPE_MASK(NUS_PROTO_TYPE_EVENT),
        .dispatch = nus_proto_dispatch_nav_image,
    },
    {
        .cmd = NUS_PROTO_CMD_TRAFFIC_SIGN,
        .type_mask = NUS_PROTO_TYPE_MASK(NUS_PROTO_TYPE_EVENT),
        .dispatch = nus_proto_dispatch_traffic_sign,
    },
    {
        .cmd = NUS_PROTO_CMD_DEVICE_INFO,
        .type_mask = NUS_PROTO_TYPE_MASK(NUS_PROTO_TYPE_REQUEST),
        .dispatch = nus_proto_dispatch_device_info,
    },
    {
        .cmd = NUS_PROTO_CMD_CURRENT_TIME,
        .type_mask = NUS_PROTO_TYPE_MASK(NUS_PROTO_TYPE_EVENT),
        .dispatch = nus_proto_dispatch_current_time,
    },
    {
        .cmd = NUS_PROTO_CMD_FILE_TRANSFER,
        .type_mask = NUS_PROTO_TYPE_MASK(NUS_PROTO_TYPE_COMMAND),
        .dispatch = nus_proto_dispatch_file_transfer,
    },
    {
        .cmd = NUS_PROTO_CMD_OTA,
        .type_mask = NUS_PROTO_TYPE_MASK(NUS_PROTO_TYPE_REQUEST) |
                     NUS_PROTO_TYPE_MASK(NUS_PROTO_TYPE_EVENT) |
                     NUS_PROTO_TYPE_MASK(NUS_PROTO_TYPE_COMMAND),
        .dispatch = nus_proto_dispatch_ota,
    },
};

static bool nus_proto_is_valid_type(nus_proto_msg_type_t type)
{
    return type == NUS_PROTO_TYPE_REQUEST ||
           type == NUS_PROTO_TYPE_RESPONSE ||
           type == NUS_PROTO_TYPE_EVENT ||
           type == NUS_PROTO_TYPE_COMMAND ||
           type == NUS_PROTO_TYPE_ACK;
}

static uint16_t nus_proto_read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t nus_proto_read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void nus_proto_write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)(value >> 8);
}

static void nus_proto_write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
    data[2] = (uint8_t)((value >> 16) & 0xFF);
    data[3] = (uint8_t)((value >> 24) & 0xFF);
}

static esp_err_t nus_proto_read_text(const uint8_t *payload,
                                     uint16_t payload_len,
                                     uint16_t *offset,
                                     nus_proto_text_t *out)
{
    if (payload == NULL || offset == NULL || out == NULL || *offset >= payload_len) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t text_len = payload[*offset];
    (*offset)++;

    if ((uint16_t)(payload_len - *offset) < text_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    out->data = (const char *)&payload[*offset];
    out->len = text_len;
    *offset += text_len;
    return ESP_OK;
}

static esp_err_t nus_proto_read_len_prefixed_u32(const uint8_t *payload,
                                                 uint16_t payload_len,
                                                 uint16_t *offset,
                                                 uint32_t *out)
{
    if (payload == NULL || offset == NULL || out == NULL || *offset >= payload_len) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t value_len = payload[*offset];
    (*offset)++;

    if (value_len != (uint8_t)sizeof(uint32_t) ||
        (uint16_t)(payload_len - *offset) < value_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    *out = nus_proto_read_u32_le(&payload[*offset]);
    *offset += value_len;
    return ESP_OK;
}

static esp_err_t nus_proto_write_text(uint8_t *out,
                                      uint16_t out_size,
                                      uint16_t *offset,
                                      const nus_proto_text_t *text)
{
    if (out == NULL || offset == NULL || text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (text->len > 0 && text->data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (*offset > out_size) {
        return ESP_ERR_INVALID_SIZE;
    }
    if ((uint16_t)(out_size - *offset) < (uint16_t)(1 + text->len)) {
        return ESP_ERR_INVALID_SIZE;
    }

    out[*offset] = text->len;
    (*offset)++;
    if (text->len > 0) {
        memcpy(&out[*offset], text->data, text->len);
        *offset += text->len;
    }
    return ESP_OK;
}

static esp_err_t nus_proto_write_len_prefixed_u32(uint8_t *out,
                                                  uint16_t out_size,
                                                  uint16_t *offset,
                                                  uint32_t value)
{
    if (out == NULL || offset == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (*offset > out_size ||
        (uint16_t)(out_size - *offset) < (uint16_t)(1 + sizeof(uint32_t))) {
        return ESP_ERR_INVALID_SIZE;
    }

    out[*offset] = (uint8_t)sizeof(uint32_t);
    (*offset)++;
    nus_proto_write_u32_le(&out[*offset], value);
    *offset += sizeof(uint32_t);
    return ESP_OK;
}

static esp_err_t nus_proto_emit_tx_error(nus_protocol_t *protocol,
                                         esp_err_t error,
                                         uint8_t cmd,
                                         nus_proto_msg_type_t type)
{
    if (protocol && protocol->callbacks.on_tx_error && error != ESP_OK) {
        protocol->callbacks.on_tx_error(error, cmd, type, protocol->user_ctx);
    }
    return error;
}

static esp_err_t nus_proto_auto_ack(nus_protocol_t *protocol,
                                    const nus_proto_frame_t *frame,
                                    nus_proto_status_t status)
{
    if (protocol == NULL || frame == NULL || !protocol->auto_ack) {
        return ESP_OK;
    }

    if (frame->type != NUS_PROTO_TYPE_REQUEST &&
        frame->type != NUS_PROTO_TYPE_EVENT &&
        frame->type != NUS_PROTO_TYPE_COMMAND) {
        return ESP_OK;
    }

    return nus_protocol_send_ack(protocol, frame->cmd, status);
}

static void nus_proto_emit_parse_error(nus_protocol_t *protocol,
                                       nus_proto_parse_error_t error,
                                       const uint8_t *data,
                                       uint16_t len)
{
    if (protocol && protocol->callbacks.on_parse_error) {
        protocol->callbacks.on_parse_error(error, data, len, protocol->user_ctx);
    }
}

static const nus_proto_dispatch_entry_t *nus_proto_find_dispatch_entry(uint8_t cmd)
{
    for (size_t i = 0; i < (sizeof(s_dispatch_table) / sizeof(s_dispatch_table[0])); i++) {
        if (s_dispatch_table[i].cmd == cmd) {
            return &s_dispatch_table[i];
        }
    }
    return NULL;
}

static esp_err_t nus_proto_dispatch_unknown(nus_protocol_t *protocol,
                                            const nus_proto_frame_t *frame,
                                            nus_proto_status_t default_status)
{
    nus_proto_status_t status = default_status;
    if (protocol->callbacks.on_unknown) {
        status = protocol->callbacks.on_unknown(frame, protocol->user_ctx);
    }
    return nus_proto_auto_ack(protocol, frame, status);
}

static esp_err_t nus_proto_dispatch_frame(nus_protocol_t *protocol,
                                          const nus_proto_frame_t *frame)
{
    if (protocol->callbacks.on_frame) {
        protocol->callbacks.on_frame(frame, protocol->user_ctx);
    }

    if (!nus_proto_is_valid_type(frame->type)) {
        return nus_proto_dispatch_unknown(protocol, frame, NUS_PROTO_STATUS_INVALID_TYPE);
    }

    if (frame->type == NUS_PROTO_TYPE_ACK) {
        nus_proto_status_t status = NUS_PROTO_STATUS_INVALID_PAYLOAD;
        const uint8_t *extra = NULL;
        uint16_t extra_len = 0;

        if (frame->payload_len > 0) {
            status = (nus_proto_status_t)frame->payload[0];
            extra = frame->payload + 1;
            extra_len = frame->payload_len - 1;
        }

        if (protocol->callbacks.on_ack) {
            protocol->callbacks.on_ack(frame->cmd, status, extra, extra_len, protocol->user_ctx);
        }
        return ESP_OK;
    }

    if (frame->type == NUS_PROTO_TYPE_RESPONSE) {
        if (protocol->callbacks.on_response) {
            protocol->callbacks.on_response(frame, protocol->user_ctx);
        }
        return ESP_OK;
    }

    const nus_proto_dispatch_entry_t *entry = nus_proto_find_dispatch_entry(frame->cmd);
    if (entry == NULL) {
        return nus_proto_dispatch_unknown(protocol, frame, NUS_PROTO_STATUS_UNSUPPORTED_CMD);
    }

    if ((entry->type_mask & NUS_PROTO_TYPE_MASK(frame->type)) == 0) {
        return nus_proto_dispatch_unknown(protocol, frame, NUS_PROTO_STATUS_INVALID_TYPE);
    }

    return entry->dispatch(protocol, frame);
}

static esp_err_t nus_proto_dispatch_nav_instruction(nus_protocol_t *protocol,
                                                    const nus_proto_frame_t *frame)
{
    nus_proto_nav_instruction_t message = {0};
    esp_err_t err = nus_protocol_parse_nav_instruction_payload(frame->payload,
                                                               frame->payload_len,
                                                               &message);
    nus_proto_status_t status = NUS_PROTO_STATUS_OK;

    if (err != ESP_OK) {
        status = NUS_PROTO_STATUS_INVALID_PAYLOAD;
    } else if (protocol->callbacks.on_nav_instruction) {
        status = protocol->callbacks.on_nav_instruction(&message, protocol->user_ctx);
    } else {
        status = NUS_PROTO_STATUS_UNSUPPORTED_CMD;
    }

    return nus_proto_auto_ack(protocol, frame, status);
}

static esp_err_t nus_proto_dispatch_nav_image(nus_protocol_t *protocol,
                                              const nus_proto_frame_t *frame)
{
    nus_proto_nav_image_t message = {0};
    esp_err_t err = nus_protocol_parse_nav_image_payload(frame->payload,
                                                         frame->payload_len,
                                                         &message);
    nus_proto_status_t status = NUS_PROTO_STATUS_OK;

    if (err != ESP_OK) {
        status = NUS_PROTO_STATUS_INVALID_PAYLOAD;
    } else if (protocol->callbacks.on_nav_image) {
        status = protocol->callbacks.on_nav_image(&message, protocol->user_ctx);
    } else {
        status = NUS_PROTO_STATUS_UNSUPPORTED_CMD;
    }

    return nus_proto_auto_ack(protocol, frame, status);
}

static esp_err_t nus_proto_dispatch_traffic_sign(nus_protocol_t *protocol,
                                                 const nus_proto_frame_t *frame)
{
    nus_proto_traffic_sign_t message = {0};
    esp_err_t err = nus_protocol_parse_traffic_sign_payload(frame->payload,
                                                            frame->payload_len,
                                                            &message);
    nus_proto_status_t status = NUS_PROTO_STATUS_OK;

    if (err != ESP_OK) {
        status = NUS_PROTO_STATUS_INVALID_PAYLOAD;
    } else if (protocol->callbacks.on_traffic_sign) {
        status = protocol->callbacks.on_traffic_sign(&message, protocol->user_ctx);
    } else {
        status = NUS_PROTO_STATUS_UNSUPPORTED_CMD;
    }

    return nus_proto_auto_ack(protocol, frame, status);
}

static esp_err_t nus_proto_dispatch_device_info(nus_protocol_t *protocol,
                                                const nus_proto_frame_t *frame)
{
    if (frame->payload_len != 0) {
        return nus_proto_auto_ack(protocol, frame, NUS_PROTO_STATUS_INVALID_PAYLOAD);
    }

    if (protocol->callbacks.on_device_info_request == NULL) {
        return nus_proto_auto_ack(protocol, frame, NUS_PROTO_STATUS_UNSUPPORTED_CMD);
    }

    nus_proto_device_info_t info = {0};
    nus_proto_status_t status = protocol->callbacks.on_device_info_request(&info,
                                                                           protocol->user_ctx);
    if (status != NUS_PROTO_STATUS_OK) {
        return nus_proto_auto_ack(protocol, frame, status);
    }

    esp_err_t err = nus_protocol_send_device_info_response(protocol, &info);
    if (err != ESP_OK) {
        return nus_proto_auto_ack(protocol, frame, NUS_PROTO_STATUS_TX_FAILED);
    }

    return ESP_OK;
}

static esp_err_t nus_proto_dispatch_current_time(nus_protocol_t *protocol,
                                                 const nus_proto_frame_t *frame)
{
    nus_proto_current_time_t message = {0};
    esp_err_t err = nus_protocol_parse_current_time_payload(frame->payload,
                                                            frame->payload_len,
                                                            &message);
    nus_proto_status_t status = NUS_PROTO_STATUS_OK;

    if (err != ESP_OK) {
        status = NUS_PROTO_STATUS_INVALID_PAYLOAD;
    } else if (protocol->callbacks.on_current_time) {
        status = protocol->callbacks.on_current_time(&message, protocol->user_ctx);
    } else {
        status = NUS_PROTO_STATUS_UNSUPPORTED_CMD;
    }

    return nus_proto_auto_ack(protocol, frame, status);
}

static esp_err_t nus_proto_dispatch_file_transfer(nus_protocol_t *protocol,
                                                  const nus_proto_frame_t *frame)
{
    nus_proto_file_transfer_t message = {0};
    esp_err_t err = nus_protocol_parse_file_transfer_payload(frame->payload,
                                                             frame->payload_len,
                                                             &message);
    nus_proto_status_t status = NUS_PROTO_STATUS_OK;

    if (err != ESP_OK) {
        status = NUS_PROTO_STATUS_INVALID_PAYLOAD;
    } else if (protocol->callbacks.on_file_transfer) {
        status = protocol->callbacks.on_file_transfer(&message, protocol->user_ctx);
    } else {
        status = NUS_PROTO_STATUS_UNSUPPORTED_CMD;
    }

    return nus_proto_auto_ack(protocol, frame, status);
}

static esp_err_t nus_proto_dispatch_ota(nus_protocol_t *protocol,
                                        const nus_proto_frame_t *frame)
{
    nus_proto_status_t status = NUS_PROTO_STATUS_UNSUPPORTED_CMD;
    if (protocol->callbacks.on_ota) {
        status = protocol->callbacks.on_ota(frame, protocol->user_ctx);
    }

    return nus_proto_auto_ack(protocol, frame, status);
}

esp_err_t nus_protocol_init(nus_protocol_t *protocol,
                            const nus_protocol_config_t *config,
                            const nus_protocol_callbacks_t *callbacks)
{
    if (protocol == NULL || config == NULL || config->send_cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(protocol, 0, sizeof(*protocol));
    protocol->send_cb = config->send_cb;
    protocol->user_ctx = config->user_ctx;
    protocol->tx_wait_ticks = config->tx_wait_ticks;
    protocol->auto_ack = config->auto_ack;
    if (callbacks) {
        protocol->callbacks = *callbacks;
    }

    return ESP_OK;
}

void nus_protocol_reset(nus_protocol_t *protocol)
{
    if (protocol == NULL) {
        return;
    }

    protocol->rx_len = 0;
    protocol->expected_len = 0;
}

esp_err_t nus_protocol_input(nus_protocol_t *protocol, const uint8_t *data, uint16_t len)
{
    if (protocol == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        if (protocol->rx_len == 0 && byte != NUS_PROTOCOL_SOF) {
            continue;
        }

        if (protocol->rx_len >= sizeof(protocol->rx_buf)) {
            nus_proto_emit_parse_error(protocol,
                                       NUS_PROTO_PARSE_ERROR_FRAME_TOO_LONG,
                                       protocol->rx_buf,
                                       protocol->rx_len);
            nus_protocol_reset(protocol);
            continue;
        }

        protocol->rx_buf[protocol->rx_len] = byte;
        protocol->rx_len++;

        if (protocol->rx_len == NUS_PROTOCOL_HEADER_LEN) {
            uint16_t payload_len = nus_proto_read_u16_le(&protocol->rx_buf[3]);
            if (payload_len > NUS_PROTOCOL_MAX_PAYLOAD_LEN) {
                nus_proto_emit_parse_error(protocol,
                                           NUS_PROTO_PARSE_ERROR_FRAME_TOO_LONG,
                                           protocol->rx_buf,
                                           protocol->rx_len);
                nus_protocol_reset(protocol);
                continue;
            }
            protocol->expected_len = NUS_PROTOCOL_FRAME_OVERHEAD + payload_len;
        }

        if (protocol->expected_len > 0 && protocol->rx_len == protocol->expected_len) {
            uint16_t received_crc = nus_proto_read_u16_le(&protocol->rx_buf[protocol->rx_len - 2]);
            uint16_t calculated_crc = nus_protocol_crc16_mcrf4xx(protocol->rx_buf,
                                                                 protocol->rx_len - 2);
            if (received_crc != calculated_crc) {
                ESP_LOGW(NUS_PROTO_TAG,
                         "Bad CRC cmd=0x%02x got=0x%04x expected=0x%04x",
                         protocol->rx_buf[1],
                         received_crc,
                         calculated_crc);
                nus_proto_emit_parse_error(protocol,
                                           NUS_PROTO_PARSE_ERROR_BAD_CRC,
                                           protocol->rx_buf,
                                           protocol->rx_len);
                nus_protocol_reset(protocol);
                continue;
            }

            nus_proto_msg_type_t type = (nus_proto_msg_type_t)protocol->rx_buf[2];
            if (!nus_proto_is_valid_type(type)) {
                nus_proto_emit_parse_error(protocol,
                                           NUS_PROTO_PARSE_ERROR_INVALID_TYPE,
                                           protocol->rx_buf,
                                           protocol->rx_len);
            }

            nus_proto_frame_t frame = {
                .cmd = protocol->rx_buf[1],
                .type = type,
                .payload = &protocol->rx_buf[NUS_PROTOCOL_HEADER_LEN],
                .payload_len = nus_proto_read_u16_le(&protocol->rx_buf[3]),
                .crc = received_crc,
            };

            esp_err_t err = nus_proto_dispatch_frame(protocol, &frame);
            nus_protocol_reset(protocol);
            if (err != ESP_OK) {
                return err;
            }
        }
    }

    return ESP_OK;
}

uint16_t nus_protocol_crc16_mcrf4xx(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    if (data == NULL && len > 0) {
        return 0;
    }

    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc & 0x0001) != 0) {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

esp_err_t nus_protocol_pack_frame(uint8_t cmd,
                                  nus_proto_msg_type_t type,
                                  const uint8_t *payload,
                                  uint16_t payload_len,
                                  uint8_t *out,
                                  uint16_t out_size,
                                  uint16_t *written)
{
    if (out == NULL || written == NULL || !nus_proto_is_valid_type(type)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_len > 0 && payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_len > NUS_PROTOCOL_MAX_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint16_t frame_len = NUS_PROTOCOL_FRAME_OVERHEAD + payload_len;
    if (out_size < frame_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    out[0] = NUS_PROTOCOL_SOF;
    out[1] = cmd;
    out[2] = (uint8_t)type;
    nus_proto_write_u16_le(&out[3], payload_len);
    if (payload_len > 0) {
        memcpy(&out[NUS_PROTOCOL_HEADER_LEN], payload, payload_len);
    }

    uint16_t crc = nus_protocol_crc16_mcrf4xx(out, frame_len - NUS_PROTOCOL_CRC_LEN);
    nus_proto_write_u16_le(&out[frame_len - NUS_PROTOCOL_CRC_LEN], crc);
    *written = frame_len;
    return ESP_OK;
}

esp_err_t nus_protocol_send_frame(nus_protocol_t *protocol,
                                  uint8_t cmd,
                                  nus_proto_msg_type_t type,
                                  const uint8_t *payload,
                                  uint16_t payload_len)
{
    if (protocol == NULL || protocol->send_cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t frame[NUS_PROTOCOL_MAX_FRAME_LEN];
    uint16_t frame_len = 0;
    esp_err_t err = nus_protocol_pack_frame(cmd,
                                            type,
                                            payload,
                                            payload_len,
                                            frame,
                                            sizeof(frame),
                                            &frame_len);
    if (err != ESP_OK) {
        return nus_proto_emit_tx_error(protocol, err, cmd, type);
    }

    err = protocol->send_cb(frame, frame_len, protocol->tx_wait_ticks, protocol->user_ctx);
    return nus_proto_emit_tx_error(protocol, err, cmd, type);
}

esp_err_t nus_protocol_send_ack(nus_protocol_t *protocol,
                                uint8_t cmd,
                                nus_proto_status_t status)
{
    uint8_t payload[] = {
        (uint8_t)status,
    };
    return nus_protocol_send_frame(protocol,
                                   cmd,
                                   NUS_PROTO_TYPE_ACK,
                                   payload,
                                   sizeof(payload));
}

esp_err_t nus_protocol_send_device_info_response(nus_protocol_t *protocol,
                                                 const nus_proto_device_info_t *info)
{
    if (protocol == NULL || info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[NUS_PROTOCOL_MAX_PAYLOAD_LEN];
    uint16_t payload_len = 0;
    esp_err_t err = nus_protocol_pack_device_info_payload(info,
                                                          payload,
                                                          sizeof(payload),
                                                          &payload_len);
    if (err != ESP_OK) {
        return nus_proto_emit_tx_error(protocol,
                                       err,
                                       NUS_PROTO_CMD_DEVICE_INFO,
                                       NUS_PROTO_TYPE_RESPONSE);
    }

    return nus_protocol_send_frame(protocol,
                                   NUS_PROTO_CMD_DEVICE_INFO,
                                   NUS_PROTO_TYPE_RESPONSE,
                                   payload,
                                   payload_len);
}

esp_err_t nus_protocol_parse_nav_instruction_payload(const uint8_t *payload,
                                                     uint16_t payload_len,
                                                     nus_proto_nav_instruction_t *out)
{
    if (payload == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t offset = 0;
    esp_err_t err = nus_proto_read_text(payload, payload_len, &offset, &out->direction);
    if (err != ESP_OK) {
        return err;
    }
    err = nus_proto_read_len_prefixed_u32(payload, payload_len, &offset, &out->distance_m);
    if (err != ESP_OK) {
        return err;
    }
    err = nus_proto_read_text(payload, payload_len, &offset, &out->next_direction);
    if (err != ESP_OK) {
        return err;
    }
    err = nus_proto_read_len_prefixed_u32(payload,
                                          payload_len,
                                          &offset,
                                          &out->destination_distance_m);
    if (err != ESP_OK) {
        return err;
    }
    err = nus_proto_read_len_prefixed_u32(payload,
                                          payload_len,
                                          &offset,
                                          &out->remaining_time_minutes);
    if (err != ESP_OK) {
        return err;
    }

    if ((uint16_t)(payload_len - offset) < sizeof(uint16_t)) {
        return ESP_ERR_INVALID_SIZE;
    }

    out->current_speed_mps = nus_proto_read_u16_le(&payload[offset]);
    offset += sizeof(uint16_t);

    err = nus_proto_read_len_prefixed_u32(payload,
                                          payload_len,
                                          &offset,
                                          &out->current_time_epoch_seconds);
    if (err != ESP_OK) {
        return err;
    }
    if (offset != payload_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

esp_err_t nus_protocol_parse_nav_image_payload(const uint8_t *payload,
                                               uint16_t payload_len,
                                               nus_proto_nav_image_t *out)
{
    if (payload == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_len < 7) {
        return ESP_ERR_INVALID_SIZE;
    }

    out->image_type = payload[0];
    out->width = nus_proto_read_u16_le(&payload[1]);
    out->height = nus_proto_read_u16_le(&payload[3]);
    out->data_len = nus_proto_read_u16_le(&payload[5]);
    if ((uint16_t)(payload_len - 7) != out->data_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    out->data = &payload[7];
    return ESP_OK;
}

esp_err_t nus_protocol_parse_traffic_sign_payload(const uint8_t *payload,
                                                  uint16_t payload_len,
                                                  nus_proto_traffic_sign_t *out)
{
    if (payload == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_len < 3) {
        return ESP_ERR_INVALID_SIZE;
    }

    out->sign_type = payload[0];
    out->data_len = nus_proto_read_u16_le(&payload[1]);
    if ((uint16_t)(payload_len - 3) != out->data_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    out->data = &payload[3];
    return ESP_OK;
}

esp_err_t nus_protocol_parse_device_info_payload(const uint8_t *payload,
                                                 uint16_t payload_len,
                                                 nus_proto_device_info_t *out)
{
    if (payload == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_len != NUS_PROTO_DEVICE_INFO_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    out->hardware_version = nus_proto_read_u32_le(&payload[0]);
    out->firmware_version = nus_proto_read_u32_le(&payload[4]);
    memcpy(out->manufacturer_id, &payload[8], NUS_PROTO_MANUFACTURER_ID_LEN);
    memcpy(out->serial_number, &payload[24], NUS_PROTO_SERIAL_NUMBER_LEN);
    out->product_id = nus_proto_read_u32_le(&payload[56]);
    out->model_id = nus_proto_read_u32_le(&payload[60]);
    return ESP_OK;
}

esp_err_t nus_protocol_parse_current_time_payload(const uint8_t *payload,
                                                  uint16_t payload_len,
                                                  nus_proto_current_time_t *out)
{
    if (payload == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_len == sizeof(uint32_t)) {
        out->epoch_seconds = nus_proto_read_u32_le(payload);
        return ESP_OK;
    }
    return ESP_ERR_INVALID_SIZE;
}

esp_err_t nus_protocol_parse_file_transfer_payload(const uint8_t *payload,
                                                   uint16_t payload_len,
                                                   nus_proto_file_transfer_t *out)
{
    if (payload == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_len < 10) {
        return ESP_ERR_INVALID_SIZE;
    }

    out->file_size = nus_proto_read_u32_le(&payload[0]);
    out->offset = nus_proto_read_u32_le(&payload[4]);
    out->data_len = nus_proto_read_u16_le(&payload[8]);
    if ((uint16_t)(payload_len - 10) != out->data_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    out->data = &payload[10];
    return ESP_OK;
}

esp_err_t nus_protocol_pack_nav_instruction_payload(const nus_proto_nav_instruction_t *message,
                                                    uint8_t *out,
                                                    uint16_t out_size,
                                                    uint16_t *written)
{
    if (message == NULL || out == NULL || written == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t offset = 0;
    esp_err_t err = nus_proto_write_text(out, out_size, &offset, &message->direction);
    if (err != ESP_OK) {
        return err;
    }
    err = nus_proto_write_len_prefixed_u32(out, out_size, &offset, message->distance_m);
    if (err != ESP_OK) {
        return err;
    }
    err = nus_proto_write_text(out, out_size, &offset, &message->next_direction);
    if (err != ESP_OK) {
        return err;
    }
    err = nus_proto_write_len_prefixed_u32(out,
                                           out_size,
                                           &offset,
                                           message->destination_distance_m);
    if (err != ESP_OK) {
        return err;
    }
    err = nus_proto_write_len_prefixed_u32(out,
                                           out_size,
                                           &offset,
                                           message->remaining_time_minutes);
    if (err != ESP_OK) {
        return err;
    }

    if ((uint16_t)(out_size - offset) < sizeof(uint16_t)) {
        return ESP_ERR_INVALID_SIZE;
    }

    nus_proto_write_u16_le(&out[offset], message->current_speed_mps);
    offset += sizeof(uint16_t);

    err = nus_proto_write_len_prefixed_u32(out,
                                           out_size,
                                           &offset,
                                           message->current_time_epoch_seconds);
    if (err != ESP_OK) {
        return err;
    }

    *written = offset;
    return ESP_OK;
}

esp_err_t nus_protocol_pack_nav_image_payload(const nus_proto_nav_image_t *message,
                                              uint8_t *out,
                                              uint16_t out_size,
                                              uint16_t *written)
{
    if (message == NULL || out == NULL || written == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (message->data_len > 0 && message->data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_size < 7 || message->data_len > (uint16_t)(out_size - 7)) {
        return ESP_ERR_INVALID_SIZE;
    }

    out[0] = message->image_type;
    nus_proto_write_u16_le(&out[1], message->width);
    nus_proto_write_u16_le(&out[3], message->height);
    nus_proto_write_u16_le(&out[5], message->data_len);
    if (message->data_len > 0) {
        memcpy(&out[7], message->data, message->data_len);
    }
    *written = 7 + message->data_len;
    return ESP_OK;
}

esp_err_t nus_protocol_pack_traffic_sign_payload(const nus_proto_traffic_sign_t *message,
                                                 uint8_t *out,
                                                 uint16_t out_size,
                                                 uint16_t *written)
{
    if (message == NULL || out == NULL || written == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (message->data_len > 0 && message->data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_size < 3 || message->data_len > (uint16_t)(out_size - 3)) {
        return ESP_ERR_INVALID_SIZE;
    }

    out[0] = message->sign_type;
    nus_proto_write_u16_le(&out[1], message->data_len);
    if (message->data_len > 0) {
        memcpy(&out[3], message->data, message->data_len);
    }
    *written = 3 + message->data_len;
    return ESP_OK;
}

esp_err_t nus_protocol_pack_device_info_payload(const nus_proto_device_info_t *info,
                                                uint8_t *out,
                                                uint16_t out_size,
                                                uint16_t *written)
{
    if (info == NULL || out == NULL || written == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_size < NUS_PROTO_DEVICE_INFO_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    nus_proto_write_u32_le(&out[0], info->hardware_version);
    nus_proto_write_u32_le(&out[4], info->firmware_version);
    memcpy(&out[8], info->manufacturer_id, NUS_PROTO_MANUFACTURER_ID_LEN);
    memcpy(&out[24], info->serial_number, NUS_PROTO_SERIAL_NUMBER_LEN);
    nus_proto_write_u32_le(&out[56], info->product_id);
    nus_proto_write_u32_le(&out[60], info->model_id);

    *written = NUS_PROTO_DEVICE_INFO_PAYLOAD_LEN;
    return ESP_OK;
}

esp_err_t nus_protocol_pack_current_time_payload(const nus_proto_current_time_t *message,
                                                 uint8_t *out,
                                                 uint16_t out_size,
                                                 uint16_t *written)
{
    if (message == NULL || out == NULL || written == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_size < sizeof(uint32_t)) {
        return ESP_ERR_INVALID_SIZE;
    }

    nus_proto_write_u32_le(out, message->epoch_seconds);
    *written = sizeof(uint32_t);
    return ESP_OK;
}

esp_err_t nus_protocol_pack_file_transfer_payload(const nus_proto_file_transfer_t *message,
                                                  uint8_t *out,
                                                  uint16_t out_size,
                                                  uint16_t *written)
{
    if (message == NULL || out == NULL || written == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (message->data_len > 0 && message->data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_size < 10 || message->data_len > (uint16_t)(out_size - 10)) {
        return ESP_ERR_INVALID_SIZE;
    }

    nus_proto_write_u32_le(&out[0], message->file_size);
    nus_proto_write_u32_le(&out[4], message->offset);
    nus_proto_write_u16_le(&out[8], message->data_len);
    if (message->data_len > 0) {
        memcpy(&out[10], message->data, message->data_len);
    }
    *written = 10 + message->data_len;
    return ESP_OK;
}
