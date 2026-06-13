# ESP-IDF Nordic UART Service (NUS)

Project nay bien ESP32 thanh BLE GATT server tuong thich Nordic UART Service
(NUS). Lop `nus.c/nus.h` quan ly BLE stack, advertising, trang thai ket noi,
notify, timeout, RX callback va TX queue. File `gatts_demo.c` chi con la vi du
ung dung su dung wrapper NUS.

| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

## NUS UUID

Service va characteristic dung UUID NUS chuan:

| Name | UUID | Direction |
| ---- | ---- | --------- |
| NUS Service | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | BLE service |
| NUS RX | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` | Phone/app write vao ESP |
| NUS TX | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` | ESP notify len phone/app |

## File Chinh

- `main/nus.h`: public API cho app.
- `main/nus.c`: BLE GATT server, advertising, connection state, TX queue,
  timeout va callback noi bo.
- `main/nus_protocol.h`: public API cho frame protocol, parser, packer va
  callbacks.
- `main/nus_protocol.c`: CRC-16/MCRF4XX, streaming parser, command dispatcher,
  ACK/response helper va payload codec.
- `main/gatts_demo.c`: vi du khoi tao NUS, noi RX vao protocol dispatcher va
  dang ky callbacks.
- `main/CMakeLists.txt`: build `gatts_demo.c`, `nus.c` va `nus_protocol.c`.

## Build

Trong ESP-IDF terminal:

```bash
idf.py set-target esp32h2
idf.py build
idf.py -p COM8 flash monitor
```

Neu PowerShell tren Windows bi chon sai Python ESP-IDF, co the dung:

```powershell
$env:PATH = 'C:\Espressif\tools\idf-python\3.11.2;' + $env:PATH
. C:\Espressif\frameworks\esp-idf-v5.5.3\export.ps1
$env:IDF_TARGET='esp32h2'
idf.py build
```

## Su Dung Nhan Va Gui

Du lieu phone/app gui xuong ESP se vao RX callback:

```c
static void app_nus_rx_cb(const uint8_t *data, uint16_t len)
{
    ESP_LOG_BUFFER_HEX("APP", data, len);

    // Gui lai du lieu vua nhan qua NUS TX notify.
    esp_err_t err = nus_send(data, len, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGW("APP", "nus_send failed: %s", esp_err_to_name(err));
    }
}
```

De gui du lieu tu ESP len phone/app:

```c
const uint8_t msg[] = "hello";
esp_err_t err = nus_send(msg, sizeof(msg) - 1, pdMS_TO_TICKS(100));
```

`nus_send()` khong gui truc tiep trong BLE callback. Ham nay dua data vao
FreeRTOS queue, task `nus_tx` se tu cat goi theo MTU va gui notification. Client
phai connect va enable notify tren NUS TX thi ham moi tra `ESP_OK`.

## Cau Hinh NUS

Cau hinh nam trong `nus_config_t`:

```c
void app_main(void)
{
    nus_config_t config = NUS_CONFIG_DEFAULT();

    config.device_name = "Tiem NUS";
    config.adv_interval_min_ms = 80;
    config.adv_interval_max_ms = 120;
    config.adv_timeout_ms = 0;       // 0 = advertising khong tu stop

    config.conn_interval_min_ms = 20;
    config.conn_interval_max_ms = 40;
    config.conn_latency = 0;
    config.conn_timeout_ms = 4000;

    config.idle_timeout_ms = 0;      // 0 = khong tu disconnect khi idle
    config.auto_start_adv = true;
    config.restart_adv_after_disconnect = true;

    nus_set_state_callback(app_nus_state_cb);
    ESP_ERROR_CHECK(nus_init_with_config(&config, app_nus_rx_cb));
}
```

Neu khong can tuy bien:

```c
ESP_ERROR_CHECK(nus_init(app_nus_rx_cb));
```

## Advertising

Advertising co the tu start sau init bang `auto_start_adv = true`, hoac dieu
khien thu cong:

```c
nus_start_adv(30000); // start advertising va tu stop sau 30 giay
nus_stop_adv();       // stop advertising thu cong
```

API lien quan:

```c
bool nus_is_advertising(void);
esp_err_t nus_start_adv(uint32_t timeout_ms);
esp_err_t nus_stop_adv(void);
```

`timeout_ms = 0` nghia la khong dat timeout advertising.

## Timeout

Co hai timeout rieng:

- `adv_timeout_ms`: neu dang advertising ma chua co client connect, NUS tu stop
  advertising sau thoi gian nay.
- `idle_timeout_ms`: neu da connect ma khong co RX/TX trong thoi gian nay, NUS
  phat event `NUS_STATE_IDLE_TIMEOUT` va disconnect client.

Khi app co activity rieng ma muon reset idle timer:

```c
nus_reset_idle_timeout();
```

## State Callback

Dang ky state callback de theo doi trang thai:

```c
static void app_nus_state_cb(nus_state_event_t event)
{
    switch (event) {
    case NUS_STATE_ADV_STARTED:
        break;
    case NUS_STATE_ADV_STOPPED:
        break;
    case NUS_STATE_ADV_TIMEOUT:
        break;
    case NUS_STATE_CONNECTED:
        break;
    case NUS_STATE_DISCONNECTED:
        break;
    case NUS_STATE_NOTIFY_ENABLED:
        break;
    case NUS_STATE_NOTIFY_DISABLED:
        break;
    case NUS_STATE_IDLE_TIMEOUT:
        break;
    default:
        break;
    }
}
```

## API Tom Tat

```c
esp_err_t nus_init(nus_rx_cb_t rx_cb);
esp_err_t nus_init_with_config(const nus_config_t *config, nus_rx_cb_t rx_cb);

void nus_set_rx_callback(nus_rx_cb_t rx_cb);
void nus_set_state_callback(nus_state_cb_t state_cb);

bool nus_is_connected(void);
bool nus_is_advertising(void);
bool nus_is_notify_enabled(void);
uint16_t nus_get_mtu(void);
uint16_t nus_get_max_payload_len(void);

esp_err_t nus_start_adv(uint32_t timeout_ms);
esp_err_t nus_stop_adv(void);
esp_err_t nus_send(const uint8_t *data, uint16_t len, TickType_t ticks_to_wait);
void nus_flush_tx_queue(void);
void nus_reset_idle_timeout(void);
```

## Gioi Han Mac Dinh

- `NUS_MAX_DATA_LEN = 500`
- `NUS_TX_QUEUE_LEN = 10`
- MTU local duoc set la 500.
- Payload notify thuc te moi goi la `MTU - 3`.
- Neu data lon hon payload notify, `nus_tx` tu cat thanh nhieu notification.

## Test Bang Dien Thoai

Co the dung app BLE bat ky co ho tro Nordic UART Service, vi du Nordic nRF
Connect:

1. Scan device name `Tiem NUS`.
2. Connect vao device.
3. Enable notification cho characteristic NUS TX.
4. Write data vao characteristic NUS RX.
5. Demo hien tai parse frame protocol, goi callback tuong ung va tra ACK hoac
   response qua NUS TX.


## NUS Protocol

Protocol nam tren NUS RX/TX va duoc xu ly trong `nus_protocol.c`. Parser la
streaming parser, nen co the nhan frame bi chia nho theo BLE write hoac nhieu
frame trong cung mot lan RX.

Frame:

```text
SOF(0xAC) | CMD(1) | TYPE(1) | LEN(2 LE) | PAYLOAD | CRC16/MCRF4XX(2 LE)
```

CRC tinh tren tat ca byte tu `SOF` den het `PAYLOAD`, khong tinh 2 byte CRC.
Gioi han frame hien tai la `NUS_MAX_DATA_LEN` byte, payload toi da la
`NUS_PROTOCOL_MAX_PAYLOAD_LEN` byte.

### Type

| Type | Value |
| ---- | ----- |
| `REQUEST` | `0x00` |
| `RESPONSE` | `0x01` |
| `EVENT` | `0x02` |
| `COMMAND` | `0x03` |
| `ACK` | `0x04` |

ACK payload co 1 byte status:

| Status | Value |
| ------ | ----- |
| `OK` | `0x00` |
| `INVALID_FRAME` | `0x01` |
| `INVALID_TYPE` | `0x02` |
| `INVALID_PAYLOAD` | `0x03` |
| `UNSUPPORTED_CMD` | `0x04` |
| `APP_ERROR` | `0x05` |
| `TX_FAILED` | `0x06` |

### Commands

| CMD | Type tu mobile | Payload | Device reply |
| --- | -------------- | ------- | ------------ |
| `NAV_INSTRUCTION(0x01)` | `EVENT` | `u8 len + direction`, `u8 len + distance`, `u8 len + next_direction`, `u8 len + destination_distance`, `u8 len + remaining_time`, `u16 current_speed` | `ACK` |
| `NAV_IMAGE(0x02)` | `EVENT` | `u8 image_type`, `u16 width`, `u16 height`, `u16 data_len`, `data` | `ACK` |
| `TRAFFIC_SIGN(0x03)` | `EVENT` | `u8 sign_type`, `u16 data_len`, `data` | `ACK` |
| `DEVICE_INFO(0x04)` | `REQUEST` | empty | `RESPONSE` with 6 text fields: hardware, firmware, manufacturer, serial, product, model |
| `CURRENT_TIME(0x05)` | `EVENT` | `u32` or `u64 epoch_seconds` little-endian | `ACK` |
| `FILE_TRANSFER(0x06)` | `COMMAND` | `u32 file_size`, `u32 offset`, `u16 data_len`, `data` | `ACK` |
| `OTA(0x07)` | `REQUEST`/`EVENT`/`COMMAND` | raw, TBD | callback returns status, then `ACK` |

### Protocol API

Khoi tao dispatcher:

```c
static nus_protocol_t s_protocol;

static esp_err_t proto_send(const uint8_t *data,
                            uint16_t len,
                            TickType_t ticks_to_wait,
                            void *user_ctx)
{
    return nus_send(data, len, ticks_to_wait);
}

nus_protocol_config_t proto_config = NUS_PROTOCOL_CONFIG_DEFAULT(proto_send, NULL);
proto_config.tx_wait_ticks = 0; // non-blocking ACK/response inside BLE RX callback
const nus_protocol_callbacks_t callbacks = {
    .on_nav_instruction = app_proto_nav_instruction_cb,
    .on_device_info_request = app_proto_device_info_cb,
    .on_current_time = app_proto_current_time_cb,
    .on_unknown = app_proto_unknown_cb,
};

ESP_ERROR_CHECK(nus_protocol_init(&s_protocol, &proto_config, &callbacks));
```

Noi BLE RX vao parser:

```c
static void app_nus_rx_cb(const uint8_t *data, uint16_t len)
{
    ESP_ERROR_CHECK(nus_protocol_input(&s_protocol, data, len));
}
```

Gui frame thu cong khi can:

```c
nus_protocol_send_ack(&s_protocol, NUS_PROTO_CMD_CURRENT_TIME, NUS_PROTO_STATUS_OK);
nus_protocol_send_frame(&s_protocol, cmd, NUS_PROTO_TYPE_EVENT, payload, payload_len);
```

De mo rong command moi, them enum CMD, payload parser/packer neu can, them entry
vao `s_dispatch_table` trong `nus_protocol.c`, sau do them callback vao
`nus_protocol_callbacks_t`.
