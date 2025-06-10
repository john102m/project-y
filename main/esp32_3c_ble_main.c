/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#include "esp_adc/adc_oneshot.h"

static const char *TAG = "ESP32C3";
#define BLE_TAG "BLE"
#define LED_GPIO 8                // Onboard LED pin
#define STAT_PIN GPIO_NUM_1       // GPIO1
#define NOTIFY_DEBOUNCE_US 500000 // 500ms
#define INTR_DEBOUNCE_DELAY_MS 500

volatile uint32_t last_interrupt_time = 0;
QueueHandle_t delay_queue, gpio_evt_queue = NULL;

// home made struct for connection info - can be used in notifications
typedef struct
{
    esp_gatt_if_t gatts_if;
    uint16_t conn_id;
    // BLE Service & Characteristic Handles
    uint16_t service_handle;
    uint16_t char_handle;

    int64_t last_sent_time_us; // For timing-based debounce
    esp_bd_addr_t remote_bda;

    // Optional: char last_message[64]; // To compare last sent message
} ble_connection_t;

// Global variable to store the characteristic handle (and other stuff related to the connection)
static ble_connection_t ble_connection = {0}; // Zero-initialized global
static bool ble_connected = false;

// UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
static const uint8_t SERVICE_UUID[ESP_UUID_LEN_128] = {
    0x4b, 0x91, 0x31, 0xc3,            // time_low
    0xc9, 0xc5,                        // time_mid
    0xcc, 0x8f,                        // time_hi_and_version
    0x9e, 0x45,                        // clock_seq
    0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f // node
};

// UUID: beb5483e-36e1-4688-b7f5-ea07361b26a8
static const uint8_t CHARACTERISTIC_UUID[ESP_UUID_LEN_128] = {
    0xa8, 0x26, 0x1b, 0x36,
    0x07, 0xea,
    0xf5, 0xb7,
    0x88, 0x46,
    0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe};

// need this to copy the SERVICE_UUID into the service id of type:   esp_gatt_srvc_id_t service_id   (with memcopy - who knew!)
esp_bt_uuid_t service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = {0}};

// BLE Advertising Data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,                                                // Determines whether scan response data is included (disabled here).
    .include_name = true,                                                 // Advertises the device name.
    .include_txpower = true,                                              // Advertises the transmission power, helping remote devices estimate distance.
    .min_interval = 0x0006,                                               // Minimum connection interval, determining how often devices exchange data (in BLE time units).
    .max_interval = 0x0010,                                               // Maximum connection interval, balancing latency vs. power consumption.
    .appearance = 0x00,                                                   // Defines the device category (0x00 = unspecified/custom).
    .manufacturer_len = 0,                                                // No manufacturer-specific data included.
    .p_manufacturer_data = NULL,                                          // No custom manufacturer data provided.
    .service_data_len = 0,                                                // No additional service data included.
    .p_service_data = NULL,                                               // No service-specific data provided.
    .service_uuid_len = ESP_UUID_LEN_128,                                 // Specifies that the advertised service UUID is 128-bit.
    .p_service_uuid = service_uuid.uuid.uuid128,                          // Pointer to the service UUID being advertised.
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), // Flags indicate that the device is generally discoverable and does not support classic Bluetooth (BR/EDR).
};

// BLE Advertising Config
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,                                    // Minimum advertising interval (32 * 0.625ms = 20ms).
    .adv_int_max = 0x40,                                    // Maximum advertising interval (64 * 0.625ms = 40ms).
    .adv_type = ADV_TYPE_IND,                               // Uses connectable undirected advertising, allowing any device to connect.
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,                  // Advertises using the public Bluetooth address.
    .channel_map = ADV_CHNL_ALL,                            // Uses all three advertising channels for maximum visibility.
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, // Allows any device to scan or connect, without filtering.
};

// Function to send notification or indication
void send_ble_notification_or_indication(ble_connection_t *conn_info, const char *message, bool is_indication)
{
    int64_t now = esp_timer_get_time(); // Current time in microseconds

    if ((now - conn_info->last_sent_time_us) < NOTIFY_DEBOUNCE_US)
    {
        ESP_LOGW(BLE_TAG, "Debounced: Skipping repeated notification");
        return;
    }

    conn_info->last_sent_time_us = now;

    uint16_t length = strlen(message); // No +1 to avoid sending null terminator unless needed
    uint8_t *notify_value = malloc(length);
    if (!notify_value)
    {
        ESP_LOGE(BLE_TAG, "Failed to allocate memory for notification");
        return;
    }

    memcpy(notify_value, message, length);

    esp_err_t err = esp_ble_gatts_send_indicate(
        conn_info->gatts_if,
        conn_info->conn_id,
        conn_info->char_handle,
        length,
        notify_value,
        is_indication);

    free(notify_value);

    if (err != ESP_OK)
    {
        ESP_LOGE(BLE_TAG, "Notification send failed: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(BLE_TAG, "Sent BLE message (%d bytes) as %s: %s",
                 length, is_indication ? "Indication" : "Notification", message);
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    switch (event)
    {

    case ESP_GATTS_REG_EVT:
        // Define the service ID and initialize the UUID field properly
        ESP_LOGI(BLE_TAG, "GATT Server registered");
        ESP_LOGI(BLE_TAG, "Creating service with UUID: %02x%02x...", SERVICE_UUID[0], SERVICE_UUID[1]);

        // Define the service ID with proper initialization
        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0,
                .uuid = service_uuid, // Here we use the previously defined service_uuid
            },
        };
        // Create the service
        esp_ble_gatts_create_service(gatts_if, &service_id, 10);

        ESP_LOGI(BLE_TAG, "GATT Server registered");
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(BLE_TAG, "Service created, handle = %d", param->create.service_handle);

        //                  👇 Save the service handle before adding a characteristic
        ble_connection.service_handle = param->create.service_handle;

        esp_bt_uuid_t char_uuid = {
            .len = ESP_UUID_LEN_128,
            .uuid = {.uuid128 = {0}}};
        memcpy(char_uuid.uuid.uuid128, CHARACTERISTIC_UUID, ESP_UUID_LEN_128);

        esp_gatt_char_prop_t char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

        ESP_LOGI(BLE_TAG, "Adding characteristic with UUID: %02x%02x...", CHARACTERISTIC_UUID[0], CHARACTERISTIC_UUID[1]);

        esp_err_t add_char_ret = esp_ble_gatts_add_char(
            ble_connection.service_handle, &char_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            char_property,
            NULL, NULL);

        if (add_char_ret != ESP_OK)
        {
            ESP_LOGE(BLE_TAG, "Failed to add characteristic: %s", esp_err_to_name(add_char_ret));
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(BLE_TAG, "Characteristic added, handle=%d", param->add_char.attr_handle);
        ble_connection.char_handle = param->add_char.attr_handle;
        // Start service
        esp_ble_gatts_start_service(ble_connection.service_handle);
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(BLE_TAG, "Device connected! conn_id=%d", param->connect.conn_id);

        // Send notification/indication back to client
        ble_connection.gatts_if = gatts_if;
        ble_connection.conn_id = param->connect.conn_id;
        memcpy(ble_connection.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

        ble_connected = true;
        // control the blink rate of the blink task
        xQueueSend(delay_queue, &(uint32_t){1000}, portMAX_DELAY);
        // Send a custom message as a notification
        send_ble_notification_or_indication(&ble_connection, "Hello from ESP32!", false); // false = notification
        esp_ble_gap_stop_advertising();                                                   // Stop advertising after connection

        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ble_connected = false;
        // control the blink rate of the blink task
        xQueueSend(delay_queue, &(uint32_t){100}, portMAX_DELAY);
        ESP_LOGI(BLE_TAG, "Device disconnected, restarting advertising...");
        esp_ble_gap_start_advertising(&adv_params); // Restart advertising

        break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(BLE_TAG, "Read request received!");

        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.len = 5;
        memcpy(rsp.attr_value.value, "Hello", 5);

        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(BLE_TAG, "Write request received: %.*s", param->write.len, param->write.value);

        // Validate handle before sending response
        if (param->write.handle == ble_connection.char_handle)
        {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.len = param->write.len;
            memcpy(rsp.attr_value.value, param->write.value, param->write.len);

            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                        ESP_GATT_OK, &rsp);

            // Echo write message as a notification
            send_ble_notification_or_indication(&ble_connection, (const char *)param->write.value, false); // false = notification
        }
        else
        {
            ESP_LOGW(BLE_TAG, "Write request received for unknown handle: %d", param->write.handle);
        }
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(BLE_TAG, "Indication confirmed, status = %d", param->conf.status);
        break;
    default:
        ESP_LOGI(BLE_TAG, "Unhandled GATT event: %d", event);
        break;
    }
}

void IRAM_ATTR gpio_isr_handler(void *arg)
{
    int gpio_num = (int)arg;                    // Get GPIO number from arg
    uint32_t now = esp_timer_get_time() / 1000; // Convert to milliseconds
    if (now - last_interrupt_time > INTR_DEBOUNCE_DELAY_MS)
    {
        last_interrupt_time = now;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
    }
}
void gpio_event_task(void *arg)
{
    int gpio_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            int level = gpio_get_level(gpio_num);
            ESP_LOGI("GPIO", "Detected change on GPIO %d, new state: %d", gpio_num, level);
            char info_str[32];
            snprintf(info_str, sizeof(info_str), "Status: %s", level ? "Not Charging" : "Charging");
            send_ble_notification_or_indication(&ble_connection, info_str, false);
        }
    }
}

void init_pins(void)
{
    gpio_evt_queue = xQueueCreate(1, sizeof(int)); // Example: Queue for 10 integers

    // Configure STAT_PIN as input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STAT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE};
    gpio_config(&io_conf);
    // gpio_install_isr_service(0);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(STAT_PIN, gpio_isr_handler, (void *)STAT_PIN));

    // Initialize LED GPIO
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(BLE_TAG, "Adv data set, starting advertising...");
        esp_ble_gap_start_advertising(&adv_params); // CONFIG_BT_BLE_42_FEATURES_SUPPORTED=y
        break;
    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
        if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS)
        {
            char rssi_str[16];
            snprintf(rssi_str, sizeof(rssi_str), "RSSI:%d", param->read_rssi_cmpl.rssi);

            // Send to the phone app
            send_ble_notification_or_indication(&ble_connection, rssi_str, false);
        }
        break;
    default:
        break;
    }
}

void ble_init()
{

    // Copy SERVICE_UUID into service_uuid for advertising
    memcpy(service_uuid.uuid.uuid128, SERVICE_UUID, ESP_UUID_LEN_128);

    ESP_LOGI(BLE_TAG, "Initializing BLE...");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);

    esp_err_t ret = esp_ble_gatts_app_register(0);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "GATT app registration failed!");
    }
    // Make sure you configure the advertising data before calling start_advertising():
    ESP_LOGI(BLE_TAG, "Configuring advertising data...");

    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(BLE_TAG, "Failed to configure adv data: %s", esp_err_to_name(ret));
    }
}
void get_mac_address()
{
    const uint8_t *mac = esp_bt_dev_get_address();
    ESP_LOGI("BT", "Bluetooth MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void ble_rssi_notify_task(void *pvParameter)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 seconds

        if (ble_connected)
        {
            // int rssi;
            //  Get RSSI — connection handle = remote_bda
            esp_ble_gap_read_rssi(ble_connection.remote_bda); // You'll need to save this too!

            // In the GAP callback, you'll get the RSSI result in ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT
            // You can cache it and send it from there, or here if you store it safely
        }
    }
}
float calibrate_adc(int raw_adc)
{
    float Vmax = 3.3;  // FSD voltage due to ADC_ATTEN_DB_12
    float Dmax = 4095; // Max ADC raw value (12-bit resolution)

    return (raw_adc / Dmax) * Vmax; // Corrected voltage output
}

void read_battery_voltage_task()
{

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_cfg);

    for (;;)
    {
        int raw_value;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &raw_value);

        ESP_LOGI(TAG, "ADC Raw: %d", raw_value);

        float voltage = calibrate_adc(raw_value);
        ESP_LOGI(TAG, "Voltage: %.2fV", voltage);
        if (ble_connected)
        {
            char volt_str[20]; // Increased buffer size to accommodate the status text
            snprintf(volt_str, sizeof(volt_str), "Voltage: %.2fV", voltage);
            send_ble_notification_or_indication(&ble_connection, volt_str, false);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void blink_task(void *pvParameter)
{
    uint32_t count = 0, current_delay = *(uint32_t *)pvParameter;

    for (;;)
    {
        gpio_set_level(LED_GPIO, count++ % 2);
        // Check if a new delay value is available
        if (xQueueReceive(delay_queue, &current_delay, 0))
        {
            ESP_LOGI(TAG, "Updated delay: %ld ms\n", current_delay);
        }
        vTaskDelay(pdMS_TO_TICKS(current_delay));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated or has old version; erase and retry
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_pins();
    delay_queue = xQueueCreate(1, sizeof(uint32_t));

    ble_init();

    ESP_LOGI(TAG, "Limited Information:\n");
    ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    get_mac_address();
    xTaskCreate(blink_task, "Blink Task", 2048, (void *)&(uint32_t){1000}, 1, NULL);
    xTaskCreate(ble_rssi_notify_task, "RSSI Notify Task", 2048, NULL, 5, NULL);
    xTaskCreate(read_battery_voltage_task, "Battery Voltage  Task", 2048, NULL, 5, NULL);
    xTaskCreate(gpio_event_task, "GPIO Event Task", 2048, NULL, 5, NULL);

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, 20); // -12 dBm
    esp_power_level_t power = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
    ESP_LOGI(TAG, "TX Power level (ADV): %d", power);

    // only use this when you are sure there are other non terminating taske
    // vTaskDelete(NULL); // deletes the main task safely
    ESP_LOGI(TAG, "Waiting for BLE connections........\n");
    // control the blink rate of the blink task
    xQueueSend(delay_queue, &(uint32_t){100}, portMAX_DELAY);
    while (true)
    {
        vTaskDelay(portMAX_DELAY); // main task sleeps forever
    }
}
