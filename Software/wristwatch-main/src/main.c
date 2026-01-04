#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "time.h"

// ==================== CONFIG ====================
#include "secrets.h"
#define IMU_INT_PIN     3    // example GPIO
#define I2C_MASTER_SDA  18
#define I2C_MASTER_SCL  19
#define I2C_MASTER_NUM  I2C_NUM_0
#define I2C_FREQ_HZ     400000

// ====================== IMU =====================
#define BAT_MEAS_ADC 4
#define BMS_STATUS_N 5
#define VEML_SUPPLY 6
#define BAT_MEAS_EN_N 7

// ====================== IMU =====================
#define LIS2DW12_I2C_ADDRESS 0x19
#define LIS2DW12_REG_CTRL1 0x20
#define LIS2DW12_REG_CTRL2 0x21
#define LIS2DW12_REG_CTRL4_INT1_PAD_CTRL 0x23
#define LIS2DW12_REG_CTRL6 0x25
#define LIS2DW12_REG_CTRL7 0x3F
#define LIS2DW12_REG_WAKE_UP_THS 0x34
#define LIS2DW12_REG_WAKE_UP_DUR 0x35

// ===================== LEDS  ====================
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"
#define LED_STRIP_RMT_CHANNEL RMT_CHANNEL_0
#define LED_STRIP_RES_HZ      (10 * 1000 * 1000) // 10MHz resolution
#define NUM_LEDS              60
#define LED_DATA_PIN          10   // change to your pin
#define LED_EN_PIN            2  // PMOSFET enable pin (active low)
static led_strip_handle_t led_strip;

static const char *TAG = "wristwatch";

// Flag to indicate WiFi time sync complete
static bool time_synced = false;

static void init_led_strip(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_DATA_PIN,   // The GPIO that connected to the LED strip's data line
        .max_leds = NUM_LEDS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = (10 * 1000 * 1000), // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
    };

    // LED Strip object handle
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");

    // Setup PMOS enable pin
    gpio_reset_pin(LED_EN_PIN);
    gpio_set_direction(LED_EN_PIN, GPIO_MODE_OUTPUT);
}

// ========== Event Handler ==========
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Connected, starting SNTP...");
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "pool.ntp.org");
        esp_sntp_init();
    }
}

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized");
    time_synced = true;
}

// ========== WiFi Init ==========
static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static esp_err_t imu_write_reg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM,
                                      LIS2DW12_I2C_ADDRESS,
                                      data, sizeof(data),
                                      1000 / portTICK_PERIOD_MS);
}

// Returns microseconds until next Wednesday 00:00
uint64_t time_until_next_wednesday_midnight_us(void)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Days until Wednesday (tm_wday: Sunday=0 .. Saturday=6)
    int days_ahead = (3 - timeinfo.tm_wday + 7) % 7;
    if (days_ahead == 0) {
        // If today is Wednesday, check if it's already past midnight
        if (timeinfo.tm_hour > 0 || timeinfo.tm_min > 0 || timeinfo.tm_sec > 0) {
            days_ahead = 7; // schedule for next week
        }
    }

    // Build target Wednesday midnight
    struct tm target = timeinfo;
    target.tm_mday += days_ahead;
    target.tm_hour = 0;
    target.tm_min = 0;
    target.tm_sec = 0;
    target.tm_isdst = -1;

    time_t target_time = mktime(&target);
    return (uint64_t)(target_time - now) * 1000000ULL;
}

// ========== IMU Init (stub) ==========
static void imu_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    // Ctrl1: 12.5Hz, Low Power Mode, LP Mode 1 (~1uA @1v8)
    ESP_ERROR_CHECK(imu_write_reg(LIS2DW12_REG_CTRL1, 0x20));

    // Ctrl6: High Pass enabled? +-4g FS
    ESP_ERROR_CHECK(imu_write_reg(LIS2DW12_REG_CTRL6, 0b00011000)); // High-pass filter disabled

    // Ctrl7: Enable interrupts
    ESP_ERROR_CHECK(imu_write_reg(LIS2DW12_REG_CTRL7, 0x20));

    // Wakeup duration 2? samples above threshold. Really odd bits.
    ESP_ERROR_CHECK(imu_write_reg(LIS2DW12_REG_WAKE_UP_DUR, 0b01100000));

    // Wakeup threshold in gs LSB = 1/64 FS. FS is 4G. This is set as 0.75g.
    ESP_ERROR_CHECK(imu_write_reg(LIS2DW12_REG_WAKE_UP_THS, 0b00001100));

    // Interrupt1 pulse wakeup signal
    ESP_ERROR_CHECK(imu_write_reg(LIS2DW12_REG_CTRL4_INT1_PAD_CTRL, 0x20));

    ESP_LOGI(TAG, "IMU configured");
}

/// @brief Set LED enable pin
/// @param state On|Off state True|False
static void enable_leds(bool state)
{
    gpio_set_level(LED_EN_PIN, !state); // active low
}

static void clear_strip(void)
{
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
}

// ========== Display Time ==========
static void display_time(void)
{
    init_led_strip();
    enable_leds(true);
    clear_strip();

    int64_t start_time = esp_timer_get_time() / 1000; // ms

    while ((esp_timer_get_time() / 1000) - start_time < 10000) { // 10s
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        uint8_t hour = timeinfo.tm_hour;
        uint8_t minute = timeinfo.tm_min;
        uint8_t second = timeinfo.tm_sec;

        uint8_t hourIndex   = 59 - ((((hour * 60 + minute) / 24) + 29) % NUM_LEDS);
        uint8_t minuteIndex = 59 - ((minute + 29) % NUM_LEDS);
        uint8_t secondIndex = 59 - ((second + 29) % NUM_LEDS);

        led_strip_clear(led_strip);
        // Color mixing logic
        if (hourIndex == minuteIndex && minuteIndex == secondIndex) {
            // All three overlap: White (red + green + blue)
            led_strip_set_pixel(led_strip, hourIndex, 5, 5, 5);
        } else {
            // Hour pixel
            if (hourIndex == secondIndex && hourIndex != minuteIndex) {
                // Purple (blue + red)
                led_strip_set_pixel(led_strip, hourIndex, 5, 0, 5);
            } else if (hourIndex == minuteIndex && hourIndex != secondIndex) {
                // Cyan (green + blue)
                led_strip_set_pixel(led_strip, hourIndex, 0, 5, 5);
            } else if (hourIndex != minuteIndex && hourIndex != secondIndex) {
                led_strip_set_pixel(led_strip, hourIndex, 0, 0, 5); // blue
            }
            // Minute pixel
            if (minuteIndex == secondIndex && minuteIndex != hourIndex) {
                // Yellow (red + green)
                led_strip_set_pixel(led_strip, minuteIndex, 5, 5, 0);
            } else if (minuteIndex == hourIndex && minuteIndex != secondIndex) {
                // Cyan (green + blue)
                led_strip_set_pixel(led_strip, minuteIndex, 0, 5, 5);
            } else if (minuteIndex != hourIndex && minuteIndex != secondIndex) {
                led_strip_set_pixel(led_strip, minuteIndex, 0, 5, 0); // green
            }
            // Second pixel
            if (secondIndex != hourIndex && secondIndex != minuteIndex) {
                led_strip_set_pixel(led_strip, secondIndex, 5, 0, 0); // red
            }
        }
        led_strip_refresh(led_strip);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    clear_strip();
    enable_leds(false);
    ESP_LOGI(TAG, "LEDs off after 10s");
}


static void configure_gpio(int pinNumber, gpio_mode_t mode, gpio_pullup_t pullUp, gpio_pulldown_t pullDown) {
    // Configure GPIO Pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pinNumber),
        .mode = mode,
        .pull_up_en = pullUp,
        .pull_down_en = pullDown,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// ========== Display Charging Percent ==========
static void display_charge_percent(void)
{
    configure_gpio(BAT_MEAS_EN_N, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE);
    configure_gpio(BAT_MEAS_ADC, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE);
    init_led_strip();
    enable_leds(true);
    clear_strip();

    int adc_value;
    adc_oneshot_unit_handle_t adc_handle;

    // Initialize ADC Oneshot Mode Driver on the ADC Unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Configure ADC channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_4, &config));

    // Simple moving average smoothing
    const int SMOOTH_N = 8;
    int adc_buffer[SMOOTH_N];
    int buffer_index = 0;
    int buffer_filled = 0;

    // While the BMS Status Pin is pulled Low
    while (!gpio_get_level(BMS_STATUS_N) && !gpio_get_level(BMS_STATUS_N)) {
        // Read ADC value with Oneshot
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_4, &adc_value));

        // Store in buffer
        adc_buffer[buffer_index] = adc_value;
        buffer_index = (buffer_index + 1) % SMOOTH_N;
        if (buffer_filled < SMOOTH_N) buffer_filled++;

        // Compute average
        int sum = 0;
        for (int i = 0; i < buffer_filled; i++) {
            sum += adc_buffer[i];
        }
        float avg_adc = (float)sum / buffer_filled;

        float voltage = ((avg_adc / 4095.0f) * 2.50f) * (2.20f); // ADC to voltage conversion
        ESP_LOGI(TAG, "ADC Value (smoothed): %.1f, Voltage: %.2f V", avg_adc, voltage);

        // 3.20v to 4.20v maps to 0% to 100%
        int percent = (int)(((voltage - 3.20f) / (4.20f - 3.20f)) * 100.0f);
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
        ESP_LOGI(TAG, "Battery Charge: %d%%", percent);
        int leds_to_light = (percent * NUM_LEDS) / 100;

        led_strip_clear(led_strip);

        for (int i = 0; i < leds_to_light; i++) {
            int ledIndex = 59 - ((i + 29) % NUM_LEDS);
            led_strip_set_pixel(led_strip, ledIndex, 0, 2, 0); // green
        }

        led_strip_refresh(led_strip);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    configure_gpio(BAT_MEAS_EN_N, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE);
    clear_strip();
    enable_leds(false);
    ESP_LOGI(TAG, "LEDs off after 10s");
}

// ========== Setup Deep Sleep ==========
static void enter_deep_sleep(void)
{
    // Stop WiFi stack cleanly if it was started
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_netif_deinit();
    esp_event_loop_delete_default();

    // De-Init I2C driveresp_deep_sleep_enable_gpio_wakeup
    i2c_driver_delete(I2C_MASTER_NUM);

    // Configure IMU interrupt as wakeup source
    esp_deep_sleep_enable_gpio_wakeup((1ULL << IMU_INT_PIN), ESP_GPIO_WAKEUP_GPIO_HIGH); // High level triggers wake

    // Configure BMS Status Pin as wakeup source.
    esp_deep_sleep_enable_gpio_wakeup((1ULL << BMS_STATUS_N), ESP_GPIO_WAKEUP_GPIO_LOW); // Low level triggers wake

    // Configure weekly wakeup (Wed 00:00)
    // For now, simulate with fixed interval (e.g. 7 days * 24h * 3600s)
    // uint64_t wake_interval_us = 7ULL * 24ULL * 3600ULL * 1000000ULL;
    // For now, simulate with fixed interval 5 minutes for validation of logics.
    // uint64_t wake_interval_us = 300ULL * 1000000ULL;
    esp_sleep_enable_timer_wakeup(time_until_next_wednesday_midnight_us());

    ESP_LOGI(TAG, "Entering deep sleep...");
    esp_deep_sleep_start();
}

// ========== app_main ==========
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize timezone (for EST/EDT)
    setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0/2", 1);
    tzset();

    // Configure BMS GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BMS_STATUS_N),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Check wakeup reason
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
        // First boot
        ESP_LOGI(TAG, "First boot: configure IMU + sync time");
        imu_init();
        wifi_init_sta();
        sntp_set_time_sync_notification_cb(time_sync_notification_cb);

        int64_t wifi_start_time = esp_timer_get_time() / 1000; // ms
        const int64_t wifi_timeout_ms = 60000; // 1 minutes
        while (!time_synced) {
            if ((esp_timer_get_time() / 1000) - wifi_start_time > wifi_timeout_ms) {
                ESP_LOGW(TAG, "WiFi/SNTP sync timeout. Returning to sleep.");
                enter_deep_sleep();
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    else if (cause == ESP_SLEEP_WAKEUP_GPIO) {
        if (gpio_get_level(BMS_STATUS_N)) {
            ESP_LOGI(TAG, "Wakeup from IMU interrupt.");
            display_time();
        } else {
            ESP_LOGI(TAG, "Wakeup from BMS interrupt.");
            display_charge_percent();
        }
    }
    else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(TAG, "Weekly wakeup: sync RTC via WiFi");
        wifi_init_sta();
        sntp_set_time_sync_notification_cb(time_sync_notification_cb);

        int64_t wifi_start_time = esp_timer_get_time() / 1000; // ms
        const int64_t wifi_timeout_ms = 120000; // 2 minutes
        while (!time_synced) {
            if ((esp_timer_get_time() / 1000) - wifi_start_time > wifi_timeout_ms) {
                ESP_LOGW(TAG, "WiFi/SNTP sync timeout. Returning to sleep.");
                enter_deep_sleep();
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    else {
        ESP_LOGI(TAG, "Unknown wakeup cause %d", cause);
    }

    // Always return to sleep
    enter_deep_sleep();
}
