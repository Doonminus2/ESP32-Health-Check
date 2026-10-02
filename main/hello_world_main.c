/*
 * ESP32 Health Check + VIN Monitor (ESP-IDF v5.1+)
 * ------------------------------------------------
 * 1) ตรวจชิป: รุ่น, revision, flash, MAC, heap, สาเหตุการ reset
 * 2) ทดสอบ Wi-Fi: สแกนหา AP รอบตัว (ถ้าเจอ = ภาค RF ยังดี)
 * 3) วัด VIN ผ่าน ADC1 + วงจรแบ่งแรงดัน (ห้ามต่อ VIN เข้าขา GPIO ตรงๆ!)
 *
 * การต่อวงจร (ค่าเริ่มต้น 20k / 10k -> 5V เหลือ ~1.67V ที่ขา GPIO):
 *
 *   VIN(5V) ---[ R_TOP 20k ]---+---[ R_BOTTOM 10k ]--- GND
 *                              |
 *                         VIN_SENSE_GPIO
 *
 *   ถ้ามีแค่ 10k ให้เอา 10k สองตัวต่ออนุกรมเป็น R_TOP (=20k)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "HEALTH";

/* ================== ตั้งค่าตรงนี้ ================== */
// ต้องเป็นขาของ ADC1 เท่านั้น (ADC2 ใช้ร่วมกับ Wi-Fi ไม่ได้)
//   ESP32 (classic): GPIO 32-39   | ESP32-S3: GPIO 1-10 | ESP32-C3: GPIO 0-4
#define VIN_SENSE_GPIO   34
#define R_TOP_OHM        10000.0f
#define R_BOTTOM_OHM     10000.0f  // GPIO -> GND
#define ADC_SAMPLES      64         // เฉลี่ยกี่ครั้งต่อการอ่าน 1 รอบ
/* =================================================== */

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define VIN_ADC_ATTEN ADC_ATTEN_DB_12
#else
#define VIN_ADC_ATTEN ADC_ATTEN_DB_11
#endif

static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t s_chan;
static adc_cali_handle_t s_cali;
static bool s_cali_ok = false;

/* ---------- 1) ตรวจชิป ---------- */
static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "Power-on (ปกติ)";
    case ESP_RST_EXT:       return "External pin";
    case ESP_RST_SW:        return "Software restart";
    case ESP_RST_PANIC:     return "PANIC - โปรแกรมก่อนหน้า crash!";
    case ESP_RST_INT_WDT:   return "Interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "Task watchdog";
    case ESP_RST_WDT:       return "Other watchdog";
    case ESP_RST_DEEPSLEEP: return "Wake from deep sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT - ไฟเลี้ยงตก!";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "Unknown";
    }
}

static void check_chip(void)
{
    esp_chip_info_t info;
    esp_chip_info(&info);

    uint32_t flash_size = 0;
    esp_err_t ferr = esp_flash_get_size(NULL, &flash_size);

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    printf("\n========== ESP32 HEALTH CHECK ==========\n");
    printf("Target       : %s\n", CONFIG_IDF_TARGET);
    printf("CPU cores    : %d\n", info.cores);
    printf("Revision     : v%d.%d\n", info.revision / 100, info.revision % 100);
    printf("Features     : %s%s%s%s\n",
           (info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
           (info.features & CHIP_FEATURE_BT)       ? "BT "   : "",
           (info.features & CHIP_FEATURE_BLE)      ? "BLE "  : "",
           (info.features & CHIP_FEATURE_EMB_FLASH) ? "EmbFlash " : "");
    if (ferr == ESP_OK) {
        printf("Flash size   : %lu MB\n", (unsigned long)(flash_size / (1024 * 1024)));
    } else {
        printf("Flash size   : อ่านไม่ได้ (%s) <-- ผิดปกติ\n", esp_err_to_name(ferr));
    }
    printf("MAC (STA)    : %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("Free heap    : %lu bytes\n", (unsigned long)esp_get_free_heap_size());
    printf("Reset reason : %s\n", reset_reason_str(esp_reset_reason()));
    printf("IDF version  : %s\n", esp_get_idf_version());
    printf("=========================================\n\n");
}

/* ---------- 2) ทดสอบ Wi-Fi ---------- */
static void wifi_scan_test(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Scanning Wi-Fi...");
    err = esp_wifi_scan_start(NULL, true);   // blocking scan
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi scan FAIL: %s", esp_err_to_name(err));
        return;
    }

    uint16_t total = 0;
    esp_wifi_scan_get_ap_num(&total);

    wifi_ap_record_t recs[10];
    uint16_t n = 10;
    esp_wifi_scan_get_ap_records(&n, recs);

    if (total == 0) {
        ESP_LOGW(TAG, "Wi-Fi: ไม่พบ AP เลย (ถ้ารอบตัวมี Wi-Fi อยู่ แสดงว่าภาค RF/เสาอาจมีปัญหา)");
    } else {
        ESP_LOGI(TAG, "Wi-Fi OK: พบ %u AP", total);
        for (int i = 0; i < n; i++) {
            printf("  %2d) %-32s RSSI %d dBm\n", i + 1, (char *)recs[i].ssid, recs[i].rssi);
        }
    }
    esp_wifi_stop();
    printf("\n");
}

/* ---------- 3) วัด VIN ---------- */
static void vin_adc_init(void)
{
    adc_unit_t unit;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(VIN_SENSE_GPIO, &unit, &s_chan));
    if (unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "GPIO%d ไม่ใช่ ADC1 - เปลี่ยน VIN_SENSE_GPIO", VIN_SENSE_GPIO);
        abort();
    }

    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &s_adc));

    adc_oneshot_chan_cfg_t ccfg = {
        .atten = VIN_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, s_chan, &ccfg));

    // Calibration (ใช้ค่าที่โรงงานเขียนไว้ใน eFuse)
    esp_err_t err = ESP_FAIL;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1,
        .chan = s_chan,
        .atten = VIN_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cal, &s_cali);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1,
        .atten = VIN_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_line_fitting(&cal, &s_cali);
#endif
    s_cali_ok = (err == ESP_OK);
    if (s_cali_ok) {
        ESP_LOGI(TAG, "ADC calibration: OK");
    } else {
        ESP_LOGW(TAG, "ADC calibration: ไม่มี - ค่าที่ได้จะเป็นค่าประมาณหยาบๆ");
    }
}

static float read_vin_volts(int *pin_mv_out)
{
    int64_t sum_mv = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        int raw = 0, mv = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc, s_chan, &raw));
        if (s_cali_ok) {
            adc_cali_raw_to_voltage(s_cali, raw, &mv);
        } else {
            mv = raw * 3100 / 4095;   // ประมาณ (12-bit, atten 11/12 dB)
        }
        sum_mv += mv;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    int pin_mv = (int)(sum_mv / ADC_SAMPLES);
    if (pin_mv_out) *pin_mv_out = pin_mv;

    float ratio = (R_TOP_OHM + R_BOTTOM_OHM) / R_BOTTOM_OHM;
    return (pin_mv / 1000.0f) * ratio;
}

static const char *judge_vin(float v)
{
    if (v < 0.3f)  return "ไม่มีไฟ / ยังไม่ได้ต่อวงจรแบ่งแรงดัน";
    if (v < 4.4f)  return "ต่ำเกินไป (สาย USB/อะแดปเตอร์จ่ายไม่พอ?)";
    if (v < 4.75f) return "ค่อนข้างต่ำ แต่ยังใช้ได้ (ปกติถ้าผ่านไดโอดบนบอร์ด)";
    if (v <= 5.25f) return "ปกติ (~5V)";
    if (v <= 5.6f) return "สูงกว่าปกติเล็กน้อย";
    return "สูงเกิน! ตรวจแหล่งจ่าย";
}

void app_main(void)
{
    check_chip();
    wifi_scan_test();
    vin_adc_init();

    ESP_LOGI(TAG, "เริ่มวัด VIN ที่ GPIO%d (R_TOP=%.0f, R_BOTTOM=%.0f)",
             VIN_SENSE_GPIO, R_TOP_OHM, R_BOTTOM_OHM);

    while (1) {
        int pin_mv = 0;
        float vin = read_vin_volts(&pin_mv);
        printf("Pin: %4d mV  ->  VIN: %.2f V   [%s]\n", pin_mv, vin, judge_vin(vin));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}