/*
 * ESP32 GPIO Probe (ESP-IDF v6)
 *
 * ใช้วงจรแบ่งแรงดันชุดเดิมเป็น "หัววัด":
 *
 *   หัววัด ── 10kΩ ──┬── 10kΩ ── GND
 *                    │
 *                 GPIO34
 *
 * วิธีใช้: เอาปลายสาย "หัววัด" ไปจิ้ม/เสียบที่ขาที่อยากทดสอบ
 *   - ขา S ของ GPIO ไหนก็ได้ -> บอกว่าเป็น GPIO เบอร์อะไร + แรงดันตอน HIGH/LOW
 *   - ขา V / 3V3 / VIN       -> บอกว่าเป็นขาไฟ 3.3V หรือ 5V
 *   - GND หรือไม่ได้เสียบ    -> บอกว่าไม่มีไฟ
 *
 * หลักการ: ESP32 จะสั่งขา GPIO ให้เป็น HIGH ทีละขา แล้วดูว่า GPIO34 เห็นไฟตอนไหน
 * ข้อควรระวัง: ถอดสายสัญญาณของโมดูลอื่นออกก่อนทดสอบ เพราะโค้ดจะสั่งขาเป็น HIGH ชั่วขณะ
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_rom_sys.h"

#define PROBE_CH   ADC_CHANNEL_6          /* GPIO34 */
#define R_TOP      10000.0f
#define R_BOTTOM   10000.0f
#define DIVIDER    ((R_TOP + R_BOTTOM) / R_BOTTOM)
#define SAMPLES    16

/* ขาที่สั่ง output ได้ และไม่ใช่ขา flash (6-11), UART (1,3), ปุ่ม BOOT (0)
 * ขา 34-39 เป็น input อย่างเดียว ทดสอบด้วยวิธีนี้ไม่ได้ */
static const int TEST_PINS[] = {
    2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19,
    21, 22, 23, 25, 26, 27, 32, 33
};
#define N_PINS (sizeof(TEST_PINS) / sizeof(TEST_PINS[0]))

typedef struct {
    int   pin;      /* เบอร์ GPIO ที่เจอ, -1 = ไม่ใช่ GPIO ที่ทดสอบ */
    float v_idle;   /* แรงดันก่อนสั่งขาใดๆ */
    float v_high;   /* แรงดันตอนสั่งขานั้นเป็น HIGH */
    float v_low;    /* แรงดันตอนสั่งขานั้นเป็น LOW */
} probe_result_t;

static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t cali;

static void adc_setup(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, PROBE_CH, &ch_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .default_vref = 1100,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_cfg, &cali));
}

/* ตั้งทุกขาเป็น input แบบไม่มี pull-up/down เพื่อไม่ให้รบกวนการวัด */
static void pins_init(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < N_PINS; i++) {
        mask |= 1ULL << TEST_PINS[i];
    }
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

/* อ่านแรงดันที่หัววัด (คูณกลับวงจรแบ่งแรงดันแล้ว) หน่วย V */
static float read_volts(void)
{
    int sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int mv = 0;
        adc_oneshot_get_calibrated_result(adc, cali, PROBE_CH, &mv);
        sum += mv;
    }
    return (sum / (float)SAMPLES) * DIVIDER / 1000.0f;
}

static probe_result_t scan(void)
{
    probe_result_t r = { .pin = -1 };

    r.v_idle = read_volts();
    if (r.v_idle > 0.3f) {
        return r;  /* มีไฟจากที่อื่นอยู่แล้ว (ขาไฟ หรือโมดูลขับอยู่) ไม่ต้องสลับขา */
    }

    for (int i = 0; i < N_PINS; i++) {
        int p = TEST_PINS[i];

        gpio_set_level(p, 1);
        gpio_set_direction(p, GPIO_MODE_OUTPUT);
        esp_rom_delay_us(100);
        float vh = read_volts();

        gpio_set_level(p, 0);
        esp_rom_delay_us(100);
        float vl = read_volts();

        gpio_set_direction(p, GPIO_MODE_INPUT);

        if (vh > 1.5f) {
            r.pin = p;
            r.v_high = vh;
            r.v_low = vl;
            break;
        }
    }
    return r;
}

/* แปลงผลเป็นรหัส เพื่อ print เฉพาะตอนที่เปลี่ยนจุดเสียบ */
static int result_key(const probe_result_t *r)
{
    if (r->pin >= 0)                          return r->pin;
    if (r->v_idle > 4.3f)                     return 105;  /* 5V */
    if (r->v_idle > 2.9f && r->v_idle < 3.6f) return 103;  /* 3.3V */
    if (r->v_idle > 0.3f)                     return 101;  /* แรงดันอื่นๆ */
    return 100;                                            /* ไม่มีไฟ */
}

static void print_result(const probe_result_t *r)
{
    if (r->pin >= 0) {
        bool ok = (r->v_high >= 3.0f) && (r->v_low <= 0.3f);
        printf("[GPIO %2d]  HIGH = %.2f V   LOW = %.2f V   -> %s\n",
               r->pin, r->v_high, r->v_low,
               ok ? "ปกติ" : "ผิดปกติ (ขาอาจเสีย หรือมีอะไรต่อพ่วงอยู่)");
    } else if (r->v_idle > 4.3f) {
        printf("[ขาไฟ 5V]   %.2f V   (ขา V ตอน jumper อยู่ที่ 5V หรือขา VIN)\n", r->v_idle);
    } else if (r->v_idle > 2.9f && r->v_idle < 3.6f) {
        printf("[ขาไฟ 3.3V] %.2f V   (ขา V ตอน jumper อยู่ที่ 3V3 หรือขา 3V3)\n", r->v_idle);
    } else if (r->v_idle > 0.3f) {
        printf("[มีแรงดัน]  %.2f V   (อาจเป็นขาสัญญาณที่มีโมดูลขับไฟอยู่)\n", r->v_idle);
    } else {
        printf("[ไม่มีไฟ]   ยังไม่ได้เสียบ / เสียบที่ GND / หรือเป็นขา GPIO 0,1,3,34-39\n");
    }
}

void app_main(void)
{
    adc_setup();
    pins_init();

    printf("\n========== ESP32 GPIO PROBE ==========\n");
    printf("เอาสายหัววัดไปเสียบที่ขาที่อยากทดสอบ แล้วดูผลด้านล่าง\n");
    printf("(จะ print ใหม่ทุกครั้งที่ย้ายจุดเสียบ)\n");
    printf("======================================\n\n");

    int last_key = -1;
    int candidate = -1;
    int stable = 0;

    while (1) {
        probe_result_t r = scan();
        int key = result_key(&r);

        /* ต้องได้ผลเดิม 2 รอบติดกันก่อน กันค่ากระตุกตอนกำลังเสียบสาย */
        if (key == candidate) {
            stable++;
        } else {
            candidate = key;
            stable = 1;
        }

        if (stable == 2 && key != last_key) {
            print_result(&r);
            last_key = key;
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}