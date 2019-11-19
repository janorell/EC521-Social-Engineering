#include <stdio.h>
#include <stdlib.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "speed.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (3.4179) // sample test interval for the first timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

#define HIGH  1
#define LOW   0

// A simple structure to pass "events" to main task
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

// Initialize queue handler for timer-based events
xQueueHandle timer_queue;

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para) {
    int timer_idx = (int)para;

    // Retrieve the interrupt status and the counter value from the timer that reported the interrupt
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value =
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

    // Prepare basic event data that will be then sent back to the main program task
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    // Clear the interrupt and update the alarm time for the timer with without reload
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        evt.type = TEST_WITHOUT_RELOAD;
        TIMERG0.int_clr_timers.t0 = 1; // Clear the interrupt, Timer 0 in group 0
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32); // After the alarm triggers, we need to re-enable it to trigger it next time
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        evt.type = TEST_WITH_RELOAD;
        TIMERG0.int_clr_timers.t1 = 1; // Clear the interrupt, Timer 0 in group 0
    } else {
        evt.type = -1; // not supported even type
    }

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Initialize timer 0 in group 0 for 1 sec alarm interval and auto reload
static void alarm_init(int timer_idx) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_DIS;
    config.intr_type = TIMER_INTR_LEVEL;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    // Timer's counter will initially start from value below.
    // Also, if auto_reload is set, this value will be automatically reload on alarm
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    // // Configure the alarm value and the interrupt on alarm.
    // timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    // timer_enable_intr(TIMER_GROUP_0, timer_idx);
    // timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
        // (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    // // Start timer
    // timer_start(TIMER_GROUP_0, timer_idx);
}

static void check_efuse() {
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type) {
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static int getState(uint32_t adc_reading) {
  if(adc_reading < 1500) return HIGH;
  else return LOW;
}

// static uint32_t measureTimeForRisingEdges (uint32_t adc_reading) {
//   uint32_t time = 0;
//   int state;
//   if(adc_reading < 1500) state = HIGH;
//   else state = LOW;
//
// }

double speed_app_main() {
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Configure timer
    timer_queue = xQueueCreate(10, sizeof(timer_event_t)); // Create a FIFO queue for timer-based
    alarm_init(TIMER_0); // Initiate alarm using timer API

    int prevState = LOW;
    int risingEdgeCount = 0;
    double counter_time_sec = 0;
    double speed = 0;
    double distance = 10.3;  // in centimeters

    while (1) {
        uint32_t adc_reading = 0;

        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }

        adc_reading /= NO_OF_SAMPLES;

        // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        int state = getState(adc_reading);
        if(state == HIGH && prevState == LOW) {// from LOW to HIGH
          if(risingEdgeCount == 0) {
            timer_start(TIMER_GROUP_0, TIMER_0);
          } else if (risingEdgeCount == 1) {
            timer_pause(TIMER_GROUP_0, TIMER_0);
            esp_err_t ret = timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &counter_time_sec);
            speed = distance / counter_time_sec;
            timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
            timer_start(TIMER_GROUP_0, TIMER_0);
            risingEdgeCount = 0;
            counter_time_sec = 0;
            printf("Speed: %fcm/s\n", speed);
            return speed;
          }
        } else if (state == LOW && prevState == HIGH) {// from HIGH to LOW
          risingEdgeCount = 1;
        }
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        // uint32_t dist_mm = adc_reading * 5 + 300;
        // printf("Distance: %dmm\n", dist_mm);
        prevState = state;
        vTaskDelay(1);
        //vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
