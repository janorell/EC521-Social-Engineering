/*
  - Quest4
  - Code to run crawler

  Team7: Vanessa Schuweh, Vindhya Kuchibhotla, Jennifer Norell
  November 08, 2019
*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sys/param.h>
#include <lwip/netdb.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_attr.h"
#include "esp_types.h"
#include "esp_adc_cal.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"

#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/uart_struct.h"

#include "driver/mcpwm.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "driver/uart.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

// 14-Segment Display for Alphanumeric
#define SLAVE_ADDR_ALPHA                   0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

//WiFi Init
#ifdef CONFIG_EXAMPLE_IPV4
  #define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
  #define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif
#define PORT CONFIG_EXAMPLE_PORT
static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// LIDAR
#define SLAVE_ADDR                         0x62 // 0x62
#define RegisterMeasure                    0x00 // Register to write to initiate ranging.
#define MeasureValue                       0x04 // Value to initiate ranging.
#define RegisterHighLowB                   0x8f // Register to get both High and Low bytes in 1 call.
#define LIDAR_DIST                         50 //set distance to stop in cm

// Speed sensor
#define PCNT_TEST_UNIT                     PCNT_UNIT_0
#define PCNT_H_LIM_VAL                     10000
#define PCNT_L_LIM_VAL                     -10
#define PCNT_THRESH1_VAL                   10000
#define PCNT_THRESH0_VAL                   -5
#define PCNT_INPUT_SIG_IO                  34  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO                 5  // Control GPIO HIGH=count up, LOW=count down

//Timer 
#define TIMER_DIVIDER                      16  //  Hardware timer clock divider
#define TIMER_SCALE                        (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC                (1.0) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC                (1.0) // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD                0  // testing will be done without auto reload
#define TEST_WITH_RELOAD                   1  // testing will be done with auto reload
#define DEFAULT_VREF                       1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES                      5 //Multisampling

//Micro Lidar
#define RX (16)
#define TX (17)
#define RX2 (25)
#define TX2 (26)
static const int BUF_SIZE = 256;
static const int BUF_SIZE2 = 256;

//PID speed
#define setpoint 0.2 //speed for PID to maintain in m/s
#define Kp 0.75
#define Ki 0.5
#define Kd 0.0

//global variables
int dt_complete = 0;
double derivative, error, output;
double dt = 0.1;
double prev_error = 0.0; // Set up PID loop
double integral = 0.0;
double speed;
int start_flag = 0; 
int motor_speed; 

// ALPHANUMERIC /////////////////////////////////////////////////////////////////////////////////
static const uint16_t alphafonttable[] = { //table of alphanumeric codes
    0b0000000000000001,
    0b0000000000000010,
    0b0000000000000100,
    0b0000000000001000,
    0b0000000000010000,
    0b0000000000100000,
    0b0000000001000000,
    0b0000000010000000,
    0b0000000100000000,
    0b0000001000000000,
    0b0000010000000000,
    0b0000100000000000,
    0b0001000000000000,
    0b0010000000000000,
    0b0100000000000000,
    0b1000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000,
    0b0001001011001001,
    0b0001010111000000,
    0b0001001011111001,
    0b0000000011100011,
    0b0000010100110000,
    0b0001001011001000,
    0b0011101000000000,
    0b0001011100000000,
    0b0000000000000000, //
    0b0000000000000110, // !
    0b0000001000100000, // "
    0b0001001011001110, // #
    0b0001001011101101, // $
    0b0000110000100100, // %
    0b0010001101011101, // &
    0b0000010000000000, // '
    0b0010010000000000, // (
    0b0000100100000000, // )
    0b0011111111000000, // *
    0b0001001011000000, // +
    0b0000100000000000, // ,
    0b0000000011000000, // -
    0b0100000000000000, // .
    0b0000110000000000, // /
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0001001000000000, // :
    0b0000101000000000, // ;
    0b0010010000000000, // <
    0b0000000011001000, // =
    0b0000100100000000, // >
    0b0001000010000011, // ?
    0b0000001010111011, // @
    0b0000000011110111, // A
    0b0001001010001111, // B
    0b0000000000111001, // C
    0b0001001000001111, // D
    0b0000000011111001, // E
    0b0000000001110001, // F
    0b0000000010111101, // G
    0b0000000011110110, // H
    0b0001001000000000, // I
    0b0000000000011110, // J
    0b0010010001110000, // K
    0b0000000000111000, // L
    0b0000010100110110, // M
    0b0010000100110110, // N
    0b0000000000111111, // O
    0b0000000011110011, // P
    0b0010000000111111, // Q
    0b0010000011110011, // R
    0b0000000011101101, // S
    0b0001001000000001, // T
    0b0000000000111110, // U
    0b0000110000110000, // V
    0b0010100000110110, // W
    0b0010110100000000, // X
    0b0001010100000000, // Y
    0b0000110000001001, // Z
    0b0000000000111001, // [
    0b0010000100000000, //
    0b0000000000001111, // ]
    0b0000110000000011, // ^
    0b0000000000001000, // _
    0b0000000100000000, // `
    0b0001000001011000, // a
    0b0010000001111000, // b
    0b0000000011011000, // c
    0b0000100010001110, // d
    0b0000100001011000, // e
    0b0000000001110001, // f
    0b0000010010001110, // g
    0b0001000001110000, // h
    0b0001000000000000, // i
    0b0000000000001110, // j
    0b0011011000000000, // k
    0b0000000000110000, // l
    0b0001000011010100, // m
    0b0001000001010000, // n
    0b0000000011011100, // o
    0b0000000101110000, // p
    0b0000010010000110, // q
    0b0000000001010000, // r
    0b0010000010001000, // s
    0b0000000001111000, // t
    0b0000000000011100, // u
    0b0010000000000100, // v
    0b0010100000010100, // w
    0b0010100011000000, // x
    0b0010000000001100, // y
    0b0000100001001000, // z
    0b0000100101001001, // {
    0b0001001000000000, // |
    0b0010010010001001, // }
    0b0000010100100000, // ~
    0b0011111111111111,

};

int alpha_oscillator() {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

int no_blink() {
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, ( SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

int set_brightness_max(uint8_t val) {
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, ( SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

static void alpha_display (char string[5]){ //function to display onto alphanumeric
    int current[] = {0,0,0,0};
    uint16_t displaybuffer[8];
    for (int i = 0; i < 4; i++) {
        current[i] = string[i];
    }
    for (int i = 0; i < 4; i++){
        displaybuffer[i] = alphafonttable[current[i]];
    }
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, (SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd4);
}
////////////////////////////////////////////////////////////////////////////////////////////////

// Microlidars /////////////////////////////////////////////////////////////////////////////////
static void uart_initialize() {
  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

  uart_config_t uart_config2 = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_2, &uart_config2);
  uart_set_pin(UART_NUM_2, TX2, RX2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_2, BUF_SIZE2 * 2, 0, 0, NULL, 0);
}

static int sensors1() { //function that returns distance from microLidar sensor 1
  uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
  uint32_t distance = 0;

  uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 100);
  uart_flush(UART_NUM_1);
  if (data[0] == 0x59 && data[1] == 0x59) {
    distance = (data[3]<<8)+data[2];
  }
  printf("Distance Sensor 1: %dcm\n", distance);
  return distance;
}

static int sensors2() { //function that returns distance from microLidar sensor 2
  uint8_t *data2 = (uint8_t *) malloc(BUF_SIZE2);
  uint32_t distance2 = 0;

  uart_read_bytes(UART_NUM_2, data2, BUF_SIZE2, 100);
  uart_flush(UART_NUM_2);
  if (data2[0] == 0x59 && data2[1] == 0x59) {
    distance2 = (data2[3]<<8)+data2[2];
  }
  printf("Distance Sensor 2: %dcm\n", distance2+40);
  return distance2;
}
////////////////////////////////////////////////////////////////////////////////////////////////

// PID /////////////////////////////////////////////////////////////////////////////////////////
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
void IRAM_ATTR timer_group1_isr(void *para) {
  int timer_idx = (int)para;

  // Retrieve the interrupt status and the counter value from the timer that reported the interrupt
  uint32_t intr_status = TIMERG0.int_st_timers.val;
  TIMERG0.hw_timer[timer_idx].update = 1;
  uint64_t timer_counter_value = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;

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

  // PID
  TIMERG1.int_clr_timers.t0 = 1;
  // Indicate timer has fired
  dt_complete = 1;
}

static void periodic_timer_init() {
  /* Select and initialize basic parameters of the timer */
  timer_config_t config;
  config.divider = TIMER_DIVIDER;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_PAUSE;
  config.alarm_en = TIMER_ALARM_EN;
  config.intr_type = TIMER_INTR_LEVEL;
  config.auto_reload = TEST_WITH_RELOAD;
  timer_init(TIMER_GROUP_1, TIMER_0, &config);

  // Timer's counter will initially start from value below
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);

  // Configure the alarm value and the interrupt on alarm
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, TIMER_INTERVAL1_SEC * TIMER_SCALE);
  timer_enable_intr(TIMER_GROUP_1, TIMER_0);
  timer_isr_register(TIMER_GROUP_1, TIMER_0, timer_group1_isr, (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

  // Start timer
  timer_start(TIMER_GROUP_1, TIMER_0);
}
///////////////////////////////////////////////////////////////////////////////////////////////

// Speed //////////////////////////////////////////////////////////////////////////////////////
xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle
int16_t count = 0;
int16_t prevCount = 0; 

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
  int unit;  // the PCNT unit that originated an interrupt
  uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

void IRAM_ATTR timer_group0_isr(void *para){
  int timer_idx = (int) para;

  /* Retrieve the interrupt status and the counter value
    from the timer that reported the interrupt */
  timer_intr_t timer_intr = timer_group_intr_get_in_isr(TIMER_GROUP_0);
  uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

  /* Prepare basic event data
    that will be then sent back to the main program task */
  timer_event_t evt;
  evt.timer_group = 0;
  evt.timer_idx = timer_idx;
  evt.timer_counter_value = timer_counter_value;

  /* Clear the interrupt
    and update the alarm time for the timer with without reload */
  if (timer_intr & TIMER_INTR_T0) {
    evt.type = TEST_WITHOUT_RELOAD;
    timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, timer_idx, timer_counter_value);
  } else if (timer_intr & TIMER_INTR_T1) {
    evt.type = TEST_WITH_RELOAD;
    timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_1);
  } else {
    evt.type = -1; // not supported even type
  }

  /* After the alarm has been triggered
    we need enable it again, so it is triggered the next time */
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

  /* Now just send the event data back to the main program task */
  xQueueSendFromISR(timer_queue, &evt, NULL);
}

static void example_tg0_timer_init(int timer_idx, bool auto_reload, double timer_interval_sec){
  /* Select and initialize basic parameters of the timer */
  timer_config_t config;
  config.divider = TIMER_DIVIDER;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_PAUSE;
  config.alarm_en = TIMER_ALARM_EN;
  config.intr_type = TIMER_INTR_LEVEL;
  config.auto_reload = auto_reload;
  #ifdef CONFIG_IDF_TARGET_ESP32S2BETA
    config.clk_sel = TIMER_SRC_CLK_APB;
  #endif
  timer_init(TIMER_GROUP_0, timer_idx, &config);

  /* Timer's counter will initially start from value below.
    Also, if auto_reload is set, this value will be automatically reload on alarm */
  timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

  /* Configure the alarm value and the interrupt on alarm. */
  timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
  timer_enable_intr(TIMER_GROUP_0, timer_idx);
  timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

  timer_start(TIMER_GROUP_0, timer_idx);
}

static void IRAM_ATTR pcnt_example_intr_handler(void *arg) {
  uint32_t intr_status = PCNT.int_st.val;
  int i;
  pcnt_evt_t evt;
  portBASE_TYPE HPTaskAwoken = pdFALSE;

  for (i = 0; i < PCNT_UNIT_MAX; i++) {
    if (intr_status & (BIT(i))) {
      evt.unit = i;
      /* Save the PCNT event type that caused an interrupt
        to pass it to the main program */
      evt.status = PCNT.status_unit[i].val;
      PCNT.int_clr.val = BIT(i);
      xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
      if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
      }
    }
  }
}

static void pcnt_example_init(void){
  /* Prepare configuration for the PCNT unit */
  pcnt_config_t pcnt_config = {
    // Set PCNT input signal and control GPIOs
    .pulse_gpio_num = PCNT_INPUT_SIG_IO,
    .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
    .channel = PCNT_CHANNEL_0,
    .unit = PCNT_TEST_UNIT,
    // What to do on the positive / negative edge of pulse input?
    .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
    .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
    // What to do when control input is low or high?
    .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
    .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
    // Set the maximum and minimum limit values to watch
    .counter_h_lim = PCNT_H_LIM_VAL,
    .counter_l_lim = PCNT_L_LIM_VAL,
  };
  /* Initialize PCNT unit */
  pcnt_unit_config(&pcnt_config);

  /* Configure and enable the input filter */
  pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
  pcnt_filter_enable(PCNT_TEST_UNIT);

  /* Set threshold 0 and 1 values and enable events to watch */
  pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
  pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
  pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
  pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
  
  /* Enable events on zero, maximum and minimum limit values */
  pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
  pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

  /* Initialize PCNT's counter */
  pcnt_counter_pause(PCNT_TEST_UNIT);
  pcnt_counter_clear(PCNT_TEST_UNIT);

  /* Register ISR handler and enable interrupts for PCNT unit */
  pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
  pcnt_intr_enable(PCNT_TEST_UNIT);

  /* Everything is set up, now go to counting */
  pcnt_counter_resume(PCNT_TEST_UNIT);
}

static void timer_example_evt_task(void *arg){
  char  speedstring[10]; //variable for printing to alphanumeric
  while (1) {
    timer_event_t evt;
    xQueueReceive(timer_queue, &evt, portMAX_DELAY);
    pcnt_get_counter_value(PCNT_TEST_UNIT, &count);

    //calculate speed from pulse counter
    double pulseCount = count - prevCount;
    prevCount = count; 
    double rotations = (pulseCount/6) * 60/TIMER_INTERVAL1_SEC;  
    float mps = rotations * 0.62 / 60;

    printf("rpm: %f, mps: %f\n", rotations, mps);
    speed = mps; //setting global variable 

    sprintf(speedstring, "%f", speed); //print speed to alphanumeric
    alpha_display(speedstring);
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////

// Servo control ///////////////////////////////////////////////////////////////////////////////
void mcpwm_initialize() {
  printf("initializing mcpwm servo control gpio......\n");
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 4);    //Set GPIO 4 as PWM0A (A5) , to which ESC is connected
	mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, 21);    //Set GPIO 21 as PWM1A (21), to which steering servo is connected

	//2. initial mcpwm configuration
	printf("Configuring Initial Parameters of mcpwm......\n");
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
	pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
	pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
}

void motorDrive(int motor_drive) { //function to control motor speed
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, motor_drive);
}

void motorTurn(int motor_turn){ //function to change steering
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, motor_turn);
}
////////////////////////////////////////////////////////////////////////////////////////////////

// LIDAR ///////////////////////////////////////////////////////////////////////////////////////
// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Read register
uint8_t readRegister(uint8_t reg) {
  uint8_t data;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  //printf("data: %x   \n", data);
  return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
  uint8_t data1;
  uint8_t data2;
  int16_t data3;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data1, ACK_VAL);
  i2c_master_read_byte(cmd, &data2, ACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  int16_t data2_16 = data1 << 8;
  data3 = data2_16 | data2;
  return data3;
}

int lidarGetRange() { //function to get distance from LIDAR
   int val = 0;

   writeRegister(RegisterMeasure, MeasureValue);
   vTaskDelay(200 / portTICK_RATE_MS);
   val = read16(RegisterHighLowB);

   printf("Distance: %d\n", val);
   return val;
}
/////////////////////////////////////////////////////////////////////////////////////////////////

//Speed control
void pid_speed() {
  error = setpoint - speed;
  dt = 0.1; // important!!!
  integral = integral + error * dt;
  derivative = (error - prev_error) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  prev_error = error;
  
  printf("PID speed output: %f\n", output);
  if (output == 0) { //stay the same
    motorDrive(motor_speed);
  } else if (output < .35) { //slowing down
    if (motor_speed >= 1300){ //don't let drive slower than 1300. It won't move past that
      motor_speed = 1300;
    } else {
      motor_speed += 20; // decrement by 20 if too fast
    }
    motorDrive(motor_speed); //tell motor to drive
  } else if (output > .35) { //speeding up
    if (motor_speed <= 1200){ //don't let drive faster than 1200
      motor_speed = 1200;
    } else {
      motor_speed = motor_speed - 20; //increment by 20 if too slow
    }
    motorDrive(motor_speed);
  } else {
    vTaskDelay(1/portTICK_PERIOD_MS); //do nothing
  }
}

//Steering
void pid_turn(){
  int val1;
  int val2;
  int distDiff;

  val1 = sensors1();
  val2 = sensors2();

  distDiff = val1 - val2;
  if(distDiff == 0){
    motorTurn(1400); //drive straight
  } else if(distDiff < 8){ //steering error 8. give 8cm of padding for steering
    motorTurn(1200); //turn right
  } else if(distDiff > 8){
    motorTurn(1700); //turn left
  }
}

// WiFi /////////////////////////////////////////////////////////////////////////////////
static void udp_client_task(void *pvParameters) { //task wating for message from web server
  char rx_buffer[128];
  char addr_str[128];
  int addr_family;
  int ip_protocol;
  
  while (1) {
    #ifdef CONFIG_EXAMPLE_IPV4
            struct sockaddr_in dest_addr;
            dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(PORT);
            addr_family = AF_INET;
            ip_protocol = IPPROTO_IP;
            inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
    #else // IPV6
            struct sockaddr_in6 dest_addr;
            inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            addr_family = AF_INET6;
            ip_protocol = IPPROTO_IPV6;
            inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
    #endif

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }
    
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
      
    while (1) {
      int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
      if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        break;
      }

      ESP_LOGI(TAG, "Message sent");
      struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
      socklen_t socklen = sizeof(source_addr);
      int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
      
      // Error occurred during receiving
      if (len < 0) {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        break;
      }
      // Data received
      else {
        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
        ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
        ESP_LOGI(TAG, "%s", rx_buffer);

        if (strcmp(rx_buffer, "1") == 0) {
          ESP_LOGI(TAG, "Start");
          start_flag = 1; //set global variable 
        } else if (strcmp(rx_buffer, "-1") == 0) {
          ESP_LOGI(TAG, "Stop");
          start_flag = 0; //set global variable
        }
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    if (sock != -1) {
      ESP_LOGE(TAG, "Shutting down socket and restarting...");
      shutdown(sock, 0);
      close(sock);
    }
  }
  vTaskDelete(NULL);
}

static void move_task(void *pvParameters) {
  int distCollision;
  motor_speed = 1280; //set initial speed at startup
  while(1){
    distCollision  = lidarGetRange();
    printf("flag: %d\n", start_flag);
    if (start_flag == 1 && distCollision >= LIDAR_DIST){
      if (speed == 0.0){ //if starting from stop
        motorDrive(motor_speed); //start driving
        motorTurn(1400); //straighten out steering
      }
      if (dt_complete == 1){
        pid_speed(); //adjust speed
        pid_turn(); //adjust steering
        dt_complete = 0;
        TIMERG1.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
      }
    } else if (start_flag == 0 || distCollision <= LIDAR_DIST){
      motorDrive(1400); //if stop button pressed or collision stop driving
    } else {
      vTaskDelay(1); //do nothing
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void app_main() {
  periodic_timer_init();

  //i2c init
	i2c_master_init();
	i2c_scanner();
  
  //alphanumeric init
  int ret;
  printf(">> Test Alphanumeric Display: \n");
  
  // Set up routines
  // Turn on alpha oscillator
  ret = alpha_oscillator();
  if(ret == ESP_OK) {printf("- oscillator: ok \n");}
  
  // Set display blink off
  ret = no_blink();
  if(ret == ESP_OK) {printf("- blink: off \n");}
  ret = set_brightness_max(0xF);
  if(ret == ESP_OK) {printf("- brightness: max \n");}

  mcpwm_initialize();
  uart_initialize();

  //UDP init
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  /* Initialize PCNT event queue and PCNT functions */
  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
  pcnt_example_init();
  timer_queue = xQueueCreate(10, sizeof(timer_event_t));
  example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD, TIMER_INTERVAL1_SEC);
  xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL); //task to get speed

  xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL); //task to receive message from web server
  
  motorTurn(1400); //turn steering straight at start

  xTaskCreate(move_task, "move", 4096, NULL, 5, NULL); //task to drive
}