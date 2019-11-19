#include <stdio.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "esp_types.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "driver/i2c.h"
#include "./ADXL343.h"

#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"

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

// Speed timer
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

#define RX (16)
#define TX (17)
#define RX2 (25)
#define TX2 (26)

static const int BUF_SIZE = 256;
static const int BUF_SIZE2 = 256;

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

static int sensors1() {
  uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
  uint32_t distance = 0;

  int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 100);
  uart_flush(UART_NUM_1);
  if (data[0] == 0x59 && data[1] == 0x59) {
    distance = (data[3]<<8)+data[2];
  }
  printf("Distance Sensor 1: %dcm\n", distance);
  return distance;
}

static int sensors2() {
  uint8_t *data2 = (uint8_t *) malloc(BUF_SIZE2);
  uint32_t distance2 = 0;

  int len2 = uart_read_bytes(UART_NUM_2, data2, BUF_SIZE2, 100);
  uart_flush(UART_NUM_2);
  if (data2[0] == 0x59 && data2[1] == 0x59) {
    distance2 = (data2[3]<<8)+data2[2];
  }
  printf("Distance Sensor 2: %dcm\n", distance2-20);
  return distance2;
}
//////////////////////////////////////////////////////////////////////////////////////////

// Speed /////////////////////////////////////////////////////////////////////////////////
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
///////////////////////////////////////////////////////////////////////////////////////////

// Servo control /////////////////////////////////////////////////////////////////////////////////
// static void mcpwm_example_gpio_initialize()
// {
//     printf("initializing mcpwm servo control gpio......\n");
//     mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 4);    //Set GPIO 4 as PWM0A (A5) , to which ESC is connected
// 	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 21);    //Set GPIO 21 as PWM0A (21), to which steering servo is connected
// }

// /**
//  * @brief Use this function to calcute pulse width for per degree rotation
//  *
//  * @param  degree_of_rotation the angle in degree to which servo has to rotate
//  *
//  * @return
//  *     - calculated pulse width
//  */

// static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
// {
//     uint32_t cal_pulsewidth = 0;
//     cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
//     return cal_pulsewidth;
// }

// void mcpwm_example_servo_control(void *arg)
// {
//     uint32_t angle, count;
//     //1. mcpwm gpio initialization
//     mcpwm_example_gpio_initialize();

//     //2. initial mcpwm configuration
//     printf("Configuring Initial Parameters of mcpwm......\n");
//     mcpwm_config_t pwm_config;
//     pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
//     pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
//     pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
//     pwm_config.counter_mode = MCPWM_UP_COUNTER;
//     pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
//     mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
// }

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

void calibrateESC() {
  vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);  // LOW signal in microseconds
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700); // NEUTRAL signal in microseconds
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value

  vTaskDelay(1000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
	mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 2100); // HIGH signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1400);  // LOW signal in microseconds
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 700); // NEUTRAL signal in microseconds
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
}

void motorForward() {
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1200); 
}

void motorBackward(){
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1600); 
}

void motorStop() {
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); 
}

void turnLeft(){
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 2100);
}

void turnRight(){
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 700);
}

void turnStraight(){
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1400);
}
/////////////////////////////////////////////////////////////////////////////////////////////

// LIDAR ///////////////////////////////////////////////////////////////////////////////////////////
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
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
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

int lidarGetRange() {
   int val = 0;

   writeRegister(RegisterMeasure, MeasureValue);
   vTaskDelay(200 / portTICK_RATE_MS);
   val = read16(RegisterHighLowB);

   printf("Distance: %d\n", val);
   return val;
}
/////////////////////////////////////////////////////////////////////////////////////////////

void app_main() {
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

	printf("Testing servo motor.......\n");
	//1. mcpwm gpio initialization
	//mcpwm_example_gpio_initialize();
	
	i2c_master_init();
	i2c_scanner();

  mcpwm_initialize();
  calibrateESC();
  printf("Calibrating\n");

  uart_initialize();

	int distCollision;
  int prevState = LOW;
  int risingEdgeCount = 0;
  double counter_time_sec = 0;
  double speed = 0;
  double distance = 10.3;  // in centimeters

	while (1){
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
        //esp_err_t ret = timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &counter_time_sec);
        timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &counter_time_sec);
        speed = distance / counter_time_sec;
        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
        timer_start(TIMER_GROUP_0, TIMER_0);
        risingEdgeCount = 0;
        counter_time_sec = 0;
        printf("Speed: %fcm/s\n", speed);
      }
    } else if (state == LOW && prevState == HIGH) {// from HIGH to LOW
      risingEdgeCount = 1;
    }
    prevState = state;
    vTaskDelay(1);

    sensors1();
    sensors2();

		distCollision = lidarGetRange();
		if (distCollision <= 50){
     printf("Stop\n");
			//motorStop();
      motorForward();
      //motorBackward();
		} else {
			printf("Forward\n");
			//motorForward();
      motorStop();
		}
	}
}