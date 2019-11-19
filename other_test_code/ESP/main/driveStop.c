#include <stdio.h>
#include <math.h>

//#include "./ADXL343.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/i2c.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "driveStop.h"

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

// ADXL343
#define SLAVE_ADDR                         0x62 // 0x62
#define RegisterMeasure                    0x00 // Register to write to initiate ranging.
#define MeasureValue                       0x04 // Value to initiate ranging.
#define RegisterHighLowB                   0x8f // Register to get both High and Low bytes in 1 call.



// Servo control /////////////////////////////////////////////////////////////////////////////////
void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 4);    //Set GPIO 4 as PWM0A (A5) , to which ESC is connected
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 21);    //Set GPIO 21 as PWM0A (21), to which steering servo is connected
}

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

void calibrateESC() {
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 4);    //Set GPIO 4 as PWM0A (A5) , to which ESC is connected
	//mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 21);    //Set GPIO 21 as PWM0A (21), to which steering servo is connected

	//2. initial mcpwm configuration
	printf("Configuring Initial Parameters of mcpwm......\n");
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
	pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
	pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

	vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);  // LOW signal in microseconds
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700); // NEUTRAL signal in microseconds
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 21);    //Set GPIO 21 as PWM0A (21), to which steering servo is connected
  vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);  // LOW signal in microseconds
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700); // NEUTRAL signal in microseconds
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
}

void motorForward() {
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1200);
}

void motorBackward(){
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500);
}

void motorStop() {
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);
}

void turnLeft(){
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100);
}

void turnRight(){
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);
}

void turnStraight(){
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);
}
/////////////////////////////////////////////////////////////////////////////////////////////

// LIDAR ///////////////////////////////////////////////////////////////////////////////////////////
// Function to initiate i2c -- note the MSB declaration!
void i2c_master_init(){
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
void i2c_scanner() {
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
  //i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
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

void drive_app_main()
{
	printf("Testing servo motor.......\n");
	//1. mcpwm gpio initialization
	//mcpwm_example_gpio_initialize();

	i2c_master_init();
	i2c_scanner();

//xTaskCreate(calibrateESC, "calibrateESC", 4096, NULL, 5, NULL);
  calibrateESC();
  printf("Calibrating\n");

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 4);
  // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1200);
  // vTaskDelay(2000 / portTICK_PERIOD_MS);
  // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 21);
  // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400);

	int distCollision;

	while (1){
		distCollision = lidarGetRange();
		if (distCollision <= 50){
     printf("Stop\n");
			motorStop();
		} else {
			printf("Forward\n");
			motorForward();
      //calibrateESC();
		}
	}
}
