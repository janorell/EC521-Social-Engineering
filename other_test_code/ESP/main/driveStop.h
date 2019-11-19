#include <stdio.h>
#include <math.h>

// #include "./ADXL343.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/i2c.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
//
// //You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
// #define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
// #define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
// #define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate
//
// // Master I2C
// #define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
// #define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
// #define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
// #define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
// #define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
// #define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
// #define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
// #define READ_BIT                           I2C_MASTER_READ  // i2c master read
// #define ACK_CHECK_EN                       true // i2c master will check ack
// #define ACK_CHECK_DIS                      false// i2c master will not check ack
// #define ACK_VAL                            0x00 // i2c ack value
// #define NACK_VAL                           0xFF // i2c nack value
//
// #define SLAVE_ADDR                         0x62 // 0x62
// #define RegisterMeasure                    0x00 // Register to write to initiate ranging.
// #define MeasureValue                       0x04 // Value to initiate ranging.
// #define RegisterHighLowB                   0x8f // Register to get both High and Low bytes in 1 call.

// #pragma once

// static void i2c_master_init()
//
// static void i2c_scanner()
void i2c_master_init();

void i2c_scanner();

void calibrateESC();

int lidarGetRange();

// static void mcpwm_example_gpio_initialize()
void mcpwm_example_gpio_initialize();

void motorForward();

void motorBackward();

void motorStop();

void turnLeft();

void turnRight();

void turnStraight();
