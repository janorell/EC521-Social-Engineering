
#include <stdio.h>
#include <string.h>
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "soc/uart_struct.h"

// #define RX (16)
// #define TX (17)
// #define RX2 (25)
// #define TX2 (26)
//
// static const int BUF_SIZE = 256;
// static const int BUF_SIZE2 = 256;

// #pragma once

// static void uart_initialize()
//
int sensors1();

int sensors2();
void uart_initialize();

// int sensors1();
//
// int sensors2();
