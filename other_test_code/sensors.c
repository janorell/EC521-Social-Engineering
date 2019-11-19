
#include <stdio.h>
#include <string.h>
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "soc/uart_struct.h"


#define RX (16)
#define TX (17)
#define RX2 (25)
#define TX2 (26)

static const int BUF_SIZE = 256;
static const int BUF_SIZE2 = 256;
//static int distarray[2];

static void uart_initialize()
{
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



//static void echo_task()
static int sensors1()
{

  //  while(1){

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint32_t distance = 0;


    int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 100);
    uart_flush(UART_NUM_1);

        if (data[0] == 0x59 && data[1] == 0x59)
        {
            distance = (data[3]<<8)+data[2];

    }
        printf("Distance Sensor 1: %dcm\n", distance);
   // }
    return distance;
}

static int sensors2()
{

  //  while(1){

    uint8_t *data2 = (uint8_t *) malloc(BUF_SIZE2);
    uint32_t distance2 = 0;

    int len2 = uart_read_bytes(UART_NUM_2, data2, BUF_SIZE2, 100);
    uart_flush(UART_NUM_2);

    if (data2[0] == 0x59 && data2[1] == 0x59)
        {
            distance2 = (data2[3]<<8)+data2[2];

        }
        printf("Distance Sensor 2: %dcm\n", distance2-20);
   // }
    return distance2;
}

void app_main()
{
    uart_initialize();

    while(1){
    sensors1();
    sensors2();
    }
   // xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
}
