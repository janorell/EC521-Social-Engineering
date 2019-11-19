/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <string.h>

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_station.h"
/**
 * This is an example which echos any data it receives on UART1 back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: UART1
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below
 */

#define ECHO_TEST_TXD  (GPIO_NUM_1)
#define ECHO_TEST_RXD  (GPIO_NUM_3)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)



void app_main() {
	wifi_app_main();

	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,
		256, 0, 0, NULL, 0));

	/* Tell VFS to use UART driver */
	esp_vfs_dev_uart_use_driver(UART_NUM_0);

	char inputStr[200];
	// int num = 0;

	// char switchS[2];
	// strcpy(switchS, "s");
	char *start_mode = "start";
	char *stop_mode = "stop";
	// char s_mode[2];
	// char t_mode[2];
	// strcpy(s_mode, "s");
	// strcpy(t_mode, "t");
	char clear;
			while (1) {
				scanf("%c", &clear); // &clear
				scanf("%[^\n]", inputStr);
				printf("Read: %s\n", inputStr);
				if (strcmp(inputStr, start_mode) == 0) {
					printf("START MODE\n");
				} else if (strcmp(inputStr, stop_mode) == 0) {
					printf("STOP MODE\n");
				}

			}


}
