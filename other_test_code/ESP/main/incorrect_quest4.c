#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driveStop.h"
#include "pid_skill.h"
#include "sensors.h"
#include "wifi_udp_client.h"

#define fbaseline 40 // 40cm
#define sbaseline 40 // 40cm
#define Kp 0.75
#define Ki 0.5
#define Kd 0.0
/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
// #define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
// #define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD
#define EXAMPLE_WIFI_SSID "Group_7"
#define EXAMPLE_WIFI_PASS "smart444"
//#ifdef CONFIG_EXAMPLE_IPV4
//#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
//#else
//#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
//#endif
#define HOST_IP_ADDR "192.168.1.124"
#define PORT 8082
// #define PORT CONFIG_EXAMPLE_PORT

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_in sourceAddr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(sourceAddr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

				if (strcmp(rx_buffer, "start") == 0) {
					ESP_LOGI(TAG, "Start");
				}
				else if (strcmp(rx_buffer, "stop") == 0) {
					ESP_LOGI(TAG, "Stop");
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

double fintegral, fderivative, ferror, fprev_error, foutput, fdt;

void PID_front() {

while (1) {
    int fvalue = lidarGetRange();
    ferror = fbaseline - fvalue;
    fdt = 0.1; // important!!!
    fintegral = fintegral + ferror * fdt;
    fderivative = (ferror - fprev_error) / fdt;
    foutput = Kp * ferror + Ki * fintegral + Kd * fderivative;
    fprev_error = ferror;

    printf("Output: %f\n", foutput);
    if ((int)foutput == 0) {
      motorForward();
    } else if ((int)foutput < 0) {
      motorBackward();
    } else if ((int)foutput > 0) {
      motorStop();
    } else {
      vTaskDelay(100/portTICK_PERIOD_MS);
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }

}
double sintegral, sderivative, serror, sprev_error, soutput, sdt;

void PID_side() {

while (1) {
    int sens1 = sensor1();
    int sens2 = sensor2();
    int svalue = sens1 - sens2;
    serror = sbaseline - svalue;
      sdt = 0.1; // important!!!
      sintegral = sintegral + serror * sdt;
      sderivative = (serror - sprev_error) / sdt;
      soutput = Kp * serror + Ki * sintegral + Kd * sderivative;
      sprev_error = serror;


        printf("Output: %f\n", soutput);
    if ((int)soutput == 0) {
      turnStraight();
    } else if ((int)soutput < 0) {
      turnLeft();
    } else if ((int)soutput > 0) {
      turnRight();
    } else {
      vTaskDelay(100/portTICK_PERIOD_MS);
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }

}

static void call_PID() {

  fdt = 0.0;
  fprev_error = 0.0;		// Set up PID loop
  fintegral = 0.0;

  sdt = 0.0;
  sprev_error = 0.0;		// Set up PID loop
  sintegral = 0.0;

  periodic_timer_init();

  while(1){
  	if (dt_complete == 1) {
         PID_side();
         PID_front();
         dt_complete = 0;
         // Re-enable alarm
         TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
  	}
  }
}

void app_main() {
  wifi_udp_client();
  uart_initialize();
  mcpwm_example_gpio_initialize();
  i2c_master_init();
  i2c_scanner();
  calibrateESC();

  while(1) {
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(call_PID, "call_PID", 4096, NULL, 5, NULL);
  }
}
