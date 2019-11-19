#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"
//
#pragma once

// static void periodic_timer_init()
void periodic_timer_init();
