#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"


/* First use ADC to read analog data from breakout board, then:                  */
/*      --> advertise as Nordic UART Service, and wait to connect to EnvisionPCB */
/*      --> format analog data to send                                           */
/*      --> when BLE is connected, write data to the EnvisionPCB                 */


static gpio_config_t gp0 = {
    .pin_bit_mask = (1ULL<<32),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

void app_main(void)
{
    gpio_config(&gp0);
    printf("returned from gpio_config()...\n");

    gpio_set_level(GPIO_NUM_32, 1);
    printf("did LED turn on?\n");
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NUM_32, 0);
    for(int i = 10; i >= 0; i--){
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}