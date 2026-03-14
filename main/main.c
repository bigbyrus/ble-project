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

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* First use ADC to read analog data from breakout board, then:                  */
/*      --> advertise as Nordic UART Service, and wait to connect to EnvisionPCB */
/*      --> format analog data to send                                           */
/*      --> when BLE is connected, write data to the EnvisionPCB                 */

/* Variables: */
const static char *TAG = "ADC";
static int adc_raw[2];
static adc_oneshot_unit_handle_t handle1;
static adc_oneshot_unit_init_cfg_t init_cfg = {
    .unit_id = ADC_UNIT_1
};
static adc_oneshot_chan_cfg_t cfg = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT
};
static gpio_config_t gp0 = {
    .pin_bit_mask = (1ULL<<32),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};
static gpio_config_t gp1 = {
    .pin_bit_mask = (1ULL<<23),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

/* Configure Channels 0 and 3 to be read from ADC Unit 1*/
static void adc_init(adc_oneshot_unit_handle_t *handle, adc_oneshot_unit_init_cfg_t init_cfg, adc_oneshot_chan_cfg_t cfg){
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*handle, ADC_CHANNEL_0, &cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*handle, ADC_CHANNEL_3, &cfg));
}

/* Read raw adc data */
static void adc_task(void *pvParameters){
    while(1){
        ESP_ERROR_CHECK(adc_oneshot_read(handle1, ADC_CHANNEL_0, &adc_raw[0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1, ADC_CHANNEL_0, adc_raw[0]);

        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_ERROR_CHECK(adc_oneshot_read(handle1, ADC_CHANNEL_3, &adc_raw[1]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1, ADC_CHANNEL_3, adc_raw[1]);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* Blink LED on board */
static void led_task(void *pvParameters){
    while(1){
        gpio_set_level(GPIO_NUM_23, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_23, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    /* turn on switch */
    gpio_config(&gp0);
    gpio_config(&gp1);
    gpio_set_level(GPIO_NUM_32, 1);

    /* initialize ADC1_CH0 and ADC1_CH3 */
    adc_init(&handle1, init_cfg, cfg);

    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 4096, NULL, 5, NULL);
}