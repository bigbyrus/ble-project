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

/* NimBLE APIs */
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_gap.h"

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

static uint16_t uart_rx_handle;
static uint16_t uart_tx_handle;
static uint16_t conn_id;
static esp_gatt_if_t gattc_if_global;

static const ble_uuid128_t uart_svc_uuid =
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,
        0xB5,0x01,0x00,0x40,0x6E);

static const ble_uuid128_t uart_rx_chr_uuid =
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,
        0xB5,0x02,0x00,0x40,0x6E);

static const ble_uuid128_t uart_tx_chr_uuid =
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,
        0xB5,0x03,0x00,0x40,0x6E);



void ble_host_task(void *param){
    ESP_LOGI(tag, "inside host task");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static int should_connect(const struct ble_gap_disc_desc *disc){
    struct ble_hs_adv_fields fields;
    int rc, i;

    if(disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND && 
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND){
        return 0;
    }

    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if(rc != 0){
        return 0;
    }

    for(i=0; i<fields.num_uuids16; i++){
        if(ble_uuid_u16(&fields.uuids16[i].u) == BLECENT_SVC_ALERT_UUID){
            return 1;
        }
    }

    return 0;
}

static void connect_device(void *disc){
    uint8_t our_addr;
    int rc;
    ble_addr_t *addr;

    if(!should_connect((struct ble_gap_disc_desc *) disc))
        return;
    
    rc = ble_gap_disc_cancel();
    if(rc != 0){
        MODLOG_DFLT(DEBUG, "Failed to cancel scan, rc=%d\n", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &our_addr);
    if(rc != 0){
        MODLOG_DFLT(ERROR, "Error determining address, rc=%d\n", rc);
        return;
    }

    addr = &((struct ble_gap_disc_desc *)disc)->addr;

    rc = ble_gap_connect(our_addr, addr, 30000, NULL, gap_event_handler, NULL);
    if(rc != 0){
        MODLOG_DFLT(ERROR, "Failed to connect to device, addr_type=%d addr=%s\n",
                    addr->type, addr_str(addr->val));
        return;
    }
}

static void on_disc_complete(const struct peer *peer, int status, void *arg){
    if(status != 0){
        MODLOG_DFLT(ERROR, "Error: Service discovery failed, status=%d conn_handle=%d\n",
                    status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    MODLOG_DFLT(INFO, "Service discovery completed, status=%d conn_handle=%d\n",
                status, peer->conn_handle);
    
    read_write_subscribe(peer);
}

static void sync_callback(void){
    int rc;
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    start_scan();
}

static void reset_callback(void *param){
    MODLOG_DFLT(ERROR, "Resetting..., reason=%d\n", reason);
}

static void start_scan(void){
    int rc;
    uint8_t our_addr;
    struct ble_gap_disc_params disc_params = {0};
    
    rc = ble_hs_id_infer_auto(0, &our_addr);
    if(rc != 0){
        MODLOG_DFLT(ERROR, "error determining address type, rc = %d\n", rc);
        return;
    }

    disc_params.filter_duplicates = 1;
    disc_params.passive = 1;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(our_addr, BLE_HS_FOREVER, &disc_params, gap_event_handler, NULL);

    if(rc != 0){
        MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure, rc=%d\n", rc);
    }
}

static int gap_event_handler(struct ble_gap_event *event, void *arg){
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int rc;

    switch(event->type){
        case BLE_GAP_EVENT_DISC:
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            if(rc != 0)
                return 0;
            print_adv_fields(&fields);
            connect_device(&event->disc);
        
        case BLE_GAP_EVENT_CONNECT:
            if(event->connect.status == 0){
                MODLOG_DFLT(INFO, "Connection established ");

                rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
                assert(rc==0);
                print_conn_desc(&desc);
                MODLOG_DFLT(INFO, "\n");

                rc = peer_add(event->connect.conn_handle);
                if(rc != 0){
                    MODLOG_DFLT(ERROR, "Failed to add peer, rc=%d\n", rc);
                    return 0;
                }

                rc = peer_disc_all(event->connect.conn_handle, on_disc_complete, NULL);
                if(rc != 0){
                    MODLOG_DFLT(ERROR, "Failed to discover services; rc=%d\n", rc);
                    return 0;
                }
            }
            else{
                MODLOG_DFLT(ERROR, "Error: Connection failed, status=%d\n", event->connect.status);
                start_scan();
            }
            return 0;

        case BLE_GAP_DISCONNECT:
            MODLOG_DFLT(INFO, "disconnected, reason=%d ", event->disconnect.reason);
            print_conn_desc(&event->disconnect.conn);
            MODLOG_DFLT(INFO, "\n");

            peer_delete(event->disconnect.conn.conn_handle);

            start_scan();
            return 0;

        case BLE_GAP_EVENT_NOTIFY_RX:
            MODLOG_DFLT(INFO, "received %s, conn_handle=%d attr_handle=%d attr_len=%d\n",
                        event->notify_rx.indication ? "indication" : "notification", 
                        event->notify_rx.conn_handle, event->notify_rx.attr_handle,
                        OS_MBUF_PKTLEN(event->notify_rx.om));
            
            return 0;
        
        default:
            return 0;
    }
}

/* Configure Channels 0 and 3 to be read from ADC Unit 1*/
static void adc_init(adc_oneshot_unit_handle_t *handle, adc_oneshot_unit_init_cfg_t init_cfg, adc_oneshot_chan_cfg_t cfg){
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*handle, ADC_CHANNEL_0, &cfg));
    //ESP_ERROR_CHECK(adc_oneshot_config_channel(*handle, ADC_CHANNEL_3, &cfg));
}

/* Read raw adc data */
static void adc_task(void *pvParameters){
    while(1){
        ESP_ERROR_CHECK(adc_oneshot_read(handle1, ADC_CHANNEL_0, &adc_raw[0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1, ADC_CHANNEL_0, adc_raw[0]);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    /* NVS flash initialization */
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* turn on switch */
    gpio_config(&gp0);
    gpio_set_level(GPIO_NUM_32, 1);

    /* initialize ADC1_CH0 and ADC1_CH3 */
    adc_init(&handle1, init_cfg, cfg);

    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);

    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if(ret != ESP_OK){
        ESP_LOGE(tag, "Failed to initialize NimBLE stack: %d ", ret);
        return;
    }

    ble_hs_cfg.reset_cb = reset_callback;
    ble_hs_cfg.sync_cb = sync_callback;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    int rc;
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    int m;
    m = ble_svc_gap_device_name_set("cyrus");
    assert(m == 0);

    ble_store_config_init();
    nimble_port_freertos_init(ble_host_task);
}