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

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "esp_central.h"


/* Function Declarations: */
void ble_store_config_init(void);
static void start_scan(void);
static int gap_event_handler(struct ble_gap_event *event, void *arg);
static void adc_task(void *arg);
static int ble_gattc_subscribe(const struct peer* peer);

/* Variables: */
const static char *TAG = "BLE";
static int adc_raw[2];
static adc_oneshot_unit_handle_t handle1;
static uint16_t uart_rx_handle;
static uint16_t curr_handle;
static adc_oneshot_unit_init_cfg_t init_cfg = {
    .unit_id = ADC_UNIT_1
};
static adc_oneshot_chan_cfg_t cfg = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT
};
/* static gpio_config_t gp0 = {
    .pin_bit_mask = (1ULL<<32),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
}; */

static const ble_uuid128_t uart_svc_uuid =
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,
        0xB5,0x01,0x00,0x40,0x6E);

static const ble_uuid128_t uart_rx_chr_uuid =
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,
        0xB5,0x02,0x00,0x40,0x6E);

static const ble_uuid128_t uart_tx_chr_uuid =
    BLE_UUID128_INIT(0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,
        0xB5,0x03,0x00,0x40,0x6E);


/* NEED A GATT EVENT HANDLER TO RECEIVE NOTIFICATIONS (maybe) */
static int ble_gattc_subscribe(const struct peer* peer){
    const struct peer_dsc *dsc;
    int rc;
    uint8_t value[2];

    dsc = peer_dsc_find_uuid(peer,
                             &uart_svc_uuid.u,
                             &uart_tx_chr_uuid.u,
                             BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16));
    if(dsc == NULL){
        ESP_LOGE(TAG, "Peer lacks a CCCD for the subscribable characteristic\n");
        return 1;
    }

    /* Write 0x00 and 0x01 (The subscription code) to the CCCD */
    value[0] = 1;
    value[1] = 0;
    rc = ble_gattc_write_flat(peer->conn_handle, dsc->dsc.handle,
                              value, sizeof(value), NULL, NULL);
    if(rc != 0){
        ESP_LOGE(TAG,
                    "Failed to subscribe to the subscribable characteristic; "
                    "rc=%d\n", rc);
        return 2;
    }

    return 0;
}


/* NimBLE stack is configured running in rtos task, then this function is invoked */
void ble_host_task(void *param){
    ESP_LOGI(TAG, "inside host task");
    nimble_port_run();

    /* this instruction is reached when nimble_port_stop() is called */
    nimble_port_freertos_deinit();
}

/* Perform GATT operations on the peripheral (peer) device,  */
/*      --> subscribe to notifications                       */
/*      --> write to Rx characteristic                       */
static void characteristic_disc(const struct peer *peer){
    const struct peer_chr *rx_chr;
    const struct peer_chr *tx_chr;
    int rc;
    
    rx_chr = peer_chr_find_uuid(peer, &uart_svc_uuid.u, &uart_rx_chr_uuid.u);

    if(rx_chr == NULL){
        ESP_LOGE(TAG, "NUS RX characteristic not found");
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }

    /* use in RTOS task to write data to pcb */
    uart_rx_handle = rx_chr->chr.val_handle;
    curr_handle = peer->conn_handle;

    tx_chr = peer_chr_find_uuid(peer, &uart_svc_uuid.u, &uart_tx_chr_uuid.u);

    if(tx_chr == NULL){
        ESP_LOGE(TAG, "NUS TX characteristic not found");
        ble_gap_terminate(curr_handle, BLE_ERR_REM_USER_CONN_TERM);
    }

    rc = ble_gattc_subscribe(peer);

    if(rc != 0){
        ESP_LOGE(TAG, "Failed to subscribe, rc=%d", rc);
        ble_gap_terminate(curr_handle, BLE_ERR_REM_USER_CONN_TERM);
    }

    /* START ADC_TASK HERE */
    ESP_LOGI(TAG, "Subscribed to NUS TX, writing analog data now...");
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);

    return;
}

/* Filtering process, don't connect to discovered devices if they don't */
/* PUBLICLY ADVERTISE the Nordic UART Service                           */
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

    for(i=0; i<fields.num_uuids128; i++){
        if(ble_uuid_cmp(&fields.uuids128[i].u, &uart_svc_uuid.u) == 0){
            return 1;
        }
    }

    return 0;
}

/* attempts to connect to a device if it publicly advertises the Nordic UART Service */
/* although if it does not, we return, doing nothing                                 */
static void connect_uart_device(void *disc){
    uint8_t our_addr;
    int rc;
    ble_addr_t *addr;

    if(!should_connect((struct ble_gap_disc_desc *) disc))
        return;
    
    rc = ble_gap_disc_cancel();
    if(rc != 0){
        ESP_LOGD(TAG, "Failed to cancel scan, rc=%d\n", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &our_addr);
    if(rc != 0){
        ESP_LOGE(TAG, "Error determining address, rc=%d\n", rc);
        return;
    }

    addr = &((struct ble_gap_disc_desc *)disc)->addr;

    rc = ble_gap_connect(our_addr, addr, 30000, NULL, gap_event_handler, NULL);
    if(rc != 0){
        ESP_LOGE(TAG, "Failed to connect to device, addr_type=%d addr=%s\n",
                    addr->type, addr_str(addr->val));
        return;
    }

    ESP_LOGI(TAG, "Connection established succesfully :)");
}

/* Connection to a peer device has been established, this function attempts to perform */
/* GATT procedures on the peripheral device                                            */
static void on_disc_complete(const struct peer *peer, int status, void *arg){
    if(status != 0){
        ESP_LOGE(TAG, "Error: Service discovery failed, status=%d conn_handle=%d\n",
                    status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    ESP_LOGI(TAG, "Service discovery completed, status=%d conn_handle=%d\n",
                status, peer->conn_handle);
    
    characteristic_disc(peer);
}

/* Once BLE stack is initialized, configured, and running ble_host_task() in an RTOS task */
/*  --> ble_host_task() calls nimble_port_run()                                           */
/*  --> when BLE stack has 'synced' this function is called                               */
/*  --> scanning for nearby BLE devices begins                                            */
static void sync_callback(void){
    int rc;
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    start_scan();
}

static void reset_callback(int reason){
    ESP_LOGE(TAG, "Resetting..., reason: %d\n", reason);
}

/* Now that BLE stack has synced, we start scanning and transfer control to the */
/* GAP Event Handler to handle device Discovery and Connection                  */
static void start_scan(void){
    int rc;
    uint8_t our_addr;
    struct ble_gap_disc_params disc_params = {0};
    
    rc = ble_hs_id_infer_auto(0, &our_addr);
    if(rc != 0){
        ESP_LOGE(TAG, "error determining address type, rc = %d\n", rc);
        return;
    }

    disc_params.filter_duplicates = 1;
    disc_params.passive = 0;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(our_addr, BLE_HS_FOREVER, &disc_params, gap_event_handler, NULL);

    if(rc != 0){
        ESP_LOGE(TAG, "Error initiating GAP discovery procedure, rc=%d\n", rc);
    }
}

/* GAP Event Handler allows for this BLE Central device to discover and */ 
/* connect to Peripheral BLE devices                                    */
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
            connect_uart_device(&event->disc);
            return 0;
        
        case BLE_GAP_EVENT_CONNECT:
            if(event->connect.status == 0){
                ESP_LOGI(TAG, "Connection established ");

                rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
                assert(rc==0);
                print_conn_desc(&desc);
                ESP_LOGI(TAG, "\n");

                rc = peer_add(event->connect.conn_handle);
                if(rc != 0){
                    ESP_LOGE(TAG, "Failed to add peer, rc=%d\n", rc);
                    return 0;
                }

                rc = peer_disc_all(event->connect.conn_handle, on_disc_complete, NULL);
                if(rc != 0){
                    ESP_LOGE(TAG, "Failed to discover services; rc=%d\n", rc);
                    return 0;
                }
            }
            else{
                ESP_LOGE(TAG, "Error: Connection failed, status=%d\n", event->connect.status);
                start_scan();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnected, reason=%d ", event->disconnect.reason);
            print_conn_desc(&event->disconnect.conn);
            ESP_LOGI(TAG, "\n");

            peer_delete(event->disconnect.conn.conn_handle);

            start_scan();
            return 0;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ESP_LOGI(TAG, "discovery complete; reason=%d\n",
                        event->disc_complete.reason);
            return 0;
        
        case BLE_GAP_EVENT_NOTIFY_RX:
            int len = OS_MBUF_PKTLEN(event->notify_rx.om);
            uint8_t data[128];

            os_mbuf_copydata(event->notify_rx.om, 0, len, data);

            ESP_LOGI(TAG, "Received from CIRCUITPY: %.*s", len, data);
            return 0;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI("BLE", "MTU updated to %d", event->mtu.value);
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

/* Read ADC data (12-bits) and send it to BLE peripheral as two 8-bit unsigned integers */
static void adc_task(void *arg){
    uint8_t data[2];
    int rc;
    while(1){
        adc_oneshot_read(handle1, ADC_CHANNEL_0, &adc_raw[0]);
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1, ADC_CHANNEL_0, adc_raw[0]);

        /* store LSB first, MSB last (little endian) */
        data[0] = adc_raw[0] & 0xFF;
        data[1] = (adc_raw[0] >> 8) & 0xFF;

        struct os_mbuf *om = ble_hs_mbuf_from_flat(data, 2);

        rc = ble_gattc_write_no_rsp(curr_handle,
                                uart_rx_handle,
                                om);

        if(rc != 0){
            ESP_LOGE(TAG, "Write failed, rc=%d", rc);
            ble_gap_terminate(curr_handle, BLE_ERR_REM_USER_CONN_TERM);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
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

    /* initialize ADC1_CH0 */
    adc_init(&handle1, init_cfg, cfg);

    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to initialize NimBLE stack: %d ", ret);
        return;
    }

    /* configure host by defining Callback functions */
    ble_hs_cfg.reset_cb = reset_callback;
    ble_hs_cfg.sync_cb = sync_callback;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* manage peer services/characteristics objects in ble */
    int rc;
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    /* set this device's name */
    int m;
    m = ble_svc_gap_device_name_set("cyrus");
    assert(m == 0);

    /* continue to configure the host */
    ble_store_config_init();

    /* create RTOS task where NimBLE will run */
    nimble_port_freertos_init(ble_host_task);
}