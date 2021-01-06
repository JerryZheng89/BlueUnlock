#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "mbedtls/aes.h"

#define GATTC_TAG "GATTC_DEMO"

#define REMOTE_SERVICE_UUID         {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                                     0xDE, 0xEF, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00}
                                        //0x00FF
#define REMOTE_NOTIFY_CHAR_UUID     {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                                     0xDE, 0xEF, 0x12, 0x12, 0x24, 0x15, 0x00, 0x00}
                                        //0xFF01
#define REMOTE_LED_CHAR_UUID        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                                     0xDE, 0xEF, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00}
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0         // 0
#define INVALID_HANDLE   0

// led
#define GPIO_LED_0      2
#define GPIO_LED_PIN_SEL (1ULL<<GPIO_LED_0)

typedef struct {
    esp_gatt_if_t gattc_if;
    uint16_t  conn_id;
    uint16_t  char_handle;
}led_blink_args_t;

static const char remote_device_name[] = "Nordic_Blinky";
static char remote_device_mac[] = "c9:a7:ab:5e:2d:be";
static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;
/* led blink task */
// static TaskHandle_t led_task_handle = NULL;
// static led_blink_args_t xLEDTaskArgs;
/* aces key */
static const uint8_t key[17] = "Y4b711C7V0U4Mnl9";
// static uint8_t timeStamp[9]={0};
static void macStrTobuffer(char *mac, uint8_t *buffer);

/* openLock task */
static xQueueHandle ble_evt_queue = NULL;
static xQueueHandle timeOut_sig_queue = NULL;
xQueueHandle server_msg_queue = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
/* blink the led of ble client */
static void ble_openLock_task(void *arg);
extern int system_get_time_ms(void);
extern void esp_wait_sntp_sync(void);
extern void esp_initialize_sntp(void);


static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t remote_filter_led_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = REMOTE_LED_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {

    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ONLY_WLST,//BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x58,   // 100ms
    .scan_window            = 0x58,   // 100ms
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

typedef enum {
    BLE_GAP_EVENT,
    BLE_GATT_EVENT,
}ble_event_type_t;

typedef struct {
    ble_event_type_t event_type;
    esp_gap_ble_cb_event_t gap_event;
    esp_gattc_cb_event_t gattc_event;
    esp_gatt_if_t gattc_if;
    esp_ble_gattc_cb_param_t gattc_param;
    esp_ble_gap_cb_param_t gap_param;
}ble_event_data_t;

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    ble_event_data_t *p_data = (ble_event_data_t *)pvPortMalloc(sizeof(ble_event_data_t));
    p_data->event_type = BLE_GATT_EVENT;
    p_data->gattc_event = event;
    p_data->gattc_if = gattc_if;
    memcpy(&(p_data->gattc_param), param, sizeof(esp_ble_gattc_cb_param_t));

    if (xQueueSend(ble_evt_queue, (void *)&p_data, 20/portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(GATTC_TAG, "gattc event send failed!!!");
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ble_event_data_t *p_data = (ble_event_data_t *)pvPortMalloc(sizeof(ble_event_data_t));
    p_data->event_type = BLE_GAP_EVENT;
    p_data->gap_event = event;
    memcpy(&(p_data->gap_param), param, sizeof(esp_ble_gap_cb_param_t));

    if (xQueueSend(ble_evt_queue, (void *)&p_data, 20/portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(GATTC_TAG, "gap event send failed!!!");
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void sync_sntp_task(void *arg)
{
    esp_initialize_sntp();
    for (;;) {
        esp_wait_sntp_sync();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}


static void ble_openLock_task(void *arg)
{
    ble_event_data_t *eventMsg;
    BaseType_t result;
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    uint8_t cipher[17] = {0};
    uint8_t plain[17] = "5FD849CE00000101";
    mbedtls_aes_context ctx; 

    static QueueSetHandle_t xQueueSet = NULL;
    QueueHandle_t xQueueContainsData;
    
    xQueueSet = xQueueCreateSet(16);  //ble_evnt_queue(10)+server_msg_queue(5)+timeOut_sig_queue(1) = 16
    xQueueAddToSet(ble_evt_queue, xQueueSet);
    xQueueAddToSet(server_msg_queue, xQueueSet);
    xQueueAddToSet(timeOut_sig_queue, xQueueSet);

    ESP_LOGI(GATTC_TAG, "ble openlock task started!");

    vTaskDelay(2000/portTICK_PERIOD_MS);

    while (1) {
        xQueueContainsData = ( QueueHandle_t )xQueueSelectFromSet(xQueueSet, portMAX_DELAY);

        if (xQueueContainsData == ble_evt_queue) {
            result=xQueueReceive(ble_evt_queue, &eventMsg, 20/portTICK_PERIOD_MS);
            if (result == pdPASS)
            {
                if (eventMsg->event_type == BLE_GATT_EVENT) {

                    ESP_LOGI(GATTC_TAG, "esp gattc event received!");
                    switch (eventMsg->gattc_event) {
                        case ESP_GATTC_REG_EVT: {
                            ESP_LOGI(GATTC_TAG, "esp gattc REG_EVT");
                            // esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
                            // if (scan_ret){
                                // ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
                            // }
                            break;
                        }

                        case ESP_GATTC_CONNECT_EVT: {
                            ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", eventMsg->gattc_param.connect.conn_id, eventMsg->gattc_if);
                            gl_profile_tab[PROFILE_A_APP_ID].conn_id = eventMsg->gattc_param.connect.conn_id;
                            memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, eventMsg->gattc_param.connect.remote_bda, sizeof(esp_bd_addr_t));
                            ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
                            esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
                            esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (eventMsg->gattc_if, eventMsg->gattc_param.connect.conn_id);
                            if (mtu_ret){
                                ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
                            }
                            break;
                        }

                        case ESP_GATTC_OPEN_EVT: {
                            if (eventMsg->gattc_param.open.status != ESP_GATT_OK){
                                ESP_LOGE(GATTC_TAG, "open failed, status %d", eventMsg->gattc_param.open.status);
                                break;
                            }
                            ESP_LOGI(GATTC_TAG, "open success");
                            break;
                        }

                        case ESP_GATTC_DIS_SRVC_CMPL_EVT: {
                            if (eventMsg->gattc_param.dis_srvc_cmpl.status != ESP_GATT_OK){
                                ESP_LOGE(GATTC_TAG, "discover service failed, status %d", eventMsg->gattc_param.dis_srvc_cmpl.status);
                                break;
                            }
                            ESP_LOGI(GATTC_TAG, "discover service complete conn_id %d", eventMsg->gattc_param.dis_srvc_cmpl.conn_id);
                            esp_ble_gattc_search_service(eventMsg->gattc_if, eventMsg->gattc_param.cfg_mtu.conn_id, &remote_filter_service_uuid);
                            break;
                        }

                        case ESP_GATTC_CFG_MTU_EVT: {
                            if (eventMsg->gattc_param.cfg_mtu.status != ESP_GATT_OK){
                                ESP_LOGE(GATTC_TAG,"config mtu failed, error status = %x", eventMsg->gattc_param.cfg_mtu.status);
                            }
                            ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", eventMsg->gattc_param.cfg_mtu.status, eventMsg->gattc_param.cfg_mtu.mtu, eventMsg->gattc_param.cfg_mtu.conn_id);
                            break;
                        }

                        case ESP_GATTC_SEARCH_RES_EVT: {
                            ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", eventMsg->gattc_param.search_res.conn_id, eventMsg->gattc_param.search_res.is_primary);
                            ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", eventMsg->gattc_param.search_res.start_handle, eventMsg->gattc_param.search_res.end_handle, eventMsg->gattc_param.search_res.srvc_id.inst_id);
                            if (eventMsg->gattc_param.search_res.srvc_id.uuid.len == ESP_UUID_LEN_128 && eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[13] == remote_filter_char_uuid.uuid.uuid128[13]) {
                                ESP_LOGI(GATTC_TAG, "service found");
                                get_server = true;
                                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = eventMsg->gattc_param.search_res.start_handle;
                                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = eventMsg->gattc_param.search_res.end_handle;
                                ESP_LOGI(GATTC_TAG, "UUID128: %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", 
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[15],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[14],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[13],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[12],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[11],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[10],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[9],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[8],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[7],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[6],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[5],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[4],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[3],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[2],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[1],
                                                                        eventMsg->gattc_param.search_res.srvc_id.uuid.uuid.uuid128[0]);
                            }
                            break;
                        }

                        case ESP_GATTC_SEARCH_CMPL_EVT: {
                            if (eventMsg->gattc_param.search_cmpl.status != ESP_GATT_OK){
                                ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", eventMsg->gattc_param.search_cmpl.status);
                                break;
                            }
                            if(eventMsg->gattc_param.search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
                                ESP_LOGI(GATTC_TAG, "Get service information from remote device");
                            } else if (eventMsg->gattc_param.search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
                                ESP_LOGI(GATTC_TAG, "Get service information from flash");
                            } else {
                                ESP_LOGI(GATTC_TAG, "unknown service source");
                            }
                            ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
                            if (get_server){
                                uint16_t count = 0;
                                esp_gatt_status_t status = esp_ble_gattc_get_attr_count( eventMsg->gattc_if,
                                                                                         eventMsg->gattc_param.search_cmpl.conn_id,
                                                                                         ESP_GATT_DB_CHARACTERISTIC,
                                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                                         INVALID_HANDLE,
                                                                                         &count);
                                if (status != ESP_GATT_OK){
                                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                                }

                                if (count > 0){
                                    char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                                    if (!char_elem_result){
                                        ESP_LOGE(GATTC_TAG, "gattc no mem");
                                    }else{
                                        status = esp_ble_gattc_get_char_by_uuid( eventMsg->gattc_if,
                                                                                 eventMsg->gattc_param.search_cmpl.conn_id,
                                                                                 gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                                 gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                                 remote_filter_char_uuid,
                                                                                 char_elem_result,
                                                                                 &count);
                                        if (status != ESP_GATT_OK){
                                            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                                        }                        

                                        /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                                        if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                                            gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                                            esp_ble_gattc_register_for_notify (eventMsg->gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                                        }
                                    }
                                    /* free char_elem_result */
                                    free(char_elem_result);
                                }else{
                                    ESP_LOGE(GATTC_TAG, "no char found");
                                }
                            }
                             break;
                        }

                        case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
                            ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
                            if (eventMsg->gattc_param.reg_for_notify.status != ESP_GATT_OK){
                                ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", eventMsg->gattc_param.reg_for_notify.status);
                            }else{
                                uint16_t count = 0;
                                uint16_t notify_en = 1;
                                esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( eventMsg->gattc_if,
                                                                                             gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                                             ESP_GATT_DB_DESCRIPTOR,
                                                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                                             gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                                             &count);
                                if (ret_status != ESP_GATT_OK){
                                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                                }
                                if (count > 0){
                                    descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                                    if (!descr_elem_result){
                                        ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                                    }else{
                                        ret_status = esp_ble_gattc_get_descr_by_char_handle( eventMsg->gattc_if,
                                                                                             gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                                             eventMsg->gattc_param.reg_for_notify.handle,
                                                                                             notify_descr_uuid,
                                                                                             descr_elem_result,
                                                                                             &count);
                                        if (ret_status != ESP_GATT_OK){
                                            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                                        }
                                        /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                                        if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                                            ret_status = esp_ble_gattc_write_char_descr( eventMsg->gattc_if,
                                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                                         descr_elem_result[0].handle,
                                                                                         sizeof(notify_en),
                                                                                         (uint8_t *)&notify_en,
                                                                                         ESP_GATT_WRITE_TYPE_RSP,
                                                                                         ESP_GATT_AUTH_REQ_NONE);
                                        }

                                        if (ret_status != ESP_GATT_OK){
                                            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                                        }

                                        /* free descr_elem_result */
                                        free(descr_elem_result);
                                    }
                                }
                                else{
                                    ESP_LOGE(GATTC_TAG, "decsr not found");
                                }

                            }
                            break;
                        }
                        case ESP_GATTC_NOTIFY_EVT: {
                            if (eventMsg->gattc_param.notify.is_notify){
                                ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
                            }else{
                                ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
                            }
                            esp_log_buffer_hex(GATTC_TAG, eventMsg->gattc_param.notify.value, eventMsg->gattc_param.notify.value_len);

                            if (eventMsg->gattc_param.notify.value[0] > 0){
                                gpio_set_level(GPIO_LED_0, 1);
                            }else{
                                gpio_set_level(GPIO_LED_0, 0);
                            }
                            break;
                        }

                        case ESP_GATTC_WRITE_DESCR_EVT: {
                            if (eventMsg->gattc_param.write.status != ESP_GATT_OK){
                                ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", eventMsg->gattc_param.write.status);
                                break;
                            }
                            ESP_LOGI(GATTC_TAG, "write descr success ");
                            uint16_t count = 0;
                            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( eventMsg->gattc_if,
                                                                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                ESP_GATT_DB_DESCRIPTOR,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                &count);

                            ESP_LOGI(GATTC_TAG, "characters count=%d", count);
                            #if 1
                            char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                            ret_status = esp_ble_gattc_get_char_by_uuid( eventMsg->gattc_if,
                                                                eventMsg->gattc_param.search_cmpl.conn_id,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                remote_filter_led_char_uuid,
                                                                char_elem_result,
                                                                &count);
                            // xTaskCreate(led_blink_task, "led blink", configMINIMAL_STACK_SIZE*3, (void *)&xLEDTaskArgs, 5, &led_task_handle);
                            char unEncryptData[17];
                            time_t now;
                            time(&now);
                            sprintf(unEncryptData, "%02X%02X%02X%02X", 
                                                    (uint8_t)(now>>24), 
                                                    (uint8_t)((now>>16)&0xff), 
                                                    (uint8_t)((now>>8)&0xff), 
                                                    (uint8_t)((now)&0xff));

                            memcpy(unEncryptData+8, "jerryA0A", 9);
                            ESP_LOGI(GATTC_TAG, "time is %ld, plainData:%s", (long)now, unEncryptData);

                            memset(cipher, 0, 17);
                            mbedtls_aes_init(&ctx);
                            mbedtls_aes_setkey_enc(&ctx, key, 128);
                            mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT,(const uint8_t *)unEncryptData, cipher);
                            mbedtls_aes_free(&ctx);

                            ret_status = esp_ble_gattc_write_char( eventMsg->gattc_if,
                                                                   gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                   char_elem_result[0].char_handle,
                                                                   16, //sizeof(led_en),
                                                                   cipher, //&len_en,
                                                                   ESP_GATT_WRITE_TYPE_RSP,
                                                                   ESP_GATT_AUTH_REQ_NONE );
                            
                            if (ret_status != ESP_GATT_OK){
                                ESP_LOGE(GATTC_TAG, "esp led_on write_char error");
                            }

                            vTaskDelay(20/portTICK_PERIOD_MS);
                            free(char_elem_result);
                            esp_ble_gattc_close(eventMsg->gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id);
                            #endif
                            break;
                        }

                        case ESP_GATTC_SRVC_CHG_EVT: {
                            esp_bd_addr_t bda;
                            memcpy(bda, eventMsg->gattc_param.srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
                            ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
                            esp_log_buffer_hex(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
                            break;
                        }
                        case ESP_GATTC_WRITE_CHAR_EVT: {
                            if (eventMsg->gattc_param.write.status != ESP_GATT_OK){
                                ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", eventMsg->gattc_param.write.status);
                                break;
                            }
                            ESP_LOGI(GATTC_TAG, "write char success ");
                            break;
                        }

                        case ESP_GATTC_DISCONNECT_EVT: {
                            connect = false;
                            get_server = false;
                            ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", eventMsg->gattc_param.disconnect.reason);
                            break;
                        }

                        default: 
                            break;
                    }
                } else { //ble_gap_vent

                    ESP_LOGI(GATTC_TAG, "receive gap event succ");
                    switch (eventMsg->gap_event) {

                        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
                            //the unit of the duration is second
                            uint32_t duration = 30;
                            esp_ble_gap_start_scanning(duration);
                            ESP_LOGI(GATTC_TAG, "start scan %ds", duration);
                            break;
                        }

                        case ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT:{
                            esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
                            if (scan_ret){
                                ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
                            }
                            break;
                        }

                        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
                            //scan start complete event to indicate scan start successfully or failed
                            if (eventMsg->gap_param.scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                                ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", eventMsg->gap_param.scan_start_cmpl.status);
                                break;
                            }
                            ESP_LOGI(GATTC_TAG, "scan start success");
                            break;
                        }
                        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
                            char *mac = malloc(18);
                            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)&(eventMsg->gap_param);
                            switch (scan_result->scan_rst.search_evt) {
                            case ESP_GAP_SEARCH_INQ_RES_EVT:
                                // esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
                                memset(mac, 0, 18);
                                sprintf(mac, MACSTR, MAC2STR(scan_result->scan_rst.bda));
                                ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
                                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                                    ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                                ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
                                esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
                        
                    #if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
                                if (scan_result->scan_rst.adv_data_len > 0) {
                                    ESP_LOGI(GATTC_TAG, "adv data:");
                                    esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
                                }
                                if (scan_result->scan_rst.scan_rsp_len > 0) {
                                    ESP_LOGI(GATTC_TAG, "scan resp:");
                                    esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
                                }
                    #endif
                                ESP_LOGI(GATTC_TAG, "\n");
                        
                                if (adv_name != NULL) {
                                    if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                                        ESP_LOGI(GATTC_TAG, "searched device %s\n", remote_device_name);
                                            ESP_LOGI(GATTC_TAG, "mac %s, bda %s\n", remote_device_mac, mac);
                                        if (strncmp(mac, remote_device_mac, 18) == 0) {
                                            ESP_LOGI(GATTC_TAG, "searched device mac %s\n", remote_device_mac);
                                            if (connect == false) {
                                                connect = true;
                                                ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                                                esp_ble_gap_stop_scanning();
                                                esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                                            }
                                        }
                                    }
                                }

                                break;
                            case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                                ESP_LOGI(GATTC_TAG, "serach inquery complete!");
                                break;
                            default:
                                break;
                            }
                            free(mac);
                            break;
                        }


                        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
                            if (eventMsg->gap_param.scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
                                ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", eventMsg->gap_param.scan_stop_cmpl.status);
                                break;
                            }
                            ESP_LOGI(GATTC_TAG, "stop scan successfully");
                            break;

                        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
                            if (eventMsg->gap_param.adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
                                ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x", eventMsg->gap_param.adv_stop_cmpl.status);
                                break;
                            }
                            ESP_LOGI(GATTC_TAG, "stop adv successfully");
                            break;
                        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
                             ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                                      eventMsg->gap_param.update_conn_params.status,
                                      eventMsg->gap_param.update_conn_params.min_int,
                                      eventMsg->gap_param.update_conn_params.max_int,
                                      eventMsg->gap_param.update_conn_params.conn_int,
                                      eventMsg->gap_param.update_conn_params.latency,
                                      eventMsg->gap_param.update_conn_params.timeout);
                            break;
                        case ESP_GAP_BLE_AUTH_CMPL_EVT: 
                            ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_AUTH_CMPL_EVT");
                            break;             
                        default:
                            break;
                    }

                } 
            } 
            vPortFree(eventMsg);
        } else if (xQueueContainsData == server_msg_queue) {
            char *mac=NULL;

            result = xQueueReceive(server_msg_queue, &mac, 0);
            if (result == pdTRUE) {
                memcpy(remote_device_mac, mac, 18);
                ESP_LOGI(GATTC_TAG, "server cmmand start to unlock the mac %s", remote_device_mac);


                /***************************
                 * set scan white list
                 */
                uint8_t *remote_bda;
                uint16_t listLength = 0;
                remote_bda = pvPortMalloc(6);
                macStrTobuffer(remote_device_mac, remote_bda);
                // ESP_ERROR_CHECK(esp_ble_gap_clear_whitelist());
                vTaskDelay(20/portTICK_PERIOD_MS);
                ESP_ERROR_CHECK(esp_ble_gap_get_whitelist_size(&listLength));
                ESP_LOGI(GATTC_TAG, "set whitelist,%d",listLength);
                listLength = 0;
                esp_log_buffer_hex(GATTC_TAG, remote_bda, sizeof(esp_bd_addr_t));
                ESP_ERROR_CHECK(esp_ble_gap_update_whitelist(pdTRUE, remote_bda, BLE_WL_ADDR_TYPE_RANDOM));
                vTaskDelay(20/portTICK_PERIOD_MS);
                ESP_ERROR_CHECK(esp_ble_gap_get_whitelist_size(&listLength));
                ESP_LOGI(GATTC_TAG, "after set,%d",listLength);

                memset(remote_bda, 0, 6);
                esp_read_mac(remote_bda,2);
                ESP_LOGI(GATTC_TAG, "Local BLE MAC address:");
                esp_log_buffer_hex(GATTC_TAG, remote_bda, sizeof(esp_bd_addr_t));
                free(remote_bda);

            }


            mbedtls_aes_init(&ctx);
            if (mbedtls_aes_setkey_enc(&ctx, key, 128) == 0) {
                ESP_LOGI(GATTC_TAG, "Before Encrypt:%s", plain);
                mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, plain, cipher);
                memset(plain, 0, 17); //clear plain
                mbedtls_aes_setkey_dec(&ctx, key, 128);
                mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_DECRYPT, cipher, plain);
                ESP_LOGI(GATTC_TAG, "After Decrypt:%s", plain);

            }
            mbedtls_aes_free(&ctx);
        }
    }

    vTaskDelete(NULL);
}


static int8_t charHexToByte(char hex)
{
    int8_t data = -1;
    if (hex >= '0' && hex <= '9')
        data = hex - '0';
    else if (hex >= 'a' && hex <= 'f')
        data = hex - 'a' + 10;
    else if (hex >= 'A' && hex <= 'F')
        data = hex - 'A' + 10;

    return data;
}

static void macStrTobuffer(char *mac, uint8_t *buffer) 
{
    int8_t hexL;
    int8_t hexH;
    int j = 0;
    for (int i=0; i<17; i++) {
        hexH = charHexToByte(mac[i]);
        hexL = charHexToByte(mac[i+1]);
        if(hexL >0 && hexH >0) {
            buffer[j++] = (hexH<<4) + hexL;
        }
        i=i+2;
    }
}


void  initialize_ble_client(void)
{
    esp_err_t ret;

    ble_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (ble_evt_queue == NULL){
        ESP_LOGE(GATTC_TAG, "queue create failed");
        return;
    }

    server_msg_queue = xQueueCreate(5, sizeof(uint32_t));
    timeOut_sig_queue = xQueueCreate(1, sizeof(char *));  //sizeof(char *) 返回char *指针所占的空间


    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    if (xTaskCreate(ble_openLock_task, "ble openLock", 0x900, NULL, 7, NULL) != pdPASS){
        ESP_LOGE(GATTC_TAG, "create task failed");
    }

    if (xTaskCreate(sync_sntp_task, "snyc sntp", 0x800, NULL, 5, NULL) != pdPASS){
        ESP_LOGE(GATTC_TAG, "create task failed");
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }



}
