/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wpa2.h"
#include "esp_log.h"
#include "esp_smartconfig.h"

#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/timer.h"

#define GPIO_SMART_CONFIG           0
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_SMART_CONFIG) 
#define GPIO_SMART_LED              2
#define GPIO_LED_PIN_SEL    (1ULL<<GPIO_SMART_LED)
// need change to enum define
#define KEY_SHORT_PRESS             1
#define KEY_LONG_PRESS              2
// gpio interrupt
#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "BLUE_UNLOCK";
/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t s_wifi_event_group;
/* key press event queue */
static xQueueHandle gpio_evt_queue = NULL;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const int WIFI_START_BIT = BIT2;
static const int CONFIG_START_BIT = BIT3;
/* smartconfig task */
static void smartconfig_example_task(void * parm);


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

int system_get_time_ms(void)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int time_ms = (int64_t)tv_now.tv_sec*1000L + ((int64_t)tv_now.tv_usec)/1000;
    return time_ms;
}

esp_err_t config_key_scan(TickType_t ticks_to_wait)
{
    uint32_t io_num;
    BaseType_t press_key = pdFALSE;
    BaseType_t lift_key = pdFALSE;
    int backup_time = 0;
    while (1) {

        // receive key interrupt message
        xQueueReceive(gpio_evt_queue, &io_num, ticks_to_wait);
        if (gpio_get_level(io_num) == 0) {
            press_key = pdTRUE;
            backup_time = system_get_time_ms();
        } else if (press_key) {
            lift_key = pdTRUE;
            backup_time = system_get_time_ms() - backup_time;
        }

        if (press_key & lift_key) {
            press_key = pdFALSE;
            lift_key = pdFALSE;

            ESP_LOGI(TAG, "key press time len:%dms", backup_time);
            if (backup_time < 30) {
                continue;
            } else if (backup_time > 2000) {
                return KEY_LONG_PRESS;
            } else {
                return KEY_SHORT_PRESS;
            }
        }
    }
}


static void button_scan_task(void * parm)
{
    esp_err_t ret = 0;
    static TaskHandle_t xSmartConfigTask=NULL;

    while (1) {
        ret = config_key_scan(portMAX_DELAY);
        if (ret == -1)
            vTaskDelete(NULL);

        switch (ret) {
            case KEY_SHORT_PRESS: 
                ESP_LOGI(TAG, "key short press");

                if (xSmartConfigTask == NULL || !(xEventGroupGetBits(s_wifi_event_group) & CONFIG_START_BIT) ) {
                    if (xEventGroupGetBits(s_wifi_event_group) | CONNECTED_BIT) {
                        esp_wifi_disconnect();
                    }
                    xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, &xSmartConfigTask);
                }
                break;
            case KEY_LONG_PRESS: 
                if (xEventGroupGetBits(s_wifi_event_group) & CONFIG_START_BIT) {
                    if (xSmartConfigTask != NULL) {
                        vTaskDelete(xSmartConfigTask);
                        xEventGroupClearBits(s_wifi_event_group, CONFIG_START_BIT);
                    }
                }
                ESP_LOGI(TAG, "key long press");
                break;
        }

    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        //xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
        xEventGroupSetBits(s_wifi_event_group, WIFI_START_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (!(xEventGroupGetBits(s_wifi_event_group) & CONFIG_START_BIT)) {
            esp_wifi_connect();
        }
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        ESP_LOGI(TAG, "wifi disconnected!");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
        ESP_LOGI(TAG, "wifi connected succ!!!");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = { 0 };
        uint8_t password[65] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);

        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        ESP_ERROR_CHECK( esp_wifi_connect() );
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}


static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 1);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data", 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .port = 1883,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

static void initialise_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void vBlinkLED_task(void * parm)
{
    for (;;) {
        gpio_set_level(GPIO_SMART_LED, 0);
        vTaskDelay(100/portTICK_PERIOD_MS);
        gpio_set_level(GPIO_SMART_LED, 1);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

static void smartconfig_example_task(void * parm)
{
    EventBits_t uxBits;
    TaskHandle_t configBlinkLedTask;
    xEventGroupSetBits(s_wifi_event_group, CONFIG_START_BIT);
    ESP_ERROR_CHECK(esp_smartconfig_stop());
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    xTaskCreate(vBlinkLED_task, "Config Blink Led", 2000, NULL, 2, &configBlinkLedTask);
    uxBits = xEventGroupWaitBits(s_wifi_event_group, ESPTOUCH_DONE_BIT, true, false, pdMS_TO_TICKS(60*1000));
    if(uxBits & ESPTOUCH_DONE_BIT) {
        ESP_LOGI(TAG, "smartconfig over");
    } else {
        ESP_LOGI(TAG, "smartconfig 60s timeout");
    }

    vTaskDelete(configBlinkLedTask);
    esp_smartconfig_stop();
    xEventGroupClearBits(s_wifi_event_group, CONFIG_START_BIT);
    vTaskDelete(NULL);
}

void initialize_led(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    //gpio_input_pin_sel = ((1ULL<<18) | (1ULL<<19))
    io_conf.pin_bit_mask = GPIO_LED_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void initialize_button(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    //gpio_input_pin_sel = ((1ULL<<18) | (1ULL<<19))
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_SMART_CONFIG, gpio_isr_handler, (void*) GPIO_SMART_CONFIG);
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    initialise_wifi();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    initialize_button();
    initialize_led();
    xTaskCreate(button_scan_task, "config button scan", 2000, NULL, 2, NULL);

    EventBits_t uxBits;
    uxBits = xEventGroupWaitBits(s_wifi_event_group, WIFI_START_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (uxBits & WIFI_START_BIT) {
        ESP_LOGI(TAG, "start to connect to ap");
        esp_wifi_connect();
    }

    uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (uxBits & CONNECTED_BIT) {
        mqtt_app_start();
    }
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());

    //mqtt_app_start();
}
