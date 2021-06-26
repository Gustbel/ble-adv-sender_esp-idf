/*
   This example code is based in the esp-idf iBeacon example
   Gustavo Belbruno - 05/2021
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_tls.h"

static const char* DEMO_TAG = "ADV_DEV_DEMO";

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
esp_err_t status;


static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_NONCONN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


uint8_t adv_data_raw[27] = {
          2, ESP_BLE_AD_TYPE_FLAG, (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), // advertising flags
          0x05, ESP_BLE_AD_TYPE_INT_RANGE,
          0x06, 0x00, // BLE connection settings between 7.5ms
          0x30, 0x00, // and 30ms
          17, ESP_BLE_AD_TYPE_16SRV_CMPL,
          0x31, 0x12,
          0x32, 0x12,
          0x33, 0x12,
          0x34, 0x12,
          0x35, 0x12,
          0x36, 0x12,
          0x37, 0x12,
          0x38, 0x12};

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
		case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:{
		    esp_ble_gap_start_advertising(&ble_adv_params);

		    break;
		}
		case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		    //adv start complete event to indicate adv start successfully or failed
		    if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
		        ESP_LOGE(DEMO_TAG, "Adv start failed: %s", esp_err_to_name(err));
		    }
		    break;

		case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		    if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
		        ESP_LOGE(DEMO_TAG, "Adv stop failed: %s", esp_err_to_name(err));
		    }
		    else {
		        ESP_LOGI(DEMO_TAG, "Stop adv successfully");
		    }
		    break;

		default:
		    break;
    }
}


void ble_adv_device_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(DEMO_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG, "gap register error: %s", esp_err_to_name(status));
        return;
    }

}

void ble_adv_device_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_adv_device_appRegister();
}


static void ble_update(void *pvParameters)
{
	while(1)
	{	
		adv_data_raw[11]++;
		
		printf("Data[11]: 0x%x\n", adv_data_raw[11]);
				
		esp_ble_gap_config_adv_data_raw((uint8_t*)&adv_data_raw, sizeof(adv_data_raw));
		
		vTaskDelay( (1000) / portTICK_PERIOD_MS);
	}
	
    vTaskDelete(NULL);
}

void ble_adv_init()
{
	
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
 esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    status = esp_bt_controller_init(&bt_cfg);
	esp_bt_controller_enable(ESP_BT_MODE_BLE);

    ble_adv_device_init();


    if (status == ESP_OK){
        esp_ble_gap_config_adv_data_raw((uint8_t*)&adv_data_raw, sizeof(adv_data_raw));
    }
    else {
        ESP_LOGE(DEMO_TAG, "Config iBeacon data failed: %s\n", esp_err_to_name(status));
    }
}



void app_main(void)
{
	    ESP_ERROR_CHECK(nvs_flash_init());
	    ble_adv_init();
	    xTaskCreate(&ble_update, "update", 8192, NULL, 2, NULL);
}
