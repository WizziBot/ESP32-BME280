#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/FreeRTOSConfig.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_intr_alloc.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "./components/BME280_driver/bme280.h"

#include "wifi_sta.h"
#include "http_server.h"

/* DEFINES */
#define TAG_BME "BME"

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 10000 //400kHz
#define I2C_TIMEOUT 1048575
#define ACK 0
#define NACK 1
#define CACK 1
#define NCACK 0

#define BME_SLAVE_ADDRESS (uint8_t)0b01110110
#define BME_ID_REGISTER_ADDR (uint8_t)0xD0

/* GLOBAL */
SemaphoreHandle_t data_lock;
static char data_str[64];

/* GETTING ID REGISTER TEST */
int bme_test(){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL){
        ESP_LOGE(TAG_BME,"Unable to create i2c link: Insufficient memory.");
        return -1;
    }
    uint8_t BME_ID = 0;
    esp_err_t send_err;

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd,(BME_SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE,CACK));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd,BME_ID_REGISTER_ADDR,CACK));

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd,(BME_SLAVE_ADDRESS << 1) | I2C_MASTER_READ,CACK));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd,&BME_ID,NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_err = i2c_master_cmd_begin(I2C_NUM_0,cmd,3000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    if (send_err == ESP_OK){
        ESP_LOGI(TAG_BME,"ID REGISTER: %u",BME_ID);
    }

    return (int)send_err;
}

int8_t BME280_I2C_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void* intf_ptr)
{
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BME_SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);

	return (uint8_t)espRc;
}

int8_t BME280_I2C_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void* intf_ptr) {
    esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BME_SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BME_SLAVE_ADDRESS << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);

	return (uint8_t)espRc;
}

void BME280_delay_us (uint32_t period, void *intf_ptr){
    vTaskDelay(period/(portTICK_PERIOD_MS*1000)); // Delay for number of microseconds
}


void task_bme280_normal_mode(void* ignore){
    struct bme280_settings settings = {
        .osr_p = BME280_OVERSAMPLING_16X,
        .osr_t = BME280_OVERSAMPLING_2X,
        .osr_h = BME280_OVERSAMPLING_1X,
        .standby_time = BME280_STANDBY_TIME_10_MS,
        .filter = BME280_FILTER_COEFF_16,
    };
    struct bme280_dev dev = {
        .intf = BME280_I2C_INTF,
        .intf_ptr = NULL,
        .read = BME280_I2C_read,
        .write = BME280_I2C_write,
        .delay_us = BME280_delay_us,
        .settings = settings,
    };

    int8_t com_rslt;

    com_rslt = bme280_init(&dev);

    uint8_t settings_sel;
    settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;

	com_rslt = bme280_set_sensor_settings(settings_sel, &dev);

	com_rslt += bme280_set_sensor_mode(BME280_NORMAL_MODE,&dev);

    if (com_rslt == 0) {
        struct bme280_data* data;
        if((data = malloc(sizeof(struct bme280_data))) == NULL){
            ESP_LOGE(TAG_BME, "unable to allocate bme280_data structure.");
            vTaskDelete(NULL);
            return;
        }
        if((data_lock = xSemaphoreCreateMutex()) == NULL){
            ESP_LOGE(TAG_BME, "unable to allocate data_lock mutex.");
            vTaskDelete(NULL);
            return;
        }

		while(true) {
            com_rslt = bme280_get_sensor_data(BME280_ALL,data,&dev);
            
			if (com_rslt == 0) {
				ESP_LOGI(TAG_BME, "%.5f degC / %.5f hPa / %.5f %%",
					data->temperature,
					data->pressure/100, // Pa -> hPa
					data->humidity);
                if (xSemaphoreTake(data_lock,5000/portTICK_PERIOD_MS) == pdTRUE){
                    memset(data_str,0,sizeof(data_str));
                    sprintf(data_str,"%.5f degC / %.5f hPa / %.5f %%",
                        data->temperature,
                        data->pressure/100, // Pa -> hPa
                        data->humidity);
                    xSemaphoreGive(data_lock);
                    vTaskDelay(5000/portTICK_PERIOD_MS); // wait 5s
                }
			} else {
                free(data);
				ESP_LOGE(TAG_BME, "measure error. code: %d", com_rslt);
			}
		}
	} else {
		ESP_LOGE(TAG_BME, "init or setting error. code: %d", com_rslt);
	}

	vTaskDelete(NULL);
}

void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ESP_ERROR_CHECK(nvs_flash_init());
    } else if (ret != ESP_OK){
        ESP_ERROR_CHECK(ret);
    }
    
    ESP_LOGI(TAG_BME,"BME280 driver starting...");
    // Config i2c port
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,         // GPIO
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,         // GPIO
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,  // HZ FREQ
        .clk_flags = 0,                          // you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0,&conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0,conf.mode,0,0,0));
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_NUM_0,I2C_TIMEOUT));
    ESP_LOGI(TAG_BME,"Configured I2C driver, running ID test...");

    if(bme_test() == ESP_OK){
        xTaskCreate(&task_bme280_normal_mode, "BME280N",  2048, NULL, 5, NULL);
    }
    ESP_LOGI(TAG_BME,"BME280 normal mode started.");
    if(wifi_sta() == WIFI_SUCCESS){

        http_server_init(data_lock,data_str);
    }
}
