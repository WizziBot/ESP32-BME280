#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <sys/param.h>
#include "nvs_flash.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_err.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "http_server.h"

#define TAG_HTTPD "HTTPD"
#define HTTPD_SERVER_PORT 8081

struct sensor_data_t {
    SemaphoreHandle_t data_lock;
    char* data_str;
};

struct sensor_data_t sensor_data = {0};

/* An HTTP GET handler */
static esp_err_t data_get_handler(httpd_req_t *req)
{
    // Mutex used
    if(xSemaphoreTake(sensor_data.data_lock,500/portTICK_PERIOD_MS) == pdTRUE){
        char* buf = malloc(64*sizeof(char));
        strncpy(buf,sensor_data.data_str,64*sizeof(char));
        xSemaphoreGive(sensor_data.data_lock);
        const char* resp_str = (const char*) buf;

        httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    } else {
        const char* resp_str = "Could not get data: mutex obtained.";
        httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static const httpd_uri_t get_sensor_data = {
    .uri       = "/data",
    .method    = HTTP_GET,
    .handler   = data_get_handler,
};

static httpd_handle_t start_webserver()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.server_port = HTTPD_SERVER_PORT;

    // Start the httpd server
    ESP_LOGI(TAG_HTTPD, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG_HTTPD, "Registering URI handlers");
        httpd_register_uri_handler(server, &get_sensor_data);
        return server;
    }

    ESP_LOGI(TAG_HTTPD, "Error starting server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG_HTTPD, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG_HTTPD, "Failed to stop http server");
        }
    }
}

void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG_HTTPD, "Starting webserver");
        *server = start_webserver();
    }
}

void http_server_init(SemaphoreHandle_t data_lock,char* data_str)
{
    static httpd_handle_t server = NULL;
    
    struct sensor_data_t data_recvd = {
        .data_lock = data_lock,
        .data_str = data_str
    };
    sensor_data = data_recvd;

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));

    server = start_webserver(data_str);
}