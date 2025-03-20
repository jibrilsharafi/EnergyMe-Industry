#include "ota_service.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_control.h"

static const char *TAG = "ota_service";
static httpd_handle_t server_handle = NULL;
static TaskHandle_t ota_task_handle = NULL;

// OTA status flags
static bool ota_update_in_progress = false;
static bool ota_update_successful = false;

// LED pattern for OTA in progress - blue breathing
void ota_led_update_task(void *pvParameter) {
    const int max_brightness = 255;
    
    while (ota_update_in_progress) {
        // Fade up
        for (int i = 0; i <= max_brightness; i += 5) {
            led_control_set_rgb(0, 0, i);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        // Fade down
        for (int i = max_brightness; i >= 0; i -= 5) {
            led_control_set_rgb(0, 0, i);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    
    // Update complete - show status with LED
    if (ota_update_successful) {
        // Success - flash green 3 times
        for (int i = 0; i < 3; i++) {
            led_control_set_rgb(0, 255, 0);
            vTaskDelay(300 / portTICK_PERIOD_MS);
            led_control_set_rgb(0, 0, 0);
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
    } else {
        // Failure - flash red 3 times
        for (int i = 0; i < 3; i++) {
            led_control_set_rgb(255, 0, 0);
            vTaskDelay(300 / portTICK_PERIOD_MS);
            led_control_set_rgb(0, 0, 0);
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
    }
    
    // Return to normal operation color (green)
    led_control_set_color(RGB_COLOR_GREEN);
    vTaskDelete(NULL);
}

// OTA update task
void ota_update_task(void *pvParameter) {
    char *firmware_url = (char *)pvParameter;
    
    // Set OTA status flags
    ota_update_in_progress = true;
    ota_update_successful = false;
    
    // Start LED indicator task
    TaskHandle_t led_task_handle = NULL;
    xTaskCreate(ota_led_update_task, "ota_led_task", 2048, NULL, 5, &led_task_handle);
    
    ESP_LOGI(TAG, "Starting OTA update from: %s", firmware_url);
    
    // Configure HTTP client for OTA
    esp_http_client_config_t config = {
        .url = firmware_url,
        .timeout_ms = 5000,
        .keep_alive_enable = true,
    };
    
    // Configure OTA function
    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    
    // Perform OTA update
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA update successful");
        ota_update_successful = true;
        // Wait for LED sequence to complete
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        ESP_LOGI(TAG, "Rebooting system...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA update failed with error: %s", esp_err_to_name(ret));
    }
    
    // Update completed (but failed if we get here)
    ota_update_in_progress = false;
    free(firmware_url);
    ota_task_handle = NULL;
    vTaskDelete(NULL);
}

// HTTP handler for OTA trigger endpoint
static esp_err_t ota_update_handler(httpd_req_t *req) {
    if (ota_update_in_progress) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "OTA update already in progress");
        return ESP_FAIL;
    }
    
    // Get JSON data from request
    char buf[1024];
    int total_len = req->content_len;
    int cur_len = 0;
    int received = 0;
    
    if (total_len >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too large");
        return ESP_FAIL;
    }
    
    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len - cur_len);
        if (received <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive data");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[cur_len] = '\0';
    
    // Parse JSON to get firmware URL
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *url_obj = cJSON_GetObjectItem(root, "firmware_url");
    if (!url_obj || !cJSON_IsString(url_obj)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid firmware_url");
        return ESP_FAIL;
    }
    
    // Copy the URL as it will be used in a separate task
    char *firmware_url = strdup(url_obj->valuestring);
    cJSON_Delete(root);
    
    if (!firmware_url) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }
    
    // Create task to handle the OTA update
    esp_err_t ret = xTaskCreate(ota_update_task, "ota_task", 8192, firmware_url, 5, &ota_task_handle);
    if (ret != pdPASS) {
        free(firmware_url);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to start OTA task");
        return ESP_FAIL;
    }
    
    // Send response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"OTA update started\"}");
    return ESP_OK;
}

// HTTP handler for OTA status endpoint
static esp_err_t ota_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "update_in_progress", ota_update_in_progress);
    
    // Get running partition info
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t app_desc;
    esp_ota_get_partition_description(running, &app_desc);
    
    cJSON_AddStringToObject(root, "version", app_desc.version);
    cJSON_AddStringToObject(root, "project_name", app_desc.project_name);
    cJSON_AddStringToObject(root, "time", app_desc.time);
    cJSON_AddStringToObject(root, "date", app_desc.date);
    cJSON_AddStringToObject(root, "partition", running->label);
    
    char *response = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    
    free(response);
    cJSON_Delete(root);
    return ESP_OK;
}

// Register HTTP endpoints
esp_err_t ota_service_init(httpd_handle_t server) {
    esp_err_t ret = ESP_OK;
    bool server_created = false;
    
    // If no server provided, create one
    if (server == NULL) {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.max_uri_handlers = 10;  // Increase if needed
        
        ESP_LOGI(TAG, "Starting HTTP server for OTA");
        ret = httpd_start(&server, &config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
            return ret;
        }
        server_created = true;
    }
    
    // Register OTA update endpoint
    httpd_uri_t ota_update_uri = {
        .uri = "/api/ota/update",
        .method = HTTP_POST,
        .handler = ota_update_handler,
        .user_ctx = NULL
    };
    
    // Register OTA status endpoint
    httpd_uri_t ota_status_uri = {
        .uri = "/api/ota/status",
        .method = HTTP_GET,
        .handler = ota_status_handler,
        .user_ctx = NULL
    };
    
    ret = httpd_register_uri_handler(server, &ota_update_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register OTA update handler: %s", esp_err_to_name(ret));
        if (server_created) {
            httpd_stop(server);
        }
        return ret;
    }
    
    ret = httpd_register_uri_handler(server, &ota_status_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register OTA status handler: %s", esp_err_to_name(ret));
        if (server_created) {
            httpd_stop(server);
        }
        return ret;
    }
    
    server_handle = server;
    ESP_LOGI(TAG, "OTA service initialized");
    return ESP_OK;
}

// Public function to start an OTA update
esp_err_t ota_start_update(const char *firmware_url) {
    if (ota_update_in_progress) {
        ESP_LOGE(TAG, "OTA update already in progress");
        return ESP_FAIL;
    }
    
    // Copy the URL as it will be used in a separate task
    char *url_copy = strdup(firmware_url);
    if (!url_copy) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }
    
    // Create task to handle the OTA update
    esp_err_t ret = xTaskCreate(ota_update_task, "ota_task", 8192, url_copy, 5, &ota_task_handle);
    if (ret != pdPASS) {
        free(url_copy);
        ESP_LOGE(TAG, "Failed to start OTA task");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}