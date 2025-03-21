#include "http_service.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_control.h"
#include "esp_random.h"
#include "mbedtls/md5.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "sdkconfig.h"
#include "mux_control.h" // Add this for mux_control_select

static const char *TAG = "http_service";
static httpd_handle_t server_handle = NULL;
static TaskHandle_t ota_task_handle = NULL;
static TaskHandle_t restart_task_handle = NULL; // For restart functionality

// OTA status flags
static bool ota_update_in_progress = false;
static bool ota_update_successful = false;

// Forward declarations
static void restart_task(void *pvParameter);

// Authentication related variables
#if CONFIG_HTTP_SERVER_ENABLE_AUTH
static char auth_nonce[33] = {0};
static uint8_t nonce_count = 0;
#endif

// Component registration structure
#define MAX_REGISTERED_COMPONENTS 10
typedef struct {
    const char *name;
    void (*data_callback)(cJSON *json);
    httpd_uri_t *endpoints;
    size_t endpoint_count;
} registered_component_t;

static registered_component_t registered_components[MAX_REGISTERED_COMPONENTS] = {0};
static size_t registered_component_count = 0;

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

// Generate a random nonce for digest authentication
#if CONFIG_HTTP_SERVER_ENABLE_AUTH
static void generate_auth_nonce(void) {
    sprintf(auth_nonce, "%08lx%08lx%08lx%08lx", esp_random(), esp_random(), esp_random(), esp_random());
    nonce_count = 0;
}

// Calculate MD5 hash for digest authentication
static void calculate_md5(const char *input, char *output) {
    mbedtls_md5_context ctx;
    unsigned char digest[16];
    
    mbedtls_md5_init(&ctx);
    mbedtls_md5_starts(&ctx);
    mbedtls_md5_update(&ctx, (const unsigned char *)input, strlen(input));
    mbedtls_md5_finish(&ctx, digest);
    mbedtls_md5_free(&ctx);
    
    for (int i = 0; i < 16; i++) {
        sprintf(output + (i * 2), "%02x", digest[i]);
    }
    output[32] = '\0';
}

// Validate digest authentication
static bool validate_digest_auth(httpd_req_t *req) {
    char auth_header[512] = {0};
    
    // If authentication is disabled in config, always allow
    if (!CONFIG_HTTP_SERVER_ENABLE_AUTH) {
        return true;
    }
    
    // Check if we need to generate a new nonce
    if (strlen(auth_nonce) == 0 || nonce_count > 5) {
        generate_auth_nonce();
    }
    
    // Get Authorization header
    if (httpd_req_get_hdr_value_str(req, "Authorization", auth_header, sizeof(auth_header)) != ESP_OK) {
        // No authorization header present, send a challenge
        httpd_resp_set_status(req, "401 Unauthorized");
        
        // Format the authentication challenge string
        char auth_str[256];
        snprintf(auth_str, sizeof(auth_str),
                "Digest realm=\"%s\", nonce=\"%s\", algorithm=MD5, qop=\"auth\"", 
                CONFIG_HTTP_SERVER_AUTH_REALM, auth_nonce);
                
        // Set the WWW-Authenticate header
        httpd_resp_set_hdr(req, "WWW-Authenticate", auth_str);
        httpd_resp_send(req, "Authentication required", -1);
        return false;
    }
    
    // Parse the authorization header
    char username[64] = {0};
    char realm[64] = {0};
    char nonce[64] = {0};
    char uri[128] = {0};
    char response[64] = {0};
    char qop[16] = {0};
    char nc[16] = {0};
    char cnonce[64] = {0};
    
    // Extract values from auth header
    if (sscanf(auth_header, 
               "Digest username=\"%[^\"]\", realm=\"%[^\"]\", nonce=\"%[^\"]\", uri=\"%[^\"]\", "
               "response=\"%[^\"]\", qop=%[^,], nc=%[^,], cnonce=\"%[^\"]\"",
               username, realm, nonce, uri, response, qop, nc, cnonce) < 7) {
        // Malformed header
        ESP_LOGE(TAG, "Malformed digest auth header");
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "Bad request", -1);
        return false;
    }
    
    // Verify nonce
    if (strcmp(nonce, auth_nonce) != 0) {
        // Invalid nonce, send a new challenge
        generate_auth_nonce();
        
        // Format the auth challenge with new nonce
        char auth_str[256];
        snprintf(auth_str, sizeof(auth_str),
                "Digest realm=\"%s\", nonce=\"%s\", algorithm=MD5, qop=\"auth\"", 
                CONFIG_HTTP_SERVER_AUTH_REALM, auth_nonce);
                
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_set_hdr(req, "WWW-Authenticate", auth_str);
        httpd_resp_send(req, "Invalid nonce", -1);
        return false;
    }
    
    // Calculate expected response
    char ha1[33] = {0};
    char ha2[33] = {0};
    char expected_response[33] = {0};
    char method[10] = {0};
    
    // Get request method
    if (req->method == HTTP_GET) {
        strcpy(method, "GET");
    } else if (req->method == HTTP_POST) {
        strcpy(method, "POST");
    } else if (req->method == HTTP_PUT) {
        strcpy(method, "PUT");
    } else if (req->method == HTTP_DELETE) {
        strcpy(method, "DELETE");
    } else {
        strcpy(method, "UNKNOWN");
    }
    
    // Calculate HA1 = MD5(username:realm:password)
    char ha1_input[256];
    snprintf(ha1_input, sizeof(ha1_input), "%s:%s:%s", 
             username, CONFIG_HTTP_SERVER_AUTH_REALM, CONFIG_HTTP_SERVER_AUTH_PASSWORD);
    calculate_md5(ha1_input, ha1);
    
    // Calculate HA2 = MD5(method:uri)
    char ha2_input[256];
    snprintf(ha2_input, sizeof(ha2_input), "%s:%s", method, uri);
    calculate_md5(ha2_input, ha2);
    
    // Calculate response = MD5(HA1:nonce:nc:cnonce:qop:HA2)
    char response_input[512];
    snprintf(response_input, sizeof(response_input), "%s:%s:%s:%s:%s:%s", 
             ha1, nonce, nc, cnonce, qop, ha2);
    calculate_md5(response_input, expected_response);
    
    // Compare responses
    if (strcmp(response, expected_response) != 0) {
        // Auth failed
        ESP_LOGE(TAG, "Authentication failed for user %s", username);
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_send(req, "Authentication failed", -1);
        return false;
    }
    
    // Auth successful
    nonce_count++;
    ESP_LOGI(TAG, "User %s successfully authenticated", username);
    return true;
}
#endif

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
    BaseType_t ret = xTaskCreate(ota_update_task, "ota_task", 8192, firmware_url, 5, &ota_task_handle);
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

// HTTP handler for system data endpoint
static esp_err_t system_data_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    
    // Add system info
    cJSON *system_info = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "system_info", system_info);
    
    // Simplified system info - removed chip info that's not available
    cJSON_AddStringToObject(system_info, "model", "ESP32-S3");
    cJSON_AddNumberToObject(system_info, "cores", 2); // Assuming dual core
    
    // Get running partition info
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t app_desc;
    esp_ota_get_partition_description(running, &app_desc);
    
    cJSON_AddStringToObject(system_info, "version", app_desc.version);
    cJSON_AddStringToObject(system_info, "project_name", app_desc.project_name);
    cJSON_AddStringToObject(system_info, "date", app_desc.date);
    cJSON_AddStringToObject(system_info, "time", app_desc.time);
    
    // Get MAC address
    uint8_t mac[6];
    char mac_str[18];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON_AddStringToObject(system_info, "mac_address", mac_str);
    
    // Add system uptime using xTaskGetTickCount instead of esp_timer_get_time
    uint32_t ticks = xTaskGetTickCount();
    uint32_t uptime_seconds = ticks / configTICK_RATE_HZ;
    cJSON_AddNumberToObject(system_info, "uptime_seconds", uptime_seconds);
    
    // Call registered component data callbacks to add their data
    for (int i = 0; i < registered_component_count; i++) {
        if (registered_components[i].data_callback) {
            cJSON *component_data = cJSON_CreateObject();
            cJSON_AddItemToObject(root, registered_components[i].name, component_data);
            registered_components[i].data_callback(component_data);
        }
    }
    
    // Send the JSON response
    char *response = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    
    free(response);
    cJSON_Delete(root);
    return ESP_OK;
}

// HTTP handler for root endpoint - serves a basic HTML dashboard
static esp_err_t root_handler(httpd_req_t *req) {
    extern const unsigned char index_html_start[] asm("_binary_index_html_start");
    extern const unsigned char index_html_end[] asm("_binary_index_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_size);
    
    return ESP_OK;
}

// Handler for JavaScript files
static esp_err_t main_js_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/javascript");
    extern const uint8_t main_js_start[] asm("_binary_main_js_start");
    extern const uint8_t main_js_end[] asm("_binary_main_js_end");
    httpd_resp_send(req, (const char *)main_js_start, main_js_end - main_js_start);
    return ESP_OK;
}

// Register a new component with the HTTP server
esp_err_t http_server_register_component(const char *name, 
                                         http_server_data_callback_t data_callback,
                                         const httpd_uri_t *endpoints, 
                                         size_t endpoint_count) {
    if (registered_component_count >= MAX_REGISTERED_COMPONENTS) {
        ESP_LOGE(TAG, "Cannot register component %s: maximum reached", name);
        return ESP_FAIL;
    }
    
    if (server_handle == NULL) {
        ESP_LOGE(TAG, "HTTP server not started");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Register the component
    registered_components[registered_component_count].name = name;
    registered_components[registered_component_count].data_callback = data_callback;
    registered_components[registered_component_count].endpoints = (httpd_uri_t *)endpoints;
    registered_components[registered_component_count].endpoint_count = endpoint_count;
    
    // Register endpoints with the server
    for (int i = 0; i < endpoint_count; i++) {
        esp_err_t ret = httpd_register_uri_handler(server_handle, &endpoints[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register endpoint %s: %s", 
                     endpoints[i].uri, esp_err_to_name(ret));
            return ret;
        }
    }
    
    registered_component_count++;
    ESP_LOGI(TAG, "Component %s registered with %d endpoints", name, endpoint_count);
    return ESP_OK;
}

// API handler for device readings
static esp_err_t readings_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    
    // Create voltage object
    cJSON *voltage = cJSON_CreateObject();
    cJSON_AddNumberToObject(voltage, "l1", 230.5); // Example data - replace with actual readings
    cJSON_AddItemToObject(root, "voltage", voltage);
    
    // Create current object
    cJSON *current = cJSON_CreateObject();
    cJSON_AddNumberToObject(current, "l1", 2.34);  // Example data
    cJSON_AddItemToObject(root, "current", current);
    
    // Create power object
    cJSON *power = cJSON_CreateObject();
    cJSON_AddNumberToObject(power, "l1", 540.0);   // Example data
    cJSON_AddItemToObject(root, "power", power);
    
    // Add energy consumption data
    cJSON *energy = cJSON_CreateObject();
    cJSON_AddNumberToObject(energy, "total", 125.45);  // Example data
    cJSON_AddItemToObject(root, "energy", energy);
    
    // Add active power data
    cJSON *activePower = cJSON_CreateObject();
    cJSON_AddNumberToObject(activePower, "total", 750.0);  // Example data
    cJSON_AddItemToObject(root, "activePower", activePower);
    
    // Add power factor data
    cJSON *powerFactor = cJSON_CreateObject();
    cJSON_AddNumberToObject(powerFactor, "total", 0.95);  // Example data
    cJSON_AddItemToObject(root, "powerFactor", powerFactor);
    
    // Add multiplexer data
    cJSON *mux = cJSON_CreateObject();
    cJSON_AddNumberToObject(mux, "channel", 3);  // Example data - replace with actual channel
    cJSON_AddItemToObject(root, "mux", mux);
    
    // Convert JSON to string
    char *json_str = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    // Free JSON objects
    free(json_str);
    cJSON_Delete(root);
    
    return ESP_OK;
}

// API handler for setting multiplexer channel
static esp_err_t mux_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;
    
    if (remaining > sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too large");
        return ESP_FAIL;
    }
    
    if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive content");
        return ESP_FAIL;
    }
    
    buf[ret] = '\0';  // Null-terminate the received data
    
    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *channel_json = cJSON_GetObjectItem(root, "channel");
    if (!cJSON_IsNumber(channel_json)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid channel");
        return ESP_FAIL;
    }
    
    int channel = channel_json->valueint;
    cJSON_Delete(root);
    
    // Validate channel range
    if (channel < 0 || channel > 15) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Channel out of range (0-15)");
        return ESP_FAIL;
    }
    
    // Set the multiplexer channel
    esp_err_t err = mux_control_select(channel);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set channel");
        return ESP_FAIL;
    }
    
    // Send success response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true}");
    
    return ESP_OK;
}

// Implementation of restart task
static void restart_task(void *pvParameter)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_restart();
}

// API handler for device restart
static esp_err_t restart_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true,\"message\":\"Device will restart shortly\"}");
    
    // Schedule a restart after response is sent
    BaseType_t ret = xTaskCreate(restart_task, "restart_task", 2048, NULL, 5, &restart_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create restart task");
    }
    
    return ESP_OK;
}

// Initialize the HTTP server
esp_err_t http_server_init(void) {
    ESP_LOGI(TAG, "Initializing HTTP server");
    
    // Initialize authentication if enabled
#if CONFIG_HTTP_SERVER_ENABLE_AUTH
    ESP_LOGI(TAG, "Digest authentication enabled with realm: %s", CONFIG_HTTP_SERVER_AUTH_REALM);
    generate_auth_nonce();
#else
    ESP_LOGI(TAG, "Authentication disabled");
#endif
    
    // Create HTTP server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = CONFIG_HTTP_SERVER_MAX_URI_HANDLERS;
    config.stack_size = CONFIG_HTTP_SERVER_STACK_SIZE;
    config.task_priority = CONFIG_HTTP_SERVER_TASK_PRIORITY;
    config.server_port = CONFIG_HTTP_SERVER_PORT;
    config.lru_purge_enable = true;
    
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    esp_err_t ret = httpd_start(&server_handle, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register built-in handlers
    // Root page handler
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL
    };
    
    // JavaScript handler
    httpd_uri_t js_uri = {
        .uri = "/main.js",
        .method = HTTP_GET,
        .handler = main_js_handler,
        .user_ctx = NULL
    };
    
    // System data API endpoint
    httpd_uri_t system_data_uri = {
        .uri = "/api/system/data",
        .method = HTTP_GET,
        .handler = system_data_handler,
        .user_ctx = NULL
    };
    
    // OTA update endpoint
    httpd_uri_t ota_update_uri = {
        .uri = "/api/ota/update",
        .method = HTTP_POST,
        .handler = ota_update_handler,
        .user_ctx = NULL
    };
    
    // OTA status endpoint
    httpd_uri_t ota_status_uri = {
        .uri = "/api/ota/status",
        .method = HTTP_GET,
        .handler = ota_status_handler,
        .user_ctx = NULL
    };
    
    // API readings endpoint
    httpd_uri_t readings_uri = {
        .uri = "/api/readings",
        .method = HTTP_GET,
        .handler = readings_handler,
        .user_ctx = NULL
    };
    
    // MUX control endpoint
    httpd_uri_t mux_uri = {
        .uri = "/api/mux",
        .method = HTTP_POST,
        .handler = mux_handler,
        .user_ctx = NULL
    };
    
    // Restart device endpoint
    httpd_uri_t restart_uri = {
        .uri = "/api/restart",
        .method = HTTP_POST,
        .handler = restart_handler,
        .user_ctx = NULL
    };
    
    // Register built-in endpoints
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_handle, &root_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_handle, &js_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_handle, &system_data_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_handle, &ota_update_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_handle, &ota_status_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_handle, &readings_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_handle, &mux_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_handle, &restart_uri));
    
    ESP_LOGI(TAG, "HTTP server initialized and started");
    return ESP_OK;
}