#include <string.h>
#include <sys/param.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "sdkconfig.h"
#include "log_handler.h"
#include <sys/time.h>
#include <time.h>
#include <math.h>

// Project components
#include "network_events.h"

static const char *TAG = "log_handler";

#if CONFIG_LOG_UDP_OUTPUT_ENABLE
int udp_log_fd = -1;
static struct sockaddr_in udp_server_addr;
static bool network_connected = false;
#endif // CONFIG_LOG_UDP_OUTPUT_ENABLE

// Static variables
static bool is_initialized = false;
static uint8_t mac[6];
static char device_id[13]; // MAC-based device ID (6 bytes -> 12 hex chars + null)
static char buffer[CONFIG_LOG_UDP_BUFFER_SIZE];

// Custom log function that adds ISO timestamp format
static int udp_log_vprintf(const char *fmt, va_list args)
{
    // Get current time
    struct timeval tv;
    gettimeofday(&tv, NULL);

    // Format timestamp in ISO format
    time_t now = tv.tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    // Create timestamp and device ID prefix
    int prefix_len = snprintf(buffer, sizeof(buffer),
                              "[%04d-%02d-%02d %02d:%02d:%02d.%03ld] [device_id=%s] ",
                              timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                              timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
                              tv.tv_usec / 1000, device_id);

    // Append the actual log message
    int msg_len = vsnprintf(buffer + prefix_len,
                            sizeof(buffer) - prefix_len,
                            fmt, args);

    int total_len = prefix_len + msg_len;

    // Only send via UDP if network is connected and socket is valid
    // Fire and forget approach - we don't care if it fails
#if CONFIG_LOG_UDP_OUTPUT_ENABLE
    if (network_connected && udp_log_fd >= 0)
    {
        sendto(udp_log_fd, buffer, total_len, 0,
               (struct sockaddr *)&udp_server_addr, sizeof(udp_server_addr));
        // No error checking - we don't care if it fails
    }
#endif

    // Always print to console
    return vprintf(fmt, args);
}

// Initialize UDP socket
static void init_udp_socket(void)
{
#if CONFIG_LOG_UDP_OUTPUT_ENABLE
    // Close existing socket if any
    if (udp_log_fd >= 0) {
        close(udp_log_fd);
    }

    // Create UDP socket
    udp_log_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_log_fd < 0) {
        ESP_LOGW(TAG, "Failed to create UDP socket, will continue without UDP logging");
        return;
    }

    // Initialize server address
    memset(&udp_server_addr, 0, sizeof(udp_server_addr));
    udp_server_addr.sin_family = AF_INET;
    udp_server_addr.sin_port = htons(CONFIG_LOG_UDP_SERVER_PORT);
    inet_aton(CONFIG_LOG_UDP_SERVER_IP, &udp_server_addr.sin_addr);

    // Set non-blocking socket with short timeout
    struct timeval send_timeout = {0, 100000}; // 100ms timeout
    setsockopt(udp_log_fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
    
    ESP_LOGI(TAG, "UDP logging socket created");
#endif
}

// Network event handler
static void network_event_handler(void *arg, esp_event_base_t event_base, 
                                 int32_t event_id, void *event_data)
{
#if CONFIG_LOG_UDP_OUTPUT_ENABLE
    if (event_base == NETWORK_EVENT) {
        switch (event_id) {
            case NETWORK_EVENT_GOT_IP:
                ESP_LOGI(TAG, "Network connected, enabling UDP logging");
                network_connected = true;
                
                // Initialize UDP logging if not already done
                if (!is_initialized) {
                    // Get the default MAC address for device identification
                    esp_efuse_mac_get_default(mac);
                    
                    // Create device ID from MAC
                    snprintf(device_id, sizeof(device_id), "%02x%02x%02x%02x%02x%02x",
                             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                             
                    // Initialize UDP socket
                    init_udp_socket();
                    
                    // Redirect ESP_LOG to our custom function
                    esp_log_set_vprintf(udp_log_vprintf);
                    is_initialized = true;
                    
                    ESP_LOGI(TAG, "Log handler initialized for UDP logging to %s:%d",
                             CONFIG_LOG_UDP_SERVER_IP, CONFIG_LOG_UDP_SERVER_PORT);
                } else {
                    // If already initialized, just recreate socket
                    init_udp_socket();
                }
                break;
                
            case NETWORK_EVENT_DISCONNECTED:
            case NETWORK_EVENT_LOST_IP:
                ESP_LOGI(TAG, "Network disconnected, disabling UDP logging");
                network_connected = false;
                
                // Close the socket but keep the log handler initialized
                if (udp_log_fd >= 0) {
                    close(udp_log_fd);
                    udp_log_fd = -1;
                }
                break;
                
            default:
                break;
        }
    }
#endif
}

// Initialize the log handler
esp_err_t log_handler_init(void)
{
#if CONFIG_LOG_UDP_OUTPUT_ENABLE
    ESP_LOGI(TAG, "Registering for network events");
    
    // Register for network events
    esp_err_t ret = esp_event_handler_register(NETWORK_EVENT, ESP_EVENT_ANY_ID, 
                                              &network_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register for network events");
        return ret;
    }
    
    // Note: Actual initialization will happen when we receive a network connected event
    return ESP_OK;
#else
    ESP_LOGW(TAG, "Enhanced logging is disabled in menuconfig");
    return ESP_OK;
#endif // CONFIG_LOG_UDP_OUTPUT_ENABLE
}

esp_err_t log_handler_deinit(void)
{
#if CONFIG_LOG_UDP_OUTPUT_ENABLE
    if (!is_initialized)
    {
        return ESP_OK;
    }

    // Remove ESP log hook
    esp_log_set_vprintf(vprintf);

    // Close socket if it was open
    if (udp_log_fd >= 0)
    {
        close(udp_log_fd);
        udp_log_fd = -1;
    }
    
    // Unregister from events
    esp_event_handler_unregister(NETWORK_EVENT, ESP_EVENT_ANY_ID, &network_event_handler);

    network_connected = false;
    is_initialized = false;
    ESP_LOGW(TAG, "Log handler deinitialized");

    return ESP_OK;
#else
    return ESP_OK;
#endif // CONFIG_LOG_UDP_OUTPUT_ENABLE
}