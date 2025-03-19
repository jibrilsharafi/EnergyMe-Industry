#include <string.h>
#include <sys/param.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
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

static const char *TAG = "log_handler";

#if CONFIG_LOG_UDP_OUTPUT_ENABLE
int udp_log_fd = -1;
static struct sockaddr_in udp_server_addr;
#endif // CONFIG_LOG_UDP_OUTPUT_ENABLE

// Static variables
static bool is_initialized = false;
static uint8_t mac[6];
static char device_id[13]; // MAC-based device ID (6 bytes -> 12 hex chars + null)
static char buffer[CONFIG_LOG_UDP_BUFFER_SIZE];

int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    if (getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen) == -1)
    {
        printf("getsockopt failed");
        return -1;
    }
    return result;
}

int show_socket_error_reason(int socket)
{
    int err = get_socket_error_code(socket);
    printf("UDP socket error %d (%s)\n", err, strerror(err));
    return err;
}

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

    // Only send via UDP if network is connected
    if (udp_log_fd >= 0)
    {
        int err = sendto(udp_log_fd, buffer, total_len, 0,
                         (struct sockaddr *)&udp_server_addr, sizeof(udp_server_addr));

        if (err < 0)
        {
            show_socket_error_reason(udp_log_fd);
            log_handler_deinit();
            return vprintf(fmt, args);
        }
    }

    return vprintf(fmt, args);
}

esp_err_t log_handler_init(void)
{
#if CONFIG_LOG_UDP_OUTPUT_ENABLE
    struct timeval send_timeout = {1,0};

    ESP_LOGI(TAG, "Initializing log handler for UDP output to %s:%d",
             CONFIG_LOG_UDP_SERVER_IP, CONFIG_LOG_UDP_SERVER_PORT);

    // Check if already initialized
    if (is_initialized)
    {
        ESP_LOGW(TAG, "Log handler already initialized");
        return ESP_OK;
    }

    // Get the default MAC address for device identification
    esp_efuse_mac_get_default(mac);

    // Create device ID from MAC (last 3 bytes as hex)
    snprintf(device_id, sizeof(device_id), "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Initialize socket for UDP logging
    udp_log_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_log_fd < 0)
    {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return ESP_FAIL;
    }

    memset(&udp_server_addr, 0, sizeof(udp_server_addr));
    udp_server_addr.sin_family = AF_INET;
    udp_server_addr.sin_port = htons(CONFIG_LOG_UDP_SERVER_PORT);
    inet_aton(CONFIG_LOG_UDP_SERVER_IP, &udp_server_addr.sin_addr);

    int err = setsockopt(udp_log_fd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&send_timeout, sizeof(send_timeout));
	if (err < 0) {
	   ESP_LOGE("UDP_LOGGING", "Failed to set SO_SNDTIMEO. Error %d", err);
	}

    // Redirect ESP_LOG to our custom function
    esp_log_set_vprintf(udp_log_vprintf);

    is_initialized = true;
    ESP_LOGI(TAG, "Log handler initialized for UDP logging to %s:%d",
             CONFIG_LOG_UDP_SERVER_IP, CONFIG_LOG_UDP_SERVER_PORT);

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

    is_initialized = false;
    ESP_LOGW(TAG, "Log handler deinitialized");

    return ESP_OK;
#else
    return ESP_OK;
#endif // CONFIG_LOG_UDP_OUTPUT_ENABLE
}