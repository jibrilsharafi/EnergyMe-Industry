#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"

#define CONFIG_LED_GPIO_R 22
#define CONFIG_LED_GPIO_G 23
#define CONFIG_LED_GPIO_B 21

#define MAX(a, b) ((a) > (b) ? (a) : (b))

void update_led_state(void);

// Service state enumerations
typedef enum {
    SERVICE_OK,
    SERVICE_FAULTED,
    SERVICE_INITIATING,
    SERVICE_WARNING
} ServiceState;

typedef struct {
    ServiceState wifi;
    ServiceState ethernet;
    ServiceState modbus;
    ServiceState mqtt;
    ServiceState http_server;
    // Add other services as needed
} ServiceStates;

// Device state enumeration (ordered by priority, highest to lowest)
typedef enum {
    DEVICE_STATE_FAULT,
    DEVICE_STATE_WARNING,
    DEVICE_STATE_INITIATING,
    DEVICE_STATE_OPERATIONAL
} DeviceState;

// LED state structure
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint16_t blink_on_ms;
    uint16_t blink_off_ms;
    uint8_t brightness;
} LedState;

// Global variables
static ServiceStates g_service_states = {
    .wifi = SERVICE_INITIATING,
    .ethernet = SERVICE_INITIATING,
    .modbus = SERVICE_INITIATING,
    .mqtt = SERVICE_INITIATING,
    .http_server = SERVICE_INITIATING
};
static DeviceState g_device_state = DEVICE_STATE_INITIATING;
static SemaphoreHandle_t g_state_mutex;
static QueueHandle_t g_led_state_queue;

// Function to update service state and recalculate device state
void update_service_state(ServiceState* service, ServiceState new_state) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    *service = new_state;
    
    // Recalculate device state based on service states
    if (g_service_states.wifi == SERVICE_FAULTED ||
        g_service_states.ethernet == SERVICE_FAULTED ||
        g_service_states.modbus == SERVICE_FAULTED ||
        g_service_states.mqtt == SERVICE_FAULTED ||
        g_service_states.http_server == SERVICE_FAULTED) {
        g_device_state = DEVICE_STATE_FAULT;
    } else if (g_service_states.wifi == SERVICE_WARNING ||
               g_service_states.ethernet == SERVICE_WARNING ||
               g_service_states.modbus == SERVICE_WARNING ||
               g_service_states.mqtt == SERVICE_WARNING ||
               g_service_states.http_server == SERVICE_WARNING) {
        g_device_state = DEVICE_STATE_WARNING;
    } else if (g_service_states.wifi == SERVICE_INITIATING ||
               g_service_states.ethernet == SERVICE_INITIATING ||
               g_service_states.modbus == SERVICE_INITIATING ||
               g_service_states.mqtt == SERVICE_INITIATING ||
               g_service_states.http_server == SERVICE_INITIATING) {
        g_device_state = DEVICE_STATE_INITIATING;
    } else {
        g_device_state = DEVICE_STATE_OPERATIONAL;
    }
    
    xSemaphoreGive(g_state_mutex);
    
    // Automatically update LED state
    update_led_state();
}

// LED control task
void led_control_task(void *pvParameters) {
    LedState led_state = {0};
    TickType_t last_change = xTaskGetTickCount();
    bool led_on = false;

    // Configure LEDC for PWM control
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[3] = {
        {.channel = LEDC_CHANNEL_0, .gpio_num = CONFIG_LED_GPIO_R},
        {.channel = LEDC_CHANNEL_1, .gpio_num = CONFIG_LED_GPIO_G},
        {.channel = LEDC_CHANNEL_2, .gpio_num = CONFIG_LED_GPIO_B}
    };

    for (int i = 0; i < 3; i++) {
        ledc_channel[i].speed_mode = LEDC_SPEED_MODE_MAX;
        ledc_channel[i].timer_sel = LEDC_TIMER_0;
        ledc_channel[i].intr_type = LEDC_INTR_DISABLE;
        ledc_channel[i].duty = 0;
        ledc_channel_config(&ledc_channel[i]);
    }

    while (1) {
        if (xQueueReceive(g_led_state_queue, &led_state, 0) == pdTRUE) {
            // Validate blink timing
            led_state.blink_on_ms = MAX(led_state.blink_on_ms, 10);
            led_state.blink_off_ms = MAX(led_state.blink_off_ms, 10);
        }

        TickType_t now = xTaskGetTickCount();
        if (now - last_change >= pdMS_TO_TICKS(led_on ? led_state.blink_on_ms : led_state.blink_off_ms)) {
            led_on = !led_on;
            last_change = now;
            
            // Set LED color and brightness
            uint8_t r = led_on ? (led_state.red * led_state.brightness / 255) : 0;
            uint8_t g = led_on ? (led_state.green * led_state.brightness / 255) : 0;
            uint8_t b = led_on ? (led_state.blue * led_state.brightness / 255) : 0;
            
            ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0, r);
            ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1, g);
            ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2, b);
            ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1);
            ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2);
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to prevent task starvation
    }
}

// Function to update LED state based on device state
void update_led_state() {
    LedState led_state = {0};
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    DeviceState state = g_device_state;
    xSemaphoreGive(g_state_mutex);

    switch (state) {
        case DEVICE_STATE_OPERATIONAL:
            led_state.green = 255;
            led_state.blink_on_ms = 1000;
            led_state.blink_off_ms = 1000;
            break;
        case DEVICE_STATE_INITIATING:
            led_state.blue = 255;
            led_state.blink_on_ms = 500;
            led_state.blink_off_ms = 500;
            break;
        case DEVICE_STATE_WARNING:
            led_state.red = 255;
            led_state.green = 255;
            led_state.blink_on_ms = 500;
            led_state.blink_off_ms = 500;
            break;
        case DEVICE_STATE_FAULT:
            led_state.red = 255;
            led_state.blink_on_ms = 200;
            led_state.blink_off_ms = 200;
            break;
    }

    led_state.brightness = 50; // Adjust as needed

    xQueueSend(g_led_state_queue, &led_state, portMAX_DELAY);
}

// Initialization function
void init_led_control() {
    g_state_mutex = xSemaphoreCreateMutex();
    g_led_state_queue = xQueueCreate(1, sizeof(LedState));

    xTaskCreatePinnedToCore(led_control_task, "LED Control Task", 2048, NULL, 5, NULL, 1);
}

// Example of how other tasks might use this
void wifi_task(void *pvParameters) {
    while (1) {
        // Do some work
        if (1) {
            update_service_state(&g_service_states.wifi, SERVICE_OK);
        } else if (0) {
            update_service_state(&g_service_states.wifi, SERVICE_FAULTED);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// // REST API example (to be integrated with your HTTP server)
// cJSON* get_device_status_json() {
//     cJSON *root = cJSON_CreateObject();
//     xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    
//     cJSON_AddStringToObject(root, "device_state", 
//         g_device_state == DEVICE_STATE_OPERATIONAL ? "OPERATIONAL" :
//         g_device_state == DEVICE_STATE_INITIATING ? "INITIATING" :
//         g_device_state == DEVICE_STATE_WARNING ? "WARNING" : "FAULT");
    
//     cJSON *services = cJSON_AddObjectToObject(root, "services");
//     cJSON_AddStringToObject(services, "wifi", 
//         g_service_states.wifi == SERVICE_OK ? "OK" :
//         g_service_states.wifi == SERVICE_INITIATING ? "INITIATING" :
//         g_service_states.wifi == SERVICE_WARNING ? "WARNING" : "FAULTED");
//     // Add other services similarly
    
//     xSemaphoreGive(g_state_mutex);
//     return root;
// }

// // HTTP server route handler example
// void handle_get_status(httpd_req_t *req) {
//     cJSON *status = get_device_status_json();
//     char *status_str = cJSON_Print(status);
//     httpd_resp_set_type(req, "application/json");
//     httpd_resp_send(req, status_str, strlen(status_str));
//     free(status_str);
//     cJSON_Delete(status);
// }

void app_main() {
    init_led_control();
    xTaskCreatePinnedToCore(wifi_task, "WiFi Task", 2048, NULL, 5, NULL, 1);
    // Add other tasks as needed
}