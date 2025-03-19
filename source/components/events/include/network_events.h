#pragma once

#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Network events for application use
 */
ESP_EVENT_DECLARE_BASE(NETWORK_EVENT);

/**
 * @brief Network event types
 */
typedef enum {
    NETWORK_EVENT_CONNECTED,    /*!< Network interface connected */
    NETWORK_EVENT_DISCONNECTED, /*!< Network interface disconnected */
    NETWORK_EVENT_GOT_IP,       /*!< Network interface got IP */
    NETWORK_EVENT_LOST_IP,      /*!< Network interface lost IP */
} network_event_id_t;

#ifdef __cplusplus
}
#endif
