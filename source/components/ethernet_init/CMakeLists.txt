idf_component_register(SRCS "ethernet_init.c"
                      INCLUDE_DIRS "."
                      REQUIRES driver esp_netif esp_eth esp_event)