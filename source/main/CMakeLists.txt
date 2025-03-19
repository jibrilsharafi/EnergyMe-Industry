idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES ade7880 ethernet_init mux_control led_control log_handler
)