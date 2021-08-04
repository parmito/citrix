# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.c" "UartGps.c" "Gps.c" "Sd.c" "http_client.c" "Debug.c" "Io.c" "app_wifi.c" "State.c" "http_server.c" "UartGsm.c" "UartConfig.c" "tinygps.c"  "http_client.c" "ds18b20.c" "owb_gpio.c" "owb_rmt.c" "owb.c")
set(COMPONENT_ADD_INCLUDEDIRS " ")

register_component()
