# 注册 main 组件的源文件（ESP-IDF 构建系统）
#
# 注意 IDF v6.1 的组件拆分：头文件属于哪个组件就必须在 PRIV_REQUIRES 里声明，
# 否则报 "xxx.h: No such file or directory"（driver 已拆成 esp_driver_* 系列）。
#   esp_timer     -> esp_timer.h      （WiFi 断线重连用的定时器）
idf_component_register(
    SRCS "main.c" "lp5860.c" "ledmatrix.c" "wifi_app.c" "tcp_server.c" "ota_app.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_driver_spi esp_driver_gpio esp_timer nvs_flash esp_wifi esp_netif esp_event lwip app_update esp_http_client
)
