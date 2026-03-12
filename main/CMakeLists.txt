idf_component_register(
    SRCS "app_main.c"
         "config.c"
         "config_store.c"
         "display.c"
         "touch.c"
         "led.c"
         "wifi_mgr.c"
         "wifi_prov.c"
         "power.c"
         "http.c"
         "mqtt.c"
         "audio.c"
         "record.c"
         "battery.c"
         "avatar_img.c"
         "scenario_img.c"
         "stream_player.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_psram esp_wifi nvs_flash esp_event esp_lcd
             esp_netif esp_timer freertos mqtt lwip
             espressif__led_strip esp_http_client json mbedtls
             espressif__es8311 esp_adc esp_rom
)

# Re-run CMake whenever .env changes so new values are always picked up
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../.env)

# Inject secrets from environment variables (source .env before building)
foreach(VAR SECRET_SSID SECRET_PASSWORD SECRET_APIKEY SECRET_DOLL_BODY_ID SECRET_SERVER_URL SECRET_MQTT_URL SECRET_STREAM_RECORDER_URL SECRET_STREAM_PLAYER_URL)
    if(DEFINED ENV{${VAR}})
        target_compile_definitions(${COMPONENT_LIB} PRIVATE ${VAR}="$ENV{${VAR}}")
    endif()
endforeach()
