idf_component_register(SRCS "audio_vu_main.c" "pantalla.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2s esp_driver_i2c esp_driver_gpio
                                esp_driver_spi esp_driver_touch_sens esp_lcd
                                esp_timer)

# Pantalla seleccionada al compilar:  idf.py -DPANTALLA=1 build
#   0 sin ESP-LyraP-LCD32: el VU sale por consola y por el LED RGB
#   1 con ESP-LyraP-LCD32, controlador ST7789   (el habitual)
#   2 con ESP-LyraP-LCD32, controlador ILI9341  (algunas series del panel)
if(NOT DEFINED PANTALLA)
    set(PANTALLA 1)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE PANTALLA=${PANTALLA})
message(STATUS "audio_vu: compilando con PANTALLA=${PANTALLA}")
