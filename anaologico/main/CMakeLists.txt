idf_component_register(SRCS "analogico_main.c"
                       INCLUDE_DIRS ""
                       REQUIRES esp_adc esp_driver_dac esp_driver_gpio esp_timer)

# Parte seleccionada al compilar:  idf.py -DPARTE=2 build
#   1 cuantizacion, escala y ruido (oneshot, sin DMA)
#   2 muestreo, Nyquist y plegado  (coseno hardware + ADC por DMA)
if(NOT DEFINED PARTE)
    set(PARTE 1)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE PARTE=${PARTE})
message(STATUS "analogico: compilando con PARTE=${PARTE}")
