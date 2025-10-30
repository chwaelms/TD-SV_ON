file(GLOB SRCS "./*.c" "./*.cc")

idf_component_register(
    SRCS ${SRCS}
    PRIV_REQUIRES bt nvs_flash driver spi_flash esp_adc icm42670 tflite-lib esp_timer esp-dsp esp32-camera esp_lcd
    INCLUDE_DIRS "./include"
)

target_compile_options(${COMPONENT_LIB} PRIVATE
-Wno-maybe-uninitialized
-Wno-missing-field-initializers
-Wno-error=sign-compare
-Wno-error=double-promotion
-Wno-type-limits)