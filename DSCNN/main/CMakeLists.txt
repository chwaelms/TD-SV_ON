
#
# Main component of TF Micro project 'micro_speech'.
#
file(GLOB SRCS "./*.c" "./*.cc")

idf_component_register(
    SRCS ${SRCS}
    PRIV_REQUIRES bt spi_flash nvs_flash
    INCLUDE_DIRS "")

target_compile_options(${COMPONENT_LIB} PRIVATE
-Wno-maybe-uninitialized
-Wno-missing-field-initializers
-Wno-error=sign-compare
-Wno-error=double-promotion
-Wno-type-limits)