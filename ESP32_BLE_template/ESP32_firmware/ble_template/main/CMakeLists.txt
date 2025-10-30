idf_component_register(
    SRCS "ble_communication.c" "main.c"
    PRIV_REQUIRES bt
    INCLUDE_DIRS "./include"
)

target_compile_options(${COMPONENT_LIB} PRIVATE
-Wno-maybe-uninitialized
-Wno-missing-field-initializers
-Wno-error=sign-compare
-Wno-error=double-promotion
-Wno-type-limits)