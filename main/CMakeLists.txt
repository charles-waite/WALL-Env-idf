idf_component_register(SRC_DIRS          "." "drivers" "zap-generated"
                       PRIV_INCLUDE_DIRS "." "drivers" "zap-generated" "${ESP_MATTER_PATH}/examples/common/utils")

target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")
