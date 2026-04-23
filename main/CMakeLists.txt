idf_component_register(SRC_DIRS          "." "drivers" "zap-generated"
                       PRIV_INCLUDE_DIRS "." "drivers" "zap-generated"
                       REQUIRES esp_matter button app_reset bsec2 BME68x_Sensor_library)

target_compile_options(${COMPONENT_LIB} PRIVATE "-DCHIP_HAVE_CONFIG_H")
