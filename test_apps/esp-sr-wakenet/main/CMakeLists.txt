idf_component_register(SRCS "app_main.cpp" "test_wakenet.cpp"
                       INCLUDE_DIRS "." "samples"
                       REQUIRES unity esp-sr esp_timer
                       WHOLE_ARCHIVE)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
