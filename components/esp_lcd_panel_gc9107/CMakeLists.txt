set(includes "C:/Users/Stiver/esp/esp-idf/components/esp_lcd/include" "C:/Users/Stiver/esp/esp-idf/components/esp_lcd/interface")

idf_component_register(SRCS "esp_lcd_panel_gc9107.c"
                        INCLUDE_DIRS ${includes})
