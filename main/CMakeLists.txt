idf_component_register(SRCS "main.c"
                        REQUIRES fotoresistor wifi_ap
                        PRIV_REQUIRES ntc_adc
                       INCLUDE_DIRS ".")
