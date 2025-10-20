idf_component_register(SRCS
        "time_utils.c"
        "ui_render.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES espressif__qrcode wifi_manager web_server)