idf_component_register(SRCS "thing_thermostat.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp32-owb nvs_flash esp32-ds18b20 web_thing_server)


