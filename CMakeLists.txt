target_sources_ifdef(CONFIG_BT_REMOTE_STATUS_LED app PRIVATE src/status_led.c)
target_sources_ifdef(CONFIG_BT_REMOTE_PAIRING_WINDOW app PRIVATE src/pairing.c src/behavior_pairing.c)
target_sources_ifdef(CONFIG_BT_REMOTE_DIAG app PRIVATE src/diag.c)
