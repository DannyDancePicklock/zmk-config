#pragma once

#include <stdbool.h>

/* Kopplungsmodus der BT Remote (siehe pairing.c) */
bool bt_remote_pairing_active(void);
void bt_remote_pairing_start(void);
void bt_remote_pairing_cancel(void);

/* Status-LED neu auswerten (in status_led.c) */
void bt_remote_status_led_refresh(void);
