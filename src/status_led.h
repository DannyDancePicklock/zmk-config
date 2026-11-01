#pragma once

/* Ein/Aus-Animationen der Status-LED (in status_led.c) */

/* Ausschalten: rot blink, blink, ausfaden. Sperrt die LED danach. */
void bt_remote_status_led_power_off_animation(void);

/* Sperre aufheben, falls das Ausschalten doch nicht geklappt hat */
void bt_remote_status_led_unlock(void);

/* Einschalten: 2x gruen (darf schon ganz frueh beim Start laufen) */
void bt_remote_status_led_power_on_blink(void);
