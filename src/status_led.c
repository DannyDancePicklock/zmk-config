/*
 * Status-LED fuer die BT Remote
 *
 * - Kopplungsmodus (Profil offen, wartet auf neues Geraet): schnelles Blinken
 * - Verbindung hergestellt: einmal kurz aufleuchten
 * - Sonst (getrennt, aber gekoppelt / Schlaf): aus
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <zmk/ble.h>
#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define CONNECT_BLINK_MS CONFIG_BT_REMOTE_STATUS_LED_CONNECT_MS
#define PAIRING_BLINK_MS CONFIG_BT_REMOTE_STATUS_LED_PAIRING_MS

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_NODELABEL(status_led), gpios);

static bool was_connected;
static bool pairing_active;
static bool led_on;

static struct k_work_delayable blink_work;
static struct k_work_delayable update_work;

static void led_set(bool on) {
    led_on = on;
    gpio_pin_set_dt(&led, on);
}

/* Takt fuer das Blinken: im Kopplungsmodus umschalten, sonst ausschalten
 * (beendet das einmalige Aufleuchten nach dem Verbinden). */
static void blink_handler(struct k_work *work) {
    if (pairing_active) {
        led_set(!led_on);
        k_work_reschedule(&blink_work, K_MSEC(PAIRING_BLINK_MS));
    } else {
        led_set(false);
    }
}

/* Aktuellen Bluetooth-Zustand auswerten und LED-Modus setzen */
static void update_handler(struct k_work *work) {
    bool connected = zmk_ble_active_profile_is_connected();
    bool open = zmk_ble_active_profile_is_open();

    if (connected) {
        pairing_active = false;
        if (!was_connected) {
            /* Frisch verbunden: einmal kurz aufleuchten */
            led_set(true);
            k_work_reschedule(&blink_work, K_MSEC(CONNECT_BLINK_MS));
        }
    } else if (open) {
        if (!pairing_active) {
            /* Kopplungsmodus: schnell blinken */
            pairing_active = true;
            led_set(true);
            k_work_reschedule(&blink_work, K_MSEC(PAIRING_BLINK_MS));
        }
    } else {
        /* Getrennt, aber gekoppelt: LED aus, spart Akku */
        pairing_active = false;
        k_work_cancel_delayable(&blink_work);
        led_set(false);
    }

    was_connected = connected;
}

static int status_led_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *act = as_zmk_activity_state_changed(eh);

    if (act) {
        if (act->state == ZMK_ACTIVITY_SLEEP) {
            /* Vor dem Tiefschlaf sicher ausschalten, sonst bleibt der Pin
             * im Schlaf eventuell auf HIGH stehen. */
            pairing_active = false;
            k_work_cancel_delayable(&blink_work);
            k_work_cancel_delayable(&update_work);
            led_set(false);
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (as_zmk_ble_active_profile_changed(eh)) {
        k_work_reschedule(&update_work, K_NO_WAIT);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(status_led, status_led_listener);
ZMK_SUBSCRIPTION(status_led, zmk_ble_active_profile_changed);
ZMK_SUBSCRIPTION(status_led, zmk_activity_state_changed);

static int status_led_init(void) {
    if (!gpio_is_ready_dt(&led)) {
        LOG_ERR("Status-LED: GPIO nicht bereit");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    k_work_init_delayable(&blink_work, blink_handler);
    k_work_init_delayable(&update_work, update_handler);

    /* Nach dem Start kurz warten, bis Bluetooth und die gespeicherten
     * Profile geladen sind, dann den Anfangszustand pruefen. */
    k_work_reschedule(&update_work, K_SECONDS(1));

    return 0;
}

SYS_INIT(status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
