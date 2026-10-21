/*
 * RGB-Status-LED fuer die BT Remote
 *
 * - Kopplungsmodus (Profil offen, wartet auf neues Geraet): blau, schnelles Blinken
 * - Verbindung hergestellt: gruen, einmal kurz aufleuchten
 * - Akku schwach: rot, kurzer Blitz in regelmaessigen Abstaenden
 * - Sonst (verbunden, getrennt oder Schlaf): aus
 *
 * Prioritaet: Kopplungsmodus > Verbindungsblitz > Akku-Warnung
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
#include <zmk/events/battery_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define CONNECT_BLINK_MS CONFIG_BT_REMOTE_STATUS_LED_CONNECT_MS
#define PAIRING_BLINK_MS CONFIG_BT_REMOTE_STATUS_LED_PAIRING_MS
#define LOWBAT_PERCENT CONFIG_BT_REMOTE_STATUS_LED_LOWBAT_PERCENT
#define LOWBAT_FLASH_MS CONFIG_BT_REMOTE_STATUS_LED_LOWBAT_FLASH_MS
#define LOWBAT_INTERVAL_MS (CONFIG_BT_REMOTE_STATUS_LED_LOWBAT_INTERVAL_S * 1000)

/* Hysterese, damit die Warnung nicht um die Schwelle herum flattert */
#define LOWBAT_CLEAR_PERCENT (LOWBAT_PERCENT + 5)

enum led_color {
    COLOR_OFF = 0,
    COLOR_RED = BIT(0),
    COLOR_GREEN = BIT(1),
    COLOR_BLUE = BIT(2),
};

static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(DT_NODELABEL(led_red), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_NODELABEL(led_green), gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(DT_NODELABEL(led_blue), gpios);

static bool was_connected;
static bool pairing_active;
static bool pairing_led_on;
static bool flash_active;
static bool battery_low;

static struct k_work_delayable blink_work;
static struct k_work_delayable update_work;
static struct k_work_delayable lowbat_work;

static void led_show(enum led_color color) {
    gpio_pin_set_dt(&led_red, (color & COLOR_RED) != 0);
    gpio_pin_set_dt(&led_green, (color & COLOR_GREEN) != 0);
    gpio_pin_set_dt(&led_blue, (color & COLOR_BLUE) != 0);
}

/* Kurzes einmaliges Aufleuchten in einer Farbe (nicht im Kopplungsmodus) */
static void flash(enum led_color color, int ms) {
    if (pairing_active) {
        return;
    }
    flash_active = true;
    led_show(color);
    k_work_reschedule(&blink_work, K_MSEC(ms));
}

/* Takt: im Kopplungsmodus blau umschalten, sonst einen laufenden Blitz beenden */
static void blink_handler(struct k_work *work) {
    if (pairing_active) {
        pairing_led_on = !pairing_led_on;
        led_show(pairing_led_on ? COLOR_BLUE : COLOR_OFF);
        k_work_reschedule(&blink_work, K_MSEC(PAIRING_BLINK_MS));
        return;
    }

    flash_active = false;
    led_show(COLOR_OFF);
}

/* Akku-Warnung: regelmaessig kurz rot blitzen, solange der Akku schwach ist */
static void lowbat_handler(struct k_work *work) {
    if (!battery_low) {
        return;
    }
    if (!pairing_active && !flash_active) {
        flash(COLOR_RED, LOWBAT_FLASH_MS);
    }
    k_work_reschedule(&lowbat_work, K_MSEC(LOWBAT_INTERVAL_MS));
}

/* Aktuellen Bluetooth-Zustand auswerten und LED-Modus setzen */
static void update_handler(struct k_work *work) {
    bool connected = zmk_ble_active_profile_is_connected();
    bool open = zmk_ble_active_profile_is_open();

    if (connected) {
        pairing_active = false;
        if (!was_connected) {
            /* Frisch verbunden: einmal kurz gruen */
            flash(COLOR_GREEN, CONNECT_BLINK_MS);
        }
    } else if (open) {
        if (!pairing_active) {
            /* Kopplungsmodus: blau, schnell blinken */
            pairing_active = true;
            flash_active = false;
            pairing_led_on = true;
            led_show(COLOR_BLUE);
            k_work_reschedule(&blink_work, K_MSEC(PAIRING_BLINK_MS));
        }
    } else {
        /* Getrennt, aber gekoppelt: LED aus, spart Akku */
        pairing_active = false;
        flash_active = false;
        k_work_cancel_delayable(&blink_work);
        led_show(COLOR_OFF);
    }

    was_connected = connected;
}

static void battery_update(uint8_t soc) {
    if (!battery_low && soc <= LOWBAT_PERCENT) {
        battery_low = true;
        k_work_reschedule(&lowbat_work, K_NO_WAIT);
    } else if (battery_low && soc >= LOWBAT_CLEAR_PERCENT) {
        battery_low = false;
        k_work_cancel_delayable(&lowbat_work);
    }
}

static int status_led_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *act = as_zmk_activity_state_changed(eh);
    if (act) {
        if (act->state == ZMK_ACTIVITY_SLEEP) {
            /* Vor dem Tiefschlaf sicher ausschalten */
            pairing_active = false;
            flash_active = false;
            k_work_cancel_delayable(&blink_work);
            k_work_cancel_delayable(&update_work);
            k_work_cancel_delayable(&lowbat_work);
            led_show(COLOR_OFF);
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_battery_state_changed *bat = as_zmk_battery_state_changed(eh);
    if (bat) {
        battery_update(bat->state_of_charge);
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
ZMK_SUBSCRIPTION(status_led, zmk_battery_state_changed);

static int status_led_init(void) {
    const struct gpio_dt_spec *leds[] = {&led_red, &led_green, &led_blue};

    for (int i = 0; i < ARRAY_SIZE(leds); i++) {
        if (!gpio_is_ready_dt(leds[i])) {
            LOG_ERR("Status-LED: GPIO nicht bereit");
            return -ENODEV;
        }
        gpio_pin_configure_dt(leds[i], GPIO_OUTPUT_INACTIVE);
    }

    k_work_init_delayable(&blink_work, blink_handler);
    k_work_init_delayable(&update_work, update_handler);
    k_work_init_delayable(&lowbat_work, lowbat_handler);

    /* Nach dem Start kurz warten, bis Bluetooth und die gespeicherten
     * Profile geladen sind, dann den Anfangszustand pruefen. */
    k_work_reschedule(&update_work, K_SECONDS(1));

    return 0;
}

SYS_INIT(status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
