/*
 * RGB-Status-LED fuer die BT Remote
 *
 * - Kopplungsmodus: blau, schnelles Blinken. Mit CONFIG_BT_REMOTE_PAIRING_WINDOW
 *   nur waehrend des per Kombi gestarteten Kopplungsmodus (siehe pairing.c),
 *   sonst immer, wenn kein PC gespeichert ist.
 * - Verbindung hergestellt: gruen, einmal kurz aufleuchten
 * - Akku schwach: rot, kurzer Blitz in regelmaessigen Abstaenden
 * - Sonst (verbunden, getrennt oder Schlaf): aus
 *
 * - Laden (nur Boards mit Ladeanzeige, z. B. XIAO nRF52840): Solange USB
 *   steckt, leuchtet rot dauerhaft, waehrend der Akku geladen wird, und geht
 *   aus, wenn er voll ist. Die Akku-Warnung ist am Kabel stumm.
 *
 * Prioritaet: Kopplungsmodus > Verbindungsblitz > Laden > Akku-Warnung
 *
 * Testmodus (CONFIG_BT_REMOTE_STATUS_LED_DEBUG): Solange nicht verbunden,
 * leuchtet rot dauerhaft. Beim Verbinden kurz gruen, dann aus. Mit
 * Ladeanzeige gilt das nur im Akkubetrieb, am Kabel zeigt rot das Laden.
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

#include "pairing.h"
#include "status_led.h"

/* Ladeanzeige nur, wenn das Board einen CHG-Pin im Devicetree hat
 * (zephyr,user { charge-gpios = ... }) und USB in ZMK aktiv ist. */
#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), charge_gpios) && IS_ENABLED(CONFIG_ZMK_USB)
#define HAS_CHARGE_STATUS 1
#include <zmk/usb.h>
#include <zmk/events/usb_conn_state_changed.h>
#else
#define HAS_CHARGE_STATUS 0
#endif

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
static bool leds_locked;

static struct k_work_delayable blink_work;
static struct k_work_delayable update_work;
static struct k_work_delayable lowbat_work;

#if HAS_CHARGE_STATUS
/* Abfrage-Takt des CHG-Pins, solange USB steckt */
#define CHARGE_POLL_MS 2000

static const struct gpio_dt_spec charge_pin = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), charge_gpios);
static bool usb_powered;
static bool charging;

/* Statisch angelegt, damit ein fruehes USB-Ereignis nie auf eine
 * unfertige Arbeit trifft */
static void charge_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(charge_work, charge_handler);
#endif

/* Haengt die Remote gerade am USB-Kabel? (nur mit Ladeanzeige relevant) */
static bool on_usb_power(void) {
#if HAS_CHARGE_STATUS
    return usb_powered;
#else
    return false;
#endif
}

/* Farbe, wenn gerade nichts blinkt:
 * - am USB-Kabel (mit Ladeanzeige): rot = laedt, aus = voll
 * - im Testmodus: rot, solange nicht verbunden */
static enum led_color idle_color(void) {
#if HAS_CHARGE_STATUS
    if (usb_powered) {
        return charging ? COLOR_RED : COLOR_OFF;
    }
#endif
#if IS_ENABLED(CONFIG_BT_REMOTE_STATUS_LED_DEBUG)
    if (!zmk_ble_active_profile_is_connected()) {
        return COLOR_RED;
    }
#endif
    return COLOR_OFF;
}

/* Direkt schalten, ohne Ruecksicht auf die Sperre (nur fuer Animationen) */
static void led_raw(enum led_color color) {
    gpio_pin_set_dt(&led_red, (color & COLOR_RED) != 0);
    gpio_pin_set_dt(&led_green, (color & COLOR_GREEN) != 0);
    gpio_pin_set_dt(&led_blue, (color & COLOR_BLUE) != 0);
}

static void led_show(enum led_color color) {
    /* Waehrend der Ausschalt-Animation gehoert die LED nur ihr */
    if (leds_locked) {
        return;
    }
    led_raw(color);
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
    led_show(idle_color());
}

/* Akku-Warnung: regelmaessig kurz rot blitzen, solange der Akku schwach ist */
static void lowbat_handler(struct k_work *work) {
    if (!battery_low) {
        return;
    }
    /* Am Ladekabel keine Warnung, da zeigt rot schon das Laden */
    if (!pairing_active && !flash_active && !on_usb_power()) {
        flash(COLOR_RED, LOWBAT_FLASH_MS);
    }
    k_work_reschedule(&lowbat_work, K_MSEC(LOWBAT_INTERVAL_MS));
}

#if HAS_CHARGE_STATUS
/* Ruhefarbe neu setzen, wenn gerade nichts blinkt */
static void apply_idle(void) {
    if (!pairing_active && !flash_active) {
        led_show(idle_color());
    }
}

/* Ladezustand pruefen: Nur solange USB steckt, wird der CHG-Pin gelesen
 * (alle 2 s). Ohne Kabel ist der Pin abgeklemmt und kostet nichts. */
static void charge_handler(struct k_work *work) {
    bool powered = zmk_usb_is_powered();
    bool now_charging = false;

    if (powered) {
        if (!usb_powered) {
            gpio_pin_configure_dt(&charge_pin, GPIO_INPUT);
        }
        now_charging = gpio_pin_get_dt(&charge_pin) > 0;
        k_work_reschedule(&charge_work, K_MSEC(CHARGE_POLL_MS));
    } else if (usb_powered) {
        gpio_pin_configure_dt(&charge_pin, GPIO_DISCONNECTED);
    }

    if (powered != usb_powered || now_charging != charging) {
        LOG_INF("Status-LED: USB %s, %s", powered ? "steckt" : "ab",
                now_charging ? "laedt" : "laedt nicht");
        usb_powered = powered;
        charging = now_charging;
        apply_idle();
    }
}
#endif

/* Aktuellen Bluetooth-Zustand auswerten und LED-Modus setzen */
static void update_handler(struct k_work *work) {
    bool connected = zmk_ble_active_profile_is_connected();
#if IS_ENABLED(CONFIG_BT_REMOTE_PAIRING_WINDOW)
    bool open = zmk_ble_active_profile_is_open() && bt_remote_pairing_active();
#else
    bool open = zmk_ble_active_profile_is_open();
#endif

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
        /* Getrennt, gekoppelt oder Kopplungsmodus abgebrochen: LED aus */
        pairing_active = false;
        flash_active = false;
        k_work_cancel_delayable(&blink_work);
        led_show(idle_color());
    }

    was_connected = connected;
}

/* ---- Ein/Aus-Animationen (siehe power.c) ---------------------------------- */

#define ANIM_BLINK_ON_MS 150
#define ANIM_BLINK_OFF_MS 150

/* Ausfaden per Software-PWM: 1 kHz, Helligkeit quadratisch runter, damit es
 * fuers Auge gleichmaessig dunkler wird. Blockiert ca. 0,8 s - danach geht
 * die Remote ohnehin aus. */
#define FADE_STEPS 40
#define FADE_STEP_MS 20
#define FADE_PERIOD_US 1000

static void fade_out(const struct gpio_dt_spec *led) {
    for (int s = FADE_STEPS; s > 0; s--) {
        uint32_t on_us = (uint32_t)FADE_PERIOD_US * s * s / (FADE_STEPS * FADE_STEPS);

        for (int p = 0; p < FADE_STEP_MS * 1000 / FADE_PERIOD_US; p++) {
            gpio_pin_set_dt(led, 1);
            k_busy_wait(on_us);
            gpio_pin_set_dt(led, 0);
            k_busy_wait(FADE_PERIOD_US - on_us);
        }
    }
}

static void blink_raw(enum led_color color, int count) {
    for (int i = 0; i < count; i++) {
        led_raw(color);
        k_msleep(ANIM_BLINK_ON_MS);
        led_raw(COLOR_OFF);
        k_msleep(ANIM_BLINK_OFF_MS);
    }
}

/* Ausschalten: rot blink, blink, ausfaden. Danach bleibt die LED gesperrt
 * und aus, bis bt_remote_status_led_unlock() kommt (nur wenn das
 * Ausschalten schiefgeht). */
void bt_remote_status_led_power_off_animation(void) {
    leds_locked = true;
    pairing_active = false;
    flash_active = false;
    k_work_cancel_delayable(&blink_work);
    k_work_cancel_delayable(&update_work);
    k_work_cancel_delayable(&lowbat_work);
#if HAS_CHARGE_STATUS
    k_work_cancel_delayable(&charge_work);
#endif

    led_raw(COLOR_OFF);
    k_msleep(ANIM_BLINK_OFF_MS);
    blink_raw(COLOR_RED, 2);
    fade_out(&led_red);
    led_raw(COLOR_OFF);
}

void bt_remote_status_led_unlock(void) {
    leds_locked = false;
    k_work_reschedule(&update_work, K_NO_WAIT);
#if HAS_CHARGE_STATUS
    k_work_reschedule(&charge_work, K_NO_WAIT);
#endif
}

/* Einschalten: 2x gruen. Laeuft ganz frueh beim Start (vor status_led_init),
 * deshalb werden die Pins hier selbst eingerichtet. */
void bt_remote_status_led_power_on_blink(void) {
    const struct gpio_dt_spec *leds[] = {&led_red, &led_green, &led_blue};

    for (int i = 0; i < ARRAY_SIZE(leds); i++) {
        if (!gpio_is_ready_dt(leds[i])) {
            return;
        }
        gpio_pin_configure_dt(leds[i], GPIO_OUTPUT_INACTIVE);
    }
    blink_raw(COLOR_GREEN, 2);
}

/* Von blackbox.c aufgerufen: Markierung gesetzt -> einmal gelb blitzen */
void bt_remote_status_led_marker(void) { flash(COLOR_RED | COLOR_GREEN, 600); }

/* Von pairing.c aufgerufen, wenn der Kopplungsmodus startet oder endet */
void bt_remote_status_led_refresh(void) { k_work_reschedule(&update_work, K_NO_WAIT); }

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
#if HAS_CHARGE_STATUS
            k_work_cancel_delayable(&charge_work);
#endif
            led_show(COLOR_OFF);
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_battery_state_changed *bat = as_zmk_battery_state_changed(eh);
    if (bat) {
        battery_update(bat->state_of_charge);
        return ZMK_EV_EVENT_BUBBLE;
    }

#if HAS_CHARGE_STATUS
    if (as_zmk_usb_conn_state_changed(eh)) {
        /* Kabel rein oder raus: Ladezustand sofort neu pruefen */
        k_work_reschedule(&charge_work, K_NO_WAIT);
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    if (as_zmk_ble_active_profile_changed(eh)) {
        k_work_reschedule(&update_work, K_NO_WAIT);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(status_led, status_led_listener);
ZMK_SUBSCRIPTION(status_led, zmk_ble_active_profile_changed);
ZMK_SUBSCRIPTION(status_led, zmk_activity_state_changed);
ZMK_SUBSCRIPTION(status_led, zmk_battery_state_changed);
#if HAS_CHARGE_STATUS
ZMK_SUBSCRIPTION(status_led, zmk_usb_conn_state_changed);
#endif

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

#if HAS_CHARGE_STATUS
    if (!gpio_is_ready_dt(&charge_pin)) {
        LOG_ERR("Status-LED: Lade-Pin nicht bereit");
    } else {
        /* Erst mal abgeklemmt, gelesen wird nur am USB-Kabel */
        gpio_pin_configure_dt(&charge_pin, GPIO_DISCONNECTED);
        k_work_reschedule(&charge_work, K_SECONDS(1));
    }
#endif

    return 0;
}

SYS_INIT(status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
