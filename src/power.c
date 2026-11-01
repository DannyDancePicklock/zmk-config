/*
 * Ein/Aus per Tastenkombi - ersetzt den Schiebeschalter
 *
 * Aus: Play + Unten 3 s halten (Hold-Tap in der Keymap). Die LED macht
 *      rot blink, blink, ausfaden. Sobald Play losgelassen ist, geht die
 *      Remote in den Tiefschlaf (ZMK "Soft Off", nur ein paar Mikroampere).
 *      Nur Play kann sie wieder wecken (soft_off_wakers im Overlay).
 *
 * Ein: Play 3 s halten -> 2x gruen, dann startet die Remote normal.
 *      Kurzes Antippen (z. B. in der Tasche) weckt den Chip nur fuer einen
 *      Augenblick, er geht sofort wieder aus, ohne Bluetooth zu starten.
 *      USB einstecken schaltet sie ebenfalls ein.
 *
 * Woher weiss die Firmware nach dem Aufwachen, dass sie "aus" war?
 * Vor dem Ausschalten wird ein Merker in GPREGRET2 geschrieben. Das
 * Register uebersteht den Tiefschlaf, aber nicht das Abstecken des Akkus
 * (dann startet sie ganz normal). Der automatische Schlaf nach 2 h Ruhe
 * setzt den Merker nicht - da weckt wie bisher jede Taste sofort.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_power.h>

#include <zmk/pm.h>

#include "power.h"
#if IS_ENABLED(CONFIG_BT_REMOTE_STATUS_LED)
#include "status_led.h"
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define POWER_OFF_MAGIC 0xA5
#define POWER_ON_HOLD_MS 3000
#define POLL_MS 20
#define RELEASE_STABLE_MS 100
#define RELEASE_TIMEOUT_MS 10000

/* Die Play-Taste als Ein-Taste (Knoten power_key im Overlay) */
static const struct gpio_dt_spec power_key = GPIO_DT_SPEC_GET(DT_NODELABEL(power_key), gpios);

static bool key_pressed(void) { return gpio_pin_get_dt(&power_key) > 0; }

static bool usb_plugged(void) {
    return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

/* Warten, bis Play sicher losgelassen ist (mit Zeitlimit) */
static void wait_for_release(void) {
    int stable = 0;

    for (int t = 0; t < RELEASE_TIMEOUT_MS && stable < RELEASE_STABLE_MS; t += POLL_MS) {
        k_msleep(POLL_MS);
        stable = key_pressed() ? 0 : stable + POLL_MS;
    }
}

static void announce_on(void) {
    LOG_INF("Eingeschaltet");
#if IS_ENABLED(CONFIG_BT_REMOTE_STATUS_LED)
    bt_remote_status_led_power_on_blink();
#endif
}

/* Frueh beim Start: Bluetooth, USB und Tasten laufen noch nicht. Wenn
 * gleich wieder ausgeschaltet wird, bleibt alles aus. */
FUNC_NORETURN static void stay_off(void) {
    NRF_POWER->GPREGRET2 = POWER_OFF_MAGIC;
    /* Nur Play darf wecken (Pegel-Interrupt -> SENSE im Tiefschlaf) */
    gpio_pin_interrupt_configure_dt(&power_key, GPIO_INT_LEVEL_ACTIVE);
    sys_poweroff();
}

static int power_boot_check(void) {
    bool from_off = (NRF_POWER->GPREGRET2 == POWER_OFF_MAGIC);

    NRF_POWER->GPREGRET2 = 0;
    if (!from_off || !gpio_is_ready_dt(&power_key)) {
        return 0;
    }

    gpio_pin_configure_dt(&power_key, GPIO_INPUT);
    k_busy_wait(200); /* Pull-up einschwingen lassen */

    if (!key_pressed()) {
        if (usb_plugged()) {
            /* Aufgewacht durchs USB-Kabel: einschalten */
            announce_on();
            return 0;
        }
        /* Nur kurz angetippt: weiter aus */
        stay_off();
    }

    /* Play muss 3 s gehalten werden */
    for (int t = 0; t < POWER_ON_HOLD_MS; t += POLL_MS) {
        k_msleep(POLL_MS);
        if (!key_pressed()) {
            stay_off();
        }
    }

    announce_on();

    /* Erst weiterstarten, wenn Play los ist - sonst geht beim Loslassen
     * ein Play/Pause an den PC */
    wait_for_release();
    return 0;
}

/* Vor dem externen Flash-Speicher (Prio 80) pruefen, damit der beim
 * sofortigen Wieder-Ausschalten im Stromsparmodus bleibt */
SYS_INIT(power_boot_check, POST_KERNEL, 45);

void bt_remote_power_off(void) {
    LOG_INF("Ausschalten");

#if IS_ENABLED(CONFIG_BT_REMOTE_STATUS_LED)
    bt_remote_status_led_power_off_animation();
#endif

    /* Solange Play gedrueckt ist, wuerde die Remote sofort wieder aufwachen */
    wait_for_release();

    NRF_POWER->GPREGRET2 = POWER_OFF_MAGIC;
    int err = zmk_pm_soft_off();

    /* Hier kommt man nur an, wenn das Ausschalten fehlgeschlagen ist */
    NRF_POWER->GPREGRET2 = 0;
    LOG_ERR("Ausschalten fehlgeschlagen (%d)", err);
#if IS_ENABLED(CONFIG_BT_REMOTE_STATUS_LED)
    bt_remote_status_led_unlock();
#endif
}
