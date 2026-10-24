/*
 * Kopplungsmodus der BT Remote
 *
 * Normales ZMK ist sichtbar, sobald kein PC gespeichert ist. Hier nicht:
 * - Der Kopplungsmodus startet NUR ueber die Tastenkombi (Behavior
 *   "zmk,behavior-bt-remote-pairing"). Dabei wird der alte PC geloescht.
 * - Play/Pause bricht ihn ab. Die Remote ist dann unsichtbar und bleibt es.
 * - Der Zustand liegt nur im RAM: nach Ausschalten, Reset oder Tiefschlaf
 *   ist der Kopplungsmodus immer aus.
 * - Sobald ein PC erfolgreich gekoppelt ist, endet er automatisch.
 *
 * Ein gekoppelter, aber gerade nicht verbundener PC kann sich weiterhin
 * jederzeit neu verbinden (das regelt ZMK wie gewohnt).
 *
 * Benoetigt patches/zmk-v0.3-pairing-window.patch in ZMK.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

#include <dt-bindings/zmk/hid_usage.h>
#include <dt-bindings/zmk/hid_usage_pages.h>

#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/keycode_state_changed.h>

#include "pairing.h"
#include "blackbox.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* In ZMK ble.c definiert, aber nicht im Header deklariert */
int update_advertising(void);

static bool pairing_window;

/* Ueberschreibt den schwachen Standard aus dem ZMK-Patch:
 * Ein offenes Profil ist nur im Kopplungsmodus sichtbar und koppelbar. */
bool zmk_ble_open_profile_adv_allowed(void) { return pairing_window; }

bool bt_remote_pairing_active(void) { return pairing_window; }

static void notify_led(void) {
#if IS_ENABLED(CONFIG_BT_REMOTE_STATUS_LED)
    bt_remote_status_led_refresh();
#endif
}

void bt_remote_pairing_start(void) {
    LOG_INF("Kopplungsmodus gestartet");

    /* Erst freigeben, dann loeschen: zmk_ble_clear_bonds() startet am Ende
     * selbst das Advertising neu und fragt dabei den Hook oben ab. */
    pairing_window = true;
    bt_remote_blackbox_pairing(BB_PAIRWIN_START);
    zmk_ble_clear_bonds();

    notify_led();
}

static void disconnect_peripheral_conn(struct bt_conn *conn, void *data) {
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) == 0 && info.role == BT_CONN_ROLE_PERIPHERAL &&
        info.state == BT_CONN_STATE_CONNECTED) {
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
}

void bt_remote_pairing_cancel(void) {
    if (!pairing_window) {
        return;
    }

    LOG_INF("Kopplungsmodus abgebrochen");
    pairing_window = false;
    bt_remote_blackbox_pairing(BB_PAIRWIN_CANCEL);

    if (zmk_ble_active_profile_is_open()) {
        /* Ein PC, der gerade mitten im Koppeln steckt, wird getrennt */
        bt_conn_foreach(BT_CONN_TYPE_LE, disconnect_peripheral_conn, NULL);
    }

    /* Advertising stoppen (offenes Profil ist jetzt nicht mehr erlaubt) */
    update_advertising();

    notify_led();
}

static int pairing_listener(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *kc = as_zmk_keycode_state_changed(eh);
    if (kc) {
        /* Play/Pause gedrueckt waehrend des Kopplungsmodus: abbrechen */
        if (kc->state && pairing_window && kc->usage_page == HID_USAGE_CONSUMER &&
            kc->keycode == HID_USAGE_CONSUMER_PLAY_PAUSE) {
            bt_remote_pairing_cancel();
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (as_zmk_ble_active_profile_changed(eh)) {
        /* Profil ist nicht mehr offen = ein PC wurde erfolgreich gekoppelt */
        if (pairing_window && !zmk_ble_active_profile_is_open()) {
            LOG_INF("Kopplung erfolgreich, Kopplungsmodus beendet");
            pairing_window = false;
            bt_remote_blackbox_pairing(BB_PAIRWIN_DONE);
            notify_led();
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(bt_remote_pairing, pairing_listener);
ZMK_SUBSCRIPTION(bt_remote_pairing, zmk_keycode_state_changed);
ZMK_SUBSCRIPTION(bt_remote_pairing, zmk_ble_active_profile_changed);
