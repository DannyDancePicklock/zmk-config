/*
 * Diagnose-Ausgabe der BT Remote (nur im Diagnose-Build aktiv)
 *
 * Schreibt per USB (PuTTY, COM-Port der Remote) mit, was zwischen Remote
 * und PC passiert: Verbindung auf/ab mit Grund, Verschluesselung,
 * Kopplungsergebnis, Verbindungsparameter und PHY.
 * Aendert nichts am Verhalten der Remote, hoert nur zu.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(diag, LOG_LEVEL_DBG);

static const char *hci_reason_str(uint8_t r) {
    switch (r) {
    case 0x00: return "OK";
    case 0x02: return "unbekannte Verbindung";
    case 0x05: return "Authentifizierung fehlgeschlagen";
    case 0x06: return "Schluessel fehlt (PC hat alte Kopplung?)";
    case 0x08: return "Verbindung abgelaufen (Funkkontakt verloren)";
    case 0x13: return "PC hat getrennt";
    case 0x14: return "PC: zu wenig Ressourcen";
    case 0x15: return "PC schaltet aus";
    case 0x16: return "Remote hat getrennt";
    case 0x1A: return "PC: Funktion nicht unterstuetzt";
    case 0x1E: return "ungueltige LMP/LL-Parameter";
    case 0x1F: return "unbekannter Fehler";
    case 0x22: return "LL-Antwort zu spaet";
    case 0x23: return "LL-Kollision";
    case 0x28: return "Zeitpunkt verpasst";
    case 0x29: return "Kopplung mit Unit Key nicht unterstuetzt";
    case 0x3B: return "Verbindungsparameter abgelehnt";
    case 0x3D: return "Verschluesselung kaputt (MIC)";
    case 0x3E: return "Verbindungsaufbau fehlgeschlagen";
    default: return "?";
    }
}

static const char *sec_err_str(enum bt_security_err e) {
    switch (e) {
    case BT_SECURITY_ERR_SUCCESS: return "OK";
    case BT_SECURITY_ERR_AUTH_FAIL: return "Authentifizierung fehlgeschlagen";
    case BT_SECURITY_ERR_PIN_OR_KEY_MISSING: return "Schluessel fehlt";
    case BT_SECURITY_ERR_OOB_NOT_AVAILABLE: return "OOB nicht verfuegbar";
    case BT_SECURITY_ERR_AUTH_REQUIREMENT: return "Sicherheitsanforderung passt nicht";
    case BT_SECURITY_ERR_PAIR_NOT_SUPPORTED: return "Kopplungsart nicht unterstuetzt";
    case BT_SECURITY_ERR_PAIR_NOT_ALLOWED: return "Kopplung nicht erlaubt";
    case BT_SECURITY_ERR_INVALID_PARAM: return "ungueltige Parameter";
    case BT_SECURITY_ERR_KEY_REJECTED: return "Schluessel abgelehnt";
    case BT_SECURITY_ERR_UNSPECIFIED: return "unbekannt";
    default: return "?";
    }
}

static void addr_str(struct bt_conn *conn, char *buf, size_t len) {
    bt_addr_le_to_str(bt_conn_get_dst(conn), buf, len);
}

static void log_conn_info(struct bt_conn *conn, const char *prefix) {
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) == 0 && info.type == BT_CONN_TYPE_LE) {
        LOG_INF("%s Intervall %u x 1,25 ms, Latenz %u, Timeout %u x 10 ms, Sicherheit %d", prefix,
                info.le.interval, info.le.latency, info.le.timeout, info.security.level);
    }
}

static void diag_connected(struct bt_conn *conn, uint8_t err) {
    char a[BT_ADDR_LE_STR_LEN];
    addr_str(conn, a, sizeof(a));
    if (err) {
        LOG_ERR(">>> VERBINDUNG FEHLGESCHLAGEN mit %s: 0x%02x (%s)", a, err, hci_reason_str(err));
        return;
    }
    LOG_INF(">>> VERBUNDEN mit %s", a);
    log_conn_info(conn, "    ");
}

static void diag_disconnected(struct bt_conn *conn, uint8_t reason) {
    char a[BT_ADDR_LE_STR_LEN];
    addr_str(conn, a, sizeof(a));
    LOG_WRN("<<< GETRENNT von %s: Grund 0x%02x (%s)", a, reason, hci_reason_str(reason));
}

static void diag_security_changed(struct bt_conn *conn, bt_security_t level,
                                  enum bt_security_err err) {
    if (err) {
        LOG_ERR("*** VERSCHLUESSELUNG FEHLGESCHLAGEN: Fehler %d (%s), Stufe %d", err,
                sec_err_str(err), level);
    } else {
        LOG_INF("*** Verschluesselt, Sicherheitsstufe %d", level);
    }
}

static bool diag_le_param_req(struct bt_conn *conn, struct bt_le_conn_param *p) {
    LOG_INF("    PC moechte Parameter: Intervall %u-%u, Latenz %u, Timeout %u", p->interval_min,
            p->interval_max, p->latency, p->timeout);
    return true;
}

static void diag_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                                  uint16_t timeout) {
    LOG_INF("    Neue Parameter: Intervall %u x 1,25 ms, Latenz %u, Timeout %u x 10 ms", interval,
            latency, timeout);
}

#if defined(CONFIG_BT_USER_PHY_UPDATE)
static void diag_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param) {
    LOG_INF("    PHY: senden %u, empfangen %u (1 = 1 Mbit, 2 = 2 Mbit)", param->tx_phy,
            param->rx_phy);
}
#endif

BT_CONN_CB_DEFINE(diag_conn_cb) = {
    .connected = diag_connected,
    .disconnected = diag_disconnected,
    .security_changed = diag_security_changed,
    .le_param_req = diag_le_param_req,
    .le_param_updated = diag_le_param_updated,
#if defined(CONFIG_BT_USER_PHY_UPDATE)
    .le_phy_updated = diag_le_phy_updated,
#endif
};

static void diag_pairing_complete(struct bt_conn *conn, bool bonded) {
    LOG_INF("### KOPPLUNG ERFOLGREICH (%s)", bonded ? "gespeichert" : "NICHT gespeichert");
}

static void diag_pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
    LOG_ERR("### KOPPLUNG FEHLGESCHLAGEN: Fehler %d (%s)", reason, sec_err_str(reason));
}

static void diag_bond_deleted(uint8_t id, const bt_addr_le_t *peer) {
    char a[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(peer, a, sizeof(a));
    LOG_INF("    Kopplung geloescht: %s", a);
}

static struct bt_conn_auth_info_cb diag_auth_info_cb = {
    .pairing_complete = diag_pairing_complete,
    .pairing_failed = diag_pairing_failed,
    .bond_deleted = diag_bond_deleted,
};

/* Herzschlag: zeigt im PuTTY, dass die Remote noch lebt */
static void heartbeat_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(heartbeat, heartbeat_fn);

static void heartbeat_fn(struct k_work *work) {
    LOG_INF("... DIAGNOSE laeuft (%u s)", (uint32_t)(k_uptime_get() / 1000));
    k_work_schedule(&heartbeat, K_SECONDS(30));
}

static int diag_init(void) {
    int err = bt_conn_auth_info_cb_register(&diag_auth_info_cb);
    LOG_INF("=== FRET Remote V1 - DIAGNOSE-VERSION === (auth_info %d)", err);
    k_work_schedule(&heartbeat, K_SECONDS(5));
    return 0;
}

SYS_INIT(diag_init, APPLICATION, 99);
