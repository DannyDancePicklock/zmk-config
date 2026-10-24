/*
 * Blackbox der BT Remote (nur im BLACKBOX-Build aktiv)
 *
 * Schreibt ohne USB-Kabel still mit, was passiert, und speichert es im
 * Flash der Remote (eigener 128-KB-Bereich, ca. 4000 Eintraege, danach
 * werden die aeltesten ueberschrieben). Ueberlebt Ausschalten, Tiefschlaf
 * und Akku leer.
 *
 * Aufgezeichnet wird:
 *   - jeder Start mit Grund (Strom an, Aufwachen, Reset ...) und Taktquelle
 *   - Bluetooth: Verbinden, Trennen (mit Grund und Dauer), Verschluesselung,
 *     Kopplung, Verbindungsparameter, PHY
 *   - Kopplungsmodus an/aus, aktives Profil, USB-Strom, Ausgang BLE/USB
 *   - jeder Tastendruck (mit "war verbunden?"), Raddrehungen zusammengefasst
 *   - Akkustand, Ruhe/Tiefschlaf, alle 5 Minuten ein Lebenszeichen
 *   - Markierung: Lauter + Leiser gleichzeitig druecken (LED blitzt gelb)
 *
 * Auslesen: Remote per USB anstecken, PuTTY auf den COM-Port oeffnen.
 * Sobald PuTTY verbunden ist, gibt die Remote nach ca. 2 s den kompletten
 * Inhalt aus. Erneut ausgeben: PuTTY schliessen und wieder oeffnen.
 *
 * Loeschen: passiert automatisch beim ersten Start nach der Reset-Datei
 * (settings_reset), also bei jedem frischen Test-Durchgang.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <soc.h>

#include <dt-bindings/zmk/hid_usage.h>
#include <dt-bindings/zmk/hid_usage_pages.h>

#include <zmk/ble.h>
#include <zmk/battery.h>
#include <zmk/endpoints.h>
#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/ble_active_profile_changed.h>

#include "blackbox.h"

LOG_MODULE_REGISTER(blackbox, LOG_LEVEL_INF);

#if IS_ENABLED(CONFIG_BT_REMOTE_STATUS_LED)
void bt_remote_status_led_marker(void);
#endif

/* ------------------------------------------------------------------------ */
/* Speicherformat                                                            */
/* ------------------------------------------------------------------------ */

#define BB_MAGIC 0xB1AC
#define BB_VER 1
#define BB_PAGE 4096
#define BB_REC_SIZE 32
#define BB_SIZE FIXED_PARTITION_SIZE(blackbox_partition)
#define BB_COUNT (BB_SIZE / BB_REC_SIZE)
#define BB_PER_PAGE (BB_PAGE / BB_REC_SIZE)

#define BB_HEARTBEAT_MS (5 * 60 * 1000)

enum bb_type {
    BB_BOOT = 1,
    BB_CLOCK,
    BB_CONN_OK,
    BB_CONN_FAIL,
    BB_DISC,
    BB_SEC_OK,
    BB_SEC_FAIL,
    BB_PAIR_OK,
    BB_PAIR_FAIL,
    BB_BOND_DEL,
    BB_PARAM_REQ,
    BB_PARAM_UPD,
    BB_PHY,
    BB_KEY,
    BB_WHEEL,
    BB_MARKER,
    BB_BATTERY,
    BB_ACTIVITY,
    BB_USB,
    BB_ENDPOINT,
    BB_PROFILE,
    BB_PAIRWIN,
    BB_HEART,
    BB_DROPPED,
};

struct bb_rec {
    uint32_t seq;  /* fortlaufende Nummer, 0xFFFFFFFF = leer */
    uint32_t t_ms; /* Zeit seit Start in ms */
    uint16_t boot; /* Start-Nummer */
    uint8_t type;
    uint8_t a;
    uint16_t b;
    uint16_t c;
    uint16_t d;
    uint16_t e;
    uint8_t addr[6];
    uint8_t addr_type;
    uint8_t ver;
    uint16_t magic;
    uint16_t reserved;
} __packed __aligned(4);

BUILD_ASSERT(sizeof(struct bb_rec) == BB_REC_SIZE, "Blackbox-Eintrag muss 32 Byte haben");
BUILD_ASSERT(BB_SIZE % BB_PAGE == 0, "Blackbox-Bereich muss aus ganzen Seiten bestehen");

K_MSGQ_DEFINE(bb_q, sizeof(struct bb_rec), 96, 4);
K_MUTEX_DEFINE(bb_lock);

static const struct flash_area *bb_fa;
static bool bb_ready;
static uint32_t bb_idx;
static uint32_t bb_seq;
static uint16_t bb_boot;
static atomic_t bb_dropped;

/* ------------------------------------------------------------------------ */
/* Ereignisse einsammeln (aus beliebigen Threads, blockiert nie)              */
/* ------------------------------------------------------------------------ */

static void bb_put(uint8_t type, uint8_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e,
                   const bt_addr_le_t *addr) {
    struct bb_rec r;

    memset(&r, 0, sizeof(r));
    r.t_ms = k_uptime_get_32();
    r.type = type;
    r.a = a;
    r.b = b;
    r.c = c;
    r.d = d;
    r.e = e;
    if (addr) {
        memcpy(r.addr, addr->a.val, sizeof(r.addr));
        r.addr_type = addr->type;
    }

    if (k_msgq_put(&bb_q, &r, K_NO_WAIT) != 0) {
        atomic_inc(&bb_dropped);
    }
}

/* ------------------------------------------------------------------------ */
/* Flash                                                                      */
/* ------------------------------------------------------------------------ */

static bool rec_valid(const struct bb_rec *r) {
    return r->magic == BB_MAGIC && r->ver == BB_VER && r->seq != 0xFFFFFFFF;
}

static bool rec_blank(const struct bb_rec *r) {
    const uint8_t *p = (const uint8_t *)r;
    for (int i = 0; i < sizeof(*r); i++) {
        if (p[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

static int rec_read(uint32_t idx, struct bb_rec *r) {
    return flash_area_read(bb_fa, idx * BB_REC_SIZE, r, sizeof(*r));
}

/* Schreibt einen Eintrag an die naechste freie Stelle (bb_lock gehalten) */
static void bb_store(struct bb_rec *r) {
    struct bb_rec cur;

    if (!bb_ready) {
        return;
    }

    if (bb_idx % BB_PER_PAGE == 0) {
        /* Neue Seite: vorher loeschen (hier fallen die aeltesten Eintraege weg) */
        flash_area_erase(bb_fa, bb_idx * BB_REC_SIZE, BB_PAGE);
    } else if (rec_read(bb_idx, &cur) == 0 && !rec_blank(&cur)) {
        /* Sollte nie passieren (z. B. Strom weg mitten im Schreiben):
         * Seite neu beginnen statt ueber alte Daten zu schreiben */
        bb_idx -= bb_idx % BB_PER_PAGE;
        flash_area_erase(bb_fa, bb_idx * BB_REC_SIZE, BB_PAGE);
    }

    r->seq = bb_seq++;
    r->boot = bb_boot;
    r->ver = BB_VER;
    r->magic = BB_MAGIC;
    r->reserved = 0xFFFF;

    if (flash_area_write(bb_fa, bb_idx * BB_REC_SIZE, r, sizeof(*r)) != 0) {
        LOG_ERR("Blackbox: Schreiben fehlgeschlagen");
    }
    bb_idx = (bb_idx + 1) % BB_COUNT;
}

/* Einen Eintrag schreiben, vorher ggf. "verloren"-Hinweis (bb_lock gehalten) */
static void bb_store_one(struct bb_rec *r) {
    atomic_val_t lost = atomic_set(&bb_dropped, 0);
    if (lost > 0) {
        struct bb_rec d;
        memset(&d, 0, sizeof(d));
        d.t_ms = r->t_ms;
        d.type = BB_DROPPED;
        d.b = (uint16_t)MIN(lost, 0xFFFF);
        bb_store(&d);
    }
    bb_store(r);
}

/* Alles aus der Warteschlange in den Flash schreiben (bb_lock gehalten) */
static void bb_flush_locked(void) {
    struct bb_rec r;

    while (k_msgq_get(&bb_q, &r, K_NO_WAIT) == 0) {
        bb_store_one(&r);
    }
}

/* Letzte Schreibposition suchen */
static void bb_scan(void) {
    struct bb_rec r;
    uint32_t max_seq = 0;
    int32_t max_idx = -1;
    uint16_t max_boot = 0;

    for (uint32_t i = 0; i < BB_COUNT; i++) {
        if (rec_read(i, &r) == 0 && rec_valid(&r) && r.seq >= max_seq) {
            max_seq = r.seq;
            max_idx = i;
            max_boot = r.boot;
        }
    }

    if (max_idx < 0) {
        bb_idx = 0;
        bb_seq = 1;
        bb_boot = 1;
    } else {
        bb_idx = (max_idx + 1) % BB_COUNT;
        bb_seq = max_seq + 1;
        bb_boot = max_boot + 1;
    }
}

/* ------------------------------------------------------------------------ */
/* Neuer Test-Durchgang? (nach der Reset-Datei fehlt unser Merker)           */
/* ------------------------------------------------------------------------ */

static int session_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
                      void *param) {
    *(bool *)param = true;
    return 0;
}

static bool session_known(void) {
    bool found = false;
    settings_load_subtree_direct("bbox/s", session_cb, &found);
    return found;
}

/* ------------------------------------------------------------------------ */
/* Ausgabe in PuTTY                                                           */
/* ------------------------------------------------------------------------ */

static const char *hci_reason(uint8_t r) {
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
    case 0x1E: return "ungueltige LL-Parameter";
    case 0x1F: return "unbekannter Fehler";
    case 0x22: return "LL-Antwort zu spaet";
    case 0x23: return "LL-Kollision";
    case 0x28: return "Zeitpunkt verpasst";
    case 0x3B: return "Verbindungsparameter abgelehnt";
    case 0x3D: return "Verschluesselung kaputt (MIC)";
    case 0x3E: return "Verbindungsaufbau fehlgeschlagen";
    default: return "?";
    }
}

static const char *sec_err(uint8_t e) {
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

static const char *key_name(uint8_t k) {
    switch (k) {
    case 1: return "Play/Pause";
    case 2: return "Zurueck";
    case 3: return "Weiter";
    case 4: return "Lauter";
    case 5: return "Leiser";
    default: return "andere";
    }
}

static const char *ja_nein(int v) { return v ? "ja" : "nein"; }

static void fmt_addr(const struct bb_rec *r, char *buf, size_t len) {
    snprintk(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X (%s)", r->addr[5], r->addr[4], r->addr[3],
             r->addr[2], r->addr[1], r->addr[0], r->addr_type ? "zufaellig" : "fest");
}

static void fmt_reset(uint32_t rr, char *buf, size_t len) {
    if (rr == 0) {
        snprintk(buf, len, "Strom an / Akku eingesteckt");
        return;
    }
    int n = snprintk(buf, len, "%s%s%s%s%s%s", (rr & BIT(0)) ? "Reset-Taster " : "",
                     (rr & BIT(1)) ? "Watchdog " : "", (rr & BIT(2)) ? "Software-Reset " : "",
                     (rr & BIT(3)) ? "ABSTURZ(Lockup) " : "",
                     (rr & BIT(16)) ? "Aufwachen-aus-Tiefschlaf " : "",
                     (rr & BIT(20)) ? "USB-angesteckt " : "");
    snprintk(buf + n, len - n, "[0x%08x]", rr);
}

static const char *lfclk_src(uint8_t s) {
    switch (s & 0x3) {
    case 0: return "RC (intern, ungenau)";
    case 1: return "Quarz";
    case 2: return "aus HF-Takt erzeugt";
    default: return "?";
    }
}

/* Warten, bis PuTTY hinterherkommt, damit nichts verloren geht */
static void bb_pace(void) {
#if IS_ENABLED(CONFIG_LOG) && !IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE)
    for (int i = 0; i < 250 && log_buffered_cnt() > 2; i++) {
        k_msleep(2);
    }
#endif
    k_msleep(8);
}

static void print_rec(const struct bb_rec *r) {
    char t[24], addr[40], extra[96];
    uint32_t ms = r->t_ms;

    snprintk(t, sizeof(t), "%u:%02u:%02u.%03u", ms / 3600000, (ms / 60000) % 60, (ms / 1000) % 60,
             ms % 1000);
    fmt_addr(r, addr, sizeof(addr));

    switch (r->type) {
    case BB_BOOT:
        fmt_reset(((uint32_t)r->c << 16) | r->b, extra, sizeof(extra));
        printk("\n[S%u %s] ======== START #%u ======== Grund: %s | USB-Strom: %s\n", r->boot, t,
               r->boot, extra, ja_nein(r->a));
        break;
    case BB_CLOCK:
        printk("[S%u %s] Takt 32 kHz: %s, %s (angefordert: %s) | Takt 64 MHz: %s, %s\n", r->boot,
               t, lfclk_src(r->d), (r->d & 0x100) ? "laeuft" : "STEHT", lfclk_src(r->a),
               (r->e & 1) ? "Quarz" : "RC", (r->e & 0x100) ? "laeuft" : "aus");
        break;
    case BB_CONN_OK:
        printk("[S%u %s] >>> VERBUNDEN mit %s | Intervall %u (%u ms), Latenz %u, Timeout %u ms\n",
               r->boot, t, addr, r->b, r->b * 5 / 4, r->c, r->d * 10);
        break;
    case BB_CONN_FAIL:
        printk("[S%u %s] >>> VERBINDUNG FEHLGESCHLAGEN mit %s: 0x%02x (%s)\n", r->boot, t, addr,
               r->a, hci_reason(r->a));
        break;
    case BB_DISC: {
        uint32_t dur = ((uint32_t)r->c << 16) | r->b;
        printk("[S%u %s] <<< GETRENNT von %s nach %u.%03u s: 0x%02x (%s)\n", r->boot, t, addr,
               dur / 1000, dur % 1000, r->a, hci_reason(r->a));
        break;
    }
    case BB_SEC_OK:
        printk("[S%u %s] *** Verschluesselt, Stufe %u\n", r->boot, t, r->a);
        break;
    case BB_SEC_FAIL:
        printk("[S%u %s] *** VERSCHLUESSELUNG FEHLGESCHLAGEN: %u (%s), Stufe %u\n", r->boot, t,
               r->a, sec_err(r->a), r->b);
        break;
    case BB_PAIR_OK:
        printk("[S%u %s] ### KOPPLUNG ERFOLGREICH (%s)\n", r->boot, t,
               r->a ? "gespeichert" : "NICHT gespeichert");
        break;
    case BB_PAIR_FAIL:
        printk("[S%u %s] ### KOPPLUNG FEHLGESCHLAGEN: %u (%s)\n", r->boot, t, r->a, sec_err(r->a));
        break;
    case BB_BOND_DEL:
        printk("[S%u %s]     Kopplung geloescht: %s\n", r->boot, t, addr);
        break;
    case BB_PARAM_REQ:
        printk("[S%u %s]     PC moechte: Intervall %u-%u, Latenz %u, Timeout %u ms\n", r->boot, t,
               r->b, r->c, r->d, r->e * 10);
        break;
    case BB_PARAM_UPD:
        printk("[S%u %s]     Neue Parameter: Intervall %u (%u ms), Latenz %u, Timeout %u ms\n",
               r->boot, t, r->b, r->b * 5 / 4, r->c, r->d * 10);
        break;
    case BB_PHY:
        printk("[S%u %s]     PHY: senden %u, empfangen %u (1 = 1 Mbit, 2 = 2 Mbit)\n", r->boot, t,
               r->a, r->b);
        break;
    case BB_KEY:
        printk("[S%u %s] Taste %s | verbunden: %s | Ausgang: %s\n", r->boot, t, key_name(r->a),
               ja_nein(r->b), r->c == ZMK_TRANSPORT_USB ? "USB" : "Bluetooth");
        break;
    case BB_WHEEL:
        printk("[S%u %s] Rad: %u x Plus, %u x Minus | verbunden: %s\n", r->boot, t, r->b, r->c,
               ja_nein(r->d));
        break;
    case BB_MARKER:
        printk("[S%u %s] !!!!!!!!!!!!!!!! MARKIERUNG (Lauter + Leiser) !!!!!!!!!!!!!!!!\n",
               r->boot, t);
        break;
    case BB_BATTERY:
        printk("[S%u %s] Akku: %u %%\n", r->boot, t, r->a);
        break;
    case BB_ACTIVITY:
        printk("[S%u %s] Zustand: %s\n", r->boot, t,
               r->a == ZMK_ACTIVITY_ACTIVE ? "aktiv"
               : r->a == ZMK_ACTIVITY_IDLE ? "Ruhe"
                                           : "TIEFSCHLAF (geht aus)");
        break;
    case BB_USB:
        printk("[S%u %s] USB: %s\n", r->boot, t,
               r->a == ZMK_USB_CONN_NONE      ? "abgesteckt"
               : r->a == ZMK_USB_CONN_POWERED ? "nur Strom"
                                              : "verbunden (PC erkennt Remote)");
        break;
    case BB_ENDPOINT:
        printk("[S%u %s] Tasten gehen jetzt an: %s\n", r->boot, t,
               r->a == ZMK_TRANSPORT_USB ? "USB" : "Bluetooth");
        break;
    case BB_PROFILE:
        printk("[S%u %s] Profil %u: verbunden %s, ohne PC (offen) %s\n", r->boot, t, r->a,
               ja_nein(r->b), ja_nein(r->c));
        break;
    case BB_PAIRWIN:
        printk("[S%u %s] Kopplungsmodus: %s\n", r->boot, t,
               r->a == BB_PAIRWIN_START    ? "GESTARTET (alter PC geloescht)"
               : r->a == BB_PAIRWIN_CANCEL ? "abgebrochen"
                                           : "beendet, PC gekoppelt");
        break;
    case BB_HEART:
        printk("[S%u %s] ... lebt | verbunden: %s | Akku %u %%\n", r->boot, t, ja_nein(r->a),
               r->b);
        break;
    case BB_DROPPED:
        printk("[S%u %s] (%u Eintraege verloren, Warteschlange war voll)\n", r->boot, t, r->b);
        break;
    default:
        printk("[S%u %s] ? Typ %u\n", r->boot, t, r->type);
        break;
    }
}

static void bb_dump(void) {
    struct bb_rec r;
    uint32_t min_seq = UINT32_MAX, count = 0;
    int32_t min_idx = -1;

    k_mutex_lock(&bb_lock, K_FOREVER);
    bb_flush_locked();

    for (uint32_t i = 0; i < BB_COUNT; i++) {
        if (rec_read(i, &r) == 0 && rec_valid(&r)) {
            count++;
            if (r.seq < min_seq) {
                min_seq = r.seq;
                min_idx = i;
            }
        }
    }

    printk("\n\n================ BLACKBOX START ================\n");
    printk("Eintraege: %u von max. %u | aktueller Start: #%u | jetzt: %u s seit Start\n", count,
           BB_COUNT, bb_boot, k_uptime_get_32() / 1000);
    printk("Format: [S<Start-Nr> <Std:Min:Sek seit diesem Start>]\n");
    bb_pace();

    if (min_idx >= 0) {
        for (uint32_t k = 0; k < BB_COUNT; k++) {
            uint32_t idx = (min_idx + k) % BB_COUNT;
            if (rec_read(idx, &r) == 0 && rec_valid(&r)) {
                print_rec(&r);
                bb_pace();
            }
        }
    }

    printk("================ BLACKBOX ENDE (%u Eintraege) ================\n\n", count);
    k_mutex_unlock(&bb_lock);
}

/* ------------------------------------------------------------------------ */
/* Schreib-Thread                                                             */
/* ------------------------------------------------------------------------ */

#if DT_HAS_CHOSEN(zephyr_console) && IS_ENABLED(CONFIG_UART_LINE_CTRL)
static const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static bool terminal_open(void) {
    uint32_t dtr = 0;
    if (!device_is_ready(console) || !zmk_usb_is_powered()) {
        return false;
    }
    if (uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr) != 0) {
        return false;
    }
    return dtr != 0;
}
#else
static bool terminal_open(void) { return false; }
#endif

static void bb_thread(void *p1, void *p2, void *p3) {
    struct bb_rec r;
    bool was_open = false;
    int64_t dump_at = 0;
    int64_t next_heart = BB_HEARTBEAT_MS;

    if (flash_area_open(FIXED_PARTITION_ID(blackbox_partition), &bb_fa) != 0) {
        LOG_ERR("Blackbox: Flash-Bereich nicht gefunden");
        return;
    }

    /* Neuer Test-Durchgang (nach Reset-Datei): Blackbox leeren */
    if (!session_known()) {
        LOG_INF("Blackbox: neuer Durchgang, Speicher wird geleert");
        flash_area_erase(bb_fa, 0, BB_SIZE);
        uint8_t one = 1;
        settings_save_one("bbox/s", &one, sizeof(one));
    }

    k_mutex_lock(&bb_lock, K_FOREVER);
    bb_scan();
    bb_ready = true;
    k_mutex_unlock(&bb_lock);

    LOG_INF("Blackbox bereit: Start #%u, naechster Platz %u", bb_boot, bb_idx);

    /* Taktquellen jetzt, wo Bluetooth laeuft */
    uint32_t lf = NRF_CLOCK->LFCLKSTAT;
    uint32_t hf = NRF_CLOCK->HFCLKSTAT;
    bb_put(BB_CLOCK, NRF_CLOCK->LFCLKSRCCOPY & 0x3, 0, 0,
           (lf & 0x3) | ((lf & BIT(16)) ? 0x100 : 0), (hf & 0x1) | ((hf & BIT(16)) ? 0x100 : 0),
           NULL);

    while (true) {
        if (k_msgq_get(&bb_q, &r, K_MSEC(500)) == 0) {
            k_mutex_lock(&bb_lock, K_FOREVER);
            bb_store_one(&r);
            bb_flush_locked();
            k_mutex_unlock(&bb_lock);
        }

        int64_t now = k_uptime_get();

        if (now >= next_heart) {
            next_heart = now + BB_HEARTBEAT_MS;
            bb_put(BB_HEART, zmk_ble_active_profile_is_connected(),
                   zmk_battery_state_of_charge(), 0, 0, 0, NULL);
        }

        /* PuTTY geoeffnet? Dann kurz warten und alles ausgeben */
        bool open = terminal_open();
        if (open && !was_open) {
            dump_at = now + 2000;
        }
        if (!open) {
            dump_at = 0;
        }
        was_open = open;

        if (dump_at && now >= dump_at) {
            dump_at = 0;
            bb_dump();
        }
    }
}

K_THREAD_DEFINE(bb_tid, 2048, bb_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0,
                3000);

/* ------------------------------------------------------------------------ */
/* Bluetooth-Ereignisse                                                       */
/* ------------------------------------------------------------------------ */

static int64_t conn_start;

static void bb_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        bb_put(BB_CONN_FAIL, err, 0, 0, 0, 0, bt_conn_get_dst(conn));
        return;
    }
    struct bt_conn_info info;
    uint16_t iv = 0, lat = 0, to = 0;
    if (bt_conn_get_info(conn, &info) == 0 && info.type == BT_CONN_TYPE_LE) {
        iv = info.le.interval;
        lat = info.le.latency;
        to = info.le.timeout;
    }
    conn_start = k_uptime_get();
    bb_put(BB_CONN_OK, 0, iv, lat, to, 0, bt_conn_get_dst(conn));
}

static void bb_disconnected(struct bt_conn *conn, uint8_t reason) {
    uint32_t dur = conn_start ? (uint32_t)(k_uptime_get() - conn_start) : 0;
    conn_start = 0;
    bb_put(BB_DISC, reason, dur & 0xFFFF, dur >> 16, 0, 0, bt_conn_get_dst(conn));
}

static void bb_security_changed(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err) {
    if (err) {
        bb_put(BB_SEC_FAIL, err, level, 0, 0, 0, NULL);
    } else {
        bb_put(BB_SEC_OK, level, 0, 0, 0, 0, NULL);
    }
}

static bool bb_le_param_req(struct bt_conn *conn, struct bt_le_conn_param *p) {
    bb_put(BB_PARAM_REQ, 0, p->interval_min, p->interval_max, p->latency, p->timeout, NULL);
    return true;
}

static void bb_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                                uint16_t timeout) {
    bb_put(BB_PARAM_UPD, 0, interval, latency, timeout, 0, NULL);
}

#if defined(CONFIG_BT_USER_PHY_UPDATE)
static void bb_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param) {
    bb_put(BB_PHY, param->tx_phy, param->rx_phy, 0, 0, 0, NULL);
}
#endif

BT_CONN_CB_DEFINE(bb_conn_cb) = {
    .connected = bb_connected,
    .disconnected = bb_disconnected,
    .security_changed = bb_security_changed,
    .le_param_req = bb_le_param_req,
    .le_param_updated = bb_le_param_updated,
#if defined(CONFIG_BT_USER_PHY_UPDATE)
    .le_phy_updated = bb_le_phy_updated,
#endif
};

static void bb_pairing_complete(struct bt_conn *conn, bool bonded) {
    bb_put(BB_PAIR_OK, bonded, 0, 0, 0, 0, NULL);
}

static void bb_pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
    bb_put(BB_PAIR_FAIL, reason, 0, 0, 0, 0, NULL);
}

static void bb_bond_deleted(uint8_t id, const bt_addr_le_t *peer) {
    bb_put(BB_BOND_DEL, 0, 0, 0, 0, 0, peer);
}

static struct bt_conn_auth_info_cb bb_auth_info_cb = {
    .pairing_complete = bb_pairing_complete,
    .pairing_failed = bb_pairing_failed,
    .bond_deleted = bb_bond_deleted,
};

void bt_remote_blackbox_pairing(uint8_t what) { bb_put(BB_PAIRWIN, what, 0, 0, 0, 0, NULL); }

/* ------------------------------------------------------------------------ */
/* ZMK-Ereignisse                                                             */
/* ------------------------------------------------------------------------ */

static atomic_t wheel_plus;
static atomic_t wheel_minus;
static bool vol_up_held;
static bool vol_dn_held;

static void wheel_flush(struct k_work *work) {
    atomic_val_t p = atomic_set(&wheel_plus, 0);
    atomic_val_t m = atomic_set(&wheel_minus, 0);
    if (p || m) {
        bb_put(BB_WHEEL, 0, (uint16_t)p, (uint16_t)m, zmk_ble_active_profile_is_connected(), 0,
               NULL);
    }
}

static K_WORK_DELAYABLE_DEFINE(wheel_work, wheel_flush);

static void on_key(const struct zmk_keycode_state_changed *kc) {
    uint8_t id = 0;

    if (kc->usage_page == HID_USAGE_KEY) {
        if (kc->keycode == HID_USAGE_KEY_KEYPAD_PLUS || kc->keycode == HID_USAGE_KEY_KEYPAD_MINUS) {
            if (kc->state) {
                atomic_inc(kc->keycode == HID_USAGE_KEY_KEYPAD_PLUS ? &wheel_plus : &wheel_minus);
                k_work_reschedule(&wheel_work, K_MSEC(1500));
            }
            return;
        }
    } else if (kc->usage_page == HID_USAGE_CONSUMER) {
        switch (kc->keycode) {
        case HID_USAGE_CONSUMER_PLAY_PAUSE: id = 1; break;
        case HID_USAGE_CONSUMER_SCAN_PREVIOUS_TRACK: id = 2; break;
        case HID_USAGE_CONSUMER_SCAN_NEXT_TRACK: id = 3; break;
        case HID_USAGE_CONSUMER_VOLUME_INCREMENT: id = 4; break;
        case HID_USAGE_CONSUMER_VOLUME_DECREMENT: id = 5; break;
        default: break;
        }
    }

    /* Markierung: Lauter + Leiser gleichzeitig (heben sich am PC auf) */
    if (id == 4 || id == 5) {
        bool *self = (id == 4) ? &vol_up_held : &vol_dn_held;
        bool other = (id == 4) ? vol_dn_held : vol_up_held;
        *self = kc->state;
        if (kc->state && other) {
            bb_put(BB_MARKER, 0, 0, 0, 0, 0, NULL);
#if IS_ENABLED(CONFIG_BT_REMOTE_STATUS_LED)
            bt_remote_status_led_marker();
#endif
        }
    }

    if (kc->state) {
        struct zmk_endpoint_instance ep = zmk_endpoints_selected();
        bb_put(BB_KEY, id, zmk_ble_active_profile_is_connected(), ep.transport, kc->usage_page,
               (uint16_t)kc->keycode, NULL);
    }
}

static int bb_listener(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *kc = as_zmk_keycode_state_changed(eh);
    if (kc) {
        on_key(kc);
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_battery_state_changed *bat = as_zmk_battery_state_changed(eh);
    if (bat) {
        bb_put(BB_BATTERY, bat->state_of_charge, 0, 0, 0, 0, NULL);
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_activity_state_changed *act = as_zmk_activity_state_changed(eh);
    if (act) {
        bb_put(BB_ACTIVITY, act->state, 0, 0, 0, 0, NULL);
        if (act->state == ZMK_ACTIVITY_SLEEP) {
            /* Gleich ist der Strom weg: jetzt sofort alles wegschreiben */
            wheel_flush(NULL);
            k_mutex_lock(&bb_lock, K_MSEC(500));
            bb_flush_locked();
            k_mutex_unlock(&bb_lock);
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_usb_conn_state_changed *usb = as_zmk_usb_conn_state_changed(eh);
    if (usb) {
        bb_put(BB_USB, usb->conn_state, 0, 0, 0, 0, NULL);
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_endpoint_changed *ep = as_zmk_endpoint_changed(eh);
    if (ep) {
        bb_put(BB_ENDPOINT, ep->endpoint.transport, 0, 0, 0, 0, NULL);
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_ble_active_profile_changed *prof = as_zmk_ble_active_profile_changed(eh);
    if (prof) {
        bb_put(BB_PROFILE, prof->index, zmk_ble_active_profile_is_connected(),
               zmk_ble_active_profile_is_open(), 0, 0, NULL);
        return ZMK_EV_EVENT_BUBBLE;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(blackbox, bb_listener);
ZMK_SUBSCRIPTION(blackbox, zmk_keycode_state_changed);
ZMK_SUBSCRIPTION(blackbox, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(blackbox, zmk_activity_state_changed);
ZMK_SUBSCRIPTION(blackbox, zmk_usb_conn_state_changed);
ZMK_SUBSCRIPTION(blackbox, zmk_endpoint_changed);
ZMK_SUBSCRIPTION(blackbox, zmk_ble_active_profile_changed);

/* ------------------------------------------------------------------------ */
/* Start                                                                      */
/* ------------------------------------------------------------------------ */

static int bb_init(void) {
    /* Startgrund lesen und fuer den naechsten Start zuruecksetzen */
    uint32_t rr = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = 0xFFFFFFFF;
    uint8_t vbus = (NRF_POWER->USBREGSTATUS & 0x1) ? 1 : 0;

    bb_put(BB_BOOT, vbus, rr & 0xFFFF, rr >> 16, 0, 0, NULL);
    bt_conn_auth_info_cb_register(&bb_auth_info_cb);
    return 0;
}

SYS_INIT(bb_init, APPLICATION, 98);
