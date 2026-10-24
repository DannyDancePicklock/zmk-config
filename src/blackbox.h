#pragma once

#include <stdint.h>
#include <zephyr/sys/util.h>

/* Blackbox der BT Remote (siehe blackbox.c) */

enum bt_remote_blackbox_pairwin {
    BB_PAIRWIN_START = 1,  /* Kopplungsmodus per Kombi gestartet */
    BB_PAIRWIN_CANCEL = 2, /* mit Play abgebrochen */
    BB_PAIRWIN_DONE = 3,   /* PC erfolgreich gekoppelt */
};

#if IS_ENABLED(CONFIG_BT_REMOTE_BLACKBOX)
void bt_remote_blackbox_pairing(uint8_t what);
#else
static inline void bt_remote_blackbox_pairing(uint8_t what) { ARG_UNUSED(what); }
#endif
