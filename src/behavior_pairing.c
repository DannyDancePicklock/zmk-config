/*
 * Behavior: Kopplungsmodus der BT Remote starten (alter PC wird geloescht).
 * Wird in der Keymap ueber einen Hold-Tap (2 s halten) an die Kombi gebunden.
 */

#define DT_DRV_COMPAT zmk_behavior_bt_remote_pairing

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include "pairing.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_pairing_binding_pressed(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    bt_remote_pairing_start();
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_pairing_binding_released(struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_bt_remote_pairing_driver_api = {
    .binding_pressed = on_pairing_binding_pressed,
    .binding_released = on_pairing_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_bt_remote_pairing_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
