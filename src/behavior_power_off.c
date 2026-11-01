/*
 * Behavior: BT Remote ausschalten (Animation, dann Soft Off).
 * Wird in der Keymap ueber einen Hold-Tap (3 s halten) an die Kombi gebunden.
 */

#define DT_DRV_COMPAT zmk_behavior_bt_remote_power_off

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include "power.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_power_off_binding_pressed(struct zmk_behavior_binding *binding,
                                        struct zmk_behavior_binding_event event) {
    bt_remote_power_off();
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_power_off_binding_released(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_bt_remote_power_off_driver_api = {
    .binding_pressed = on_power_off_binding_pressed,
    .binding_released = on_power_off_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_bt_remote_power_off_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
