#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#if !DT_HAS_COMPAT_STATUS_OKAY(bosch_bma530)
#error "No bosch,bma530 instance marked okay in the devicetree."
#endif

void main(void)
{
    const struct device *acc = DEVICE_DT_GET_ANY(bosch_bma530);
    if (!device_is_ready(acc)) {
        LOG_ERR("BMA530 not ready");
        return;
    }

    while (1) {
        struct sensor_value x, y, z;
        if (sensor_sample_fetch(acc) == 0 &&
            sensor_channel_get(acc, SENSOR_CHAN_ACCEL_X, &x) == 0 &&
            sensor_channel_get(acc, SENSOR_CHAN_ACCEL_Y, &y) == 0 &&
            sensor_channel_get(acc, SENSOR_CHAN_ACCEL_Z, &z) == 0) {
            LOG_INF("RAW LSB: x=%d y=%d z=%d", x.val1, y.val1, z.val1);
        }
        k_sleep(K_MSEC(10));
    }
}
