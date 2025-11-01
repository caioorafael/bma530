#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bma530));

int main(void)
{
   
    if (!device_is_ready(dev)) {
        LOG_ERR("BMA530 not ready");
        return -1;
    }

    while (1) {
        struct sensor_value x, y, z;
        if (sensor_sample_fetch(dev) == 0 &&
            sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &x) == 0 &&
            sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &y) == 0 &&
            sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &z) == 0) {
            LOG_INF("RAW: x=%d y=%d z=%d", x.val1, y.val1, z.val1);
        }
        k_sleep(K_MSEC(10));
    }
    return 0;
}
