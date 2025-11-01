/*
 * Copyright (c) 2025 Caio Camargo
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include "bma530.h"

#define DT_DRV_COMPAT bosch_bma530

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "BMA530 driver enabled without any devices in devicetree"
#endif

LOG_MODULE_REGISTER(bma530, CONFIG_SENSOR_LOG_LEVEL);

// /* Convert raw temperature to Celsius */
// static void bma530_convert_temp(int8_t raw_temp, struct sensor_value *val)
// {
// 	// Formula: Temp_C = raw_temp * 1K/LSB + 23C [cite: 181, 2559]
// 	float temp_c = (float)raw_temp * BMA530_TEMP_LSB_PER_K + BMA530_TEMP_OFFSET_DEG_C;
// 	sensor_value_from_float(val, temp_c);
// }

// /* Convert raw acceleration to m/s^2 */
// static int bma530_convert_accel(int16_t raw_val, uint8_t range_reg, struct sensor_value *val)
// {
// 	float sensitivity; // LSB/g

// 	switch (range_reg) {
// 	case BMA530_RANGE_2G:
// 		sensitivity = 16384.0f; // [cite: 90]
// 		break;
// 	case BMA530_RANGE_4G:
// 		sensitivity = 8192.0f; // [cite: 94]
// 		break;
// 	case BMA530_RANGE_8G:
// 		sensitivity = 4096.0f; // [cite: 97]
// 		break;
// 	case BMA530_RANGE_16G:
// 		sensitivity = 2048.0f; // [cite: 101]
// 		break;
// 	default:
// 		LOG_ERR("Invalid range selected");
// 		return -EINVAL;
// 	}

// 	// Formula: Accel_g = raw_val / sensitivity
// 	// Convert g to m/s^2: Accel_ms2 = Accel_g * SENSOR_G
// 	double accel_ms2 = (double)raw_val / sensitivity * SENSOR_G;
// 	return sensor_value_from_double(val, accel_ms2);
// }

/* Fetch samples from the sensor */
static int bma530_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct bma530_data *data = dev->data;
	uint8_t buf[6];
	int rc;

	__ASSERT(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_ACCEL_XYZ ||
		 chan == SENSOR_CHAN_DIE_TEMP, "Unsupported channel");

	/* Burst read XYZ: 0x18..0x1D, little-endian per axis */
	rc = bma530_burst_read(dev, BMA530_REG_ACC_DATA_0, buf, sizeof(buf));
	if (rc < 0) {
		LOG_ERR("ACC burst read failed (%d)", rc);
		return rc;
	}

	/* Zephyr endianness helpers */
	data->accel.x = (int16_t)sys_get_le16(&buf[0]);
	data->accel.y = (int16_t)sys_get_le16(&buf[2]);
	data->accel.z = (int16_t)sys_get_le16(&buf[4]);

	/* Temperature (optional) */
	rc = bma530_reg_read(dev, BMA530_REG_TEMP_DATA, (uint8_t *)&data->temp_raw);
	if (rc < 0) {
		LOG_DBG("Temp read failed (%d) — ignoring", rc);
		/* not fatal */
	}

	return 0;
}

/* Get channel data */
static int bma530_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct bma530_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		val->val1 = data->accel.x;
		val->val2 = 0;
		return 0;
	case SENSOR_CHAN_ACCEL_Y:
		val->val1 = data->accel.y;
		val->val2 = 0;
		return 0;
	case SENSOR_CHAN_ACCEL_Z:
		val->val1 = data->accel.z;
		val->val2 = 0;
		return 0;
	case SENSOR_CHAN_ACCEL_XYZ:
		val[0].val1 = data->accel.x; val[0].val2 = 0;
		val[1].val1 = data->accel.y; val[1].val2 = 0;
		val[2].val1 = data->accel.z; val[2].val2 = 0;
		return 0;
	case SENSOR_CHAN_DIE_TEMP: {
		/* Se quiser temperatura em °C via Sensor API */
		struct sensor_value t;
		float temp_c = (float)data->temp_raw * BMA530_TEMP_LSB_PER_K + BMA530_TEMP_OFFSET_DEG_C;
		sensor_value_from_float(&t, temp_c);
		*val = t;
		return 0;
	}
	default:
		return -ENOTSUP;
	}
}

static int bma530_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	uint8_t reg_val;
	int rc = 0;

	// TODO: Implement attribute setting (e.g., range, ODR)
	// Remember the disable->configure->enable sequence 
	// Example for range:
	if (chan == SENSOR_CHAN_ACCEL_XYZ && attr == SENSOR_ATTR_FULL_SCALE) {
		uint8_t new_range_reg;
		int64_t range_g = sensor_ms2_to_g(val); // Convert m/s^2 to g for comparison

		if (range_g <= 2) {
			new_range_reg = BMA530_RANGE_2G;
		} else if (range_g <= 4) {
			new_range_reg = BMA530_RANGE_4G;
		} else if (range_g <= 8) {
			new_range_reg = BMA530_RANGE_8G;
		} else if (range_g <= 16) {
			new_range_reg = BMA530_RANGE_16G;
		} else {
			LOG_ERR("Unsupported range: %lld g", range_g);
			return -EINVAL;
		}

		// Disable sensor
		rc = bma530_reg_write(dev, BMA530_REG_ACC_CONF_0, BMA530_ACC_DISABLE);
		if (rc < 0) { return rc; }
		k_sleep(K_MSEC(5)); // Allow time for sensor to settle/stop

		// Read ACC_CONF_2, modify range, write back
		rc = bma530_reg_read(dev, BMA530_REG_ACC_CONF_2, &reg_val);
		if (rc < 0) { goto enable_sensor; } // Try to re-enable on error

		reg_val &= ~BMA530_ACC_RANGE_MSK; // Clear current range
		reg_val |= (new_range_reg << BMA530_ACC_RANGE_POS) & BMA530_ACC_RANGE_MSK;
		rc = bma530_reg_write(dev, BMA530_REG_ACC_CONF_2, reg_val);
		if (rc < 0) { goto enable_sensor; }

enable_sensor:
		// Enable sensor
		int enable_rc = bma530_reg_write(dev, BMA530_REG_ACC_CONF_0, BMA530_ACC_ENABLE);
		if (enable_rc < 0) {
			LOG_ERR("Failed to re-enable sensor after config change (%d)", enable_rc);
			// Return original error if there was one, otherwise the enable error
			return (rc < 0) ? rc : enable_rc;
		}
		k_sleep(K_MSEC(5)); // Allow time for sensor to stabilise
		return rc; // Return the result of the write operation
	}
	// Add SENSOR_ATTR_SAMPLING_FREQUENCY similarly, using ACC_CONF_1

	return -ENOTSUP;
}

static int bma530_attr_get(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, struct sensor_value *val)
{
	// TODO: Implement attribute getting
	return -ENOTSUP;
}

/* Initialise the sensor */
static int bma530_init(const struct device *dev)
{
	const struct bma530_config *config = dev->config;
	struct bma530_data *data = dev->data;
	uint8_t chip_id = 0;
	uint8_t health = 0;
	int rc;

#ifdef CONFIG_SENSOR_BMA530_TRIGGER
	data->dev = dev; // Store device pointer for triggers
#endif

	if ((config->bus != NULL) && (config->bus->init != NULL)) {
		rc = config->bus->init(dev);
		if (rc < 0) {
			return rc;
		}
	} else {
		LOG_ERR("No bus implementation bound to BMA530 instance");
		return -ENODEV;
	}

	// --- BMA530 Specific I2C/SPI Auto-detection Handling ---
	// The first transaction selects I2C
	// The datasheet recommends an initial dummy read
	// This read might fail or return invalid data, which is expected
	// I2C NACKs this first transaction
	LOG_DBG("Performing initial dummy read...");
	rc = bma530_reg_read(dev, BMA530_REG_CHIP_ID, &chip_id);
	// We expect this might fail or NACK
	if (rc != 0 && rc != -EIO) {
		LOG_WRN("Initial dummy read failed unexpectedly (%d)", rc);
	} else {
		LOG_DBG("Initial dummy read completed (rc=%d), interface likely selected.", rc);
	}
        // A small delay might be prudent after interface selection.
        k_sleep(K_NSEC(100));


#ifdef CONFIG_SENSOR_BMA530_TRIGGER
	if (config->int2_gpio.port != NULL) {
		LOG_DBG("Configuring INT2/CSB pin as output for I2C stability.");
		rc = gpio_pin_configure_dt(&config->int2_gpio, GPIO_OUTPUT_INACTIVE | config->int2_gpio.dt_flags);
                if (rc < 0) {
                        LOG_ERR("Failed to configure INT2/CSB pin as output (%d). I2C communication might be unstable.", rc);
                        // Continue, but warn
                }
                 // Set output mode in INT2_CONF as well
                 uint8_t int2_conf_val = BMA530_INT_MODE_OUTPUT_EN_LATCH | BMA530_INT_LVL_ACTIVE_HIGH; 
                 rc = bma530_reg_write(dev, BMA530_REG_INT2_CONF, int2_conf_val);
                 if (rc < 0) {
                     LOG_WRN("Failed to write INT2_CONF register (%d)", rc);
                 }

	} else {
            LOG_WRN("INT2/CSB GPIO not defined in DT. I2C might falsely detect SPI if INT2 pin floats low.");
            rc = bma530_reg_write(dev, BMA530_REG_INT2_CONF, 0x00);
            if (rc < 0) {
                LOG_WRN("Failed to ensure INT2_CONF output disabled (%d)", rc);
            }
        }
#else
        // If triggers aren't enabled, disable INT2 output mode
        rc = bma530_reg_write(dev, BMA530_REG_INT2_CONF, 0x00); // Disable output mode
        if (rc < 0) {
            LOG_WRN("Failed to write INT2_CONF register to disable output (%d)", rc);
        }
#endif // CONFIG_SENSOR_BMA530_TRIGGER

	// Now perform the actual Chip ID read
	rc = bma530_reg_read(dev, BMA530_REG_CHIP_ID, &chip_id);
	if (rc < 0) {
		LOG_ERR("Failed to read chip ID after interface selection (%d)", rc);
		return rc;
	}

	if (chip_id != BMA530_CHIP_ID) {
		LOG_ERR("Invalid chip ID: expected 0x%02X, got 0x%02X", BMA530_CHIP_ID, chip_id);
		return -ENODEV;
	}
	LOG_INF("Chip ID OK (0x%02X)", chip_id);

	// Check health status
	rc = bma530_reg_read(dev, BMA530_REG_HEALTH_STATUS, &health);
	if (rc < 0) {
		LOG_ERR("Failed to read health status (%d)", rc);
		return rc;
	}
	if ((health & BMA530_HEALTH_STATUS_MSK) != BMA530_HEALTH_STATUS_OK) {
		LOG_ERR("Sensor health check failed (status: 0x%02X)", health);
		return -ENODEV;
	}
	LOG_INF("Health status OK (0x%02X)", health);

	// Apply Default Configuration
	rc = bma530_reg_write(dev, BMA530_REG_ACC_CONF_0, BMA530_ACC_DISABLE);
	if (rc < 0) { return rc; }
	k_sleep(K_MSEC(5)); // Time to settle

	uint8_t acc_conf2 = (BMA530_RANGE_8G << BMA530_ACC_RANGE_POS) & BMA530_ACC_RANGE_MSK;

	rc = bma530_reg_write(dev, BMA530_REG_ACC_CONF_2, acc_conf2);
	if (rc < 0) { goto enable_sensor_init_err; }


	// Set default ODR , BWP (normal/avg4), Power Mode (HPM default) in ACC_CONF_1
	uint8_t acc_conf1 = BMA530_POWER_MODE_HPM |           // Default HPM
						BMA530_ACC_BWP_NORM_AVG4 |      // Default BWP
						((BMA530_ODR_100HZ << BMA530_ACC_ODR_POS) & BMA530_ACC_ODR_MSK); // Default ODR
	rc = bma530_reg_write(dev, BMA530_REG_ACC_CONF_1, acc_conf1);
	if (rc < 0) { goto enable_sensor_init_err; }
	data->accel_hpm = (acc_conf1 & BMA530_POWER_MODE_MSK) == BMA530_POWER_MODE_HPM;


enable_sensor_init_err:
	// Enable sensor
	int enable_rc = bma530_reg_write(dev, BMA530_REG_ACC_CONF_0, BMA530_ACC_ENABLE);
	if (enable_rc < 0) {
		LOG_ERR("Failed to enable sensor during init (%d)", enable_rc);
		return (rc < 0) ? rc : enable_rc; // Return original or enable error
	}
	// Add delay for sensor startup time if needed, depends on mode/ODR
	k_sleep(K_MSEC(5));

#ifdef CONFIG_SENSOR_BMA530_TRIGGER
	rc = bma530_trigger_init(dev);
	if (rc < 0) {
		LOG_ERR("Failed to initialize triggers (%d)", rc);
		return rc;
	}
#endif

	LOG_INF("BMA530 driver initialized successfully.");
	return 0;
}

/* Sensor API structure */
static const struct sensor_driver_api bma530_api_funcs = {
	.sample_fetch = bma530_sample_fetch,
	.channel_get  = bma530_channel_get,
	.attr_set     = bma530_attr_set,
	.attr_get     = bma530_attr_get,
#ifdef CONFIG_SENSOR_BMA530_TRIGGER
	.trigger_set  = bma530_trigger_set,
#endif
};

#ifdef CONFIG_SENSOR_BMA530_TRIGGER
int bma530_trigger_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* Trigger support not implemented yet. */
	return -ENOTSUP;
}

int bma530_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trig);
	ARG_UNUSED(handler);
	/* Trigger support not implemented yet. */
	return -ENOTSUP;
}
#endif

/* Driver instance initialisation macro */
#define BMA530_DEFINE_I2C(inst)								\
	static struct bma530_data bma530_data_##inst;					\
	static const struct bma530_config bma530_config_##inst = {		\
		.i2c = I2C_DT_SPEC_INST_GET(inst),				            \
		.bus = &bma530_i2c_bus_ops,					\
		IF_ENABLED(CONFIG_SENSOR_BMA530_TRIGGER, (			    \
			.int1_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int1_gpios, {0}), \
			.int2_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int2_gpios, {0}), \
		))								                            \
		/* Initialize Kconfig defaults here */				        \
	};																    \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, 								\
		bma530_init, NULL,			\
			      &bma530_data_##inst, &bma530_config_##inst,	\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,		\
			      &bma530_api_funcs);

/* Create instances from device tree */
DT_INST_FOREACH_STATUS_OKAY(BMA530_DEFINE_I2C)