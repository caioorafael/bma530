#ifndef ZEPHYR_DRIVERS_SENSOR_BMA530_BMA530_H_
#define ZEPHYR_DRIVERS_SENSOR_BMA530_BMA530_H_

#include <stdbool.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>

/* BMA530 Register Addresses */
#define BMA530_REG_CHIP_ID          0x00
#define BMA530_REG_HEALTH_STATUS    0x02
#define BMA530_REG_CMD_SUSPEND      0x04
#define BMA530_REG_CONFIG_STATUS    0x10 
#define BMA530_REG_SENSOR_STATUS    0x11
#define BMA530_REG_INT_STATUS_INT1_0 0x12
#define BMA530_REG_ACC_DATA_0       0x18 // LSB X
#define BMA530_REG_ACC_DATA_1       0x19 // MSB X
#define BMA530_REG_ACC_DATA_2       0x1A // LSB Y
#define BMA530_REG_ACC_DATA_3       0x1B // MSB Y
#define BMA530_REG_ACC_DATA_4       0x1C // LSB Z
#define BMA530_REG_ACC_DATA_5       0x1D // MSB Z
#define BMA530_REG_TEMP_DATA        0x1E // 
#define BMA530_REG_ACC_CONF_0       0x30 // Sensor control
#define BMA530_REG_ACC_CONF_1       0x31 // ODR, BWP, Power Mode
#define BMA530_REG_ACC_CONF_2       0x32 // Range, IIR, Noise, DRDY Clear
#define BMA530_REG_INT1_CONF        0x34 // 
#define BMA530_REG_INT2_CONF        0x35 // 
#define BMA530_REG_INT_MAP_0        0x36 // 
#define BMA530_REG_IF_CONF_1        0x3B // SPI3/I3C Enable
#define BMA530_REG_CMD              0x7E // Soft Reset

/* BMA530 Chip ID */
#define BMA530_CHIP_ID              0xC2 // 

/* BMA530 Acceleration Ranges (g) and corresponding register values */
#define BMA530_RANGE_2G             0x00 // 
#define BMA530_RANGE_4G             0x01 // 
#define BMA530_RANGE_8G             0x02 // 
#define BMA530_RANGE_16G            0x03 // 

/* BMA530 ODR (Hz) and corresponding register values */
#define BMA530_ODR_1_5625HZ         0x00 // LPM only
#define BMA530_ODR_12_5HZ           0x03 // 
#define BMA530_ODR_25HZ             0x04 // 
#define BMA530_ODR_50HZ             0x05 // 
#define BMA530_ODR_100HZ            0x06 // Default ?
#define BMA530_ODR_200HZ            0x07 // 
#define BMA530_ODR_400HZ            0x08 // 
#define BMA530_ODR_800HZ            0x09 // HPM only
#define BMA530_ODR_1600HZ           0x0A // HPM only


/* ACC_CONF_0 Bits */
#define BMA530_ACC_ENABLE           0x0F 
#define BMA530_ACC_DISABLE          0x00 

/* ACC_CONF_1 Bits */
#define BMA530_POWER_MODE_POS       7   
#define BMA530_POWER_MODE_MSK       BIT(BMA530_POWER_MODE_POS)
#define BMA530_POWER_MODE_LPM       (0 << BMA530_POWER_MODE_POS) // 
#define BMA530_POWER_MODE_HPM       (1 << BMA530_POWER_MODE_POS) // 
#define BMA530_ACC_BWP_POS          4    // 
#define BMA530_ACC_BWP_MSK          (0x07 << BMA530_ACC_BWP_POS)
#define BMA530_ACC_BWP_NORM_AVG4    (0x02 << BMA530_ACC_BWP_POS) // Default HPM: normal, LPM: avg4 
#define BMA530_ACC_ODR_POS          0    // 
#define BMA530_ACC_ODR_MSK          (0x0F << BMA530_ACC_ODR_POS)

/* ACC_CONF_2 Bits */
#define BMA530_ACC_RANGE_POS        0    // 
#define BMA530_ACC_RANGE_MSK        (0x03 << BMA530_ACC_RANGE_POS)

/* INTx_CONF Bits */
#define BMA530_INT_MODE_OUTPUT_EN_LATCH   0x01 // 
#define BMA530_INT_MODE_POS         0    //
#define BMA530_INT_MODE_MSK         (0x03 << BMA530_INT_MODE_POS)
#define BMA530_INT_OD_POS           2    // 
#define BMA530_INT_OD_MSK           BIT(BMA530_INT_OD_POS)
#define BMA530_INT_LVL_POS          3    // 
#define BMA530_INT_LVL_MSK          BIT(BMA530_INT_LVL_POS)
#define BMA530_INT_LVL_ACTIVE_HIGH  (1 << BMA530_INT_LVL_POS) // 

/* Health Status Mask */
#define BMA530_HEALTH_STATUS_OK     0x0F // 
#define BMA530_HEALTH_STATUS_MSK    0x0F // 

/* Sensor time resolution (3.2 kHz) -> period in microseconds */
#define BMA530_SENSOR_TIME_RES_US   312.5f //approximately

/* Temperature constants */
#define BMA530_TEMP_OFFSET_DEG_C    23.0f // 0 LSB corresponds to 23 degC
#define BMA530_TEMP_LSB_PER_K       1.0f  // 1 LSB/K

/* Structure for raw accel data */
struct bma530_accel_data {
	int16_t x;
	int16_t y;
	int16_t z;
};

struct bma530_bus_io {
	int (*init)(const struct device *dev);
	int (*read_data)(const struct device *dev, uint8_t reg_addr, uint8_t *data, uint8_t len);
	int (*write_data)(const struct device *dev, uint8_t reg_addr, const uint8_t *data, uint8_t len);
	int (*read_reg)(const struct device *dev, uint8_t reg_addr, uint8_t *data);
	int (*write_reg)(const struct device *dev, uint8_t reg_addr, uint8_t data);
	int (*update_reg)(const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t value);
};

/* Structure for driver runtime data */
struct bma530_data {
	struct bma530_accel_data accel;
	int8_t temp_raw; /* Raw temperature reading */
	bool accel_hpm; /* true if accelerometer operates in HPM */
#ifdef CONFIG_SENSOR_BMA530_TRIGGER
	const struct device *dev;
	struct gpio_callback int1_cb_data;
	struct gpio_callback int2_cb_data;
	struct k_work trigger_work; // Or k_timer for polling

	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trigger;
	// TODO: Add handlers/triggers for other interrupt sources (FIFO, motion, etc.)
#endif
};

/* Structure for driver configuration data (ROM) */
struct bma530_config {
	struct i2c_dt_spec i2c;
	const struct bma530_bus_io *bus;
#ifdef CONFIG_SENSOR_BMA530_TRIGGER
	struct gpio_dt_spec int1_gpio;
	struct gpio_dt_spec int2_gpio;
#endif
	// TODO: Add Kconfig based defaults here (e.g., default_range, default_odr), now it is done in dts file.
};

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
extern const struct bma530_bus_io bma530_i2c_bus_ops;
#endif

#ifdef CONFIG_SENSOR_BMA530_TRIGGER
int bma530_trigger_init(const struct device *dev);
int bma530_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);
#endif

static inline const struct bma530_bus_io *bma530_bus(const struct device *dev)
{
	const struct bma530_config *cfg = dev->config;
	return cfg->bus;
}

static inline int bma530_reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	return bma530_bus(dev)->read_reg(dev, reg, val);
}

static inline int bma530_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	return bma530_bus(dev)->write_reg(dev, reg, val);
}

static inline int bma530_burst_read(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
	return bma530_bus(dev)->read_data(dev, reg, buf, len);
}

static inline int bma530_burst_write(const struct device *dev, uint8_t reg, const uint8_t *buf, uint8_t len)
{
	return bma530_bus(dev)->write_data(dev, reg, buf, len);
}

static inline int bma530_update_reg(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
	return bma530_bus(dev)->update_reg(dev, reg, mask, value);
}

#endif /* ZEPHYR_DRIVERS_SENSOR_BMA530_BMA530_H_ */