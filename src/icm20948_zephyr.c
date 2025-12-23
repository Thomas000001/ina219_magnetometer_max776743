#include "icm20948_zephyr.h"
#include <zephyr/logging/log.h>
#include <string.h> // For memset, strcmp if used for string orientation
#include <math.h>   // For M_PI, sqrtf, asinf, atan2f

LOG_MODULE_REGISTER(icm20948, CONFIG_SENSOR_LOG_LEVEL); // Adjust log level in prj.conf

/* --- Internal Helper Functions (static) --- */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
// Helper to switch banks
static int icm20948_switch_bank(icm20948_dev_t *dev, uint8_t new_bank) {
    if (new_bank == dev->current_bank) {
        return 0;
    }

    uint8_t reg_val = (new_bank << 4);
    int ret;

    if (dev->use_spi) {
        // For SPI, REG_BANK_SEL write does not have read bit
        uint8_t spi_cmd[2] = {ICM20948_REG_BANK_SEL, reg_val};
        const struct spi_buf tx_buf = {.buf = spi_cmd, .len = sizeof(spi_cmd)};
        const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
        ret = spi_write_dt(&dev->bus_cfg.spi, &tx);
    } else {
        ret = i2c_reg_write_byte_dt(&dev->bus_cfg.i2c, ICM20948_REG_BANK_SEL, reg_val);
    }

    if (ret == 0) {
        dev->current_bank = new_bank;
    } else {
        LOG_ERR("Failed to switch bank to %d: %d", new_bank, ret);
    }
    return ret;
}

// Helper to write an 8-bit register
static int icm20948_write_register8(icm20948_dev_t *dev, uint8_t bank, uint8_t reg, uint8_t val) {
    int ret = icm20948_switch_bank(dev, bank);
    if (ret != 0) {
        return ret;
    }

    if (dev->use_spi) {
        uint8_t spi_cmd[2] = {reg, val}; // MSB is 0 for write
        const struct spi_buf tx_buf = {.buf = spi_cmd, .len = sizeof(spi_cmd)};
        const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
        ret = spi_write_dt(&dev->bus_cfg.spi, &tx);
    } else {
        ret = i2c_reg_write_byte_dt(&dev->bus_cfg.i2c, reg, val);
    }
    // Original library had a delayMicroseconds(5) here.
    // k_busy_wait(5) or k_usleep(5) can be used if absolutely necessary,
    // but often not needed with proper bus operations. For now, omitted.
    return ret;
}

// Helper to read an 8-bit register
static int icm20948_read_register8(icm20948_dev_t *dev, uint8_t bank, uint8_t reg, uint8_t *val) {
    int ret = icm20948_switch_bank(dev, bank);
    if (ret != 0) {
        return ret;
    }

    if (dev->use_spi) {
        uint8_t spi_cmd = reg | 0x80; // MSB is 1 for read
        const struct spi_buf tx_buf = {.buf = &spi_cmd, .len = 1};
        const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
        struct spi_buf rx_buf = {.buf = val, .len = 1};
        const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};
        ret = spi_transceive_dt(&dev->bus_cfg.spi, &tx, &rx);
    } else {
        ret = i2c_reg_read_byte_dt(&dev->bus_cfg.i2c, reg, val);
    }
    return ret;
}

// Helper to write a 16-bit register (MSB first)
static int icm20948_write_register16(icm20948_dev_t *dev, uint8_t bank, uint8_t reg, int16_t val) {
    int ret = icm20948_switch_bank(dev, bank);
    if (ret != 0) {
        return ret;
    }
    uint8_t data[2];
    data[0] = (val >> 8) & 0xFF; // MSB
    data[1] = val & 0xFF;        // LSB

    if (dev->use_spi) {
        uint8_t spi_cmd[3] = {reg, data[0], data[1]};
        const struct spi_buf tx_buf = {.buf = spi_cmd, .len = sizeof(spi_cmd)};
        const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
        ret = spi_write_dt(&dev->bus_cfg.spi, &tx);
    } else {
        ret = i2c_burst_write_dt(&dev->bus_cfg.i2c, reg, data, 2);
    }
    return ret;
}

// Helper to read a 16-bit register (MSB first)
static int icm20948_read_register16(icm20948_dev_t *dev, uint8_t bank, uint8_t reg, int16_t *val) {
    int ret = icm20948_switch_bank(dev, bank);
    if (ret != 0) {
        return ret;
    }
    uint8_t data[2];

    if (dev->use_spi) {
        uint8_t spi_cmd = reg | 0x80;
        const struct spi_buf tx_buf = {.buf = &spi_cmd, .len = 1};
        const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
        struct spi_buf rx_buf = {.buf = data, .len = 2};
        const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};
        ret = spi_transceive_dt(&dev->bus_cfg.spi, &tx, &rx);
    } else {
        ret = i2c_burst_read_dt(&dev->bus_cfg.i2c, reg, data, 2);
    }

    if (ret == 0) {
        *val = ((int16_t)data[0] << 8) | data[1];
    }
    return ret;
}

static int icm20948_reset_icm(icm20948_dev_t *dev) {
    int ret = icm20948_write_register8(dev, 0, ICM20948_PWR_MGMT_1, ICM20948_RESET);
    if (ret != 0) {
        LOG_ERR("Failed to reset ICM20948: %d", ret);
        return ret;
    }
    k_msleep(100); // Datasheet: "wait for registers to reset", original: delay(10), increased for safety
    return 0;
}

// Helper for AK09916 register write via SLV4 (simplified from original)
static int icm20948_write_ak09916_register8_slv4(icm20948_dev_t *dev, uint8_t reg, uint8_t val) {
    int ret;
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_ADDR, AK09916_ADDRESS); // Write mode
    if (ret) return ret;
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_REG, reg);
    if (ret) return ret;
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_DO, val);
    if (ret) return ret;
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVX_EN);
    if (ret) return ret;

    // Wait for transaction to complete (original had a polling loop with timeout)
    // This is a simplification; a robust implementation might need a delay or status check.
    // For now, a small delay. The I2C_MST_STATUS could be polled.
    k_msleep(10); // Increased delay from original
    uint8_t status;
    for(int i=0; i<10; i++){ // Poll for max 100ms
        ret = icm20948_read_register8(dev, 3, ICM20948_I2C_SLV4_CTRL, &status);
        if(ret) return ret;
        if(!(status & ICM20948_I2C_SLVX_EN)) break;
        k_msleep(10);
    }
    if(status & ICM20948_I2C_SLVX_EN) {
        LOG_WRN("SLV4 write timeout");
        // Might need to disable SLV4 explicitly if stuck
        // icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_CTRL, 0x00);
    }
    return 0;
}

// Helper for AK09916 register read via SLV4
static int icm20948_read_ak09916_register8_slv4(icm20948_dev_t *dev, uint8_t reg, uint8_t *val) {
    int ret;
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_ADDR, AK09916_ADDRESS | AK09916_READ); // Read mode
    if (ret) return ret;
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_REG, reg);
    if (ret) return ret;
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVX_EN);
    if (ret) return ret;

    k_msleep(10);
    uint8_t status;
     for(int i=0; i<10; i++){ // Poll for max 100ms
        ret = icm20948_read_register8(dev, 3, ICM20948_I2C_SLV4_CTRL, &status);
        if(ret) return ret;
        if(!(status & ICM20948_I2C_SLVX_EN)) break;
        k_msleep(10);
    }
    if(status & ICM20948_I2C_SLVX_EN) {
        LOG_WRN("SLV4 read trigger timeout");
    }

    return icm20948_read_register8(dev, 3, ICM20948_I2C_SLV4_DI, val);
}

static int icm20948_enable_i2c_master(icm20948_dev_t *dev) {
    int ret = icm20948_write_register8(dev, 0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN);
    if (ret) return ret;
    // Set I2C clock to 345.60 kHz (matches original)
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_MST_CTRL, 0x07);
    if (ret) return ret;
    k_msleep(10);
    return 0;
}

static int icm20948_i2c_master_reset(icm20948_dev_t *dev) {
    uint8_t reg_val;
    int ret = icm20948_read_register8(dev, 0, ICM20948_USER_CTRL, &reg_val);
    if(ret) return ret;
    reg_val |= ICM20948_I2C_MST_RST_BIT;
    ret = icm20948_write_register8(dev, 0, ICM20948_USER_CTRL, reg_val);
    if(ret) return ret;
    k_msleep(10);
    return 0;
}


static int icm20948_enable_mag_data_read(icm20948_dev_t *dev, uint8_t reg, uint8_t bytes_to_read) {
    int ret;
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | AK09916_READ);
    if (ret) return ret;
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_SLV0_REG, reg);
    if (ret) return ret;
    ret = icm20948_write_register8(dev, 3, ICM20948_I2C_SLV0_CTRL, ICM20948_I2C_SLVX_EN | bytes_to_read);
    if (ret) return ret;
    k_msleep(10); // Allow transaction to start
    return 0;
}

static void correct_acc_raw_values(icm20948_dev_t *dev, xyzFloat_t *corr_acc_raw_val){
    // Ensure acc_range_factor is not zero to prevent division by zero
    float factor_x = (dev->acc_range_factor != 0.0f) ? dev->acc_range_factor : 1.0f;
    float factor_y = (dev->acc_range_factor != 0.0f) ? dev->acc_range_factor : 1.0f;
    float factor_z = (dev->acc_range_factor != 0.0f) ? dev->acc_range_factor : 1.0f;

    corr_acc_raw_val->x = (corr_acc_raw_val->x -(dev->acc_offset_val.x / factor_x));
    corr_acc_raw_val->y = (corr_acc_raw_val->y -(dev->acc_offset_val.y / factor_y));
    corr_acc_raw_val->z = (corr_acc_raw_val->z -(dev->acc_offset_val.z / factor_z));

    // Apply correction factor if it's not 1.0 (to avoid division by zero if it's uninitialized or 0)
    if (dev->acc_corr_factor.x != 0.0f && dev->acc_corr_factor.x != 1.0f) corr_acc_raw_val->x /= dev->acc_corr_factor.x;
    if (dev->acc_corr_factor.y != 0.0f && dev->acc_corr_factor.y != 1.0f) corr_acc_raw_val->y /= dev->acc_corr_factor.y;
    if (dev->acc_corr_factor.z != 0.0f && dev->acc_corr_factor.z != 1.0f) corr_acc_raw_val->z /= dev->acc_corr_factor.z;
}

static void correct_gyr_raw_values(icm20948_dev_t *dev, xyzFloat_t *corr_gyr_raw_val){
    // Ensure gyr_range_factor is not zero
    float factor_x = (dev->gyr_range_factor != 0.0f) ? dev->gyr_range_factor : 1.0f;
    float factor_y = (dev->gyr_range_factor != 0.0f) ? dev->gyr_range_factor : 1.0f;
    float factor_z = (dev->gyr_range_factor != 0.0f) ? dev->gyr_range_factor : 1.0f;

    corr_gyr_raw_val->x -= (dev->gyr_offset_val.x / factor_x);
    corr_gyr_raw_val->y -= (dev->gyr_offset_val.y / factor_y);
    corr_gyr_raw_val->z -= (dev->gyr_offset_val.z / factor_z);
}


/* --- Public Function Implementations --- */

int icm20948_init(icm20948_dev_t *dev, const struct device *bus_dev, bool is_spi, uint16_t addr_or_cs_info) {
    if (!dev || !bus_dev) {
        return -EINVAL;
    }
    memset(dev, 0, sizeof(icm20948_dev_t)); // Initialize struct
    dev->use_spi = is_spi;

    if (is_spi) {
        dev->bus_cfg.spi.bus = bus_dev;
        // Simplified SPI setup: assumes DT configuration provides the CS GPIO.
        // addr_or_cs_info is assumed to be a dummy value for now, or requires more complex handling
        // to set up spi_config.frequency and spi_config.cs based on addr_or_cs_info.
        // For a real Zephyr driver, you'd get spi_dt_spec from device tree.
        // This part needs to be adapted based on how you get spi_config and cs_control.
        // dev->bus_cfg.spi.config.frequency = 7000000; // Default from original
        // dev->bus_cfg.spi.config.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA; // SPI_MODE0
        // dev->bus_cfg.spi.config.slave = 0;
        // dev->bus_cfg.spi.config.cs = NULL; // Let bus master handle if possible, or set up cs_control
        LOG_WRN("SPI initialization is simplified and may need adjustment for CS and spi_config.");
        // For now, we rely on the user to have a spi_dt_spec configured in device tree
        // and pass dev->bus_cfg.spi (which is struct spi_dt_spec) correctly.
        // The function signature would ideally be:
        // int icm20948_init_spi(icm20948_dev_t *dev, const struct spi_dt_spec* spi_spec)
        // For this example, we assume bus_dev is the spi controller and use a default config.
        // The user must ensure the spi_dt_spec is properly formed if they populate dev->bus_cfg.spi directly.
        // If not using DT_SPEC, manual configuration of spi_config is tricky without more info.
    } else {
        dev->bus_cfg.i2c.bus = bus_dev;
        dev->bus_cfg.i2c.addr = addr_or_cs_info;
    }

    if (!device_is_ready(bus_dev)) {
        LOG_ERR("Bus device not ready");
        return -ENODEV;
    }

    dev->current_bank = 0xFF; // Force initial bank switch

    int ret = icm20948_reset_icm(dev);
    if (ret != 0) {
        return ret;
    }

    uint8_t whoami_val;
    int tries = 0;
    while(tries < 5) { // Reduced tries from 10
        ret = icm20948_who_am_i(dev, &whoami_val);
        if(ret == 0 && whoami_val == ICM20948_WHO_AM_I_CONTENT) {
            break;
        }
        LOG_WRN("WhoAmI check failed (try %d). Got 0x%02X, ret %d. Resetting...", tries + 1, whoami_val, ret);
        ret = icm20948_reset_icm(dev);
        if (ret != 0) return ret; // If reset fails, bail out
        k_msleep(300); // Delay from original after reset
        tries++;
    }

    if (tries == 5) {
        LOG_ERR("Failed to verify ICM20948 WhoAmI after multiple tries.");
        return -EIO;
    }

    xyzFloat_init(&dev->acc_offset_val);
    xyzFloat_init_values(&dev->acc_corr_factor, 1.0f, 1.0f, 1.0f);
    dev->acc_range_factor = 1.0f; // Corresponds to 2G initially before setAccRange
    xyzFloat_init(&dev->gyr_offset_val);
    dev->gyr_range_factor = 1.0f; // Corresponds to 250DPS initially
    dev->fifo_type = ICM20948_FIFO_ACC; // Default

    ret = icm20948_set_sleep_mode(dev, false); // Wake up
    if (ret != 0) return ret;

    // Aligns ODR
    ret = icm20948_write_register8(dev, 2, ICM20948_ODR_ALIGN_EN, 1);
    if (ret != 0) return ret;

    LOG_INF("ICM20948 initialized successfully.");
    return 0;
}


int icm20948_who_am_i(icm20948_dev_t *dev, uint8_t *value) {
    return icm20948_read_register8(dev, 0, ICM20948_WHO_AM_I, value);
}

int icm20948_set_sleep_mode(icm20948_dev_t *dev, bool sleep_en) {
    int ret = icm20948_read_register8(dev, 0, ICM20948_PWR_MGMT_1, &dev->reg_val_cache);
    if (ret != 0) return ret;
    if (sleep_en) {
        dev->reg_val_cache |= ICM20948_SLEEP_BIT;
    } else {
        dev->reg_val_cache &= ~ICM20948_SLEEP_BIT;
    }
    // Also ensure device is not in reset and clock is auto selected (best source)
    // dev->reg_val_cache &= ~ICM20948_RESET; // Reset is a pulse, should not be set here
    // dev->reg_val_cache |= 0x01; // CLKSEL = Auto
    return icm20948_write_register8(dev, 0, ICM20948_PWR_MGMT_1, dev->reg_val_cache);
}

int icm20948_set_acc_range(icm20948_dev_t *dev, icm20948_acc_range_t acc_range_enum) {
    int ret = icm20948_read_register8(dev, 2, ICM20948_ACCEL_CONFIG, &dev->reg_val_cache);
    if (ret != 0) return ret;
    dev->reg_val_cache &= ~(0x06); // Clear bits [2:1]
    dev->reg_val_cache |= (acc_range_enum << 1);
    ret = icm20948_write_register8(dev, 2, ICM20948_ACCEL_CONFIG, dev->reg_val_cache);
    if(ret == 0) {
        dev->acc_range_factor = (float)(1 << acc_range_enum); // 2G->1, 4G->2, 8G->4, 16G->8 (scaling factor for raw to Gs, if raw is always +/-16384 for full scale)
                                                            // Or rather, full scale changes: 2G, 4G, 8G, 16G.
                                                            // So Sensitivity = FullScale / 32768. To get G, measurement = raw * FullScale / 32768
                                                            // Or, measurement_in_G = (raw / 32768.0) * current_full_scale_G
                                                            // The original lib uses accRangeFactor = 1 for 2G, 2 for 4G, etc.
                                                            // and then result_g = raw_corrected * accRangeFactor / 16384.0
                                                            // This seems like accRangeFactor is intended to be the G value of full scale.
                                                            // 2G -> range_factor = 2.0
                                                            // 4G -> range_factor = 4.0
        switch(acc_range_enum){
            case ICM20948_ACC_RANGE_2G:  dev->acc_range_factor = 2.0f; break;
            case ICM20948_ACC_RANGE_4G:  dev->acc_range_factor = 4.0f; break;
            case ICM20948_ACC_RANGE_8G:  dev->acc_range_factor = 8.0f; break;
            case ICM20948_ACC_RANGE_16G: dev->acc_range_factor = 16.0f; break;
            default: dev->acc_range_factor = 2.0f; break;
        }
    }
    return ret;
}

int icm20948_set_gyr_range(icm20948_dev_t *dev, icm20948_gyro_range_t gyro_range_enum) {
    int ret = icm20948_read_register8(dev, 2, ICM20948_GYRO_CONFIG_1, &dev->reg_val_cache);
    if (ret != 0) return ret;
    dev->reg_val_cache &= ~(0x06); // Clear bits [2:1]
    dev->reg_val_cache |= (gyro_range_enum << 1);
    ret = icm20948_write_register8(dev, 2, ICM20948_GYRO_CONFIG_1, dev->reg_val_cache);
     if(ret == 0) {
        // Original: gyrRangeFactor = (1<<gyroRange); then value_dps = raw_corrected * gyrRangeFactor * 250.0 / 32768.0;
        // This implies gyrRangeFactor for 250dps is 1, 500dps is 2, etc.
        // And the base sensitivity is for 250dps.
        switch(gyro_range_enum){
            case ICM20948_GYRO_RANGE_250:  dev->gyr_range_factor = 250.0f; break;
            case ICM20948_GYRO_RANGE_500:  dev->gyr_range_factor = 500.0f; break;
            case ICM20948_GYRO_RANGE_1000: dev->gyr_range_factor = 1000.0f; break;
            case ICM20948_GYRO_RANGE_2000: dev->gyr_range_factor = 2000.0f; break;
            default: dev->gyr_range_factor = 250.0f; break;
        }
    }
    return ret;
}


int icm20948_read_sensor_data(icm20948_dev_t *dev) {
    int ret = icm20948_switch_bank(dev, 0); // Data is in bank 0
    if (ret != 0) {
        return ret;
    }

    if (dev->use_spi) {
        uint8_t spi_cmd = ICM20948_ACCEL_OUT | 0x80; // Start reg + Read bit
        const struct spi_buf tx_buf = {.buf = &spi_cmd, .len = 1};
        const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
        struct spi_buf rx_buf = {.buf = dev->buffer, .len = 20}; // Read 20 bytes
        const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};
        ret = spi_transceive_dt(&dev->bus_cfg.spi, &tx, &rx);
    } else {
        ret = i2c_burst_read_dt(&dev->bus_cfg.i2c, ICM20948_ACCEL_OUT, dev->buffer, 20);
    }
    return ret;
}

int icm20948_get_acc_raw_values(icm20948_dev_t *dev, xyzFloat_t *acc_raw_val) {
    if (!acc_raw_val) return -EINVAL;
    // Assumes dev->buffer is already filled by icm20948_read_sensor_data()
    acc_raw_val->x = (float)((int16_t)((dev->buffer[0] << 8) | dev->buffer[1]));
    acc_raw_val->y = (float)((int16_t)((dev->buffer[2] << 8) | dev->buffer[3]));
    acc_raw_val->z = (float)((int16_t)((dev->buffer[4] << 8) | dev->buffer[5]));
    return 0;
}

int icm20948_get_corrected_acc_raw_values(icm20948_dev_t *dev, xyzFloat_t *corr_acc_raw_val) {
    int ret = icm20948_get_acc_raw_values(dev, corr_acc_raw_val);
    if (ret == 0) {
        correct_acc_raw_values(dev, corr_acc_raw_val);
    }
    return ret;
}

int icm20948_get_g_values(icm20948_dev_t *dev, xyzFloat_t *g_val) {
    int ret = icm20948_get_corrected_acc_raw_values(dev, g_val);
    if (ret == 0) {
        // Sensitivity: raw value of 16384 corresponds to acc_range_factor (e.g. 2G)
        // So, value_in_G = (corrected_raw / 16384.0) * acc_range_factor
        // The original library used: gVal->x = gVal->x * accRangeFactor / 16384.0;
        // Where accRangeFactor was 1 for 2G, 2 for 4G. This seems like a different interpretation.
        // Let's use sensitivity = FullScale_G / 32768.0 for bipolar
        // If acc_range_factor is 2.0f for 2G range:
        float scale = dev->acc_range_factor / 32768.0f; // For +- full scale
                                                       // Original was accRangeFactor / 16384.0, where accRangeFactor was 1,2,4,8.
                                                       // if dev->acc_range_factor is 2.0, 4.0, 8.0, 16.0 (actual G value)
                                                       // then scaling is (corrected_raw / 32768.0f) * (full_scale_G_value / 2.0f) IF raw is always +/-32768
                                                       // OR (corrected_raw / (32768.0f / (dev->acc_range_factor/2.0f)))
                                                       // This means LSB = (dev->acc_range_factor / 32768.0f) G/LSB
        g_val->x *= scale;
        g_val->y *= scale;
        g_val->z *= scale;
    }
    return ret;
}

int icm20948_get_gyr_raw_values(icm20948_dev_t *dev, xyzFloat_t *gyr_raw_val) {
    if (!gyr_raw_val) return -EINVAL;
    gyr_raw_val->x = (float)((int16_t)((dev->buffer[6] << 8) | dev->buffer[7]));
    gyr_raw_val->y = (float)((int16_t)((dev->buffer[8] << 8) | dev->buffer[9]));
    gyr_raw_val->z = (float)((int16_t)((dev->buffer[10] << 8) | dev->buffer[11]));
    return 0;
}

int icm20948_get_corrected_gyr_raw_values(icm20948_dev_t *dev, xyzFloat_t *corr_gyr_val) {
    int ret = icm20948_get_gyr_raw_values(dev, corr_gyr_val);
    if (ret == 0) {
        correct_gyr_raw_values(dev, corr_gyr_val);
    }
    return ret;
}

int icm20948_get_gyr_values(icm20948_dev_t *dev, xyzFloat_t *gyr_val) {
    int ret = icm20948_get_corrected_gyr_raw_values(dev, gyr_val);
    if (ret == 0) {
        // Sensitivity: FullScale_DPS / 32768.0
        // Original: gyrVal->x = gyrVal->x * gyrRangeFactor * 250.0 / 32768.0;
        // where gyrRangeFactor was 1 for 250DPS, 2 for 500DPS.
        // If dev->gyr_range_factor stores 250.0, 500.0 etc directly:
        float scale = dev->gyr_range_factor / 32768.0f;
        gyr_val->x *= scale;
        gyr_val->y *= scale;
        gyr_val->z *= scale;
    }
    return ret;
}

int icm20948_get_temperature(icm20948_dev_t *dev, float *temp) {
    if (!temp) return -EINVAL;
    // Assumes dev->buffer is filled
    int16_t raw_temp = (int16_t)((dev->buffer[12] << 8) | dev->buffer[13]);
    *temp = ((float)raw_temp - ICM20948_ROOM_TEMP_OFFSET) / ICM20948_T_SENSITIVITY + 21.0f;
    return 0;
}


int icm20948_get_mag_values(icm20948_dev_t *dev, xyzFloat_t *mag_val) {
    if (!mag_val) return -EINVAL;
    // Assumes dev->buffer contains mag data from EXT_SLV_SENS_DATA (offset 14 for AK09916_HXL)
    // This means icm20948_enable_mag_data_read must have been called appropriately.
    // The original readAllData reads 20 bytes starting from ACCEL_OUT.
    // ACCEL_OUT (6B), GYRO_OUT (6B), TEMP_OUT (2B) = 14 bytes.
    // So, buffer[14] onwards is EXT_SLV_SENS_DATA_00.
    // AK09916 data (HXL, HXH, HYL, HYH, HZL, HZH, ST2) is 7 bytes.
    // Original reads 8 bytes: AK09916_HXL to ST2 (0x11 to 0x18).
    // Check if enableMagDataRead(AK09916_HXL, 0x08) was done.

    // If using the main buffer dev->buffer which is filled by read_sensor_data()
    // and mag data is enabled to be read into EXT_SLV_SENS_DATA_xx registers.
    int16_t mx = (int16_t)((dev->buffer[15] << 8) | dev->buffer[14]); // HXH, HXL
    int16_t my = (int16_t)((dev->buffer[17] << 8) | dev->buffer[16]); // HYH, HYL
    int16_t mz = (int16_t)((dev->buffer[19] << 8) | dev->buffer[18]); // HZH, HZL

    mag_val->x = (float)mx * AK09916_MAG_LSB;
    mag_val->y = (float)my * AK09916_MAG_LSB;
    mag_val->z = (float)mz * AK09916_MAG_LSB;
    return 0;
}


int icm20948_init_magnetometer(icm20948_dev_t *dev) {
    int ret;
    ret = icm20948_enable_i2c_master(dev);
    if (ret) { LOG_ERR("Enable I2C Master failed: %d", ret); return ret; }

    // Original: resetMag(); reset_ICM20948(); sleep(false); write ODR_ALIGN_EN
    // Some of this is redundant if called after main init.
    // We need to ensure I2C master is on, then talk to AK09916
    ret = icm20948_reset_mag(dev); // Resets AK09916 via I2C master
    if (ret) { LOG_ERR("Mag reset failed: %d", ret); return ret; }


    // The original library then resets ICM20948 and reconfigures some ICM settings.
    // This seems disruptive if init_magnetometer is called after ICM is already running.
    // For now, assuming ICM is configured and I2C master is enabled.
    // If not, the caller should ensure icm20948_init is called first.

    bool mag_init_success = false;
    uint8_t tries = 0;
    while (!mag_init_success && tries < 5) { // Max 5 tries
        // Re-ensure I2C master is enabled in case it was disabled or reset.
        // This could be part of a loop that retries enabling I2C master if mag comms fail.
        // ret = icm20948_enable_i2c_master(dev);
        // if (ret) { k_msleep(50); tries++; continue; }

        uint16_t mag_id_val;
        ret = icm20948_who_am_i_mag(dev, &mag_id_val);
        if (ret == 0 && (mag_id_val == AK09916_WHO_AM_I_1_VAL || mag_id_val == AK09916_WHO_AM_I_2_VAL)) {
            mag_init_success = true;
        } else {
            LOG_WRN("Mag WhoAmI check failed (try %d), got 0x%04X, ret %d. Resetting I2C Master.", tries + 1, mag_id_val, ret);
            ret = icm20948_i2c_master_reset(dev); // Reset ICM's I2C master interface
            if (ret) {LOG_ERR("I2C Master reset failed: %d", ret);}
            k_msleep(50); // Wait after master reset
        }
        tries++;
    }

    if (!mag_init_success) {
        LOG_ERR("Failed to initialize magnetometer.");
        return -EIO;
    }

    ret = icm20948_set_mag_op_mode(dev, AK09916_CONT_MODE_100HZ);
    if (ret) { LOG_ERR("Set mag op mode failed: %d", ret); return ret; }

    LOG_INF("Magnetometer initialized.");
    return 0;
}

int icm20948_who_am_i_mag(icm20948_dev_t *dev, uint16_t *mag_id) {
    uint8_t msb, lsb;
    int ret = icm20948_read_ak09916_register8_slv4(dev, AK09916_WIA_1, &msb);
    if (ret) return ret;
    ret = icm20948_read_ak09916_register8_slv4(dev, AK09916_WIA_1 + 1, &lsb); // WIA_2 is next reg
    if (ret) return ret;
    *mag_id = ((uint16_t)msb << 8) | lsb;
    return 0;
}

int icm20948_set_mag_op_mode(icm20948_dev_t *dev, ak09916_op_mode_t op_mode) {
    int ret = icm20948_write_ak09916_register8_slv4(dev, AK09916_CNTL_2, op_mode);
    if (ret) return ret;
    k_msleep(10); // Delay from original
    if (op_mode != AK09916_PWR_DOWN) {
        // Enable continuous read of magnetometer data into EXT_SLV_SENS_DATA registers
        // Original reads 8 bytes (HXL to ST2). ST2 is at 0x18. HXL is 0x11. (0x18-0x11)+1 = 8 bytes.
        return icm20948_enable_mag_data_read(dev, AK09916_HXL, 0x08);
    }
    return 0;
}

int icm20948_reset_mag(icm20948_dev_t *dev) {
    int ret = icm20948_write_ak09916_register8_slv4(dev, AK09916_CNTL_3, 0x01); // SRST bit
    if (ret) return ret;
    k_msleep(100); // Delay from original
    return 0;
}


// --- Placeholder for other functions ---
// Many functions like setAccDLPF, autoOffsets, getAngles, FIFO management etc.
// would follow a similar pattern:
// 1. Read existing register value if modifying bits.
// 2. Update bits.
// 3. Write new register value.
// 4. Handle errors and return status.

const char* icm20948_get_orientation_as_string(icm20948_dev_t *dev) {
    // Note: Returning pointers to static strings is generally okay for simple cases,
    // but be mindful of thread-safety if this driver were used in a highly concurrent way
    // where the underlying orientation could change between call to get_orientation and this.
    // For typical sensor reading loops, it's fine.
    static const char* orientations_str[] = {
        "z up", "z down", "y up", "y down", "x up", "x down"
    };
    icm20948_orientation_t orient = icm20948_get_orientation(dev);
    if (orient >= ICM20948_FLAT && orient <= ICM20948_YX_1) {
        return orientations_str[orient];
    }
    return "unknown";
}

icm20948_orientation_t icm20948_get_orientation(icm20948_dev_t *dev){
    xyzFloat_t angle_val;
    icm20948_orientation_t orientation = ICM20948_FLAT; // Default

    if(icm20948_get_angles(dev, &angle_val) != 0) {
        LOG_WRN("Failed to get angles for orientation calculation.");
        return orientation; // Return default on error
    }

    if(fabsf(angle_val.x) < 45.0f){      // |x| < 45
        if(fabsf(angle_val.y) < 45.0f){  // |y| < 45
            if(angle_val.z > 0.0f){      //  z  > 0
                orientation = ICM20948_FLAT;
            } else {                     //  z  < 0
                orientation = ICM20948_FLAT_1;
            }
        } else {                         // |y| > 45
            if(angle_val.y > 0.0f){      //  y  > 0
                orientation = ICM20948_XY;
            } else {                     //  y  < 0
                orientation = ICM20948_XY_1;
            }
        }
    } else {                             // |x| >= 45
        if(angle_val.x > 0.0f){          //  x  >  0
            orientation = ICM20948_YX;
        } else {                         //  x  <  0
            orientation = ICM20948_YX_1;
        }
    }
    return orientation;
}

int icm20948_get_angles(icm20948_dev_t *dev, xyzFloat_t *angle_val) {
    if(!angle_val) return -EINVAL;

    xyzFloat_t g_val;
    int ret = icm20948_get_g_values(dev, &g_val);
    if (ret != 0) {
        return ret;
    }

    // Clamp g_val to +/- 1.0 to avoid domain errors with asinf
    g_val.x = (g_val.x > 1.0f) ? 1.0f : (g_val.x < -1.0f ? -1.0f : g_val.x);
    g_val.y = (g_val.y > 1.0f) ? 1.0f : (g_val.y < -1.0f ? -1.0f : g_val.y);
    g_val.z = (g_val.z > 1.0f) ? 1.0f : (g_val.z < -1.0f ? -1.0f : g_val.z);


    angle_val->x = asinf(g_val.x) * (180.0f / (float)M_PI);
    angle_val->y = asinf(g_val.y) * (180.0f / (float)M_PI);
    angle_val->z = asinf(g_val.z) * (180.0f / (float)M_PI); // Original also did this for Z

    return 0;
}

float icm20948_get_pitch(icm20948_dev_t *dev) {
    xyzFloat_t g_val; // Changed from angle_val in original logic mistake
    if (icm20948_get_g_values(dev, &g_val) != 0) {
        return 0.0f; // Error
    }
    // Original used angle_val.x, angle_val.y, angle_val.z which were already asin results.
    // Pitch and Roll are typically calculated from accelerometer g-values.
    // Pitch: rotation around Y-axis
    // Roll: rotation around X-axis
    // float pitch = atan2f(-g_val.x, sqrtf(g_val.y * g_val.y + g_val.z * g_val.z)) * (180.0f / M_PI);
    // Simpler version if Z is mostly vertical for small angles, matching original intent more closely:
     return atan2f(g_val.x, sqrtf(g_val.y * g_val.y + g_val.z * g_val.z)) * (180.0f / (float)M_PI);
    // The original library's getPitch and getRoll were:
    // pitch = (atan2(-angleVal.x, sqrt(abs((angleVal.y*angleVal.y + angleVal.z*angleVal.z))))*180.0)/M_PI;
    // roll = (atan2(angleVal.y, angleVal.z)*180.0)/M_PI;
    // This used angleVal which were arcsin results. This is unconventional.
    // Using g_val directly is more standard for pitch/roll from accel.
    // For consistency with potential original intent if it expected angles:
    // xyzFloat_t angle_val_for_pitch_roll;
    // icm20948_get_angles(dev, &angle_val_for_pitch_roll);
    // return (atan2f(-angle_val_for_pitch_roll.x, sqrtf(fabsf(angle_val_for_pitch_roll.y*angle_val_for_pitch_roll.y + angle_val_for_pitch_roll.z*angle_val_for_pitch_roll.z))) * 180.0f) / (float)M_PI;
}

float icm20948_get_roll(icm20948_dev_t *dev) {
    xyzFloat_t g_val;
     if (icm20948_get_g_values(dev, &g_val) != 0) {
        return 0.0f; // Error
    }
    // return atan2f(g_val.y, sqrtf(g_val.x * g_val.x + g_val.z * g_val.z)) * (180.0f / M_PI); // Standard alternative for roll
    return atan2f(g_val.y, g_val.z) * (180.0f / (float)M_PI); // Matches original library structure if g_val.z is dominant
    // For consistency with potential original intent if it expected angles:
    // xyzFloat_t angle_val_for_pitch_roll;
    // icm20948_get_angles(dev, &angle_val_for_pitch_roll);
    // return (atan2f(angle_val_for_pitch_roll.y, angle_val_for_pitch_roll.z) * 180.0f) / (float)M_PI;
}

// ... Many other functions need to be implemented following this pattern ...
// Example: icm20948_set_acc_dlpf
int icm20948_set_acc_dlpf(icm20948_dev_t *dev, icm20948_dlpf_t dlpf) {
    int ret = icm20948_read_register8(dev, 2, ICM20948_ACCEL_CONFIG, &dev->reg_val_cache);
    if (ret != 0) return ret;

    if (dlpf == ICM20948_DLPF_OFF) {
        dev->reg_val_cache &= ~BIT(0); // ACCEL_FCHOICE = 1 (bypass DLPF)
    } else {
        dev->reg_val_cache |= BIT(0);  // ACCEL_FCHOICE = 0 (enable DLPF)
        dev->reg_val_cache &= ~(0x07 << 3); // Clear bits [5:3] for ACCEL_DLPFCFG
        dev->reg_val_cache |= (dlpf << 3);
    }
    return icm20948_write_register8(dev, 2, ICM20948_ACCEL_CONFIG, dev->reg_val_cache);
}

int icm20948_enable_acc(icm20948_dev_t *dev, bool enable) {
    uint8_t val;
    int ret = icm20948_read_register8(dev, 0, ICM20948_PWR_MGMT_2, &val);
    if (ret) return ret;

    if (enable) {
        val &= ~(BIT(3) | BIT(4) | BIT(5));   // 啟用加速度計 XYZ
    } else {
        val |=  (BIT(3) | BIT(4) | BIT(5));   // 停用加速度計 XYZ
    }
    return icm20948_write_register8(dev, 0, ICM20948_PWR_MGMT_2, val);
}

int icm20948_enable_gyr(icm20948_dev_t *dev, bool enable) {
    uint8_t val;
    int ret = icm20948_read_register8(dev, 0, ICM20948_PWR_MGMT_2, &val);
    if (ret) return ret;

    if (enable) {
        val &= ~(BIT(0) | BIT(1) | BIT(2));   // 啟用陀螺儀 XYZ
    } else {
        val |=  (BIT(0) | BIT(1) | BIT(2));   // 停用陀螺儀 XYZ
    }
    return icm20948_write_register8(dev, 0, ICM20948_PWR_MGMT_2, val);
}

// 一次關閉 Accel + Gyro（XYZ 全停用）
int icm20948_disable_accel_and_gyro(icm20948_dev_t *dev) {
    return icm20948_write_register8(dev, 0, ICM20948_PWR_MGMT_2, 0x3F);
}