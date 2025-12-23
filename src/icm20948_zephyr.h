#ifndef ICM20948_ZEPHYR_H_
#define ICM20948_ZEPHYR_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h> // For BIT macros if needed
#include <stdbool.h>
#include "xyzFloat_c.h" // 使用轉換後的 xyzFloat_c.h

/* Enums from original library (can be kept as is) */
typedef enum {
    ICM20948_NO_CYCLE              = 0x00,
    ICM20948_GYR_CYCLE             = 0x10,
    ICM20948_ACC_CYCLE             = 0x20,
    ICM20948_ACC_GYR_CYCLE         = 0x30,
    ICM20948_ACC_GYR_I2C_MST_CYCLE = 0x70
} icm20948_cycle_t;

typedef enum {
    ICM20948_ACT_HIGH, ICM20948_ACT_LOW
} icm20948_int_pin_pol_t;

typedef enum {
    ICM20948_FSYNC_INT      = 0x01,
    ICM20948_WOM_INT        = 0x02,
    ICM20948_DMP_INT        = 0x04,
    ICM20948_DATA_READY_INT = 0x08,
    ICM20948_FIFO_OVF_INT   = 0x10,
    ICM20948_FIFO_WM_INT    = 0x20
} icm20948_int_type_t;

typedef enum {
    ICM20948_FIFO_ACC        = 0x10,
    ICM20948_FIFO_GYR        = 0x0E,
    ICM20948_FIFO_ACC_GYR    = 0x1E
} icm20948_fifo_type_t;

typedef enum {
    ICM20948_CONTINUOUS, ICM20948_STOP_WHEN_FULL
} icm20948_fifo_mode_t;

typedef enum {
    ICM20948_GYRO_RANGE_250, ICM20948_GYRO_RANGE_500, ICM20948_GYRO_RANGE_1000, ICM20948_GYRO_RANGE_2000
} icm20948_gyro_range_t;

typedef enum {
    ICM20948_DLPF_0, ICM20948_DLPF_1, ICM20948_DLPF_2, ICM20948_DLPF_3, ICM20948_DLPF_4, ICM20948_DLPF_5,
    ICM20948_DLPF_6, ICM20948_DLPF_7, ICM20948_DLPF_OFF
} icm20948_dlpf_t;

typedef enum {
    ICM20948_GYR_AVG_1, ICM20948_GYR_AVG_2, ICM20948_GYR_AVG_4, ICM20948_GYR_AVG_8, ICM20948_GYR_AVG_16,
    ICM20948_GYR_AVG_32, ICM20948_GYR_AVG_64, ICM20948_GYR_AVG_128
} icm20948_gyro_avg_low_power_t;

typedef enum {
    ICM20948_ACC_RANGE_2G, ICM20948_ACC_RANGE_4G, ICM20948_ACC_RANGE_8G, ICM20948_ACC_RANGE_16G
} icm20948_acc_range_t;

typedef enum {
    ICM20948_ACC_AVG_4, ICM20948_ACC_AVG_8, ICM20948_ACC_AVG_16, ICM20948_ACC_AVG_32
} icm20948_acc_avg_low_power_t;

typedef enum {
    ICM20948_WOM_COMP_DISABLE, ICM20948_WOM_COMP_ENABLE
} icm20948_wom_comp_en_t;

typedef enum {
    AK09916_PWR_DOWN           = 0x00,
    AK09916_TRIGGER_MODE       = 0x01,
    AK09916_CONT_MODE_10HZ     = 0x02,
    AK09916_CONT_MODE_20HZ     = 0x04,
    AK09916_CONT_MODE_50HZ     = 0x06,
    AK09916_CONT_MODE_100HZ    = 0x08
} ak09916_op_mode_t;

typedef enum {
  ICM20948_FLAT, ICM20948_FLAT_1, ICM20948_XY, ICM20948_XY_1, ICM20948_YX, ICM20948_YX_1
} icm20948_orientation_t;


/* Register definitions (from original .h, using #define) */
#define AK09916_ADDRESS                0x0C

/* Registers ICM20948 USER BANK 0*/
#define ICM20948_WHO_AM_I              0x00
#define ICM20948_USER_CTRL             0x03
#define ICM20948_LP_CONFIG             0x05
#define ICM20948_PWR_MGMT_1            0x06
#define ICM20948_PWR_MGMT_2            0x07
#define ICM20948_INT_PIN_CFG           0x0F
#define ICM20948_INT_ENABLE            0x10
#define ICM20948_INT_ENABLE_1          0x11
#define ICM20948_INT_ENABLE_2          0x12
#define ICM20948_INT_ENABLE_3          0x13
#define ICM20948_I2C_MST_STATUS        0x17
#define ICM20948_INT_STATUS            0x19
#define ICM20948_INT_STATUS_1          0x1A
#define ICM20948_INT_STATUS_2          0x1B
#define ICM20948_INT_STATUS_3          0x1C
#define ICM20948_DELAY_TIME_H          0x28
#define ICM20948_DELAY_TIME_L          0x29
#define ICM20948_ACCEL_OUT             0x2D // accel data registers begin
#define ICM20948_GYRO_OUT              0x33 // gyro data registers begin
#define ICM20948_TEMP_OUT              0x39
#define ICM20948_EXT_SLV_SENS_DATA_00  0x3B
#define ICM20948_FIFO_EN_1             0x66 // 新增
#define ICM20948_FIFO_EN_2             0x67 // 新增
#define ICM20948_FIFO_RST              0x68 // 新增
#define ICM20948_FIFO_MODE             0x69 // 新增
#define ICM20948_FIFO_COUNT            0x70 // 新增
#define ICM20948_FIFO_R_W              0x72 // 新增
#define ICM20948_DATA_RDY_STATUS       0x74 // 新增
#define ICM20948_FIFO_CFG              0x76 // 新增
// ... (add all other register definitions from the original ICM20948_WE.h) ...
#define ICM20948_REG_BANK_SEL          0x7F

/* Registers ICM20948 USER BANK 1*/ // 新增區塊
#define ICM20948_SELF_TEST_X_GYRO      0x02
#define ICM20948_SELF_TEST_Y_GYRO      0x03
#define ICM20948_SELF_TEST_Z_GYRO      0x04
#define ICM20948_SELF_TEST_X_ACCEL     0x0E
#define ICM20948_SELF_TEST_Y_ACCEL     0x0F
#define ICM20948_SELF_TEST_Z_ACCEL     0x10
#define ICM20948_XA_OFFS_H             0x14
#define ICM20948_XA_OFFS_L             0x15
#define ICM20948_YA_OFFS_H             0x17
#define ICM20948_YA_OFFS_L             0x18
#define ICM20948_ZA_OFFS_H             0x1A
#define ICM20948_ZA_OFFS_L             0x1B
#define ICM20948_TIMEBASE_CORR_PLL     0x28

/* Registers ICM20948 USER BANK 2*/
#define ICM20948_GYRO_SMPLRT_DIV       0x00 // 新增
#define ICM20948_GYRO_CONFIG_1         0x01 // <<< 錯誤點
#define ICM20948_GYRO_CONFIG_2         0x02 // 新增
#define ICM20948_XG_OFFS_USRH          0x03 // 新增
#define ICM20948_XG_OFFS_USRL          0x04 // 新增
#define ICM20948_YG_OFFS_USRH          0x05 // 新增
#define ICM20948_YG_OFFS_USRL          0x06 // 新增
#define ICM20948_ZG_OFFS_USRH          0x07 // 新增
#define ICM20948_ZG_OFFS_USRL          0x08 // 新增
#define ICM20948_ODR_ALIGN_EN          0x09 // <<< 錯誤點
#define ICM20948_ACCEL_SMPLRT_DIV_1    0x10 // 新增
#define ICM20948_ACCEL_SMPLRT_DIV_2    0x11 // 新增
#define ICM20948_ACCEL_INTEL_CTRL      0x12 // 新增
#define ICM20948_ACCEL_WOM_THR         0x13 // 新增
#define ICM20948_ACCEL_CONFIG          0x14 // <<< 錯誤點
#define ICM20948_ACCEL_CONFIG_2        0x15 // 新增
#define ICM20948_FSYNC_CONFIG          0x52 // 新增
#define ICM20948_TEMP_CONFIG           0x53 // 新增
#define ICM20948_MOD_CTRL_USR          0x54 // 新增

/* Registers ICM20948 USER BANK 3*/
#define ICM20948_I2C_MST_ODR_CFG       0x00 // 新增
#define ICM20948_I2C_MST_CTRL          0x01 // <<< 錯誤點
#define ICM20948_I2C_MST_DELAY_CTRL    0x02 // 新增
#define ICM20948_I2C_SLV0_ADDR         0x03 // <<< 錯誤點
#define ICM20948_I2C_SLV0_REG          0x04 // <<< 錯誤點
#define ICM20948_I2C_SLV0_CTRL         0x05 // <<< 錯誤點
#define ICM20948_I2C_SLV0_DO           0x06 // 新增
#define ICM20948_I2C_SLV4_ADDR         0x13 // <<< 錯誤點
#define ICM20948_I2C_SLV4_REG          0x14 // <<< 錯誤點
#define ICM20948_I2C_SLV4_CTRL         0x15 // <<< 錯誤點
#define ICM20948_I2C_SLV4_DO           0x16 // <<< 錯誤點
#define ICM20948_I2C_SLV4_DI           0x17 // <<< 錯誤點

/* Registers AK09916 */
#define AK09916_WIA_1                  0x00
#define AK09916_WIA_2                  0x01 // 新增
#define AK09916_CNTL_2                 0x31
#define AK09916_CNTL_3                 0x32
#define AK09916_STATUS_1               0x10
#define AK09916_HXL                    0x11
#define AK09916_HXH                    0x12 // 新增
#define AK09916_HYL                    0x13 // 新增
#define AK09916_HYH                    0x14 // 新增
#define AK09916_HZL                    0x15 // 新增
#define AK09916_HZH                    0x16 // 新增
#define AK09916_STATUS_2               0x18
// ... (add all other AK09916 register definitions)

/* Register Bits */
#define ICM20948_RESET                 0x80
#define ICM20948_I2C_MST_EN            0x20
#define ICM20948_SLEEP_BIT             0x40 // Renamed from ICM20948_SLEEP to avoid conflict with k_sleep
#define ICM20948_LP_EN                 0x20
#define ICM20948_BYPASS_EN             0x02
// ... (add all other register bit definitions) ...
#define ICM20948_I2C_SLVX_EN           0x80
#define AK09916_READ                   0x80
#define ICM20948_I2C_MST_RST_BIT       0x02 // Renamed

/* Others */
#define AK09916_WHO_AM_I_1_VAL         0x4809 // Renamed from AK09916_WHO_AM_I_1
#define AK09916_WHO_AM_I_2_VAL         0x0948 // Renamed from AK09916_WHO_AM_I_2
#define ICM20948_WHO_AM_I_CONTENT      0xEA
#define ICM20948_ROOM_TEMP_OFFSET      0.0f
#define ICM20948_T_SENSITIVITY         333.87f
#define AK09916_MAG_LSB                0.1495f



// Structure to hold device data and configuration (replaces the C++ class)
typedef struct {
    union { // Union for I2C or SPI device specs
        struct i2c_dt_spec i2c;
        struct spi_dt_spec spi;
    } bus_cfg;
    bool use_spi;

    uint8_t current_bank;
    uint8_t buffer[20]; // For readAllData
    xyzFloat_t acc_offset_val;
    xyzFloat_t acc_corr_factor;
    xyzFloat_t gyr_offset_val;
    float acc_range_factor; // Store calculated factor instead of enum value
    float gyr_range_factor; // Store calculated factor instead of enum value
    icm20948_fifo_type_t fifo_type;

    // Add any other members that were part of the C++ class state
    uint8_t reg_val_cache; // For regVal in original code

} icm20948_dev_t;

/* Public function prototypes (C equivalent of public methods) */

// Initialization
int icm20948_init(icm20948_dev_t *dev, const struct device *bus_dev, bool is_spi, uint16_t addr_or_cs_gpio_pin);
// If SPI, addr_or_cs_gpio_pin is the CS GPIO pin number. This is a simplified init.
// A more robust SPI init would take a full spi_dt_spec or allow more spi_config parameters.
// For I2C, it's the I2C address.

// Basic settings
int icm20948_auto_offsets(icm20948_dev_t *dev);
int icm20948_set_acc_offsets_minmax(icm20948_dev_t *dev, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
int icm20948_set_acc_offsets_struct(icm20948_dev_t *dev, xyzFloat_t offset);
xyzFloat_t icm20948_get_acc_offsets(icm20948_dev_t *dev);
int icm20948_set_gyr_offsets_values(icm20948_dev_t *dev, float x_offset, float y_offset, float z_offset);
int icm20948_set_gyr_offsets_struct(icm20948_dev_t *dev, xyzFloat_t offset);
xyzFloat_t icm20948_get_gyr_offsets(icm20948_dev_t *dev);
int icm20948_who_am_i(icm20948_dev_t *dev, uint8_t *value);
int icm20948_enable_acc(icm20948_dev_t *dev, bool enable);
int icm20948_set_acc_range(icm20948_dev_t *dev, icm20948_acc_range_t acc_range);
int icm20948_set_acc_dlpf(icm20948_dev_t *dev, icm20948_dlpf_t dlpf);
int icm20948_set_acc_sample_rate_divider(icm20948_dev_t *dev, uint16_t acc_spl_rate_div);
int icm20948_enable_gyr(icm20948_dev_t *dev, bool enable);
int icm20948_set_gyr_range(icm20948_dev_t *dev, icm20948_gyro_range_t gyro_range);
int icm20948_set_gyr_dlpf(icm20948_dev_t *dev, icm20948_dlpf_t dlpf);
int icm20948_set_gyr_sample_rate_divider(icm20948_dev_t *dev, uint8_t gyr_spl_rate_div);
int icm20948_set_temp_dlpf(icm20948_dev_t *dev, icm20948_dlpf_t dlpf);
int icm20948_set_i2c_mst_sample_rate(icm20948_dev_t *dev, uint8_t rate_exp);
// setSPIClockSpeed would be part of spi_config in Zephyr, handled at init for spi_dt_spec

// x,y,z results
int icm20948_read_sensor_data(icm20948_dev_t *dev); // Renamed from readSensor to avoid conflict, reads all data to dev->buffer
int icm20948_get_acc_raw_values(icm20948_dev_t *dev, xyzFloat_t *acc_raw_val);
int icm20948_get_corrected_acc_raw_values(icm20948_dev_t *dev, xyzFloat_t *corr_acc_raw_val);
int icm20948_get_g_values(icm20948_dev_t *dev, xyzFloat_t *g_val);
// FIFO functions would need more significant rewrite for Zephyr, potentially using interrupts
// For now, let's assume direct read functions
int icm20948_get_acc_raw_values_from_fifo(icm20948_dev_t *dev, xyzFloat_t *acc_raw_val);
int icm20948_get_corrected_acc_raw_values_from_fifo(icm20948_dev_t *dev, xyzFloat_t *acc_raw_val);
int icm20948_get_g_values_from_fifo(icm20948_dev_t *dev, xyzFloat_t *g_val);

float icm20948_get_resultant_g(xyzFloat_t *g_val); // This can be a utility function not needing *dev
int icm20948_get_temperature(icm20948_dev_t *dev, float *temp);
int icm20948_get_gyr_raw_values(icm20948_dev_t *dev, xyzFloat_t *gyr_raw_val);
int icm20948_get_corrected_gyr_raw_values(icm20948_dev_t *dev, xyzFloat_t *corr_gyr_val);
int icm20948_get_gyr_values(icm20948_dev_t *dev, xyzFloat_t *gyr_val);
int icm20948_get_gyr_values_from_fifo(icm20948_dev_t *dev, xyzFloat_t *gyr_val);
int icm20948_get_mag_values(icm20948_dev_t *dev, xyzFloat_t *mag_val);


// Angles and Orientation
int icm20948_get_angles(icm20948_dev_t *dev, xyzFloat_t *angle_val);
icm20948_orientation_t icm20948_get_orientation(icm20948_dev_t *dev);
const char* icm20948_get_orientation_as_string(icm20948_dev_t *dev); // Returns const char*
float icm20948_get_pitch(icm20948_dev_t *dev);
float icm20948_get_roll(icm20948_dev_t *dev);


// Power, Sleep, Standby
int icm20948_enable_cycle(icm20948_dev_t *dev, icm20948_cycle_t cycle);
int icm20948_enable_low_power(icm20948_dev_t *dev, bool en_lp);
int icm20948_set_gyr_average_in_cycle_mode(icm20948_dev_t *dev, icm20948_gyro_avg_low_power_t avg);
int icm20948_set_acc_average_in_cycle_mode(icm20948_dev_t *dev, icm20948_acc_avg_low_power_t avg);
int icm20948_set_sleep_mode(icm20948_dev_t *dev, bool sleep_en); // Renamed from sleep


// Interrupts
int icm20948_set_int_pin_polarity(icm20948_dev_t *dev, icm20948_int_pin_pol_t pol);
int icm20948_enable_int_latch(icm20948_dev_t *dev, bool latch);
int icm20948_enable_clear_int_by_any_read(icm20948_dev_t *dev, bool clear_by_any_read);
int icm20948_set_fsync_int_polarity(icm20948_dev_t *dev, icm20948_int_pin_pol_t pol);
int icm20948_enable_interrupt(icm20948_dev_t *dev, icm20948_int_type_t int_type);
int icm20948_disable_interrupt(icm20948_dev_t *dev, icm20948_int_type_t int_type);
int icm20948_read_and_clear_interrupts(icm20948_dev_t *dev, uint8_t *int_source);
bool icm20948_check_interrupt(uint8_t source, icm20948_int_type_t type); // Utility
int icm20948_set_wake_on_motion_threshold(icm20948_dev_t *dev, uint8_t wom_thresh, icm20948_wom_comp_en_t wom_comp_en);


// FIFO
int icm20948_enable_fifo(icm20948_dev_t *dev, bool fifo_en);
int icm20948_set_fifo_mode(icm20948_dev_t *dev, icm20948_fifo_mode_t mode);
int icm20948_start_fifo(icm20948_dev_t *dev, icm20948_fifo_type_t fifo_type);
int icm20948_stop_fifo(icm20948_dev_t *dev);
int icm20948_reset_fifo(icm20948_dev_t *dev);
int icm20948_get_fifo_count(icm20948_dev_t *dev, int16_t *count);
int icm20948_get_number_of_fifo_data_sets(icm20948_dev_t *dev, int16_t *num_sets);
int icm20948_find_fifo_begin(icm20948_dev_t *dev);


// Magnetometer (AK09916 specific, accessed via ICM20948 I2C Master)
int icm20948_init_magnetometer(icm20948_dev_t *dev);
int icm20948_who_am_i_mag(icm20948_dev_t *dev, uint16_t *mag_id);
int icm20948_set_mag_op_mode(icm20948_dev_t *dev, ak09916_op_mode_t op_mode);
int icm20948_reset_mag(icm20948_dev_t *dev);


#endif // ICM20948_ZEPHYR_H_