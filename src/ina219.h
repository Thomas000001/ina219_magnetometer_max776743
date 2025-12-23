#ifndef INA219_H_
#define INA219_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

// INA219寄存器地址
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE   0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

// INA219預設I2C地址
#define INA219_ADDR_DEFAULT     0x40

// 配置寄存器位定義
#define INA219_CONFIG_RESET     0x8000

// ADC解析度/平均設定
typedef enum {
    INA219_ADC_9BIT         = 0x0000,  // 9-bit, 1 sample, 84us
    INA219_ADC_10BIT        = 0x0001,  // 10-bit, 1 sample, 148us
    INA219_ADC_11BIT        = 0x0002,  // 11-bit, 1 sample, 276us
    INA219_ADC_12BIT        = 0x0003,  // 12-bit, 1 sample, 532us
    INA219_ADC_12BIT_2S     = 0x0009,  // 12-bit, 2 samples, 1.06ms
    INA219_ADC_12BIT_4S     = 0x000A,  // 12-bit, 4 samples, 2.13ms
    INA219_ADC_12BIT_8S     = 0x000B,  // 12-bit, 8 samples, 4.26ms
    INA219_ADC_12BIT_16S    = 0x000C,  // 12-bit, 16 samples, 8.51ms
    INA219_ADC_12BIT_32S    = 0x000D,  // 12-bit, 32 samples, 17.02ms
    INA219_ADC_12BIT_64S    = 0x000E,  // 12-bit, 64 samples, 34.05ms
    INA219_ADC_12BIT_128S   = 0x000F   // 12-bit, 128 samples, 68.10ms
} ina219_adc_mode_t;

// PGA增益設定
typedef enum {
    INA219_GAIN_40MV        = 0x0000,  // 40mV Range 最高靈敏度
    INA219_GAIN_80MV        = 0x0800,  // 80mV Range
    INA219_GAIN_160MV       = 0x1000,  // 160mV Range
    INA219_GAIN_320MV       = 0x1800   // 320mV Range 最大量程
} ina219_gain_t;

// 匯流排電壓範圍
typedef enum {
    INA219_BUSRANGE_16V     = 0x0000,  // 16V Range
    INA219_BUSRANGE_32V     = 0x2000   // 32V Range
} ina219_busrange_t;

// 操作模式
typedef enum {
    INA219_MODE_POWERDOWN           = 0x0000,
    INA219_MODE_SHUNT_TRIGGERED     = 0x0001,
    INA219_MODE_BUS_TRIGGERED       = 0x0002,
    INA219_MODE_SHUNT_BUS_TRIGGERED = 0x0003,
    INA219_MODE_ADC_OFF             = 0x0004,
    INA219_MODE_SHUNT_CONTINUOUS    = 0x0005,
    INA219_MODE_BUS_CONTINUOUS      = 0x0006,
    INA219_MODE_SHUNT_BUS_CONTINUOUS = 0x0007
} ina219_mode_t;

// INA219裝置結構
struct ina219_data {
    const struct device *i2c_dev;
    uint8_t addr;
    uint16_t config;
    uint16_t cal_value;
    float current_lsb;
    float power_lsb;
    float shunt_ohms;
};

// 函數原型
int ina219_init(struct ina219_data *data, const struct device *i2c_dev, uint8_t addr);
int ina219_reset(struct ina219_data *data);
int ina219_set_calibration(struct ina219_data *data, float shunt_ohms, float max_expected_current);
int ina219_set_adc_mode(struct ina219_data *data, ina219_adc_mode_t shunt_adc, ina219_adc_mode_t bus_adc);
int ina219_set_mode(struct ina219_data *data, ina219_mode_t mode);
int ina219_set_bus_range(struct ina219_data *data, ina219_busrange_t range);
int ina219_set_gain(struct ina219_data *data, ina219_gain_t gain);
float ina219_read_current(struct ina219_data *data);
float ina219_read_bus_voltage(struct ina219_data *data);
float ina219_read_shunt_voltage(struct ina219_data *data);
float ina219_read_power(struct ina219_data *data);

#endif /* INA219_H_ */