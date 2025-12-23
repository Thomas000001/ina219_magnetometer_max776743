#include "ina219.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ina219, LOG_LEVEL_DBG);

// 寫入16位元寄存器
static int ina219_write_reg(struct ina219_data *data, uint8_t reg, uint16_t val) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (val >> 8) & 0xFF;  // MSB
    buf[2] = val & 0xFF;          // LSB
    
    return i2c_write(data->i2c_dev, buf, sizeof(buf), data->addr);
}

// 讀取16位元寄存器
static int ina219_read_reg(struct ina219_data *data, uint8_t reg, uint16_t *val) {
    uint8_t tx_buf[1] = {reg};
    uint8_t rx_buf[2];
    int ret;
    
    // 發送寄存器地址
    ret = i2c_write(data->i2c_dev, tx_buf, 1, data->addr);
    if (ret < 0) {
        LOG_ERR("Failed to write register address: %d", ret);
        return ret;
    }
    
    // 讀取數據
    ret = i2c_read(data->i2c_dev, rx_buf, 2, data->addr);
    if (ret < 0) {
        LOG_ERR("Failed to read register data: %d", ret);
        return ret;
    }
    
    // 組合MSB和LSB
    *val = (rx_buf[0] << 8) | rx_buf[1];
    return 0;
}

// 初始化INA219
int ina219_init(struct ina219_data *data, const struct device *i2c_dev, uint8_t addr) {
    if (!i2c_dev) {
        LOG_ERR("I2C device is NULL");
        return -EINVAL;
    }
    
    data->i2c_dev = i2c_dev;
    data->addr = addr;
    
    // 重置裝置
    int ret = ina219_reset(data);
    if (ret < 0) {
        LOG_ERR("Failed to reset INA219");
        return ret;
    }
    
    // 等待重置完成
    k_msleep(1);
    
    // 設定預設配置
    data->config = INA219_BUSRANGE_32V |
                   INA219_GAIN_320MV |
                   INA219_ADC_12BIT |
                   (INA219_ADC_12BIT << 3) |
                   INA219_MODE_SHUNT_BUS_CONTINUOUS;
    
    ret = ina219_write_reg(data, INA219_REG_CONFIG, data->config);
    if (ret < 0) {
        LOG_ERR("Failed to write config register");
        return ret;
    }
    
    LOG_INF("INA219 initialized at address 0x%02x", addr);
    return 0;
}

// 重置INA219
int ina219_reset(struct ina219_data *data) {
    return ina219_write_reg(data, INA219_REG_CONFIG, INA219_CONFIG_RESET);
}

// 設定校準值 - 修正浮點數警告
int ina219_set_calibration(struct ina219_data *data, float shunt_ohms, float max_expected_current) {
    data->shunt_ohms = shunt_ohms;
    
    // 計算校準值 - 使用 float 常數 (加 f 後綴)
    // Current_LSB = Maximum Expected Current / 32768
    data->current_lsb = max_expected_current / 32768.0f;  // 使用 float 常數
    
    // Cal = 0.04096 / (Current_LSB * Rshunt)
    data->cal_value = (uint16_t)(0.04096f / (data->current_lsb * shunt_ohms));  // 使用 float 常數
    
    // Power_LSB = 20 * Current_LSB
    data->power_lsb = 20.0f * data->current_lsb;  // 使用 float 常數
    
    // 修正 LOG_INF 的浮點數格式問題
    LOG_INF("Calibration: shunt=%.4f ohms, cal_value=%u, current_lsb=%.6f", 
            (double)shunt_ohms, data->cal_value, (double)data->current_lsb);
    
    return ina219_write_reg(data, INA219_REG_CALIBRATION, data->cal_value);
}

// 設定ADC模式
int ina219_set_adc_mode(struct ina219_data *data, ina219_adc_mode_t shunt_adc, ina219_adc_mode_t bus_adc) {
    // 清除ADC位元
    data->config &= ~(0x0780);  // 清除BADC (bits 7-10)
    data->config &= ~(0x0078);  // 清除SADC (bits 3-6)
    
    // 設定新的ADC模式
    data->config |= ((bus_adc & 0x0F) << 7);    // BADC
    data->config |= ((shunt_adc & 0x0F) << 3);  // SADC
    
    LOG_DBG("Setting ADC mode: shunt=0x%02x, bus=0x%02x, config=0x%04x", 
            shunt_adc, bus_adc, data->config);
    
    return ina219_write_reg(data, INA219_REG_CONFIG, data->config);
}

// 設定操作模式
int ina219_set_mode(struct ina219_data *data, ina219_mode_t mode) {
    data->config &= ~0x0007;  // 清除MODE位元
    data->config |= (mode & 0x0007);
    
    return ina219_write_reg(data, INA219_REG_CONFIG, data->config);
}

// 設定匯流排電壓範圍
int ina219_set_bus_range(struct ina219_data *data, ina219_busrange_t range) {
    data->config &= ~0x2000;  // 清除BRNG位元
    data->config |= (range & 0x2000);
    
    return ina219_write_reg(data, INA219_REG_CONFIG, data->config);
}

// 設定PGA增益
int ina219_set_gain(struct ina219_data *data, ina219_gain_t gain) {
    data->config &= ~0x1800;  // 清除PG位元
    data->config |= (gain & 0x1800);
    
    return ina219_write_reg(data, INA219_REG_CONFIG, data->config);
}

// 讀取電流（mA）- 修正浮點數警告
float ina219_read_current(struct ina219_data *data) {
    uint16_t raw;
    int ret = ina219_read_reg(data, INA219_REG_CURRENT, &raw);
    if (ret < 0) {
        LOG_ERR("Failed to read current register");
        return 0.0f;
    }
    
    // 轉換為mA - 使用 float 常數
    return (int16_t)raw * data->current_lsb * 1000.0f;
}

// 讀取匯流排電壓（V）
float ina219_read_bus_voltage(struct ina219_data *data) {
    uint16_t raw;
    int ret = ina219_read_reg(data, INA219_REG_BUSVOLTAGE, &raw);
    if (ret < 0) {
        LOG_ERR("Failed to read bus voltage register");
        return 0.0f;
    }
    
    // 位元1是CNVR（轉換就緒）
    // 位元0是OVF（溢位）
    // 實際電壓值在位元3-15
    return ((raw >> 3) * 4) / 1000.0f;  // LSB = 4mV, 使用 float 常數
}

// 讀取分流電壓（mV）
float ina219_read_shunt_voltage(struct ina219_data *data) {
    uint16_t raw;
    int ret = ina219_read_reg(data, INA219_REG_SHUNTVOLTAGE, &raw);
    if (ret < 0) {
        LOG_ERR("Failed to read shunt voltage register");
        return 0.0f;
    }
    
    // LSB = 10uV
    return (int16_t)raw * 0.01f;  // 轉換為mV, 使用 float 常數
}

// 讀取功率（mW）- 修正浮點數警告
float ina219_read_power(struct ina219_data *data) {
    uint16_t raw;
    int ret = ina219_read_reg(data, INA219_REG_POWER, &raw);
    if (ret < 0) {
        LOG_ERR("Failed to read power register");
        return 0.0f;
    }
    
    return raw * data->power_lsb * 1000.0f;  // 轉換為mW, 使用 float 常數
}