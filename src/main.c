

// /*20251217先將icm20948的執行緒刪除，單獨從ina219來判斷週期，再加上max77643控制電壓*/
// #include <zephyr/kernel.h>
// #include <zephyr/drivers/i2c.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/logging/log.h>
// #include <stdio.h>
// #include <math.h>

// // 包含新的驅動程式標頭檔
// #include "icm20948_zephyr.h"
// #include "ina219.h"  // 新的 INA219 驅動程式

// LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// /* 驅動程式實例 */
// static struct ina219_data my_ina219;  // 新的 INA219 資料結構

// /* 传感器数据刷新时间 */
// #define SENSOR_SAMPLE_INTERVAL K_MSEC(1)

// /* INA219 配置參數 */
// #define INA219_SHUNT_RESISTANCE    0.1f    // 0.1歐姆分流電阻
// #define INA219_MAX_EXPECTED_CURRENT 0.5f   // 最大預期電流500mA
// #define INA219_I2C_ADDRESS         0x40    // INA219 I2C 地址

// /* 執行緒配置 */
// #define INA219_STACK_SIZE 1024
// #define INA219_PRIORITY 7

// K_THREAD_STACK_DEFINE(ina219_stack, INA219_STACK_SIZE);
// struct k_thread ina219_thread_data;

// K_MUTEX_DEFINE(uart_mutex);
// K_MUTEX_DEFINE(i2c_mutex);

// /* 移动平均窗口大小 */
// #define CURRENT_FILTER_SIZE 8
// #define SLOPE_FILTER_SIZE 5
// #define MAG_MOVING_AVG_SIZE 5

// /* 共享数据结构 */
// struct shared_data {
//     float current;
//     float filtered_mag_x;
//     float voltage;
//     float mag_sample_time;
//     float cur_sample_time;
// };

// static struct shared_data motor_data;
// static K_MUTEX_DEFINE(motor_data_mutex);

// /*sliding window*/
// /* 移动平均器结构体 */
// typedef struct {
//     float values[CURRENT_FILTER_SIZE];
//     int index;
//     int count;
//     float sum;
// } MovingAverage;

// /* 初始化移动平均器 */
// static void moving_average_init(MovingAverage *ma, int window_size) {
//     ma->index = 0;
//     ma->count = 0;
//     ma->sum = 0.0f;
//     for (int i = 0; i < window_size && i < CURRENT_FILTER_SIZE; i++) {
//         ma->values[i] = 0.0f;
//     }
// }

// /* 添加值到移动平均器并返回平均值 */
// static float moving_average_add(MovingAverage *ma, float value, int window_size) {
//     if (window_size > CURRENT_FILTER_SIZE) window_size = CURRENT_FILTER_SIZE;

//     if (ma->count == window_size) {
//         ma->sum -= ma->values[ma->index];
//     } else {
//         ma->count++;
//     }

//     ma->values[ma->index] = value;
//     ma->sum += value;
//     ma->index = (ma->index + 1) % window_size;

//     return ma->sum / ma->count;
// }

// /* 一階低通濾波器結構體 */
// typedef struct {
//     float prev_output;     // Y(n-1) 上一次的輸出值
//     float alpha;          // 濾波係數 α (0 < α <= 1)
//     int initialized;      // 是否已初始化標誌
// } FirstOrderFilter;

// /* 數據濾波器 */
// static FirstOrderFilter mag_x_filter = {
//     .prev_output = 0.0f,
//     .alpha = 0.25f,        // 預設α值，可根據需求調整
//     .initialized = 0
// };

// static FirstOrderFilter mag_y_filter = {
//     .prev_output = 0.0f,
//     .alpha = 0.1f,
//     .initialized = 0
// };

// static FirstOrderFilter mag_z_filter = {
//     .prev_output = 0.0f,
//     .alpha = 0.1f,
//     .initialized = 0
// };

// /* 初始化一階濾波器 */
// static void first_order_filter_init(FirstOrderFilter *filter, float alpha) {
//     filter->prev_output = 0.0f;
//     filter->alpha = alpha;
//     filter->initialized = 0;
// }

// /* 設置濾波係數 */
// static void first_order_filter_set_alpha(FirstOrderFilter *filter, float alpha) {
//     // 確保 α 在有效範圍內
//     if (alpha > 1.0f) alpha = 1.0f;
//     if (alpha < 0.0f) alpha = 0.0f;
//     filter->alpha = alpha;
// }

// /* 應用一階低通濾波 */
// static float first_order_filter_apply(FirstOrderFilter *filter, float input) {
//     float output;
    
//     // 如果是第一次運行，直接使用輸入值作為初始值
//     if (!filter->initialized) {
//         filter->prev_output = input;
//         filter->initialized = 1;
//         return input;
//     }
    
//     // 應用一階濾波公式: Y(n) = αX(n) + (1-α)Y(n-1)
//     output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    
//     // 更新上一次的輸出值
//     filter->prev_output = output;
    
//     return output;
// }

// /* 峰值檢測法 */
// typedef enum {
//     STATE_UNKNOWN,
//     STATE_RISING,
//     STATE_FALLING
// } SignalState;

// typedef struct {
//     float threshold;      // 最小變化閾值
//     SignalState state;
//     float last_peak_time;
//     float period;
//     float last_mag;
//     float current_sum;
//     float current_start;
//     float current_end;
//     int current_count;
//     int state_count;
// } PeriodDetector;


// /*週期檢測算法-斜率判斷法*/
// bool detect_period(PeriodDetector *detector, float current_mag, float current_time, float current, float voltage) {
//     float MIN_PEAK_INTERVAL = 0.1f; // 最小峰值間隔時間，避免誤判
//     float diff = current_mag - detector->last_mag;
    
//     // 使用閾值避免噪聲造成的誤判
//     if (fabs(diff) < detector->threshold) {
//         return false;
//     }
    
//     SignalState new_state = (diff > 0) ? STATE_RISING : STATE_FALLING;
//     if (new_state == STATE_FALLING && detector->state == STATE_RISING && detector->state_count == 0) {
//         detector->state_count = 1;
//         detector->last_peak_time = current_time;
//     }
//     else if (new_state == STATE_FALLING && detector->state == STATE_RISING && detector->state_count == 1) {
//         detector->state_count = 2;
//     }
//     if (detector->state_count == 1){
//         if (detector->current_count == 0) {
//             detector->current_start = current;
//         }
//         detector->current_sum += current;
//         detector->current_count++;
//     }
//     else if (detector->state_count == 2){
//         detector->current_end = current;
//         detector->period = current_time - detector->last_peak_time;
//         float average_current = detector->current_sum / detector->current_count;
//         if(detector->period >= MIN_PEAK_INTERVAL){
//             printf("----------Average Current during one period: %.4f mA----------\n", average_current);
//             printf("----------Current Sum: %.4f mA----------\n", detector->current_sum);
//             printf("----------Current Count: %d----------\n", detector->current_count);
//             printf("----------Current Start: %.4f mA----------\n", detector->current_start);
//             printf("----------Current End: %.4f mA----------\n", detector->current_end);
//             printf("----------Period: %.4f s----------\n", detector->period);
//             printf("----------current time: %.4f s----------\n", current_time);
//             printf("----------last peak time: %.4f s----------\n", detector->last_peak_time);
//             printf("----------power: %.4f mW----------\n", voltage * average_current);
//             printf("----------energy: %.4f mJ----------\n", voltage * average_current * detector->period);
//             detector->current_sum = 0.0f;
//             detector->current_count = 0;
//             detector->state_count = 1;  // 重置為1以繼續檢測下一個週期
//             detector->last_peak_time = current_time;
//         }
//         else{
//             detector->state_count = 1;  // 若週期過短，忽略此次檢測，繼續等待下一個峰值
//         }
//     }
//     detector->last_mag = current_mag;
//     detector->state = new_state;
//     return false;
// }

// /* 週期檢測器實例 */
// PeriodDetector cycle_detector = {
//     .threshold = 0.5f,  // 根據磁場變化調整閾值
//     .state = STATE_UNKNOWN,
//     .last_peak_time = 0.0f,
//     .period = 0.0f,
//     .last_mag = 0.0f,
//     .current_sum = 0.0f,
//     .current_start = 0.0f,
//     .current_end = 0.0f,
//     .current_count = 0,
//     .state_count = 0
// };



// /* 初始化 INA219 - 使用新的驅動程式 */
// static int new_ina219_init(void) {
//     int ret;
//     LOG_INF("Starting INA219 initialization");

//     // 獲取 I2C 裝置
//     const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
//     if (!device_is_ready(i2c_dev)) {
//         LOG_ERR("I2C device for INA219 not ready");
//         return -ENODEV;
//     }

//     // 初始化 INA219
//     ret = ina219_init(&my_ina219, i2c_dev, INA219_I2C_ADDRESS);
//     if (ret != 0) {
//         LOG_ERR("Failed to initialize INA219: %d", ret);
//         return ret;
//     }

//     // 設定校準值
//     ret = ina219_set_calibration(&my_ina219, INA219_SHUNT_RESISTANCE, INA219_MAX_EXPECTED_CURRENT);
//     if (ret != 0) {
//         LOG_ERR("Failed to set INA219 calibration: %d", ret);
//         return ret;
//     }

//     // 設定 ADC 模式為 128 樣本平均（最高精度）
//     // 這對於您的 AC/DC 分析很重要
//     ret = ina219_set_adc_mode(&my_ina219, INA219_ADC_9BIT, INA219_ADC_9BIT);
//     if (ret != 0) {
//         LOG_ERR("Failed to set INA219 ADC mode: %d", ret);
//         return ret;
//     }
//     LOG_INF("INA219 ADC mode set to 128-sample averaging");

//     // 設定匯流排電壓範圍為 32V
//     ret = ina219_set_bus_range(&my_ina219, INA219_BUSRANGE_32V);
//     if (ret != 0) {
//         LOG_ERR("Failed to set INA219 bus range: %d", ret);
//         return ret;
//     }

//     // 設定 PGA 增益為 320mV
//     ret = ina219_set_gain(&my_ina219, INA219_GAIN_40MV);
//     if (ret != 0) {
//         LOG_ERR("Failed to set INA219 gain: %d", ret);
//         return ret;
//     }

//     // 設定連續測量模式
//     ret = ina219_set_mode(&my_ina219, INA219_MODE_SHUNT_BUS_CONTINUOUS);
//     if (ret != 0) {
//         LOG_ERR("Failed to set INA219 mode: %d", ret);
//         return ret;
//     }

//     LOG_INF("INA219 initialized successfully");
//     return 0;
// }

// /* INA219 讀取執行緒 - 使用新的驅動程式 */
// void ina219_read_thread_entry(void *arg1, void *arg2, void *arg3) {
//     ARG_UNUSED(arg1);
//     ARG_UNUSED(arg2);
//     ARG_UNUSED(arg3);

//     uint32_t start_time = k_uptime_get_32();
//     MovingAverage current_filter;
//     moving_average_init(&current_filter, CURRENT_FILTER_SIZE);

//     LOG_INF("INA219 thread started");

//     while (1) {
//         uint32_t ina_thread_start = k_uptime_get_32();

//         // 讀取電流值（已包含 I2C 通訊）
//         k_mutex_lock(&i2c_mutex, K_FOREVER);
//         float current_mA = ina219_read_current(&my_ina219);
//         float bus_voltage = ina219_read_bus_voltage(&my_ina219);
//         float shunt_voltage = ina219_read_shunt_voltage(&my_ina219);
//         // float power_mW = ina219_read_power(&my_ina219);
//         k_mutex_unlock(&i2c_mutex);
//         uint32_t current_time_ticks = k_uptime_get_32();

//         // 應用移動平均濾波
//         float filtered_current = moving_average_add(&current_filter, current_mA, CURRENT_FILTER_SIZE);
//         // 應用一階平滑濾波
//         // float double_filtered_current = first_order_filter_apply(&mag_y_filter, filtered_current);

//         // uint32_t current_time_ticks = k_uptime_get_32();
//         float elapsed_seconds = (float)(current_time_ticks - start_time) / 1000.0f;

//         // 更新共享資料
//         k_mutex_lock(&motor_data_mutex, K_FOREVER);
//         motor_data.cur_sample_time = elapsed_seconds;
//         // motor_data.current = double_filtered_current;
//         motor_data.current = filtered_current;
//         motor_data.voltage = bus_voltage + shunt_voltage / 1000.0f; // 將分流電壓轉換為伏特並加到總電壓
//         k_mutex_unlock(&motor_data_mutex);

//         bool detected_period = detect_period(&cycle_detector, filtered_current, elapsed_seconds, filtered_current, motor_data.voltage);
        
//         k_mutex_lock(&uart_mutex, K_FOREVER);
//         // 只輸出電流值以減少 UART 負載
//         printf("T1:%.5f\n"
//                "C:%.4f\n",
//                (double)motor_data.cur_sample_time, 
//                (double)motor_data.current);
//         k_mutex_unlock(&uart_mutex);

//         k_sleep(SENSOR_SAMPLE_INTERVAL);

//         uint32_t ina_thread_end = k_uptime_get_32();
//         float ina_thread_duration = (float)(ina_thread_end - ina_thread_start);
        
//         // 偵錯輸出執行時間（可選）
//         if (ina_thread_duration > 10.0f) { // 只在超過 10ms 時輸出
//             LOG_DBG("INA219 Thread Duration: %.2f ms", (double)ina_thread_duration);
//         }
//     }
// }

// /* 主函數 */
// int main(void) {
//     LOG_INF("System Booting Up...");

//     // 初始化 INA219 - 使用新的驅動程式
//     if (new_ina219_init() != 0) {
//         LOG_ERR("INA219 initialization failed. Halting.");
//         return -1;
//     }

//     // 初始化共享資料結構
//     k_mutex_lock(&motor_data_mutex, K_FOREVER);
//     motor_data.current = 0.0f;
//     motor_data.filtered_mag_x = 0.0f;
//     motor_data.mag_sample_time = 0.0f;
//     motor_data.cur_sample_time = 0.0f;
//     k_mutex_unlock(&motor_data_mutex);

//     // 創建 INA219 執行緒
//     k_tid_t ina_tid = k_thread_create(&ina219_thread_data, 
//                                       ina219_stack, 
//                                       INA219_STACK_SIZE,
//                                       ina219_read_thread_entry, 
//                                       NULL, NULL, NULL,
//                                       INA219_PRIORITY, 
//                                       0, 
//                                       K_NO_WAIT);
//     if (!ina_tid) {
//         LOG_ERR("Failed to create INA219 thread");
//     } else {
//         k_thread_name_set(ina_tid, "ina219_reader");
//     }

//     LOG_INF("System initialization complete. Threads started.");
//     return 0;
// }


/*
 * 整合程式：MAX77643 電壓斜坡控制 + INA219 電流記錄
 * 
 * 功能：
 * - Button 1: 同時啟動 MAX77643 電壓輸出 + INA219 電流記錄
 * - Button 2: 同時停止電壓輸出 + 電流記錄
 * 
 * 日期：20251217
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

// 包含驅動程式標頭檔
#include "ina219.h"
#include "max77643.h"
#include "motor_period_detector.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ============ I2C 設備定義 ============ */
#define MAX77643_NODE DT_NODELABEL(max77643)
static const struct i2c_dt_spec dev_max77643 = I2C_DT_SPEC_GET(MAX77643_NODE);

/* ============ GPIO 定義 ============ */
#define BUTTON1_NODE DT_ALIAS(sw0)
#define BUTTON2_NODE DT_ALIAS(sw1)

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BUTTON1_NODE, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(BUTTON2_NODE, gpios);

#define LED1_NODE DT_ALIAS(led0)
#define LED2_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

/* ============ MAX77643 軟啟動參數 ============ */
#define SOFT_START_VOLTAGE_V    0.8f
#define TARGET_VOLTAGE_V        5.0f
#define SOFT_START_STEP_V       0.2f
#define SOFT_START_DELAY_MS     100

/* ============ 斜坡下降參數 ============ */
#define FINAL_VOLTAGE_V         0.5f
#define VOLTAGE_STEP_V          0.2f
#define STEP_INTERVAL_SEC       15
#define HOLD_TIME_MIN           5

/* ============ INA219 配置參數 ============ */
#define INA219_SHUNT_RESISTANCE    0.1f
#define INA219_MAX_EXPECTED_CURRENT 0.5f
#define INA219_I2C_ADDRESS         0x40
#define SENSOR_SAMPLE_INTERVAL_MS  1

/* ====== 週期檢測配置 ====== */
#define PERIOD_DETECT_METHOD    DETECT_METHOD_PEAK  // 使用峰值檢測法
#define PERIOD_DETECT_MIN_VOLTAGE   5.0f                // 週期檢測啟動電壓閾值 (V)
#define PERIODS_BEFORE_VOLTAGE_DROP 7                   // 每個電壓等級需要檢測的週期數

/* ====== 週期統計配置 ====== */
#define PERIODS_TO_SKIP         2       /* 每個電壓等級要跳過的週期數（第1個） */
#define PERIODS_TO_AVERAGE      7       /* 用於計算平均的週期數（第2~6個） */

/* ====== 電壓穩定等待時間 ====== */
#define VOLTAGE_SETTLE_MIN_MS   100     /* 電壓穩定最小等待時間 (ms) */
#define VOLTAGE_SETTLE_MAX_MS   800    /* 電壓穩定最大等待時間 (ms) */
#define VOLTAGE_SETTLE_PERIODS  1.0f    /* 等待幾個週期的時間 */


/* ============ 執行緒配置 ============ */
#define INA219_STACK_SIZE 2048
#define INA219_PRIORITY 7

K_THREAD_STACK_DEFINE(ina219_stack, INA219_STACK_SIZE);
struct k_thread ina219_thread_data;
static k_tid_t ina219_tid = NULL;

/* ============ 同步與互斥鎖 ============ */
K_MUTEX_DEFINE(uart_mutex);
K_MUTEX_DEFINE(i2c_mutex);
K_MUTEX_DEFINE(period_mutex);  /* 週期計數專用 */

/* ============ 驅動程式實例 ============ */
static max77643_t max77643_device;
static struct ina219_data my_ina219;

/* ====== 週期檢測器實例 ====== */
static motor_period_detector_t period_detector;

/* ============ GPIO 回調結構 ============ */
static struct gpio_callback button1_cb_data;
static struct gpio_callback button2_cb_data;

/* ============ 全域控制旗標 ============ */
static volatile bool request_start = false;   // Button 1 請求開始
static volatile bool request_stop = false;    // Button 2 請求停止
static volatile bool system_running = false;  // 系統運行狀態
static volatile bool ina219_logging_active = false;

/* ====== 追蹤當前 MAX77643 設定的電壓值用於能耗計算 ====== */
static volatile float current_set_voltage = 0.0f;


/* ====== 當前電壓等級的週期計數器（使用 atomic）====== */
static atomic_t current_voltage_period_count = ATOMIC_INIT(0);

static volatile bool period_detect_enabled = false;

/* 週期等待控制變數 */
static volatile int periods_target = 0;           /* 目標週期數 */
static volatile int periods_completed = 0;        /* 已完成的週期數 */
static volatile bool periods_waiting = false;     /* 是否正在等待 */
static volatile bool periods_done = false;        /* 是否達標 */

/* ====== 週期檢測結果緩存（用於同步輸出）====== */
static volatile bool period_result_ready = false;
static period_data_t period_result_cache;

/* ====== 週期統計累積結構 ====== */
typedef struct {
    float period_sum;           /* 週期時間累積 */
    float avg_current_sum;      /* 平均電流累積 */
    float peak_current_sum;     /* 峰值電流累積 */
    float energy_sum;           /* 能量累積 */
    float voltage_sum;          /* 電壓累積 */

    float ac_current_sum;       // 新增20251230：AC 電流累積
    float dc_current_sum;       // 新增20251230：DC 電流累積
    float ac_time_sum;          // 新增20260108：AC 時間累積
    int ac_valid_count;         // 新增20251230：有效 AC 計數
    int dc_valid_count;         // 新增20251230：有效 DC 計數

    int valid_count;            /* 有效週期計數（不含跳過的） */
    int total_count;            /* 總週期計數（含跳過的） */
} period_accumulator_t;

static period_accumulator_t period_accum;

/* ====== 平均結果輸出旗標 ====== */
static volatile bool avg_result_ready = false;
static volatile float avg_result_period = 0.0f;
static volatile float avg_result_current = 0.0f;
static volatile float avg_result_peak = 0.0f;
static volatile float avg_result_energy = 0.0f;
static volatile float avg_result_voltage = 0.0f;
static volatile float avg_result_ac = 0.0f;      // 新增20251230：平均 AC 電流
static volatile float avg_result_dc = 0.0f;      // 新增20251230：平均 DC 電流
static volatile int avg_result_count = 0;
static volatile int avg_result_ac_count = 0;     // 新增20251230：有效 AC 數量
static volatile int avg_result_dc_count = 0;     // 新增20251230：有效 DC 數量
static volatile float avg_result_ac_time = 0.0f;    // 新增20260108：平均 AC 時間

/* ====== 上一個電壓等級的平均週期（用於計算穩定等待時間）====== */
static volatile float last_avg_period_ms = 100.0f;  /* 預設 500ms */


/* 重置累積器 */
static void reset_period_accumulator(void) {
    memset((void*)&period_accum, 0, sizeof(period_accumulator_t));
}

/* 計算並準備平均結果輸出 */
static void prepare_averaged_result(void) {
    if (period_accum.valid_count > 0) {
        avg_result_period = period_accum.period_sum / period_accum.valid_count;
        avg_result_current = period_accum.avg_current_sum / period_accum.valid_count;
        avg_result_peak = period_accum.peak_current_sum / period_accum.valid_count;
        avg_result_energy = period_accum.energy_sum / period_accum.valid_count;
        avg_result_voltage = period_accum.voltage_sum / period_accum.valid_count;
        avg_result_count = period_accum.valid_count;

        /* 新增20251230：計算平均 AC */
        if (period_accum.ac_valid_count > 0) {
            avg_result_ac = period_accum.ac_current_sum / period_accum.ac_valid_count;
            avg_result_ac_time = period_accum.ac_time_sum / period_accum.ac_valid_count;
            avg_result_ac_count = period_accum.ac_valid_count;
        } else {
            avg_result_ac = 0.0f;
            avg_result_ac_time = 0.0f;
            avg_result_ac_count = 0;
        }

        /* 新增20251230：計算平均 DC */
        if (period_accum.dc_valid_count > 0) {
            avg_result_dc = period_accum.dc_current_sum / period_accum.dc_valid_count;
            avg_result_dc_count = period_accum.dc_valid_count;
        } else {
            avg_result_dc = 0.0f;
            avg_result_dc_count = 0;
        }
        

        avg_result_ready = true;
        
        /* 儲存平均週期（轉換為 ms），供下次電壓穩定等待使用 */
        last_avg_period_ms = avg_result_period * 1000.0f;
    }
}
/* ============ 移動平均濾波器 ============ */
#define CURRENT_FILTER_SIZE 8

typedef struct {
    float values[CURRENT_FILTER_SIZE];
    int index;
    int count;
    float sum;
} MovingAverage;

static void moving_average_init(MovingAverage *ma, int window_size) {
    ma->index = 0;
    ma->count = 0;
    ma->sum = 0.0f;
    for (int i = 0; i < window_size && i < CURRENT_FILTER_SIZE; i++) {
        ma->values[i] = 0.0f;
    }
}

static float moving_average_add(MovingAverage *ma, float value, int window_size) {
    if (window_size > CURRENT_FILTER_SIZE) window_size = CURRENT_FILTER_SIZE;

    if (ma->count == window_size) {
        ma->sum -= ma->values[ma->index];
    } else {
        ma->count++;
    }

    ma->values[ma->index] = value;
    ma->sum += value;
    ma->index = (ma->index + 1) % window_size;

    return ma->sum / ma->count;
}

/* ====== 週期完成回調函數 ====== */
void on_period_detected(period_data_t *data) {
    if (!data->valid) return;

    memcpy((void*)&period_result_cache, data, sizeof(period_data_t));
    period_result_ready = true;

    k_mutex_lock(&period_mutex, K_FOREVER);
    if (periods_waiting) {
        periods_completed++;
        
        /* 累積週期數據（跳過前 PERIODS_TO_SKIP 個） */
        period_accum.total_count++;
        
        if (periods_completed > PERIODS_TO_SKIP) {
            /* 第2個週期開始才累積用於平均計算 */
            period_accum.period_sum += data->period_time;
            period_accum.avg_current_sum += data->average_current;
            period_accum.peak_current_sum += data->peak_current;
            period_accum.energy_sum += data->energy;
            period_accum.voltage_sum += data->voltage;
            period_accum.valid_count++;

            /* 累積 AC 值（如果有效） */
            if (data->ac_valid) {
                period_accum.ac_current_sum += data->ac_current;
                period_accum.ac_time_sum += data->ac_time;  // ← 新增20260108：累積 AC 時間
                period_accum.ac_valid_count++;
            }
            
            /* 累積 DC 值（如果有效） */
            if (data->dc_valid) {
                period_accum.dc_current_sum += data->dc_current;
                period_accum.dc_valid_count++;
            }

        }
        
        if (periods_completed >= periods_target) {
            periods_done = true;

            prepare_averaged_result();
        }
    }
    k_mutex_unlock(&period_mutex);
}

/* ============ GPIO 按鈕回調函數 ============ */

// Button 1: 請求開始
void button1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (!system_running) {
        request_start = true;
        printk("\n[Button 1] 請求開始\n");
    } else {
        printk("\n[Button 1] 系統已在運行中\n");
    }
}

// Button 2: 請求停止
void button2_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (system_running) {
        request_stop = true;
        printk("\n[Button 2] 請求停止\n");
    } else {
        printk("\n[Button 2] 系統未運行\n");
    }
}

/* ============ GPIO 初始化 ============ */
int init_gpio(void) {
    int ret;

    // LED1
    if (!device_is_ready(led1.port)) {
        printk("LED1 設備未就緒\n");
        return -1;
    }
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

    // LED2
    if (!device_is_ready(led2.port)) {
        printk("LED2 設備未就緒\n");
        return -1;
    }
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);

    // Button 1
    if (!device_is_ready(button1.port)) {
        printk("Button 1 設備未就緒\n");
        return -1;
    }
    ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
    if (ret != 0) return ret;
    ret = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) return ret;
    gpio_init_callback(&button1_cb_data, button1_pressed, BIT(button1.pin));
    gpio_add_callback(button1.port, &button1_cb_data);

    // Button 2
    if (!device_is_ready(button2.port)) {
        printk("Button 2 設備未就緒\n");
        return -1;
    }
    ret = gpio_pin_configure_dt(&button2, GPIO_INPUT);
    if (ret != 0) return ret;
    ret = gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) return ret;
    gpio_init_callback(&button2_cb_data, button2_pressed, BIT(button2.pin));
    gpio_add_callback(button2.port, &button2_cb_data);

    return 0;
}

/* ============ INA219 初始化 ============ */
static int init_ina219(void) {
    int ret;
    LOG_INF("Starting INA219 initialization");

    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device for INA219 not ready");
        return -ENODEV;
    }

    ret = ina219_init(&my_ina219, i2c_dev, INA219_I2C_ADDRESS);
    if (ret != 0) {
        LOG_ERR("Failed to initialize INA219: %d", ret);
        return ret;
    }

    ret = ina219_set_calibration(&my_ina219, INA219_SHUNT_RESISTANCE, INA219_MAX_EXPECTED_CURRENT);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 calibration: %d", ret);
        return ret;
    }

    ret = ina219_set_adc_mode(&my_ina219, INA219_ADC_12BIT_4S, INA219_ADC_12BIT_4S);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 ADC mode: %d", ret);
        return ret;
    }

    ret = ina219_set_bus_range(&my_ina219, INA219_BUSRANGE_32V);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 bus range: %d", ret);
        return ret;
    }

    ret = ina219_set_gain(&my_ina219, INA219_GAIN_40MV);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 gain: %d", ret);
        return ret;
    }

    ret = ina219_set_mode(&my_ina219, INA219_MODE_SHUNT_BUS_CONTINUOUS);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 mode: %d", ret);
        return ret;
    }

    LOG_INF("INA219 initialized successfully");
    return 0;
}

/* ============ INA219 讀取執行緒 ============ */
void ina219_read_thread_entry(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    uint32_t start_time = 0;
    MovingAverage current_filter;
    bool first_sample = true;

    LOG_INF("INA219 thread started");

    while (1) {
        // 等待啟用
        if (!ina219_logging_active) {
            first_sample = true;  // 重置，下次啟動時重新初始化
            k_sleep(K_MSEC(50));
            continue;
        }

        // 第一次取樣時初始化
        if (first_sample) {
            start_time = k_uptime_get_32();
            moving_average_init(&current_filter, CURRENT_FILTER_SIZE);

             /* ====== 初始化週期檢測器 ====== */
            period_detector_init(&period_detector, PERIOD_DETECT_METHOD);
            period_detector_set_callback(&period_detector, on_period_detected);

            /* ====== 重置週期檢測啟用旗標 ====== */
            period_detect_enabled = false;

            first_sample = false;
            printk("INA219 開始記錄電流（週期檢測已啟用）\n");
            printk("  - ADC 模式: 12BIT_4S\n");
            printk("  - 取樣間隔: %d ms\n", SENSOR_SAMPLE_INTERVAL_MS);
            printk("  - 檢測方法: 峰值檢測法\n");
            printk("  - 每電壓等級: 檢測%d個週期，跳過第1個，對後%d個取平均\n",
                    PERIODS_BEFORE_VOLTAGE_DROP, PERIODS_TO_AVERAGE);
        }

        // 讀取電流值
        k_mutex_lock(&i2c_mutex, K_FOREVER);
        float current_mA = ina219_read_current(&my_ina219);
        // float bus_voltage = ina219_read_bus_voltage(&my_ina219);
        // float shunt_voltage = ina219_read_shunt_voltage(&my_ina219);
        k_mutex_unlock(&i2c_mutex);

        uint32_t current_time_ticks = k_uptime_get_32();
        float filtered_current = moving_average_add(&current_filter, current_mA, CURRENT_FILTER_SIZE);
        float elapsed_seconds = (float)(current_time_ticks - start_time) / 1000.0f;
        // float total_voltage = bus_voltage + shunt_voltage / 1000.0f;

        /* ====== 使用 MAX77643 設定的電壓值 ====== */
        float voltage_for_calc = current_set_voltage;


        /* ====== 只在 period_detect_enabled 為 true 時進行週期檢測 ====== */
        /* 由主程式在適當時機設定 period_detect_enabled = true */
        if (period_detect_enabled) {
            period_detector_process(&period_detector, filtered_current, voltage_for_calc, elapsed_seconds);
        }

        // 輸出資料
        k_mutex_lock(&uart_mutex, K_FOREVER);
        printf("T:%.5f,C:%.4f,V:%.4f\n",
               (double)elapsed_seconds,
               (double)filtered_current,
               (double)voltage_for_calc);

        // 如果有新的週期檢測結果，輸出
        if (period_result_ready) {
            
            bool is_skipped = (periods_completed <= PERIODS_TO_SKIP);
            printf("P:%.4f,AVG:%.2f,PEAK:%.2f,E:%.4f,N:%d,CNT:%d,ST:%.4f,ET:%.4f,AC:%.2f,ACT:%.4f,ACST:%.4f,ACET:%.4f,ACN:%d,DC:%.2f,ACV:%d,DCV:%d,PT:%.4f,VT:%.4f%s\n",
                    (double)period_result_cache.period_time,
                    (double)period_result_cache.average_current,
                    (double)period_result_cache.peak_current,
                    (double)period_result_cache.energy,
                    period_result_cache.sample_count,
                    periods_completed,
                    (double)period_result_cache.start_time,
                    (double)period_result_cache.end_time,
                    (double)(period_result_cache.ac_valid ? period_result_cache.ac_current : 0.0f),
                    (double)(period_result_cache.ac_valid ? period_result_cache.ac_time : 0.0f),        
                    (double)(period_result_cache.ac_valid ? period_result_cache.ac_start_time : 0.0f), 
                    (double)(period_result_cache.ac_valid ? period_result_cache.ac_end_time : 0.0f),   
                    period_result_cache.ac_valid ? period_result_cache.ac_sample_count : 0,            
                    (double)(period_result_cache.dc_valid ? period_result_cache.dc_current : 0.0f),
                    (int)period_result_cache.ac_valid,
                    (int)period_result_cache.dc_valid,
                    (double)period_result_cache.peak_time,      
                    (double)period_result_cache.valley_time,    
                    is_skipped ? ",SKIP" : "");

            
            period_result_ready = false;
        }

        // ====== 如果平均結果準備好了，輸出 PAVG 行 ======
        if (avg_result_ready) {
            printf("PAVG:%.4f,AVG:%.2f,PEAK:%.2f,E:%.4f,V:%.2f,N:%d,AC:%.2f,ACT:%.4f,DC:%.2f\n",
                   (double)avg_result_period,
                   (double)avg_result_current,
                   (double)avg_result_peak,
                   (double)avg_result_energy,
                   (double)avg_result_voltage,
                   avg_result_count,
                   (double)avg_result_ac,
                   (double)avg_result_ac_time,
                   (double)avg_result_dc);
            avg_result_ready = false;
        }

        k_mutex_unlock(&uart_mutex);

        k_sleep(K_MSEC(SENSOR_SAMPLE_INTERVAL_MS));
        // k_usleep(500);  
    }
}

/* ============ MAX77643 控制函數 ============ */

// 關閉 SBB0 輸出
void shutdown_sbb0(void) {
    k_mutex_lock(&i2c_mutex, K_FOREVER);
    max77643_set_en_sbb(&max77643_device, 0, EN_SBB_OFF);
    k_mutex_unlock(&i2c_mutex);

    /* ====== 更新全域電壓變數為 0 ====== */
    current_set_voltage = 0.0f;
    
    gpio_pin_set_dt(&led1, 0);
    printk("SBB0 已關閉\n");
}

// 檢查是否應該停止（用於可中斷的延遲）
static inline bool should_stop(void) {
    return request_stop;
}

// 可中斷的毫秒延遲
bool interruptible_delay_ms(int ms) {
    int count = ms / 10;
    if (count < 1) count = 1;
    for (int i = 0; i < count; i++) {
        if (should_stop()) {
            return false;
        }
        k_sleep(K_MSEC(10));
    }
    return true;
}

// 可中斷的秒延遲
bool interruptible_delay_seconds(int seconds) {
    int count = seconds * 10;
    for (int i = 0; i < count; i++) {
        if (should_stop()) {
            return false;
        }
        k_sleep(K_MSEC(100));
    }
    return true;
}

// 可中斷的分鐘延遲
bool interruptible_delay_minutes(int minutes) {
    for (int min = 0; min < minutes; min++) {
        printk("  維持中... %d/%d 分鐘\n", min + 1, minutes);
        if (!interruptible_delay_seconds(60)) {
            return false;
        }
    }
    return true;
}

/* ====== 等待指定數量的週期完成 ====== */
bool wait_for_periods(int num_periods, float voltage, int timeout_seconds)  {

    k_mutex_lock(&uart_mutex, K_FOREVER);
    printf("  等待 %d 個週期完成 (電壓 = %.2fV)...\n", num_periods, (double)voltage);
    k_mutex_unlock(&uart_mutex);

    k_mutex_lock(&period_mutex, K_FOREVER);

    printf("確認每次下降電壓後是否有重置計數器...\n");

    reset_period_accumulator();

    /* 設定目標並啟動等待（這些變數會在 on_period_detected 中被檢查）*/
    periods_target = num_periods;
    periods_completed = 0;
    periods_done = false;

    avg_result_ready = false;  /* 重置平均結果旗標 */
    
    /* 確保週期檢測已啟用 */
    period_detect_enabled = true;
    printf("\n週期檢測已啟動\n");
    periods_waiting = true;  /* 啟動等待，從這一刻開始計數 */
    printf("\n開始週期計數\n");
    k_mutex_unlock(&period_mutex);
    
    int elapsed_100ms = 0;
    int timeout_100ms = timeout_seconds * 10;
    
    while (!periods_done) {
        if (should_stop()) {
            periods_waiting = false;
            k_mutex_lock(&uart_mutex, K_FOREVER);
            printf("  等待週期被中斷\n");
            k_mutex_unlock(&uart_mutex);
            return false;
        }
        
        k_sleep(K_MSEC(100));
        elapsed_100ms++;
        
        if (timeout_seconds > 0 && elapsed_100ms >= timeout_100ms) {
            periods_waiting = false;
            k_mutex_lock(&uart_mutex, K_FOREVER);
            printf("  等待週期超時 (%d 秒)，已完成 %d/%d 週期\n", 
                   timeout_seconds, periods_completed, num_periods);
            k_mutex_unlock(&uart_mutex);
            return false;
        }
        
        if (elapsed_100ms % 50 == 0) {
            k_mutex_lock(&uart_mutex, K_FOREVER);
            printf("  已完成 %d/%d 週期 (有效: %d)\n", 
                    periods_completed, num_periods, period_accum.valid_count);
            k_mutex_unlock(&uart_mutex);
        }
    }
    k_mutex_lock(&uart_mutex, K_FOREVER);
    periods_waiting = false; //關閉計數器
    periods_completed = 0;   //重置計數器
    period_detect_enabled = false; //關閉週期檢測
    periods_done = false;
    k_mutex_unlock(&uart_mutex);

    
    
    return true;
}

// 配置 SBB0 電流限制
int configure_sbb0_current_limit(void) {
    int ret;

    printk("\n【配置 SBB0 電流限制】\n");

    k_mutex_lock(&i2c_mutex, K_FOREVER);
    ret = max77643_set_ip_sbb(&max77643_device, 0, IP_SBB_AMP_0_333A);
    k_mutex_unlock(&i2c_mutex);
    
    if (ret != MAX77643_NO_ERROR) {
        printk("設置 IP_SBB0 失敗: %d\n", ret);
        return ret;
    }

    printk("  IP_SBB0 設置為 0.333A\n");
    return MAX77643_NO_ERROR;
}

// 軟啟動函數
bool soft_start_sbb0(float start_voltage, float target_voltage,
                     float step_voltage, int step_delay_ms) {
    int ret;
    float current_voltage = start_voltage;

    float dv_dt = step_voltage / (step_delay_ms / 1000.0f);

    printk("\n【軟啟動】開始\n");
    printk("  起始電壓: %.2fV -> 目標電壓: %.2fV\n", (double)start_voltage, (double)target_voltage);
    printk("  dv/dt: %.2f V/s\n", (double)dv_dt);

    // 設置起始電壓並開啟輸出
    k_mutex_lock(&i2c_mutex, K_FOREVER);
    ret = max77643_set_tv_sbb(&max77643_device, 0, start_voltage);
    if (ret == MAX77643_NO_ERROR) {
        ret = max77643_set_en_sbb(&max77643_device, 0, EN_SBB_ON);
    }
    k_mutex_unlock(&i2c_mutex);
    
    if (ret != MAX77643_NO_ERROR) {
        printk("開啟 SBB0 失敗: %d\n", ret);
        return false;
    }

    /* ====== 更新全域電壓變數 ====== */
    current_set_voltage = start_voltage;
    
    gpio_pin_set_dt(&led1, 1);  // LED1 亮起表示輸出開啟
    printk("SBB0 已開啟，起始電壓: %.2fV\n", (double)start_voltage);

    if (!interruptible_delay_ms(step_delay_ms)) {
        return false;
    }

    // 逐步上升電壓
    while (current_voltage < target_voltage - 0.01f) {
        if (should_stop()) {
            printk("軟啟動被中斷\n");
            return false;
        }

        current_voltage += step_voltage;
        if (current_voltage > target_voltage) {
            current_voltage = target_voltage;
        }

        k_mutex_lock(&i2c_mutex, K_FOREVER);
        ret = max77643_set_tv_sbb(&max77643_device, 0, current_voltage);
        k_mutex_unlock(&i2c_mutex);
        
        if (ret != MAX77643_NO_ERROR) {
            printk("設置電壓失敗: %d\n", ret);
            return false;
        }

        /* ====== 更新全域電壓變數 ====== */
        current_set_voltage = current_voltage;

        k_mutex_lock(&uart_mutex, K_FOREVER);
        printf("  電壓: %.2fV\n", (double)current_voltage);
        k_mutex_unlock(&uart_mutex);

        if (!interruptible_delay_ms(step_delay_ms)) {
            return false;
        }
    }

    // 確保最終電壓精確
    k_mutex_lock(&i2c_mutex, K_FOREVER);
    max77643_set_tv_sbb(&max77643_device, 0, target_voltage);
    k_mutex_unlock(&i2c_mutex);

    /* ====== 更新全域電壓變數為目標值 ====== */
    current_set_voltage = target_voltage;

    k_mutex_lock(&uart_mutex, K_FOREVER);
    printf("【軟啟動完成】目標電壓: %.2fV\n", (double)target_voltage);
    k_mutex_unlock(&uart_mutex);

    return true;
}

bool direct_start_sbb0(float voltage) {
    int ret;

    printk("\n【直接啟動】設置電壓: %.2fV\n", (double)voltage);

    k_mutex_lock(&i2c_mutex, K_FOREVER);
    
    /* 先設置目標電壓 */
    ret = max77643_set_tv_sbb(&max77643_device, 0, voltage);
    if (ret != MAX77643_NO_ERROR) {
        k_mutex_unlock(&i2c_mutex);
        printk("設置電壓失敗: %d\n", ret);
        return false;
    }
    
    /* 開啟輸出 */
    ret = max77643_set_en_sbb(&max77643_device, 0, EN_SBB_ON);
    k_mutex_unlock(&i2c_mutex);
    
    if (ret != MAX77643_NO_ERROR) {
        printk("開啟 SBB0 失敗: %d\n", ret);
        return false;
    }

    /* 更新全域電壓變數 */
    current_set_voltage = voltage;
    
    gpio_pin_set_dt(&led1, 1);  /* LED1 亮起表示輸出開啟 */
    printk("SBB0 已開啟，電壓: %.2fV\n", (double)voltage);

    return true;
}

/* ============ 系統啟動函數 ============ */
void system_start(void) {
    int ret;
    float current_voltage;

    system_running = true;
    request_stop = false;
    
    gpio_pin_set_dt(&led2, 1);  // LED2 亮起表示系統運行中

    printk("\n========================================\n");
    printk("  系統啟動：電壓輸出 + 電流記錄\n");
    printk("========================================\n");
    printk("按 Button 2 停止\n");
    printk("----------------------------------------\n");
    printk("週期檢測配置:\n");
    printk("  - 每電壓等級檢測: %d 個週期\n", PERIODS_BEFORE_VOLTAGE_DROP);
    printk("  - 跳過前: %d 個週期\n", PERIODS_TO_SKIP);
    printk("  - 用於平均: %d 個週期\n", PERIODS_TO_AVERAGE);

    // 啟動 INA219 電流記錄
    ina219_logging_active = true;

    // 配置電流限制
    ret = configure_sbb0_current_limit();
    if (ret != MAX77643_NO_ERROR) {
        printk("配置電流限制失敗\n");
        goto system_stop;
    }

    // 階段一：軟啟動
    // printf("\n【階段 1】軟啟動到 %.1fV\n", (double)TARGET_VOLTAGE_V);
    // if (!soft_start_sbb0(SOFT_START_VOLTAGE_V, TARGET_VOLTAGE_V,
    //                      SOFT_START_STEP_V, SOFT_START_DELAY_MS)) {
    //     goto system_stop;
    // }

    // 階段一：直接啟動
    printf("\n【階段 1】直接啟動到 %.1fV\n", (double)TARGET_VOLTAGE_V);
    if (!direct_start_sbb0(TARGET_VOLTAGE_V)) {
        goto system_stop;
    }

    // 階段二：維持電壓
    printf("\n【階段 2】維持 %.1fV 電壓 %d 分鐘\n", (double)TARGET_VOLTAGE_V, HOLD_TIME_MIN);
    if (!interruptible_delay_minutes(HOLD_TIME_MIN)) {
        goto system_stop;
    }

    // 階段三：斜坡下降
    printf("\n【階段 3】電壓斜坡下降(每 %d 個週期下降一次)\n", PERIODS_BEFORE_VOLTAGE_DROP);
    printf("從 %.1fV 下降至 %.1fV\n", (double)TARGET_VOLTAGE_V, (double)FINAL_VOLTAGE_V);

    current_voltage = TARGET_VOLTAGE_V;

    while (current_voltage > FINAL_VOLTAGE_V + 0.01f) {
        if (should_stop()) {
            goto system_stop;
        }

         /* ====== 先等待指定數量的週期完成 ====== */
        if (!wait_for_periods(PERIODS_BEFORE_VOLTAGE_DROP, current_voltage, STEP_INTERVAL_SEC)) {
            /* 超時或被中斷，詢問是否繼續 */
            if (should_stop()) goto system_stop;
            printk("  週期檢測超時，繼續下降電壓\n");
        }

        current_voltage -= VOLTAGE_STEP_V;


        if (current_voltage < FINAL_VOLTAGE_V) {
            current_voltage = FINAL_VOLTAGE_V;
        }
        

        k_mutex_lock(&i2c_mutex, K_FOREVER);
        ret = max77643_set_tv_sbb(&max77643_device, 0, current_voltage);
        k_mutex_unlock(&i2c_mutex);
        
        if (ret != MAX77643_NO_ERROR) {
            printk("設置電壓失敗: %d\n", ret);
            goto system_stop;
        }
        
        printk("\n--- 電壓已下降至 %.2fV ---\n", (double)current_voltage);
        
        /* ====== 計算自適應穩定等待時間 ====== */
        int settle_time_ms = (int)(last_avg_period_ms * VOLTAGE_SETTLE_PERIODS);

        /* 限制在合理範圍內 */
        if (settle_time_ms < VOLTAGE_SETTLE_MIN_MS) {
            settle_time_ms = VOLTAGE_SETTLE_MIN_MS;
        } else if (settle_time_ms > VOLTAGE_SETTLE_MAX_MS) {
            settle_time_ms = VOLTAGE_SETTLE_MAX_MS;
        }

        /* ====== 等待電壓穩定 ====== */
        printk("  等待電壓穩定 (%d ms, 基於平均週期 %.1f ms)...\n", 
            settle_time_ms, (double)last_avg_period_ms);
        
        printk("  等待電壓穩定 (%d ms)...\n", settle_time_ms);
        if (!interruptible_delay_ms(settle_time_ms)) {
            goto system_stop;
        }

        /* ====== 更新全域電壓變數 ====== */
        current_set_voltage = current_voltage;

        
        /* ====== 重置週期檢測器狀態，確保從新的峰值開始 ====== */
        period_detector_reset(&period_detector);
        printk("  週期檢測器已重置，開始新電壓等級的檢測\n");
    }
    /* 最後一個電壓等級也要等待週期完成 */
    if (current_voltage <= FINAL_VOLTAGE_V + 0.01f && current_voltage >= FINAL_VOLTAGE_V - 0.01f) {
        printk("\n--- 電壓 %.2fV (最終) ---\n", (double)current_voltage);
        wait_for_periods(PERIODS_BEFORE_VOLTAGE_DROP, current_voltage, 15);
    }

    // 測試完成
    printk("\n========================================\n");
    printk("  測試完成！\n");
    printk("========================================\n");
    printk("最終電壓: %.1fV\n", (double)current_voltage);
    printk("按 Button 2 可關閉輸出\n");
    printk("按 Button 1 可重新開始\n\n");

    // 測試完成後停止記錄但保持電壓輸出
    ina219_logging_active = false;
    system_running = false;
    gpio_pin_set_dt(&led2, 0);
    return;

system_stop:
    // 停止所有輸出
    ina219_logging_active = false;
    shutdown_sbb0();
    system_running = false;
    request_stop = false;
    gpio_pin_set_dt(&led2, 0);
    
    printk("\n========================================\n");
    printk("  系統已停止\n");
    printk("========================================\n");
    printk("SBB0 已關閉，電流記錄已停止\n");
    printk("按 Button 1 可重新開始\n\n");
}

/* ============ 系統停止函數 ============ */
void system_stop(void) {
    ina219_logging_active = false;
    shutdown_sbb0();
    system_running = false;
    request_stop = false;
    gpio_pin_set_dt(&led2, 0);
    
    printk("\n========================================\n");
    printk("  系統已停止\n");
    printk("========================================\n");
    printk("SBB0 已關閉，電流記錄已停止\n");
    printk("按 Button 1 可重新開始\n\n");
}

/* ============ 主函數 ============ */
int main(void) {
    int ret;

    printk("\n");
    printk("========================================\n");
    printk("  MAX77643 電壓控制 + INA219 電流記錄\n");
    printk("========================================\n");
    printk("Button 1: 開始（電壓輸出 + 電流記錄）\n");
    printk("Button 2: 停止（關閉電壓 + 停止記錄）\n");
    printk("LED1: SBB0 輸出狀態\n");
    printk("LED2: 系統運行中\n");
    printk("========================================\n\n");

    // 初始化 MAX77643
    k_mutex_lock(&i2c_mutex, K_FOREVER);
    ret = max77643_init(&max77643_device, &dev_max77643, 0);
    k_mutex_unlock(&i2c_mutex);
    
    if (ret != MAX77643_NO_ERROR) {
        printk("MAX77643 初始化失敗: %d\n", ret);
        return ret;
    }
    printk("MAX77643 初始化成功\n");

    // 初始化 INA219
    ret = init_ina219();
    if (ret != 0) {
        printk("INA219 初始化失敗: %d\n", ret);
        return ret;
    }

    // 初始化 GPIO
    ret = init_gpio();
    if (ret != 0) {
        printk("GPIO 初始化失敗: %d\n", ret);
        return ret;
    }
    printk("GPIO 初始化成功\n");

    /* ====== 初始化週期檢測器 ====== */
    period_detector_init(&period_detector, PERIOD_DETECT_METHOD);
    printk("週期檢測器初始化成功\n");

    // 創建 INA219 執行緒
    ina219_tid = k_thread_create(&ina219_thread_data,
                                  ina219_stack,
                                  INA219_STACK_SIZE,
                                  ina219_read_thread_entry,
                                  NULL, NULL, NULL,
                                  INA219_PRIORITY,
                                  0,
                                  K_NO_WAIT);
    if (!ina219_tid) {
        printk("創建 INA219 執行緒失敗\n");
        return -1;
    }
    k_thread_name_set(ina219_tid, "ina219_reader");

    printk("\n系統就緒，按 Button 1 開始...\n\n");

    // 主循環
    while (true) {
        // Button 1: 請求開始
        if (request_start && !system_running) {
            request_start = false;
            system_start();
        }

        // Button 2: 請求停止（在非運行期間也可關閉輸出）
        if (request_stop) {
            request_stop = false;
            if (system_running) {
                // 運行中的停止由 system_start() 內部處理
                // 這裡不需要額外操作
            } else {
                // 確保輸出關閉
                shutdown_sbb0();
            }
        }

        k_sleep(K_MSEC(50));
    }

    return 0;
}


/*20251218 數據補償：Raw ADC → [Hampel Filter] → [Phase-Locked Reconstruction] → [Cubic Spline] → Output*/
// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/i2c.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/logging/log.h>
// #include <stdio.h>
// #include <math.h>

// // 包含驅動程式標頭檔
// #include "ina219.h"
// #include "max77643.h"
// #include "signal_reconstruction.h"

// LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// /* ============ I2C 設備定義 ============ */
// #define MAX77643_NODE DT_NODELABEL(max77643)
// static const struct i2c_dt_spec dev_max77643 = I2C_DT_SPEC_GET(MAX77643_NODE);

// /* ============ GPIO 定義 ============ */
// #define BUTTON1_NODE DT_ALIAS(sw0)
// #define BUTTON2_NODE DT_ALIAS(sw1)

// static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BUTTON1_NODE, gpios);
// static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(BUTTON2_NODE, gpios);

// #define LED1_NODE DT_ALIAS(led0)
// #define LED2_NODE DT_ALIAS(led1)

// static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
// static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

// /* ============ MAX77643 軟啟動參數 ============ */
// #define SOFT_START_VOLTAGE_V    0.8f
// #define TARGET_VOLTAGE_V        5.0f
// #define SOFT_START_STEP_V       0.1f
// #define SOFT_START_DELAY_MS     100

// /* ============ 斜坡下降參數 ============ */
// #define FINAL_VOLTAGE_V         0.5f
// #define VOLTAGE_STEP_V          0.2f
// #define STEP_INTERVAL_SEC       15
// #define HOLD_TIME_MIN           5

// /* ============ INA219 配置參數 ============ */
// #define INA219_SHUNT_RESISTANCE    0.1f
// #define INA219_MAX_EXPECTED_CURRENT 0.5f
// #define INA219_I2C_ADDRESS         0x40
// #define SENSOR_SAMPLE_INTERVAL_MS  1

// /* ====== 新增：信號重建配置 ====== */
// #define EXPECTED_SAMPLE_INTERVAL   0.006f   // 預期取樣間隔 6ms
// #define STATISTICS_UPDATE_INTERVAL 50       // 每 50 個樣本更新一次統計

// /* ============ 執行緒配置 ============ */
// #define INA219_STACK_SIZE 2048
// #define INA219_PRIORITY 7

// K_THREAD_STACK_DEFINE(ina219_stack, INA219_STACK_SIZE);
// struct k_thread ina219_thread_data;
// static k_tid_t ina219_tid = NULL;

// /* ============ 同步與互斥鎖 ============ */
// K_MUTEX_DEFINE(uart_mutex);
// K_MUTEX_DEFINE(i2c_mutex);

// /* ============ 驅動程式實例 ============ */
// static max77643_t max77643_device;
// static struct ina219_data my_ina219;

// /* ====== 信號重建器實例 ====== */
// static signal_reconstructor_t current_reconstructor;


// /* ============ GPIO 回調結構 ============ */
// static struct gpio_callback button1_cb_data;
// static struct gpio_callback button2_cb_data;

// /* ============ 全域控制旗標 ============ */
// static volatile bool request_start = false;   // Button 1 請求開始
// static volatile bool request_stop = false;    // Button 2 請求停止
// static volatile bool system_running = false;  // 系統運行狀態
// static volatile bool ina219_logging_active = false;

// /* ============ 移動平均濾波器 ============ */
// #define CURRENT_FILTER_SIZE 8

// typedef struct {
//     float values[CURRENT_FILTER_SIZE];
//     int index;
//     int count;
//     float sum;
// } MovingAverage;

// static void moving_average_init(MovingAverage *ma, int window_size) {
//     ma->index = 0;
//     ma->count = 0;
//     ma->sum = 0.0f;
//     for (int i = 0; i < window_size && i < CURRENT_FILTER_SIZE; i++) {
//         ma->values[i] = 0.0f;
//     }
// }

// static float moving_average_add(MovingAverage *ma, float value, int window_size) {
//     if (window_size > CURRENT_FILTER_SIZE) window_size = CURRENT_FILTER_SIZE;

//     if (ma->count == window_size) {
//         ma->sum -= ma->values[ma->index];
//     } else {
//         ma->count++;
//     }

//     ma->values[ma->index] = value;
//     ma->sum += value;
//     ma->index = (ma->index + 1) % window_size;

//     return ma->sum / ma->count;
// }

// /* ============ 一階低通濾波器（保留以備後用）============ */
// typedef struct {
//     float prev_output;
//     float alpha;
//     int initialized;
// } FirstOrderFilter;


// /* 數據濾波器 */
// static FirstOrderFilter cur_filter = {
//     .prev_output = 0.0f,
//     .alpha = 0.25f,        // 預設α值，可根據需求調整
//     .initialized = 0
// };

// static void first_order_filter_init(FirstOrderFilter *filter, float alpha) {
//     filter->prev_output = 0.0f;
//     filter->alpha = alpha;
//     filter->initialized = 0;
// }

// static float first_order_filter_apply(FirstOrderFilter *filter, float input) {
//     if (!filter->initialized) {
//         filter->prev_output = input;
//         filter->initialized = 1;
//         return input;
//     }
//     float output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
//     filter->prev_output = output;
//     return output;
// }

// /* ====== 輸出模式選擇 ====== */
// typedef enum {
//     OUTPUT_RAW = 0,           // 只輸出原始值
//     OUTPUT_FILTERED = 1,      // 只輸出濾波後的值
//     OUTPUT_BOTH = 2           // 同時輸出原始和濾波值
// } output_mode_t;
// static output_mode_t current_output_mode = OUTPUT_RAW;


// /* ============ GPIO 按鈕回調函數 ============ */

// // Button 1: 請求開始
// void button1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
//     if (!system_running) {
//         request_start = true;
//         printk("\n[Button 1] 請求開始\n");
//     } else {
//         printk("\n[Button 1] 系統已在運行中\n");
//     }
// }

// // Button 2: 請求停止
// void button2_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
//     if (system_running) {
//         request_stop = true;
//         printk("\n[Button 2] 請求停止\n");
//     } else {
//         printk("\n[Button 2] 系統未運行\n");
//     }
// }

// /* ============ GPIO 初始化 ============ */
// int init_gpio(void) {
//     int ret;

//     // LED1
//     if (!device_is_ready(led1.port)) {
//         printk("LED1 設備未就緒\n");
//         return -1;
//     }
//     gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

//     // LED2
//     if (!device_is_ready(led2.port)) {
//         printk("LED2 設備未就緒\n");
//         return -1;
//     }
//     gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);

//     // Button 1
//     if (!device_is_ready(button1.port)) {
//         printk("Button 1 設備未就緒\n");
//         return -1;
//     }
//     ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
//     if (ret != 0) return ret;
//     ret = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
//     if (ret != 0) return ret;
//     gpio_init_callback(&button1_cb_data, button1_pressed, BIT(button1.pin));
//     gpio_add_callback(button1.port, &button1_cb_data);

//     // Button 2
//     if (!device_is_ready(button2.port)) {
//         printk("Button 2 設備未就緒\n");
//         return -1;
//     }
//     ret = gpio_pin_configure_dt(&button2, GPIO_INPUT);
//     if (ret != 0) return ret;
//     ret = gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_TO_ACTIVE);
//     if (ret != 0) return ret;
//     gpio_init_callback(&button2_cb_data, button2_pressed, BIT(button2.pin));
//     gpio_add_callback(button2.port, &button2_cb_data);

//     return 0;
// }

// /* ============ INA219 初始化 ============ */
// static int init_ina219(void) {
//     int ret;
//     LOG_INF("Starting INA219 initialization");

//     const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
//     if (!device_is_ready(i2c_dev)) {
//         LOG_ERR("I2C device for INA219 not ready");
//         return -ENODEV;
//     }

//     ret = ina219_init(&my_ina219, i2c_dev, INA219_I2C_ADDRESS);
//     if (ret != 0) {
//         LOG_ERR("Failed to initialize INA219: %d", ret);
//         return ret;
//     }

//     ret = ina219_set_calibration(&my_ina219, INA219_SHUNT_RESISTANCE, INA219_MAX_EXPECTED_CURRENT);
//     if (ret != 0) {
//         LOG_ERR("Failed to set INA219 calibration: %d", ret);
//         return ret;
//     }

//     ret = ina219_set_adc_mode(&my_ina219, INA219_ADC_12BIT, INA219_ADC_12BIT);
//     if (ret != 0) {
//         LOG_ERR("Failed to set INA219 ADC mode: %d", ret);
//         return ret;
//     }

//     ret = ina219_set_bus_range(&my_ina219, INA219_BUSRANGE_32V);
//     if (ret != 0) {
//         LOG_ERR("Failed to set INA219 bus range: %d", ret);
//         return ret;
//     }

//     ret = ina219_set_gain(&my_ina219, INA219_GAIN_40MV);
//     if (ret != 0) {
//         LOG_ERR("Failed to set INA219 gain: %d", ret);
//         return ret;
//     }

//     ret = ina219_set_mode(&my_ina219, INA219_MODE_SHUNT_BUS_CONTINUOUS);
//     if (ret != 0) {
//         LOG_ERR("Failed to set INA219 mode: %d", ret);
//         return ret;
//     }

//     LOG_INF("INA219 initialized successfully");
//     return 0;
// }

// /* ============ INA219 讀取執行緒 ============ */
// void ina219_read_thread_entry(void *arg1, void *arg2, void *arg3) {
//     ARG_UNUSED(arg1);
//     ARG_UNUSED(arg2);
//     ARG_UNUSED(arg3);

//     uint32_t start_time = 0;
//     bool first_sample = true;

//     /* ====== 樣本計數器（用於統計更新） ====== */
//     int sample_counter = 0;

//     LOG_INF("INA219 thread started with signal reconstruction");

//     while (1) {
//         // 等待啟用
//         if (!ina219_logging_active) {
//             first_sample = true;  // 重置，下次啟動時重新初始化
//             k_sleep(K_MSEC(50));
//             continue;
//         }

//         // 第一次取樣時初始化
//         if (first_sample) {
//             start_time = k_uptime_get_32();
//             /* ====== 初始化信號重建器 ====== */
//             signal_reconstructor_init(&current_reconstructor, EXPECTED_SAMPLE_INTERVAL);
//             sample_counter = 0;

//             first_sample = false;
//             printk("INA219 開始記錄電流（信號重建已啟用）\n");
//             printk("  - Hampel 濾波器：窗口=%d, 閾值=%.1f\n", 
//                    HAMPEL_WINDOW_SIZE, (double)HAMPEL_THRESHOLD);
//             printk("  - 相位鎖定重建：已啟用\n");
//             printk("  - Cubic Spline：點數=%d\n", SPLINE_POINTS);
//         }

//         // 讀取電流值
//         k_mutex_lock(&i2c_mutex, K_FOREVER);
//         float raw_current_mA = ina219_read_current(&my_ina219);
//         float bus_voltage = ina219_read_bus_voltage(&my_ina219);
//         float shunt_voltage = ina219_read_shunt_voltage(&my_ina219);
//         // float raw_current_mA = (shunt_voltage) / INA219_SHUNT_RESISTANCE; // 直接計算電流
//         k_mutex_unlock(&i2c_mutex);

//         uint32_t current_time_ticks = k_uptime_get_32();
//         float elapsed_seconds = (float)(current_time_ticks - start_time) / 1000.0f;
//         float total_voltage = bus_voltage + shunt_voltage / 1000.0f;


//         /* ====== 通過信號重建器處理電流值 ====== */
//         float filtered_current = signal_reconstructor_process(
//             &current_reconstructor, 
//             raw_current_mA, 
//             elapsed_seconds
//         );
        
//         sample_counter++;

//         /* ====== 定期更新統計信息 ====== */
//         if (sample_counter % STATISTICS_UPDATE_INTERVAL == 0) {
//             signal_reconstructor_update_statistics(&current_reconstructor);
            
//             // 每 500 個樣本輸出一次統計報告
//             if (sample_counter % 500 == 0) {
//                 float peak, rms, avg;
//                 int total, filtered_count, interpolated;
                
//                 signal_reconstructor_get_statistics(&current_reconstructor,
//                     &peak, &rms, &avg, &total, &filtered_count, &interpolated);
                
//                 k_mutex_lock(&uart_mutex, K_FOREVER);
//                 printk("\n===== 信號重建統計 =====\n");
//                 printk("總樣本: %d, 濾波修正: %d, 插值填補: %d\n", 
//                        total, filtered_count, interpolated);
//                 printk("峰值: %.2f mA, RMS: %.2f mA, 平均: %.2f mA\n",
//                        (double)peak, (double)rms, (double)avg);
//                 if (current_reconstructor.period_tracker.period_valid) {
//                     printk("週期: %.3f s (%.1f Hz)\n",
//                            (double)current_reconstructor.period_tracker.measured_period,
//                            (double)(1.0f / current_reconstructor.period_tracker.measured_period));
//                 }
//                 printk("========================\n\n");
//                 k_mutex_unlock(&uart_mutex);
//             }
//         }


//         /* ====== 根據輸出模式選擇輸出內容 ====== */
//         k_mutex_lock(&uart_mutex, K_FOREVER);
//         switch (current_output_mode) {
//             case OUTPUT_RAW:
//                 // 只輸出原始值
//                 printf("T1:%.5f\n"
//                         "C:%.4f\n"
//                         "V:%.4f\n",
//                        (double)elapsed_seconds,
//                        (double)raw_current_mA,
//                        (double)total_voltage);
//                 break;
                
//             case OUTPUT_FILTERED:
//                 // 只輸出濾波後的值
//                 printf("T1:%.5f\n"
//                         "C:%.4f\n"
//                         "V:%.4f\n",
//                        (double)elapsed_seconds,
//                        (double)filtered_current,
//                        (double)total_voltage);
//                 break;
                
//             case OUTPUT_BOTH:
//             default:
//                 // 同時輸出原始和濾波值（用於比較）
//                 // 格式: T:時間,CR:原始電流,CF:濾波電流,V:電壓
//                 printf("T1:%.5f\n"
//                         "CR:%.4f\n"
//                         "CF:%.4f\n"
//                         "V:%.4f\n",
//                        (double)elapsed_seconds,
//                        (double)raw_current_mA,
//                        (double)filtered_current,
//                        (double)total_voltage);
//                 break;
//         }
//         k_mutex_unlock(&uart_mutex);

//         // k_sleep(K_MSEC(SENSOR_SAMPLE_INTERVAL_MS));
//         k_usleep(500);  // 0.5ms 睡眠以提高取樣率
//     }
// }

// /* ============ MAX77643 控制函數 ============ */

// // 關閉 SBB0 輸出
// void shutdown_sbb0(void) {
//     k_mutex_lock(&i2c_mutex, K_FOREVER);
//     max77643_set_en_sbb(&max77643_device, 0, EN_SBB_OFF);
//     k_mutex_unlock(&i2c_mutex);
    
//     gpio_pin_set_dt(&led1, 0);
//     printk("SBB0 已關閉\n");
// }

// // 檢查是否應該停止（用於可中斷的延遲）
// static inline bool should_stop(void) {
//     return request_stop;
// }

// // 可中斷的毫秒延遲
// bool interruptible_delay_ms(int ms) {
//     int count = ms / 10;
//     if (count < 1) count = 1;
//     for (int i = 0; i < count; i++) {
//         if (should_stop()) {
//             return false;
//         }
//         k_sleep(K_MSEC(10));
//     }
//     return true;
// }

// // 可中斷的秒延遲
// bool interruptible_delay_seconds(int seconds) {
//     int count = seconds * 10;
//     for (int i = 0; i < count; i++) {
//         if (should_stop()) {
//             return false;
//         }
//         k_sleep(K_MSEC(100));
//     }
//     return true;
// }

// // 可中斷的分鐘延遲
// bool interruptible_delay_minutes(int minutes) {
//     for (int min = 0; min < minutes; min++) {
//         printk("  維持中... %d/%d 分鐘\n", min + 1, minutes);
//         if (!interruptible_delay_seconds(60)) {
//             return false;
//         }
//     }
//     return true;
// }

// // 配置 SBB0 電流限制
// int configure_sbb0_current_limit(void) {
//     int ret;

//     printk("\n【配置 SBB0 電流限制】\n");

//     k_mutex_lock(&i2c_mutex, K_FOREVER);
//     ret = max77643_set_ip_sbb(&max77643_device, 0, IP_SBB_AMP_0_333A);
//     k_mutex_unlock(&i2c_mutex);
    
//     if (ret != MAX77643_NO_ERROR) {
//         printk("設置 IP_SBB0 失敗: %d\n", ret);
//         return ret;
//     }

//     printk("  IP_SBB0 設置為 0.333A\n");
//     return MAX77643_NO_ERROR;
// }

// // 軟啟動函數
// bool soft_start_sbb0(float start_voltage, float target_voltage,
//                      float step_voltage, int step_delay_ms) {
//     int ret;
//     float current_voltage = start_voltage;

//     float dv_dt = step_voltage / (step_delay_ms / 1000.0f);

//     printk("\n【軟啟動】開始\n");
//     printk("  起始電壓: %.2fV -> 目標電壓: %.2fV\n", (double)start_voltage, (double)target_voltage);
//     printk("  dv/dt: %.2f V/s\n", (double)dv_dt);

//     // 設置起始電壓並開啟輸出
//     k_mutex_lock(&i2c_mutex, K_FOREVER);
//     ret = max77643_set_tv_sbb(&max77643_device, 0, start_voltage);
//     if (ret == MAX77643_NO_ERROR) {
//         ret = max77643_set_en_sbb(&max77643_device, 0, EN_SBB_ON);
//     }
//     k_mutex_unlock(&i2c_mutex);
    
//     if (ret != MAX77643_NO_ERROR) {
//         printk("開啟 SBB0 失敗: %d\n", ret);
//         return false;
//     }

//     gpio_pin_set_dt(&led1, 1);  // LED1 亮起表示輸出開啟
//     printk("SBB0 已開啟，起始電壓: %.2fV\n", (double)start_voltage);

//     if (!interruptible_delay_ms(step_delay_ms)) {
//         return false;
//     }

//     // 逐步上升電壓
//     while (current_voltage < target_voltage - 0.01f) {
//         if (should_stop()) {
//             printk("軟啟動被中斷\n");
//             return false;
//         }

//         current_voltage += step_voltage;
//         if (current_voltage > target_voltage) {
//             current_voltage = target_voltage;
//         }

//         k_mutex_lock(&i2c_mutex, K_FOREVER);
//         ret = max77643_set_tv_sbb(&max77643_device, 0, current_voltage);
//         k_mutex_unlock(&i2c_mutex);
        
//         if (ret != MAX77643_NO_ERROR) {
//             printk("設置電壓失敗: %d\n", ret);
//             return false;
//         }

//         printk("  電壓: %.2fV\n", (double)current_voltage);

//         if (!interruptible_delay_ms(step_delay_ms)) {
//             return false;
//         }
//     }

//     // 確保最終電壓精確
//     k_mutex_lock(&i2c_mutex, K_FOREVER);
//     max77643_set_tv_sbb(&max77643_device, 0, target_voltage);
//     k_mutex_unlock(&i2c_mutex);

//     printk("【軟啟動完成】目標電壓: %.2fV\n", (double)target_voltage);
//     return true;
// }

// /* ============ 系統啟動函數 ============ */
// void system_start(void) {
//     int ret;
//     float current_voltage;

//     system_running = true;
//     request_stop = false;
    
//     gpio_pin_set_dt(&led2, 1);  // LED2 亮起表示系統運行中

//     printk("\n========================================\n");
//     printk("  系統啟動：電壓輸出 + 電流記錄 + 信號重建\n");
//     printk("========================================\n");
//     /* ====== 新增：顯示信號重建信息 ====== */
//     printk("信號重建功能：\n");
//     printk("  - Hampel 濾波器（異常值檢測）\n");
//     printk("  - 相位鎖定重建（缺失樣本填補）\n");
//     printk("  - Cubic Spline（波形平滑）\n");
//     printk("按 Button 2 停止\n");
//     printk("----------------------------------------\n");


//     // 啟動 INA219 電流記錄
//     ina219_logging_active = true;

//     // 配置電流限制
//     ret = configure_sbb0_current_limit();
//     if (ret != MAX77643_NO_ERROR) {
//         printk("配置電流限制失敗\n");
//         goto system_stop;
//     }

//     // 階段一：軟啟動
//     printk("\n【階段 1】軟啟動到 %.1fV\n", (double)TARGET_VOLTAGE_V);
//     if (!soft_start_sbb0(SOFT_START_VOLTAGE_V, TARGET_VOLTAGE_V,
//                          SOFT_START_STEP_V, SOFT_START_DELAY_MS)) {
//         goto system_stop;
//     }

//     // 階段二：維持電壓
//     printk("\n【階段 2】維持 %.1fV 電壓 %d 分鐘\n", (double)TARGET_VOLTAGE_V, HOLD_TIME_MIN);
//     if (!interruptible_delay_minutes(HOLD_TIME_MIN)) {
//         goto system_stop;
//     }

//     // 階段三：斜坡下降
//     printk("\n【階段 3】電壓斜坡下降\n");
//     printk("從 %.1fV 下降至 %.1fV\n", (double)TARGET_VOLTAGE_V, (double)FINAL_VOLTAGE_V);

//     current_voltage = TARGET_VOLTAGE_V;

//     while (current_voltage > FINAL_VOLTAGE_V + 0.01f) {
//         if (should_stop()) {
//             goto system_stop;
//         }

//         current_voltage -= VOLTAGE_STEP_V;
//         if (current_voltage < FINAL_VOLTAGE_V) {
//             current_voltage = FINAL_VOLTAGE_V;
//         }

//         k_mutex_lock(&i2c_mutex, K_FOREVER);
//         ret = max77643_set_tv_sbb(&max77643_device, 0, current_voltage);
//         k_mutex_unlock(&i2c_mutex);
        
//         if (ret != MAX77643_NO_ERROR) {
//             printk("設置電壓失敗: %d\n", ret);
//             goto system_stop;
//         }

//         printk("電壓: %.2fV\n", (double)current_voltage);

//         if (current_voltage > FINAL_VOLTAGE_V + 0.01f) {
//             if (!interruptible_delay_seconds(STEP_INTERVAL_SEC)) {
//                 goto system_stop;
//             }
//         }
//     }

//     // 測試完成
//     printk("\n========================================\n");
//     printk("  測試完成！\n");
//     printk("========================================\n");
//     printk("最終電壓: %.1fV\n", (double)current_voltage);
//     printk("按 Button 2 可關閉輸出\n");
//     printk("按 Button 1 可重新開始\n\n");

//     // 測試完成後停止記錄但保持電壓輸出
//     ina219_logging_active = false;
//     system_running = false;
//     gpio_pin_set_dt(&led2, 0);
//     return;

// system_stop:
//     // 停止所有輸出
//     ina219_logging_active = false;
//     shutdown_sbb0();
//     system_running = false;
//     request_stop = false;
//     gpio_pin_set_dt(&led2, 0);
    
//     printk("\n========================================\n");
//     printk("  系統已停止\n");
//     printk("========================================\n");
//     printk("SBB0 已關閉，電流記錄已停止\n");
//     printk("按 Button 1 可重新開始\n\n");
// }

// /* ============ 系統停止函數 ============ */
// void system_stop(void) {
//     ina219_logging_active = false;
//     shutdown_sbb0();
//     system_running = false;
//     request_stop = false;
//     gpio_pin_set_dt(&led2, 0);
    
//     printk("\n========================================\n");
//     printk("  系統已停止\n");
//     printk("========================================\n");
//     printk("SBB0 已關閉，電流記錄已停止\n");
//     printk("按 Button 1 可重新開始\n\n");
// }

// /* ============ 主函數 ============ */
// int main(void) {
//     int ret;

//     printk("\n");
//     printk("========================================\n");
//     printk("  MAX77643 電壓控制 + INA219 電流記錄\n");
//     printk("  + 信號重建模組\n");               /* ====== 新增 ====== */
//     printk("========================================\n");
//     printk("Button 1: 開始（電壓輸出 + 電流記錄）\n");
//     printk("Button 2: 停止（關閉電壓 + 停止記錄）\n");
//     printk("LED1: SBB0 輸出狀態\n");
//     printk("LED2: 系統運行中\n");
//     printk("========================================\n");
//     /* ====== 新增：顯示信號重建配置 ====== */
//     printk("\n信號重建配置:\n");
//     printk("  Hampel 窗口: %d 點\n", HAMPEL_WINDOW_SIZE);
//     printk("  Hampel 閾值: %.1f MAD\n", (double)HAMPEL_THRESHOLD);
//     printk("  Spline 點數: %d\n", SPLINE_POINTS);
//     printk("  預期取樣間隔: %.3f s\n", (double)EXPECTED_SAMPLE_INTERVAL);
//     printk("========================================\n\n");


//     // 初始化 MAX77643
//     k_mutex_lock(&i2c_mutex, K_FOREVER);
//     ret = max77643_init(&max77643_device, &dev_max77643, 0);
//     k_mutex_unlock(&i2c_mutex);
    
//     if (ret != MAX77643_NO_ERROR) {
//         printk("MAX77643 初始化失敗: %d\n", ret);
//         return ret;
//     }
//     printk("MAX77643 初始化成功\n");

//     // 初始化 INA219
//     ret = init_ina219();
//     if (ret != 0) {
//         printk("INA219 初始化失敗: %d\n", ret);
//         return ret;
//     }

//     // 初始化 GPIO
//     ret = init_gpio();
//     if (ret != 0) {
//         printk("GPIO 初始化失敗: %d\n", ret);
//         return ret;
//     }
//     printk("GPIO 初始化成功\n");

//      /* ====== 預初始化信號重建器 ====== */
//     signal_reconstructor_init(&current_reconstructor, EXPECTED_SAMPLE_INTERVAL);
//     printk("信號重建器初始化成功\n");

//     // 創建 INA219 執行緒
//     ina219_tid = k_thread_create(&ina219_thread_data,
//                                   ina219_stack,
//                                   INA219_STACK_SIZE,
//                                   ina219_read_thread_entry,
//                                   NULL, NULL, NULL,
//                                   INA219_PRIORITY,
//                                   0,
//                                   K_NO_WAIT);
//     if (!ina219_tid) {
//         printk("創建 INA219 執行緒失敗\n");
//         return -1;
//     }
//     k_thread_name_set(ina219_tid, "ina219_reader");

//     printk("\n系統就緒，按 Button 1 開始...\n\n");

//     // 主循環
//     while (true) {
//         // Button 1: 請求開始
//         if (request_start && !system_running) {
//             request_start = false;
//             system_start();
//         }

//         // Button 2: 請求停止（在非運行期間也可關閉輸出）
//         if (request_stop) {
//             request_stop = false;
//             if (system_running) {
//                 // 運行中的停止由 system_start() 內部處理
//                 // 這裡不需要額外操作
//             } else {
//                 // 確保輸出關閉
//                 shutdown_sbb0();
//             }
//         }

//         k_sleep(K_MSEC(50));
//     }

//     return 0;
// }