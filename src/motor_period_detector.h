/*
 * motor_period_detector.h
 * 
 * 馬達週期檢測模組
 * 功能：
 * 1. 通過電流信號檢測馬達旋轉週期
 * 2. 計算每個週期內的平均電流、峰值電流、能量消耗
 * 
 * 適用於 nRF5340DK (ARM Cortex-M33)
 * 
 * 日期：20251218
 */

#ifndef MOTOR_PERIOD_DETECTOR_H
#define MOTOR_PERIOD_DETECTOR_H

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>

/* ============ 配置參數 ============ */
#define PERIOD_BUFFER_SIZE      1000     // 一個週期內最大樣本數
#define MIN_PERIOD_MS           70     // 最小有效週期 (ms)
#define MAX_PERIOD_MS           2000    // 最大有效週期 (ms)
#define PEAK_DETECTION_WINDOW   7       // 峰值檢測窗口大小
#define VALLEY_DETECTION_WINDOW 7       // 谷值檢測窗口大小
#define MIN_SAMPLES_PER_PERIOD  10      // 每週期最少樣本數

/* Peak Prominence 參數 */
#define MIN_PROMINENCE_MA       3.0f    // 最小峰值突出度 (mA)|新增：20260101


/* AC/DC 計算參數 */
#define AC_DC_WINDOW_SIZE       2       // 峰值/谷值前後各取幾個樣本（總共 2*N+1 個）


/* ============ 檢測方法選擇 ============ */
typedef enum {
    DETECT_METHOD_PEAK,         // 峰值檢測法
    DETECT_METHOD_ZERO_CROSS,   // 零交叉檢測法
    DETECT_METHOD_THRESHOLD     // 閾值檢測法
} detect_method_t;

/* ============ 週期數據結構 ============ */
typedef struct {
    float period_time;          // 週期時間 (秒)
    float average_current;      // 平均電流 (mA)
    float peak_current;         // 峰值電流 (mA)
    float min_current;          // 最小電流 (mA)
    float rms_current;          // RMS 電流 (mA)
    float energy;               // 能量消耗 (mJ) = 平均電流 × 電壓 × 時間
    int sample_count;           // 樣本數量
    float start_time;           // 週期開始時間
    float end_time;             // 週期結束時間
    // 新增：20260103
    float peak_time;            // 峰值時間點
    float valley_time;          // 谷值時間點

    float voltage;              // 平均電壓 (V)
    bool valid;                 // 數據是否有效

    /* 新增20251230：峰值和谷值附近的平均電流 */
    float ac_current;           // 峰值附近平均電流 (mA) - 峰值前後各2個+峰值
    float dc_current;           // 谷值附近平均電流 (mA) - 谷值前後各2個+谷值
    bool ac_valid;              // AC 值是否有效
    bool dc_valid;              // DC 值是否有效
} period_data_t;

/* ============ 檢測器狀態 ============ */
typedef enum {
    STATE_INIT,
    STATE_WAITING_FIRST_PEAK,
    STATE_WAITING_VALLEY,       // 新增：等待谷值
    STATE_COLLECTING,
    STATE_PERIOD_COMPLETE
} detector_state_t;

/* ============ 週期檢測器結構 ============ */
typedef struct {
    /* 檢測方法 */
    detect_method_t method;
    
    /* 狀態機 */
    detector_state_t state;
    
    /* 當前週期數據緩衝 */
    float current_buffer[PERIOD_BUFFER_SIZE];
    float voltage_buffer[PERIOD_BUFFER_SIZE];
    float time_buffer[PERIOD_BUFFER_SIZE];
    int buffer_idx;
    
    /* 峰值檢測相關 */
    float last_peak_value;      // 上一個峰值
    float last_peak_time;       // 上一個峰值時間
    float current_max;          // 當前週期最大值
    float current_max_time;     // 當前最大值時間
    int samples_since_max;      // 距離最大值的樣本數
    int peak_buffer_idx;        // 20251230：峰值在緩衝區中的索引

    /* 谷值檢測相關 */
    float current_min;          // 當前最小值
    float current_min_time;     // 當前最小值時間
    int samples_since_min;      // 距離最小值的樣本數
    int valley_buffer_idx;      // 20251230：谷值在緩衝區中的索引

    /* 20260101新增：已確認的谷值（用於 prominence 計算）*/
    float confirmed_valley_value;   // 已確認的谷值電流
    float confirmed_valley_time;    // 已確認的谷值時間
    bool valley_confirmed;          // 谷值是否已確認
    
    /* 零交叉檢測相關 */
    float dc_offset;            // 直流偏移量
    float prev_value;           // 前一個值
    bool prev_above_offset;     // 前一個值是否在偏移量之上
    
    /* 閾值檢測相關 */
    float threshold_high;       // 上閾值
    float threshold_low;        // 下閾值
    bool above_threshold;       // 是否在閾值之上
    
    /* 週期統計 */
    float measured_period;      // 測量到的週期 (秒)
    float period_sum;           // 週期累加（用於計算平均）
    int period_count;           // 有效週期計數
    float average_period;       // 平均週期

    /* 假峰值統計（新增20260101）*/
    int false_peak_count;       // 被過濾的假峰值計數
    
    /* 最近一次完整週期的數據 */
    period_data_t last_period;
    
    /* 回調函數（週期完成時調用）*/
    void (*on_period_complete)(period_data_t *data);
    
} motor_period_detector_t;

/* ============ 函數原型 ============ */

/**
 * @brief 初始化週期檢測器
 * @param detector 檢測器指針
 * @param method 檢測方法
 */
void period_detector_init(motor_period_detector_t *detector, detect_method_t method);

/**
 * @brief 設置週期完成回調函數
 * @param detector 檢測器指針
 * @param callback 回調函數
 */
void period_detector_set_callback(motor_period_detector_t *detector, 
                                   void (*callback)(period_data_t *data));

/**
 * @brief 處理新的電流樣本
 * @param detector 檢測器指針
 * @param current 電流值 (mA)
 * @param voltage 電壓值 (V)
 * @param timestamp 時間戳 (秒)
 * @return true 如果檢測到完整週期
 */
bool period_detector_process(motor_period_detector_t *detector, 
                             float current, 
                             float voltage,
                             float timestamp);

/**
 * @brief 設置閾值（用於閾值檢測法）
 * @param detector 檢測器指針
 * @param high 上閾值
 * @param low 下閾值
 */
void period_detector_set_threshold(motor_period_detector_t *detector, 
                                    float high, float low);

/**
 * @brief 獲取最近一次週期數據
 * @param detector 檢測器指針
 * @return 週期數據指針
 */
period_data_t* period_detector_get_last_period(motor_period_detector_t *detector);

/**
 * @brief 獲取平均週期時間
 * @param detector 檢測器指針
 * @return 平均週期 (秒)，如果無效返回 0
 */
float period_detector_get_average_period(motor_period_detector_t *detector);

/**
 * @brief 獲取週期計數
 * @param detector 檢測器指針
 * @return 已檢測到的有效週期數量
 */
int period_detector_get_period_count(motor_period_detector_t *detector);

/**
 * @brief 重置檢測器
 * @param detector 檢測器指針
 */
void period_detector_reset(motor_period_detector_t *detector);

/**
 * @brief 更新直流偏移量（用於零交叉檢測）
 * @param detector 檢測器指針
 * @param current 當前電流值
 */
void period_detector_update_dc_offset(motor_period_detector_t *detector, float current);

#endif // MOTOR_PERIOD_DETECTOR_H