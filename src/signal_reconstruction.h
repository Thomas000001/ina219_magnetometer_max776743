/*
 * signal_reconstruction.h
 * 
 * 信號重建模組 - 用於馬達電流波形還原
 * 包含：Hampel 濾波器、相位鎖定重建、Cubic Spline 插值
 * 
 * 適用於 nRF5340DK (ARM Cortex-M33 @ 128MHz)
 * 
 * 日期：20251218
 */

#ifndef SIGNAL_RECONSTRUCTION_H
#define SIGNAL_RECONSTRUCTION_H

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* ============ 緩衝區配置 ============ */
#define SAMPLE_BUFFER_SIZE      100     // 樣本緩衝區大小
#define HAMPEL_WINDOW_SIZE      7       // Hampel 濾波器窗口大小（必須為奇數）
#define HAMPEL_THRESHOLD        3.0f    // Hampel 閾值（MAD 倍數）
#define MAD_SCALE_FACTOR        1.4826f // MAD 到標準差的轉換因子（高斯分佈）

/* ============ 相位鎖定配置 ============ */
#define MIN_PERIOD_MS           200     // 最小週期 (ms)
#define MAX_PERIOD_MS           500     // 最大週期 (ms)
#define ZERO_CROSSING_HYSTERESIS 0.5f   // 零交叉檢測遲滯 (mA)

/* ============ Cubic Spline 配置 ============ */
#define SPLINE_POINTS           20      // 用於插值的點數
#define INTERPOLATION_FACTOR    4       // 插值倍數（用於峰值檢測）

/* ============ 樣本質量標記 ============ */
typedef enum {
    QUALITY_MEASURED = 0,       // 正常測量值
    QUALITY_FILTERED = 1,       // 經過濾波修正
    QUALITY_INTERPOLATED = 2,   // 插值重建
    QUALITY_PHASE_LOCKED = 3,   // 相位鎖定重建
    QUALITY_INVALID = 4         // 無效數據
} sample_quality_t;

/* ============ 單一樣本結構 ============ */
typedef struct {
    float value;                // 電流值 (mA)
    float timestamp;            // 時間戳 (秒)
    sample_quality_t quality;   // 樣本質量標記
} sample_t;

/* ============ 循環緩衝區結構 ============ */
typedef struct {
    sample_t samples[SAMPLE_BUFFER_SIZE];
    int write_idx;              // 寫入索引
    int count;                  // 當前樣本數量
    float last_timestamp;       // 上一個時間戳
    float expected_interval;    // 預期取樣間隔 (秒)
} sample_buffer_t;

/* ============ 週期追蹤器結構 ============ */
typedef struct {
    float last_zero_crossing;   // 上一次零交叉時間
    float measured_period;      // 測量到的週期 (秒)
    int samples_per_period;     // 每週期樣本數
    float dc_offset;            // 直流偏移量
    bool period_valid;          // 週期是否有效
    int zero_crossing_count;    // 零交叉計數
    float prev_value;           // 前一個值（用於零交叉檢測）
} period_tracker_t;

/* ============ Hampel 濾波器結構 ============ */
typedef struct {
    float window[HAMPEL_WINDOW_SIZE];
    int window_idx;
    int window_count;
    int outliers_detected;      // 檢測到的異常值數量
} hampel_filter_t;

/* ============ Cubic Spline 結構 ============ */
typedef struct {
    float x[SPLINE_POINTS];     // x 座標（時間）
    float y[SPLINE_POINTS];     // y 座標（電流值）
    float a[SPLINE_POINTS];     // 係數 a
    float b[SPLINE_POINTS];     // 係數 b
    float c[SPLINE_POINTS];     // 係數 c
    float d[SPLINE_POINTS];     // 係數 d
    int n;                      // 點數
    bool coefficients_valid;    // 係數是否有效
} cubic_spline_t;

/* ============ 信號重建器主結構 ============ */
typedef struct {
    sample_buffer_t buffer;
    period_tracker_t period_tracker;
    hampel_filter_t hampel;
    cubic_spline_t spline;
    
    // 統計信息
    int total_samples;
    int filtered_samples;
    int interpolated_samples;
    int phase_locked_samples;
    
    // 輸出結果
    float current_filtered;     // 濾波後的電流值
    float peak_current;         // 峰值電流
    float rms_current;          // RMS 電流
    float average_current;      // 平均電流
} signal_reconstructor_t;

/* ============ 函數原型 ============ */

// 初始化函數
void signal_reconstructor_init(signal_reconstructor_t *sr, float expected_sample_interval);
void sample_buffer_init(sample_buffer_t *buf, float expected_interval);
void period_tracker_init(period_tracker_t *pt);
void hampel_filter_init(hampel_filter_t *hf);
void cubic_spline_init(cubic_spline_t *spline);

// Hampel 濾波器
float hampel_filter_process(hampel_filter_t *hf, float value, bool *is_outlier);
float quick_select_median(float *arr, int n);

// 相位鎖定重建
void period_tracker_update(period_tracker_t *pt, float value, float timestamp);
float phase_locked_reconstruct(signal_reconstructor_t *sr, float timestamp);
float get_current_phase(period_tracker_t *pt, float timestamp);

// Cubic Spline 插值
void cubic_spline_compute_coefficients(cubic_spline_t *spline);
float cubic_spline_evaluate(cubic_spline_t *spline, float x);
void cubic_spline_add_point(cubic_spline_t *spline, float x, float y);

// 樣本緩衝區操作
void sample_buffer_add(sample_buffer_t *buf, float value, float timestamp, sample_quality_t quality);
sample_t* sample_buffer_get(sample_buffer_t *buf, int offset);
int sample_buffer_detect_gaps(sample_buffer_t *buf, float *gap_timestamps, int max_gaps);

// 主處理流程
float signal_reconstructor_process(signal_reconstructor_t *sr, float raw_value, float timestamp);
void signal_reconstructor_update_statistics(signal_reconstructor_t *sr);

// 工具函數
void signal_reconstructor_reset_statistics(signal_reconstructor_t *sr);
void signal_reconstructor_get_statistics(signal_reconstructor_t *sr, 
                                          float *peak, float *rms, float *avg,
                                          int *total, int *filtered, int *interpolated);

#endif // SIGNAL_RECONSTRUCTION_H