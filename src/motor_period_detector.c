/*
 * motor_period_detector.c
 * 
 * 馬達週期檢測模組實現
 * 
 * 檢測方法：
 * 1. 峰值檢測法 - 通過檢測電流峰值來確定週期
 * 2. 零交叉檢測法 - 通過信號穿越直流偏移點來確定週期
 * 3. 閾值檢測法 - 通過信號穿越設定閾值來確定週期
 * 
 * 日期：20251218
 */

#include "motor_period_detector.h"
#include <string.h>
#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(period_detector, LOG_LEVEL_ERR);

/* ============ 內部輔助函數 ============ */

/**
 * @brief 計算週期統計數據
 */
static void calculate_period_statistics(motor_period_detector_t *detector, 
                                         float end_time) {
    period_data_t *pd = &detector->last_period;
    int n = detector->buffer_idx;
    
    if (n < MIN_SAMPLES_PER_PERIOD) {
        pd->valid = false;
        return;
    }
    
    /* 計算基本統計 */
    float sum = 0.0f;
    float sum_sq = 0.0f;
    float max_val = -1e10f;
    float min_val = 1e10f;
    float voltage_sum = 0.0f;
    
    for (int i = 0; i < n; i++) {
        float c = detector->current_buffer[i];
        sum += c;
        sum_sq += c * c;
        
        if (c > max_val) max_val = c;
        if (c < min_val) min_val = c;
        
        voltage_sum += detector->voltage_buffer[i];
    }
    
    /* 填充週期數據 */
    pd->sample_count = n;
    pd->start_time = detector->time_buffer[0];
    pd->end_time = end_time;
    pd->period_time = end_time - pd->start_time;
    pd->average_current = sum / n;
    pd->peak_current = max_val;
    pd->min_current = min_val;
    pd->rms_current = sqrtf(sum_sq / n);
    pd->voltage = voltage_sum / n;
    
    /* 計算能量 (mJ) = 平均電流(mA) × 電壓(V) × 時間(s) */
    pd->energy = pd->average_current * pd->voltage * pd->period_time;
    
    /* 驗證週期是否在有效範圍內 */
    float period_ms = pd->period_time * 1000.0f;
    if (period_ms >= MIN_PERIOD_MS && period_ms <= MAX_PERIOD_MS) {
        pd->valid = true;
        
        /* 更新週期統計 */
        detector->measured_period = pd->period_time;
        detector->period_sum += pd->period_time;
        detector->period_count++;
        detector->average_period = detector->period_sum / detector->period_count;
    } else {
        pd->valid = false;
    }
}

/**
 * @brief 重置週期緩衝區
 */
static void reset_period_buffer(motor_period_detector_t *detector) {
    detector->buffer_idx = 0;
    detector->current_max = -1e10f;
    detector->samples_since_max = 0;
}

/**
 * @brief 添加樣本到緩衝區
 */
static void add_sample_to_buffer(motor_period_detector_t *detector,
                                  float current, float voltage, float timestamp) {
    if (detector->buffer_idx < PERIOD_BUFFER_SIZE) {
        detector->current_buffer[detector->buffer_idx] = current;
        detector->voltage_buffer[detector->buffer_idx] = voltage;
        detector->time_buffer[detector->buffer_idx] = timestamp;
        detector->buffer_idx++;
    }
}

/* ============ 峰值檢測法 ============ */

/**
 * @brief 使用峰值檢測法處理樣本
 * 
 * 原理：檢測電流信號的峰值，兩個連續峰值之間為一個週期
 * 
 * 峰值判定：當信號達到局部最大值後開始下降，且下降超過一定樣本數，
 *          則認為之前的最大值為峰值
 */
static bool process_peak_detection(motor_period_detector_t *detector,
                                    float current, float voltage, float timestamp) {
    bool period_complete = false;
    
    /* 更新當前最大值 */
    if (current > detector->current_max) {
        detector->current_max = current;
        detector->current_max_time = timestamp;
        detector->samples_since_max = 0;
    } else {
        detector->samples_since_max++;
    }
    
    switch (detector->state) {
        case STATE_INIT:
        case STATE_WAITING_FIRST_PEAK:
            /* 等待第一個峰值 */
            if (detector->samples_since_max >= PEAK_DETECTION_WINDOW) {
                /* 檢測到第一個峰值 */
                detector->last_peak_value = detector->current_max;
                detector->last_peak_time = detector->current_max_time;
                detector->state = STATE_COLLECTING;
                
                /* 重置緩衝區並開始收集 */
                reset_period_buffer(detector);
                add_sample_to_buffer(detector, current, voltage, timestamp);
                
                LOG_DBG("First peak detected: %.2f mA at %.3f s", 
                        (double)detector->last_peak_value, 
                        (double)detector->last_peak_time);
            }
            break;
            
        case STATE_COLLECTING:
            /* 收集週期數據 */
            add_sample_to_buffer(detector, current, voltage, timestamp);
            
            /* 檢測下一個峰值 */
            if (detector->samples_since_max >= PEAK_DETECTION_WINDOW) {
                /* 驗證這是一個有效的新峰值 */
                float time_diff = detector->current_max_time - detector->last_peak_time;
                float time_diff_ms = time_diff * 1000.0f;
                
                if (time_diff_ms >= MIN_PERIOD_MS && time_diff_ms <= MAX_PERIOD_MS) {
                    /* 有效週期完成 */
                    calculate_period_statistics(detector, detector->current_max_time);
                    
                    if (detector->last_period.valid) {
                        period_complete = true;
                        detector->state = STATE_PERIOD_COMPLETE;
                        
                        /* 調用回調函數 */
                        if (detector->on_period_complete) {
                            detector->on_period_complete(&detector->last_period);
                        }
                    }
                    
                    /* 準備下一個週期 */
                    detector->last_peak_value = detector->current_max;
                    detector->last_peak_time = detector->current_max_time;
                    reset_period_buffer(detector);
                    detector->state = STATE_COLLECTING;
                }
                
                /* 重置最大值追蹤 */
                detector->current_max = current;
                detector->current_max_time = timestamp;
                detector->samples_since_max = 0;
            }
            break;
            
        case STATE_PERIOD_COMPLETE:
            /* 週期完成後繼續收集下一個週期 */
            detector->state = STATE_COLLECTING;
            add_sample_to_buffer(detector, current, voltage, timestamp);
            break;
    }
    
    return period_complete;
}

/* ============ 零交叉檢測法 ============ */

/**
 * @brief 使用零交叉檢測法處理樣本
 * 
 * 原理：檢測信號從低於直流偏移到高於直流偏移的穿越點（上升沿零交叉）
 *      兩個連續上升沿零交叉之間為一個週期
 */
static bool process_zero_crossing(motor_period_detector_t *detector,
                                   float current, float voltage, float timestamp) {
    bool period_complete = false;
    
    /* 更新直流偏移（使用慢速指數移動平均）*/
    float alpha = 0.001f;
    detector->dc_offset = alpha * current + (1.0f - alpha) * detector->dc_offset;
    
    /* 判斷當前值相對於偏移量的位置 */
    bool current_above = (current > detector->dc_offset);
    
    /* 檢測上升沿零交叉 */
    bool rising_cross = (!detector->prev_above_offset && current_above);
    
    switch (detector->state) {
        case STATE_INIT:
            /* 初始化直流偏移 */
            detector->dc_offset = current;
            detector->prev_above_offset = current_above;
            detector->state = STATE_WAITING_FIRST_PEAK;
            break;
            
        case STATE_WAITING_FIRST_PEAK:
            if (rising_cross) {
                /* 第一個零交叉點 */
                detector->last_peak_time = timestamp;
                detector->state = STATE_COLLECTING;
                reset_period_buffer(detector);
                
                LOG_DBG("First zero crossing at %.3f s, DC offset: %.2f mA", 
                        (double)timestamp, (double)detector->dc_offset);
            }
            break;
            
        case STATE_COLLECTING:
            add_sample_to_buffer(detector, current, voltage, timestamp);
            
            if (rising_cross) {
                /* 下一個零交叉點 - 週期完成 */
                float time_diff = timestamp - detector->last_peak_time;
                float time_diff_ms = time_diff * 1000.0f;
                
                if (time_diff_ms >= MIN_PERIOD_MS && time_diff_ms <= MAX_PERIOD_MS) {
                    calculate_period_statistics(detector, timestamp);
                    
                    if (detector->last_period.valid) {
                        period_complete = true;
                        
                        if (detector->on_period_complete) {
                            detector->on_period_complete(&detector->last_period);
                        }
                    }
                }
                
                /* 準備下一個週期 */
                detector->last_peak_time = timestamp;
                reset_period_buffer(detector);
            }
            break;
            
        case STATE_PERIOD_COMPLETE:
            detector->state = STATE_COLLECTING;
            break;
    }
    
    /* 更新前一個值的狀態 */
    detector->prev_value = current;
    detector->prev_above_offset = current_above;
    
    return period_complete;
}

/* ============ 閾值檢測法 ============ */

/**
 * @brief 使用閾值檢測法處理樣本
 * 
 * 原理：設定上下閾值，檢測信號從下閾值以下上升到上閾值以上的時刻
 *      兩個連續上升穿越之間為一個週期
 */
static bool process_threshold(motor_period_detector_t *detector,
                               float current, float voltage, float timestamp) {
    bool period_complete = false;
    
    /* 判斷當前狀態 */
    bool now_above = (current > detector->threshold_high);
    bool now_below = (current < detector->threshold_low);
    
    switch (detector->state) {
        case STATE_INIT:
            detector->above_threshold = now_above;
            detector->state = STATE_WAITING_FIRST_PEAK;
            break;
            
        case STATE_WAITING_FIRST_PEAK:
            /* 等待從低到高的穿越 */
            if (!detector->above_threshold && now_above) {
                detector->last_peak_time = timestamp;
                detector->state = STATE_COLLECTING;
                reset_period_buffer(detector);
                detector->above_threshold = true;
                
                LOG_DBG("First threshold crossing at %.3f s", (double)timestamp);
            } else if (now_below) {
                detector->above_threshold = false;
            }
            break;
            
        case STATE_COLLECTING:
            add_sample_to_buffer(detector, current, voltage, timestamp);
            
            /* 檢測狀態變化 */
            if (now_below) {
                detector->above_threshold = false;
            } else if (!detector->above_threshold && now_above) {
                /* 新的上升穿越 - 週期完成 */
                float time_diff = timestamp - detector->last_peak_time;
                float time_diff_ms = time_diff * 1000.0f;
                
                if (time_diff_ms >= MIN_PERIOD_MS && time_diff_ms <= MAX_PERIOD_MS) {
                    calculate_period_statistics(detector, timestamp);
                    
                    if (detector->last_period.valid) {
                        period_complete = true;
                        
                        if (detector->on_period_complete) {
                            detector->on_period_complete(&detector->last_period);
                        }
                    }
                }
                
                /* 準備下一個週期 */
                detector->last_peak_time = timestamp;
                reset_period_buffer(detector);
                detector->above_threshold = true;
            }
            break;
            
        case STATE_PERIOD_COMPLETE:
            detector->state = STATE_COLLECTING;
            break;
    }
    
    return period_complete;
}

/* ============ 公開 API 實現 ============ */

void period_detector_init(motor_period_detector_t *detector, detect_method_t method) {
    memset(detector, 0, sizeof(motor_period_detector_t));
    
    detector->method = method;
    detector->state = STATE_INIT;
    detector->current_max = -1e10f;
    detector->dc_offset = 50.0f;  // 預設直流偏移（可根據實際情況調整）
    detector->threshold_high = 52.0f;  // 預設上閾值
    detector->threshold_low = 50.0f;   // 預設下閾值
    detector->last_period.valid = false;
    
    const char *method_name;
    switch (method) {
        case DETECT_METHOD_PEAK:
            method_name = "Peak Detection";
            break;
        case DETECT_METHOD_ZERO_CROSS:
            method_name = "Zero Crossing";
            break;
        case DETECT_METHOD_THRESHOLD:
            method_name = "Threshold";
            break;
        default:
            method_name = "Unknown";
    }
    
    LOG_INF("Period detector initialized with method: %s", method_name);
}

void period_detector_set_callback(motor_period_detector_t *detector, 
                                   void (*callback)(period_data_t *data)) {
    detector->on_period_complete = callback;
}

bool period_detector_process(motor_period_detector_t *detector, 
                             float current, 
                             float voltage,
                             float timestamp) {
    switch (detector->method) {
        case DETECT_METHOD_PEAK:
            return process_peak_detection(detector, current, voltage, timestamp);
            
        case DETECT_METHOD_ZERO_CROSS:
            return process_zero_crossing(detector, current, voltage, timestamp);
            
        case DETECT_METHOD_THRESHOLD:
            return process_threshold(detector, current, voltage, timestamp);
            
        default:
            return false;
    }
}

void period_detector_set_threshold(motor_period_detector_t *detector, 
                                    float high, float low) {
    detector->threshold_high = high;
    detector->threshold_low = low;
    LOG_INF("Threshold set: high=%.2f, low=%.2f", (double)high, (double)low);
}

period_data_t* period_detector_get_last_period(motor_period_detector_t *detector) {
    return &detector->last_period;
}

float period_detector_get_average_period(motor_period_detector_t *detector) {
    if (detector->period_count > 0) {
        return detector->average_period;
    }
    return 0.0f;
}

int period_detector_get_period_count(motor_period_detector_t *detector) {
    return detector->period_count;
}

void period_detector_reset(motor_period_detector_t *detector) {
    detect_method_t method = detector->method;
    void (*callback)(period_data_t *) = detector->on_period_complete;
    float th_high = detector->threshold_high;
    float th_low = detector->threshold_low;
    
    period_detector_init(detector, method);
    
    detector->on_period_complete = callback;
    detector->threshold_high = th_high;
    detector->threshold_low = th_low;
    
    LOG_INF("Period detector reset");
}

void period_detector_update_dc_offset(motor_period_detector_t *detector, float current) {
    float alpha = 0.01f;
    detector->dc_offset = alpha * current + (1.0f - alpha) * detector->dc_offset;
}