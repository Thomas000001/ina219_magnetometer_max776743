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
 * @brief 計算峰值附近的平均電流 (AC)
 * @param detector 檢測器指針
 * @param peak_idx 峰值在緩衝區中的索引
 * @param ac_value 輸出的 AC 值
 * @return 是否成功計算
 * 20251230新增
 */
// static bool calculate_ac_current(motor_period_detector_t *detector, 
//                                   int peak_idx, 
//                                   float *ac_value) {
//     int window = AC_DC_WINDOW_SIZE;
//     int start_idx = peak_idx - window;
//     int end_idx = peak_idx + window;
    
//     /* 檢查邊界 */
//     if (start_idx < 0 || end_idx >= detector->buffer_idx) {
//         return false;
//     }
    
//     /* 計算平均值 */
//     float sum = 0.0f;
//     int count = 0;
//     for (int i = start_idx; i <= end_idx; i++) {
//         sum += detector->current_buffer[i];
//         count++;
//     }
    
//     if (count > 0) {
//         *ac_value = sum / count;
//         return true;
//     }
    
//     return false;
// }

/**
 * @brief 使用百分比閾值法計算 AC 電流和 AC 時間
 * 
 * 方法：
 * 1. 計算閾值 = 谷值 + (峰值 - 谷值) × 百分比
 * 2. 從峰值向左找第一個低於閾值的點 → t_start
 * 3. 從峰值向右找第一個低於閾值的點 → t_end
 * 4. AC_time = t_end - t_start
 * 5. AC_current = 閾值範圍內所有樣本的平均值
 */
static bool calculate_ac_with_threshold(motor_period_detector_t *detector,
                                         int peak_idx,
                                         float peak_value,
                                         float valley_value,
                                         period_data_t *pd) {
    int n = detector->buffer_idx;
    
    /* 檢查基本條件 */
    if (peak_idx < 0 || peak_idx >= n || n < MIN_SAMPLES_PER_PERIOD) {
        pd->ac_valid = false;
        return false;
    }
    
    /* 計算閾值：谷值 + (峰值 - 谷值) × 百分比 */
    float amplitude = peak_value - valley_value;
    if (amplitude <= 0.0f) {
        pd->ac_valid = false;
        return false;
    }
    
    float threshold = valley_value + amplitude * AC_THRESHOLD_PERCENT;
    pd->ac_threshold = threshold;
    
    /* 從峰值向左搜索，找到第一個低於閾值的點 */
    int left_idx = peak_idx;
    for (int i = peak_idx - 1; i >= 0; i--) {
        if (detector->current_buffer[i] < threshold) {
            left_idx = i + 1;  /* 第一個高於閾值的點 */
            break;
        }
        if (i == 0) {
            left_idx = 0;  /* 整個左側都高於閾值 */
        }
    }
    
    /* 從峰值向右搜索，找到第一個低於閾值的點 */
    int right_idx = peak_idx;
    for (int i = peak_idx + 1; i < n; i++) {
        if (detector->current_buffer[i] < threshold) {
            right_idx = i - 1;  /* 最後一個高於閾值的點 */
            break;
        }
        if (i == n - 1) {
            right_idx = n - 1;  /* 整個右側都高於閾值 */
        }
    }
    
    /* 確保索引有效 */
    if (left_idx > right_idx || left_idx < 0 || right_idx >= n) {
        pd->ac_valid = false;
        return false;
    }
    
    /* 計算 AC 時間 */
    pd->ac_start_time = detector->time_buffer[left_idx];
    pd->ac_end_time = detector->time_buffer[right_idx];
    pd->ac_time = pd->ac_end_time - pd->ac_start_time;
    
    /* 計算 AC 區域內的平均電流 */
    float sum = 0.0f;
    int count = 0;
    for (int i = left_idx; i <= right_idx; i++) {
        sum += detector->current_buffer[i];
        count++;
    }
    
    if (count > 0) {
        pd->ac_current = sum / count;
        pd->ac_sample_count = count;
        pd->ac_valid = true;
        
        LOG_DBG("AC calculated: threshold=%.2f mA, time=%.4f s, "
                "avg=%.2f mA, samples=%d, idx=[%d,%d]",
                (double)threshold,
                (double)pd->ac_time,
                (double)pd->ac_current,
                count,
                left_idx, right_idx);
        
        return true;
    }
    
    pd->ac_valid = false;
    return false;
}

/**
 * @brief 計算谷值附近的平均電流 (DC)
 * @param detector 檢測器指針
 * @param valley_idx 谷值在緩衝區中的索引
 * @param dc_value 輸出的 DC 值
 * @return 是否成功計算
 * 20251230新增
 */
static bool calculate_dc_current(motor_period_detector_t *detector, 
                                  int valley_idx, 
                                  float *dc_value) {
    int window = AC_DC_WINDOW_SIZE;
    int start_idx = valley_idx - window;
    int end_idx = valley_idx + window;
    
    /* 檢查邊界 */
    if (start_idx < 0 || end_idx >= detector->buffer_idx) {
        return false;
    }
    
    /* 計算平均值 */
    float sum = 0.0f;
    int count = 0;
    for (int i = start_idx; i <= end_idx; i++) {
        sum += detector->current_buffer[i];
        count++;
    }
    
    if (count > 0) {
        *dc_value = sum / count;
        return true;
    }
    
    return false;
}

/**
 * @brief 計算週期統計數據
 */
static void calculate_period_statistics(motor_period_detector_t *detector, 
                                         float start_time, float end_time, float peak_current) {
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
    pd->start_time = start_time;
    pd->end_time = end_time;
    pd->peak_time = detector->current_max_time;           
    pd->valley_time = detector->confirmed_valley_time;    
    pd->period_time = end_time - start_time;
    pd->average_current = sum / n;
    pd->peak_current = peak_current;
    pd->min_current = min_val;
    pd->rms_current = sqrtf(sum_sq / n);
    pd->voltage = voltage_sum / n;
    
    /* 計算能量 (mJ) = 平均電流(mA) × 電壓(V) × 時間(s) */
    pd->energy = pd->average_current * pd->voltage * pd->period_time;

    pd->ac_valid = false;
    pd->ac_current = 0.0f;
    pd->ac_time = 0.0f;
    pd->ac_start_time = 0.0f;
    pd->ac_end_time = 0.0f;
    pd->ac_sample_count = 0;
    pd->ac_threshold = 0.0f;

    /* DC 計算保持不變 */
    pd->dc_valid = calculate_dc_current(detector, detector->valley_buffer_idx, &pd->dc_current);
    
    if (pd->dc_valid) {
        LOG_DBG("DC (valley area avg): %.2f mA at idx %d", 
                (double)pd->dc_current, detector->valley_buffer_idx);
    }
    
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
 * @brief 根據時間戳在緩衝區中找到最接近的索引
 * @param detector 檢測器指針
 * @param target_time 目標時間戳
 * @return 最接近的索引，找不到返回 -1
 */
static int find_index_by_time(motor_period_detector_t *detector, float target_time) {
    int n = detector->buffer_idx;
    if (n == 0) return -1;
    
    int best_idx = -1;
    float min_diff = 1e10f;
    
    for (int i = 0; i < n; i++) {
        float diff = fabsf(detector->time_buffer[i] - target_time);
        if (diff < min_diff) {
            min_diff = diff;
            best_idx = i;
        }
    }
    
    return best_idx;
}

/**
 * @brief 計算待處理週期的 AC 時間
 * 
 * 在谷值確認後調用，此時緩衝區包含完整的「峰值→谷值」下降數據
 * 
 * @param detector 檢測器指針
 * @return true 如果 AC 計算成功
 */
static bool calculate_pending_ac(motor_period_detector_t *detector) {
    if (!detector->pending_period.pending) {
        return false;
    }
    
    pending_period_t *pp = &detector->pending_period;
    period_data_t *pd = &pp->data;
    
    int n = detector->buffer_idx;
    int peak_idx = pp->peak_buffer_idx;
    float peak_value = pp->peak_value;
    float valley_value = detector->confirmed_valley_value;
    
    /* 檢查基本條件 */
    if (peak_idx < 0 || peak_idx >= n || n < MIN_SAMPLES_PER_PERIOD) {
        LOG_WRN("AC calc failed: invalid indices (peak_idx=%d, n=%d)", peak_idx, n);
        pd->ac_valid = false;
        return false;
    }
    
    /* 計算閾值：谷值 + (峰值 - 谷值) × 百分比 */
    float amplitude = peak_value - valley_value;
    if (amplitude <= 0.0f) {
        LOG_WRN("AC calc failed: invalid amplitude (peak=%.2f, valley=%.2f)", 
                (double)peak_value, (double)valley_value);
        pd->ac_valid = false;
        return false;
    }
    
    float threshold = valley_value + amplitude * AC_THRESHOLD_PERCENT;
    pd->ac_threshold = threshold;
    
    /* 從峰值向左搜索，找到第一個低於閾值的點 */
    int left_idx = peak_idx;
    for (int i = peak_idx - 1; i >= 0; i--) {
        if (detector->current_buffer[i] < threshold) {
            left_idx = i + 1;  /* 第一個高於閾值的點 */
            break;
        }
        if (i == 0) {
            left_idx = 0;  /* 整個左側都高於閾值 */
        }
    }
    
    /* 從峰值向右搜索，找到第一個低於閾值的點 */
    /* 【關鍵修改】：現在緩衝區包含完整的下降數據，可以正確找到 */
    int right_idx = peak_idx;
    for (int i = peak_idx + 1; i < n; i++) {
        if (detector->current_buffer[i] < threshold) {
            right_idx = i - 1;  /* 最後一個高於閾值的點 */
            break;
        }
        if (i == n - 1) {
            right_idx = n - 1;  /* 整個右側都高於閾值（不應該發生）*/
            LOG_WRN("AC calc: right search reached buffer end");
        }
    }
    
    /* 確保索引有效 */
    if (left_idx > right_idx || left_idx < 0 || right_idx >= n) {
        LOG_WRN("AC calc failed: invalid range [%d, %d]", left_idx, right_idx);
        pd->ac_valid = false;
        return false;
    }
    
    /* 計算 AC 時間 */
    pd->ac_start_time = detector->time_buffer[left_idx];
    pd->ac_end_time = detector->time_buffer[right_idx];
    pd->ac_time = pd->ac_end_time - pd->ac_start_time;
    
    /* 計算 AC 區域內的平均電流 */
    float sum = 0.0f;
    int count = 0;
    for (int i = left_idx; i <= right_idx; i++) {
        sum += detector->current_buffer[i];
        count++;
    }
    
    if (count > 0) {
        pd->ac_current = sum / count;
        pd->ac_sample_count = count;
        pd->ac_valid = true;
        
        LOG_DBG("AC calculated (delayed): threshold=%.2f mA, time=%.4f s, "
                "avg=%.2f mA, samples=%d, idx=[%d,%d]",
                (double)threshold,
                (double)pd->ac_time,
                (double)pd->ac_current,
                count,
                left_idx, right_idx);
        
        return true;
    }
    
    pd->ac_valid = false;
    return false;
}

/**
 * @brief 重置週期緩衝區
 */
static void reset_period_buffer(motor_period_detector_t *detector) {
    detector->buffer_idx = 0;
}

/**
 * @brief 重置峰值追蹤（用於開始尋找新的峰值）
 */
static void reset_peak_tracking(motor_period_detector_t *detector) {
    detector->current_max = -1e10f;
    detector->current_max_time = 0.0f;
    detector->samples_since_max = 0;
    detector->peak_buffer_idx = -1;
}

/**
 * @brief 重置谷值追蹤（用於開始尋找谷值）
 */
static void reset_valley_tracking(motor_period_detector_t *detector, float current, float timestamp) {
    detector->current_min = current;
    detector->current_min_time = timestamp;
    detector->samples_since_min = 0;
    detector->valley_buffer_idx = -1;
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
    
    switch (detector->state) {
        case STATE_INIT:
        case STATE_WAITING_FIRST_PEAK:
            /* ===== 等待第一個峰值 ===== */
            
            /* 更新當前最大值 */
            if (current > detector->current_max) {
                detector->current_max = current;
                detector->current_max_time = timestamp;
                detector->samples_since_max = 0;
            } else {
                detector->samples_since_max++;
            }
            
            /* 檢測是否達到峰值（信號開始下降） */
            if (detector->samples_since_max >= PEAK_DETECTION_WINDOW) {
                /* 檢測到第一個峰值 */
                detector->last_peak_value = detector->current_max;
                detector->last_peak_time = detector->current_max_time;
                
                /* ★ 新增20260104：同時設定 confirmed_peak，供第一次谷值驗證使用 ★ */
                detector->confirmed_peak_value = detector->current_max;
                detector->confirmed_peak_time = detector->current_max_time;
                detector->peak_confirmed = true;

                LOG_DBG("First peak detected: %.2f mA at %.3f s", 
                        (double)detector->last_peak_value, 
                        (double)detector->last_peak_time);
                
                /* 進入等待谷值狀態 */
                detector->state = STATE_WAITING_VALLEY;
                reset_valley_tracking(detector, current, timestamp);
                
                /* 開始收集週期數據 */
                reset_period_buffer(detector);
                add_sample_to_buffer(detector, current, voltage, timestamp);
            }
            break;
            
        case STATE_WAITING_VALLEY:
            /* ===== 等待谷值（信號下降階段） ===== */
            
            /* 持續收集數據 */
            add_sample_to_buffer(detector, current, voltage, timestamp);
            
            /* 追蹤最小值 */
            if (current <= detector->current_min) {
                detector->current_min = current;
                detector->current_min_time = timestamp;
                detector->samples_since_min = 0;
                /* 新增20251230：記錄谷值在緩衝區中的索引 */
                detector->valley_buffer_idx = detector->buffer_idx - 1;
            } else {
                detector->samples_since_min++;
            }
            
            /* 檢測是否達到谷值（信號開始上升） */
            if (detector->samples_since_min >= VALLEY_DETECTION_WINDOW) {
                
                /* ★ 20260104新增：Valley Prominence 檢查 ★ */
                float valley_prominence = detector->confirmed_peak_value - detector->current_min;
                if (valley_prominence < MIN_PROMINENCE_MA) {
                    /* 這是假谷值！峰值區域的微小下降 */
                    detector->false_valley_count++;  // 需要在結構體中新增此計數器
                    
                    LOG_INF("假谷值被過濾 #%d: valley=%.2f mA, peak=%.2f mA, "
                            "prominence=%.2f mA < %.2f mA (閾值)",
                            detector->false_valley_count,
                            (double)detector->current_min,
                            (double)detector->last_peak_value,
                            (double)valley_prominence,
                            (double)MIN_PROMINENCE_MA);
                    
                    /* 重置谷值追蹤，繼續尋找真正的谷值 */
                    detector->current_min = current;
                    detector->current_min_time = timestamp;
                    detector->samples_since_min = 0;
                    
                    /* 保持在 STATE_WAITING_VALLEY 狀態 */
                    break;
                }

                /* ★ prominence 足夠大，這是真正的谷值 ★ */
                LOG_DBG("Valid valley: %.2f mA, prominence=%.2f mA",
                        (double)detector->current_min, (double)valley_prominence);
                
                /* 確認谷值，進入收集狀態尋找下一個峰值 */


                /* ★ 確認谷值，保存用於後續 prominence 計算 ★ */
                detector->confirmed_valley_value = detector->current_min;
                detector->confirmed_valley_time = detector->current_min_time;
                detector->valley_confirmed = true;

                LOG_DBG("Valley detected: %.2f mA at %.3f s", 
                        (double)detector->current_min, 
                        (double)detector->current_min_time);
                
                 /* ↓↓↓ 新增20260110：谷值確認後，計算待處理週期的 AC 並觸發回調 ↓↓↓ */
                if (detector->pending_period.pending) {
                    /* 更新待處理週期的谷值時間 */
                    detector->pending_period.data.valley_time = detector->confirmed_valley_time;
                    
                    /* 計算 AC 時間 */
                    calculate_pending_ac(detector);
                    
                    /* 複製回 last_period 以保持一致性 */
                    memcpy(&detector->last_period, 
                           &detector->pending_period.data, 
                           sizeof(period_data_t));
                
                    /* 觸發回調 */
                    if (detector->on_period_complete) {
                        detector->on_period_complete(&detector->pending_period.data);
                    }
                    
                    period_complete = true;
                    
                    /* 清除待處理標記 */
                    detector->pending_period.pending = false;
                    
                    LOG_DBG("Period complete with AC: time=%.4f s, AC_time=%.4f s", 
                            (double)detector->pending_period.data.period_time,
                            (double)detector->pending_period.data.ac_time);
                }

                /* 重置緩衝區，準備下一個週期 */
                reset_period_buffer(detector);

                detector->state = STATE_COLLECTING;

                /* 重置峰值追蹤，準備尋找下一個峰值 */
                reset_peak_tracking(detector);
                /* 將當前值作為新的峰值追蹤起點 */
                detector->current_max = current;
                detector->current_max_time = timestamp;
                detector->samples_since_max = 0;
                
                /* 開始收集新週期的數據 */
                add_sample_to_buffer(detector, current, voltage, timestamp);

                break;
            }
            
        case STATE_COLLECTING:
            /* ===== 收集週期數據，尋找下一個峰值（信號上升階段） ===== */
            
            /* 收集數據 */
            add_sample_to_buffer(detector, current, voltage, timestamp);
            
            /* 更新當前最大值 */
            if (current > detector->current_max) {
                detector->current_max = current;
                detector->current_max_time = timestamp;
                detector->samples_since_max = 0;
                /* 新增20251230：記錄峰值在緩衝區中的索引 */
                detector->peak_buffer_idx = detector->buffer_idx - 1;
            } else {
                detector->samples_since_max++;
            }
            
            /* 檢測是否達到新的峰值 */
            if (detector->samples_since_max >= PEAK_DETECTION_WINDOW) {
                /* 驗證這是一個有效的新峰值 */

                /* 新增20260101：Peak Prominence 檢查*/
                float prominence = detector->current_max - detector->confirmed_valley_value;
                
                if (prominence < MIN_PROMINENCE_MA) {
                    /*  這是假峰值！谷值區域的微小突起  */
                    detector->false_peak_count++;
                    
                    LOG_INF("假峰值被過濾 #%d: peak=%.2f mA, valley=%.2f mA, "
                            "prominence=%.2f mA < %.2f mA (閾值)",
                            detector->false_peak_count,
                            (double)detector->current_max, 
                            (double)detector->confirmed_valley_value, 
                            (double)prominence, 
                            (double)MIN_PROMINENCE_MA);
                    
                    /* 重置峰值追蹤，繼續尋找真正的峰值 */
                    detector->current_max = current;
                    detector->current_max_time = timestamp;
                    detector->samples_since_max = 0;
                    
                    /* 保持在 STATE_COLLECTING 狀態，繼續尋找 */
                    break;
                }
                
                /*  prominence 足夠大，這是真正的峰值  */
                LOG_DBG("Valid peak: %.2f mA, prominence=%.2f mA", 
                        (double)detector->current_max, (double)prominence);
                
                /* 保存已確認的峰值，供下次谷值驗證使用 */
                detector->confirmed_peak_value = detector->current_max;
                detector->confirmed_peak_time = detector->current_max_time;
                detector->peak_confirmed = true;

                float time_diff = detector->current_max_time - detector->last_peak_time;
                float time_diff_ms = time_diff * 1000.0f;
                
                if (time_diff_ms >= MIN_PERIOD_MS && time_diff_ms <= MAX_PERIOD_MS) {
                    /* 保存當前峰值資訊 */
                    float this_peak_value = detector->current_max;
                    float this_peak_time = detector->current_max_time;
                    
                    /* 計算週期統計 */
                    calculate_period_statistics(detector, 
                                                detector->last_peak_time, 
                                                this_peak_time, 
                                                this_peak_value);
                    
                    if (detector->last_period.valid) {
                         /* 暫存週期數據，等待谷值確認後計算 AC */
                        memcpy(&detector->pending_period.data, 
                               &detector->last_period, 
                               sizeof(period_data_t));
                        detector->pending_period.peak_value = this_peak_value;
                        detector->pending_period.peak_buffer_idx = detector->peak_buffer_idx;
                        detector->pending_period.pending = true;
                        
                        LOG_DBG("Period pending AC calculation: %.3f s", 
                                (double)detector->last_period.period_time);
                    }
                    
                    /* 更新上一個峰值資訊 */
                    detector->last_peak_value = this_peak_value;
                    detector->last_peak_time = this_peak_time;
                    
                    
                    /* 進入等待谷值狀態 */
                    detector->state = STATE_WAITING_VALLEY;
                    reset_valley_tracking(detector, current, timestamp);
                    
                    /* 繼續收集數據（當前樣本屬於下一個週期） */
                    add_sample_to_buffer(detector, current, voltage, timestamp);
                    
                } else if (time_diff_ms > MAX_PERIOD_MS) {
                    /* 週期過長，可能漏檢了峰值，重新開始 */
                    LOG_WRN("Period too long (%.1f ms), resetting", (double)time_diff_ms);
                    
                    detector->last_peak_value = detector->current_max;
                    detector->last_peak_time = detector->current_max_time;
                    reset_period_buffer(detector);
                    
                    detector->state = STATE_WAITING_VALLEY;
                    reset_valley_tracking(detector, current, timestamp);
                    add_sample_to_buffer(detector, current, voltage, timestamp);
                }
                /* 如果 time_diff_ms < MIN_PERIOD_MS，忽略這個假峰值，繼續等待 */
            }
            break;
            
        case STATE_PERIOD_COMPLETE:
            /* 這個狀態在新的設計中不再使用，直接轉到 STATE_WAITING_VALLEY */
            detector->state = STATE_WAITING_VALLEY;
            reset_valley_tracking(detector, current, timestamp);
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
                    float this_peak_value = detector->current_max;
                    
                    calculate_period_statistics(detector,
                                                detector->last_peak_time,  // 上一個零交叉時間
                                                timestamp,                  // 當前零交叉時間
                                                this_peak_value);           // 週期內的峰值電流
                    
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
                    float this_peak_value = detector->current_max;
                    
                    calculate_period_statistics(detector,
                                                detector->last_peak_time,  // 上一個零交叉時間
                                                timestamp,                  // 當前零交叉時間
                                                this_peak_value);           // 週期內的峰值電流
                    
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
    detector->confirmed_peak_value = 0.0f;
    detector->confirmed_peak_time = 0.0f;
    detector->peak_confirmed = false;
    detector->false_valley_count = 0;

    detector->peak_buffer_idx = -1;   // 新增
    detector->valley_buffer_idx = -1; // 新增
    detector->pending_period.pending = false;
    detector->pending_period.peak_value = 0.0f;
    detector->pending_period.peak_buffer_idx = -1;
    
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

    detector->pending_period.pending = false;
    
    LOG_INF("Period detector reset");
}

void period_detector_update_dc_offset(motor_period_detector_t *detector, float current) {
    float alpha = 0.01f;
    detector->dc_offset = alpha * current + (1.0f - alpha) * detector->dc_offset;
}