/*
 * signal_reconstruction.c
 * 
 * 信號重建模組實現
 * 
 * 處理流程：
 * 1. Hampel 濾波器 - 檢測並修正異常值
 * 2. 相位鎖定重建 - 利用週期性填補缺失樣本
 * 3. Cubic Spline - 平滑插值以還原波形細節
 * 
 * 日期：20251218
 */

#include "signal_reconstruction.h"
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(signal_recon, LOG_LEVEL_INF);

/* ============================================================
 * 初始化函數
 * ============================================================ */

/**
 * @brief 初始化信號重建器
 * @param sr 信號重建器指針
 * @param expected_sample_interval 預期取樣間隔（秒）
 */
void signal_reconstructor_init(signal_reconstructor_t *sr, float expected_sample_interval) {
    memset(sr, 0, sizeof(signal_reconstructor_t));
    
    sample_buffer_init(&sr->buffer, expected_sample_interval);
    period_tracker_init(&sr->period_tracker);
    hampel_filter_init(&sr->hampel);
    cubic_spline_init(&sr->spline);
    
    LOG_INF("Signal reconstructor initialized, interval=%.4fs", (double)expected_sample_interval);
}

/**
 * @brief 初始化樣本緩衝區
 */
void sample_buffer_init(sample_buffer_t *buf, float expected_interval) {
    memset(buf, 0, sizeof(sample_buffer_t));
    buf->expected_interval = expected_interval;
    buf->last_timestamp = -1.0f;  // 標記為未初始化
}

/**
 * @brief 初始化週期追蹤器
 */
void period_tracker_init(period_tracker_t *pt) {
    memset(pt, 0, sizeof(period_tracker_t));
    pt->measured_period = 0.35f;  // 預設週期 350ms（根據你的馬達）
    pt->samples_per_period = 58;  // 350ms / 6ms ≈ 58 樣本
    pt->period_valid = false;
    pt->prev_value = 0.0f;
}

/**
 * @brief 初始化 Hampel 濾波器
 */
void hampel_filter_init(hampel_filter_t *hf) {
    memset(hf, 0, sizeof(hampel_filter_t));
}

/**
 * @brief 初始化 Cubic Spline
 */
void cubic_spline_init(cubic_spline_t *spline) {
    memset(spline, 0, sizeof(cubic_spline_t));
    spline->coefficients_valid = false;
}

/* ============================================================
 * Hampel 濾波器實現
 * 
 * Hampel 濾波器使用中位數絕對偏差 (MAD) 來檢測異常值
 * 比基於均值的方法更能抵抗異常值的影響
 * ============================================================ */

/**
 * @brief 快速選擇算法找中位數（O(n) 平均複雜度）
 * @param arr 輸入陣列（會被修改）
 * @param n 陣列長度
 * @return 中位數
 */
float quick_select_median(float *arr, int n) {
    // 對於小陣列，使用簡單的插入排序
    float temp[HAMPEL_WINDOW_SIZE];
    memcpy(temp, arr, n * sizeof(float));
    
    // 插入排序（對於 n<=11 比快速選擇更快）
    for (int i = 1; i < n; i++) {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    
    // 返回中位數
    if (n % 2 == 0) {
        return (temp[n/2 - 1] + temp[n/2]) / 2.0f;
    } else {
        return temp[n/2];
    }
}

/**
 * @brief Hampel 濾波器處理單一樣本
 * @param hf Hampel 濾波器指針
 * @param value 輸入值
 * @param is_outlier 輸出：是否為異常值
 * @return 處理後的值（異常值會被替換為中位數）
 */
float hampel_filter_process(hampel_filter_t *hf, float value, bool *is_outlier) {
    *is_outlier = false;
    
    // 將新值加入窗口
    hf->window[hf->window_idx] = value;
    hf->window_idx = (hf->window_idx + 1) % HAMPEL_WINDOW_SIZE;
    
    if (hf->window_count < HAMPEL_WINDOW_SIZE) {
        hf->window_count++;
        return value;  // 窗口未滿，直接返回
    }
    
    // 計算中位數
    float median = quick_select_median(hf->window, HAMPEL_WINDOW_SIZE);
    
    // 計算絕對偏差
    float abs_dev[HAMPEL_WINDOW_SIZE];
    for (int i = 0; i < HAMPEL_WINDOW_SIZE; i++) {
        abs_dev[i] = fabsf(hf->window[i] - median);
    }
    
    // 計算 MAD（中位數絕對偏差）
    float mad = quick_select_median(abs_dev, HAMPEL_WINDOW_SIZE);
    
    // 計算閾值
    float threshold = HAMPEL_THRESHOLD * MAD_SCALE_FACTOR * mad;
    
    // 防止閾值過小（避免正常小波動被誤判）
    if (threshold < 0.5f) {
        threshold = 0.5f;
    }
    
    // 檢查當前值是否為異常值
    // 注意：我們檢查的是窗口中心位置的值
    int center_idx = (hf->window_idx + HAMPEL_WINDOW_SIZE/2) % HAMPEL_WINDOW_SIZE;
    float center_value = hf->window[center_idx];
    
    if (fabsf(center_value - median) > threshold) {
        *is_outlier = true;
        hf->outliers_detected++;
        hf->window[center_idx] = median;  // 替換異常值
        return median;
    }
    
    return center_value;
}

/* ============================================================
 * 週期追蹤與相位鎖定重建
 * 
 * 利用馬達電流的週期性特徵：
 * 1. 通過零交叉檢測追蹤信號週期
 * 2. 當樣本缺失時，參考上一個週期同相位位置的值
 * ============================================================ */

/**
 * @brief 更新週期追蹤器
 * @param pt 週期追蹤器指針
 * @param value 當前電流值
 * @param timestamp 時間戳
 */
void period_tracker_update(period_tracker_t *pt, float value, float timestamp) {
    // 更新 DC 偏移量（使用指數移動平均）
    float alpha = 0.01f;  // 慢速追蹤
    pt->dc_offset = alpha * value + (1.0f - alpha) * pt->dc_offset;
    
    // 將信號移除 DC 偏移
    float centered_value = value - pt->dc_offset;
    float centered_prev = pt->prev_value - pt->dc_offset;
    
    // 零交叉檢測（從負到正，帶遲滯）
    if (centered_prev < -ZERO_CROSSING_HYSTERESIS && 
        centered_value > ZERO_CROSSING_HYSTERESIS) {
        
        if (pt->zero_crossing_count > 0) {
            // 計算週期
            float new_period = timestamp - pt->last_zero_crossing;
            
            // 驗證週期是否在合理範圍內
            float min_period = MIN_PERIOD_MS / 1000.0f;
            float max_period = MAX_PERIOD_MS / 1000.0f;
            
            if (new_period >= min_period && new_period <= max_period) {
                // 使用指數移動平均平滑週期估計
                if (pt->period_valid) {
                    pt->measured_period = 0.7f * pt->measured_period + 0.3f * new_period;
                } else {
                    pt->measured_period = new_period;
                    pt->period_valid = true;
                }
                
                // 更新每週期樣本數
                // 假設取樣間隔約 6ms
                pt->samples_per_period = (int)(pt->measured_period / 0.006f + 0.5f);
            }
        }
        
        pt->last_zero_crossing = timestamp;
        pt->zero_crossing_count++;
    }
    
    pt->prev_value = value;
}

/**
 * @brief 獲取當前相位（0.0 ~ 1.0）
 * @param pt 週期追蹤器指針
 * @param timestamp 當前時間戳
 * @return 相位值（0.0 = 週期開始, 1.0 = 週期結束）
 */
float get_current_phase(period_tracker_t *pt, float timestamp) {
    if (!pt->period_valid || pt->measured_period <= 0) {
        return 0.0f;
    }
    
    float elapsed = timestamp - pt->last_zero_crossing;
    float phase = fmodf(elapsed, pt->measured_period) / pt->measured_period;
    
    // 確保相位在 [0, 1) 範圍內
    if (phase < 0) phase += 1.0f;
    
    return phase;
}

/**
 * @brief 相位鎖定重建缺失樣本
 * @param sr 信號重建器指針
 * @param timestamp 缺失樣本的時間戳
 * @return 重建的電流值
 */
float phase_locked_reconstruct(signal_reconstructor_t *sr, float timestamp) {
    period_tracker_t *pt = &sr->period_tracker;
    sample_buffer_t *buf = &sr->buffer;
    
    if (!pt->period_valid || pt->samples_per_period <= 0) {
        // 週期未知，使用線性插值
        if (buf->count >= 2) {
            sample_t *s1 = sample_buffer_get(buf, -1);
            sample_t *s2 = sample_buffer_get(buf, -2);
            if (s1 && s2) {
                float slope = (s1->value - s2->value) / (s1->timestamp - s2->timestamp);
                return s1->value + slope * (timestamp - s1->timestamp);
            }
        }
        return sr->current_filtered;  // 返回上一個有效值
    }
    
    // 計算應該參考的前一週期索引
    int prev_cycle_offset = -pt->samples_per_period;
    sample_t *prev_cycle_sample = sample_buffer_get(buf, prev_cycle_offset);
    
    if (prev_cycle_sample && prev_cycle_sample->quality != QUALITY_INVALID) {
        // 找到前一週期的對應點
        float value1 = prev_cycle_sample->value;
        
        // 嘗試獲取前兩個週期的值進行加權平均（更穩健）
        sample_t *prev2_cycle_sample = sample_buffer_get(buf, prev_cycle_offset * 2);
        
        if (prev2_cycle_sample && prev2_cycle_sample->quality != QUALITY_INVALID) {
            // 加權平均：最近週期權重較高
            return 0.7f * value1 + 0.3f * prev2_cycle_sample->value;
        }
        
        return value1;
    }
    
    // 無法使用相位鎖定，回退到最後已知值
    return sr->current_filtered;
}

/* ============================================================
 * Cubic Spline 插值
 * 
 * 自然三次樣條插值，提供 C² 連續性（二階導數連續）
 * 適合還原平滑的週期性波形
 * ============================================================ */

/**
 * @brief 添加數據點到 Spline
 * @param spline Spline 指針
 * @param x x 座標（時間）
 * @param y y 座標（電流值）
 */
void cubic_spline_add_point(cubic_spline_t *spline, float x, float y) {
    if (spline->n >= SPLINE_POINTS) {
        // 緩衝區已滿，移除最舊的點
        for (int i = 0; i < SPLINE_POINTS - 1; i++) {
            spline->x[i] = spline->x[i + 1];
            spline->y[i] = spline->y[i + 1];
        }
        spline->n = SPLINE_POINTS - 1;
    }
    
    spline->x[spline->n] = x;
    spline->y[spline->n] = y;
    spline->n++;
    spline->coefficients_valid = false;  // 需要重新計算係數
}

/**
 * @brief 計算 Cubic Spline 係數
 * 
 * 使用 Thomas 算法（三對角矩陣求解）
 * 複雜度 O(n)，適合嵌入式系統
 */
void cubic_spline_compute_coefficients(cubic_spline_t *spline) {
    int n = spline->n;
    
    if (n < 3) {
        spline->coefficients_valid = false;
        return;
    }
    
    // 計算區間長度
    float h[SPLINE_POINTS];
    for (int i = 0; i < n - 1; i++) {
        h[i] = spline->x[i + 1] - spline->x[i];
        if (h[i] <= 0) {
            // 無效的 x 序列
            spline->coefficients_valid = false;
            return;
        }
    }
    
    // 設置三對角矩陣的係數
    float alpha[SPLINE_POINTS];
    for (int i = 1; i < n - 1; i++) {
        alpha[i] = (3.0f / h[i]) * (spline->y[i + 1] - spline->y[i]) -
                   (3.0f / h[i - 1]) * (spline->y[i] - spline->y[i - 1]);
    }
    
    // Thomas 算法求解
    float l[SPLINE_POINTS], mu[SPLINE_POINTS], z[SPLINE_POINTS];
    
    l[0] = 1.0f;
    mu[0] = 0.0f;
    z[0] = 0.0f;
    
    for (int i = 1; i < n - 1; i++) {
        l[i] = 2.0f * (spline->x[i + 1] - spline->x[i - 1]) - h[i - 1] * mu[i - 1];
        if (fabsf(l[i]) < 1e-10f) {
            l[i] = 1e-10f;  // 避免除以零
        }
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    
    l[n - 1] = 1.0f;
    z[n - 1] = 0.0f;
    spline->c[n - 1] = 0.0f;
    
    // 回代求解
    for (int j = n - 2; j >= 0; j--) {
        spline->c[j] = z[j] - mu[j] * spline->c[j + 1];
        spline->b[j] = (spline->y[j + 1] - spline->y[j]) / h[j] -
                       h[j] * (spline->c[j + 1] + 2.0f * spline->c[j]) / 3.0f;
        spline->d[j] = (spline->c[j + 1] - spline->c[j]) / (3.0f * h[j]);
        spline->a[j] = spline->y[j];
    }
    
    spline->coefficients_valid = true;
}

/**
 * @brief 評估 Cubic Spline 在指定 x 位置的值
 * @param spline Spline 指針
 * @param x 要評估的 x 座標
 * @return 插值結果
 */
float cubic_spline_evaluate(cubic_spline_t *spline, float x) {
    if (!spline->coefficients_valid || spline->n < 3) {
        return 0.0f;
    }
    
    // 找到包含 x 的區間
    int i = 0;
    for (i = 0; i < spline->n - 1; i++) {
        if (x < spline->x[i + 1]) break;
    }
    
    // 確保索引有效
    if (i >= spline->n - 1) i = spline->n - 2;
    if (i < 0) i = 0;
    
    // 計算相對位置
    float dx = x - spline->x[i];
    
    // 評估三次多項式：a + b*dx + c*dx² + d*dx³
    return spline->a[i] + 
           spline->b[i] * dx + 
           spline->c[i] * dx * dx + 
           spline->d[i] * dx * dx * dx;
}

/* ============================================================
 * 樣本緩衝區操作
 * ============================================================ */

/**
 * @brief 添加樣本到緩衝區
 */
void sample_buffer_add(sample_buffer_t *buf, float value, float timestamp, sample_quality_t quality) {
    buf->samples[buf->write_idx].value = value;
    buf->samples[buf->write_idx].timestamp = timestamp;
    buf->samples[buf->write_idx].quality = quality;
    
    buf->write_idx = (buf->write_idx + 1) % SAMPLE_BUFFER_SIZE;
    
    if (buf->count < SAMPLE_BUFFER_SIZE) {
        buf->count++;
    }
    
    buf->last_timestamp = timestamp;
}

/**
 * @brief 獲取緩衝區中的樣本（相對於最新樣本的偏移）
 * @param buf 緩衝區指針
 * @param offset 偏移量（負數表示過去的樣本，-1 = 最新樣本）
 * @return 樣本指針，或 NULL（如果無效）
 */
sample_t* sample_buffer_get(sample_buffer_t *buf, int offset) {
    if (buf->count == 0) return NULL;
    
    // offset = -1 表示最新樣本
    int idx = buf->write_idx - 1 + offset;
    
    // 處理環繞
    while (idx < 0) idx += SAMPLE_BUFFER_SIZE;
    idx = idx % SAMPLE_BUFFER_SIZE;
    
    // 檢查是否在有效範圍內
    if (-offset > buf->count) return NULL;
    
    return &buf->samples[idx];
}

/**
 * @brief 檢測樣本間隙
 * @param buf 緩衝區指針
 * @param gap_timestamps 輸出：間隙的時間戳陣列
 * @param max_gaps 最大間隙數量
 * @return 檢測到的間隙數量
 */
int sample_buffer_detect_gaps(sample_buffer_t *buf, float *gap_timestamps, int max_gaps) {
    int gap_count = 0;
    
    if (buf->count < 2) return 0;
    
    float threshold = buf->expected_interval * 1.5f;  // 1.5倍間隔視為間隙
    
    for (int i = 1; i < buf->count && gap_count < max_gaps; i++) {
        sample_t *curr = sample_buffer_get(buf, -i);
        sample_t *prev = sample_buffer_get(buf, -i - 1);
        
        if (curr && prev) {
            float interval = curr->timestamp - prev->timestamp;
            if (interval > threshold) {
                // 計算缺失的樣本數
                int missing = (int)(interval / buf->expected_interval) - 1;
                
                // 記錄每個缺失樣本的預期時間戳
                for (int j = 1; j <= missing && gap_count < max_gaps; j++) {
                    gap_timestamps[gap_count++] = prev->timestamp + j * buf->expected_interval;
                }
            }
        }
    }
    
    return gap_count;
}

/* ============================================================
 * 主處理流程
 * ============================================================ */

/**
 * @brief 處理單一原始樣本
 * 
 * 完整處理流程：
 * 1. Hampel 濾波 - 檢測並替換異常值
 * 2. 間隙檢測 - 檢查是否有缺失樣本
 * 3. 相位鎖定重建 - 填補缺失樣本
 * 4. 週期追蹤 - 更新週期估計
 * 5. Cubic Spline - 更新插值係數（用於峰值分析）
 * 
 * @param sr 信號重建器指針
 * @param raw_value 原始電流值 (mA)
 * @param timestamp 時間戳 (秒)
 * @return 處理後的電流值
 */
float signal_reconstructor_process(signal_reconstructor_t *sr, float raw_value, float timestamp) {
    sample_quality_t quality = QUALITY_MEASURED;
    float processed_value = raw_value;
    
    sr->total_samples++;
    
    /* ===== 步驟 1: Hampel 濾波 ===== */
    bool is_outlier = false;
    processed_value = hampel_filter_process(&sr->hampel, raw_value, &is_outlier);
    
    if (is_outlier) {
        quality = QUALITY_FILTERED;
        sr->filtered_samples++;
        LOG_DBG("Outlier detected at t=%.4f: %.2f -> %.2f", 
                (double)timestamp, (double)raw_value, (double)processed_value);
    }
    
    /* ===== 步驟 2: 間隙檢測與填補 ===== */
    if (sr->buffer.last_timestamp > 0) {
        float interval = timestamp - sr->buffer.last_timestamp;
        float expected = sr->buffer.expected_interval;
        
        // 檢測是否有缺失樣本（間隔大於預期的 1.5 倍）
        if (interval > expected * 1.5f) {
            int missing_count = (int)(interval / expected) - 1;
            
            LOG_DBG("Gap detected: %d missing samples", missing_count);
            
            // 使用相位鎖定重建填補缺失樣本
            for (int i = 1; i <= missing_count && i <= 5; i++) {  // 最多填補 5 個
                float gap_timestamp = sr->buffer.last_timestamp + i * expected;
                float reconstructed = phase_locked_reconstruct(sr, gap_timestamp);
                
                sample_buffer_add(&sr->buffer, reconstructed, gap_timestamp, QUALITY_PHASE_LOCKED);
                sr->phase_locked_samples++;
                sr->interpolated_samples++;
            }
        }
    }
    
    /* ===== 步驟 3: 更新週期追蹤 ===== */
    period_tracker_update(&sr->period_tracker, processed_value, timestamp);
    
    /* ===== 步驟 4: 添加當前樣本到緩衝區 ===== */
    sample_buffer_add(&sr->buffer, processed_value, timestamp, quality);
    
    /* ===== 步驟 5: 更新 Cubic Spline ===== */
    cubic_spline_add_point(&sr->spline, timestamp, processed_value);
    
    // 定期重新計算樣條係數（每 10 個樣本）
    if (sr->total_samples % 10 == 0 && sr->spline.n >= 5) {
        cubic_spline_compute_coefficients(&sr->spline);
    }
    
    /* ===== 更新輸出 ===== */
    sr->current_filtered = processed_value;
    
    return processed_value;
}

/**
 * @brief 更新統計信息（峰值、RMS、平均值）
 * 
 * 建議每個週期調用一次
 */
void signal_reconstructor_update_statistics(signal_reconstructor_t *sr) {
    if (sr->buffer.count < 10) return;
    
    float sum = 0.0f;
    float sum_sq = 0.0f;
    float max_val = -1e10f;
    int valid_count = 0;
    
    // 遍歷緩衝區計算統計值
    for (int i = 0; i < sr->buffer.count; i++) {
        sample_t *s = sample_buffer_get(&sr->buffer, -i - 1);
        if (s && s->quality != QUALITY_INVALID) {
            // 根據質量加權
            float weight = 1.0f;
            if (s->quality == QUALITY_PHASE_LOCKED || s->quality == QUALITY_INTERPOLATED) {
                weight = 0.9f;  // 重建樣本權重稍低
            }
            
            sum += s->value * weight;
            sum_sq += s->value * s->value * weight;
            valid_count++;
            
            if (s->value > max_val) {
                max_val = s->value;
            }
        }
    }
    
    if (valid_count > 0) {
        sr->average_current = sum / valid_count;
        sr->rms_current = sqrtf(sum_sq / valid_count);
        sr->peak_current = max_val;
    }
}

/**
 * @brief 重置統計信息
 */
void signal_reconstructor_reset_statistics(signal_reconstructor_t *sr) {
    sr->total_samples = 0;
    sr->filtered_samples = 0;
    sr->interpolated_samples = 0;
    sr->phase_locked_samples = 0;
    sr->peak_current = 0.0f;
    sr->rms_current = 0.0f;
    sr->average_current = 0.0f;
}

/**
 * @brief 獲取統計信息
 */
void signal_reconstructor_get_statistics(signal_reconstructor_t *sr, 
                                          float *peak, float *rms, float *avg,
                                          int *total, int *filtered, int *interpolated) {
    if (peak) *peak = sr->peak_current;
    if (rms) *rms = sr->rms_current;
    if (avg) *avg = sr->average_current;
    if (total) *total = sr->total_samples;
    if (filtered) *filtered = sr->filtered_samples;
    if (interpolated) *interpolated = sr->interpolated_samples;
}