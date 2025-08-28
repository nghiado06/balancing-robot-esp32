#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

#include <Arduino.h>

// Tần số nốt (Hz)
#define NOTE_C5 523
#define NOTE_E5 659
#define NOTE_G5 784
#define NOTE_C6 1047

// Hình dáng mắt
const uint8_t EYE_SHAPE[8] = {
    0b00111100,
    0b01111110,
    0b11111111,
    0b11111111,
    0b11111111,
    0b11111111,
    0b01111110,
    0b00111100};

// ====== THÔNG SỐ CƠ KHÍ & RÀNG BUỘC ======
#define R_MM 35.0f                                         // Bán Kính Bánh xe
#define L_MM 160.0f                                        // Khoảng cách 2 Bánh xe
#define STEPS_PER_REV 200                                  // Bước trên mỗi vòng quay
#define MICROSTEP 16                                       // Số microstep
#define CIRC_MM (2.0f * PI * R_MM)                         // Chu vi bánh xe
#define STEPS_PER_MM (STEPS_PER_REV * MICROSTEP) / CIRC_MM // Bước trên mỗi mm

#define VMAX_MM_S 400.0f // Giới hạn tốc độ tuyến tính
#define F_MAX_HZ 4500.0f // Tần số step tối đa
#define DF_MAX_HZ 600.0f // Độ dốc step mỗi tick (ramp)

// ====== PID & MAPPING ======
#define KP 12.0f
#define KI 1.0f
#define KD 0.25f
#define K_V 0.03f             // đổi v_ref -> theta_ref (deg / (mm/s))
#define THETA_REF_LIMIT 12.0f // giới hạn tham chiếu nghiêng

// ====== COMPLEMENTARY FILTER ======
#define ALPHA 0.98f // 0.96–0.99

// ====== CALIB + AUTO-TRIM ======
#define K_TRIM 0.02f
#define TRIM_MAX 3.0f

// ======== ULTRA SONIC =======
#define THRESHOLD_DISTANCE 20.0f
#define TIMEOUT_DURATION 25000UL

#define V_DEAD 8.0f
#define YAW_DEAD 2.0f

// ======== HAND MOTION ========
#define K_VCMD_MM_S_PER_DEG 25.0f
#define K_YAWCMD_DPS_PER_DEG 20.0f
#define HM_ALPHA 0.90f
#define HM_LINK_TIMEOUT_MS 400

#endif // PARAM_CONFIG_H