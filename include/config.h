#ifndef CONFIG_H
#define CONFIG_H

// --- Cấu hình I2C ---
#define I2C_ADDR 0x30 
#define SDA_PIN 40
#define SCL_PIN 39

#define MPU_SDA_PIN 1
#define MPU_SCL_PIN 2
// --- Chân điều khiển động cơ ---
#define M4_INA1 4
#define M4_INA2 5
#define M3_INA1 6
#define M3_INA2 7
// #define M3_INA1 16
// #define M3_INA2 15
// #define M4_INA1 18
// #define M4_INA2 17
#define BUTTON_PIN 47

#define TRIG_PIN 14
#define ECHO_PIN 21
// Tốc độ mặc định
#define SPEED_DEFAULT 150
// Hệ số tốc độ toàn cục: 1.0 (100% gốc), 0.9 (giảm 10%), 0.5 (giảm một nửa)
#define SPEED_SCALE 1

#endif