#ifndef CONFIG_H
#define CONFIG_H

// --- Cấu hình I2C ---
#define I2C_ADDR 0x30 
#define SDA_PIN 14
#define SCL_PIN 21

#define MPU_SDA_PIN 1
#define MPU_SCL_PIN 2
// --- Chân điều khiển động cơ ---
// #define M3_INA1 5
// #define M3_INA2 4
// #define M4_INA1 7
// #define M4_INA2 6
#define M3_INA1 16
#define M3_INA2 15
#define M4_INA1 18
#define M4_INA2 17

// Tốc độ mặc định
#define SPEED_DEFAULT 150


#endif