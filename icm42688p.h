


#ifndef INC_ICM42688P_H_
#define INC_ICM42688P_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define ICM42688_I2C_ADDR       		(0x68 << 1)

/* ---------- Registers ---------- */
#define ICM42688_REG_DEVICE_CONFIG      0x11
#define ICM42688_REG_PWR_MGMT0          0x4E
#define ICM42688_REG_GYRO_CONFIG0       0x4F
#define ICM42688_REG_ACCEL_CONFIG0      0x50
#define ICM42688_REG_GYRO_CONFIG1       0x51
#define ICM42688_REG_GYRO_ACCEL_CONFIG0 0x52
#define ICM42688_REG_ACCEL_CONFIG1      0x53
#define ICM42688_REG_WHO_AM_I           0x75
#define ICM42688_REG_FIFO_CONFIG1       0x16
#define ICM42688_REG_FIFO_CONFIG        0x5F
#define ICM42688_REG_FIFO_DATA          0x30
#define ICM42688_WHO_AM_I_VAL           0x47
#define ICM42688_PWR_TEMP_ON_LN_LN      0x1F
#define ICM42688_REG_BANK_SEL           0x76

/* GYRO_CONFIG0 FS bits */
#define ICM42688_GYRO_FS_2000DPS  		(0x00u)
#define ICM42688_GYRO_FS_1000DPS  		(0x01u << 5)
#define ICM42688_GYRO_FS_500DPS   		(0x02u << 5)
#define ICM42688_GYRO_FS_250DPS   		(0x03u << 5)
#define ICM42688_GYRO_FS_125DPS   		(0x04u << 5)
#define ICM42688_GYRO_FS_62DPS5   		(0x05u << 5)
#define ICM42688_GYRO_FS_31DPS25  		(0x06u << 5)
#define ICM42688_GYRO_FS_15DPS625 		(0x07u << 5)

/* ACCEL_CONFIG0 FS bits */
#define ICM42688_ACCEL_FS_16G     (0x00u)
#define ICM42688_ACCEL_FS_8G      (0x01u << 5)
#define ICM42688_ACCEL_FS_4G      (0x02u << 5)
#define ICM42688_ACCEL_FS_2G      (0x03u << 5)

/* ODR choices we map to */
#define IMU_50HZ         (0x09u)
#define IMU_100HZ        (0x08u)
#define IMU_200HZ        (0x07u)
#define IMU_500HZ        (0x05u)




typedef struct {
    I2C_HandleTypeDef *hi2c;

} icm42688_dev_t;

typedef struct {
    uint16_t timestamp;
    uint8_t header;
    float temp;
    float data[6];
    uint16_t real_timestamp;
} imu_data_t;

// Usage function
HAL_StatusTypeDef imu_write_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t val);
HAL_StatusTypeDef imu_read_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t *pdata, uint16_t len);

HAL_StatusTypeDef icm42688_init(icm42688_dev_t *dev, int imu_rate);
HAL_StatusTypeDef icm42688_startMeasure(icm42688_dev_t *dev);
HAL_StatusTypeDef icm42688_readFifo(icm42688_dev_t *dev, uint8_t *Fifo_data, imu_data_t *outputFifo);

#endif /* INC_ICM42688P_H_ */
