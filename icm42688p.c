
// Includes
#include "icm42688p.h"


// Write register IMU functions
HAL_StatusTypeDef imu_write_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t data)
{
	 return HAL_I2C_Mem_Write(dev->hi2c, ICM42688_I2C_ADDR, reg, 1, &data, 1, 100);
}
// Read register IMU functions
HAL_StatusTypeDef imu_read_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t *pdata, uint16_t len)
{
	 return HAL_I2C_Mem_Read(dev->hi2c,  ICM42688_I2C_ADDR, reg, 1, pdata, len, 100);
}

// Init IMU Functions
HAL_StatusTypeDef icm42688_init(icm42688_dev_t *dev, int imu_rate)
{
	if (!dev || !dev->hi2c) return HAL_ERROR;
	HAL_StatusTypeDef status;
	uint8_t data = 0;

	// Check ID IMU
	status = imu_read_reg(dev, ICM42688_REG_WHO_AM_I, &data, 1);
	if(status != HAL_OK) return status;
	if (data == ICM42688_WHO_AM_I_VAL){

		// Reset IMU sensor
		data = 0x00; // sel bank 0
		status = imu_write_reg(dev, ICM42688_REG_BANK_SEL, data);
		data = 0x01;
		status = imu_write_reg(dev, ICM42688_REG_DEVICE_CONFIG, data);
		HAL_Delay(100); // Add delay 100ms for make sure IMU ready for reset

		// Configure accel and gyro rate
		switch (imu_rate){
			case IMU_500HZ:
				data = 0xEF;
				imu_write_reg(dev, ICM42688_REG_GYRO_CONFIG0, data);
				data = 0x6F;
				imu_write_reg(dev, ICM42688_REG_ACCEL_CONFIG0, data);
				break;
			case IMU_200HZ:
				data = 0xE7;
				imu_write_reg(dev, ICM42688_REG_GYRO_CONFIG0, data);
				data = 0x67;
				imu_write_reg(dev, ICM42688_REG_ACCEL_CONFIG0, data);
				break;
			case IMU_100HZ:
				data = 0xE8;
				imu_write_reg(dev, ICM42688_REG_GYRO_CONFIG0, data);
				data = 0x68;
				imu_write_reg(dev, ICM42688_REG_ACCEL_CONFIG0, data);
				break;
			case IMU_50HZ:
				data = 0xE9;
				imu_write_reg(dev, ICM42688_REG_GYRO_CONFIG0, data);
				data = 0x69;
				imu_write_reg(dev, ICM42688_REG_ACCEL_CONFIG0, data);
				break;
			default:
				data = 0xE9;
				imu_write_reg(dev, ICM42688_REG_GYRO_CONFIG0, data);
				data = 0x69;
				imu_write_reg(dev, ICM42688_REG_ACCEL_CONFIG0, data);
		}
		// Configure FIFO
		data = 0x40;
		imu_write_reg(dev, ICM42688_REG_FIFO_CONFIG1, data);
		data = 0x03;
		imu_write_reg(dev, ICM42688_REG_FIFO_CONFIG, data);
		HAL_Delay(10);

		// Enable Low Noise mode for acc and gyro
		data = 0x1F;
		imu_write_reg(dev, ICM42688_REG_PWR_MGMT0, data);
		HAL_Delay(10);

		return HAL_OK;
	} else
		return HAL_ERROR;
}

// Read FIFO Data
HAL_StatusTypeDef icm42688_readFifo(icm42688_dev_t *dev, uint8_t *fifo_data, imu_data_t *outputFifo)
{
	if (!dev || !dev->hi2c) return HAL_ERROR;
	HAL_StatusTypeDef status;
	status = imu_read_reg(dev, ICM42688_REG_FIFO_DATA, fifo_data, 16);
	if(status != HAL_OK) return status;

    // Scaling factors
    float acc_scale = (2 * 9.81f) / 32768.0f; // ±2g range
    float gyro_scale = (15.625f / 32768.0f); // ±15.625 deg/s range

	// Combine fifo bits
	uint8_t header = fifo_data[0];

	int16_t acc_x = (int16_t)((fifo_data[1]  << 8) | fifo_data[2]);
	int16_t acc_y = (int16_t)((fifo_data[3]  << 8) | fifo_data[4]);
	int16_t acc_z = (int16_t)((fifo_data[5]  << 8) | fifo_data[6]);

	int16_t gyro_x = (int16_t)((fifo_data[7]  << 8) | fifo_data[8]);
	int16_t gyro_y = (int16_t)((fifo_data[9]  << 8) | fifo_data[10]);
	int16_t gyro_z = (int16_t)((fifo_data[11]  << 8) | fifo_data[12]);

	uint8_t temp = (int8_t)fifo_data[13];

	int16_t timestamp = (uint16_t)((fifo_data[14] << 8) | fifo_data[15]);

	// Convert raw data to physical units
	float acc_raw[3] = {0};
	float gyro_raw[3] = {0};

    acc_raw[0] = acc_x * acc_scale;
    acc_raw[1] = acc_y * acc_scale;
    acc_raw[2] = acc_z * acc_scale;
    gyro_raw[0] = gyro_x * gyro_scale;
    gyro_raw[1] = gyro_y * gyro_scale;
    gyro_raw[2] = gyro_z * gyro_scale;

    //
    outputFifo->header = header;
    outputFifo->temp = temp;
    outputFifo->data[0] = acc_raw[0];
    outputFifo->data[1] = acc_raw[1];
    outputFifo->data[2] = acc_raw[2];

    outputFifo->data[3] = gyro_raw[0];
    outputFifo->data[4] = gyro_raw[1];
    outputFifo->data[5] = gyro_raw[2];

    outputFifo->real_timestamp = timestamp;

	return HAL_OK;
}


// Start Measurement Functions
HAL_StatusTypeDef icm42688_startMeasure(icm42688_dev_t *dev)
{
	if (!dev || !dev->hi2c) return HAL_ERROR;
	uint8_t data = 0;
	// Enable Measurement
	data = 0x0F;
	return imu_write_reg(dev, ICM42688_REG_PWR_MGMT0, data);
}
//
