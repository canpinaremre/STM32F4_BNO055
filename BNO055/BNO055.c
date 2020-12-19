#include <drivers/BNO055/BNO055.h>

void reset_Buffer(void){
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;
	buffer[3] = 0;
	buffer[4] = 0;
	buffer[5] = 0;
	buffer[6] = 0;
	buffer[7] = 0;
	buffer[8] = 0;
}

uint8_t BNO055_Chip_ID_Check(I2C_HandleTypeDef *huart) {
	reset_Buffer();

	if (read8(huart,BNO055_READ_ADDR,BNO055_CHIP_ID_ADDR) == BNO055_ID) {
		return 1;
	} else {
		return 0;
	}

}

uint8_t BNO055_Init(I2C_HandleTypeDef *huart, bno055_opmode_t mode,uint8_t delay_time) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}

	BNO055_SetMode(huart, mode);
	HAL_Delay(delay_time);
	return 1;
}

uint8_t BNO055_Get_Page(I2C_HandleTypeDef *huart) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 2;
	}

	return read8(huart,BNO055_READ_ADDR,BNO055_PAGE_ADDR);
}

uint8_t BNO055_Set_Page(I2C_HandleTypeDef *huart,uint8_t page) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}
	write8(huart,BNO055_READ_ADDR,BNO055_PAGE_ADDR,page);
	return 1;
}

//Reading Raw Data from BNO055
uint8_t BNO055_Read_Eul(I2C_HandleTypeDef *huart,float *eulerXYZ) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}

	buffer[1] = read8(huart,BNO055_READ_ADDR,BNO055_EUL_DATA_x_LSB_ADDR);
	buffer[2] = read8(huart,BNO055_READ_ADDR,BNO055_EUL_DATA_x_MSB_ADDR);
	buffer[3] = read8(huart,BNO055_READ_ADDR,BNO055_EUL_DATA_y_LSB_ADDR);
	buffer[4] = read8(huart,BNO055_READ_ADDR,BNO055_EUL_DATA_y_MSB_ADDR);
	buffer[5] = read8(huart,BNO055_READ_ADDR,BNO055_EUL_DATA_z_LSB_ADDR);
	buffer[6] = read8(huart,BNO055_READ_ADDR,BNO055_EUL_DATA_z_MSB_ADDR);

	data_read[0] = (buffer[1]) | ((buffer[2]) << 8);
	data_read[1] = (buffer[3]) | ((buffer[4]) << 8);
	data_read[2] = (buffer[5]) | ((buffer[6]) << 8);
	eulerXYZ[0] = ((float) data_read[0]) / 16.0;
	eulerXYZ[1] = (float) data_read[1] / 16.0;
	eulerXYZ[2] = (float) data_read[2] / 16.0;

	return 1;
}

uint8_t BNO055_Read_Qua(I2C_HandleTypeDef *huart,float *quaternionWXYZ) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}
	//TODO: Check NDOF OR Other modes if they supports Quaternion

	buffer[1] = read8(huart,BNO055_READ_ADDR,BNO055_QUATERNION_DATA_W_LSB_ADDR);
	buffer[2] = read8(huart,BNO055_READ_ADDR,BNO055_QUATERNION_DATA_W_MSB_ADDR);
	buffer[3] = read8(huart,BNO055_READ_ADDR,BNO055_QUATERNION_DATA_X_LSB_ADDR);
	buffer[4] = read8(huart,BNO055_READ_ADDR,BNO055_QUATERNION_DATA_X_MSB_ADDR);
	buffer[5] = read8(huart,BNO055_READ_ADDR,BNO055_QUATERNION_DATA_Y_LSB_ADDR);
	buffer[6] = read8(huart,BNO055_READ_ADDR,BNO055_QUATERNION_DATA_Y_MSB_ADDR);
	buffer[7] = read8(huart,BNO055_READ_ADDR,BNO055_QUATERNION_DATA_Z_LSB_ADDR);
	buffer[8] = read8(huart,BNO055_READ_ADDR,BNO055_QUATERNION_DATA_Z_MSB_ADDR);

	data_read[0] = (buffer[1]) | ((buffer[2]) << 8);
	data_read[1] = (buffer[3]) | ((buffer[4]) << 8);
	data_read[2] = (buffer[5]) | ((buffer[6]) << 8);
	data_read[3] = (buffer[7]) | ((buffer[8]) << 8);

	quaternionWXYZ[0] = (float) data_read[0] / 16383.0;
	quaternionWXYZ[1] = (float) data_read[1] / 16383.0;
	quaternionWXYZ[2] = (float) data_read[2] / 16383.0;
	quaternionWXYZ[3] = (float) data_read[3] / 16383.0;

	return 1;
}

uint8_t BNO055_Read_Acc(I2C_HandleTypeDef *huart,float *accelXYZ) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}
	//TODO: Sometimes goes 0 0 0 and errors when reading chip ID


	buffer[1] = read8(huart,BNO055_READ_ADDR,BNO055_ACCEL_DATA_X_LSB_ADDR);
	buffer[2] = read8(huart,BNO055_READ_ADDR,BNO055_ACCEL_DATA_X_MSB_ADDR);
	buffer[3] = read8(huart,BNO055_READ_ADDR,BNO055_ACCEL_DATA_Y_LSB_ADDR);
	buffer[4] = read8(huart,BNO055_READ_ADDR,BNO055_ACCEL_DATA_Y_MSB_ADDR);
	buffer[5] = read8(huart,BNO055_READ_ADDR,BNO055_ACCEL_DATA_Z_LSB_ADDR);
	buffer[6] = read8(huart,BNO055_READ_ADDR,BNO055_ACCEL_DATA_Z_MSB_ADDR);

	data_read[0] = (buffer[1]) | ((buffer[2]) << 8);
	data_read[1] = (buffer[3]) | ((buffer[4]) << 8);
	data_read[2] = (buffer[5]) | ((buffer[6]) << 8);
	accelXYZ[0] = ((float) data_read[0]) / 100.0;
	accelXYZ[1] = (float) data_read[1] / 100.0;
	accelXYZ[2] = (float) data_read[2] / 100.0;



	return 1;
}
uint8_t BNO055_Read_Mag(I2C_HandleTypeDef *huart,float *magXYZ) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}


	buffer[1] = read8(huart,BNO055_READ_ADDR,BNO055_MAG_DATA_X_LSB_ADDR);
	buffer[2] = read8(huart,BNO055_READ_ADDR,BNO055_MAG_DATA_X_MSB_ADDR);
	buffer[3] = read8(huart,BNO055_READ_ADDR,BNO055_MAG_DATA_Y_LSB_ADDR);
	buffer[4] = read8(huart,BNO055_READ_ADDR,BNO055_MAG_DATA_Y_MSB_ADDR);
	buffer[5] = read8(huart,BNO055_READ_ADDR,BNO055_MAG_DATA_Z_LSB_ADDR);
	buffer[6] = read8(huart,BNO055_READ_ADDR,BNO055_MAG_DATA_Z_MSB_ADDR);

	data_read[0] = (buffer[1]) | ((buffer[2]) << 8);
	data_read[1] = (buffer[3]) | ((buffer[4]) << 8);
	data_read[2] = (buffer[5]) | ((buffer[6]) << 8);
	magXYZ[0] = ((float) data_read[0]) / 100.0;
	magXYZ[1] = (float) data_read[1] / 100.0;
	magXYZ[2] = (float) data_read[2] / 100.0;

	return 1;
}
uint8_t BNO055_Read_Gyr(I2C_HandleTypeDef *huart,float *gyrXYZ) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}

	buffer[1] = read8(huart,BNO055_READ_ADDR,BNO055_GYRO_DATA_X_LSB_ADDR);
	buffer[2] = read8(huart,BNO055_READ_ADDR,BNO055_GYRO_DATA_X_MSB_ADDR);
	buffer[3] = read8(huart,BNO055_READ_ADDR,BNO055_GYRO_DATA_Y_LSB_ADDR);
	buffer[4] = read8(huart,BNO055_READ_ADDR,BNO055_GYRO_DATA_Y_MSB_ADDR);
	buffer[5] = read8(huart,BNO055_READ_ADDR,BNO055_GYRO_DATA_Z_LSB_ADDR);
	buffer[6] = read8(huart,BNO055_READ_ADDR,BNO055_GYRO_DATA_Z_MSB_ADDR);

	data_read[0] = (buffer[1]) | ((buffer[2]) << 8);
	data_read[1] = (buffer[3]) | ((buffer[4]) << 8);
	data_read[2] = (buffer[5]) | ((buffer[6]) << 8);
	gyrXYZ[0] = ((float) data_read[0]) / 100.0;
	gyrXYZ[1] = (float) data_read[1] / 100.0;
	gyrXYZ[2] = (float) data_read[2] / 100.0;

	return 1;
}
uint8_t BNO055_Read_Grv(I2C_HandleTypeDef *huart,float *grvXYZ) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}

	buffer[1] = read8(huart,BNO055_READ_ADDR,BNO055_GRAVITY_DATA_X_LSB_ADDR);
	buffer[2] = read8(huart,BNO055_READ_ADDR,BNO055_GRAVITY_DATA_X_MSB_ADDR);
	buffer[3] = read8(huart,BNO055_READ_ADDR,BNO055_GRAVITY_DATA_Y_LSB_ADDR);
	buffer[4] = read8(huart,BNO055_READ_ADDR,BNO055_GRAVITY_DATA_Y_MSB_ADDR);
	buffer[5] = read8(huart,BNO055_READ_ADDR,BNO055_GRAVITY_DATA_Z_LSB_ADDR);
	buffer[6] = read8(huart,BNO055_READ_ADDR,BNO055_GRAVITY_DATA_Z_MSB_ADDR);

	data_read[0] = (buffer[1]) | ((buffer[2]) << 8);
	data_read[1] = (buffer[3]) | ((buffer[4]) << 8);
	data_read[2] = (buffer[5]) | ((buffer[6]) << 8);
	grvXYZ[0] = ((float) data_read[0]) / 100.0;
	grvXYZ[1] = (float) data_read[1] / 100.0;
	grvXYZ[2] = (float) data_read[2] / 100.0;

	return 1;
}
uint8_t BNO055_Read_Lia(I2C_HandleTypeDef *huart,float *liaXYZ) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}

	buffer[1] = read8(huart,BNO055_READ_ADDR,BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR);
	buffer[2] = read8(huart,BNO055_READ_ADDR,BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR);
	buffer[3] = read8(huart,BNO055_READ_ADDR,BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR);
	buffer[4] = read8(huart,BNO055_READ_ADDR,BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR);
	buffer[5] = read8(huart,BNO055_READ_ADDR,BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR);
	buffer[6] = read8(huart,BNO055_READ_ADDR,BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR);

	data_read[0] = (buffer[1]) | ((buffer[2]) << 8);
	data_read[1] = (buffer[3]) | ((buffer[4]) << 8);
	data_read[2] = (buffer[5]) | ((buffer[6]) << 8);
	liaXYZ[0] = ((float) data_read[0]) / 100.0;
	liaXYZ[1] = (float) data_read[1] / 100.0;
	liaXYZ[2] = (float) data_read[2] / 100.0;

	return 1;
}

//Sets parameters for measurements
uint8_t BNO055_Set_Eul(I2C_HandleTypeDef *huart) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}//TODO
	return 1;
}

uint8_t BNO055_Set_Qua(I2C_HandleTypeDef *huart) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}//TODO
	return 1;
}

uint8_t BNO055_Set_Acc(I2C_HandleTypeDef *huart) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}//TODO
	return 1;
}

uint8_t BNO055_Set_Mag(I2C_HandleTypeDef *huart) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}//TODO
	return 1;
}

uint8_t BNO055_Set_Gyr(I2C_HandleTypeDef *huart) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}//TODO
	return 1;
}

uint8_t BNO055_Set_Grv(I2C_HandleTypeDef *huart) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}//TODO
	return 1;
}

uint8_t BNO055_Set_Lia(I2C_HandleTypeDef *huart) {
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}//TODO
	return 1;
}

uint8_t BNO055_SetMode(I2C_HandleTypeDef *huart,bno055_opmode_t mode){
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}

	write8(huart, BNO055_WRITE_ADDR, BNO055_OPR_MODE_ADDR, mode);
	HAL_Delay(20);

	return 1;
}

bno055_opmode_t BNO055_GetMode(I2C_HandleTypeDef *huart){
	return read8(huart,BNO055_READ_ADDR,BNO055_OPR_MODE_ADDR);
}

uint8_t BNO055_SetAxisRemap(I2C_HandleTypeDef *huart,bno055_axis_remap_config_t remapcode){
	if (!BNO055_Chip_ID_Check(huart)) {
		return 0;
	}

	bno055_opmode_t modeback = BNO055_GetMode(huart);
	BNO055_SetMode(huart,OPERATION_MODE_CONFIG);
	HAL_Delay(25);
	write8(huart,BNO055_WRITE_ADDR,BNO055_AXIS_MAP_CONFIG_ADDR,remapcode);
	HAL_Delay(15);
	BNO055_SetMode(huart, modeback);
	HAL_Delay(20);

	return 1;
}

uint8_t BNO055_SetAxisSign(I2C_HandleTypeDef *huart,bno055_axis_remap_sign_t remapsign){
	if (!BNO055_Chip_ID_Check(huart)) {
			return 0;
		}
		return 1;

	bno055_opmode_t modeback = BNO055_GetMode(huart);

	BNO055_SetMode(huart,OPERATION_MODE_CONFIG);
	HAL_Delay(25);
	write8(huart,BNO055_WRITE_ADDR,BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
	HAL_Delay(10);
	/* Set the requested operating mode (see section 3.3) */
	BNO055_SetMode(huart,modeback);
	HAL_Delay(20);
}

int8_t BNO055_GetTemp(I2C_HandleTypeDef *huart){
	if (!BNO055_Chip_ID_Check(huart)) {
			return 0;
		}

	return read8(huart, BNO055_READ_ADDR,BNO055_TEMP_ADDR);
}

