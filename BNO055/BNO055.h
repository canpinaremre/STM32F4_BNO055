/*
 * @Author: canpinaremre
 * Purpose: To use BNO055 IMU
 * Language:  C
 */


#include "stm32f4xx_hal.h"
#include "main.h"
#include <module/i2c/i2c_read_write.h>

#define BNO055_READ_ADDR 0x51 //1010001 0x29 is 7 bit address
#define BNO055_WRITE_ADDR 0x50 //1010000 0x29 is 7 bit address
#define BNO055_ID 0xA0
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_PAGE_ADDR 0x07 // There are 2 pages. Page 0 and page 1
#define BNO055_OPR_MODE_ADDR 0X3D

/* Quaternion data registers */
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0X20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR 0X21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR 0X22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR 0X23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR 0X24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR 0X25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR 0X26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR 0X27

/* Euler data registers */
#define BNO055_EUL_DATA_x_LSB_ADDR 0x1A
#define BNO055_EUL_DATA_x_MSB_ADDR 0x1B
#define BNO055_EUL_DATA_y_LSB_ADDR 0x1C
#define BNO055_EUL_DATA_y_MSB_ADDR 0x1D
#define BNO055_EUL_DATA_z_LSB_ADDR 0x1E
#define BNO055_EUL_DATA_z_MSB_ADDR 0x1F

/* Accel data register */
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0X08
#define BNO055_ACCEL_DATA_X_MSB_ADDR 0X09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR 0X0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR 0X0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR 0X0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR 0X0D

/* Mag data register */
#define BNO055_MAG_DATA_X_LSB_ADDR 0X0E
#define BNO055_MAG_DATA_X_MSB_ADDR 0X0F
#define BNO055_MAG_DATA_Y_LSB_ADDR 0X10
#define BNO055_MAG_DATA_Y_MSB_ADDR 0X11
#define BNO055_MAG_DATA_Z_LSB_ADDR 0X12
#define BNO055_MAG_DATA_Z_MSB_ADDR 0X13

/* Gyro data registers */
#define BNO055_GYRO_DATA_X_LSB_ADDR 0X14
#define BNO055_GYRO_DATA_X_MSB_ADDR 0X15
#define BNO055_GYRO_DATA_Y_LSB_ADDR 0X16
#define BNO055_GYRO_DATA_Y_MSB_ADDR 0X17
#define BNO055_GYRO_DATA_Z_LSB_ADDR 0X18
#define BNO055_GYRO_DATA_Z_MSB_ADDR 0X19

/* Linear acceleration data registers */
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0X28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 0X29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR 0X2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR 0X2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR 0X2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR 0X2D

/* Gravity data registers */
#define BNO055_GRAVITY_DATA_X_LSB_ADDR 0X2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR 0X2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR 0X30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR 0X31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR 0X32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR 0X33

/* Temperature data register */
#define BNO055_TEMP_ADDR 0X34

/* Status registers */
#define BNO055_CALIB_STAT_ADDR 0X35
#define BNO055_SELFTEST_RESULT_ADDR 0X36
#define BNO055_INTR_STAT_ADDR 0X37

#define BNO055_SYS_CLK_STAT_ADDR 0X38
#define BNO055_SYS_STAT_ADDR 0X39
#define BNO055_SYS_ERR_ADDR 0X3A

/* Unit selection register */
#define BNO055_UNIT_SEL_ADDR 0X3B
#define BNO055_DATA_SELECT_ADDR 0X3C

/* Mode registers */
#define BNO055_OPR_MODE_ADDR 0X3D
#define BNO055_PWR_MODE_ADDR 0X3E

#define BNO055_SYS_TRIGGER_ADDR 0X3F
#define BNO055_TEMP_SOURCE_ADDR 0X40

/* Axis remap registers */
#define BNO055_AXIS_MAP_CONFIG_ADDR 0X41
#define BNO055_AXIS_MAP_SIGN_ADDR 0X42

/* SIC registers */
#define BNO055_SIC_MATRIX_0_LSB_ADDR 0X43
#define BNO055_SIC_MATRIX_0_MSB_ADDR 0X44
#define BNO055_SIC_MATRIX_1_LSB_ADDR 0X45
#define BNO055_SIC_MATRIX_1_MSB_ADDR 0X46
#define BNO055_SIC_MATRIX_2_LSB_ADDR 0X47
#define BNO055_SIC_MATRIX_2_MSB_ADDR 0X48
#define BNO055_SIC_MATRIX_3_LSB_ADDR 0X49
#define BNO055_SIC_MATRIX_3_MSB_ADDR 0X4A
#define BNO055_SIC_MATRIX_4_LSB_ADDR 0X4B
#define BNO055_SIC_MATRIX_4_MSB_ADDR 0X4C
#define BNO055_SIC_MATRIX_5_LSB_ADDR 0X4D
#define BNO055_SIC_MATRIX_5_MSB_ADDR 0X4E
#define BNO055_SIC_MATRIX_6_LSB_ADDR 0X4F
#define BNO055_SIC_MATRIX_6_MSB_ADDR 0X50
#define BNO055_SIC_MATRIX_7_LSB_ADDR 0X51
#define BNO055_SIC_MATRIX_7_MSB_ADDR 0X52
#define BNO055_SIC_MATRIX_8_LSB_ADDR 0X53
#define BNO055_SIC_MATRIX_8_MSB_ADDR 0X54

/* Accelerometer Offset registers */
#define ACCEL_OFFSET_X_LSB_ADDR 0X55
#define ACCEL_OFFSET_X_MSB_ADDR 0X56
#define ACCEL_OFFSET_Y_LSB_ADDR 0X57
#define ACCEL_OFFSET_Y_MSB_ADDR 0X58
#define ACCEL_OFFSET_Z_LSB_ADDR 0X59
#define ACCEL_OFFSET_Z_MSB_ADDR 0X5A

/* Magnetometer Offset registers */
#define MAG_OFFSET_X_LSB_ADDR 0X5B
#define MAG_OFFSET_X_MSB_ADDR 0X5C
#define MAG_OFFSET_Y_LSB_ADDR 0X5D
#define MAG_OFFSET_Y_MSB_ADDR 0X5E
#define MAG_OFFSET_Z_LSB_ADDR 0X5F
#define MAG_OFFSET_Z_MSB_ADDR 0X60

/* Gyroscope Offset register s*/
#define GYRO_OFFSET_X_LSB_ADDR 0X61
#define GYRO_OFFSET_X_MSB_ADDR 0X62
#define GYRO_OFFSET_Y_LSB_ADDR 0X63
#define GYRO_OFFSET_Y_MSB_ADDR 0X64
#define GYRO_OFFSET_Z_LSB_ADDR 0X65
#define GYRO_OFFSET_Z_MSB_ADDR 0X66

/* Radius registers */
#define ACCEL_RADIUS_LSB_ADDR 0X67
#define ACCEL_RADIUS_MSB_ADDR 0X68
#define MAG_RADIUS_LSB_ADDR 0X69
#define MAG_RADIUS_MSB_ADDR 0X6A



/** BNO055 power settings */
typedef enum {
	POWER_MODE_NORMAL = 0X00,
	POWER_MODE_LOWPOWER = 0X01,
	POWER_MODE_SUSPEND = 0X02
} bno055_powermode_t;

/** Operation mode settings **/
typedef enum {
	OPERATION_MODE_CONFIG = 0X00,
	OPERATION_MODE_ACCONLY = 0X01,
	OPERATION_MODE_MAGONLY = 0X02,
	OPERATION_MODE_GYRONLY = 0X03,
	OPERATION_MODE_ACCMAG = 0X04,
	OPERATION_MODE_ACCGYRO = 0X05,
	OPERATION_MODE_MAGGYRO = 0X06,
	OPERATION_MODE_AMG = 0X07,
	OPERATION_MODE_IMUPLUS = 0X08,
	OPERATION_MODE_COMPASS = 0X09,
	OPERATION_MODE_M4G = 0X0A,
	OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
	OPERATION_MODE_NDOF = 0X0C
} bno055_opmode_t;

/** Remap settings **/
typedef enum {
	REMAP_CONFIG_P0 = 0x21,
	REMAP_CONFIG_P1 = 0x24, // default
	REMAP_CONFIG_P2 = 0x24,
	REMAP_CONFIG_P3 = 0x21,
	REMAP_CONFIG_P4 = 0x24,
	REMAP_CONFIG_P5 = 0x21,
	REMAP_CONFIG_P6 = 0x21,
	REMAP_CONFIG_P7 = 0x24
} bno055_axis_remap_config_t;

/** Remap Signs **/
typedef enum {
	REMAP_SIGN_P0 = 0x04,
	REMAP_SIGN_P1 = 0x00, // default
	REMAP_SIGN_P2 = 0x06,
	REMAP_SIGN_P3 = 0x02,
	REMAP_SIGN_P4 = 0x03,
	REMAP_SIGN_P5 = 0x01,
	REMAP_SIGN_P6 = 0x07,
	REMAP_SIGN_P7 = 0x05
} bno055_axis_remap_sign_t;

uint8_t buffer[9];
int16_t data_read[5];

void reset_Buffer(void);

//Checks chip ID for BNO055
//If chip ID does not match it return 0;
uint8_t BNO055_Chip_ID_Check(I2C_HandleTypeDef *huart);

//Initialize BNO055.
//Return 1 if succesful else 0
uint8_t BNO055_Init(I2C_HandleTypeDef *huart, bno055_opmode_t mode,uint8_t delay_time);

//BNO055 Register Map Page Get and Set
//Default page is 0. Other page is 1. So return 2 in fail.
//Set returns 1 in success else 0.
uint8_t BNO055_Get_Page(I2C_HandleTypeDef *huart);
uint8_t BNO055_Set_Page(I2C_HandleTypeDef *huart,uint8_t page);

//Reading Raw Data from BNO055
uint8_t BNO055_Read_Eul(I2C_HandleTypeDef *huart,float *eulerXYZ);
uint8_t BNO055_Read_Qua(I2C_HandleTypeDef *huart,float *quaternionWXYZ);
uint8_t BNO055_Read_Acc(I2C_HandleTypeDef *huart,float *accelXYZ);
uint8_t BNO055_Read_Mag(I2C_HandleTypeDef *huart,float *magXYZ);
uint8_t BNO055_Read_Gyr(I2C_HandleTypeDef *huart,float *gyrXYZ);
uint8_t BNO055_Read_Grv(I2C_HandleTypeDef *huart,float *grvXYZ);
uint8_t BNO055_Read_Lia(I2C_HandleTypeDef *huart,float *liaXYZ);

//Sets parameters for measurements
uint8_t BNO055_Set_Eul(I2C_HandleTypeDef *huart);
uint8_t BNO055_Set_Qua(I2C_HandleTypeDef *huart);
uint8_t BNO055_Set_Acc(I2C_HandleTypeDef *huart);
uint8_t BNO055_Set_Mag(I2C_HandleTypeDef *huart);
uint8_t BNO055_Set_Gyr(I2C_HandleTypeDef *huart);
uint8_t BNO055_Set_Grv(I2C_HandleTypeDef *huart);
uint8_t BNO055_Set_Lia(I2C_HandleTypeDef *huart);

uint8_t BNO055_SetMode(I2C_HandleTypeDef *huart,bno055_opmode_t mode);
bno055_opmode_t BNO055_GetMode(I2C_HandleTypeDef *huart);
uint8_t BNO055_SetAxisRemap(I2C_HandleTypeDef *huart,bno055_axis_remap_config_t remapcode);
uint8_t BNO055_SetAxisSign(I2C_HandleTypeDef *huart,bno055_axis_remap_sign_t remapsign);

int8_t BNO055_GetTemp(I2C_HandleTypeDef *huart);

