#include "MPU9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "header.h"
#include "ml_math_func.h"

void DMPSetup(void)
{
	u8 errcode, sensor;
	sensor = INV_XYZ_GYRO | INV_XYZ_ACCEL;
	errcode = mpu_set_sensors(sensor);
	if (errcode)
		printf("error:fail to setup the sensor for DMP function\r\n");
  errcode = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); /* Push both gyro and accel data into the FIFO. */
	if (errcode)
		printf("error:fail to push gyro and accel data into FIFO\r\n");
  errcode = mpu_set_sample_rate(DEFAULT_MPU_HZ);	
	if (errcode)
		printf("error:fail to set sample rate for IMU\r\n");
}



