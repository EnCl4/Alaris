/*
* icm20948.c
*
*  Created on: Dec 26, 2020
*      Author: mokhwasomssi
*/


#include "icm20948.h"
#include "dapu_config.h"
#include "dapu_spi.h"
#include <string.h>

#if ICM20948_USE_I2C
#include "i2c.h"
/* On its own bus the IMU has no contention: the acquisition task is the only
 * caller once boot is done. */
#define ICM_BUS_LOCK()		((void)0)
#define ICM_BUS_UNLOCK()	((void)0)
#else
#define ICM_BUS_LOCK()		dapu_spi_lock()
#define ICM_BUS_UNLOCK()	dapu_spi_unlock()
#endif


static float gyro_scale_factor;
static float accel_scale_factor;

static bool  icm20948_ready;
static bool  ak09916_ready;

/* Number of who_am_i retries before declaring the device absent. The original
 * library spun forever here, which would hang the whole DAPU if a sensor were
 * unplugged during a bench test. */
#define ICM20948_WHO_AM_I_RETRIES   10u


/* The bank register would otherwise be written on every single access, so it
 * is cached. On I2C that halves the traffic of a sensor read, which matters at
 * 200 Hz. 0xFF means "unknown", forcing the next access to re-select. */
static uint8_t  s_bank_cache = 0xFFu;

#if ICM20948_USE_I2C
static bool     i2c_find_address(void);
#endif

/* Static Functions */
static void     select_user_bank(userbank ub);

static uint8_t  read_single_icm20948_reg(userbank ub, uint8_t reg);
static void     write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val);
static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len);
static void     write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len);

static uint8_t  read_single_ak09916_reg(uint8_t reg);
static void     write_single_ak09916_reg(uint8_t reg, uint8_t val);
static uint8_t* read_multiple_ak09916_reg(uint8_t reg, uint8_t len);


/* Main Functions */

/* Configures I2C slave 0 to pull ST1 + HXL..HZH + TMPS + ST2 (9 bytes from
 * MAG_ST1) out of the AK09916 automatically, at the ICM's own ODR. The
 * magnetometer can then be read as plain SPI registers with no HAL_Delay(),
 * which matters because the IMU task runs at 200 Hz. */
static void ak09916_setup_continuous_read(void)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG,  MAG_ST1);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | 9);
	HAL_Delay(10);
}

bool icm20948_init(void)
{
	uint32_t tries = ICM20948_WHO_AM_I_RETRIES;

	icm20948_ready = false;
	s_bank_cache   = 0xFFu;

	ICM_BUS_LOCK();

#if ICM20948_USE_I2C
	/* Find the module first: without an ACK there is no point clocking out
	 * register reads, and this distinguishes "wrong address" from "silent". */
	if(!i2c_find_address())
	{
		ICM_BUS_UNLOCK();
		return false;
	}
#endif

	while(tries-- > 0u && !icm20948_who_am_i())
	{
		HAL_Delay(5);
	}
	if(!icm20948_who_am_i())
	{
		ICM_BUS_UNLOCK();
		return false;
	}

	icm20948_device_reset();
	icm20948_wakeup();

	icm20948_clock_source(1);
	icm20948_odr_align_enable();

#if !ICM20948_USE_I2C
	/* Sets I2C_IF_DIS - would switch off the very interface we are using. */
	icm20948_spi_slave_enable();
#endif

	icm20948_gyro_low_pass_filter(0);
	icm20948_accel_low_pass_filter(0);

	icm20948_gyro_sample_rate_divider(0);
	icm20948_accel_sample_rate_divider(0);

	icm20948_gyro_calibration();
	icm20948_accel_calibration();

	icm20948_gyro_full_scale_select(ICM_GYRO_FULL_SCALE);
	icm20948_accel_full_scale_select(ICM_ACCEL_FULL_SCALE);

	ICM_BUS_UNLOCK();

	icm20948_ready = true;
	return true;
}

bool ak09916_init(void)
{
	uint32_t tries = ICM20948_WHO_AM_I_RETRIES;

	ak09916_ready = false;

	if(!icm20948_ready)
	{
		return false;
	}

	ICM_BUS_LOCK();

	icm20948_i2c_master_reset();
	icm20948_i2c_master_enable();
	icm20948_i2c_master_clk_frq(7);

	while(tries-- > 0u && !ak09916_who_am_i())
	{
		HAL_Delay(5);
	}
	if(!ak09916_who_am_i())
	{
		ICM_BUS_UNLOCK();
		return false;
	}

	ak09916_soft_reset();
	ak09916_operation_mode_setting(continuous_measurement_100hz);
	ak09916_setup_continuous_read();

	ICM_BUS_UNLOCK();

	ak09916_ready = true;
	return true;
}

bool icm20948_present(void) { return icm20948_ready; }
bool ak09916_present(void)  { return ak09916_ready;  }

/* ACCEL_XOUT_H (0x2D) .. GYRO_ZOUT_L (0x38) are contiguous, so accelerometer
 * and gyroscope come out of one 12 byte burst. That halves the bus traffic
 * versus two separate reads - which matters on I2C at 200 Hz - and guarantees
 * both come from the same sample instant. */
bool icm20948_read_imu(axises* accel_g, axises* gyro_dps)
{
	uint8_t* b;

	if(!icm20948_ready)
	{
		return false;
	}

	ICM_BUS_LOCK();
	b = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 12);
	ICM_BUS_UNLOCK();

	accel_g->x = (float)(int16_t)(b[0] << 8 | b[1]) / accel_scale_factor;
	accel_g->y = (float)(int16_t)(b[2] << 8 | b[3]) / accel_scale_factor;
	/* Scale factor added back because the calibration routine offsets out
	 * gravity - same convention as the upstream library. */
	accel_g->z = ((float)(int16_t)(b[4] << 8 | b[5]) + accel_scale_factor)
	             / accel_scale_factor;

	gyro_dps->x = (float)(int16_t)(b[6] << 8 | b[7])  / gyro_scale_factor;
	gyro_dps->y = (float)(int16_t)(b[8] << 8 | b[9])  / gyro_scale_factor;
	gyro_dps->z = (float)(int16_t)(b[10] << 8 | b[11]) / gyro_scale_factor;

	return true;
}

bool ak09916_read_mag_ut(axises* mag_ut)
{
	bool ok;

	if(!ak09916_ready)
	{
		return false;
	}

	ICM_BUS_LOCK();
	ok = ak09916_mag_read_uT(mag_ut);
	ICM_BUS_UNLOCK();

	return ok;
}

void icm20948_gyro_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

void icm20948_accel_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]) + accel_scale_factor; 
	// Add scale factor because calibraiton function offset gravity acceleration.
}

/* Reads the mirror registers kept up to date by the ICM's own I2C master
 * (see ak09916_setup_continuous_read). Layout of the 9 mirrored bytes:
 * [0] ST1, [1..6] HXL..HZH, [7] TMPS, [8] ST2. */
bool ak09916_mag_read(axises* data)
{
	uint8_t* buf = read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, 9);

	if(!(buf[0] & 0x01))	return false;	// ST1.DRDY - no new sample
	if(buf[8] & 0x08)		return false;	// ST2.HOFL - magnetic overflow

	data->x = (int16_t)(buf[2] << 8 | buf[1]);
	data->y = (int16_t)(buf[4] << 8 | buf[3]);
	data->z = (int16_t)(buf[6] << 8 | buf[5]);

	return true;
}

void icm20948_gyro_read_dps(axises* data)
{
	icm20948_gyro_read(data);

	data->x /= gyro_scale_factor;
	data->y /= gyro_scale_factor;
	data->z /= gyro_scale_factor;
}

void icm20948_accel_read_g(axises* data)
{
	icm20948_accel_read(data);

	data->x /= accel_scale_factor;
	data->y /= accel_scale_factor;
	data->z /= accel_scale_factor;
}

bool ak09916_mag_read_uT(axises* data)
{
	axises temp;
	bool new_data = ak09916_mag_read(&temp);
	if(!new_data)	return false;

	data->x = (float)(temp.x * 0.15);
	data->y = (float)(temp.y * 0.15);
	data->z = (float)(temp.z * 0.15);

	return true;
}	


/* Sub Functions */
static uint8_t s_whoami_raw = 0xFF;

uint8_t icm20948_whoami_raw(void) { return s_whoami_raw; }

bool icm20948_who_am_i()
{
	uint8_t icm20948_id = read_single_icm20948_reg(ub_0, B0_WHO_AM_I);

	s_whoami_raw = icm20948_id;

	if(icm20948_id == ICM20948_ID)
		return true;
	else
		return false;
}

bool ak09916_who_am_i()
{
	uint8_t ak09916_id = read_single_ak09916_reg(MAG_WIA2);

	if(ak09916_id == AK09916_ID)
		return true;
	else
		return false;
}

void icm20948_device_reset()
{
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
	HAL_Delay(100);
	s_bank_cache = 0xFFu;	/* the reset puts the device back in bank 0 */
}

void ak09916_soft_reset()
{
	write_single_ak09916_reg(MAG_CNTL3, 0x01);
	HAL_Delay(100);
}

void icm20948_wakeup()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val &= 0xBF;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

void icm20948_sleep()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= 0x40;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

void icm20948_spi_slave_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x10;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_reset()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x02;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x20;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
	HAL_Delay(100);
}

void icm20948_i2c_master_clk_frq(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL);
	new_val |= config;

	write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, new_val);	
}

void icm20948_clock_source(uint8_t source)
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= source;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
}

void icm20948_odr_align_enable()
{
	write_single_icm20948_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	new_val |= config << 3;

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	new_val |= config << 3;

	/* Upstream library wrote this back to B2_GYRO_CONFIG_1 - corrected. */
	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}

void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
	write_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
}

void icm20948_accel_sample_rate_divider(uint16_t divider)
{
	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);

	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
}

void ak09916_operation_mode_setting(operation_mode mode)
{
	write_single_ak09916_reg(MAG_CNTL2, mode);
	HAL_Delay(100);
}

void icm20948_gyro_calibration()
{
	axises temp;
	int32_t gyro_bias[3] = {0};
	uint8_t gyro_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_gyro_read(&temp);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;
	}

	gyro_bias[0] /= 100;
	gyro_bias[1] /= 100;
	gyro_bias[2] /= 100;

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Biases are additive, so change sign on calculated average gyro biases
	gyro_offset[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; 
	gyro_offset[1] = (-gyro_bias[0] / 4)       & 0xFF; 
	gyro_offset[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	gyro_offset[3] = (-gyro_bias[1] / 4)       & 0xFF;
	gyro_offset[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	gyro_offset[5] = (-gyro_bias[2] / 4)       & 0xFF;
	
	write_multiple_icm20948_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}

void icm20948_accel_calibration()
{
	axises temp;
	uint8_t* temp2;
	uint8_t* temp3;
	uint8_t* temp4;
	
	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_accel_read(&temp);
		accel_bias[0] += temp.x;
		accel_bias[1] += temp.y;
		accel_bias[2] += temp.z;
	}

	accel_bias[0] /= 100;
	accel_bias[1] /= 100;
	accel_bias[2] /= 100;

	uint8_t mask_bit[3] = {0, 0, 0};

	temp2 = read_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, 2);
	accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
	mask_bit[0] = temp2[1] & 0x01;

	temp3 = read_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, 2);
	accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
	mask_bit[1] = temp3[1] & 0x01;

	temp4 = read_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, 2);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
	mask_bit[2] = temp4[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  	accel_offset[1] = (accel_bias_reg[0])      & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  	accel_offset[3] = (accel_bias_reg[1])      & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2])      & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];
	
	write_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
	write_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
	write_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	
	switch(full_scale)
	{
		case _250dps :
			new_val |= 0x00;
			gyro_scale_factor = 131.0;
			break;
		case _500dps :
			new_val |= 0x02;
			gyro_scale_factor = 65.5;
			break;
		case _1000dps :
			new_val |= 0x04;
			gyro_scale_factor = 32.8;
			break;
		case _2000dps :
			new_val |= 0x06;
			gyro_scale_factor = 16.4;
			break;
	}

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	
	switch(full_scale)
	{
		case _2g :
			new_val |= 0x00;
			accel_scale_factor = 16384;
			break;
		case _4g :
			new_val |= 0x02;
			accel_scale_factor = 8192;
			break;
		case _8g :
			new_val |= 0x04;
			accel_scale_factor = 4096;
			break;
		case _16g :
			new_val |= 0x06;
			accel_scale_factor = 2048;
			break;
	}

	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}


/* Static Functions */
/* ---------------------------------------------------------------------------
 * Bus abstraction
 *
 * Everything above this point is bus agnostic; only bus_read()/bus_write()
 * know whether the device is on I2C1 or SPI1. Note the register address
 * carries no direction bit on I2C - that is an SPI-only convention.
 * ------------------------------------------------------------------------- */

#if ICM20948_USE_I2C

static uint8_t s_i2c_addr = (uint8_t)(ICM20948_I2C_ADDR << 1);

uint8_t icm20948_i2c_address(void) { return (uint8_t)(s_i2c_addr >> 1); }

/** Probes 0x68 and 0x69 (AD0 low / high) and latches whichever answers. */
static bool i2c_find_address(void)
{
	static const uint8_t candidates[2] = { 0x68u, 0x69u };

	for (uint32_t i = 0; i < 2u; i++)
	{
		uint8_t addr8 = (uint8_t)(candidates[i] << 1);

		if (HAL_I2C_IsDeviceReady(&hi2c1, addr8, 3, 20) == HAL_OK)
		{
			s_i2c_addr = addr8;
			return true;
		}
	}
	return false;
}

static bool bus_read(uint8_t reg, uint8_t* dst, uint8_t len)
{
	return (HAL_I2C_Mem_Read(&hi2c1, s_i2c_addr, reg,
	                         I2C_MEMADD_SIZE_8BIT, dst, len, 50) == HAL_OK);
}

static bool bus_write(uint8_t reg, const uint8_t* src, uint8_t len)
{
	return (HAL_I2C_Mem_Write(&hi2c1, s_i2c_addr, reg,
	                          I2C_MEMADD_SIZE_8BIT, (uint8_t*)src, len, 50) == HAL_OK);
}

#else	/* SPI1 */

#define ICM_CS_PORT		ICM20948_SPI_CS_PIN_PORT
#define ICM_CS_PIN		ICM20948_SPI_CS_PIN_NUMBER

/* One full-duplex transaction per access. The upstream library used
 * HAL_SPI_Transmit() followed by HAL_SPI_Receive(), which is fine on the F4 but
 * breaks on the H7: the peripheral is disabled between the two calls, stopping
 * and restarting SCK while CS is still low. See dapu_spi_txrx(). */
static bool bus_read(uint8_t reg, uint8_t* dst, uint8_t len)
{
	uint8_t tx[DAPU_SPI_MAX_XFER];
	uint8_t rx[DAPU_SPI_MAX_XFER];

	if ((uint32_t)len + 1u > sizeof(tx))
	{
		return false;
	}

	memset(tx, 0xFF, (size_t)len + 1u);
	tx[0] = READ | reg;

	if (!dapu_spi_txrx(ICM_CS_PORT, ICM_CS_PIN, tx, rx, (uint16_t)(len + 1u)))
	{
		return false;
	}

	memcpy(dst, &rx[1], len);	/* first byte is the address echo */
	return true;
}

static bool bus_write(uint8_t reg, const uint8_t* src, uint8_t len)
{
	uint8_t tx[DAPU_SPI_MAX_XFER];

	if ((uint32_t)len + 1u > sizeof(tx))
	{
		return false;
	}

	tx[0] = WRITE | reg;
	memcpy(&tx[1], src, len);

	return dapu_spi_txrx(ICM_CS_PORT, ICM_CS_PIN, tx, NULL, (uint16_t)(len + 1u));
}

#endif	/* ICM20948_USE_I2C */

static void select_user_bank(userbank ub)
{
	uint8_t value = (uint8_t)ub;

	if (value == s_bank_cache)
	{
		return;
	}
	if (bus_write(REG_BANK_SEL, &value, 1))
	{
		s_bank_cache = value;
	}
	else
	{
		s_bank_cache = 0xFFu;
	}
}

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
{
	uint8_t value = 0;

	select_user_bank(ub);
	(void)bus_read(reg, &value, 1);

	return value;
}

static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val)
{
	select_user_bank(ub);
	(void)bus_write(reg, &val, 1);
}

static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len)
{
	/* 16 bytes: the magnetometer mirror needs 9, the IMU burst needs 12.
	 * Shared static, only safe because there is a single caller at a time. */
	static uint8_t reg_val[16];

	if (len > sizeof(reg_val))
	{
		return reg_val;
	}

	select_user_bank(ub);
	(void)bus_read(reg, reg_val, len);

	return reg_val;
}

static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
	select_user_bank(ub);
	(void)bus_write(reg, val, len);
}

static uint8_t read_single_ak09916_reg(uint8_t reg)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);

	HAL_Delay(1);
	return read_single_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00);
}

static void write_single_ak09916_reg(uint8_t reg, uint8_t val)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_DO, val);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);
}

/* Unused since the magnetometer moved to the continuous mirror registers,
 * kept for one-shot debugging of the AK09916 over the I2C master. */
__attribute__((unused))
static uint8_t* read_multiple_ak09916_reg(uint8_t reg, uint8_t len)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);

	HAL_Delay(1);
	return read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, len);
}