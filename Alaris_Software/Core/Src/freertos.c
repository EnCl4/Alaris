/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "icm20948.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>
#include "fatfs.h"
#include "sd_functions.h"
#include "sd_benchmark.h"
#include "icm20948.h"
#include "adc.h"

#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osMutexId_t uartMutex;

int _write(int file, char *ptr, int len)
{
    osMutexAcquire(uartMutex, osWaitForever);
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    osMutexRelease(uartMutex);
    return len;
}

axises my_gyro;
axises my_accel;
axises my_mag;

#define RX_BUFFER_SIZE 128

volatile uint8_t uart_rx_flag;


//uint8_t tx_buffer[] = "\r\nRunning Code\r\n";
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_old_pos = 0;

void print_ubx_hex(uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        printf("%02X ", buf[i]);
    }
    printf("\r\n ");
}

static const uint8_t enable_GxGGA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28};
static const uint8_t disable_GxGGA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
static const uint8_t enable_GxGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x2F};
static const uint8_t disable_GxGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
static const uint8_t enable_GxGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x36};
static const uint8_t disable_GxGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
static const uint8_t enable_GxGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3D};
static const uint8_t disable_GxGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
static const uint8_t enable_GxRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04, 0x44};
static const uint8_t disable_GxRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
static const uint8_t enable_GxVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x05, 0x4B};
static const uint8_t disable_GxVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
static const uint8_t enable_Nav_PTV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1};
static const uint8_t disable_Nav_PTV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC};
static const uint8_t save_CFG[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF};
static const uint8_t Hz5_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
static const uint8_t full_power[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0x5A};
static const uint8_t NAV5_pedestrian[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xA2};
static const uint8_t rateUCT_10Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x00, 0x00, 0x79, 0x10};
static const uint8_t rateGPS_10Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
static const uint8_t rateGLO_10Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x02, 0x00, 0x7B, 0x14};
static const uint8_t rateGAL_10Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x04, 0x00, 0x7D, 0x18};
static const uint8_t GNSS_config[] = {0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x03, 0x04, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0x2D, 0x45};
//static const uint8_t boutrate9600[] = {};
void configNEOM8N(const uint8_t *comando, uint16_t tamanho)
{
    for (int i = 0; i < 3; i++)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)comando, tamanho, HAL_MAX_DELAY);
        HAL_Delay(3);
    }
}

static void configurarGPS(void){
	configNEOM8N(disable_GxGGA, sizeof(disable_GxGGA));
	configNEOM8N(disable_GxGLL, sizeof(disable_GxGLL));
	configNEOM8N(disable_GxGSA, sizeof(disable_GxGSA));
	configNEOM8N(disable_GxGSV, sizeof(disable_GxGSV));
	configNEOM8N(disable_GxRMC, sizeof(disable_GxRMC));
	configNEOM8N(disable_GxVTG, sizeof(disable_GxVTG));
	configNEOM8N(enable_Nav_PTV, sizeof(enable_Nav_PTV));
	configNEOM8N(full_power, sizeof(full_power));
	configNEOM8N(NAV5_pedestrian, sizeof(NAV5_pedestrian));
	configNEOM8N(rateUCT_10Hz, sizeof(rateUCT_10Hz));
	configNEOM8N(rateGPS_10Hz, sizeof(rateGPS_10Hz));
	configNEOM8N(rateGLO_10Hz, sizeof(rateGLO_10Hz));
	configNEOM8N(rateGAL_10Hz, sizeof(rateGAL_10Hz));
	configNEOM8N(GNSS_config, sizeof(GNSS_config));

	//AINDA TA FALTANDO TROCAR O BOUTRATE

	configNEOM8N(save_CFG, sizeof(save_CFG));
}

typedef struct {
	unsigned short int	year;
	unsigned char   month;
	unsigned char   day;
	unsigned char   hour;
	unsigned char 	min;
	unsigned char 	sec;
	unsigned char   valid; 	 	/* Validity flags */
	unsigned long 	tAcc;		/* GPS time of week of the navigation epoch. */
	signed   long  	nano;		/* Fraction of second, range -1e9 .. 1e9 (UTC) */
	unsigned char   fixType;		/* GNSSfix Type: 0: no fix ; 1: dead reckoning only 2: 2D-fix
												 3: 3D-fix   4: GNSS + dead reckoning combined
												 5: time only fix */
	unsigned char 	flags;		/* Fix status flags (see graphic below) */
	unsigned char	flags2;		/* Additional flags (see graphic below) */
	unsigned char	numSV;		/* Number of satellites used in Nav Solution */
	signed long		lon;			/* Longitude */
	signed long		lat;			/* Latitude (deg) */
	signed long		height;		/* Height above ellipsoid */
	signed long		hMSL;		/* Height above mean sea level */
	signed long		hAcc;		/* Horizontal accuracy estimate */
	unsigned long	vAcc;		/* Vertical accuracy estimate */
	signed long		velN;		/* NED north velocity */
	signed long		velE;		/* NED east velocity */
	signed long		velD;	 	/* NED down velocity */
	signed long 		gSpeed;		/* Ground Speed (2-D) */
	signed long		headMot;		/* Heading of motion (2-D) */
	unsigned long	sAcc;		/* Speed accuracy estimate */
	unsigned long 	headAcc;		/* Heading accuracy estimate (both motion and vehicle) */
	unsigned short	pDOP;		/* Position DOP */
/*  unsigned char	reserved;*/ /* Byte offset 78 is reserved, this is only here to indicate it */
	signed long		headVeh;		/* Heading of vehicle (2-D) */
	signed short		magDec;		/* Magnetic declination */
	unsigned short	magAcc;		/* Magnetic declination accuracy */

}gpsData;

bool parsePTV(uint8_t *buffer, uint16_t len, gpsData * data){

	if (len != 100)
	    {
	        return false;
	    }
//	if (buffer[0] != 0xB5 || buffer[1] != 0x62)
//	        return false;
	unsigned short int offset = 4; // B5 62 (HEADER) 01 (Class) 07 (ID)
	data->year = ((uint16_t)buffer[9] << 8 ) | ((uint16_t)buffer[8]) ;
	data->month = (uint16_t)buffer[6 + offset];
	data->day = (uint16_t)buffer[7 + offset];
	data->hour	 =  (uint16_t)buffer[8 + offset] ;
	data->min	 =  (uint16_t)buffer[9 + offset] ;
	data->sec	 =  (uint16_t)buffer[10 + offset];
	return true;
};

//static const uint8_t getPVTData[] =       {0xB5,0x62,0x01,0x07,0x00,0x00,0x08,0x19};

gpsData gps;

uint16_t dma_last_pos = 0;

void UART_ProcessDMA(void)
{
    uint16_t dma_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    while (dma_last_pos != dma_pos)
    {
        uint8_t byte = rx_buffer[dma_last_pos];
        dma_last_pos = (dma_last_pos + 1) % RX_BUFFER_SIZE;

        UBX_ParseByte(byte);
    }
}

typedef enum {
    UBX_SYNC1,
    UBX_SYNC2,
    UBX_CLASS,
    UBX_ID,
    UBX_LEN1,
    UBX_LEN2,
    UBX_PAYLOAD,
    UBX_CKA,
    UBX_CKB
} UBX_State;

UBX_State ubx_state = UBX_SYNC1;

uint8_t  ubx_class, ubx_id;
uint16_t ubx_len, ubx_count;
uint8_t  ubx_payload[100];
uint8_t  ck_a, ck_b;

void ubx_checksum(uint8_t b)
{
    ck_a += b;
    ck_b += ck_a;
}

void UBX_ParseByte(uint8_t b)
{
    switch (ubx_state)
    {
    case UBX_SYNC1:
        if (b == 0xB5) ubx_state = UBX_SYNC2;
        break;

    case UBX_SYNC2:
        if (b == 0x62) ubx_state = UBX_CLASS;
        else ubx_state = UBX_SYNC1;
        break;

    case UBX_CLASS:
        ubx_class = b;
        ck_a = ck_b = 0;
        ubx_checksum(b);
        ubx_state = UBX_ID;
        break;

    case UBX_ID:
        ubx_id = b;
        ubx_checksum(b);
        ubx_state = UBX_LEN1;
        break;

    case UBX_LEN1:
        ubx_len = b;
        ubx_checksum(b);
        ubx_state = UBX_LEN2;
        break;

    case UBX_LEN2:
        ubx_len |= (uint16_t)b << 8;
        ubx_checksum(b);

        if (ubx_len > sizeof(ubx_payload))
        {
            ubx_state = UBX_SYNC1;
        }
        else
        {
            ubx_count = 0;
            ubx_state = UBX_PAYLOAD;
        }
        break;

    case UBX_PAYLOAD:
        ubx_payload[ubx_count++] = b;
        ubx_checksum(b);

        if (ubx_count >= ubx_len)
            ubx_state = UBX_CKA;
        break;

    case UBX_CKA:
        if (b == ck_a) ubx_state = UBX_CKB;
        else ubx_state = UBX_SYNC1;
        break;

    case UBX_CKB:
        if (b == ck_b)
        {
            UBX_HandleMessage(ubx_class, ubx_id, ubx_payload, ubx_len);
        }
        ubx_state = UBX_SYNC1;
        break;
    }
}
bool printar = 0;
void UBX_HandleMessage(uint8_t cls, uint8_t id,
                       uint8_t *payload, uint16_t len)
{
    if (cls == 0x01 && id == 0x07 && len >= 12)
    {
        gps.year  = payload[4] | (payload[5] << 8);
        gps.month = payload[6];
        gps.day   = payload[7];
        gps.hour = payload[8];
        gps.min = payload[9];
        gps.sec = payload[10];

//        gps.validDate 		=  payload[11] 	    & 0b00000001;
//        gps.validTime 		= (payload[11] >> 1) & 0b00000001;
//        gps.fullyResolved	= (payload[11] >> 2) & 0b00000001;
//        gps.validMag		= (payload[11] >> 3) & 0b00000001;

        gps.tAcc    = (payload[15] << 24) | (payload[14] << 16) | (payload [13] << 8) | payload[12] ;
        gps.nano	  = (payload[19] << 24) | (payload[18] << 16) | (payload [17] << 8) | payload[16] ;
        gps.fixType =  payload[20] ;

            /* Handle the 'flags' in byte 21 */
//        gps.gnssFixOK		=  payload[21] 	    & 0b00000001;
//        gps.diffSoln		= (payload[21] >> 1) & 0b00000001;
//        gps.psmState		= (payload[21] >> 2) & 0b00000111;
//        gps.headVehValid  = (payload[21] >> 5) & 0b00000001;
//        gps.carrSoln		= (payload[21] >> 6) & 0b00000011;
//
//            /* Handle the 'flags' in byte 22 */
//        gps.confirmedAvai = (payload[22] >> 5) & 0b00000001;
//        gps.confirmedDate = (payload[22] >> 6) & 0b00000001;
//        gps.confirmedTime = (payload[22] >> 7) & 0b00000001;

        gps.numSV 	  =  payload[23];

        gps.lon	  = (payload[27] << 24) | (payload[26] << 16) | (payload [25] << 8) | payload[24] ;
        gps.lat	  = (payload[31] << 24) | (payload[30] << 16) | (payload [29] << 8) | payload[28] ;
        gps.height  = (payload[35] << 24) | (payload[34] << 16) | (payload [33] << 8) | payload[32] ;
        gps.hMSL    = (payload[39] << 24) | (payload[38] << 16) | (payload [37] << 8) | payload[36] ;
        gps.hAcc    = (payload[43] << 24) | (payload[42] << 16) | (payload [41] << 8) | payload[40] ;
        gps.vAcc    = (payload[47] << 24) | (payload[46] << 16) | (payload [45] << 8) | payload[44] ;
        gps.velN    = (payload[51] << 24) | (payload[50] << 16) | (payload [49] << 8) | payload[48] ;
        gps.velE    = (payload[55] << 24) | (payload[54] << 16) | (payload [53] << 8) | payload[52] ;
        gps.velD    = (payload[59] << 24) | (payload[58] << 16) | (payload [57] << 8) | payload[56] ;
        gps.gSpeed  = (payload[63] << 24) | (payload[62] << 16) | (payload [61] << 8) | payload[60] ;
        gps.headMot = (payload[67] << 24) | (payload[66] << 16) | (payload [65] << 8) | payload[64] ;
        gps.sAcc    = (payload[71] << 24) | (payload[70] << 16) | (payload [69] << 8) | payload[68] ;
        gps.headAcc = (payload[75] << 24) | (payload[74] << 16) | (payload [73] << 8) | payload[72] ;
        printar = 1;
    }
    else printar = 0;
}



#define BMP280_CS_PORT BMP280_CS_GPIO_Port
#define BMP280_CS_PIN BMP280_CS_Pin


#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_CALIB00 0x88

#define BMP280_RESET_VALUE 0xB6

static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static int32_t t_fine;

static void BMP280_CS_Low(void) { HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_RESET); }
static void BMP280_CS_High(void) { HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_SET); }

static void BMP280_SPI_Read(uint8_t reg, uint8_t *buf, uint16_t len)
{
uint8_t addr = reg | 0x80; // MSB=1 for read
BMP280_CS_Low();
HAL_SPI_Transmit(&hspi2, &addr, 1, HAL_MAX_DELAY);
HAL_SPI_Receive(&hspi2, buf, len, HAL_MAX_DELAY);
BMP280_CS_High();
}

static void BMP280_SPI_Write(uint8_t reg, uint8_t data)
{
uint8_t buf[2];
buf[0] = reg & 0x7F; // MSB=0 for write
buf[1] = data;
BMP280_CS_Low();
HAL_SPI_Transmit(&hspi2, buf, 2, HAL_MAX_DELAY);
BMP280_CS_High();
}


static uint8_t BMP280_Read8(uint8_t reg)
{
uint8_t val;
BMP280_SPI_Read(reg, &val, 1);
return val;
};

static void BMP280_ReadCalibration(void)
{
uint8_t calib[24];
BMP280_SPI_Read(BMP280_REG_CALIB00, calib, 24);


dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);


dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
}

uint8_t BMP280_Init(void)
{
uint8_t id = BMP280_Read8(BMP280_REG_ID);
if (id != 0x58) return 0; // BMP280 chip ID


// Soft reset
BMP280_SPI_Write(BMP280_REG_RESET, BMP280_RESET_VALUE);
HAL_Delay(10);


BMP280_ReadCalibration();


// Config: standby 1000 ms, filter off
BMP280_SPI_Write(BMP280_REG_CONFIG, (5 << 5) | (0 << 2));


// Ctrl_meas: temp oversampling x2, press oversampling x16, normal mode
BMP280_SPI_Write(BMP280_REG_CTRL_MEAS, (2 << 5) | (5 << 2) | 3);


return 1;
};

static int32_t BMP280_ReadRawTemp(void)
{
uint8_t buf[3];
BMP280_SPI_Read(BMP280_REG_TEMP_MSB, buf, 3);
return (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
};

static int32_t BMP280_ReadRawPress(void){
uint8_t buf[3];
BMP280_SPI_Read(BMP280_REG_PRESS_MSB, buf, 3);
return (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
};


float BMP280_ReadTemperature(void)
{
int32_t adc_T = BMP280_ReadRawTemp();


int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;


t_fine = var1 + var2;


float T = (t_fine * 5 + 128) >> 8;
return T / 100.0f;
};


float BMP280_ReadPressure(void)
{
int32_t adc_P = BMP280_ReadRawPress();


int64_t var1 = ((int64_t)t_fine) - 128000;
int64_t var2 = var1 * var1 * (int64_t)dig_P6;
var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
var2 = var2 + (((int64_t)dig_P4) << 35);
var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;


if (var1 == 0) return 0; // avoid exception


int64_t p = 1048576 - adc_P;
p = (((p << 31) - var2) * 3125) / var1;
var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
var2 = (((int64_t)dig_P8) * p) >> 19;


p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);


return (float)p / 256.0f; // Pa
};


float BMP280_ComputeAltitude(float pressure_pa, float sea_level_pa)
{
return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
};


// Example usage in main loop
void BMP280_Task(float *temperature, float *pressure, float *altitude)
{
    *temperature = BMP280_ReadTemperature();
    *pressure    = BMP280_ReadPressure();
    *altitude    = BMP280_ComputeAltitude(*pressure, 101325.0f);
}

float te, pr, al;

uint16_t pot1;
uint16_t pot2;

uint16_t readValue[2];
/* USER CODE END Variables */
/* Definitions for ICM20948_Accel */
osThreadId_t ICM20948_AccelHandle;
const osThreadAttr_t ICM20948_Accel_attributes = {
  .name = "ICM20948_Accel",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ICM20948_Gyro */
osThreadId_t ICM20948_GyroHandle;
const osThreadAttr_t ICM20948_Gyro_attributes = {
  .name = "ICM20948_Gyro",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for BMP280_Temp_Pre */
osThreadId_t BMP280_Temp_PreHandle;
const osThreadAttr_t BMP280_Temp_Pre_attributes = {
  .name = "BMP280_Temp_Pre",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for SD_data */
osThreadId_t SD_dataHandle;
const osThreadAttr_t SD_data_attributes = {
  .name = "SD_data",
  .stack_size = 128 * 32,
  .priority = (osPriority_t) osPriorityHigh6,
};
/* Definitions for pitotAnalog */
osThreadId_t pitotAnalogHandle;
const osThreadAttr_t pitotAnalog_attributes = {
  .name = "pitotAnalog",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for aoaAnalog */
osThreadId_t aoaAnalogHandle;
const osThreadAttr_t aoaAnalog_attributes = {
  .name = "aoaAnalog",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GPSdataGet */
osThreadId_t GPSdataGetHandle;
const osThreadAttr_t GPSdataGet_attributes = {
  .name = "GPSdataGet",
  .stack_size = 128 * 32,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for GPSdataPlot */
osThreadId_t GPSdataPlotHandle;
const osThreadAttr_t GPSdataPlot_attributes = {
  .name = "GPSdataPlot",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartICm20948_Accel(void *argument);
void StartICM20948_Gyro(void *argument);
void BMP280_Temp_Press_Alt(void *argument);
void SD_dataProcess(void *argument);
void pitotInit(void *argument);
void aoaInit(void *argument);
void gpsDatainit(void *argument);
void gpsPlotInit(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)readValue, 2);

	BMP280_Init();
	icm20948_init();
	ak09916_init();
	configurarGPS();

	HAL_UART_Receive_DMA(&huart2, rx_buffer, sizeof(rx_buffer));

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
		/* Create mutex */
			  const osMutexAttr_t uartMutexAttr = {
			      .name = "UART_Mutex"
			  };
			  uartMutex = osMutexNew(&uartMutexAttr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ICM20948_Accel */
  ICM20948_AccelHandle = osThreadNew(StartICm20948_Accel, NULL, &ICM20948_Accel_attributes);

  /* creation of ICM20948_Gyro */
  ICM20948_GyroHandle = osThreadNew(StartICM20948_Gyro, NULL, &ICM20948_Gyro_attributes);

  /* creation of BMP280_Temp_Pre */
  BMP280_Temp_PreHandle = osThreadNew(BMP280_Temp_Press_Alt, NULL, &BMP280_Temp_Pre_attributes);

  /* creation of SD_data */
  SD_dataHandle = osThreadNew(SD_dataProcess, NULL, &SD_data_attributes);

  /* creation of pitotAnalog */
  pitotAnalogHandle = osThreadNew(pitotInit, NULL, &pitotAnalog_attributes);

  /* creation of aoaAnalog */
  aoaAnalogHandle = osThreadNew(aoaInit, NULL, &aoaAnalog_attributes);

  /* creation of GPSdataGet */
  GPSdataGetHandle = osThreadNew(gpsDatainit, NULL, &GPSdataGet_attributes);

  /* creation of GPSdataPlot */
  GPSdataPlotHandle = osThreadNew(gpsPlotInit, NULL, &GPSdataPlot_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartICm20948_Accel */
int cell = 0;
/**
  * @brief  Function implementing the ICM20948_Accel thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartICm20948_Accel */
void StartICm20948_Accel(void *argument)
{
  /* USER CODE BEGIN StartICm20948_Accel */
  /* Infinite loop */
  for(;;)
  {
	  icm20948_accel_read(&my_accel);
	  icm20948_accel_read_g(&my_accel);
	  cell++;
    osDelay(10);
  }
  /* USER CODE END StartICm20948_Accel */
}

/* USER CODE BEGIN Header_StartICM20948_Gyro */
/**
* @brief Function implementing the ICM20948_Gyro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartICM20948_Gyro */
void StartICM20948_Gyro(void *argument)
{
  /* USER CODE BEGIN StartICM20948_Gyro */
  /* Infinite loop */
  for(;;)
  {
	icm20948_gyro_read(&my_gyro);
	icm20948_gyro_read_dps(&my_gyro);
    osDelay(10);
  }
  /* USER CODE END StartICM20948_Gyro */
}

/* USER CODE BEGIN Header_BMP280_Temp_Press_Alt */
/**
* @brief Function implementing the BMP280_Temp_Pre thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMP280_Temp_Press_Alt */
void BMP280_Temp_Press_Alt(void *argument)
{
  /* USER CODE BEGIN BMP280_Temp_Press_Alt */
  /* Infinite loop */
  for(;;)
  {
	BMP280_Task(&te, &pr, &al);
	//printf(">T:%.2f\n>P:%.2f\n>Alt:%.2f\r\n", te, pr, al);
    osDelay(250);
  }
  /* USER CODE END BMP280_Temp_Press_Alt */
}

/* USER CODE BEGIN Header_SD_dataProcess */
/**
* @brief Function implementing the SD_data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SD_dataProcess */
void SD_dataProcess(void *argument)
{
  /* USER CODE BEGIN SD_dataProcess */
	int count = 0;
	char buffer[128];
  /* Infinite loop */
  for(;;)
  {
	//char buffer[32];   // enough for a float + newline

//	sd_mount();
////	//snprintf(buffer, sizeof(buffer), "%.3f\n", te);
//	sd_append_file("log.txt", "added\n");
//	sd_unmount();

	//sd_benchmark();





	  sd_mount();

	  //snprintf(buffer, sizeof(buffer), "count:%d,C2:%d,T:%.2f,P:%.2f,Alt:%.2f,AZ:%7.3f\r\n", count, cell, te, pr, al,my_accel.z);
	  snprintf(buffer, sizeof(buffer),"%.7f, %.7f \r \n", (float)(gps.lat)/1e7, (float)(gps.lon)/1e7);
	  sd_append_file("log.txt", buffer);

	  sd_unmount();



	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	  count++;

    osDelay(100);
  }
  /* USER CODE END SD_dataProcess */
}

/* USER CODE BEGIN Header_pitotInit */
/**
* @brief Function implementing the pitotAnalog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pitotInit */
void pitotInit(void *argument)
{
  /* USER CODE BEGIN pitotInit */
  /* Infinite loop */
  for(;;)
  {
	  pot1 = (uint16_t) readValue[0];
	  printf(">pot1:%u\r\n", pot1);//

    osDelay(100);
  }
  /* USER CODE END pitotInit */
}

/* USER CODE BEGIN Header_aoaInit */
/**
* @brief Function implementing the aoaAnalog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_aoaInit */
void aoaInit(void *argument)
{
  /* USER CODE BEGIN aoaInit */
  /* Infinite loop */
  for(;;)
  {
	  pot2 = (uint16_t) readValue[1];
	  printf(">pot2:%u\r\n", pot2);//
    osDelay(100);
  }
  /* USER CODE END aoaInit */
}

/* USER CODE BEGIN Header_gpsDatainit */
/**
* @brief Function implementing the GPSdataGet thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gpsDatainit */
void gpsDatainit(void *argument)
{
  /* USER CODE BEGIN gpsDatainit */
  /* Infinite loop */
  for(;;)
  {
	  UART_ProcessDMA();
	  if (printar == 1){
	  	  		  printf("%u,%u,%u,%u,%u,%u\r\n", gps.year, gps.month, gps.day, gps.hour, gps.min, gps.sec);
	  	  		  printf("%.7f, %.7f \r \n", (float)(gps.lat)/1e7, (float)(gps.lon)/1e7);
	  	  	  }
    osDelay(250);
  }
  /* USER CODE END gpsDatainit */
}

/* USER CODE BEGIN Header_gpsPlotInit */
/**
* @brief Function implementing the GPSdataPlot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gpsPlotInit */
void gpsPlotInit(void *argument)
{
  /* USER CODE BEGIN gpsPlotInit */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10000);
  }
  /* USER CODE END gpsPlotInit */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

