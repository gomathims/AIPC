/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "OTAupdate.h"
#include "Relay_controlling.h"

#include<stdio.h>
#include <string.h>
#include <time.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CALIB_VOL1 17338.36190
#define CALIB_CUR1 301062.271062
#define CALIB_POW1 648.885586
#define CALIB_ENE1 4667.50266786

#define CALIB_VOL2 17324.5952
#define CALIB_CUR2 304157.509157
#define CALIB_POW2 739.078751
#define CALIB_ENE2 5667.50266786


#define CALIB_VOL3 17319.6761
#define CALIB_CUR3 304084.249084
#define CALIB_POW3 656.745913
#define CALIB_ENE3 5000.96266786


#define CALIB_VOL4 17319.6761
#define CALIB_CUR4 304084.249084
#define CALIB_POW4 656.745913
#define CALIB_ENE4 5000.96266786

//#define CALIB_VOL 1
//#define CALIB_CUR 1
//#define CALIB_POW 1
//#define CALIB_ENE 1

/*RX buffer size deceleration*/
#define RTC_BAT_CONNECTED 1

#define BUF_SIZE 256

#define FAULT_DELAY 7000
#define RESET_BUT_COMB_PRESS_TIME 3000
#define CONTACTOR_DEBOUNCE_DELAY 30




#define DEFAULT_LOWVOLTAGE_LIMIT 170
#define DEFAULT_HIGHVOLTAGE_LIMIT 270
#define DEFAULT_OVERLOAD_LIMIT 20

#define DEFAULT_POWER_LIMIT 3000



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t setupBL[6];
uint8_t rxDummy[4],rxDataV1[4], rxDataV2[4], rxDataV3[4], rxDataV4[4];
uint8_t rxDataC1[4], rxDataC2[4], rxDataC3[4], rxDataC4[4];
uint8_t rxDataP1[4], rxDataP2[4], rxDataP3[4], rxDataP4[4];
uint8_t rxDataE1[4], rxDataE2[4], rxDataE3[4], rxDataE4[4];
volatile float  voltL1, voltL2, voltL3,voltL4;
volatile float  curL1, curL2, curL3, curL4;
volatile float  powL1, powL2, powL3, powL4;
volatile float  eneL1, eneL2, eneL3, eneL4;

unsigned short uartGoFlag;			//0b->v1, 1b->c1, 2b->p1, 3b->e1, 4b->v2, 5b->c2, 6b->p2, 7b->e2, 8b->v3, 9b->c3, 10b->p3, 11b->e3, 12b->v4, 13b->c4, 14b->p4, 15b->e4.
unsigned short relayStatus;        //0b->rMainRelay, 1b->yMainRelay, 2b->bMainRelay, 3b->ryAuxRelay, 4b->yrAuxRelay, 5b->ybAuxRelay, 6b->byAuxRelay, 7b->rbAuxRelay, 8b->brAuxRelay, 9b->upsMainRelay, 10b->upsAuxRelay, 11,12,13,14,15b->NILL.
uint8_t contactorStatus;          //0b->CF1, 1b->CF2, 2b->CF3, 3b->CF4, 4b->CF5,5b->CF6, 6b->CF7, 7b->NILL

volatile bool healthyRphase,healthyYphase,healthyBphase,healthyUPS;
volatile bool rMainRelayState, yMainRelayState, bMainRelayState, ryAuxRelayState, yrAuxRelayState, ybAuxRelayState, byAuxRelayState, rbAuxRelayState, brAuxRelayState, upsMainRelayState, upsAuxRelayState;


uint8_t Uart3RXbuffer[BUF_SIZE];
typedef struct PhaseProtection {
    bool enabled;
    float highV;   // Volts
    float lowV;    // Volts
    float highI;   // Amps (overload)
}PhaseProtection;


PhaseProtection prot[4];   // 0..2 = R/Y/B, 3 = UPS (Inverter)
uint8_t headerProtection = 0XCC, checkSum;
bool protCheckSumMismatch;


float voltLow[4],voltHigh[4],overload[4];
float upsPowerLimit = DEFAULT_POWER_LIMIT;   // W, received from ESP (protection frame)
float powerLimit4 = DEFAULT_POWER_LIMIT;     // W, limit actually applied to the UPS
uint32_t overloadAlertL1StartTime,  overloadAlertL2StartTime, overloadAlertL3StartTime, overloadAlertL4StartTime, overPowerAlertL4StartTime;
bool overloadAlertL1,overloadAlertL2,overloadAlertL3, overloadAlertL4, overPowerAlertL4;
uint32_t resetButtonCombTScap;
bool resumel1,resumel2,resumel3,resumel4; //not used until overload reset is activated



uint64_t receivedUnixTime;
volatile static bool timeTobeSynched = false;
float bateryVoltage;


volatile uint32_t  eventFlag, previousEventFlag;   //eventFlag- 0b->R on/off, 1b->Y on/off, 2b->B on/off, 3b->YRaux on/off, 4b->BRaux on/off, 5b->RYaux on/off, 6b->BYaux on/off, 7b->RBaux on/off, 8b->YBaux on/off, 9b->UPS on/off, 10b->UPSaux on/off, 11b->overloadR on/off, 12b->overloadY on/off, 13b->overloadB on/off, 14b->overloadUPS on/off, 15b-> overPowerConsumptionUPS, 16n->STMrestart
volatile bool eventPassed;
typedef struct event{
	time_t carryEventTime;
	uint32_t carryEventFlag;
}event;

event arrEvent[20];
uint8_t headerEventSend = 0XEE;
uint8_t structTemp[sizeof(arrEvent[0])+2];

int8_t eventWriteIndex = -1;
uint32_t preSentTime;
time_t  curTime;
float avgPhases, preAvgPhases;

bool rCfState,yCfState,bCfState,ryCfState,ybCfState,brCfState,invCfState;
volatile uint32_t rCfHighTime,yCfHighTime,bCfHighTime,ryCfHighTime,ybCfHighTime,brCfHighTime,invCfHighTime, rCfLowTime,yCfLowTime,bCfLowTime,ryCfLowTime,ybCfLowTime,brCfLowTime,invCfLowTime;


RTC_TimeTypeDef rtcTime;
RTC_DateTypeDef rtcDate;

enum contactorStatus{
	SAFE,
	WARNING,
	MALFUNCTION,
	DANGER
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void UARTcollection(uint16_t Size);
void NTPsyncTimeToRTC(uint16_t Size);
void storingProtectionValue(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//int __io_putchar(int ch)
//{
//    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//    return ch;
//}

/* No debug UART on the controller board (PA2/PA3 are Y_ZX/B_ZX): printf is discarded.
   Must stay defined - syscalls.c calls this weak symbol and jumps to 0 if it is missing */



/* Protection frame from ESP: [0]=0xCC, then 13 bytes per phase (L1,L2,L3,UPS/Inverter):
   en(1) lowV(4) highV(4) highI(4); then [53..56] = UPS power limit (float W);
   [57] = ~(sum of bytes 1..56). Total 58 bytes. */
#define PROT_PHASES    4
#define PROT_POWER_OFS (1 + 13 * PROT_PHASES)
#define PROT_FRAME_LEN (PROT_POWER_OFS + 4 + 1)






void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART3) {
			otaStartRxIncomming(Uart3RXbuffer, Size); // <-- add: catch AA55A55A / BB44B44B
			NTPsyncTimeToRTC(Size);
			UARTcollection(Size);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Uart3RXbuffer, BUF_SIZE);
			__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);

	}

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		/* ESP reset/boot noise causes FE/NE/ORE, which aborts DMA reception - restart it */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Uart3RXbuffer, BUF_SIZE);
		__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
	}
}



void UARTcollection(uint16_t Size) {
	for (uint16_t i = 0; i + PROT_FRAME_LEN <= Size; i++) {
		if (Uart3RXbuffer[i] != headerProtection)
			continue;

		const uint8_t *f = &Uart3RXbuffer[i];   // frame start (not always buffer[0])
		uint8_t sum = 0;
		for (int k = 1; k < PROT_FRAME_LEN - 1; k++)
			sum += f[k];
		checkSum = (uint8_t) ~sum;

		if (checkSum == f[PROT_FRAME_LEN - 1]) {
			for (int ph = 0; ph < PROT_PHASES; ph++) {
				const uint8_t *p = &f[1 + 13 * ph];
				prot[ph].enabled = p[0];
				memcpy(&prot[ph].lowV,  &p[1], 4);
				memcpy(&prot[ph].highV, &p[5], 4);
				memcpy(&prot[ph].highI, &p[9], 4);
			}
			memcpy(&upsPowerLimit, &f[PROT_POWER_OFS], 4);
			storingProtectionValue();
			return;
		}
		protCheckSumMismatch = true;   // 0xCC seen but checksum bad: keep scanning
	}

}

void NTPsyncTimeToRTC(uint16_t Size){
//	printf("Time Synced process is in testing mode\r\n");
//	for (int i = 0; i < 10; i++) {
//		printf("%d ---->%X\r\n", i, Uart3RXbuffer[i]);
//	}
//	printf("\r\n");

	uint16_t i;
	for (i = 0; i < Size/*-i>=10*/; i++) {
		if (Uart3RXbuffer[i] == 0XDD && (Size - i) >= 10) {
				receivedUnixTime =
			    ((uint64_t)Uart3RXbuffer[i+1] << 0)  |
			    ((uint64_t)Uart3RXbuffer[i+2] << 8)  |
			    ((uint64_t)Uart3RXbuffer[i+3] << 16) |
			    ((uint64_t)Uart3RXbuffer[i+4] << 24) |
			    ((uint64_t)Uart3RXbuffer[i+5] << 32) |
			    ((uint64_t)Uart3RXbuffer[i+6] << 40) |
			    ((uint64_t)Uart3RXbuffer[i+7] << 48) |
			    ((uint64_t)Uart3RXbuffer[i+8] << 56);
			timeTobeSynched =true;
//			printf("%lu\r\n",(unsigned long)receivedUnixTime);
			return;
		}
	}
	return;


//	time_t epoch = receivedUinxTime;
//	struct tm* time = gmtime(&epoch);



}



uint8_t validateChecksum(uint8_t addr, uint8_t high, uint8_t mid, uint8_t low, uint8_t checksum){
	return checksum == (uint8_t) ~((0x55+addr+high+mid+low)&0XFF);

}

/* BL0939 has no chip select: 6 bytes of 0xFF re-aligns its SPI frame counter */
void blResetSPI(void){
	uint8_t resetBytes[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	HAL_SPI_Transmit(&hspi1, resetBytes, 6, HAL_MAX_DELAY);
}

/* Reads one BL0939 register as a single 6-byte frame, retrying on checksum failure.
   data[] receives {H, M, L, checksum}; returns 1 when the checksum matched */
uint8_t blReadReg(uint8_t addr, uint8_t *data){
	uint8_t tx[2] = {0x55, addr };
	uint8_t rx[6];

	for (uint8_t attempt = 0; attempt < 3; attempt++) {
		if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 6, HAL_MAX_DELAY) == HAL_OK
				&& validateChecksum(addr, rx[2], rx[3], rx[4], rx[5])) {
			for (uint8_t i = 0; i < 4; i++)
				data[i] = rx[i + 2];
			return 1;
		}
//		printf("BL %02X bad: %02X %02X %02X %02X %02X %02X\r\n",
//				addr, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5]);
		blResetSPI();
	}
//	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	return 0;
}



void batteryCalculation(void) {
	/*Process of monitoring the battery percentage with the help of inbuilt ADC channel connection */
	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		uint16_t bateryADCvalue = HAL_ADC_GetValue(&hadc1);
		bateryVoltage = ((float) ((bateryADCvalue * 3) * 3.6) / 4095);
	}

	HAL_ADC_Stop(&hadc1);
	/*End of this individual function*/

}

void overloadResetHandler(void) {
	/*Reset overload combination*/

	if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)
			&& HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)) {
		if (!resetButtonCombTScap) {
			resetButtonCombTScap = HAL_GetTick();
		}
	} else {
		resetButtonCombTScap = 0;
	}

	if (resetButtonCombTScap && (HAL_GetTick() - resetButtonCombTScap > RESET_BUT_COMB_PRESS_TIME)) {

		if (overloadAlertL1) {
			resumel1 = true;
		}
		if (overloadAlertL2) {
			resumel2 = true;
		}
		if (overloadAlertL3) {
			resumel3 = true;
		}
		if (overloadAlertL4 || overPowerAlertL4) {
			resumel4 = true;
		}
	}

	/*End of this individual function*/

}

void SyncTimeToSTMrtc(void){
 	 /* Process of Time sync when the ESP time get synced or at login. Then continiously running the RTC in the background with the help of the Vbat support */

	if (timeTobeSynched == 1) {
		time_t setUnixTime = (time_t) (receivedUnixTime + 19800);
		struct tm time;
		gmtime_r(&setUnixTime, &time);
		rtcTime.Hours = time.tm_hour;
		rtcTime.Minutes = time.tm_min;
		rtcTime.Seconds = time.tm_sec;
		rtcTime.SubSeconds = 0x0;
		rtcTime.TimeFormat = RTC_HOURFORMAT12_AM;
		rtcTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		rtcTime.StoreOperation = RTC_STOREOPERATION_RESET;

		rtcDate.WeekDay = time.tm_wday;
		rtcDate.Month = time.tm_mon + 1;
		rtcDate.Date = time.tm_mday;
		rtcDate.Year = ((time.tm_year) - 100);  // (126 - 100) -> 26  ->2026

		HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
		timeTobeSynched = false;

	}

	/*End of this individual function*/
}

void convRTCtimeToUNIXtime(void){
	/*This portion will help to take the time from internal RTC and shows in the terminal and convert to unix time for passing that to ESP*/
		struct tm time;
			HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
	//		printf("Time: %02d:%02d:%02d\r\n", rtcTime.Hours, rtcTime.Minutes,rtcTime.Seconds);
	//		printf("Date: %02d/%02d/20%02d\r\n", rtcDate.Date, rtcDate.Month,rtcDate.Year);

			time.tm_hour = rtcTime.Hours;
			time.tm_min = rtcTime.Minutes;
			time.tm_sec = rtcTime.Seconds;
			time.tm_wday = rtcDate.WeekDay;
			time.tm_mon = rtcDate.Month - 1;
			time.tm_mday = rtcDate.Date;
			time.tm_year = rtcDate.Year + 100;

			time.tm_isdst = 0;

			curTime = mktime(&time);


	/*End of this individual function*/

}

void BLdataCollectionL1(void) {
	/*----------------------------------------L1------------------------------------------------------*/

	HAL_GPIO_WritePin(R_sel_GPIO_Port, R_sel_Pin, SET);
//    			HAL_Delay(2);
	blResetSPI();

//	  			printf("L1 -Dum\r\n");
//	  			blReadReg(0x03,rxDummy);
//	  			printf("L1 -v\r\n");
	uint8_t okV1 = blReadReg(0x06, rxDataV1);
//	  			printf("L1 -c\r\n");
	uint8_t okC1 = blReadReg(0x04, rxDataC1);
//	  			printf("L1 -p\r\n");
	uint8_t okP1 = blReadReg(0x08, rxDataP1);
//	  			printf("L1 -e\r\n");
	uint8_t okE1 = blReadReg(0x0A, rxDataE1);

	//HAL_Delay(1000);
	HAL_GPIO_WritePin(R_sel_GPIO_Port, R_sel_Pin, RESET);

//	  			printf("data1-> %X, %X, %X, %X\r\n", rxDataV1[0], rxDataV1[1], rxDataV1[2],rxDataV1[3]);

	if (okV1) {
		voltL1 = ((rxDataV1[0] << 16 | rxDataV1[1] << 8 | rxDataV1[2])
				/ CALIB_VOL1);
		uartGoFlag = uartGoFlag | 0X0001;
		if (voltL1 <= 5) {
			voltL1 = 0;
		}
//	  				printf("Vol1 = %.2f\r\n", voltL1);

	}
	if (okC1) {
		curL1 = ((rxDataC1[0] << 16 | rxDataC1[1] << 8 | rxDataC1[2])
				/ CALIB_CUR1);
		uartGoFlag = uartGoFlag | 0X0002;
//	  				printf("curL1 = %.3f \r\n", curL1);
		if (curL1 < 0.03) {
			curL1 = 0;
			powL1 = 0;
		}

	}
	if (okP1) {
		powL1 = ((rxDataP1[0] << 16 | rxDataP1[1] << 8 | rxDataP1[2])
				/ CALIB_POW1);
		uartGoFlag = uartGoFlag | 0X0004;
//	  				printf("powL1 = %.2f \r\n", powL1);

	}
	if (okE1) {
		eneL1 = ((rxDataE1[0] << 16 | rxDataE1[1] << 8 | rxDataE1[2])
				/ CALIB_ENE1);
		uartGoFlag = uartGoFlag | 0X0008;
//	  				printf("eneL1 = %.3f \r\n", eneL1);

	}

//	  			printf("\r\n");

}

void BLdataCollectionL2(void) {
	/*----------------------------------------L2------------------------------------------------------*/

	HAL_GPIO_WritePin(Y_sel_GPIO_Port, Y_sel_Pin, SET);
	//	  			HAL_Delay(2);
	blResetSPI();
	//	  			HAL_Delay(20);

	//	  			printf("L2 -Dum\r\n");
	//	  			blReadReg(0x03,rxDummy);
	//	  			printf("L2 -v\r\n");
	uint8_t okV2 = blReadReg(0x06, rxDataV2);
	//	  			printf("L2 -c\r\n");
	uint8_t okC2 = blReadReg(0x04, rxDataC2);
	//	  			printf("L2 -p\r\n");
	uint8_t okP2 = blReadReg(0x08, rxDataP2);
	//	  			printf("L2 -E\r\n");
	uint8_t okE2 = blReadReg(0x0A, rxDataE2);

	HAL_GPIO_WritePin(Y_sel_GPIO_Port, Y_sel_Pin, RESET);
	//	  			HAL_Delay(100);
	//	  			printf("data2-> %X, %X, %X, %X\r\n", rxDataV2[0], rxDataV2[1], rxDataV2[2],rxDataV2[3]);

	if (okV2) {
		voltL2 = ((rxDataV2[0] << 16 | rxDataV2[1] << 8 | rxDataV2[2])
				/ CALIB_VOL2);
		uartGoFlag = uartGoFlag | 0X0010;
		//	  				printf("Vol2 = %.2f\r\n", voltL2);
		if (voltL2 <= 5) {
			voltL2 = 0;
		}
	}
	if (okC2) {
		curL2 = ((rxDataC2[0] << 16 | rxDataC2[1] << 8 | rxDataC2[2])
				/ CALIB_CUR2);
		uartGoFlag = uartGoFlag | 0X0020;
		//	  				printf("curL2 = %.3f \r\n", curL2);
		if (curL2 < 0.03) {
			curL2 = 0;
			powL2 = 0;
		}

	}
	if (okP2) {
		powL2 = ((rxDataP2[0] << 16 | rxDataP2[1] << 8 | rxDataP2[2])
				/ CALIB_POW2);
		uartGoFlag = uartGoFlag | 0X0040;
		//	  				printf("powL2 = %.2f \r\n", powL2);
	}
	if (okE2) {
		eneL2 = ((rxDataE2[0] << 16 | rxDataE2[1] << 8 | rxDataE2[2])
				/ CALIB_ENE2);
		uartGoFlag = uartGoFlag | 0X0080;
		//	  				printf("eneL2 = %.3f \r\n", eneL2);

	}

	//	  			printf("\r\n");
	


}

void BLdataCollectionL3(void) {
	/*----------------------------------------L3------------------------------------------------------*/

	HAL_GPIO_WritePin(B_sel_GPIO_Port, B_sel_Pin, SET);
//	HAL_Delay(2);
	blResetSPI();

//	  			printf("L3 -Dum\r\n");
//	  			blReadReg(0x03,rxDummy);
//	  			printf("L3 -v\r\n");
	uint8_t okV3 = blReadReg(0x06, rxDataV3);
//	  			printf("L3 -c\r\n");
	uint8_t okC3 = blReadReg(0x04, rxDataC3);
//	  			printf("L3 -p\r\n");
	uint8_t okP3 = blReadReg(0x08, rxDataP3);
//	  			printf("L3 -e\r\n");
	uint8_t okE3 = blReadReg(0x0A, rxDataE3);

	HAL_GPIO_WritePin(B_sel_GPIO_Port, B_sel_Pin, RESET);
//	  			printf("data3-> %X, %X, %X, %X\r\n", rxDataV3[0], rxDataV3[1], rxDataV3[2], rxDataV3[3]);

	if (okV3) {
		voltL3 = ((rxDataV3[0] << 16 | rxDataV3[1] << 8 | rxDataV3[2])
				/ CALIB_VOL3);
		uartGoFlag = uartGoFlag | 0X0100;
//	  				printf("Vol3 = %.2f\r\n", voltL3);
		if (voltL3 <= 5) {
			voltL3 = 0;
		}
	}
	if (okC3) {
		curL3 = ((rxDataC3[0] << 16 | rxDataC3[1] << 8 | rxDataC3[2])
				/ CALIB_CUR3);
		uartGoFlag = uartGoFlag | 0X0200;
//	  			printf("curL3 = %.3f \r\n", curL3);
		if (curL3 < 0.03) {
			curL3 = 0;
			powL3 = 0;
		}
	}
	if (okP3) {
		powL3 = ((rxDataP3[0] << 16 | rxDataP3[1] << 8 | rxDataP3[2])
				/ CALIB_POW3);
		uartGoFlag = uartGoFlag | 0X0400;
//	  			printf("powL3 = %.2f \r\n", powL3);
	}
	if (okE3) {
		eneL3 = ((rxDataE3[0] << 16 | rxDataE3[1] << 8 | rxDataE3[2])
				/ CALIB_ENE3);
		uartGoFlag = uartGoFlag | 0X0800;
//	  			printf("eneL3 = %.3f \r\n", eneL3);
	}
//	  			printf("\r\n");

}

void BLdataCollectionL4(void) {

	/*----------------------------------------L4------------------------------------------------------*/
	HAL_GPIO_WritePin(UPS_sel_GPIO_Port, UPS_sel_Pin, SET);
//	 HAL_Delay(2);
	blResetSPI();

//	  			printf("L4 -Dum\r\n");
//	  			blReadReg(0x03,rxDummy);
//	  			printf("L4 -v\r\n");
	uint8_t okV4 = blReadReg(0x06, rxDataV4);
//	  			printf("L4 -c\r\n");
	uint8_t okC4 = blReadReg(0x04, rxDataC4);
//	  			printf("L4 -p\r\n");
	uint8_t okP4 = blReadReg(0x08, rxDataP4);
//	  			printf("L4 -e\r\n");
	uint8_t okE4 = blReadReg(0x0A, rxDataE4);

	HAL_GPIO_WritePin(UPS_sel_GPIO_Port, UPS_sel_Pin, RESET);
//	  			printf("data3-> %X, %X, %X, %X\r\n", rxDataV3[0], rxDataV3[1], rxDataV3[2], rxDataV3[3]);

	if (okV4) {
		voltL4 = ((rxDataV4[0] << 16 | rxDataV4[1] << 8 | rxDataV4[2])
				/ CALIB_VOL4);
		uartGoFlag = uartGoFlag | 0X1000;
//	  				printf("Vol4 = %.2f\r\n", voltL4);
		if (voltL4 <= 5) {
			voltL4 = 0;
		}
	}
	if (okC4) {
		curL4 = ((rxDataC4[0] << 16 | rxDataC4[1] << 8 | rxDataC4[2])
				/ CALIB_CUR4);
		uartGoFlag = uartGoFlag | 0X2000;
//	  			printf("curL4 = %.3f \r\n", curL4);
		if (curL4 < 0.03) {
			curL4 = 0;
			powL4 = 0;
		}
	}
	if (okP4) {
		powL4 = ((rxDataP4[0] << 16 | rxDataP4[1] << 8 | rxDataP4[2])
				/ CALIB_POW4);
		uartGoFlag = uartGoFlag | 0X4000;
//	  			printf("powL4 = %.2f \r\n", powL4);
	}
	if (okE4) {
		eneL4 = ((rxDataE4[0] << 16 | rxDataE4[1] << 8 | rxDataE4[2])
				/ CALIB_ENE4);
		uartGoFlag = uartGoFlag | 0X8000;
//	  			printf("eneL4 = %.3f \r\n", eneL4);
	}
//	  			printf("\r\n");

}

void storingProtectionValue(){
	/*Protection Settings assigning condition with hiding noise of volt and current*/

			if (prot[0].enabled) {
				voltLow[0] = prot[0].lowV;
				voltHigh[0] = prot[0].highV;
				overload[0] = prot[0].highI;
			} else {
				voltLow[0] = DEFAULT_LOWVOLTAGE_LIMIT;
				voltHigh[0] = DEFAULT_HIGHVOLTAGE_LIMIT;
				overload[0] = DEFAULT_OVERLOAD_LIMIT;
			}
			if (prot[1].enabled) {
				voltLow[1] = prot[1].lowV;
				voltHigh[1] = prot[1].highV;
				overload[1] = prot[1].highI;
			} else {
				voltLow[1] = DEFAULT_LOWVOLTAGE_LIMIT;
				voltHigh[1] = DEFAULT_HIGHVOLTAGE_LIMIT;
				overload[1] = DEFAULT_OVERLOAD_LIMIT;
			}
			if (prot[2].enabled) {
				voltLow[2] = prot[2].lowV;
				voltHigh[2] = prot[2].highV;
				overload[2] = prot[2].highI;
			} else {
				voltLow[2] = DEFAULT_LOWVOLTAGE_LIMIT;
				voltHigh[2] = DEFAULT_HIGHVOLTAGE_LIMIT;
				overload[2] = DEFAULT_OVERLOAD_LIMIT;
			}

			if (prot[3].enabled) {          // UPS / Inverter limits from the ESP
				voltLow[3] = prot[3].lowV;
				voltHigh[3] = prot[3].highV;
				overload[3] = prot[3].highI;
				powerLimit4 = upsPowerLimit;
			} else {
				voltLow[3] = DEFAULT_LOWVOLTAGE_LIMIT;
				voltHigh[3] = DEFAULT_HIGHVOLTAGE_LIMIT;
				overload[3] = DEFAULT_OVERLOAD_LIMIT;
				powerLimit4 = DEFAULT_POWER_LIMIT;
			}

		/*End of this individual function*/

}

void phaseHealthCheck(){

	/*Phase health check */
	healthyRphase = ((voltL1 > voltLow[0]) && (voltL1 < voltHigh[0])) ? 1 : 0;

	healthyYphase = ((voltL2 > voltLow[1]) && (voltL2 < voltHigh[1])) ? 1 : 0;

	healthyBphase = ((voltL3 > voltLow[2]) && (voltL3 < voltHigh[2])) ? 1 : 0;

	healthyUPS = ((voltL4 > voltLow[3]) && (voltL4 < voltHigh[3])) ? 1 : 0;

	/*End of this individual function*/

}

void overloadMonitoring(){

	/*Phase overload check */

			if (healthyRphase && (curL1 > overload[0])) {
				if (!overloadAlertL1StartTime) {
					overloadAlertL1StartTime = HAL_GetTick();
				}
				if (overloadAlertL1StartTime && (HAL_GetTick() - overloadAlertL1StartTime >= FAULT_DELAY)) {
					overloadAlertL1 = true;

					//This is event noting to manage the logs
					eventFlag = (eventFlag | (1 << 11));
				}
			} else {
				overloadAlertL1StartTime = 0;
			}

			if (healthyYphase && (curL2 > overload[1])) {
				if (!overloadAlertL2StartTime) {
					overloadAlertL2StartTime = HAL_GetTick();
				}
				if (overloadAlertL2StartTime && (HAL_GetTick() - overloadAlertL2StartTime >= FAULT_DELAY)) {
					overloadAlertL2 = true;

					//This is event noting to manage the logs
					eventFlag = (eventFlag | (1 << 12));
				}
			} else {
				overloadAlertL2StartTime = 0;
			}

			if (healthyBphase && (curL3 > overload[2])) {
				if (!overloadAlertL3StartTime) {
					overloadAlertL3StartTime = HAL_GetTick();
				}

				if (overloadAlertL3StartTime && (HAL_GetTick() - overloadAlertL3StartTime >= FAULT_DELAY)) {
					overloadAlertL3 = true;

					//This is event noting to manage the logs
					eventFlag = (eventFlag | (1 << 13));
				}
			} else {
				overloadAlertL3StartTime = 0;
			}

			if(healthyUPS && (curL4 > overload[3])){
				if(!overloadAlertL4StartTime){
					overloadAlertL4StartTime = HAL_GetTick();
				}

				if(overloadAlertL4StartTime && (HAL_GetTick() - overloadAlertL4StartTime >=  FAULT_DELAY)) {
					overloadAlertL4 = true;

					//This is event noting to manage the logs
					eventFlag = (eventFlag | (1 << 14));

				}
			}else{
				overloadAlertL4StartTime = 0;
			}

			if(healthyUPS && (powL4 > powerLimit4)){
				if(!overPowerAlertL4StartTime){
					overPowerAlertL4StartTime = HAL_GetTick();
				}

				if(overPowerAlertL4StartTime && (HAL_GetTick() - overPowerAlertL4StartTime >= FAULT_DELAY)){
					overPowerAlertL4 = true;

					//to do; need to add the log further
					eventFlag = (eventFlag | (1 << 15));
				}
			}else{
				overPowerAlertL4StartTime = 0;
			}

			/*Resume to recover the relay State from the overload alert*/
			//Resume for the Recovery from overload of all phases.After remove this comment once you done the recovery process
			if (overloadAlertL1 && resumel1) {
				overloadAlertL1 = false;
				resumel1 = 0;
				overloadAlertL1StartTime = 0;
				/*This is event noting to manage the logs*/
				eventFlag = (eventFlag & (~(1 << 11)));
			}

			if (overloadAlertL2 && resumel2) {
				overloadAlertL2 = false;
				resumel2 = 0;
				overloadAlertL2StartTime = 0;
				/*This is event noting to manage the logs*/
				eventFlag = (eventFlag & (~(1 << 12)));
			}

			if (overloadAlertL3 && resumel3) {
				overloadAlertL3 = false;
				resumel3 = 0;
				overloadAlertL3StartTime = 0;
				/*This is event noting to manage the logs*/
				eventFlag = (eventFlag & (~(1 << 13)));
			}
			if ((overloadAlertL4 || overPowerAlertL4) && resumel4) {

				/*This is event noting to manage the logs*/
				if (overloadAlertL4) {
					eventFlag = (eventFlag & (~(1 << 14)));
				}
				if(overPowerAlertL4){
					eventFlag = (eventFlag & (~(1 << 15)));
				}
				// Need to implement the log for the over power of UPS
				overloadAlertL4 = false;
				overPowerAlertL4 = false;
				resumel4 = 0;
				overloadAlertL4StartTime = 0;
				overPowerAlertL4StartTime = 0;

			}
		/*End of this individual function*/

}

void SwitchoverManagement(){
	/*Switch-over Management*/


					if (!healthyRphase || overloadAlertL1) {
						if (rMainRelayState || rCfState) {
							// Function to OFF the relay once, inside it should OFF the Aux of the R main relay
							rMainRelayTurnOff();
							// give some delay if not feedback
	//						HAL_Delay(500);

						}else if(overloadAlertL1 && ((ryCfState || yrAuxRelayState) || (brCfState || brAuxRelayState))){
							rIncomingAuxRelayOff();
						}
						if (healthyYphase && healthyBphase && !yrAuxRelayState
								&& !brAuxRelayState && !overloadAlertL1 && !rCfState && !ryCfState && !brCfState  && !invCfState) {		//prevention condition
							// choose any one of the phase with the help of the current
							// with that phase ON the the Aux to the the R phase once
							/*depends on the current Aux will switch over*/
							if ((curL2 <= curL3) && !rCfState && !ryCfState && !brCfState  && !invCfState) {
								yrAuxRelayTurnOn();


							} else if ((curL3 < curL2) && !rCfState && !ryCfState && !brCfState  && !invCfState) {
								brAuxRelayTurnOn();

							}
							// else {
							// 	yrAuxRelayTurnOn();
							// }
						} else if(!overloadAlertL1) {
							if (healthyYphase && !yrAuxRelayState && !brAuxRelayState && !rCfState && !ryCfState && !brCfState  && !invCfState) {
								// ON the Y phase Aux to the R main once
								yrAuxRelayTurnOn();


							} else if (healthyBphase && !yrAuxRelayState
									&& !brAuxRelayState && !rCfState && !ryCfState && !brCfState  && !invCfState) {
								// ON the B phase Aux to the R main once
								brAuxRelayTurnOn();

							} else {
								//not to changeover the relay
							}
						}

					} else if(healthyRphase && !overloadAlertL1) {
						if (!rMainRelayState && !rCfState && !invCfState) {
							if (yrAuxRelayState || brAuxRelayState || ryCfState || brCfState) {
								// function to OFF the all Aux from other phase to R phases
								rIncomingAuxRelayOff();
	//							HAL_Delay(500);
							}

							// give some delay if not feedback
							// Turn ON the R main relay
//							if(!ryCfState && !brCfState ){
							if((!ryCfState && !yrAuxRelayState) && (!brCfState && !brAuxRelayState)){
							rMainRelayTurnOn();
							}

						}

					}

					if (!healthyYphase || overloadAlertL2) {
						if (yMainRelayState || yCfState) {
							// Function to OFF the relay once, inside it should OFF the Aux of the Y main relay
							yMainRelayTurnOff();
							// give some delay if not feedback
	//						HAL_Delay(500);

						}else if(overloadAlertL2 &&((ryCfState || ryAuxRelayState) || (ybCfState || byAuxRelayState))){
							yIncomingAuxRelayOff();
						}
						if (healthyRphase && healthyBphase && !ryAuxRelayState
								&& !byAuxRelayState && !overloadAlertL2  && !yCfState && !ryCfState && !ybCfState  && !invCfState) { 		//prevention condition
							// choose any one of the phase with the help of the current
							// with that phase ON the the Aux to the the R phase once
							/*depends on the current Aux will switch over*/
							if ((curL1 <= curL3) && !yCfState && !ryCfState && !ybCfState  && !invCfState) {
								ryAuxRelayTurnOn();
							} else if ((curL3 < curL1) && !yCfState && !ryCfState && !ybCfState  && !invCfState) {
								byAuxRelayTurnOn();
							}
//							else {
//								ryAuxRelayTurnOn();
//							}

						} else if(!overloadAlertL2){
							if (healthyRphase && !ryAuxRelayState && !byAuxRelayState && !yCfState && !ryCfState && !ybCfState  && !invCfState) {
								// ON the Y phase Aux to the R main once
								ryAuxRelayTurnOn();
							} else if (healthyBphase && !ryAuxRelayState
									&& !byAuxRelayState && !yCfState && !ryCfState && !ybCfState  && !invCfState) {
								// ON the B phase Aux to the R main once
								byAuxRelayTurnOn();
							} else {
								//not to change over the relay
							}

						}

					} else if(healthyYphase && !overloadAlertL2) {
						if (!yMainRelayState && !yCfState && !invCfState) {
							if (ryAuxRelayState || byAuxRelayState || ryCfState || ybCfState) {
								// function to OFF the all Aux from other phase to Y phases
								yIncomingAuxRelayOff();
								// give some delay if not feedback
	//							HAL_Delay(500);
							}
							// Turn ON the Y main relay
//							if(!ryCfState && !ybCfState){
							if((!ryCfState && !ryAuxRelayState) && (!ybCfState && !byAuxRelayState)){
							yMainRelayTurnOn();
							}



						}
					}

					if (!healthyBphase || overloadAlertL3) {
						if (bMainRelayState || bCfState) {
							// Function to OFF the relay once, inside it should OFF the Aux of the B main relay
							bMainRelayTurnOff();
							// give some delay if not feedback
	//						HAL_Delay(500);

						}else if(overloadAlertL3 && ((brCfState || rbAuxRelayState) || (ybCfState || ybAuxRelayState))){
							bIncomingAuxRelayOff();
						}
						if (healthyRphase && healthyYphase && !rbAuxRelayState
								&& !ybAuxRelayState && !overloadAlertL3  && !bCfState && !brCfState && !ybCfState  && !invCfState) {		//prevention condition
							// choose any one of the phase with the help of the current
							// with that phase ON the the Aux to the the R phase once
							/*depends on the current Aux will switch over*/
							if ((curL1 <= curL2) && !bCfState && !brCfState && !ybCfState  && !invCfState) {
								rbAuxRelayTurnOn();
							} else if ((curL2 < curL1) && !ybAuxRelayState  && !bCfState && !brCfState && !ybCfState  && !invCfState) {
								ybAuxRelayTurnOn();
							}
//							else {
//								rbAuxRelayTurnOn();
//							}


						} else if(!overloadAlertL3) {
							if (healthyRphase && !rbAuxRelayState && !ybAuxRelayState  && !bCfState && !brCfState && !ybCfState  && !invCfState) {
								// ON the Y phase Aux to the R main once
								rbAuxRelayTurnOn();
							} else if (healthyYphase && !rbAuxRelayState
									&& !ybAuxRelayState  && !bCfState && !brCfState && !ybCfState  && !invCfState) {
								// ON the B phase Aux to the R main once
								ybAuxRelayTurnOn();
							} else {
								//not to change over the relay
							}

						}
					} else if(healthyBphase && !overloadAlertL3) {
						if (!bMainRelayState && !bCfState && !invCfState) {
							if (rbAuxRelayState || ybAuxRelayState || brCfState || ybCfState) {
								// function to OFF the all Ax from other phase to B phases
								bIncomingAuxRelayOff();
								// give some delay if not feedback
	//							HAL_Delay(500);
							}
							// Turn ON the B main relay
//							if(!brCfState && !ybCfState){
							if((!brCfState && !rbAuxRelayState) && (!ybCfState && !ybAuxRelayState)){
							bMainRelayTurnOn();
							}


						}

					}

				if(healthyUPS && !overPowerAlertL4 && !overloadAlertL4 && !healthyRphase && !healthyYphase && !healthyBphase && !rMainRelayState && !yMainRelayState && !bMainRelayState && !rCfState && !yCfState && !bCfState && !ryCfState && !ybCfState && !brCfState){
					if(!invCfState){
						upsAuxRelayOff();
						upsMainRelayOn();
					}
				}else{
					upsMainRelayOff();
					upsAuxRelayOn();
				}

				/*End of this individual function*/


}



void feedbackHandler(){
	if(!(HAL_GPIO_ReadPin(CF1_GPIO_Port, CF1_Pin)) && !(rCfLowTime)){
		rCfHighTime = 0;
		rCfLowTime = HAL_GetTick();
	}else if(HAL_GPIO_ReadPin(CF1_GPIO_Port, CF1_Pin) && !(rCfHighTime)){
		rCfLowTime = 0;
		rCfHighTime = HAL_GetTick();
	}
		if(!(HAL_GPIO_ReadPin(CF2_GPIO_Port, CF2_Pin)) && !(yCfLowTime)){
		yCfHighTime = 0;
		yCfLowTime = HAL_GetTick();
	}else if(HAL_GPIO_ReadPin(CF2_GPIO_Port, CF2_Pin) && !(yCfHighTime)){
		yCfLowTime = 0;
		yCfHighTime = HAL_GetTick();
	}

	if(!(HAL_GPIO_ReadPin(CF3_GPIO_Port, CF3_Pin)) && !(bCfLowTime)){
		bCfHighTime = 0;
		bCfLowTime = HAL_GetTick();
	}else if(HAL_GPIO_ReadPin(CF3_GPIO_Port, CF3_Pin) && !(bCfHighTime)){
		bCfLowTime = 0;
		bCfHighTime = HAL_GetTick();
	}

	if(!(HAL_GPIO_ReadPin(CF4_GPIO_Port, CF4_Pin)) && !(ryCfLowTime)){
		ryCfHighTime = 0;
		ryCfLowTime = HAL_GetTick();
	}else if(HAL_GPIO_ReadPin(CF4_GPIO_Port, CF4_Pin) && !(ryCfHighTime)){
		ryCfLowTime = 0;
		ryCfHighTime = HAL_GetTick();
	}

	if(!(HAL_GPIO_ReadPin(CF5_GPIO_Port, CF5_Pin)) && !(ybCfLowTime)){
		ybCfHighTime = 0;
		ybCfLowTime = HAL_GetTick();
	}else if(HAL_GPIO_ReadPin(CF5_GPIO_Port, CF5_Pin) && !(ybCfHighTime)){
		ybCfLowTime = 0;
		ybCfHighTime = HAL_GetTick();
	}

	if(!(HAL_GPIO_ReadPin(CF6_GPIO_Port, CF6_Pin)) && !(brCfLowTime)){
		brCfHighTime = 0;
		brCfLowTime = HAL_GetTick();
	}else if(HAL_GPIO_ReadPin(CF6_GPIO_Port, CF6_Pin) && !(brCfHighTime)){
		brCfLowTime = 0;
		brCfHighTime = HAL_GetTick();
	}

	if(!(HAL_GPIO_ReadPin(CF7_GPIO_Port, CF7_Pin)) && !(invCfLowTime)){
		invCfHighTime = 0;
		invCfLowTime = HAL_GetTick();
	}else if(HAL_GPIO_ReadPin(CF7_GPIO_Port, CF7_Pin) && !(invCfHighTime)){
		invCfLowTime = 0;
		invCfHighTime = HAL_GetTick();
	}




	if (rCfLowTime && ((HAL_GetTick() - rCfLowTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		rCfState = true;
		contactorStatus |= 1<<0;
	} else if (rCfHighTime && ((HAL_GetTick() - rCfHighTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		rCfState = false;
		contactorStatus &= ~(1<<0);
	}


	if (yCfLowTime && ((HAL_GetTick() - yCfLowTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		yCfState = true;
		contactorStatus |= 1<<1;
	} else if (yCfHighTime && ((HAL_GetTick() - yCfHighTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		yCfState = false;
		contactorStatus &= ~(1<<1);
	}


	if (bCfLowTime && ((HAL_GetTick() - bCfLowTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		bCfState = true;
		contactorStatus |= 1<<2;
	} else if (bCfHighTime && ((HAL_GetTick() - bCfHighTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		bCfState = false;
		contactorStatus &= ~(1<<2);
	}


	if (ryCfLowTime && ((HAL_GetTick() - ryCfLowTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		ryCfState = true;
		contactorStatus |= 1<<3;
	} else if (ryCfHighTime && ((HAL_GetTick() - ryCfHighTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		ryCfState = false;
		contactorStatus &= ~(1<<3);
	}


	if (ybCfLowTime && ((HAL_GetTick() - ybCfLowTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		ybCfState = true;
		contactorStatus |= 1<<4;
	} else if (ybCfHighTime && ((HAL_GetTick() - ybCfHighTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		ybCfState = false;
		contactorStatus &= ~(1<<4);
	}


	if (brCfLowTime && ((HAL_GetTick() - brCfLowTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		brCfState = true;
		contactorStatus |= 1<<5;
	} else if (brCfHighTime && ((HAL_GetTick() - brCfHighTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		brCfState = false;
		contactorStatus &= ~(1<<5);
	}


	if (invCfLowTime && ((HAL_GetTick() - invCfLowTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		invCfState = true;
		contactorStatus |= 1<<6;
	} else if (invCfHighTime && ((HAL_GetTick() - invCfHighTime) >= CONTACTOR_DEBOUNCE_DELAY)) {
		invCfState = false;
		contactorStatus &= ~(1<<6);
	}
}



enum contactorStatus PhysicalFailFaultFinder() {


	/*To do: If physical contactor feedback failed with the help of the relay and contactor feedback state should be same*/
	

	/*To do: Charging Input of the battery is control by the relay, that relay need to OFF when Battery output is enable. */
	if((((rCfState && rMainRelayState) || (yCfState && yMainRelayState) || (bCfState && bMainRelayState)) && !upsAuxRelayState) || (upsMainRelayState && upsAuxRelayState)){
		return MALFUNCTION;
	}

	/*If Physical Contactor failure (short or weld happened)*/
	uint8_t conductingContactorCount = 0;
	for (int i = 0; i < 7; i++) {
		if ((contactorStatus & (1 << i))) {
			conductingContactorCount++;
		}
	}

	if (conductingContactorCount > 3) {
		return DANGER;
	} else if (conductingContactorCount == 3) {
		return SAFE;
	} else if (conductingContactorCount == 1 && upsMainRelayState && invCfState) {
		return SAFE;
	} else{
		return WARNING;
	}

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Uart3RXbuffer, BUF_SIZE);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);


  HAL_PWR_EnableBkUpAccess();


  HAL_ADC_Start(&hadc1);

//  HAL_Delay(5000);
//  HAL_GPIO_WritePin(R_sel_GPIO_Port, R_sel_Pin, RESET);
//  HAL_GPIO_WritePin(Y_sel_GPIO_Port, Y_sel_Pin, RESET);
//  HAL_GPIO_WritePin(B_sel_GPIO_Port, B_sel_Pin, RESET);
//  HAL_GPIO_WritePin(UPS_sel_GPIO_Port, UPS_sel_Pin, SET);


  /*The following assignment shows that the state of the contactor while MCU boot*/
  contactorStatus = 0;
  rCfState = !(GPIOB->IDR & CF1_Pin);
  contactorStatus |= rCfState<<0;
  yCfState = !(GPIOB->IDR & CF2_Pin);
  contactorStatus |= yCfState<<1;
  bCfState = !(GPIOB->IDR & CF3_Pin);
  contactorStatus |= bCfState<<2;
  ryCfState = !(GPIOB->IDR & CF4_Pin);
  contactorStatus |= ryCfState<<3;
  ybCfState = !(GPIOC->IDR & CF5_Pin);
  contactorStatus |= ybCfState<<4;
  brCfState = !(GPIOC->IDR & CF6_Pin);
  contactorStatus |= brCfState<<5;
  invCfState = !(GPIOC->IDR & CF7_Pin);
  contactorStatus |= invCfState<<6;

 voltLow[0] = DEFAULT_LOWVOLTAGE_LIMIT;
 voltHigh[0] = DEFAULT_HIGHVOLTAGE_LIMIT;
 overload[0] = DEFAULT_OVERLOAD_LIMIT;
 voltLow[1] = DEFAULT_LOWVOLTAGE_LIMIT;
 voltHigh[1] = DEFAULT_HIGHVOLTAGE_LIMIT;
 overload[1] = DEFAULT_OVERLOAD_LIMIT;
 voltLow[2] = DEFAULT_LOWVOLTAGE_LIMIT;
 voltHigh[2] = DEFAULT_HIGHVOLTAGE_LIMIT;
 overload[2] = DEFAULT_OVERLOAD_LIMIT;
 voltLow[3] = DEFAULT_LOWVOLTAGE_LIMIT;
 voltHigh[3] = DEFAULT_HIGHVOLTAGE_LIMIT;
 overload[3] = DEFAULT_OVERLOAD_LIMIT;
 powerLimit4 = DEFAULT_POWER_LIMIT;


//  This is event noting to manage the logs for restart
  eventFlag = (eventFlag | (1<<16));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*Flash process with custom boot loader*/
	  		otaIncomingCheck();       // <-- runs the (blocking) receive/apply when flagged(continuously monitor whether the ESP is sending the firmware for update or not! )
	  		if (otaUpdating())
	  			continue;      // <-- not obtain data from BL0939 and won't transmit metering  while updating the firmware

	  /*End of this individual function*/

	  	overloadResetHandler();			// <-- After clearing Overload fault physically, reset the fault flag in the STM

	  	batteryCalculation();			//<-- The Battery voltage monitoring by using the Vbat pin.


	  	/*These two must be in order sync then convert*/
	  	SyncTimeToSTMrtc();				//<-- After getting the ESP time. The time will be sync to the STM internal RTC.
	  	convRTCtimeToUNIXtime();		//<-- Converting RTC Time in to unix time, and send it to the ESP side to use it.
	  	/*These two must be in order sync then convert*/

	  	/*Data collecting from the BL0939 IC via SPI protocol communication for the L1 L2 L3*/
	  	BLdataCollectionL1();
	  	BLdataCollectionL2();
	  	BLdataCollectionL3();
	  	BLdataCollectionL4();



	  	feedbackHandler();

	  	phaseHealthCheck();

	  	overloadMonitoring();

	  	SwitchoverManagement();



	  	/*Data collecting from the BL0939 IC via SPI protocol communication for the L1 L2 L3*/







	  /*End of this individual function*/



		if ((HAL_GetTick() % 2000) <= 1000) {
			if (!HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin)) {
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			}
			if (HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin)) {
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
			}
		} else {

			if (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin)) {
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
			}
			if (!HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin)) {
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			}
		}













/*While sending the UART the values validate with the "uartGoFlag" variable and then go for it*/
		preAvgPhases = avgPhases;
		avgPhases = ((voltL1+voltL2+voltL3)/3);
		float avgVoltDiff = avgPhases - preAvgPhases;

		if(((HAL_GetTick() - preSentTime) >= 1000) || (avgVoltDiff >= (preAvgPhases/100.0)) || (avgVoltDiff <= (-(preAvgPhases/100.0)))){
			preSentTime=HAL_GetTick();

				if (uartGoFlag & 0XFFFF) {
					HAL_UART_Transmit(&huart3, (uint8_t*) "STM", 4, 10);
					uint8_t transmit[60];
					relayStatus = 0;
					relayStatus |= rMainRelayState << 0;
					relayStatus |= yMainRelayState << 1;
					relayStatus |= bMainRelayState << 2;
					relayStatus |= ryAuxRelayState << 3;
					relayStatus |= yrAuxRelayState << 4;
					relayStatus |= ybAuxRelayState << 5;
					relayStatus |= byAuxRelayState << 6;
					relayStatus |= rbAuxRelayState << 7;
					relayStatus |= brAuxRelayState << 8;
					relayStatus |= upsMainRelayState << 9;
					relayStatus |= upsAuxRelayState << 10;
					memcpy(transmit, rxDataV1, 3);
					memcpy((&transmit[3]), &rxDataV2, 3);
					memcpy((&transmit[6]), &rxDataV3, 3);
					memcpy((&transmit[9]), &rxDataV4,3);
					memcpy((&transmit[12]), &rxDataC1, 3);
					memcpy((&transmit[15]), &rxDataC2, 3);
					memcpy((&transmit[18]), &rxDataC3, 3);
					memcpy((&transmit[21]), &rxDataC4, 3);
					memcpy((&transmit[24]), &rxDataP1, 3);
					memcpy((&transmit[27]), &rxDataP2, 3);
					memcpy((&transmit[30]), &rxDataP3, 3);
					memcpy((&transmit[33]), &rxDataP4, 3);
					memcpy((&transmit[36]), &rxDataE1, 3);
					memcpy((&transmit[39]), &rxDataE2, 3);
					memcpy((&transmit[42]), &rxDataE3, 3);
					memcpy((&transmit[45]), &rxDataE4, 3);
					memcpy((&transmit[48]), &relayStatus, 2);
					memcpy((&transmit[50]), &bateryVoltage,4);
					memcpy((&transmit[54]), &curTime,4);
					memcpy((&transmit[58]), &contactorStatus,1);
					HAL_UART_Transmit(&huart3, (uint8_t*) &transmit, sizeof(transmit),10);
					uartGoFlag = 0;
				}

		}

				/*Acknowledge or Resent commends from STM to ESP via UART*/
		//		if (!(prot[0].highI) || protCheckSumMismatch ){
		//			HAL_UART_Transmit(&huart3,(uint8_t*)&headerProtection,1,10);
		//			protCheckSumMismatch = false;
		//		}
				/*End of this individual function*/


					/*Testing of the Event flags: According to the event to be set and to be reset  */

		if (previousEventFlag != eventFlag) {
			previousEventFlag = eventFlag;

			struct tm time;

			HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

			time.tm_hour = rtcTime.Hours;
			time.tm_min = rtcTime.Minutes;
			time.tm_sec = rtcTime.Seconds;
			time.tm_wday = rtcDate.WeekDay;
			time.tm_mon = rtcDate.Month - 1;
			time.tm_mday = rtcDate.Date;
			time.tm_year = rtcDate.Year + 100;

			time.tm_isdst = 0;

			time_t eventTime = mktime(&time);


			//Storing the event details in the Structure for the LOG purpose

			arrEvent[++eventWriteIndex].carryEventTime = eventTime;
			arrEvent[eventWriteIndex].carryEventFlag = eventFlag;



			if (eventWriteIndex >= 20) {
				eventWriteIndex = -1;
			}


		}


		if (eventWriteIndex >= 0) {
			memcpy(&structTemp[0], &headerEventSend, 1);
			memcpy(&structTemp[1], &arrEvent[0], sizeof(arrEvent[0]));
			HAL_UART_Transmit(&huart3, (uint8_t*) &structTemp,sizeof(structTemp), 100);

			for (uint8_t i = 0; i < eventWriteIndex; i++) {
				arrEvent[i] = arrEvent[i + 1];
			}
			eventWriteIndex--;


		}

		if (eventFlag & (1U << 16)) {
			// printf("Restart happened !\r\n");
			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1U << 16)));
		}

		/*End of testing*/





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
	if (RTC_BAT_CONNECTED)
		return;

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, EX_TAP1_SIG_Pin|EX_TAP2_SIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R_sel_Pin|B_sel_Pin|Y_sel_Pin|UPS_main_sig_Pin
                          |R_main_sig_Pin|R_AUX1_sig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UPS_sel_Pin|Y_main_sig_Pin|Y_AUX1_sig_Pin|Y_AUX2_sig_Pin
                          |B_main_sig_Pin|B_AUX1_sig_Pin|B_AUX2_sig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|LED1_Pin|UPS_AUX_sig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R_AUX2_sig_GPIO_Port, R_AUX2_sig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW2_Pin Digital_in_Pin R_ZX_Pin CF5_Pin
                           CF6_Pin CF7_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|Digital_in_Pin|R_ZX_Pin|CF5_Pin
                          |CF6_Pin|CF7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EX_TAP1_SIG_Pin EX_TAP2_SIG_Pin */
  GPIO_InitStruct.Pin = EX_TAP1_SIG_Pin|EX_TAP2_SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : R_LEAK_Pin Y_LEAK_Pin Y_ZX_Pin B_ZX_Pin
                           B_LEAK_Pin SW1_Pin */
  GPIO_InitStruct.Pin = R_LEAK_Pin|Y_LEAK_Pin|Y_ZX_Pin|B_ZX_Pin
                          |B_LEAK_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R_sel_Pin B_sel_Pin Y_sel_Pin UPS_main_sig_Pin
                           R_main_sig_Pin R_AUX1_sig_Pin */
  GPIO_InitStruct.Pin = R_sel_Pin|B_sel_Pin|Y_sel_Pin|UPS_main_sig_Pin
                          |R_main_sig_Pin|R_AUX1_sig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : UPS_sel_Pin Y_main_sig_Pin Y_AUX1_sig_Pin Y_AUX2_sig_Pin
                           B_main_sig_Pin B_AUX1_sig_Pin B_AUX2_sig_Pin */
  GPIO_InitStruct.Pin = UPS_sel_Pin|Y_main_sig_Pin|Y_AUX1_sig_Pin|Y_AUX2_sig_Pin
                          |B_main_sig_Pin|B_AUX1_sig_Pin|B_AUX2_sig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USP_LEAK_Pin UPS_ZX_Pin CF1_Pin CF2_Pin
                           CF3_Pin CF4_Pin */
  GPIO_InitStruct.Pin = USP_LEAK_Pin|UPS_ZX_Pin|CF1_Pin|CF2_Pin
                          |CF3_Pin|CF4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : UPS_AUX_sig_Pin */
  GPIO_InitStruct.Pin = UPS_AUX_sig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UPS_AUX_sig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R_AUX2_sig_Pin */
  GPIO_InitStruct.Pin = R_AUX2_sig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R_AUX2_sig_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
