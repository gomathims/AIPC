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
#include "stm_selfupdate.h"

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
#define CALIB_VOL1 14324.957805
#define CALIB_CUR1 301062.271062
#define CALIB_POW1 648.885586
#define CALIB_ENE1 4667.50266786

#define CALIB_VOL2 14374.3938844
#define CALIB_CUR2 345157.509157
#define CALIB_POW2 739.078751
#define CALIB_ENE2 5667.50266786


#define CALIB_VOL3 14388.8458298
#define CALIB_CUR3 304084.249084
#define CALIB_POW3 656.745913
#define CALIB_ENE3 5000.96266786


//#define CALIB_VOL 1
//#define CALIB_CUR 1
//#define CALIB_POW 1
//#define CALIB_ENE 1

/*RX buffer size deceleration*/
#define BUF_SIZE 256

#define OVERLOAD_DELAY 70000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t setupBL[6];
uint8_t txData[2];
uint8_t rxDataV1[4], rxDataV2[4], rxDataV3[4];
uint8_t rxDataC1[4], rxDataC2[4], rxDataC3[4];
uint8_t rxDataP1[4], rxDataP2[4], rxDataP3[4];
uint8_t rxDataE1[4], rxDataE2[4], rxDataE3[4];
volatile float  voltL1, voltL2, voltL3;
volatile float  curL1, curL2, curL3;
volatile float  powL1, powL2, powL3;
volatile float  eneL1, eneL2, eneL3;

unsigned short uartGoFlag;			//0b->v1, 1b->c1, 2b->p1, 3b->e1, 4b->v2, 5b->c2, 6b->p2, 7b->e2, 8b->v3, 9b->c3, 10b->p3, 11b->e3, 12b->"", 13b->"", 14b->"", 15b->"".
unsigned short relayStatus;
unsigned short contactorStatus;

volatile bool healthyRphase,healthyYphase,healthyBphase;
volatile bool RmainRelayState, YmainRelayState, BmainRelayState, RYauxRelayState, YRauxRelayState, YBauxRelayState, BYauxRelayState, RBauxRelayState, BRauxRelayState, UPSRelayState;
uint8_t Uart3RXbuffer[BUF_SIZE];
typedef struct PhaseProtection {
    bool enabled;
    float highV;   // Volts
    float lowV;    // Volts
    float highI;   // Amps (overload)
}PhaseProtection;


PhaseProtection prot[3];
uint8_t headerProtection = 0XCC, checkSum;
bool protCheckSumMismatch;
uint16_t voltLow[3],voltHigh[3],overload[3];
uint32_t overloadAlertL1StartTime,  overloadAlertL2StartTime, overloadAlertL3StartTime;
bool overloadAlertL1,overloadAlertL2,overloadAlertL3;
uint64_t receivedUnixTime;
bool timeSynced;



//__attribute__((section(".noinit"))) volatile uint32_t bootMagic;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t validateChecksum(uint8_t high, uint8_t mid, uint8_t low, uint8_t checksum){
	return checksum == (uint8_t) ~((high+mid+low)&0XFF);

}

void UARTcollection(uint16_t Size) {
	bool transfer = false;

	for (uint16_t i = 0; i < Size; i++) {
		if (Uart3RXbuffer[i] == headerProtection && (Size - i) >= 41) {
			transfer = true;
		}
	}
	checkSum = 0;
	for (int i = 1; i < 40; i++) {
		checkSum += Uart3RXbuffer[i];
	}
	checkSum = (~(checkSum & 0XFF));

	if (transfer && (checkSum == Uart3RXbuffer[40])) {
		prot[0].enabled = Uart3RXbuffer[1];
		memcpy((&prot[0].lowV), &Uart3RXbuffer[2], 4);
		memcpy((&prot[0].highV), &Uart3RXbuffer[6], 4);
		memcpy((&prot[0].highI), &Uart3RXbuffer[10], 4);
		prot[1].enabled = Uart3RXbuffer[14];
		memcpy((&prot[1].lowV), &Uart3RXbuffer[15], 4);
		memcpy((&prot[1].highV), &Uart3RXbuffer[19], 4);
		memcpy((&prot[1].highI), &Uart3RXbuffer[23], 4);
		prot[2].enabled = Uart3RXbuffer[27];
		memcpy((&prot[2].lowV), &Uart3RXbuffer[28], 4);
		memcpy((&prot[2].highV), &Uart3RXbuffer[32], 4);
		memcpy((&prot[2].highI), &Uart3RXbuffer[36], 4);
	}else{
		protCheckSumMismatch=true;
	}

//	printf(" L1 %d ->%f  %f  %f   \r\n", prot[0].enabled, prot[0].lowV,
//			prot[0].highV, prot[0].highI);
//	printf(" L2 %d ->%f  %f  %f   \r\n", prot[1].enabled, prot[1].lowV,
//			prot[1].highV, prot[1].highI);
//	printf(" L3 %d ->%f  %f  %f   \r\n", prot[2].enabled, prot[2].lowV,
//			prot[2].highV, prot[2].highI);

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
			timeSynced =true;
			printf("%lu\r\n",(unsigned long)receivedUnixTime);
			return;
		}
	}
	return;


//	time_t epoch = receivedUinxTime;
//	struct tm* time = gmtime(&epoch);





}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART3) {

			for (uint16_t i = 0; i < Size; i++)
			{
		    	printf("%02X ", Uart3RXbuffer[i]);
			}
			printf("Coming inside\r\n");
			NTPsyncTimeToRTC(Size);
			suOnRxBurst(Uart3RXbuffer, Size); // <-- add: catch AA55A55A / BB44B44B
			UARTcollection(Size);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Uart3RXbuffer, BUF_SIZE);
			__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);


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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
//setupBL[0] = 0xA5;
//setupBL[1] = 0x15;
//setupBL[2] = 0x00;
//setupBL[3] = 0x00;
//setupBL[4] = 0x20;
//setupBL[5] = 0x23;

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Uart3RXbuffer, BUF_SIZE);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);


//  HAL_RTC_Init(&hrtc);
  HAL_PWR_EnableBkUpAccess();

  RTC_TimeTypeDef rtcTime;
  RTC_DateTypeDef rtcDate;

/*To know the status of the boot function properly worked or not!!! just for the testing purpose*/

//  HAL_FLASH_Unlock();
//  FLASH_EraseInitTypeDef er = {0}; uint32_t pe;
//  er.TypeErase = FLASH_TYPEERASE_PAGES;
//  er.Banks   = (FLASH->OPTR & FLASH_OPTR_BFB2) ? FLASH_BANK_1 : FLASH_BANK_2;
//  er.Page = 0; er.NbPages = 1;
//  HAL_FLASHEx_Erase(&er, &pe);
//  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08040000UL, 0x1122334455667788ULL);
//  HAL_FLASH_Lock();
//  uint64_t rb = *(volatile uint64_t *)0x08040000UL;
//  printf("bank self-test: %s\r\n", (rb == 0x1122334455667788ULL) ? "OK" : "FAIL");





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


		if (timeSynced) {
			time_t setUnixTime = (time_t)(receivedUnixTime + 19800);
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
			rtcDate.Year = time.tm_year - 100;  // 2026 -> 26

			HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BCD);
			HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BCD);
			timeSynced = false;


			printf("Time: %02d:%02d:%02d\r\n", time.tm_hour, time.tm_min,time.tm_sec);
			printf("Date: %02d/%02d/20%02d\r\n", time.tm_mday, time.tm_mon, time.tm_year);

		}

	  HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
	  		printf("Time: %02d:%02d:%02d\r\n", rtcTime.Hours, rtcTime.Minutes,rtcTime.Seconds);
	  		printf("Date: %02d/%02d/20%02d\r\n", rtcDate.Date, rtcDate.Month, rtcDate.Year);








		/*Flash process with custom boot loader*/
		suService();       // <-- runs the (blocking) receive/apply when flagged(continuously monitor whether the ESP is sending the firmware for update or not! )
		if (suBusy())
			continue;      // <-- not obtain data from BL0939 and won't transmit metering  while updating the firmware

		/*End of this individual function*/


	  /*Data collecting from the BL0939 IC via SPI protocol communication for the L1 L2 L3*/

	  /*----------------------------------------L1------------------------------------------------------*/

	  	txData[0] = 0x55;

		//printf("data-> %d, %d, %d, %d\r\n",rxData[0], rxData[1], rxData[2], rxData[3]);

		HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, SET);
//		HAL_SPI_Transmit(&hspi1, setupBL, 6, HAL_MAX_DELAY);

		txData[1] = 0x06;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataV1, 4, HAL_MAX_DELAY);

		txData[1] = 0x04;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataC1, 4, HAL_MAX_DELAY);

		txData[1] = 0x08;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataP1, 4, HAL_MAX_DELAY);

		txData[1] = 0x0A;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataE1, 4, HAL_MAX_DELAY);

		HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, RESET);

//		printf("data1-> %X, %X, %X, %X\r\n", rxDataP1[0], rxDataP1[1], rxDataP1[2],rxDataP1[3]);

		if (validateChecksum(rxDataV1[0], rxDataV1[1], rxDataV1[2], rxDataV1[3]) == 0) {
			voltL1 = ((rxDataV1[0] << 16 | rxDataV1[1] << 8 | rxDataV1[2])/CALIB_VOL1);
			uartGoFlag = uartGoFlag | 0X0001;
//			printf("Vol1 = %.2f\r\n", voltL1);
		}
		if (validateChecksum(rxDataC1[0], rxDataC1[1], rxDataC1[2], rxDataC1[3]) == 0) {
			curL1 = ((rxDataC1[0] << 16 | rxDataC1[1] << 8 | rxDataC1[2])/CALIB_CUR1);
			uartGoFlag = uartGoFlag | 0X0002;
//			printf("curL1 = %.3f \r\n", curL1);
		}
		if (validateChecksum(rxDataP1[0], rxDataP1[1], rxDataP1[2], rxDataP1[3]) == 0) {
			powL1 = ((rxDataP1[0] << 16 | rxDataP1[1] << 8 | rxDataP1[2])/CALIB_POW1);
			uartGoFlag = uartGoFlag | 0X0004;
//			printf("powL1 = %.2f \r\n", powL1);
		}
		if (validateChecksum(rxDataE1[0], rxDataE1[1], rxDataE1[2], rxDataE1[3]) == 0) {
			eneL1 = ((rxDataE1[0] << 16 | rxDataE1[1] << 8 | rxDataE1[2]) / CALIB_ENE1);
			uartGoFlag = uartGoFlag | 0X0008;
//			printf("eneL1 = %.3f \r\n", eneL1);
		}

//		printf("\r\n");





		  /*----------------------------------------L2------------------------------------------------------*/
		txData[0] = 0x55;

		HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, SET);
//		HAL_SPI_Transmit(&hspi1, setupBL, 6, HAL_MAX_DELAY);


		txData[1] = 0x06;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataV2, 4, HAL_MAX_DELAY);

		txData[1] = 0x04;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataC2, 4, HAL_MAX_DELAY);

		txData[1] = 0x08;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataP2, 4, HAL_MAX_DELAY);

		txData[1] = 0x0A;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataE2, 4, HAL_MAX_DELAY);

		HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, RESET);

//		printf("data2-> %X, %X, %X, %X\r\n", rxDataP2[0], rxDataP2[1], rxDataP2[2],rxDataP2[3]);

		if (validateChecksum(rxDataV2[0], rxDataV2[1], rxDataV2[2], rxDataV2[3]) == 0) {
			voltL2 = ((rxDataV2[0] << 16 | rxDataV2[1] << 8 | rxDataV2[2]) / CALIB_VOL2);
			uartGoFlag = uartGoFlag | 0X0010;
//			printf("Vol2 = %.2f\r\n", voltL2);
		}
		if (validateChecksum(rxDataC2[0], rxDataC2[1], rxDataC2[2], rxDataC2[3]) == 0) {
			curL2 = ((rxDataC2[0] << 16 | rxDataC2[1] << 8 | rxDataC2[2]) / CALIB_CUR2);
			uartGoFlag = uartGoFlag | 0X0020;
//			printf("curL2 = %.3f \r\n", curL2);
		}
		if (validateChecksum(rxDataP2[0], rxDataP2[1], rxDataP2[2], rxDataP2[3]) == 0) {
			powL2 = ((rxDataP2[0] << 16 | rxDataP2[1] << 8 | rxDataP2[2]) / CALIB_POW2);
			uartGoFlag = uartGoFlag | 0X0040;
//			printf("powL2 = %.2f \r\n", powL2);
		}
		if (validateChecksum(rxDataE2[0], rxDataE2[1], rxDataE2[2], rxDataE2[3]) == 0) {
			eneL2 = ((rxDataE2[0] << 16 | rxDataE2[1] << 8 | rxDataE2[2]) / CALIB_ENE2);
			uartGoFlag = uartGoFlag | 0X0080;
//			printf("eneL2 = %.3f \r\n", eneL2);
		}

//		printf("\r\n");


		  /*----------------------------------------L3------------------------------------------------------*/
		txData[0] = 0x55;

		HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, SET);

//		HAL_SPI_Transmit(&hspi1, setupBL, 6, HAL_MAX_DELAY);


		txData[1] = 0x06;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataV3, 4, HAL_MAX_DELAY);

		txData[1] = 0x04;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataC3, 4, HAL_MAX_DELAY);

		txData[1] = 0x08;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataP3, 4, HAL_MAX_DELAY);

		txData[1] = 0x0A;
		HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, rxDataE3, 4, HAL_MAX_DELAY);

		HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, RESET);

//		printf("data3-> %X, %X, %X, %X\r\n", rxDataP3[0], rxDataP3[1], rxDataP3[2], rxDataP3[3]);


		if (validateChecksum(rxDataV3[0], rxDataV3[1], rxDataV3[2], rxDataV3[3]) == 0) {
			voltL3 = ((rxDataV3[0] << 16 | rxDataV3[1] << 8 | rxDataV3[2]) / CALIB_VOL3);
			uartGoFlag = uartGoFlag | 0X0100;
//			printf("Vol3 = %.2f\r\n", voltL3);
		}
		if (validateChecksum(rxDataC3[0], rxDataC3[1], rxDataC3[2], rxDataC3[3]) == 0) {
			curL3 = ((rxDataC3[0] << 16 | rxDataC3[1] << 8 | rxDataC3[2]) / CALIB_CUR3);
			uartGoFlag = uartGoFlag | 0X0200;
//			printf("curL3 = %.3f \r\n", curL3);
		}
		if (validateChecksum(rxDataP3[0], rxDataP3[1], rxDataP3[2], rxDataP3[3]) == 0) {
			powL3 = ((rxDataP3[0] << 16 | rxDataP3[1] << 8 | rxDataP3[2]) / CALIB_POW3);
			uartGoFlag = uartGoFlag | 0X0400;
//			printf("powL3 = %.2f \r\n", powL3);
		}
		if (validateChecksum(rxDataE3[0], rxDataE3[1], rxDataE3[2], rxDataE3[3]) == 0) {
			eneL3 = ((rxDataE3[0] << 16 | rxDataE3[1] << 8 | rxDataE3[2]) / CALIB_ENE3);
			uartGoFlag = uartGoFlag | 0X0800;
//			printf("eneL3 = %.3f \r\n", eneL3);
		}

//		printf("\r\n");

		/*End of this individual function*/

		/*Protection Settings assigning condition with hiding noise of volt and current*/

		if (curL1 < 0.08) {
			curL1 = 0;
			powL1 = 0;
		}
		if (curL2 < 0.08) {
			curL2 = 0;
			powL2 = 0;
		}
		if (curL3 < 0.08) {
			curL3 = 0;
			powL3 = 0;
		}

		if (prot[0].enabled) {
			voltLow[0] = prot[0].lowV;
			voltHigh[0] = prot[0].highV;
			overload[0] = prot[0].highI;
		} else {
			voltLow[0] = 170;
			voltHigh[0] = 270;
			overload[0] = 20;
		}
		if (prot[1].enabled) {
			voltLow[1] = prot[1].lowV;
			voltHigh[1] = prot[1].highV;
			overload[1] = prot[1].highI;
		} else {
			voltLow[1] = 170;
			voltHigh[1] = 270;
			overload[1] = 20;
		}
		if (prot[2].enabled) {
			voltLow[2] = prot[2].lowV;
			voltHigh[2] = prot[2].highV;
			overload[2] = prot[2].highI;
		} else {
			voltLow[2] = 170;
			voltHigh[2] = 270;
			overload[2] = 20;
		}
//			Testing purpose:
//				 printf(" L1 %d ->%d  %d  %d   \r\n",prot[0].enabled,voltLow[0],voltHigh[0],overload[0]);
//				 printf(" L2 %d ->%d  %d  %d   \r\n",prot[1].enabled,voltLow[1],voltHigh[1],overload[1]);
//				 printf(" L3 %d ->%d  %d  %d   \r\n",prot[2].enabled,voltLow[2],voltHigh[2],overload[2]);


		/*End of this individual function*/

		/*Phase health check with voltage and overload*/
		healthyRphase = ((voltL1 > voltLow[0]) && (voltL1 < voltHigh[0])) ? 1 : 0;
		healthyYphase = ((voltL2 > voltLow[1]) && (voltL2 < voltHigh[1])) ? 1 : 0;
		healthyBphase = ((voltL3 > voltLow[2]) && (voltL3 < voltHigh[2])) ? 1 : 0;

		if (healthyRphase && (curL1 > overload[0])) {
			if (!overloadAlertL1StartTime) {
				overloadAlertL1StartTime = HAL_GetTick();
			}
			if (HAL_GetTick() - overloadAlertL1StartTime >= OVERLOAD_DELAY) {
				overloadAlertL1 = true;
			}
		} else {
			overloadAlertL1StartTime = 0;
		}

		if (healthyYphase && (curL2 > overload[1])) {
			if (!overloadAlertL2StartTime) {
				overloadAlertL2StartTime = HAL_GetTick();
			}
			if (HAL_GetTick() - overloadAlertL2StartTime >= OVERLOAD_DELAY) {
				overloadAlertL2 = true;
			}
		} else {
			overloadAlertL2StartTime = 0;
		}

		if (healthyBphase && (curL1 > overload[2])) {
			if (!overloadAlertL3StartTime) {
				overloadAlertL3StartTime = HAL_GetTick();
			}

			if (HAL_GetTick() - overloadAlertL3StartTime >= OVERLOAD_DELAY) {
				overloadAlertL3 = true;
			}
		} else {
			overloadAlertL3StartTime = 0;
		}

		/*Resume to recover the relay State from the overload alert*/
		//Resume for the Recovery from overload of all phases.After remove this comment once you done the recovery process
//		if (overloadAlertL1 && resumel1) {
//			overloadAlertL1 = false;
//			overloadAlertL1StartTime =0;
//		}
//
//		if (overloadAlertL2 && resumel2) {
//			overloadAlertL2 = false;
//			overloadAlertL2StartTime =0;
//		}
//
//		if (overloadAlertL3 && resumel3) {
//			overloadAlertL3 = false;
//			overloadAlertL3StartTime =0;
//		}

		/*End of this individual function*/






		/*Switch-over Management*/


																												// | ! |    Overload management has been done by the R-phase only. Will test implement this to other phases if the test is succeed.


				if (!healthyRphase || overloadAlertL1) {
					if (RmainRelayState) {
						// Function to OFF the relay once, inside it should OFF the Aux of the R main relay
						RmainRelayTurnOFF();
						// give some delay if not feedback
//						HAL_Delay(500);
					}
					if (healthyYphase && healthyBphase && !YRauxRelayState
							&& !BRauxRelayState && !overloadAlertL1) {		//prevention condition
						// choose any one of the phase with the help of the current
						// with that phase ON the the Aux to the the R phase once
						/*depends on the current Aux will switch over*/
						if (curL2 < curL3) {
							YRauxRelayTurnON();
						} else if (curL3 < curL2) {
							BRauxRelayTurnON();
						} else {
							YRauxRelayTurnON();
						}
					} else if(!overloadAlertL1) {
						if (healthyYphase && !YRauxRelayState && !BRauxRelayState) {
							// ON the Y phase Aux to the R main once
							YRauxRelayTurnON();
						} else if (healthyBphase && !YRauxRelayState
								&& !BRauxRelayState) {
							// ON the B phase Aux to the R main once
							BRauxRelayTurnON();
						} else {
							//not to change over
						}
					}
				} else if(healthyRphase && !overloadAlertL1) {
					if (!RmainRelayState) {
						if (YRauxRelayState || BRauxRelayState) {
							// function to OFF the all Aux from other phase to R phases
							RincomingAuxRelayOFF();
//							HAL_Delay(500);
						}

						// give some delay if not feedback
						// Turn ON the R main relay
						RmainRelayTurnON();
					}

				}

				if (!healthyYphase || overloadAlertL2) {
					if (YmainRelayState) {
						// Function to OFF the relay once, inside it should OFF the Aux of the Y main relay
						YmainRelayTurnOFF();
						// give some delay if not feedback
//						HAL_Delay(500);
					}
					if (healthyRphase && healthyBphase && !RYauxRelayState
							&& !BYauxRelayState && !overloadAlertL2) { 		//prevention condition
						// choose any one of the phase with the help of the current
						// with that phase ON the the Aux to the the R phase once
						/*depends on the current Aux will switch over*/
						if (curL1 < curL3) {
							RYauxRelayTurnON();
						} else if (curL3 < curL1) {
							BYauxRelayTurnON();
						} else {
							RYauxRelayTurnON();
						}

					} else if(!overloadAlertL2){
						if (healthyRphase && !RYauxRelayState && !BYauxRelayState) {
							// ON the Y phase Aux to the R main once
							RYauxRelayTurnON();
						} else if (healthyBphase && !RYauxRelayState
								&& !BYauxRelayState) {
							// ON the B phase Aux to the R main once
							BYauxRelayTurnON();
						} else {
							//not to change over
						}

					}
				} else if(healthyYphase && !overloadAlertL2) {
					if (!YmainRelayState) {
						if (RYauxRelayState || BYauxRelayState) {
							// function to OFF the all Aux from other phase to Y phases
							YincomingAuxRelayOFF();
							// give some delay if not feedback
//							HAL_Delay(500);
						}
						// Turn ON the Y main relay
						YmainRelayTurnON();

					}
				}

				if (!healthyBphase || overloadAlertL3) {
					if (BmainRelayState) {
						// Function to OFF the relay once, inside it should OFF the Aux of the B main relay
						BmainRelayTurnOFF();
						// give some delay if not feedback
//						HAL_Delay(500);
					}
					if (healthyRphase && healthyYphase && !RBauxRelayState
							&& !YBauxRelayState && !overloadAlertL3) {		//prevention condition
						// choose any one of the phase with the help of the current
						// with that phase ON the the Aux to the the R phase once
						/*depends on the current Aux will switch over*/
						if (curL1 < curL2) {
							RBauxRelayTurnON();
						} else if (curL3 < curL1) {
							YBauxRelayTurnON();
						} else {
							RBauxRelayTurnON();
						}


					} else if(!overloadAlertL3) {
						if (healthyRphase && !RBauxRelayState && !YBauxRelayState) {
							// ON the Y phase Aux to the R main once
							RBauxRelayTurnON();
						} else if (healthyYphase && !RBauxRelayState
								&& !YBauxRelayState) {
							// ON the B phase Aux to the R main once
							YBauxRelayTurnON();
						} else {
							//not to change over
						}

					}
				} else if(healthyBphase && !overloadAlertL3) {
					if (!BmainRelayState) {
						if (!RBauxRelayState || !YBauxRelayState) {
							// function to OFF the all Aux from other phase to B phases
							BincomingAuxRelayOFF();
							// give some delay if not feedback
//							HAL_Delay(500);
						}
						// Turn ON the B main relay
						BmainRelayTurnON();
					}

				}
				/*End of this individual function*/


				/*UART Sending: Data which are gathered from the BL is transmitting to ESP via UART*/
		/*While sending the UART the values validate with the "uartGoFlag" variable and then go for it*/

		if (uartGoFlag & 0X0FFF) {
			HAL_UART_Transmit(&huart3, (uint8_t*) "STM", 4, 10);
			uint8_t transmit[38];
			relayStatus = 0;
			relayStatus |= RmainRelayState << 0;
			relayStatus |= YmainRelayState << 1;
			relayStatus |= BmainRelayState << 2;
			relayStatus |= RYauxRelayState << 3;
			relayStatus |= YRauxRelayState << 4;
			relayStatus |= YBauxRelayState << 5;
			relayStatus |= BYauxRelayState << 6;
			relayStatus |= RBauxRelayState << 7;
			relayStatus |= BRauxRelayState << 8;
			relayStatus |= UPSRelayState   << 9;
			memcpy(transmit, rxDataV1, 3);
			memcpy((&transmit[3]), &rxDataV2, 3);
			memcpy((&transmit[6]), &rxDataV3, 3);
			memcpy((&transmit[9]), &rxDataC1, 3);
			memcpy((&transmit[12]), &rxDataC2, 3);
			memcpy((&transmit[15]), &rxDataC3, 3);
			memcpy((&transmit[18]), &rxDataP1, 3);
			memcpy((&transmit[21]), &rxDataP2, 3);
			memcpy((&transmit[24]), &rxDataP3, 3);
			memcpy((&transmit[27]), &rxDataE1, 3);
			memcpy((&transmit[30]), &rxDataE2, 3);
			memcpy((&transmit[33]), &rxDataE3, 3);
			memcpy((&transmit[36]), &relayStatus, 2);
			HAL_UART_Transmit(&huart3, (uint8_t*) &transmit, sizeof(transmit),10);
			uartGoFlag = 0;
		}

		/*Acknowledge or Resent commends from STM to ESP via UART*/
//		if (!(prot[0].highI) || protCheckSumMismatch ){
//			HAL_UART_Transmit(&huart3,(uint8_t*)&headerProtection,1,10);
//			protCheckSumMismatch = false;
//		}
		/*End of this individual function*/





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
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
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

//  RTC_TimeTypeDef sTime = {0};
//  RTC_DateTypeDef sDate = {0};

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

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
//  sTime.Hours = 0x12;
//  sTime.Minutes = 0x23;
//  sTime.Seconds = 0x0;
//  sTime.SubSeconds = 0x0;
//  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
//  sDate.Month = RTC_MONTH_AUGUST;
//  sDate.Date = 0x20;
//  sDate.Year = 0x0;

//  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS2_Pin|CS3_Pin|R_YauxRelay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, B_RauxRelay_Pin|Y_BauxRelay_Pin|CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B_YauxRelay_Pin|R_BauxRelay_Pin|Y_RauxRelay_Pin|B_mainRelay_Pin
                          |R_mainRelay_Pin|Y_mainRelay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 ZE_Yphase_Pin ZE_Bphase_Pin ZE_UPS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|ZE_Yphase_Pin|ZE_Bphase_Pin|ZE_UPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS2_Pin CS3_Pin R_YauxRelay_Pin */
  GPIO_InitStruct.Pin = CS2_Pin|CS3_Pin|R_YauxRelay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B_RauxRelay_Pin Y_BauxRelay_Pin CS1_Pin */
  GPIO_InitStruct.Pin = B_RauxRelay_Pin|Y_BauxRelay_Pin|CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B_YauxRelay_Pin R_BauxRelay_Pin Y_RauxRelay_Pin B_mainRelay_Pin
                           R_mainRelay_Pin Y_mainRelay_Pin */
  GPIO_InitStruct.Pin = B_YauxRelay_Pin|R_BauxRelay_Pin|Y_RauxRelay_Pin|B_mainRelay_Pin
                          |R_mainRelay_Pin|Y_mainRelay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
