/*
 * Relay_controlling.c
 *
 *  Created on: Jun 25, 2026
 *      Author: EVIQtech
 */

#include "Relay_controlling.h"
#include "main.h"

void RmainRelayTurnON() {
	if (!RmainRelayState) {
		while (!RmainRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(R_mainRelay_GPIO_Port, R_mainRelay_Pin, GPIO_PIN_SET);
			RmainRelayState = HAL_GPIO_ReadPin(R_mainRelay_GPIO_Port, R_mainRelay_Pin);
		}
	}
	return;
}

void YmainRelayTurnON() {
	if (!YmainRelayState) {
		while (!YmainRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_mainRelay_GPIO_Port, Y_mainRelay_Pin, GPIO_PIN_SET);
			YmainRelayState = HAL_GPIO_ReadPin(Y_mainRelay_GPIO_Port, Y_mainRelay_Pin);
		}
	}
	return;

}

void BmainRelayTurnON() {
	if (!BmainRelayState) {
		while (!BmainRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_mainRelay_GPIO_Port, B_mainRelay_Pin, GPIO_PIN_SET);
			BmainRelayState = HAL_GPIO_ReadPin(B_mainRelay_GPIO_Port, B_mainRelay_Pin);
		}
	}
	return;

}

void RYauxRelayTurnON() {
	if (!RYauxRelayState) {
		while (!RYauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(R_YauxRelay_GPIO_Port, R_YauxRelay_Pin, GPIO_PIN_SET);
			RYauxRelayState = HAL_GPIO_ReadPin(R_YauxRelay_GPIO_Port, R_YauxRelay_Pin);
		}
	}
	return;

}

void YRauxRelayTurnON() {
	if (!YRauxRelayState) {
		while (!YRauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_RauxRelay_GPIO_Port, Y_RauxRelay_Pin, GPIO_PIN_SET);
			YRauxRelayState = HAL_GPIO_ReadPin(Y_RauxRelay_GPIO_Port, Y_RauxRelay_Pin);
		}
	}
	return;

}

void YBauxRelayTurnON() {
	if (!YBauxRelayState) {
		while (!YBauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_BauxRelay_GPIO_Port, Y_BauxRelay_Pin, GPIO_PIN_SET);
			YBauxRelayState = HAL_GPIO_ReadPin(Y_BauxRelay_GPIO_Port, Y_BauxRelay_Pin);
		}
	}
	return;

}

void BYauxRelayTurnON() {
	if (!BYauxRelayState) {
		while (!BYauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_YauxRelay_GPIO_Port, B_YauxRelay_Pin, GPIO_PIN_SET);
			BYauxRelayState = HAL_GPIO_ReadPin(B_YauxRelay_GPIO_Port, B_YauxRelay_Pin);
		}
	}
	return;

}

void BRauxRelayTurnON() {
	if (!BRauxRelayState) {
		while (!BRauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_RauxRelay_GPIO_Port, B_RauxRelay_Pin, GPIO_PIN_SET);
			BRauxRelayState = HAL_GPIO_ReadPin(B_RauxRelay_GPIO_Port, B_RauxRelay_Pin);
		}
	}
	return;

}

void RBauxRelayTurnON() {
	if (!RBauxRelayState) {
		while (!RBauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(R_BauxRelay_GPIO_Port, R_BauxRelay_Pin, GPIO_PIN_SET);
			RBauxRelayState = HAL_GPIO_ReadPin(R_BauxRelay_GPIO_Port, R_BauxRelay_Pin);
		}
	}
	return;

}

void RmainRelayTurnOFF() {
	RYauxRelayTurnOFF();
	RBauxRelayTurnOFF();
	if (RmainRelayState) {
		while (RmainRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(R_mainRelay_GPIO_Port, R_mainRelay_Pin, GPIO_PIN_RESET);
			RmainRelayState = HAL_GPIO_ReadPin(R_mainRelay_GPIO_Port, R_mainRelay_Pin);
		}
	}
	return;

}

void YmainRelayTurnOFF() {
	YRauxRelayTurnOFF();
	YBauxRelayTurnOFF();
	if (YmainRelayState) {
		while (YmainRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_mainRelay_GPIO_Port, Y_mainRelay_Pin, GPIO_PIN_RESET);
			YmainRelayState = HAL_GPIO_ReadPin(Y_mainRelay_GPIO_Port, Y_mainRelay_Pin);
		}
	}
	return;

}

void BmainRelayTurnOFF() {
	BRauxRelayTurnOFF();
	BYauxRelayTurnOFF();
	if (BmainRelayState) {
		while (BmainRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_mainRelay_GPIO_Port, B_mainRelay_Pin, GPIO_PIN_RESET);
			BmainRelayState = HAL_GPIO_ReadPin(B_mainRelay_GPIO_Port, B_mainRelay_Pin);
		}
	}
	return;

}

void RYauxRelayTurnOFF() {
	if (RYauxRelayState) {
		while (RYauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(R_YauxRelay_GPIO_Port, R_YauxRelay_Pin, GPIO_PIN_RESET);
			RYauxRelayState = HAL_GPIO_ReadPin(R_YauxRelay_GPIO_Port, R_YauxRelay_Pin);
		}
	}
	return;

}

void YRauxRelayTurnOFF() {
	if (YRauxRelayState) {
		while (YRauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_RauxRelay_GPIO_Port, Y_RauxRelay_Pin, GPIO_PIN_RESET);
			YRauxRelayState = HAL_GPIO_ReadPin(Y_RauxRelay_GPIO_Port, Y_RauxRelay_Pin);
		}
	}
	return;

}

void YBauxRelayTurnOFF() {
	if (YBauxRelayState) {
		while (YBauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_BauxRelay_GPIO_Port, Y_BauxRelay_Pin, GPIO_PIN_RESET);
			YBauxRelayState = HAL_GPIO_ReadPin(Y_BauxRelay_GPIO_Port, Y_BauxRelay_Pin);
		}
	}
	return;

}

void BYauxRelayTurnOFF() {
	if (BYauxRelayState) {
		while (BYauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_YauxRelay_GPIO_Port, B_YauxRelay_Pin, GPIO_PIN_RESET);
			BYauxRelayState = HAL_GPIO_ReadPin(B_YauxRelay_GPIO_Port, B_YauxRelay_Pin);
		}
	}
	return;

}

void BRauxRelayTurnOFF() {
	if (BRauxRelayState) {
		while (BRauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_RauxRelay_GPIO_Port, B_RauxRelay_Pin, GPIO_PIN_RESET);
			BRauxRelayState = HAL_GPIO_ReadPin(B_RauxRelay_GPIO_Port, B_RauxRelay_Pin);
		}
	}
	return;

}

void RBauxRelayTurnOFF() {
	if (RBauxRelayState) {
		while (RBauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(R_BauxRelay_GPIO_Port, R_BauxRelay_Pin, GPIO_PIN_RESET);
			RBauxRelayState = HAL_GPIO_ReadPin(R_BauxRelay_GPIO_Port, R_BauxRelay_Pin);
		}
	}
	return;

}

void RincomingAuxRelayOFF() {
	YRauxRelayTurnOFF();
	BRauxRelayTurnOFF();

}

void YincomingAuxRelayOFF() {
	RYauxRelayTurnOFF();
	BYauxRelayTurnOFF();

}

void BincomingAuxRelayOFF() {
	RBauxRelayTurnOFF();
	YBauxRelayTurnOFF();

}
