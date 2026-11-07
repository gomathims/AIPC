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
			HAL_GPIO_WritePin(R_main_sig_GPIO_Port, R_main_sig_Pin, GPIO_PIN_SET);
			RmainRelayState = HAL_GPIO_ReadPin(R_main_sig_GPIO_Port, R_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | 1);
		}
	}
	return;
}

void YmainRelayTurnON() {
	if (!YmainRelayState) {
		while (!YmainRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_main_sig_GPIO_Port, Y_main_sig_Pin, GPIO_PIN_SET);
			YmainRelayState = HAL_GPIO_ReadPin(Y_main_sig_GPIO_Port, Y_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<1));
		}
	}
	return;

}

void BmainRelayTurnON() {
	if (!BmainRelayState) {
		while (!BmainRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_main_sig_GPIO_Port, B_main_sig_Pin, GPIO_PIN_SET);
			BmainRelayState = HAL_GPIO_ReadPin(B_main_sig_GPIO_Port, B_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<2));
		}
	}
	return;

}

void RYauxRelayTurnON() {
	if (!RYauxRelayState) {
		while (!RYauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(R_AUX1_sig_GPIO_Port,R_AUX1_sig_Pin, GPIO_PIN_SET);
			RYauxRelayState = HAL_GPIO_ReadPin(R_AUX1_sig_GPIO_Port,R_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<5));
		}
	}
	return;

}

void RBauxRelayTurnON() {
	if (!RBauxRelayState) {
		while (!RBauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(R_AUX2_sig_GPIO_Port, R_AUX2_sig_Pin, GPIO_PIN_SET);
			RBauxRelayState = HAL_GPIO_ReadPin(R_AUX2_sig_GPIO_Port, R_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<7));
		}
	}
	return;

}

void YRauxRelayTurnON() {
	if (!YRauxRelayState) {
		while (!YRauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_AUX1_sig_GPIO_Port, Y_AUX1_sig_Pin, GPIO_PIN_SET);
			YRauxRelayState = HAL_GPIO_ReadPin(Y_AUX1_sig_GPIO_Port, Y_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<3));
		}
	}
	return;

}

void YBauxRelayTurnON() {
	if (!YBauxRelayState) {
		while (!YBauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_AUX2_sig_GPIO_Port, Y_AUX2_sig_Pin, GPIO_PIN_SET);
			YBauxRelayState = HAL_GPIO_ReadPin(Y_AUX2_sig_GPIO_Port, Y_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<8));

		}
	}
	return;

}

void BYauxRelayTurnON() {
	if (!BYauxRelayState) {
		while (!BYauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_AUX1_sig_GPIO_Port, B_AUX1_sig_Pin, GPIO_PIN_SET);
			BYauxRelayState = HAL_GPIO_ReadPin(B_AUX1_sig_GPIO_Port, B_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<6));
		}
	}
	return;

}

void BRauxRelayTurnON() {
	if (!BRauxRelayState) {
		while (!BRauxRelayState) {
			/*to do :Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_AUX2_sig_GPIO_Port, B_AUX2_sig_Pin, GPIO_PIN_SET);
			BRauxRelayState = HAL_GPIO_ReadPin(B_AUX2_sig_GPIO_Port, B_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<4));
		}
	}
	return;

}

void UPSmainRelayON(){
	if(!UPSmainRelayState) {
		while(!UPSmainRelayState){
		/*to do :Implement the zero crossing switching technique*/
		HAL_GPIO_WritePin(UPS_main_sig_GPIO_Port, UPS_main_sig_Pin, GPIO_PIN_SET);
		UPSmainRelayState = HAL_GPIO_ReadPin(UPS_main_sig_GPIO_Port, UPS_main_sig_Pin);

		//to do : Implement the log for ups task.
		}


	}
	return;

}

void UPSauxRelayON(){
	if(!UPSauxRelayState) {
		while(!UPSauxRelayState){
		/*to do :Implement the zero crossing switching technique*/
		HAL_GPIO_WritePin(UPS_AUX_sig_GPIO_Port, UPS_AUX_sig_Pin, GPIO_PIN_SET);
		UPSauxRelayState = HAL_GPIO_ReadPin(UPS_AUX_sig_GPIO_Port, UPS_AUX_sig_Pin);

		//to do : Implement the log for ups task.
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
			HAL_GPIO_WritePin(R_main_sig_GPIO_Port, R_main_sig_Pin, GPIO_PIN_RESET);
			RmainRelayState = HAL_GPIO_ReadPin(R_main_sig_GPIO_Port, R_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~1));
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
			HAL_GPIO_WritePin(Y_main_sig_GPIO_Port, Y_main_sig_Pin, GPIO_PIN_RESET);
			YmainRelayState = HAL_GPIO_ReadPin(Y_main_sig_GPIO_Port, Y_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<1)));
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
			HAL_GPIO_WritePin(B_main_sig_GPIO_Port, B_main_sig_Pin, GPIO_PIN_RESET);
			BmainRelayState = HAL_GPIO_ReadPin(B_main_sig_GPIO_Port, B_main_sig_Pin);


			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<2)));
		}
	}
	return;

}

void RYauxRelayTurnOFF() {
	if (RYauxRelayState) {
		while (RYauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(R_AUX1_sig_GPIO_Port,R_AUX1_sig_Pin, GPIO_PIN_RESET);
			RYauxRelayState = HAL_GPIO_ReadPin(R_AUX1_sig_GPIO_Port,R_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<5)));
		}
	}
	return;

}

void RBauxRelayTurnOFF() {
	if (RBauxRelayState) {
		while (RBauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(R_AUX2_sig_GPIO_Port,R_AUX2_sig_Pin, GPIO_PIN_RESET);
			RBauxRelayState = HAL_GPIO_ReadPin(R_AUX2_sig_GPIO_Port,R_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<7)));
		}
	}
	return;

}

void YRauxRelayTurnOFF() {
	if (YRauxRelayState) {
		while (YRauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_AUX1_sig_GPIO_Port,Y_AUX1_sig_Pin, GPIO_PIN_RESET);
			YRauxRelayState = HAL_GPIO_ReadPin(Y_AUX1_sig_GPIO_Port,Y_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<3)));
		}
	}
	return;

}

void YBauxRelayTurnOFF() {
	if (YBauxRelayState) {
		while (YBauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(Y_AUX2_sig_GPIO_Port,Y_AUX2_sig_Pin, GPIO_PIN_RESET);
			YBauxRelayState = HAL_GPIO_ReadPin(Y_AUX2_sig_GPIO_Port,Y_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<8)));
		}
	}
	return;

}

void BYauxRelayTurnOFF() {
	if (BYauxRelayState) {
		while (BYauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_AUX1_sig_GPIO_Port,B_AUX1_sig_Pin, GPIO_PIN_RESET);
			BYauxRelayState = HAL_GPIO_ReadPin(B_AUX1_sig_GPIO_Port,B_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<6)));
		}
	}
	return;

}

void BRauxRelayTurnOFF() {
	if (BRauxRelayState) {
		while (BRauxRelayState) {
			/*Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(B_AUX2_sig_GPIO_Port,B_AUX2_sig_Pin, GPIO_PIN_RESET);
			BRauxRelayState = HAL_GPIO_ReadPin(B_AUX2_sig_GPIO_Port,B_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<4)));
		}
	}
	return;

}

void  UPSmainRelayOFF(){
	if(UPSmainRelayState){
		while (UPSmainRelayState){
			/*to do : Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(UPS_main_sig_GPIO_Port, UPS_main_sig_Pin, GPIO_PIN_RESET);
			UPSmainRelayState = HAL_GPIO_ReadPin(UPS_main_sig_GPIO_Port, UPS_main_sig_Pin);

			//to do : Implement the log for ups task.

		}
	}
	return;
}

void  UPSauxRelayOFF(){
	if(UPSauxRelayState){
		while (UPSauxRelayState){
			/*to do : Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(UPS_AUX_sig_GPIO_Port, UPS_AUX_sig_Pin, GPIO_PIN_RESET);
			UPSauxRelayState = HAL_GPIO_ReadPin(UPS_AUX_sig_GPIO_Port, UPS_AUX_sig_Pin);

			//to do : Implement the log for ups task.

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
