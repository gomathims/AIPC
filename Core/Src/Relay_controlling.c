/*
 * Relay_controlling.c
 *
 *  Created on: Jun 25, 2026
 *      Author: EVIQtech
 */

#include "Relay_controlling.h"
#include "main.h"

void rMainRelayTurnOn() {
	if (!rMainRelayState) {
		while (!rMainRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t rMainOnTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
					if ((HAL_GetTick() - rMainOnTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin))) {
					if ((HAL_GetTick() - rMainOnTime) >= 20) {
						break;
					}

				}
			}

			HAL_GPIO_WritePin(R_main_sig_GPIO_Port, R_main_sig_Pin, GPIO_PIN_SET);
			rMainRelayState = HAL_GPIO_ReadPin(R_main_sig_GPIO_Port, R_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | 1);
		}
	}
	return;
}

void yMainRelayTurnOn() {
	if (!yMainRelayState) {
		while (!yMainRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t yMainOnTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
					if ((HAL_GetTick() - yMainOnTime) >= 20) {
						break;
					}
				}
			} else {
				while (!(HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin))) {
					if ((HAL_GetTick() - yMainOnTime) >= 20) {
						break;
					}

				}
			}


			HAL_GPIO_WritePin(Y_main_sig_GPIO_Port, Y_main_sig_Pin, GPIO_PIN_SET);
			yMainRelayState = HAL_GPIO_ReadPin(Y_main_sig_GPIO_Port, Y_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<1));
		}
	}
	return;

}

void bMainRelayTurnOn() {
	if (!bMainRelayState) {
		while (!bMainRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t bMainOnTime=HAL_GetTick();
			if (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
					if((HAL_GetTick() - bMainOnTime) >=20){
						break;
					}
				}
			} else {
				while (!(HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin))) {
					if ((HAL_GetTick() - bMainOnTime) >= 20) {
						break;
					}

				}
			}

			HAL_GPIO_WritePin(B_main_sig_GPIO_Port, B_main_sig_Pin, GPIO_PIN_SET);
			bMainRelayState = HAL_GPIO_ReadPin(B_main_sig_GPIO_Port, B_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<2));
		}
	}
	return;

}

void ryAuxRelayTurnOn() {
	if (!ryAuxRelayState) {
		while (!ryAuxRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t ryAuxOnTime =HAL_GetTick();
			if (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
					if ((HAL_GetTick() - ryAuxOnTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin))) {
					if ((HAL_GetTick() - ryAuxOnTime) >= 20) {
						break;
					}

				}
			}

			HAL_GPIO_WritePin(R_AUX1_sig_GPIO_Port,R_AUX1_sig_Pin, GPIO_PIN_SET);
			ryAuxRelayState = HAL_GPIO_ReadPin(R_AUX1_sig_GPIO_Port,R_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<5));
		}
	}
	return;

}

void rbAuxRelayTurnOn() {
	if (!rbAuxRelayState) {
		while (!rbAuxRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t rbAuxOnTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
					if ((HAL_GetTick() - rbAuxOnTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin))) {
					if ((HAL_GetTick() - rbAuxOnTime) >= 20) {
						break;
					}

				}
			}

			HAL_GPIO_WritePin(R_AUX2_sig_GPIO_Port, R_AUX2_sig_Pin, GPIO_PIN_SET);
			rbAuxRelayState = HAL_GPIO_ReadPin(R_AUX2_sig_GPIO_Port, R_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<7));
		}
	}
	return;

}

void yrAuxRelayTurnOn() {
	if (!yrAuxRelayState) {
		while (!yrAuxRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t yrAuxOnTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
					if ((HAL_GetTick() - yrAuxOnTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin))) {
					if ((HAL_GetTick() - yrAuxOnTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(Y_AUX2_sig_GPIO_Port, Y_AUX2_sig_Pin, GPIO_PIN_SET);
			yrAuxRelayState = HAL_GPIO_ReadPin(Y_AUX2_sig_GPIO_Port, Y_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<3));
		}
	}
	return;

}

void ybAuxRelayTurnOn() {
	if (!ybAuxRelayState) {
		while (!ybAuxRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t ybAuxOnTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
					if ((HAL_GetTick() - ybAuxOnTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin))) {
					if ((HAL_GetTick() - ybAuxOnTime) >= 20) {
						break;
					}

				}
			}

			HAL_GPIO_WritePin(Y_AUX1_sig_GPIO_Port, Y_AUX1_sig_Pin, GPIO_PIN_SET);
			ybAuxRelayState = HAL_GPIO_ReadPin(Y_AUX1_sig_GPIO_Port, Y_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<8));

		}
	}
	return;

}

void byAuxRelayTurnOn() {
	if (!byAuxRelayState) {
		while (!byAuxRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t byAuxOnTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
					if ((HAL_GetTick() - byAuxOnTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin))) {
					if ((HAL_GetTick() - byAuxOnTime) >= 20) {
						break;
					}

				}
			}

			HAL_GPIO_WritePin(B_AUX2_sig_GPIO_Port, B_AUX2_sig_Pin, GPIO_PIN_SET);
			byAuxRelayState = HAL_GPIO_ReadPin(B_AUX2_sig_GPIO_Port, B_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<6));
		}
	}
	return;

}

void brAuxRelayTurnOn() {
	if (!brAuxRelayState) {
		while (!brAuxRelayState) {
			/*to do :Implement the zero crossing switching technique*/

			uint32_t brAuxOnTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
					if ((HAL_GetTick() - brAuxOnTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin))) {
					if ((HAL_GetTick() - brAuxOnTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(B_AUX1_sig_GPIO_Port, B_AUX1_sig_Pin, GPIO_PIN_SET);
			brAuxRelayState = HAL_GPIO_ReadPin(B_AUX1_sig_GPIO_Port, B_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag | (1<<4));
		}
	}
	return;

}

void upsMainRelayOn(){
	if(!upsMainRelayState) {
		while(!upsMainRelayState){
		/*to do :Implement the zero crossing switching technique*/

			uint32_t upsMainOnTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(UPS_ZX_GPIO_Port, UPS_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(UPS_ZX_GPIO_Port, UPS_ZX_Pin)) {
					if ((HAL_GetTick() - upsMainOnTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(UPS_ZX_GPIO_Port, UPS_ZX_Pin))) {
					if ((HAL_GetTick() - upsMainOnTime) >= 20) {
						break;
					}

				}
			}

		HAL_GPIO_WritePin(UPS_main_sig_GPIO_Port, UPS_main_sig_Pin, GPIO_PIN_SET);
		upsMainRelayState = HAL_GPIO_ReadPin(UPS_main_sig_GPIO_Port, UPS_main_sig_Pin);

		//This is event noting to manage the logs
		eventFlag = (eventFlag | (1<<9));
		}


	}
	return;

}

void upsAuxRelayOn(){
	if(!upsAuxRelayState) {
		while(!upsAuxRelayState){
		/*to do :Implement the zero crossing switching technique*/
		HAL_GPIO_WritePin(UPS_AUX_sig_GPIO_Port, UPS_AUX_sig_Pin, GPIO_PIN_SET);
		upsAuxRelayState = HAL_GPIO_ReadPin(UPS_AUX_sig_GPIO_Port, UPS_AUX_sig_Pin);

		//This is event noting to manage the logs
		eventFlag = (eventFlag | (1<<10));
		}


	}
	return;

}

void rMainRelayTurnOff() {
	ryAuxRelayTurnOff();
	rbAuxRelayTurnOff();
	if (rMainRelayState) {
		while (rMainRelayState) {
			/*Implement the zero crossing switching technique*/
			uint32_t rMainOffTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
					if ((HAL_GetTick() - rMainOffTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin))) {
					if ((HAL_GetTick() - rMainOffTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(R_main_sig_GPIO_Port, R_main_sig_Pin, GPIO_PIN_RESET);
			rMainRelayState = HAL_GPIO_ReadPin(R_main_sig_GPIO_Port, R_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~1));
		}
	}
	return;

}

void yMainRelayTurnOff() {
	yrAuxRelayTurnOff();
	ybAuxRelayTurnOff();
	if (yMainRelayState) {
		while (yMainRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t yMainOffTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
					if ((HAL_GetTick() - yMainOffTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin))) {
					if ((HAL_GetTick() - yMainOffTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(Y_main_sig_GPIO_Port, Y_main_sig_Pin, GPIO_PIN_RESET);
			yMainRelayState = HAL_GPIO_ReadPin(Y_main_sig_GPIO_Port, Y_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<1)));
		}
	}
	return;

}

void bMainRelayTurnOff() {
	brAuxRelayTurnOff();
	byAuxRelayTurnOff();
	if (bMainRelayState) {
		while (bMainRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t bMainOffTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
					if ((HAL_GetTick() - bMainOffTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin))) {
					if ((HAL_GetTick() - bMainOffTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(B_main_sig_GPIO_Port, B_main_sig_Pin, GPIO_PIN_RESET);
			bMainRelayState = HAL_GPIO_ReadPin(B_main_sig_GPIO_Port, B_main_sig_Pin);


			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<2)));
		}
	}
	return;

}

void ryAuxRelayTurnOff() {
	if (ryAuxRelayState) {
		while (ryAuxRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t ryAuxOffTime = HAL_GetTick();
						if (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
							while (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
								if ((HAL_GetTick() - ryAuxOffTime) >= 20) {
									break;
								}

							}
						} else {
							while (!(HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin))) {
								if ((HAL_GetTick() - ryAuxOffTime) >= 20) {
									break;
								}

							}
						}
			HAL_GPIO_WritePin(R_AUX1_sig_GPIO_Port,R_AUX1_sig_Pin, GPIO_PIN_RESET);
			ryAuxRelayState = HAL_GPIO_ReadPin(R_AUX1_sig_GPIO_Port,R_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<5)));
		}
	}
	return;

}

void rbAuxRelayTurnOff() {
	if (rbAuxRelayState) {
		while (rbAuxRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t rbAuxOffTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin)) {
					if ((HAL_GetTick() - rbAuxOffTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(R_ZX_GPIO_Port, R_ZX_Pin))) {
					if ((HAL_GetTick() - rbAuxOffTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(R_AUX2_sig_GPIO_Port,R_AUX2_sig_Pin, GPIO_PIN_RESET);
			rbAuxRelayState = HAL_GPIO_ReadPin(R_AUX2_sig_GPIO_Port,R_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<7)));
		}
	}
	return;

}

void yrAuxRelayTurnOff() {
	if (yrAuxRelayState) {
		while (yrAuxRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t yrAuxOffTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
					if ((HAL_GetTick() - yrAuxOffTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin))) {
					if ((HAL_GetTick() - yrAuxOffTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(Y_AUX2_sig_GPIO_Port,Y_AUX2_sig_Pin, GPIO_PIN_RESET);
			yrAuxRelayState = HAL_GPIO_ReadPin(Y_AUX2_sig_GPIO_Port,Y_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<3)));
		}
	}
	return;

}

void ybAuxRelayTurnOff() {
	if (ybAuxRelayState) {
		while (ybAuxRelayState) {
			/*Implement the zero crossing switching technique*/

			uint32_t ybAuxOffTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin)) {
					if ((HAL_GetTick() - ybAuxOffTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(Y_ZX_GPIO_Port, Y_ZX_Pin))) {
					if ((HAL_GetTick() - ybAuxOffTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(Y_AUX1_sig_GPIO_Port,Y_AUX1_sig_Pin, GPIO_PIN_RESET);
			ybAuxRelayState = HAL_GPIO_ReadPin(Y_AUX1_sig_GPIO_Port,Y_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<8)));
		}
	}
	return;

}

void byAuxRelayTurnOff() {
	if (byAuxRelayState) {
		while (byAuxRelayState) {
			/*Implement the zero crossing switching technique*/
			uint32_t byAuxOffTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
					if ((HAL_GetTick() - byAuxOffTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin))) {
					if ((HAL_GetTick() - byAuxOffTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(B_AUX2_sig_GPIO_Port,B_AUX2_sig_Pin, GPIO_PIN_RESET);
			byAuxRelayState = HAL_GPIO_ReadPin(B_AUX2_sig_GPIO_Port,B_AUX2_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<6)));
		}
	}
	return;

}

void brAuxRelayTurnOff() {
	if (brAuxRelayState) {
		while (brAuxRelayState) {
			/*Implement the zero crossing switching technique*/
			uint32_t brAuxOffTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin)) {
					if ((HAL_GetTick() - brAuxOffTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(B_ZX_GPIO_Port, B_ZX_Pin))) {
					if ((HAL_GetTick() - brAuxOffTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(B_AUX1_sig_GPIO_Port,B_AUX1_sig_Pin, GPIO_PIN_RESET);
			brAuxRelayState = HAL_GPIO_ReadPin(B_AUX1_sig_GPIO_Port,B_AUX1_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<4)));
		}
	}
	return;

}

void  upsMainRelayOff(){
	if(upsMainRelayState){
		while (upsMainRelayState){
			/*to do : Implement the zero crossing switching technique*/

			uint32_t upsMainOffTime = HAL_GetTick();
			if (HAL_GPIO_ReadPin(UPS_ZX_GPIO_Port, UPS_ZX_Pin)) {
				while (HAL_GPIO_ReadPin(UPS_ZX_GPIO_Port, UPS_ZX_Pin)) {
					if ((HAL_GetTick() - upsMainOffTime) >= 20) {
						break;
					}

				}
			} else {
				while (!(HAL_GPIO_ReadPin(UPS_ZX_GPIO_Port, UPS_ZX_Pin))) {
					if ((HAL_GetTick() - upsMainOffTime) >= 20) {
						break;
					}

				}
			}
			HAL_GPIO_WritePin(UPS_main_sig_GPIO_Port, UPS_main_sig_Pin, GPIO_PIN_RESET);
			upsMainRelayState = HAL_GPIO_ReadPin(UPS_main_sig_GPIO_Port, UPS_main_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<9)));

		}
	}
	return;
}

void  upsAuxRelayOff(){
	if(upsAuxRelayState){
		while (upsAuxRelayState){
			/*to do : Implement the zero crossing switching technique*/
			HAL_GPIO_WritePin(UPS_AUX_sig_GPIO_Port, UPS_AUX_sig_Pin, GPIO_PIN_RESET);
			upsAuxRelayState = HAL_GPIO_ReadPin(UPS_AUX_sig_GPIO_Port, UPS_AUX_sig_Pin);

			//This is event noting to manage the logs
			eventFlag = (eventFlag & (~(1<<10)));

		}
	}
	return;

}

void rIncomingAuxRelayOff() {
	yrAuxRelayTurnOff();
	brAuxRelayTurnOff();

}

void yIncomingAuxRelayOff() {
	ryAuxRelayTurnOff();
	byAuxRelayTurnOff();

}

void bIncomingAuxRelayOff() {
	rbAuxRelayTurnOff();
	ybAuxRelayTurnOff();

}
