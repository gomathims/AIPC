/*
 * Relay_controlling.h
 *
 *  Created on: Jun 25, 2026
 *      Author: EVIQtech
 */

#include <stdbool.h>
#include <stdint.h>

#ifndef INC_RELAY_CONTROLLING_H_
#define INC_RELAY_CONTROLLING_H_

extern volatile bool RmainRelayState, YmainRelayState, BmainRelayState, RYauxRelayState, YRauxRelayState, YBauxRelayState, BYauxRelayState, RBauxRelayState, BRauxRelayState, UPSmainRelayState, UPSauxRelayState;

extern volatile uint32_t  eventFlag;;

void RmainRelayTurnON();
void YmainRelayTurnON();
void BmainRelayTurnON();
void RYauxRelayTurnON();
void YRauxRelayTurnON();
void YBauxRelayTurnON();
void BYauxRelayTurnON();
void BRauxRelayTurnON();
void RBauxRelayTurnON();
void UPSmainRelayON();
void UPSauxRelayON();

void RmainRelayTurnOFF();
void YmainRelayTurnOFF();
void BmainRelayTurnOFF();
void RYauxRelayTurnOFF();
void YRauxRelayTurnOFF();
void YBauxRelayTurnOFF();
void BYauxRelayTurnOFF();
void BRauxRelayTurnOFF();
void RBauxRelayTurnOFF();
void UPSmainRelayOFF();
void UPSauxRelayOFF();

void RincomingAuxRelayOFF();
void YincomingAuxRelayOFF();
void BincomingAuxRelayOFF();


#endif /* INC_RELAY_CONTROLLING_H_ */
