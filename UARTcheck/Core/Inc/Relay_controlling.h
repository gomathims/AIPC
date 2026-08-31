/*
 * Relay_controlling.h
 *
 *  Created on: Jun 25, 2026
 *      Author: EVIQtech
 */

#ifndef INC_RELAY_CONTROLLING_H_
#define INC_RELAY_CONTROLLING_H_
#include <stdbool.h>
extern volatile bool RmainRelayState, YmainRelayState, BmainRelayState, RYauxRelayState, YRauxRelayState, YBauxRelayState, BYauxRelayState, RBauxRelayState, BRauxRelayState, UPSRelayState;


void RmainRelayTurnON();
void YmainRelayTurnON();
void BmainRelayTurnON();
void RYauxRelayTurnON();
void YRauxRelayTurnON();
void YBauxRelayTurnON();
void BYauxRelayTurnON();
void BRauxRelayTurnON();
void RBauxRelayTurnON();

void RmainRelayTurnOFF();
void YmainRelayTurnOFF();
void BmainRelayTurnOFF();
void RYauxRelayTurnOFF();
void YRauxRelayTurnOFF();
void YBauxRelayTurnOFF();
void BYauxRelayTurnOFF();
void BRauxRelayTurnOFF();
void RBauxRelayTurnOFF();

void RincomingAuxRelayOFF();
void YincomingAuxRelayOFF();
void BincomingAuxRelayOFF();


#endif /* INC_RELAY_CONTROLLING_H_ */
