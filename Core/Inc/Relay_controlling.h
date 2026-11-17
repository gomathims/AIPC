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

extern volatile bool rMainRelayState, yMainRelayState, bMainRelayState, ryAuxRelayState, yrAuxRelayState, ybAuxRelayState, byAuxRelayState, rbAuxRelayState, brAuxRelayState, upsMainRelayState, upsAuxRelayState;

extern volatile uint32_t  eventFlag;;

void rMainRelayTurnOn();
void yMainRelayTurnOn();
void bMainRelayTurnOn();
void ryAuxRelayTurnOn();
void yrAuxRelayTurnOn();
void ybAuxRelayTurnOn();
void byAuxRelayTurnOn();
void brAuxRelayTurnOn();
void rbAuxRelayTurnOn();
void upsMainRelayOn();
void upsAuxRelayOn();

void rMainRelayTurnOff();
void yMainRelayTurnOff();
void bMainRelayTurnOff();
void ryAuxRelayTurnOff();
void yrAuxRelayTurnOff();
void ybAuxRelayTurnOff();
void byAuxRelayTurnOff();
void brAuxRelayTurnOff();
void rbAuxRelayTurnOff();
void upsMainRelayOff();
void upsAuxRelayOff();

void rIncomingAuxRelayOff();
void yIncomingAuxRelayOff();
void bIncomingAuxRelayOff();


#endif /* INC_RELAY_CONTROLLING_H_ */
