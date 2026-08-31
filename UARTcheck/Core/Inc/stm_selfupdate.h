/*
 * STM_SELFUPDATE.h
 *
 *  Created on: Aug 7, 2026
 *      Author: EVIQtech
 */






#ifndef INC_STM_SELFUPDATE_H_
#define INC_STM_SELFUPDATE_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// STM32G474 dual-bank self-update over USART3, driven by the ESP.
// Wire protocol MUST match the ESP side (src/stm_ota.h on the ESP):
//
//   ESP -> STM   AA 55 A5 5A            request  (4-byte signature)
//   STM -> ESP   0xAB                   ready: inactive bank erased, send it
//   ESP -> STM   len[4] little-endian   image length
//   ESP -> STM   <len bytes>            raw firmware, streamed
//   STM -> ESP   0xAC | 0xAE            stored OK / failed
//
//   ESP -> STM   BB 44 B4 4B            apply   (4-byte signature)
//   STM -> ESP   0xAC                   accepted (sent BEFORE the swap/reset)
//   STM swaps BFB2 and resets -> new bank boots -> app resumes "STM" frames
// ---------------------------------------------------------------------------

#define SU_REQ_SEQ    { 0xAA, 0x55, 0xA5, 0x5A }
#define SU_APPLY_SEQ  { 0xBB, 0x44, 0xB4, 0x4B }
#define SU_SEQ_LEN    4
#define SU_READY      0xAB
#define SU_DONE_OK    0xAC
#define SU_DONE_FAIL  0xAE

// Call from the USART3 RX-event handler (ISR) with the burst just received.
// ONLY detects the two command signatures and raises a flag — no flash work.
void suOnRxBurst(const uint8_t *buf, uint16_t size);

// Call once per main-loop iteration. Runs the (blocking) receive or apply when
// a signature was seen. apply() never returns (it resets the MCU).
void suService(void);

// True while an update is in progress (so the app can skip metering/TX).
bool suBusy(void);

#endif /* INC_STM_SELFUPDATE_H_ */
