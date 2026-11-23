/*
 * OTAupdate.c
 *
 *  Created on: Oct 12, 2026
 *      Author: EVIQtech
 */



#include "OTAupdate.h"
#include <string.h>

// Handles + the app's idle-DMA buffer live in main.c.
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef  hdma_usart3_rx;
extern uint8_t            Uart3RXbuffer[];
#define SU_IDLE_BUF_SIZE  256          // MUST match BUF_SIZE in main.c

// ---- STM32G474RE flash geometry: 512 KB, dual bank, 2 KB pages -------------
#define OTA_PAGE_SIZE       2048UL
#define OTA_PAGES_PER_BANK  128U
#define OTA_BANK_SIZE       (OTA_PAGE_SIZE * OTA_PAGES_PER_BANK)   // 256 KB
#define OTA_APP_BASE        0x08000000UL   // active bank is always mapped here
#define OTA_INACTIVE_BASE   0x08040000UL   // inactive bank is mapped here (BFB2 swap)

static volatile bool otaReqPending   = false;
static volatile bool otaApplyPending = false;
static volatile bool otaBusy         = false;

// Set only after a staged image has passed its CRC32 check against flash.
// RAM-only on purpose: after any reset the image must be re-staged before it
// can be applied, so a stale or half-written bank can never be swapped in.
static bool otaStagedOk = false;

bool otaUpdating(void) { return otaBusy; }

// ---------------------------------------------------------------------------
// CRC-32 (IEEE 802.3 / zlib: reflected, poly 0xEDB88320, init and final XOR
// 0xFFFFFFFF). Must match crc32Update() in the ESP's stm_ota.cpp.
// Bitwise: ~50 ms for a full 256 KB bank at 170 MHz, no table in RAM/flash.
// ---------------------------------------------------------------------------
static uint32_t otaCrc32Update(uint32_t crc, const uint8_t *p, uint32_t n) {
    while (n--) {
        crc ^= *p++;
        for (uint8_t k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Signature detection — runs in ISR context, so it only sets a flag.
// ---------------------------------------------------------------------------
static bool containsSeq(const uint8_t *buf, uint16_t size, const uint8_t *seq) {
    if (size < SU_SEQ_LEN) return false;
    for (uint16_t i = 0; i + SU_SEQ_LEN <= size; i++) {
        if (memcmp(&buf[i], seq, SU_SEQ_LEN) == 0) return true;
    }
    return false;
}

void otaStartRxIncomming(const uint8_t *buf, uint16_t size) {
    static const uint8_t req[SU_SEQ_LEN]   = SU_REQ_SEQ;
    static const uint8_t apply[SU_SEQ_LEN] = SU_APPLY_SEQ;
    if (containsSeq(buf, size, req))   otaReqPending   = true;
    if (containsSeq(buf, size, apply)) otaApplyPending = true;
}

// ---------------------------------------------------------------------------
// Low-level UART
// ---------------------------------------------------------------------------
static void otaTx(uint8_t b) { HAL_UART_Transmit(&huart3, &b, 1, 100); }

// ---- Circular-DMA reception for the bulk image ----------------------------
// While programming a flash doubleword the CPU can't poll the UART, so bytes
// must keep arriving in the background. Circular DMA into a ring buffer does
// that; we drain it into flash at our own pace.
#define SU_RING 1024
static uint8_t  s_ring[SU_RING];
static uint16_t s_tail;

static void otaRxStart(void) {
    HAL_UART_DMAStop(&huart3);
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    HAL_DMA_DeInit(&hdma_usart3_rx);
    HAL_DMA_Init(&hdma_usart3_rx);
    s_tail = 0;
    HAL_UART_Receive_DMA(&huart3, s_ring, SU_RING);
}

static void otaRxStopRestoreIdle(void) {
    HAL_UART_DMAStop(&huart3);
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    HAL_DMA_DeInit(&hdma_usart3_rx);
    HAL_DMA_Init(&hdma_usart3_rx);
    // Back to the app's normal idle-DMA reception for 0xCC protection frames.
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Uart3RXbuffer, SU_IDLE_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

static uint16_t otaRxHead(void) {
    return (uint16_t)(SU_RING - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx));
}

// Pull exactly n bytes into dst within timeoutMs. Returns false on timeout.
static bool otaRxRead(uint8_t *dst, uint32_t n, uint32_t timeoutMs) {
    uint32_t got = 0, start = HAL_GetTick();
    while (got < n) {
        uint16_t head = otaRxHead();
        while (s_tail != head && got < n) {
            dst[got++] = s_ring[s_tail];
            s_tail = (uint16_t)((s_tail + 1) % SU_RING);
        }
        if (got < n) {
            if (HAL_GetTick() - start > timeoutMs) return false;
            // HAL_IWDG_Refresh(&hiwdg);   // <-- uncomment if you run the IWDG
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Flash — dual-bank, BFB2 swap
//
// !!! VERIFY ON YOUR SILICON BEFORE TRUSTING IT WITH A REAL IMAGE !!!
// This assumes the G4 BFB2 swap keeps the ACTIVE bank at 0x08000000 and the
// INACTIVE bank at 0x08040000. Run the self-test in main() first (see the
// integration notes) and confirm a write@0x08040000 reads back correctly.
// ---------------------------------------------------------------------------
static bool otaRunningFromBank2(void) {
    return READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) != 0U;
}
static uint32_t suInactiveBankId(void) {
    // return (FLASH->OPTR & FLASH_OPTR_BFB2) ? FLASH_BANK_1 : FLASH_BANK_2;
    return otaRunningFromBank2() ? FLASH_BANK_1 : FLASH_BANK_2;
}

//********************************************Here there is something problem in this portion!!!
static bool suEraseInactive(void) {

   FLASH_EraseInitTypeDef er = {0};
   er.TypeErase = FLASH_TYPEERASE_PAGES;
   er.Banks     = suInactiveBankId();
   er.Page      = 0;
   er.NbPages   = OTA_PAGES_PER_BANK;
   uint32_t pageErr = 0;
   HAL_FLASH_Unlock();
   HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&er, &pageErr);
   HAL_FLASH_Lock();
   return st == HAL_OK;
}


// ---------------------------------------------------------------------------
// Stage 1 — receive the image into the inactive bank
// ---------------------------------------------------------------------------
static bool otaReceiveImage(void) {
    // The inactive bank is about to be erased, so whatever was staged is gone.
    otaStagedOk = false;

    // Erase first — this is the multi-second wait the ESP allows before 0xAB.

    if (!suEraseInactive()) return false;


    otaRxStart();
    otaTx(SU_READY);                 // 0xAB: erased, send it

    uint8_t lenb[4];
    if (!otaRxRead(lenb, 4, 3000)) { otaRxStopRestoreIdle(); return false; }
    uint32_t total = (uint32_t)lenb[0] | ((uint32_t)lenb[1] << 8) |
                     ((uint32_t)lenb[2] << 16) | ((uint32_t)lenb[3] << 24);
    if (total == 0 || total > OTA_BANK_SIZE) { otaRxStopRestoreIdle(); return false; }

    HAL_FLASH_Unlock();
    uint32_t addr = OTA_INACTIVE_BASE;
    uint32_t done = 0;
    bool ok = true;

    while (done < total) {
        uint8_t db[8];
        uint32_t chunk = (total - done >= 8) ? 8 : (total - done);
        if (!otaRxRead(db, chunk, 3000)) { ok = false; break; }
        for (uint32_t k = chunk; k < 8; k++) db[k] = 0xFF;   // pad final doubleword
        uint64_t dw;
        memcpy(&dw, db, 8);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dw) != HAL_OK) { ok = false; break; }
        addr += 8;
        done += chunk;
        // HAL_IWDG_Refresh(&hiwdg);   // <-- uncomment if you run the IWDG
    }

    // Trailer: CRC32 of the `total` image bytes (not the 0xFF padding), LE.
    uint8_t crcb[4];
    if (ok && !otaRxRead(crcb, 4, 3000)) ok = false;

    HAL_FLASH_Lock();
    otaRxStopRestoreIdle();
    if (!ok) return false;

    // Verify against what actually landed in flash, not what came over the
    // wire: this catches ring-buffer overruns, UART corruption AND bad writes.
    uint32_t expected = (uint32_t)crcb[0] | ((uint32_t)crcb[1] << 8) |
                        ((uint32_t)crcb[2] << 16) | ((uint32_t)crcb[3] << 24);
    uint32_t actual = otaCrc32Update(0xFFFFFFFFUL,
                                     (const uint8_t *)OTA_INACTIVE_BASE, total) ^ 0xFFFFFFFFUL;
    if (actual != expected) return false;

    otaStagedOk = true;
    return true;
}

// ---------------------------------------------------------------------------
// Stage 2 — apply: ack, flip BFB2 to the bank we just wrote, reset.
// ---------------------------------------------------------------------------
// Returns only if refused (nothing verified is staged); otherwise resets.
static void otaApply(void) {
    if (!otaStagedOk) {
        otaTx(SU_DONE_FAIL);         // 0xAE: refuse to boot an unverified bank
        return;
    }
    otaTx(SU_DONE_OK);               // 0xAC BEFORE the reset (ESP needs this first)
    HAL_Delay(50);                  // let the byte physically leave

    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    FLASH_OBProgramInitTypeDef ob = {0};
    ob.OptionType = OPTIONBYTE_USER;
    ob.USERType   = OB_USER_BFB2;
    // Currently running bank1 (BFB2=0) -> we wrote bank2 -> ENABLE BFB2 to boot it.
    // Currently running bank2 (BFB2=1) -> we wrote bank1 -> DISABLE BFB2 to boot it.
    ob.USERConfig = (FLASH->OPTR & FLASH_OPTR_BFB2) ? OB_BFB2_DISABLE : OB_BFB2_ENABLE;
    HAL_FLASHEx_OBProgram(&ob);
    HAL_FLASH_OB_Launch();          // reloads option bytes -> RESET, boots the other bank
    // never reached
    for (;;) { }
}

// ---------------------------------------------------------------------------
// Main-loop service
// ---------------------------------------------------------------------------
void otaIncomingCheck(void) {
    if (otaReqPending) {
        otaReqPending = false;
        otaBusy = true;
        bool ok = otaReceiveImage();
        otaTx(ok ? SU_DONE_OK : SU_DONE_FAIL);   // 0xAC / 0xAE
        otaBusy = false;
    }
    if (otaApplyPending) {
        otaApplyPending = false;
        otaBusy = true;
        otaApply();                  // returns only if refused
        otaBusy = false;
    }
}


