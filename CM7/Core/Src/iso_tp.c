/**
 ******************************************************************************
 * @file    iso_tp.c
 * @brief   ISO-TP (ISO 15765-2) Transport Layer — CM7 implementation.
 *
 * @note    RX FSM states:  IDLE → (FF) → WAIT_CF → (last CF) → IDLE
 * @note    TX FSM:         SF: one-shot push.
 *                          FF+CF: send FF → block on FC (N_Bs) → send CFs.
 * @note    Thread safety:  All functions run in IsoTpUdsTask context (CM7).
 *                          N_Cr timer callback runs in FreeRTOS timer daemon.
 ******************************************************************************
 */

/* Includes ----------------------------------------------------------------- */
#include "iso_tp.h"
#include "uds_server.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "cmsis_os2.h"       /* osDelay */
#include <string.h>
#include <stdio.h>

/* Private types ------------------------------------------------------------ */
typedef enum {
    ISOTP_RX_IDLE    = 0U,
    ISOTP_RX_WAIT_CF = 1U,
} IsoTp_RxState_e;

/* Private variables -------------------------------------------------------- */

/** RX FSM ----------------------------------------------------------------- */
static IsoTp_RxState_e s_rxState      = ISOTP_RX_IDLE;
static uint8_t         s_rxBuf[ISOTP_MAX_PDU_LEN];
static uint16_t        s_rxExpectedLen = 0U;
static uint16_t        s_rxReceivedLen = 0U;
static uint8_t         s_rxExpectedSN  = 0U;
static TimerHandle_t   s_ncrTimer      = NULL;

/** Queue handles (injected via IsoTp_Init) -------------------------------- */
static QueueHandle_t   s_isoTpRxQueue  = NULL;  /**< RX frames + FC for TX wait */
static QueueHandle_t   s_cm7IpcTxQueue = NULL;  /**< Outgoing frames → CM4      */

/* Private helpers ---------------------------------------------------------- */

/**
 * @brief  Build and enqueue an outgoing CAN frame to s_cm7IpcTxQueue.
 *         CM7 IpcTxTask drains this queue → OPENAMP_send → CM4 → FDCAN1 TX.
 */
static void pushTxFrame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    IpcFrame_t f;
    memset(&f, ISOTP_PAD_BYTE, sizeof(f));
    f.id       = id;
    f.dlc      = dlc;
    f.src_bus  = IPC_BUS_UDS_TX;
    f.timestamp = 0U;  /* filled by CM4 on TX */
    memcpy(f.data, data, (size_t)dlc);
    /* Pad remaining data bytes with 0xCC (Bosch convention) */
    if (dlc < 8U) {
        memset(&f.data[dlc], ISOTP_PAD_BYTE, (size_t)(8U - dlc));
    }
    if (xQueueSend(s_cm7IpcTxQueue, &f, pdMS_TO_TICKS(10U)) != pdTRUE) {
        printf("[ISO-TP][ERR] CM7 IpcTxQueue full — frame 0x%03lX dropped\r\n",
               (unsigned long)id);
    }
}

/**
 * @brief  Build and send a Flow Control frame (ECU → Tester).
 *         Frame: [FS | BS | STmin | 0xCC×5]
 * @param  flowStatus  ISOTP_FC_CTS (0x30) or ISOTP_FC_OVERFLOW (0x32).
 */
static void sendFC(uint8_t flowStatus)
{
    uint8_t d[8];
    memset(d, ISOTP_PAD_BYTE, sizeof(d));
    d[0] = flowStatus;
    d[1] = ISOTP_FC_BS;
    d[2] = ISOTP_FC_STMIN;
    pushTxFrame(UDS_RESPONSE_ID, d, 8U);
}

/* Private FSM handlers ----------------------------------------------------- */

/**
 * @brief  Handle Single Frame: deliver payload directly to UDS stack.
 */
static void handleSF(const IpcFrame_t *frame)
{
    uint8_t sf_dl = frame->data[0] & 0x0FU;
    if (sf_dl == 0U || sf_dl > ISOTP_SF_MAX_PAYLOAD) {
        printf("[ISO-TP][ERR] SF invalid DL=%u\r\n", (unsigned)sf_dl);
        return;
    }
    /* Reset any stale RX session */
    if (s_rxState == ISOTP_RX_WAIT_CF) {
        printf("[ISO-TP][WARN] SF during WAIT_CF — aborting prev RX\r\n");
        xTimerStop(s_ncrTimer, 0U);
        s_rxState = ISOTP_RX_IDLE;
    }
    /* Deliver [SID, data...] directly — no buffering needed */
    Uds_ProcessRequest(&frame->data[1], (uint16_t)sf_dl);
}

/**
 * @brief  Handle First Frame: allocate RX session, send FC, start N_Cr timer.
 */
static void handleFF(const IpcFrame_t *frame)
{
    if (s_rxState == ISOTP_RX_WAIT_CF) {
        printf("[ISO-TP][WARN] FF during WAIT_CF — aborting prev RX\r\n");
        xTimerStop(s_ncrTimer, 0U);
    }

    /* Extract FF_DL from bits [11:0] across byte 0 and byte 1 */
    uint16_t ff_dl = ((uint16_t)(frame->data[0] & 0x0FU) << 8U)
                   |  (uint16_t) frame->data[1];

    if (ff_dl < 8U) {
        printf("[ISO-TP][ERR] FF_DL=%u too small (use SF)\r\n", (unsigned)ff_dl);
        return;
    }
    if (ff_dl > ISOTP_MAX_PDU_LEN) {
        printf("[ISO-TP][ERR] FF_DL=%u > %u — sending FC Overflow\r\n",
               (unsigned)ff_dl, (unsigned)ISOTP_MAX_PDU_LEN);
        sendFC(ISOTP_FC_OVERFLOW);
        s_rxState = ISOTP_RX_IDLE;
        return;
    }

    /* Copy first 6 payload bytes (data[2..7]) into rx buffer */
    memcpy(s_rxBuf, &frame->data[2], 6U);
    s_rxExpectedLen = ff_dl;
    s_rxReceivedLen = 6U;
    s_rxExpectedSN  = 1U;
    s_rxState       = ISOTP_RX_WAIT_CF;

    /* Send FC: CTS, BS=0, STmin=0 */
    sendFC(ISOTP_FC_CTS);

    /* Start N_Cr timer — must receive CF within 75 ms */
    xTimerStart(s_ncrTimer, 0U);

    printf("[ISO-TP] FF: total=%u, rcvd=6, FC sent\r\n", (unsigned)ff_dl);
}

/**
 * @brief  Handle Consecutive Frame: verify SN, accumulate, deliver when done.
 */
static void handleCF(const IpcFrame_t *frame)
{
    if (s_rxState != ISOTP_RX_WAIT_CF) {
        printf("[ISO-TP][ERR] CF outside WAIT_CF context — ignored\r\n");
        return;
    }

    uint8_t sn = frame->data[0] & 0x0FU;
    if (sn != s_rxExpectedSN) {
        printf("[ISO-TP][ERR] SN mismatch: exp=%u got=%u — abort RX\r\n",
               (unsigned)s_rxExpectedSN, (unsigned)sn);
        xTimerStop(s_ncrTimer, 0U);
        s_rxState       = ISOTP_RX_IDLE;
        s_rxExpectedLen = 0U;
        s_rxReceivedLen = 0U;
        s_rxExpectedSN  = 0U;
        return;
    }

    /* Reset N_Cr — each valid CF extends the deadline */
    xTimerReset(s_ncrTimer, 0U);

    /* Copy up to 7 data bytes (data[1..7]) */
    uint16_t remaining = s_rxExpectedLen - s_rxReceivedLen;
    uint16_t copy_len  = (remaining > 7U) ? 7U : remaining;
    memcpy(&s_rxBuf[s_rxReceivedLen], &frame->data[1], (size_t)copy_len);
    s_rxReceivedLen += copy_len;

    /* Wraparound SN: 1→2→…→15→0→1→… */
    s_rxExpectedSN = (uint8_t)((s_rxExpectedSN + 1U) & 0x0FU);

    /* Check if assembly is complete */
    if (s_rxReceivedLen >= s_rxExpectedLen) {
        xTimerStop(s_ncrTimer, 0U);

        uint16_t total  = s_rxExpectedLen;
        s_rxState       = ISOTP_RX_IDLE;
        s_rxExpectedLen = 0U;
        s_rxReceivedLen = 0U;
        s_rxExpectedSN  = 0U;

        printf("[ISO-TP] RX complete: %u bytes → UDS\r\n", (unsigned)total);
        Uds_ProcessRequest(s_rxBuf, total);
    }
}

/* N_Cr timeout callback ---------------------------------------------------- */

/**
 * @brief  FreeRTOS timer callback — fired when N_Cr expires (75 ms gap in CFs).
 * @note   Runs in timer daemon task context. No FreeRTOS blocking calls.
 */
static void ncrTimeoutCb(TimerHandle_t xTimer)
{
    (void)xTimer;
    /* Log via direct register UART — safe from any task including timer daemon */
    printf("[ISO-TP][ERR] N_Cr timeout: exp_SN=%u rcvd=%u/%u → IDLE\r\n",
           (unsigned)s_rxExpectedSN,
           (unsigned)s_rxReceivedLen,
           (unsigned)s_rxExpectedLen);
    s_rxState       = ISOTP_RX_IDLE;
    s_rxExpectedLen = 0U;
    s_rxReceivedLen = 0U;
    s_rxExpectedSN  = 0U;
}

/* Public API --------------------------------------------------------------- */

void IsoTp_Init(QueueHandle_t isoTpRxQueue, QueueHandle_t cm7IpcTxQueue)
{
    configASSERT(isoTpRxQueue   != NULL);
    configASSERT(cm7IpcTxQueue  != NULL);

    s_isoTpRxQueue  = isoTpRxQueue;
    s_cm7IpcTxQueue = cm7IpcTxQueue;

    memset(s_rxBuf, 0, sizeof(s_rxBuf));
    s_rxState       = ISOTP_RX_IDLE;
    s_rxExpectedLen = 0U;
    s_rxReceivedLen = 0U;
    s_rxExpectedSN  = 0U;

    /* One-shot N_Cr timer (75 ms) — reset on each valid CF */
    s_ncrTimer = xTimerCreate("NcrTmr",
                               pdMS_TO_TICKS(ISOTP_TIMEOUT_N_CR_MS),
                               pdFALSE,   /* one-shot */
                               NULL,
                               ncrTimeoutCb);
    configASSERT(s_ncrTimer != NULL);

    printf("[ISO-TP] Init OK. MaxPDU=%u N_Cr=%ums N_Bs=%ums\r\n",
           (unsigned)ISOTP_MAX_PDU_LEN,
           (unsigned)ISOTP_TIMEOUT_N_CR_MS,
           (unsigned)ISOTP_TIMEOUT_N_BS_MS);
}

void IsoTp_ProcessRxFrame(const IpcFrame_t *frame)
{
    if (frame == NULL || frame->dlc == 0U) {
        return;
    }

    uint8_t pci_type = (frame->data[0] >> 4U) & 0x0FU;

    switch (pci_type) {
        case ISOTP_PCI_SF:
            handleSF(frame);
            break;
        case ISOTP_PCI_FF:
            handleFF(frame);
            break;
        case ISOTP_PCI_CF:
            handleCF(frame);
            break;
        case ISOTP_PCI_FC:
            /* FC received during TX wait — handled by IsoTp_StartTx().
             * If we reach here (no TX in progress), it's unexpected. */
            printf("[ISO-TP][WARN] Unexpected FC received (no TX in progress)\r\n");
            break;
        default:
            printf("[ISO-TP][ERR] Unknown PCI=0x%X — frame dropped\r\n",
                   (unsigned)pci_type);
            break;
    }
}

void IsoTp_StartTx(const uint8_t *pdu, uint16_t length)
{
    if (pdu == NULL || length == 0U || length > ISOTP_MAX_PDU_LEN) {
        printf("[ISO-TP][ERR] StartTx: invalid args len=%u\r\n", (unsigned)length);
        return;
    }

    /* ── Single Frame path (≤ 7 bytes) ──────────────────────────────────── */
    if (length <= ISOTP_SF_MAX_PAYLOAD) {
        uint8_t d[8];
        memset(d, ISOTP_PAD_BYTE, sizeof(d));
        d[0] = (uint8_t)length;           /* PCI: 0x0N, N = DL */
        memcpy(&d[1], pdu, (size_t)length);
        pushTxFrame(UDS_RESPONSE_ID, d, 8U);
        printf("[ISO-TP] TX SF: %u bytes\r\n", (unsigned)length);
        return;
    }

    /* ── Multi-Frame path (> 7 bytes) ───────────────────────────────────── */
    /* 1. Send First Frame */
    uint8_t ff[8];
    ff[0] = (uint8_t)(0x10U | ((length >> 8U) & 0x0FU));
    ff[1] = (uint8_t)(length & 0xFFU);
    memcpy(&ff[2], pdu, 6U);
    pushTxFrame(UDS_RESPONSE_ID, ff, 8U);
    printf("[ISO-TP] TX FF: total=%u, sent=6\r\n", (unsigned)length);

    /* 2. Wait for Flow Control (N_Bs = 75 ms) */
    IpcFrame_t fc_frame;
    if (xQueueReceive(s_isoTpRxQueue,
                      &fc_frame,
                      pdMS_TO_TICKS(ISOTP_TIMEOUT_N_BS_MS)) != pdTRUE) {
        printf("[ISO-TP][ERR] N_Bs timeout — no FC received, TX aborted\r\n");
        return;
    }

    /* Validate FC PCI */
    uint8_t pci = (fc_frame.data[0] >> 4U) & 0x0FU;
    if (pci != ISOTP_PCI_FC) {
        printf("[ISO-TP][WARN] Expected FC, got PCI=0x%X — TX aborted\r\n",
               (unsigned)pci);
        /* Push non-FC frame back to front for IsoTpUdsTask to process */
        xQueueSendToFront(s_isoTpRxQueue, &fc_frame, 0U);
        return;
    }

    uint8_t fs    = fc_frame.data[0] & 0x0FU;
    uint8_t stmin = fc_frame.data[2];

    if (fs == 0x02U) {
        printf("[ISO-TP][ERR] FC Overflow from tester — TX aborted\r\n");
        return;
    }
    if (fs != 0x00U) {
        printf("[ISO-TP][WARN] FC FlowStatus=0x%X not handled — TX aborted\r\n",
               (unsigned)fs);
        return;
    }
    /* fc_frame.data[1] = BS; for M6 we treat BS=0 (unlimited).
     * Full BS>0 block handling deferred to M8 unit tests. */

    printf("[ISO-TP] FC CTS rcvd: BS=%u STmin=%u — sending CFs\r\n",
           (unsigned)fc_frame.data[1], (unsigned)stmin);

    /* 3. Send Consecutive Frames */
    uint16_t sentLen = 6U;
    uint8_t  cfSN    = 1U;

    while (sentLen < length) {
        uint16_t remaining = length - sentLen;
        uint16_t copy_len  = (remaining > 7U) ? 7U : remaining;

        uint8_t cf[8];
        memset(cf, ISOTP_PAD_BYTE, sizeof(cf));
        cf[0] = (uint8_t)(0x20U | (cfSN & 0x0FU));
        memcpy(&cf[1], &pdu[sentLen], (size_t)copy_len);
        pushTxFrame(UDS_RESPONSE_ID, cf, 8U);

        sentLen += copy_len;
        cfSN     = (uint8_t)((cfSN + 1U) & 0x0FU);

        /* Apply STmin delay between CFs (ISO 15765-2 §6.5.5.5) */
        if (stmin > 0U && stmin <= 0x7FU) {
            osDelay((uint32_t)stmin);        /* stmin = ms directly      */
        } else if (stmin >= 0xF1U && stmin <= 0xF9U) {
            osDelay(1U);                     /* 100–900 µs → 1 ms tick   */
        }
        /* 0x80–0xF0: reserved, treat as 0 (no delay) */
    }

    printf("[ISO-TP] TX done: %u bytes in %u CF(s)\r\n",
           (unsigned)length, (unsigned)(cfSN - 1U));
}
