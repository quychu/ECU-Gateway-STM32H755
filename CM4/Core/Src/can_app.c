/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can_app.c
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "can_app.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "queue.h"

/* Private defines -----------------------------------------------------------*/

/**
 * @brief RX queue depth.
 *        At 500 kbps max ~500 frames/s; CanEngineTask drains every ~100 ms
 *        => ~50 frames max backlog. 16 slots is conservative for M4 (1 fps TX).
 */
#define CAN_ENGINE_RX_QUEUE_DEPTH   16U

/* Private variables ---------------------------------------------------------*/

/**
 * @brief RX frame queue: HAL_FDCAN_RxFifo0Callback (ISR) -> CanEngineTask.
 *        Created inside CanEngine_Init(). Module-internal only.
 */
static QueueHandle_t s_rxQueue    = NULL;
static QueueHandle_t s_ipcTxQueue = NULL;
static QueueHandle_t s_ipcRxQueue = NULL;   /**< CM7 → CM4 UDS TX commands */

HAL_StatusTypeDef CanEngine_Init(void)
{
    s_rxQueue = xQueueCreate(CAN_ENGINE_RX_QUEUE_DEPTH, sizeof(CanFrame_t));
    if (s_rxQueue == NULL) return HAL_ERROR;

    s_ipcTxQueue = xQueueCreate(8U, sizeof(CanFrame_t));
    if (s_ipcTxQueue == NULL) return HAL_ERROR;

    s_ipcRxQueue = xQueueCreate(8U, sizeof(CanFrame_t));  /* CM7→CM4 UDS TX */
    if (s_ipcRxQueue == NULL) return HAL_ERROR;

    /* ── Filter 0: range 0x100–0x1FF (gateway routing frames) ── */
    FDCAN_FilterTypeDef sFilter = {0};
    sFilter.IdType       = FDCAN_STANDARD_ID;
    sFilter.FilterIndex  = 0U;
    sFilter.FilterType   = FDCAN_FILTER_RANGE;
    sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilter.FilterID1    = CAN_ENGINE_FILTER_ID_LOW;
    sFilter.FilterID2    = CAN_ENGINE_FILTER_ID_HIGH;

    /* Cấu hình cho FDCAN1 */
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter) != HAL_OK) return HAL_ERROR;
    /* Cấu hình cho FDCAN2 */
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilter) != HAL_OK) return HAL_ERROR;

    /* ── Filter 1 (UDS 0x7E0) removed — requires StdFiltersNbr=2 which IOC resets.
     *    Instead: FDCAN1 global filter accepts non-matching std IDs into FIFO0.
     *    0x7E0 misses filter 0 (range 0x100-0x1FF) → caught by non-match accept.
     *    SW side: Gw_RouteFrame() already skips 0x7E0 (no routing loop).
     *    IPC side: all frames forwarded to CM7 for UDS/IDS processing. OK.
     *    TODO M6-cleanup: set StdFiltersNbr=2 in IOC → re-enable filter 1.    ── */

    /* Global filter FDCAN1: non-matching std IDs → FIFO0 (catches 0x7E0 UDS) */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                  FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    /* Global filter FDCAN2: reject non-matching (no UDS on Bus B) */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                  FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    /* --- Kích hoạt FDCAN1 --- */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) return HAL_ERROR;
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0U);

    /* --- Kích hoạt FDCAN2 --- */
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) return HAL_ERROR;
    /* FDCAN2 RX: uncomment khi FDCAN2 nối bus riêng (dual-bus setup).
     * Trên shared bus (test bench), giữ comment để tránh routing loop.   */
//    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
//    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF,              0U);
    return HAL_OK;
}

/**
 * @brief  CanEngine_SendTestFrame -- transmit a Classic CAN heartbeat frame.
 *
 * @param  counter  Sequence counter in data[0], incremented by caller.
 *                  Allows SavvyCAN to verify no frames are dropped.
 *
 * @note   HW path (RM0399 Ch.59):
 *           HAL_FDCAN_AddMessageToTxFifoQ checks FDCAN_TXFQS.TFQF (full flag).
 *           If not full: writes T0 (ID/XTD/RTR/ESI) + T1 (DLC/BRS/FDF/EFC/MM)
 *           + data words into Message RAM Tx buffer[TFQPI].
 *           Sets FDCAN_TXBAR.AR[n] = 1 to arm the transmit request.
 *
 * @retval HAL_OK / HAL_ERROR (Tx FIFO full = HAL_ERROR)
 */
HAL_StatusTypeDef CanEngine_SendFrame(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t counter)
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t txData[CAN_ENGINE_DLC_MAX] = { counter, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11 };

    txHeader.Identifier          = id;
    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData);
}
/**
 * @brief  CanEngine_GetRxQueue -- return the RX queue handle.
 *
 * @note   Returns NULL if called before CanEngine_Init() completes.
 *
 * @retval FreeRTOS QueueHandle_t
 */
QueueHandle_t CanEngine_GetRxQueue(void)
{
    return s_rxQueue;
}

QueueHandle_t CanEngine_GetIpcTxQueue(void)
{
    return s_ipcTxQueue;
}

QueueHandle_t CanEngine_GetIpcRxQueue(void)
{
    return s_ipcRxQueue;
}

/**
 * @brief  CanEngine_SendUdsResponse — TX ISO-TP frame on FDCAN1 (response bus).
 * @note   HW path (RM0399 Ch.59 §59.4.9):
 *           HAL_FDCAN_AddMessageToTxFifoQ checks FDCAN_TXFQS.TFQF (bit 21).
 *           Writes T0 (ID, XTD=0, RTR=0), T1 (DLC=8, FDF=0, BRS=0) and
 *           8 data bytes into Message RAM Tx buffer at offset FDCAN_TXFQS.TFQPI.
 *           Sets FDCAN_TXBAR.AR[TFQPI] = 1 to schedule transmission.
 */
HAL_StatusTypeDef CanEngine_SendUdsResponse(const CanFrame_t *frame)
{
    FDCAN_TxHeaderTypeDef txHdr = {0};
    txHdr.Identifier          = frame->id;        /* should be UDS_RESPONSE_ID */
    txHdr.IdType              = FDCAN_STANDARD_ID;
    txHdr.TxFrameType         = FDCAN_DATA_FRAME;
    txHdr.DataLength          = FDCAN_DLC_BYTES_8; /* ISO-TP always pads to 8  */
    txHdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHdr.BitRateSwitch       = FDCAN_BRS_OFF;
    txHdr.FDFormat            = FDCAN_CLASSIC_CAN;
    txHdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHdr.MessageMarker       = 0U;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHdr, (uint8_t *)frame->data);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;

    CanFrame_t frame = {0};
    FDCAN_RxHeaderTypeDef rxHdr;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHdr, frame.data) == HAL_OK) {
        frame.id        = rxHdr.Identifier;
        frame.dlc       = (uint8_t)(rxHdr.DataLength >> 16U) & 0x0FU;
        if (frame.dlc == 0U) { frame.dlc = (uint8_t)(rxHdr.DataLength & 0x0FU); }
        frame.src_bus   = (hfdcan->Instance == FDCAN1) ? GW_BUS_FDCAN1 : GW_BUS_FDCAN2;
        frame.timestamp = HAL_GetTick();

        BaseType_t xIpcWoken = pdFALSE;
        xQueueSendFromISR(s_rxQueue,    &frame, &xHigherPriorityTaskWoken);
        xQueueSendFromISR(s_ipcTxQueue, &frame, &xIpcWoken);
        xHigherPriorityTaskWoken |= xIpcWoken;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

HAL_StatusTypeDef Gw_RouteFrame(const CanFrame_t *frame)
{
    /* Skip own heartbeat IDs -- loop prevention on shared test bus */
    if (frame->id == CAN_ENGINE_TX_FDCAN1_ID || frame->id == CAN_ENGINE_TX_FDCAN2_ID) {
        return HAL_BUSY;
    }
    /* UDS frames (0x7E0) are handled by the UDS stack on CM7, not routed.
     * Routing them to FDCAN2 would cause TX errors in single-bus test setups. */
    if (frame->id == UDS_REQUEST_ID) {
        return HAL_BUSY;
    }

    /* Bidirectional: FDCAN1 RX -> FDCAN2 TX, FDCAN2 RX -> FDCAN1 TX */
    FDCAN_HandleTypeDef *dest = (frame->src_bus == GW_BUS_FDCAN1) ? &hfdcan2 : &hfdcan1;

    FDCAN_TxHeaderTypeDef txHdr = {0};
    txHdr.Identifier          = frame->id;
    txHdr.IdType              = FDCAN_STANDARD_ID;
    txHdr.TxFrameType         = FDCAN_DATA_FRAME;
    txHdr.DataLength          = (uint32_t)frame->dlc << 16U; /* reconstruct FDCAN DLC encoding */
    txHdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHdr.BitRateSwitch       = FDCAN_BRS_OFF;
    txHdr.FDFormat            = FDCAN_CLASSIC_CAN;
    txHdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHdr.MessageMarker       = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(dest, &txHdr, (uint8_t *)frame->data);
}


void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) {
        HAL_FDCAN_Stop(hfdcan);
        HAL_FDCAN_Start(hfdcan);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF,              0U);
    }
}
