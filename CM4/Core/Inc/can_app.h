/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can_app.h
 * @brief   CAN Engine public API -- FDCAN1 driver layer for CM4 (ADR-008).
 *
 * @note    AUTOSAR mapping: MCAL-adjacent driver layer.
 *          CM4 owns FDCAN1 hardware exclusively (ADR-008).
 *          CM7 will receive frame copies via rpmsg in M5+.
 *
 * @note    Naming: "CanEngine_" prefix (Section 2.2 convention).
 *          File name can_app.h kept for CubeIDE project compatibility.
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef CAN_APP_H_
#define CAN_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"

/* Public constants ----------------------------------------------------------*/

/**
 * @brief Hardware filter 0 — range filter for gateway frames.
 *        Accepts standard IDs [0x100, 0x1FF] into FIFO0.
 *        HW: FDCAN Message RAM Standard Filter Element
 *            SFEC=001 (store FIFO0), SFT=10 (range filter).
 */
#define CAN_ENGINE_FILTER_ID_LOW    0x100U
#define CAN_ENGINE_FILTER_ID_HIGH   0x1FFU

/**
 * @brief Hardware filter 1 — exact-match filter for UDS diagnostic frames.
 *        Accepts ID 0x7E0 (tester→ECU functional address, ISO 15765-4 §6.3.2).
 *        Applied to FDCAN1 only (bus connected to PC USB-to-CAN).
 *        HW: FDCAN Message RAM Standard Filter Element
 *            SFEC=001 (store FIFO0), SFT=00 (mask filter), ID1=0x7E0, ID2=0x7FF.
 */
#define UDS_REQUEST_ID              0x7E0U   /**< Tester → ECU              */
#define UDS_RESPONSE_ID             0x7E8U   /**< ECU → Tester (ID + 8)     */

/** @brief src_bus marker for UDS TX commands from CM7 → CM4. */
#define GW_BUS_UDS_TX               0xFEU

/** @brief src_bus marker for IDS alert from CM7 → CM4 (M7). */
#define GW_BUS_IDS_ALERT            0xFDU

/**
 * @brief TX heartbeat frame standard identifier.
 *        0x123 is outside the RX filter range so it is always visible
 *        in SavvyCAN regardless of the RX filter configuration.
 */
#define CAN_ENGINE_TX_FDCAN1_ID     0x123U
#define CAN_ENGINE_TX_FDCAN2_ID     0x125U
/** @brief Maximum data bytes per Classic CAN frame. */
#define CAN_ENGINE_DLC_MAX          8U

/** @brief Source bus identifier -- filled by ISR, used by Gw_RouteFrame(). */
#define GW_BUS_FDCAN1               1U
#define GW_BUS_FDCAN2               2U

/* Public types --------------------------------------------------------------*/

/**
 * @brief  CAN frame container -- transferred ISR -> CanEngineTask via queue.
 *
 * @note   Layout (20 bytes, 4-byte aligned):
 *           Offset  0 : id        [4 B] -- 11-bit standard identifier
 *           Offset  4 : dlc       [1 B] -- data length in bytes (0-8)
 *           Offset  5 : data[8]   [8 B] -- frame payload
 *           Offset 13 : src_bus   [1 B] -- GW_BUS_FDCAN1 or GW_BUS_FDCAN2
 *           Offset 14 : _pad[2]   [2 B] -- explicit padding (MISRA-C 6.7.2)
 *           Offset 16 : timestamp [4 B] -- HAL_GetTick() at ISR entry [ms]
 *           Total: 20 B (unchanged -- no queue API impact).
 */
typedef struct {
    uint32_t id;                        /**< CAN std ID (11-bit, 0x000-0x7FF) */
    uint8_t  dlc;                       /**< Data length in bytes (0-8)       */
    uint8_t  data[CAN_ENGINE_DLC_MAX];  /**< Frame payload                    */
    uint8_t  src_bus;                   /**< GW_BUS_FDCAN1 / GW_BUS_FDCAN2   */
    uint8_t  _pad[2];                   /**< Padding -- timestamp alignment   */
    uint32_t timestamp;                 /**< HAL_GetTick() at ISR entry [ms]  */
} CanFrame_t;

/* Public API ----------------------------------------------------------------*/

/**
 * @brief  Initialise the CAN Engine layer.
 *
 *         Steps (call before CanEngineTask for(;;) loop):
 *           1. xQueueCreate -- 16-slot CanFrame_t FIFO (ISR -> task path).
 *           2. HAL_FDCAN_ConfigFilter -- std-ID range 0x100-0x1FF -> FIFO0.
 *              HW: writes FDCAN_RXGFC.LSS + Message RAM filter element.
 *           3. HAL_FDCAN_ConfigGlobalFilter -- reject all non-matching frames.
 *              HW: FDCAN_GFC.ANFS=10 (reject std), ANFE=10 (reject ext).
 *           4. TX header setup -- ID=0x123, DLC=8, Classic CAN, BRS off.
 *           5. HAL_FDCAN_Start -- FDCAN_CCCR.INIT=0, bus integration starts.
 *           6. HAL_FDCAN_ActivateNotification -- FDCAN_IE.RF0NE + FDCAN_IE.BOE.
 *
 * @retval HAL_OK on success, HAL_ERROR on any peripheral or heap fault.
 */
HAL_StatusTypeDef CanEngine_Init(void);

/**
 * @brief  Transmit a test heartbeat frame on FDCAN1 (ID=0x123, DLC=8).
 *
 * @param  counter  Sequence byte in data[0] -- SavvyCAN verifies continuity.
 *
 * @note   HW path (RM0399 Ch.59): checks FDCAN_TXFQS.TFQF, writes T0/T1/data
 *         into Message RAM Tx buffer, sets FDCAN_TXBAR.AR[n] to arm TX.
 *
 * @retval HAL_OK / HAL_ERROR (Tx FIFO full)
 */
HAL_StatusTypeDef CanEngine_SendFrame(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t counter);
/**
 * @brief  Return handle to the RX FreeRTOS queue.
 *
 * @note   Valid only after CanEngine_Init() returns HAL_OK.
 *
 * @retval FreeRTOS QueueHandle_t (NULL if init not yet called)
 */
QueueHandle_t CanEngine_GetRxQueue(void);
QueueHandle_t CanEngine_GetIpcTxQueue(void);

/**
 * @brief  Return the IPC RX queue handle (CM7 → CM4 UDS TX commands).
 *         CM4 IpcRxTask dequeues IpcFrame_t where src_bus == GW_BUS_UDS_TX
 *         and forwards to FDCAN1 TX via CanEngine_SendUdsResponse().
 */
QueueHandle_t CanEngine_GetIpcRxQueue(void);

/**
 * @brief  Transmit a UDS response frame on FDCAN1.
 *         Called by CM4 IpcRxTask after receiving TX command from CM7.
 *         HW path: checks FDCAN_TXFQS.TFQF → writes T0/T1/data into
 *         Message RAM Tx buffer → sets FDCAN_TXBAR.AR[n] to arm TX.
 *
 * @param  frame  Pointer to IpcFrame_t (id=0x7E8, dlc=8, data=ISO-TP bytes).
 * @retval HAL_OK / HAL_ERROR (Tx FIFO full)
 */
HAL_StatusTypeDef CanEngine_SendUdsResponse(const CanFrame_t *frame);

/**
 * @brief  Route a received frame to the opposite FDCAN bus (bidirectional).
 *
 * @details Fast-path routing -- runs entirely on CM4, no IPC.
 *          FDCAN1 RX -> FDCAN2 TX, FDCAN2 RX -> FDCAN1 TX.
 *          Heartbeat IDs (0x123, 0x125) are silently dropped (loop prevention
 *          for shared-bus test bench). AUTOSAR layer: GW module (CM4 side).
 *
 * @param  frame  Pointer to received frame (filled by ISR, src_bus set).
 *
 * @retval HAL_OK    Frame was forwarded to destination TX FIFO.
 * @retval HAL_BUSY  Frame skipped (own heartbeat ID -- not an error).
 * @retval HAL_ERROR Destination TX FIFO full.
 */
HAL_StatusTypeDef Gw_RouteFrame(const CanFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* CAN_APP_H_ */
