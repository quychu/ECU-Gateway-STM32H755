/**
 ******************************************************************************
 * @file    ipc_types.h
 * @brief   Shared IPC frame type for CM7 — mirrors CanFrame_t in CM4/can_app.h.
 *
 * @note    Both structs MUST remain byte-identical (20 bytes, same field order).
 *          The rpmsg payload carries raw binary of this struct across SRAM4.
 *          Any change here MUST be reflected in CM4/Core/Inc/can_app.h.
 *
 * @note    AUTOSAR mapping: COM (Communication) layer data type — sits between
 *          PDU Router (CM7 application) and IPC transport (rpmsg/OpenAMP).
 ******************************************************************************
 */
#ifndef IPC_TYPES_H_
#define IPC_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── src_bus values ──────────────────────────────────────────────────────── */
#define IPC_BUS_FDCAN1    1U     /**< Frame received on FDCAN1 (CM4 side) */
#define IPC_BUS_FDCAN2    2U     /**< Frame received on FDCAN2 (CM4 side) */
#define IPC_BUS_UDS_TX   0xFEU  /**< CM7→CM4 UDS TX command (response frame) */
#define IPC_BUS_IDS_ALERT 0xFDU /**< CM7→CM4 IDS violation alert (M7)        */

/* ── UDS CAN IDs (standard diagnostic addressing, ISO 15765-4) ───────────── */
#define UDS_REQUEST_ID   0x7E0U  /**< Tester→ECU functional request ID      */
#define UDS_RESPONSE_ID  0x7E8U  /**< ECU→Tester physical response ID       */

/**
 * @brief  Inter-core frame container — 20 bytes, 4-byte aligned.
 *         Transferred via rpmsg (SRAM4 OpenAMP vring) between CM4 and CM7.
 *
 * @note   Layout (must match CanFrame_t in CM4/Core/Inc/can_app.h):
 *           Offset  0 : id        [4 B]
 *           Offset  4 : dlc       [1 B]
 *           Offset  5 : data[8]   [8 B]
 *           Offset 13 : src_bus   [1 B]  (IPC_BUS_* or IPC_BUS_UDS_TX)
 *           Offset 14 : _pad[2]   [2 B]
 *           Offset 16 : timestamp [4 B]
 *           Total: 20 B
 */
typedef struct {
    uint32_t id;          /**< CAN std ID (11-bit)                          */
    uint8_t  dlc;         /**< Data length in bytes (0–8)                   */
    uint8_t  data[8];     /**< Frame payload (padded with 0xCC if unused)   */
    uint8_t  src_bus;     /**< IPC_BUS_FDCAN1 / IPC_BUS_FDCAN2 / IPC_BUS_UDS_TX */
    uint8_t  _pad[2];     /**< Explicit padding — MISRA-C 6.7.2             */
    uint32_t timestamp;   /**< HAL_GetTick() at capture [ms]                */
} IpcFrame_t;

#ifdef __cplusplus
}
#endif

#endif /* IPC_TYPES_H_ */
