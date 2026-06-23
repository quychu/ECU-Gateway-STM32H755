/**
 ******************************************************************************
 * @file    iso_tp.h
 * @brief   ISO-TP (ISO 15765-2) Transport Layer — CM7 side.
 *
 * @note    AUTOSAR mapping: Transport Protocol module (Tp) in BSW layer.
 *          Runs on CM7 (Application Layer). Receives raw CAN frames via
 *          IpcFrame_t queue (pushed by IpcRxTask from rpmsg), assembles
 *          multi-frame PDUs, delivers to UDS via Uds_ProcessRequest().
 *          Sends responses back via IpcFrame_t queue → CM7 IpcTxTask →
 *          rpmsg → CM4 IpcRxTask → FDCAN1 TX.
 *
 * @note    Supported frame types (ISO 15765-2 Table 1):
 *            SF  (0x0N): Single Frame    — payload ≤ 7 bytes
 *            FF  (0x1N): First Frame     — payload > 7 bytes
 *            CF  (0x2N): Consecutive F   — continuation of FF
 *            FC  (0x3N): Flow Control    — receiver → sender quota/rate
 *
 * @note    Timing parameters (ISO 15765-2 Table 5):
 *            N_Cr = 75 ms  — max wait between consecutive CFs (RX side)
 *            N_Bs = 75 ms  — max wait for FC after FF sent   (TX side)
 ******************************************************************************
 */
#ifndef ISO_TP_H_
#define ISO_TP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ipc_types.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

/* ── ISO-TP frame size constants ─────────────────────────────────────────── */
#define ISOTP_MAX_PDU_LEN       4095U   /**< FF_DL max (12-bit field, classic CAN) */
#define ISOTP_SF_MAX_PAYLOAD    7U      /**< SF: max 7 data bytes (1 PCI + 7 = 8)  */
#define ISOTP_CAN_DLC           8U      /**< Always transmit 8-byte padded frames   */

/* ── PCI nibble values (upper nibble of byte 0) ──────────────────────────── */
#define ISOTP_PCI_SF            0x0U
#define ISOTP_PCI_FF            0x1U
#define ISOTP_PCI_CF            0x2U
#define ISOTP_PCI_FC            0x3U

/* ── FC FlowStatus values (lower nibble of byte 0) ──────────────────────── */
#define ISOTP_FC_CTS            0x30U   /**< Clear To Send  — proceed           */
#define ISOTP_FC_WAIT           0x31U   /**< Wait           — hold off          */
#define ISOTP_FC_OVERFLOW       0x32U   /**< Overflow/Abort — buffer too small  */

/**
 * @brief  FC parameters this ECU sends to tester after receiving FF.
 *         BS=0  → unlimited (tester sends all CFs without another FC).
 *         STmin=0 → no mandatory gap between CFs (tester sends as fast as it can).
 *         For stress testing: increase STmin to 0x0A (10 ms).
 */
#define ISOTP_FC_BS             0x00U   /**< Block Size: 0 = unlimited          */
#define ISOTP_FC_STMIN          0x00U   /**< Separation Time min: 0 = no delay  */

/* ── Timing constants (ms) ───────────────────────────────────────────────── */
#define ISOTP_TIMEOUT_N_CR_MS   75U     /**< RX: max gap between CFs            */
#define ISOTP_TIMEOUT_N_BS_MS   75U     /**< TX: max wait for FC after FF sent  */

/* ── Padding byte (Bosch convention) ────────────────────────────────────── */
#define ISOTP_PAD_BYTE          0xCCU

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise ISO-TP layer.
 *         Creates N_Cr one-shot FreeRTOS timer.
 *         Stores queue handles for RX (FC reception) and TX (frame dispatch).
 *
 * @param  isoTpRxQueue   Queue where IpcRxTask pushes IpcFrame_t from CM4.
 *                        IsoTp_StartTx() also blocks on this queue for FC.
 * @param  cm7IpcTxQueue  Queue where IsoTp pushes outgoing IpcFrame_t to CM4.
 *                        CM7 IpcTxTask drains this queue via OPENAMP_send().
 */
void IsoTp_Init(QueueHandle_t isoTpRxQueue, QueueHandle_t cm7IpcTxQueue);

/**
 * @brief  Process one received CAN frame through the ISO-TP RX FSM.
 *         Dispatch: SF → Uds_ProcessRequest() immediately.
 *                   FF → send FC, start N_Cr timer, enter WAIT_CF.
 *                   CF → accumulate; when complete call Uds_ProcessRequest().
 *                   FC → hand off to TX FSM (called from IsoTp_StartTx context).
 *
 * @param  frame  Pointer to IpcFrame_t received from s_isoTpRxQueue.
 *                data[0] PCI byte determines frame type.
 */
void IsoTp_ProcessRxFrame(const IpcFrame_t *frame);

/**
 * @brief  Transmit a UDS PDU via ISO-TP to the tester.
 *         If length ≤ 7: sends a Single Frame immediately.
 *         If length > 7: sends FF, blocks on s_isoTpRxQueue waiting for FC
 *         (timeout N_Bs = 75 ms), then sends all CF frames.
 *
 * @note   Blocking call — runs in IsoTpUdsTask context.
 *         osDelay(STmin) applied between CFs if FC specifies STmin > 0.
 *
 * @param  pdu     Pointer to assembled UDS response (owned by caller).
 * @param  length  Total bytes to send (1 – ISOTP_MAX_PDU_LEN).
 */
void IsoTp_StartTx(const uint8_t *pdu, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* ISO_TP_H_ */
