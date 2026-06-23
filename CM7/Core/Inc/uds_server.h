/**
 ******************************************************************************
 * @file    uds_server.h
 * @brief   UDS Diagnostic Server (ISO 14229-1) — CM7 public API.
 *
 * @note    Services implemented (M6):
 *            0x10  DiagnosticSessionControl  (Default / Extended / Programming)
 *            0x22  ReadDataByIdentifier      (DID F187 / F18C / F195)
 *            0x27  SecurityAccess            (seed/key, algorithm: key = ~seed)
 *            0x3E  TesterPresent             (S3 timer 5 s, SPRMIB support)
 *
 * @note    AUTOSAR mapping: DCM (Diagnostic Communication Manager) module.
 *          Sits above ISO-TP transport layer on CM7.
 *          TX path: Uds_ProcessRequest() → IsoTp_StartTx() →
 *                   s_cm7IpcTxQueue → CM7 IpcTxTask → rpmsg → CM4 → FDCAN1.
 ******************************************************************************
 */
#ifndef UDS_SERVER_H_
#define UDS_SERVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── UDS Service IDs (ISO 14229-1 Table 2) ───────────────────────────────── */
#define UDS_SID_SESSION_CTRL    0x10U
#define UDS_SID_READ_DID        0x22U
#define UDS_SID_SEC_ACCESS      0x27U
#define UDS_SID_TESTER_PRESENT  0x3EU
#define UDS_SID_POSITIVE_OFFSET 0x40U  /**< positive response = SID + 0x40  */
#define UDS_SID_NRC             0x7FU  /**< Negative Response Code frame    */

/* ── UDS Negative Response Codes (ISO 14229-1 Table A.1) ────────────────── */
#define UDS_NRC_GENERAL_REJECT          0x10U
#define UDS_NRC_SERVICE_NOT_SUPPORTED   0x11U
#define UDS_NRC_SUBFUNC_NOT_SUPPORTED   0x12U
#define UDS_NRC_INCORRECT_MSG_LEN       0x13U
#define UDS_NRC_CONDITIONS_NOT_CORRECT  0x22U
#define UDS_NRC_REQUEST_SEQ_ERROR       0x24U
#define UDS_NRC_REQUEST_OUT_OF_RANGE    0x31U
#define UDS_NRC_SECURITY_ACCESS_DENIED  0x33U
#define UDS_NRC_INVALID_KEY             0x35U

/* ── DID definitions (ISO 14229-1 Annex C) ───────────────────────────────── */
#define UDS_DID_SW_PART_NUMBER  0xF187U  /**< Software Part Number           */
#define UDS_DID_ECU_SERIAL      0xF18CU  /**< ECU Serial Number              */
#define UDS_DID_SW_SUPPLIER     0xF195U  /**< Software Supplier              */

/* ── Session constants ───────────────────────────────────────────────────── */
#define UDS_S3_TIMEOUT_MS       5000U    /**< TesterPresent keepalive window  */

/* ── Session types (0x10 sub-function) ──────────────────────────────────── */
#define UDS_SESSION_DEFAULT     0x01U
#define UDS_SESSION_PROGRAMMING 0x02U
#define UDS_SESSION_EXTENDED    0x03U

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise UDS server state and create S3 software timer.
 *         Must be called once in IsoTpUdsTask before entering the for(;;) loop.
 *         FreeRTOS scheduler must be running (timers require daemon task).
 */
void Uds_Init(void);

/**
 * @brief  Process an assembled UDS PDU from ISO-TP layer.
 *         Dispatches to per-service handler; each handler calls IsoTp_StartTx()
 *         to send the response (positive or NRC).
 *
 * @param  pdu  Pointer to assembled PDU buffer (first byte = SID).
 * @param  len  Total PDU length in bytes (≥ 1).
 */
void Uds_ProcessRequest(const uint8_t *pdu, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* UDS_SERVER_H_ */
