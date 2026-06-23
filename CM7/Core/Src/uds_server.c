/**
 ******************************************************************************
 * @file    uds_server.c
 * @brief   UDS Diagnostic Server (ISO 14229-1) — M6 implementation.
 *
 * @note    Security algorithm (0x27): key = ~seed (bitwise NOT, 32-bit).
 *          This is a demonstration algorithm — NOT cryptographically secure.
 *          Replace with AES-CMAC or OEM-defined algorithm for production.
 *
 * @note    S3_Server timer: resets on every received UDS request.
 *          Expiry drops session to Default and locks security.
 *          Per ISO 14229-1 §7.5.3, S3_Server = 5 s (configurable).
 ******************************************************************************
 */

/* Includes ----------------------------------------------------------------- */
#include "uds_server.h"
#include "iso_tp.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "stm32h7xx_hal.h"   /* HAL_GetTick() for seed generation */
#include <string.h>
#include <stdio.h>

/* Private types ------------------------------------------------------------ */
typedef uint8_t Uds_Session_t;
typedef uint8_t Uds_SecState_t;

#define UDS_SEC_LOCKED    0U
#define UDS_SEC_UNLOCKED  1U

/* Private variables -------------------------------------------------------- */
static Uds_Session_t  s_session       = UDS_SESSION_DEFAULT;
static Uds_SecState_t s_secState      = UDS_SEC_LOCKED;
static uint32_t       s_secSeed       = 0U;
static uint8_t        s_seedRequested = 0U;
static TimerHandle_t  s_s3Timer       = NULL;

/* DID data tables (ISO 14229-1 Annex C naming) ----------------------------- */
static const uint8_t k_didSwPartNum[] = "ECU_GW_v0.6";   /* 11 chars */
static const uint8_t k_didEcuSerial[] = "ECU-GW-001";    /* 10 chars */
static const uint8_t k_didSwSupplier[]= "STM32-INTERN";  /* 12 chars */

/* Forward declarations ----------------------------------------------------- */
static void handleSessionControl (const uint8_t *pdu, uint16_t len);
static void handleReadDid         (const uint8_t *pdu, uint16_t len);
static void handleSecurityAccess  (const uint8_t *pdu, uint16_t len);
static void handleTesterPresent   (const uint8_t *pdu, uint16_t len);
static void sendNrc               (uint8_t sid, uint8_t nrc);
static void resetS3Timer          (void);

/* S3 timer callback -------------------------------------------------------- */

/**
 * @brief  S3_Server expiry — tester absent > 5 s → drop to Default session.
 * @note   Runs in FreeRTOS timer daemon task. Only sets volatile state.
 */
static void s3TimeoutCb(TimerHandle_t xTimer)
{
    (void)xTimer;
    s_session       = UDS_SESSION_DEFAULT;
    s_secState      = UDS_SEC_LOCKED;
    s_seedRequested = 0U;
    /* Direct-register printf safe from daemon task (polling, no FreeRTOS API) */
    printf("[UDS] S3 timeout → DefaultSession, Security LOCKED\r\n");
}

/* Public API --------------------------------------------------------------- */

void Uds_Init(void)
{
    s_session       = UDS_SESSION_DEFAULT;
    s_secState      = UDS_SEC_LOCKED;
    s_secSeed       = 0U;
    s_seedRequested = 0U;

    s_s3Timer = xTimerCreate("S3Tmr",
                              pdMS_TO_TICKS(UDS_S3_TIMEOUT_MS),
                              pdFALSE,   /* one-shot — tester must send 0x3E repeatedly */
                              NULL,
                              s3TimeoutCb);
    configASSERT(s_s3Timer != NULL);

    printf("[UDS] Init OK. Services: 10/22/27/3E. S3=%u ms\r\n",
           (unsigned)UDS_S3_TIMEOUT_MS);
}

void Uds_ProcessRequest(const uint8_t *pdu, uint16_t len)
{
    if (pdu == NULL || len == 0U) {
        return;
    }

    uint8_t sid = pdu[0];
    printf("[UDS] RX SID=0x%02X len=%u session=%u sec=%u\r\n",
           (unsigned)sid, (unsigned)len,
           (unsigned)s_session, (unsigned)s_secState);

    /* Reset S3 timer on every UDS request (ISO 14229-1 §7.5.3) */
    resetS3Timer();

    switch (sid) {
        case UDS_SID_SESSION_CTRL:   handleSessionControl(pdu, len);  break;
        case UDS_SID_READ_DID:       handleReadDid(pdu, len);         break;
        case UDS_SID_SEC_ACCESS:     handleSecurityAccess(pdu, len);  break;
        case UDS_SID_TESTER_PRESENT: handleTesterPresent(pdu, len);   break;
        default:
            printf("[UDS] SID=0x%02X not supported\r\n", (unsigned)sid);
            sendNrc(sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
            break;
    }
}

/* ── Service handlers ─────────────────────────────────────────────────────── */

/**
 * @brief  0x10 DiagnosticSessionControl
 *         Sub-functions: 0x01 default | 0x02 programming | 0x03 extended
 *         Response: [50 sub P2_H P2_L P2star_H P2star_L]
 *         P2_Server    = 25 ms  (0x0019)
 *         P2*_Server   = 500 ms (0x01F4)
 */
static void handleSessionControl(const uint8_t *pdu, uint16_t len)
{
    if (len < 2U) {
        sendNrc(UDS_SID_SESSION_CTRL, UDS_NRC_INCORRECT_MSG_LEN);
        return;
    }
    uint8_t sub = pdu[1] & 0x7FU;   /* strip SPRMIB bit */

    switch (sub) {
        case UDS_SESSION_DEFAULT:
        case UDS_SESSION_PROGRAMMING:
        case UDS_SESSION_EXTENDED:
            s_session = sub;
            if (sub != UDS_SESSION_EXTENDED && sub != UDS_SESSION_PROGRAMMING) {
                s_secState      = UDS_SEC_LOCKED;
                s_seedRequested = 0U;
            }
            break;
        default:
            sendNrc(UDS_SID_SESSION_CTRL, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
            return;
    }

    printf("[UDS] Session → 0x%02X\r\n", (unsigned)sub);
    uint8_t resp[6] = {
        (uint8_t)(UDS_SID_SESSION_CTRL + UDS_SID_POSITIVE_OFFSET),
        sub,
        0x00U, 0x19U,   /* P2_Server = 25 ms  */
        0x01U, 0xF4U    /* P2*_Server = 500 ms */
    };
    IsoTp_StartTx(resp, 6U);
}

/**
 * @brief  0x22 ReadDataByIdentifier
 *         Supported DIDs: F187 (SW version), F18C (serial), F195 (supplier)
 *         Response: [62 DID_H DID_L <data bytes>]
 */
static void handleReadDid(const uint8_t *pdu, uint16_t len)
{
    if (len < 3U) {
        sendNrc(UDS_SID_READ_DID, UDS_NRC_INCORRECT_MSG_LEN);
        return;
    }

    uint16_t did = ((uint16_t)pdu[1] << 8U) | (uint16_t)pdu[2];

    const uint8_t *data_ptr = NULL;
    uint16_t       data_len = 0U;

    switch (did) {
        case UDS_DID_SW_PART_NUMBER:
            data_ptr = k_didSwPartNum;
            data_len = (uint16_t)(sizeof(k_didSwPartNum) - 1U);  /* exclude \0 */
            break;
        case UDS_DID_ECU_SERIAL:
            data_ptr = k_didEcuSerial;
            data_len = (uint16_t)(sizeof(k_didEcuSerial) - 1U);
            break;
        case UDS_DID_SW_SUPPLIER:
            data_ptr = k_didSwSupplier;
            data_len = (uint16_t)(sizeof(k_didSwSupplier) - 1U);
            break;
        default:
            printf("[UDS] DID=0x%04X not supported\r\n", (unsigned)did);
            sendNrc(UDS_SID_READ_DID, UDS_NRC_REQUEST_OUT_OF_RANGE);
            return;
    }

    /* Build response: [62][DID_H][DID_L][data...] — max 3 + 12 = 15 bytes */
    uint8_t  resp[20U];
    uint16_t resp_len = (uint16_t)(3U + data_len);
    resp[0] = (uint8_t)(UDS_SID_READ_DID + UDS_SID_POSITIVE_OFFSET);
    resp[1] = (uint8_t)(did >> 8U);
    resp[2] = (uint8_t)(did & 0xFFU);
    memcpy(&resp[3], data_ptr, (size_t)data_len);

    printf("[UDS] DID=0x%04X → %u bytes\r\n", (unsigned)did, (unsigned)resp_len);
    IsoTp_StartTx(resp, resp_len);
}

/**
 * @brief  0x27 SecurityAccess (seed/key, requires ExtendedDiagnostic session)
 *         Sub 0x01 requestSeed:  → [67 01 S3 S2 S1 S0]
 *         Sub 0x02 sendKey:      → [67 02] or NRC 0x35
 *         Algorithm: key = ~seed (bitwise NOT, 32-bit)
 */
static void handleSecurityAccess(const uint8_t *pdu, uint16_t len)
{
    /* Security Access only allowed in Extended or Programming session */
    if (s_session != UDS_SESSION_EXTENDED &&
        s_session != UDS_SESSION_PROGRAMMING) {
        sendNrc(UDS_SID_SEC_ACCESS, UDS_NRC_CONDITIONS_NOT_CORRECT);
        return;
    }
    if (len < 2U) {
        sendNrc(UDS_SID_SEC_ACCESS, UDS_NRC_INCORRECT_MSG_LEN);
        return;
    }

    uint8_t sub = pdu[1];

    if (sub == 0x01U) {
        /* ── Request Seed ── */
        if (s_secState == UDS_SEC_UNLOCKED) {
            /* Already unlocked: return all-zero seed (ISO 14229-1 §10.4.4.3) */
            uint8_t resp[6] = { 0x67U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U };
            printf("[UDS] SecAccess: already unlocked, seed=0\r\n");
            IsoTp_StartTx(resp, 6U);
        } else {
            s_secSeed       = HAL_GetTick();   /* pseudo-random from uptime */
            s_seedRequested = 1U;
            uint8_t resp[6] = {
                0x67U, 0x01U,
                (uint8_t)(s_secSeed >> 24U),
                (uint8_t)(s_secSeed >> 16U),
                (uint8_t)(s_secSeed >>  8U),
                (uint8_t)(s_secSeed & 0xFFU)
            };
            printf("[UDS] SecAccess seed=0x%08lX\r\n", (unsigned long)s_secSeed);
            IsoTp_StartTx(resp, 6U);
        }

    } else if (sub == 0x02U) {
        /* ── Send Key ── */
        if (!s_seedRequested) {
            sendNrc(UDS_SID_SEC_ACCESS, UDS_NRC_REQUEST_SEQ_ERROR);
            return;
        }
        if (len < 6U) {
            sendNrc(UDS_SID_SEC_ACCESS, UDS_NRC_INCORRECT_MSG_LEN);
            return;
        }

        uint32_t recv_key = ((uint32_t)pdu[2] << 24U)
                          | ((uint32_t)pdu[3] << 16U)
                          | ((uint32_t)pdu[4] <<  8U)
                          |  (uint32_t)pdu[5];
        uint32_t expected_key = ~s_secSeed;  /* algorithm: key = ~seed */

        s_seedRequested = 0U;   /* consume seed regardless of result */

        if (recv_key == expected_key) {
            s_secState = UDS_SEC_UNLOCKED;
            printf("[UDS] Security UNLOCKED (key=0x%08lX)\r\n",
                   (unsigned long)recv_key);
            uint8_t resp[2] = { 0x67U, 0x02U };
            IsoTp_StartTx(resp, 2U);
        } else {
            printf("[UDS][ERR] InvalidKey: recv=0x%08lX exp=0x%08lX\r\n",
                   (unsigned long)recv_key, (unsigned long)expected_key);
            sendNrc(UDS_SID_SEC_ACCESS, UDS_NRC_INVALID_KEY);
        }

    } else {
        sendNrc(UDS_SID_SEC_ACCESS, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
    }
}

/**
 * @brief  0x3E TesterPresent — keepalive, resets S3 timer.
 *         Sub 0x00: respond [7E 00].
 *         Sub 0x80 (SPRMIB set): suppress response, just reset S3.
 */
static void handleTesterPresent(const uint8_t *pdu, uint16_t len)
{
    if (len < 2U) {
        sendNrc(UDS_SID_TESTER_PRESENT, UDS_NRC_INCORRECT_MSG_LEN);
        return;
    }

    uint8_t sub = pdu[1];

    /* Bit 7 = suppressPosRspMsgIndicationBit (SPRMIB) */
    if (sub & 0x80U) {
        printf("[UDS] TesterPresent (suppress)\r\n");
        return;   /* S3 already reset above in Uds_ProcessRequest */
    }

    if ((sub & 0x7FU) != 0x00U) {
        sendNrc(UDS_SID_TESTER_PRESENT, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
        return;
    }

    printf("[UDS] TesterPresent OK\r\n");
    uint8_t resp[2] = {
        (uint8_t)(UDS_SID_TESTER_PRESENT + UDS_SID_POSITIVE_OFFSET),
        0x00U
    };
    IsoTp_StartTx(resp, 2U);
}

/* ── Private helpers ──────────────────────────────────────────────────────── */

/**
 * @brief  Build and send Negative Response Code frame.
 *         Frame: [7F SID NRC] — always a SF (3 bytes).
 */
static void sendNrc(uint8_t sid, uint8_t nrc)
{
    printf("[UDS] NRC: SID=0x%02X NRC=0x%02X\r\n",
           (unsigned)sid, (unsigned)nrc);
    uint8_t resp[3] = { UDS_SID_NRC, sid, nrc };
    IsoTp_StartTx(resp, 3U);
}

/**
 * @brief  Start/reset S3_Server one-shot timer (5 s).
 *         Called on every valid UDS request (ISO 14229-1 §7.5.3).
 */
static void resetS3Timer(void)
{
    if (s_s3Timer != NULL) {
        /* xTimerReset starts the timer if stopped, or resets period if running */
        xTimerReset(s_s3Timer, 0U);
    }
}
