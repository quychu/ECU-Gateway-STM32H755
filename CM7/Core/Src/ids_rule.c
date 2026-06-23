/**
 ******************************************************************************
 * @file    ids_rule.c
 * @brief   IDS Rule Engine — R1 Whitelist / R2 DLC / R3 Period / R4 Burst.
 *
 * @note    AUTOSAR: BSW Intrusion Detection Module (ISO/SAE 21434 concept).
 *          Runs entirely on CM7 Application Layer.
 *
 * @note    Database layout (const arrays, stored in CM7 Flash Bank1):
 *            Db_Whitelist[id]       — bool: is ID allowed?
 *            Db_ExpectedDlc[id]     — uint8_t: expected DLC (0 = skip R2)
 *            Db_ExpectedPeriod[id]  — uint32_t: expected period [ms] (0 = event)
 *
 * @note    State layout (CM7 AXI SRAM, CM7-private, cacheable OK):
 *            s_lastTimestamp[id]    — uint32_t: last clean frame timestamp [ms]
 *            s_isFirstFrame[id]     — bool: first-frame guard for R3
 *            s_tokenBucket          — int32_t: current token count (R4)
 *            s_lastRefillTime       — uint32_t: last refill timestamp [ms]
 *
 * @note    Performance: all lookups O(1). No dynamic allocation. No recursion.
 *          Target < 10 µs/frame at 400 MHz CM7 (Section 6.3 CLAUDE.md).
 ******************************************************************************
 */

#include "ids_rule.h"
#include "ipc_types.h"
#include <string.h>
#include <stddef.h>

/* ============================================================
 * SECTION 1 — IDS DATABASE (const — Flash)
 * ============================================================
 * Edit this section to add/remove allowed IDs.
 *
 * Whitelist policy (M7 scope):
 *   - Gateway range  : 0x100–0x1FF  (all 256 IDs, DLC=8, event)
 *   - UDS tester     : 0x7E0        (DLC variable, event)
 *   - UDS response   : 0x7E8        (DLC variable, event)
 *   - Test ID 0x110  : DLC=8, period=500 ms  ← R3 demo target
 *   - Heartbeat IDs  : 0x123, 0x125 — dropped by Gw_RouteFrame
 *                      so they never reach IDS; listed for completeness.
 * ============================================================ */

/**
 * @brief  Whitelist — true if ID is a known-good CAN identifier.
 *         Indexed by 11-bit standard CAN ID (0..2047).
 *         Stored in Flash; zero-initialised by linker; patched in Ids_Init().
 * @note   Using runtime init (not compound literal) to stay MISRA-C 2012 §8.9
 *         compliant and avoid large Flash const arrays.
 */
static bool s_whitelist[IDS_STD_ID_MAX];

/**
 * @brief  Expected DLC per ID. 0 = skip R2 DLC check (variable-length PDU).
 */
static uint8_t s_expectedDlc[IDS_STD_ID_MAX];

/**
 * @brief  Expected inter-frame period [ms]. 0 = event frame (skip R3 check).
 */
static uint32_t s_expectedPeriod[IDS_STD_ID_MAX];

/* ============================================================
 * SECTION 2 — RUNTIME STATE (AXI SRAM — CM7 private)
 * ============================================================ */

/** R3 — last timestamp of clean frame per ID */
static uint32_t s_lastTimestamp[IDS_STD_ID_MAX];

/**
 * R3 — first-frame guard.
 * True = no previous frame seen → skip R3 delta check, just store timestamp.
 * Prevents false positive when delta = (current_ts - 0) on first arrival.
 */
static bool s_isFirstFrame[IDS_STD_ID_MAX];

/** R4 — token bucket state (global bus load) */
static int32_t  s_tokenBucket;
static uint32_t s_lastRefillTime;

/* ============================================================
 * SECTION 3 — PRIVATE HELPERS
 * ============================================================ */

/**
 * @brief  Build forensic alert IpcFrame_t from a rule violation.
 *
 * @param[out] alert    Output alert frame (must be non-NULL).
 * @param[in]  frame    Offending original frame.
 * @param[in]  rule_code  IDS_RULE_R1..R4.
 */
static void buildAlert(IpcFrame_t *alert,
                       const IpcFrame_t *frame,
                       uint8_t rule_code)
{
    /* Encode alert as IpcFrame_t so it travels the existing IPC TX path. */
    alert->id        = frame->id;
    alert->dlc       = frame->dlc;
    alert->src_bus   = IPC_BUS_IDS_ALERT;
    alert->timestamp = frame->timestamp;

    /* data[0] = rule code; data[1..8] = raw offending payload (forensics) */
    alert->data[0] = rule_code;
    (void)memcpy(&alert->data[1], frame->data,
                 (frame->dlc < 7U) ? frame->dlc : 7U); /* cap at data[1..7] */

    /* Explicit padding — MISRA-C 6.7.2 */
    alert->_pad[0] = 0U;
    alert->_pad[1] = 0U;
}

/**
 * @brief  Token bucket refill based on elapsed time.
 *         Call ONCE at the start of each Ids_CheckFrame() invocation.
 *
 * @param  now_ms  Current system tick [ms].
 */
static void refillTokenBucket(uint32_t now_ms)
{
    uint32_t delta = now_ms - s_lastRefillTime;

    if (delta >= IDS_TOKEN_WINDOW_MS) {
        /* Full window(s) elapsed — compute proportional refill */
        int32_t windows     = (int32_t)(delta / IDS_TOKEN_WINDOW_MS);
        int32_t new_tokens  = windows * (int32_t)IDS_TOKEN_REFILL_RATE;

        s_tokenBucket += new_tokens;

        /* Clamp to capacity — prevent burst allowance accumulation */
        if (s_tokenBucket > (int32_t)IDS_BURST_MAX_TOKENS) {
            s_tokenBucket = (int32_t)IDS_BURST_MAX_TOKENS;
        }

        /* Advance refill timestamp by whole windows consumed */
        s_lastRefillTime += (uint32_t)windows * IDS_TOKEN_WINDOW_MS;
    }
}

/* ============================================================
 * SECTION 4 — PUBLIC API
 * ============================================================ */

/**
 * @brief  Initialize IDS whitelist DB and clear all state.
 *         Call before IdsRuleTask main loop (scheduler running).
 */
void Ids_Init(void)
{
    uint32_t id;

    /* ── Clear all state arrays ──────────────────────────── */
    (void)memset(s_whitelist,       0,  sizeof(s_whitelist));
    (void)memset(s_expectedDlc,     0,  sizeof(s_expectedDlc));
    (void)memset(s_expectedPeriod,  0,  sizeof(s_expectedPeriod));
    (void)memset(s_lastTimestamp,   0,  sizeof(s_lastTimestamp));

    /* s_isFirstFrame[] = all true — no prior frame seen */
    for (id = 0U; id < IDS_STD_ID_MAX; id++) {
        s_isFirstFrame[id] = true;
    }

    /* ── Token bucket reset ──────────────────────────────── */
    s_tokenBucket    = (int32_t)IDS_BURST_MAX_TOKENS;
    s_lastRefillTime = 0U;

    /* ── Populate whitelist DB ───────────────────────────── */

    /* Gateway routing range: 0x100 – 0x1FF (CAN engine HW filter range) */
    for (id = 0x100U; id <= 0x1FFU; id++) {
        s_whitelist[id]     = true;
        s_expectedDlc[id]   = 8U;   /* Classic CAN full frame              */
        s_expectedPeriod[id] = 0U;  /* Event frame — no period check       */
    }

    /* ── Override specific IDs with known periods (R3 demo) ── */
    /* Test ID 0x110 — period 500 ms (use for R3 inject test) */
    s_expectedPeriod[0x110U] = 500U;

    /* Test ID 0x111 — period 1000 ms */
    s_expectedPeriod[0x111U] = 1000U;

    /* ── UDS addressing (ISO 15765-4 §6.3.2) ─────────────── */
    s_whitelist[UDS_REQUEST_ID]    = true;
    s_expectedDlc[UDS_REQUEST_ID]  = 0U;  /* Variable DLC — skip R2   */
    s_expectedPeriod[UDS_REQUEST_ID] = 0U;

    s_whitelist[UDS_RESPONSE_ID]   = true;
    s_expectedDlc[UDS_RESPONSE_ID] = 0U;
    s_expectedPeriod[UDS_RESPONSE_ID] = 0U;
}

/**
 * @brief  Evaluate all 4 IDS rules against one CAN frame.
 *         Returns on FIRST violation (short-circuit evaluation).
 *         State update only happens when frame passes ALL rules.
 *
 * @param  frame      Input frame from CM4 IPC (via s_idsRxQueue).
 * @param  alert_out  Filled on violation. May be NULL (unit test mode).
 * @retval IDS_RULE_CLEAN or violation rule code.
 */
uint8_t Ids_CheckFrame(const IpcFrame_t *frame, IpcFrame_t *alert_out)
{
    if (frame == NULL) {
        return IDS_RULE_CLEAN;
    }

    uint32_t id = frame->id;
    uint32_t ts = frame->timestamp;

    /* ── [R1] Whitelist check ────────────────────────────── */
    if ((id >= IDS_STD_ID_MAX) || (!s_whitelist[id])) {
        if (alert_out != NULL) {
            buildAlert(alert_out, frame, IDS_RULE_R1_WHITELIST);
        }
        return IDS_RULE_R1_WHITELIST;
    }

    /* ── [R2] DLC check ──────────────────────────────────── */
    if ((s_expectedDlc[id] != 0U) && (frame->dlc != s_expectedDlc[id])) {
        if (alert_out != NULL) {
            buildAlert(alert_out, frame, IDS_RULE_R2_DLC);
        }
        return IDS_RULE_R2_DLC;
    }

    /* ── [R4] Token bucket — global bus flood check ───────── */
    refillTokenBucket(ts);
    s_tokenBucket--;

    if (s_tokenBucket < 0) {
        if (alert_out != NULL) {
            buildAlert(alert_out, frame, IDS_RULE_R4_BURST);
        }
        return IDS_RULE_R4_BURST;
    }

    /* ── [R3] Period anomaly check ───────────────────────── */
    if (s_expectedPeriod[id] > 0U) {
        if (s_isFirstFrame[id]) {
            /* First arrival — store baseline, pass unconditionally */
            s_isFirstFrame[id]   = false;
            s_lastTimestamp[id]  = ts;
            /* Intentional: no delta check on first frame (ISO 26262 mindset —
             * do NOT rely on delta from zero-initialised array). */
        } else {
            uint32_t delta      = ts - s_lastTimestamp[id];
            uint32_t lower_bound = s_expectedPeriod[id] - IDS_JITTER_MS;

            if (delta < lower_bound) {
                /* Frame arrived too fast — likely injected attack frame */
                if (alert_out != NULL) {
                    buildAlert(alert_out, frame, IDS_RULE_R3_PERIOD);
                }
                /* Do NOT update s_lastTimestamp — preserve legitimate baseline */
                return IDS_RULE_R3_PERIOD;
            }
        }
    }

    /* ── State update — frame passed all rules ───────────── */
    s_lastTimestamp[id] = ts;

    return IDS_RULE_CLEAN;
}
