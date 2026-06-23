/**
 ******************************************************************************
 * @file    ids_rule.h
 * @brief   IDS (Intrusion Detection System) — 4-rule engine for CM7.
 *
 * @note    AUTOSAR mapping: BSW Intrusion Detection Module (ISO/SAE 21434).
 *          Runs on CM7 Application Layer.
 *          Input : IpcFrame_t copies from CM4 via rpmsg (s_idsRxQueue).
 *          Output: IpcFrame_t alert with src_bus=IPC_BUS_IDS_ALERT pushed to
 *                  CM7 IPC TX queue → rpmsg → CM4 → LED + printf forensics.
 *
 * @note    Rule summary:
 *          R1 — ID Whitelist   : ID not in Db_Whitelist[]
 *          R2 — DLC Check      : dlc != Db_ExpectedDlc[id]
 *          R3 — Period Anomaly : frame arrived < (expected_period - jitter)
 *          R4 — Burst Flood    : token bucket exhausted (global bus load)
 *
 * @note    Performance target: < 10 µs/frame (Section 6.3 CLAUDE.md).
 *          All state arrays use CAN ID as index → O(1) lookup, no malloc.
 ******************************************************************************
 */

#ifndef IDS_RULE_H_
#define IDS_RULE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "ipc_types.h"

/* ── Rule violation codes (stored in alert data[0]) ─────────────────────── */
#define IDS_RULE_CLEAN          0x00U   /**< No violation                    */
#define IDS_RULE_R1_WHITELIST   0x01U   /**< Unknown CAN ID                  */
#define IDS_RULE_R2_DLC         0x02U   /**< DLC mismatch                    */
#define IDS_RULE_R3_PERIOD      0x03U   /**< Frame injected too fast         */
#define IDS_RULE_R4_BURST       0x04U   /**< Token bucket empty (DoS flood)  */

/* ── Tuning constants ────────────────────────────────────────────────────── */
#define IDS_STD_ID_MAX          2048U   /**< 11-bit CAN std ID range         */
#define IDS_JITTER_MS           50U     /**< Period tolerance ±50 ms         */
#define IDS_BURST_MAX_TOKENS    15U     /**< Token bucket capacity (burst allowance)      */
#define IDS_TOKEN_REFILL_RATE   1U      /**< 1 token per 100ms = 10 frames/s sustained OK */
#define IDS_TOKEN_WINDOW_MS     100U    /**< Refill window [ms]                           */
/* Sizing rationale:
 *   Legit traffic: ≤ 10 frames/s → 1 token consumed, 1 refilled → equilibrium.
 *   Attack flood:  > 10 frames/s → drain outpaces refill → R4 fires in ~15 frames.
 *   With 30-frame flood @ effective 16ms/frame (6-7 fps per ID, ~15 IDs):
 *   drain = 15 tokens, refill ≈ 2-3 tokens → net -12 → R4 fires within 17 frames. */

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * @brief  Initialize all IDS state tables.
 *         Must be called once before the FreeRTOS scheduler starts, or at
 *         the beginning of IdsRuleTask before the main loop.
 *
 * @note   Zeroes s_lastTimestamp[], s_isFirstFrame[] = true, token bucket
 *         reset to IDS_BURST_MAX_TOKENS. Runs in task context (not ISR).
 */
void Ids_Init(void);

/**
 * @brief  Evaluate all 4 rules against a single incoming CAN frame.
 *
 * @param  frame      Pointer to IpcFrame_t received from CM4 via rpmsg.
 * @param  alert_out  Output parameter — filled with forensic alert payload
 *                    if any rule fires. Caller must check return value before
 *                    using alert_out. May be NULL if caller does not need
 *                    alert details (used for unit testing).
 *
 * @retval IDS_RULE_CLEAN (0)   Frame passed all rules.
 * @retval IDS_RULE_R1..R4      First violated rule code. alert_out filled:
 *                               .src_bus   = IPC_BUS_IDS_ALERT
 *                               .id        = offending CAN ID
 *                               .dlc       = offending DLC
 *                               .data[0]   = rule code (IDS_RULE_R*)
 *                               .data[1..8]= raw frame payload (forensics)
 *                               .timestamp = capture timestamp [ms]
 *
 * @note   State update (s_lastTimestamp[]) only occurs when frame passes
 *         all rules, preserving the clean baseline timestamp.
 */
uint8_t Ids_CheckFrame(const IpcFrame_t *frame, IpcFrame_t *alert_out);

#ifdef __cplusplus
}
#endif

#endif /* IDS_RULE_H_ */
