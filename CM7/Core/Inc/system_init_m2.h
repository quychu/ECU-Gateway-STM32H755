#ifndef SYSTEM_INIT_M2_H_
#define SYSTEM_INIT_M2_H_

/**
 * @file    system_init_m2.h
 * @brief   M2 public API: DWT clock verify + memcpy benchmark
 * @note    AUTOSAR MCAL layer — include in main.c USER CODE Includes
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Enable DWT cycle counter (TRCENA + CYCCNTENA).
 *         Must call BEFORE Dwt_VerifyClock() or Benchmark_Memcpy().
 */
void Dwt_Init(void);

/**
 * @brief  Measure SYSCLK via DWT: delay 1 ms, count cycles.
 *         Prints: "[M2] SYSCLK verify: ~XXX MHz"  (expected ~400)
 */
void Dwt_VerifyClock(void);

/**
 * @brief  Benchmark memcpy 1 KB: no D-Cache vs warm D-Cache.
 *         Prints: "[M2] no_cache=X | with_cache=Y | speedup=Z.Zx"
 *         Record result in CLAUDE.md Section 6.3.
 */
void Benchmark_Memcpy(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_INIT_M2_H_ */
