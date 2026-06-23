/**
 * @file    system_init_m2.c
 * @brief   M2: DWT clock verify + memcpy benchmark (cache vs no-cache)
 * @note    AUTOSAR MCAL layer — call from main() before osKernelStart().
 *          MPU + Cache config handled by CubeMX-generated MPU_Config() +
 *          SCB_EnableICache/DCache() in main.c.
 */

#include "stm32h7xx_hal.h"
#include "system_init_m2.h"
#include <string.h>
#include <stdio.h>

/* ── Constants ─────────────────────────────────────── */
#define BENCHMARK_SIZE  (1024U)     /* 1 KB */

/* ── Benchmark buffers ─────────────────────────────── */
/* Placed in .bss → RAM_D1 (AXI SRAM, 0x24000000) — cacheable region.
 * D-Cache on CM7 will cache these lines → speedup visible in Test 2. */
static uint8_t src_buf[BENCHMARK_SIZE];
static uint8_t dst_buf[BENCHMARK_SIZE];

/* ─────────────────────────────────────────────────── */

/**
 * @brief  Enable DWT cycle counter.
 * @note   Registers:
 *           CoreDebug->DEMCR bit 24 (TRCENA)  — master gate for DWT/ITM/ETM
 *           DWT->CYCCNT                        — 32-bit cycle counter
 *           DWT->CTRL  bit 0  (CYCCNTENA)      — enable CYCCNT
 */
void Dwt_Init(void)
{
    /* Step 1: Enable DWT/ITM/ETM master gate — use |= to preserve other bits
     *         (e.g. VC_CORERESET, MON_EN used by debugger) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Step 2: Reset counter to 0 before enabling */
    DWT->CYCCNT = 0;

    /* Step 3: Start counting — 1 tick = 1 CPU cycle */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  Verify SYSCLK via DWT: delay 1 ms, count cycles.
 *         MHz = cycles / 1000  (1 ms = 1/1000 s)
 *         Expected: ~400 (MHz) for CM7 @ 400 MHz
 */
void Dwt_VerifyClock(void)
{
    uint32_t t_start = DWT->CYCCNT;
    HAL_Delay(100);                         /* 100 ms — HAL adds +1 tick guard,
                                             * 100ms makes 1-tick error = 1% only */
    uint32_t cycles = DWT->CYCCNT - t_start;

    /* cycles in 100 ms → MHz = cycles / 100000 */
    printf("~%lu MHz\r\n", cycles / 100000UL);
}

/**
 * @brief  Benchmark memcpy 1 KB: no D-Cache vs warm D-Cache.
 *         Speedup printed as fixed-point x10 to avoid float.
 */
void Benchmark_Memcpy(void)
{
    uint32_t cycles_no_cache, cycles_with_cache;

    /* Fill src with known pattern */
    for (uint32_t i = 0; i < BENCHMARK_SIZE; i++) {
        src_buf[i] = (uint8_t)(i & 0xFF);
    }

    /* ── Test 1: D-Cache DISABLED ───────────────────
     * Every load/store goes to physical AXI SRAM (3-4 cycle latency each).
     * Ref: Cortex-M7 TRM — cache miss penalty ~10-20 cycles for 32B line */
    SCB_DisableDCache();
    DWT->CYCCNT = 0;
    memcpy(dst_buf, src_buf, BENCHMARK_SIZE);
    cycles_no_cache = DWT->CYCCNT;
    SCB_EnableDCache();     /* SCB_EnableDCache() internally invalidates first */

    /* ── Test 2: D-Cache ENABLED (warm) ─────────────
     * Pre-warm: load all src/dst cache lines first,
     * then measure — pure cache-hit path. */
    memcpy(dst_buf, src_buf, BENCHMARK_SIZE);   /* warm-up run */
    DWT->CYCCNT = 0;
    memcpy(dst_buf, src_buf, BENCHMARK_SIZE);   /* measured run */
    cycles_with_cache = DWT->CYCCNT;

    /* Speedup = no_cache / with_cache, displayed as X.Y */
    uint32_t sp_x10 = (cycles_with_cache > 0U)
                      ? (cycles_no_cache * 10UL / cycles_with_cache)
                      : 0UL;

    printf("[M2] no_cache=%lu | with_cache=%lu | speedup=%lu.%lux\r\n",
           cycles_no_cache,
           cycles_with_cache,
           sp_x10 / 10UL,
           sp_x10 % 10UL);
}
