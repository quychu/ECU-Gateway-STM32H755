/**
 ******************************************************************************
 * @file    main_app_cm7.c
 * @brief   CM7 Application Tasks — M6: UDS + ISO-TP stack added.
 *
 * @note    Task architecture (M6):
 *
 *   IpcRxTask   (HIGH)        — polls OPENAMP_check_for_message()
 *                                 → rpmsg_recv_callback (main.c)
 *                                 → pushes IpcFrame_t to s_isoTpRxQueue
 *
 *   IsoTpUdsTask (NORMAL)     — dequeues s_isoTpRxQueue
 *                                 → IsoTp_ProcessRxFrame()
 *                                 → (on complete PDU) Uds_ProcessRequest()
 *                                 → Uds builds response → IsoTp_StartTx()
 *                                 → IsoTp_StartTx pushes to s_cm7IpcTxQueue
 *                                   (blocks on s_isoTpRxQueue for FC if MF TX)
 *
 *   IpcTxTask_CM7 (ABOVE_NORMAL) — dequeues s_cm7IpcTxQueue
 *                                 → AppCM7_SendIpc() → OPENAMP_send → CM4
 *                                 → CM4 IpcRxTask → CanEngine_SendUdsResponse
 *                                 → FDCAN1 TX → 0x7E8 on CAN bus
 *
 *   BlinkTask   (NORMAL)      — LED3 (PB14) 500 ms heartbeat
 *   LoggerTask  (LOW)         — UART tick log every 1 s
 *
 * @note    AUTOSAR layer: BSW (DCM + COM) on CM7.
 ******************************************************************************
 */

/* Includes ----------------------------------------------------------------- */
#include "main.h"
#include "main_app_cm7.h"
#include "iso_tp.h"
#include "uds_server.h"
#include "ipc_types.h"
#include "ids_rule.h"
#include "cmsis_os2.h"
#include "openamp.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>

/* Constants ---------------------------------------------------------------- */
#define BLINK_CM7_PERIOD_MS    500U
#define LOGGER_PERIOD_MS      1000U

/** Queue depths */
#define ISOTP_RX_QUEUE_DEPTH    16U   /**< IpcRxTask → IsoTpUdsTask         */
#define IDS_RX_QUEUE_DEPTH       8U   /**< IpcRxTask → IdsRuleTask (monitoring path — drop OK) */
#define CM7_IPC_TX_QUEUE_DEPTH  16U   /**< IsoTpUdsTask + IdsRuleTask → IpcTxTask_CM7  */

/* Private queues ----------------------------------------------------------- */
static QueueHandle_t s_isoTpRxQueue  = NULL;  /**< IpcFrame_t from CM4 → IsoTp       */
static QueueHandle_t s_idsRxQueue    = NULL;  /**< IpcFrame_t from CM4 → IdsRuleTask */
static QueueHandle_t s_cm7IpcTxQueue = NULL;  /**< IpcFrame_t to CM4 → FDCAN         */

/* Task handles ------------------------------------------------------------- */
static osThreadId_t s_blinkHandle;
static osThreadId_t s_loggerHandle;
static osThreadId_t s_ipcRxHandle;
static osThreadId_t s_isoTpUdsHandle;
static osThreadId_t s_ipcTxHandle;
static osThreadId_t s_idsRuleHandle;

/* Task attributes ---------------------------------------------------------- */
static const osThreadAttr_t k_blinkAttr = {
    .name       = "BlinkCM7",
    .stack_size = 256U * 4U,
    .priority   = (osPriority_t)osPriorityNormal,
};
static const osThreadAttr_t k_loggerAttr = {
    .name       = "Logger",
    .stack_size = 512U * 4U,
    .priority   = (osPriority_t)osPriorityLow,
};
static const osThreadAttr_t k_ipcRxAttr = {
    .name       = "IpcRxCM7",
    .stack_size = 512U * 4U,
    .priority   = (osPriority_t)osPriorityHigh,
};
static const osThreadAttr_t k_isoTpUdsAttr = {
    .name       = "IsoTpUDS",
    .stack_size = 1024U * 4U,   /* larger: holds 4095-byte rxBuf via static */
    .priority   = (osPriority_t)osPriorityNormal,
};
static const osThreadAttr_t k_ipcTxAttr = {
    .name       = "IpcTxCM7",
    .stack_size = 512U * 4U,
    .priority   = (osPriority_t)osPriorityAboveNormal,
};
static const osThreadAttr_t k_idsRuleAttr = {
    .name       = "IdsRuleTask",
    .stack_size = 256U * 4U,   /* 1 KB — IdsRuleTask holds 2× IpcFrame_t (40B) on stack, rest is static */
    .priority   = (osPriority_t)osPriorityNormal,
};

/* ---------------------------------------------------------------------------
 * @brief  MCAL-level UART write — bypass HAL_UART_Transmit.
 * @note   Poll USART_ISR.TXE_TXFNF (bit 7) then write USART_TDR.
 *         RM0399 Ch.52 §52.8.10: TXE = 1 when Tx data reg is empty.
 * ---------------------------------------------------------------------------*/
static int uart3_write(int ch)
{
    while (!(USART3->ISR & USART_ISR_TXE_TXFNF)) {}
    USART3->TDR = (uint32_t)(ch & 0xFF);
    return ch;
}

int __io_putchar(int ch)
{
    return uart3_write(ch);
}

/* Task implementations ----------------------------------------------------- */

static void BlinkTask(void *argument)
{
    (void)argument;
    for (;;) {
        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
        osDelay(BLINK_CM7_PERIOD_MS);
    }
}

static void LoggerTask(void *argument)
{
    (void)argument;
    for (;;) {
        printf("[CM7] tick=%lu sess=%u\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned)0U);  /* session printed by UDS layer */
        osDelay(LOGGER_PERIOD_MS);
    }
}

/**
 * @brief  CM7 IPC RX Task — polls OPENAMP for incoming frames from CM4.
 *         rpmsg_recv_callback (main.c) handles the push to s_isoTpRxQueue.
 * @note   Priority HIGH ensures FC frames (from tester via CM4) reach
 *         s_isoTpRxQueue promptly while IsoTpUdsTask is blocking on it.
 */
static void IpcRxTask(void *argument)
{
    (void)argument;
    for (;;) {
        OPENAMP_check_for_message();
        osDelay(1U);
    }
}

/**
 * @brief  ISO-TP + UDS combined task.
 *         Dequeues CAN frames from s_isoTpRxQueue, feeds ISO-TP FSM.
 *         ISO-TP calls Uds_ProcessRequest() when PDU is assembled.
 *         UDS calls IsoTp_StartTx() → pushes response frames to s_cm7IpcTxQueue.
 */
static void IsoTpUdsTask(void *argument)
{
    (void)argument;

    /* Both modules initialised here (FreeRTOS scheduler running) */
    IsoTp_Init(s_isoTpRxQueue, s_cm7IpcTxQueue);
    Uds_Init();

    IpcFrame_t frame;
    for (;;) {
        /* Block until a frame arrives from CM4 (or FC during multi-frame TX) */
        if (xQueueReceive(s_isoTpRxQueue, &frame, portMAX_DELAY) == pdTRUE) {
            IsoTp_ProcessRxFrame(&frame);
        }
    }
}

/**
 * @brief  CM7 IPC TX Task — sends UDS response frames to CM4 via rpmsg.
 *         ABOVE_NORMAL priority so FF/CF are dispatched to CM4 before
 *         IsoTpUdsTask loops back to wait for FC — avoiding timing issues.
 */
static void IpcTxTask_CM7(void *argument)
{
    (void)argument;
    IpcFrame_t frame;
    for (;;) {
        if (xQueueReceive(s_cm7IpcTxQueue, &frame, portMAX_DELAY) == pdTRUE) {
            __DSB(); __DMB();   /* flush before SRAM4 write in OPENAMP_send */
            int ret = AppCM7_SendIpc(&frame, sizeof(IpcFrame_t));
            if (ret < 0) {
                printf("[CM7] IpcTx ERR: OPENAMP_send=%d\r\n", ret);
            }
        }
    }
}

/**
 * @brief  IDS Rule Task — evaluates R1–R4 on every frame copy from CM4.
 *
 * @note   Receives IpcFrame_t from s_idsRxQueue (fan-out from rpmsg_recv_callback).
 *         On violation: builds forensic IpcFrame_t alert → AppCM7_PushIpcTx()
 *         → IpcTxTask_CM7 → rpmsg → CM4 IpcRxTask → LED2 + printf.
 *
 * @note   Priority NORMAL (lower than IsoTpUdsTask) — IDS is monitoring path,
 *         never blocks fast UDS response timing.
 *         Race guard: spins on s_idsRxQueue == NULL until AppCM7_Init() runs.
 */
static void IdsRuleTask(void *argument)
{
    (void)argument;

    /* Race guard: same pattern as M6 lesson learned [M6-001].
     * IdsRuleTask may be scheduled before AppCM7_Init() creates s_idsRxQueue. */
    while (s_idsRxQueue == NULL) {
        osDelay(5U);
    }

    Ids_Init();
    printf("[CM7] IdsRuleTask ready — R1/R2/R3/R4 active\r\n");

    IpcFrame_t frame;
    IpcFrame_t alert;

    for (;;) {
        if (xQueueReceive(s_idsRxQueue, &frame, portMAX_DELAY) == pdTRUE) {
            uint8_t violation = Ids_CheckFrame(&frame, &alert);
            if (violation != IDS_RULE_CLEAN) {
                printf("[IDS]  ALERT rule=R%u id=0x%03lX dlc=%u ts=%lu\r\n",
                       (unsigned)violation,
                       (unsigned long)frame.id,
                       (unsigned)frame.dlc,
                       (unsigned long)frame.timestamp);

                /* Push alert to CM4 via shared IPC TX queue */
                if (AppCM7_PushIpcTx(&alert) != pdTRUE) {
                    printf("[IDS]  IpcTxQ full — alert dropped!\r\n");
                }
            }
        }
    }
}

/* Public API --------------------------------------------------------------- */

QueueHandle_t AppCM7_GetIsoTpRxQueue(void)
{
    return s_isoTpRxQueue;
}

QueueHandle_t AppCM7_GetIdsRxQueue(void)
{
    return s_idsRxQueue;
}

BaseType_t AppCM7_PushIpcTx(const IpcFrame_t *frame)
{
    if (s_cm7IpcTxQueue == NULL || frame == NULL) {
        return pdFALSE;
    }
    return xQueueSend(s_cm7IpcTxQueue, frame, 0U);
}

void AppCM7_Init(void)
{
    /* Create queues FIRST — rpmsg_recv_callback may fire immediately after
     * osKernelStart and needs all RX queues to be non-NULL. */
    s_isoTpRxQueue  = xQueueCreate(ISOTP_RX_QUEUE_DEPTH,   sizeof(IpcFrame_t));
    s_idsRxQueue    = xQueueCreate(IDS_RX_QUEUE_DEPTH,      sizeof(IpcFrame_t));
    s_cm7IpcTxQueue = xQueueCreate(CM7_IPC_TX_QUEUE_DEPTH,  sizeof(IpcFrame_t));
    configASSERT(s_isoTpRxQueue  != NULL);
    configASSERT(s_idsRxQueue    != NULL);
    configASSERT(s_cm7IpcTxQueue != NULL);

    /* Spawn tasks */
    s_blinkHandle    = osThreadNew(BlinkTask,     NULL, &k_blinkAttr);
    s_loggerHandle   = osThreadNew(LoggerTask,    NULL, &k_loggerAttr);
    s_ipcRxHandle    = osThreadNew(IpcRxTask,     NULL, &k_ipcRxAttr);
    s_isoTpUdsHandle = osThreadNew(IsoTpUdsTask,  NULL, &k_isoTpUdsAttr);
    s_ipcTxHandle    = osThreadNew(IpcTxTask_CM7, NULL, &k_ipcTxAttr);
    s_idsRuleHandle  = osThreadNew(IdsRuleTask,   NULL, &k_idsRuleAttr);

    configASSERT(s_blinkHandle    != NULL);
    configASSERT(s_loggerHandle   != NULL);
    configASSERT(s_ipcRxHandle    != NULL);
    configASSERT(s_isoTpUdsHandle != NULL);
    configASSERT(s_ipcTxHandle    != NULL);
    configASSERT(s_idsRuleHandle  != NULL);
}
