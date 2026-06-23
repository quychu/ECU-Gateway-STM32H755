/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "main_app_cm4.h"
#include "cmsis_os2.h"
#include "can_app.h"
#include "fdcan.h"
#include "openamp.h"

extern struct rpmsg_endpoint rp_endpoint;

/* Private constants ---------------------------------------------------------*/
#define BLINK_CM4_PERIOD_MS         250U

#define CAN_TX_HEARTBEAT_PERIOD_MS  1000U

#define CAN_RX_BLOCK_TIMEOUT_MS     100U

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* Private task handles ------------------------------------------------------*/

static osThreadId_t s_blinkHandle;
static osThreadId_t s_canEngineHandle;
static osThreadId_t s_ipcTxHandle;
static osThreadId_t s_ipcRxHandle;

/* Task attribute tables -----------------------------------------------------*/

/** @brief BlinkTask attributes -- lowest priority, purely cosmetic. */
static const osThreadAttr_t k_blinkAttr = {
    .name       = "BlinkCM4",
    .stack_size = 256U * 4U,
    .priority   = (osPriority_t)osPriorityNormal,
};

static const osThreadAttr_t k_canEngineAttr = {
    .name       = "CanEngineTask",
    .stack_size = 512U * 4U,
    .priority   = (osPriority_t)osPriorityHigh,
};

static const osThreadAttr_t k_ipcTxAttr = {
    .name       = "IpcTxTask",
    .stack_size = 512U * 4U,
    .priority   = (osPriority_t)osPriorityAboveNormal,
};

static const osThreadAttr_t k_ipcRxAttr = {
    .name       = "IpcRxCM4",
    .stack_size = 512U * 4U,
    .priority   = (osPriority_t)osPriorityHigh,
};

static void BlinkTask(void *argument)
{
    (void)argument;

    for (;;) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        osDelay(BLINK_CM4_PERIOD_MS);
    }
}

/**
 * @brief  Heartbeat_Send -- TX test frames 0x123 (FDCAN1) + 0x125 (FDCAN2).
 * @note   Call from CanEngineTask to enable. Comment out call to disable.
 *         Keep this function -- used for FDCAN TX/RX debug via SavvyCAN.
 */
static void Heartbeat_Send(uint8_t *tx1Ctr, uint8_t *tx2Ctr, uint32_t *lastTick)
{
    uint32_t now = HAL_GetTick();
    if ((now - *lastTick) >= CAN_TX_HEARTBEAT_PERIOD_MS) {
        CanEngine_SendFrame(&hfdcan1, CAN_ENGINE_TX_FDCAN1_ID, (*tx1Ctr)++);
        CanEngine_SendFrame(&hfdcan2, CAN_ENGINE_TX_FDCAN2_ID, (*tx2Ctr)++);
        *lastTick = now;
    }
}

static void CanEngineTask(void *argument)
{
    (void)argument;
    if (CanEngine_Init() != HAL_OK) {
        Error_Handler();
    }

    QueueHandle_t rxQueue    = CanEngine_GetRxQueue();
    CanFrame_t    rxFrame    = {0};
    uint8_t       tx1Counter = 0U;
    uint8_t       tx2Counter = 0U;
    uint32_t      lastTxTick = 0U;

    for (;;) {
        Heartbeat_Send(&tx1Counter, &tx2Counter, &lastTxTick);

        if (xQueueReceive(rxQueue, &rxFrame, pdMS_TO_TICKS(CAN_RX_BLOCK_TIMEOUT_MS)) == pdTRUE) {
            if (Gw_RouteFrame(&rxFrame) == HAL_OK) {
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            }
        }
    }
}

static void IpcTxTask(void *argument)
{
    (void)argument;
    CanFrame_t    frame = {0};

    /* Race guard: same pattern as IpcRxTask_CM4 — wait for CanEngine_Init(). */
    QueueHandle_t ipcQ;
    while ((ipcQ = CanEngine_GetIpcTxQueue()) == NULL) {
        osDelay(5U);
    }

    for (;;) {
        if (xQueueReceive(ipcQ, &frame, portMAX_DELAY) == pdTRUE) {
            __DSB(); __DMB();
            OPENAMP_send(&rp_endpoint, &frame, (int)sizeof(CanFrame_t));
        }
    }
}

/**
 * @brief  CM4 IPC RX Task — receives UDS TX commands from CM7 and puts them
 *         on the FDCAN1 bus as ISO-TP response frames (ID 0x7E8).
 *
 * @note   Flow: CM7 IpcTxTask → OPENAMP_send → SRAM4 → HSEM notify →
 *               OPENAMP_check_for_message() → rpmsg_recv_callback (main.c) →
 *               xQueueSend(s_ipcRxQueue) → HERE → CanEngine_SendUdsResponse
 *               → FDCAN_TXBAR.AR[n] set → FDCAN1 TX → 0x7E8 on CAN bus.
 *
 * @note   Priority HIGH: ensures UDS response goes out promptly (< 50 ms
 *         P2_Server budget, ISO 14229-1 §7.4.2.2).
 */
static void IpcRxTask_CM4(void *argument)
{
    (void)argument;
    CanFrame_t    frame   = {0};

    /* Race guard: CanEngine_Init() runs inside CanEngineTask (same HIGH priority).
     * IpcRxTask_CM4 may be scheduled first → s_ipcRxQueue is still NULL.
     * Spin with 5 ms yield until CanEngine_Init() creates the queue.          */
    QueueHandle_t ipcRxQ;
    while ((ipcRxQ = CanEngine_GetIpcRxQueue()) == NULL) {
        osDelay(5U);
    }

    for (;;) {
        /* Poll OpenAMP — fires rpmsg_recv_callback which pushes to ipcRxQ */
        OPENAMP_check_for_message();

        /* Drain all queued commands from CM7 (UDS TX + IDS alerts) */
        while (xQueueReceive(ipcRxQ, &frame, 0U) == pdTRUE) {
            if (frame.src_bus == GW_BUS_UDS_TX) {
                /* UDS response frame — put on FDCAN1 TX FIFO → 0x7E8 on bus */
                if (CanEngine_SendUdsResponse(&frame) != HAL_OK) {
                    /* TX FIFO full — drop for now (M6 scope) */
                }
            } else if (frame.src_bus == GW_BUS_IDS_ALERT) {
                /* IDS violation alert from CM7 (M7).
                 * data[0] = rule code (R1..R4)
                 * id      = offending CAN ID
                 * data[1..dlc] = offending raw payload (forensics).
                 *
                 * HW: PE1 (LED2 yellow) — distinct from LED1 (PB0) CM4 heartbeat. */
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
                printf("[CM4] IDS_ALERT rule=R%u id=0x%03lX dlc=%u data=%02X %02X %02X %02X\r\n",
                       (unsigned)frame.data[0],
                       (unsigned long)frame.id,
                       (unsigned)frame.dlc,
                       frame.data[1], frame.data[2],
                       frame.data[3], frame.data[4]);
            }
        }

        osDelay(1U);   /* 1 ms poll — adds ≤ 1 ms to UDS response latency */
    }
}

void AppCM4_Init(void)
{
    s_blinkHandle     = osThreadNew(BlinkTask,      NULL, &k_blinkAttr);
    s_canEngineHandle = osThreadNew(CanEngineTask,  NULL, &k_canEngineAttr);
    s_ipcTxHandle     = osThreadNew(IpcTxTask,      NULL, &k_ipcTxAttr);
    s_ipcRxHandle     = osThreadNew(IpcRxTask_CM4,  NULL, &k_ipcRxAttr);

    configASSERT(s_blinkHandle     != NULL);
    configASSERT(s_canEngineHandle != NULL);
    configASSERT(s_ipcTxHandle     != NULL);
    configASSERT(s_ipcRxHandle     != NULL);
}
