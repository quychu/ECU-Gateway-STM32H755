#ifndef MAIN_APP_CM7_H_
#define MAIN_APP_CM7_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "queue.h"
#include "ipc_types.h"

/**
 * @brief  Initialize all CM7 application tasks and queues.
 *         Call from MX_FREERTOS_Init() → USER CODE RTOS_THREADS.
 *         Must be called AFTER osKernelInitialize(), BEFORE osKernelStart().
 */
void AppCM7_Init(void);

/**
 * @brief  Return the ISO-TP RX queue handle.
 *         IpcRxTask pushes IpcFrame_t received from CM4 into this queue.
 *         IsoTpUdsTask dequeues for normal processing.
 *         IsoTp_StartTx() also dequeues for FC reception during multi-frame TX.
 *
 * @retval QueueHandle_t (NULL if AppCM7_Init() not yet called)
 */
QueueHandle_t AppCM7_GetIsoTpRxQueue(void);

/**
 * @brief  Return the IDS RX queue handle.
 *         rpmsg_recv_callback (main.c) fan-outs every IpcFrame_t to BOTH
 *         s_isoTpRxQueue AND s_idsRxQueue so IdsRuleTask gets all frames.
 *
 * @retval QueueHandle_t (NULL if AppCM7_Init() not yet called)
 */
QueueHandle_t AppCM7_GetIdsRxQueue(void);

/**
 * @brief  Push a frame into the CM7→CM4 IPC TX queue.
 *         Used by IdsRuleTask to send alert frames to CM4 via rpmsg.
 *         Internally enqueues to s_cm7IpcTxQueue (IpcTxTask_CM7 drains it).
 *
 * @param  frame  Pointer to IpcFrame_t alert (src_bus = IPC_BUS_IDS_ALERT).
 * @retval pdTRUE on success, pdFALSE if queue full.
 */
BaseType_t AppCM7_PushIpcTx(const IpcFrame_t *frame);

/**
 * @brief  Send data to CM4 via rpmsg (wraps OPENAMP_send on rp_endpoint).
 *         Called by CM7 IpcTxTask to deliver UDS response frames to CM4.
 *
 * @param  data  Pointer to payload (typically IpcFrame_t, 20 bytes).
 * @param  len   Payload length in bytes.
 * @retval 0 on success, negative on OPENAMP error.
 */
int AppCM7_SendIpc(const void *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_APP_CM7_H_ */
