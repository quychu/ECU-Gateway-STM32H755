#ifndef MAIN_APP_CM4_H_
#define MAIN_APP_CM4_H_

#ifdef __cplusplus
extern "C" {
#endif

/* CM4/Core/Inc/main_app_cm4.h */
/**
 * @brief  Initialize CM4 application tasks.
 * Call this inside USER CODE BEGIN RTOS_THREADS in main.c (CM4)
 * AFTER osKernelInitialize(), BEFORE osKernelStart().
 */
void AppCM4_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_APP_CM4_H_ */
