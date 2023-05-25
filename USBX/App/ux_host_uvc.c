/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    ux_host_uvc.c
 * @author  MCD Application Team
 * @brief   USBX host applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_usbx_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ux_host_class_video.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern UX_HOST_CLASS_VIDEO *video_class;
extern TX_SEMAPHORE data_received_semaphore;
extern uint8_t video_buffer[20 * 1024] __attribute__((section(".sram")));
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void uvc_process_thread_entry(ULONG arg);
void Error_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
 * @brief  uvc_process_thread_entry .
 * @param  ULONG arg
 * @retval Void
 */
void uvc_process_thread_entry(ULONG arg) {
  while (1) {
    /* Suspend here until a transfer callback is called. */
    UINT status = tx_semaphore_get(&data_received_semaphore, TX_WAIT_FOREVER);
    if (status != UX_SUCCESS)
      Error_Handler();

    /* Received data. The callback function needs to obtain the actual
       number of bytes received, so the application routine can read the
       correct amount of data from the buffer. */

    /* Application can now consume video data while the video device stores
       the data into the other buffer. */

    /* Add the buffer back for video transfer. */
    status = ux_host_class_video_transfer_buffer_add(video_class, video_buffer);
    if (status != UX_SUCCESS)
      Error_Handler();
  }
}

/* USER CODE END 1 */
