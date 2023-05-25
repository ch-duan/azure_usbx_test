/*
 * uart_comm_helper.c
 *
 *  Created on: Jul 25, 2020
 *      Author: mengjingshi
 */
#include "uart_comm_helper.h"

#include <memory.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_uart.h"

#define FALSE 0
#define TRUE  1

static void delay_us(uint32_t u_sec) {
  uint16_t cnt = 0;

  while (u_sec--) {
    for (cnt = 168 / 5; cnt > 0; cnt--)
      ;
  }
}

// 延时nms
//  nms:要延时的ms数
static void delay_ms(uint16_t nms) {
  uint32_t i;
  for (i = 0; i < nms; i++)
    delay_us(1000);
}

void UART_IDLE_Callback(LinkLayerHandler *self, UART_HandleTypeDef *huart) {
#if USE_RXEVENTCALLBACK == 0
  uint8_t tmp1 = 0, tmp2 = 0;
  uint16_t len1 = 0, len2 = 0;
  tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE);
  tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE);
  __HAL_UART_CLEAR_IDLEFLAG(huart);
  len1 = RECV_BUF_MAX - __HAL_DMA_GET_COUNTER(huart->hdmarx);
  delay_ms(50);
  len2 = RECV_BUF_MAX - __HAL_DMA_GET_COUNTER(huart->hdmarx);
  if (len1 != len2) {
    return;
  }
  if ((tmp1 != RESET) && (tmp2 != RESET)) {
    self->FrameLength = len2;
  }
  self->deliver_packet(self->m_framePayload, self->FrameLength, self->arg);
  HAL_UART_DMAResume(huart);
  __HAL_DMA_DISABLE(huart->hdmarx);
  memset(self->m_framePayload, 0, self->FrameLength);
  huart->hdmarx->Instance->NDTR = RECV_BUF_MAX;
  __HAL_DMA_ENABLE(huart->hdmarx);
#else
  self->FrameLength = RECV_BUF_MAX - __HAL_DMA_GET_COUNTER(huart->hdmarx);
  if (self->FrameLength == 0) {
    // not receive idle
    return;
  }
  self->deliver_packet(self->m_framePayload, self->FrameLength, self->arg);
  HAL_UART_DMAResume(huart);
  __HAL_DMA_DISABLE(huart->hdmarx);
  memset(self->m_framePayload, 0, self->FrameLength);
  huart->hdmarx->Instance->NDTR = RECV_BUF_MAX;
  __HAL_DMA_ENABLE(huart->hdmarx);
#endif
}

void LLInit(LinkLayerHandler *self, UART_HandleTypeDef *uart) {
  self->uart = uart;
  __HAL_UART_ENABLE_IT(self->uart, UART_IT_IDLE);
#if USE_RXEVENTCALLBACK == 0
  HAL_UART_Receive_DMA(self->uart, self->m_framePayload, RECV_BUF_MAX);
#else
  HAL_UARTEx_ReceiveToIdle_DMA(self->uart, self->m_framePayload, RECV_BUF_MAX);
#endif
}

#if defined(INC_FREERTOS_H)
extern volatile int memoryLockOpen;
#include "FreeRTOS.h"
#include "task.h"
#endif
void LLSendData(LinkLayerHandler *self, uint8_t *data, size_t len) {
  if (self->uart->hdmatx != NULL) {
    HAL_UART_Transmit_DMA(self->uart, (uint8_t *) data, (uint16_t) len);
  } else {
    HAL_UART_Transmit(self->uart, (uint8_t *) data, (uint16_t) len, 0xfffff);
  }
}

void UartPacketHandlerInit(UartMessageHandler *msgHandler, UART_HandleTypeDef *uart, PacketArrived pHandlerPacket) {
  msgHandler->priv = NULL;
  msgHandler->llhandler.arg = msgHandler;
  LLInit(&(msgHandler->llhandler), uart);
  msgHandler->llhandler.deliver_packet = pHandlerPacket;
}
