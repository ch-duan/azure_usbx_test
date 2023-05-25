#ifndef UART_COMM_HELPER_H_
#define UART_COMM_HELPER_H_

#include "stm32f4xx_hal.h"
// dma receive buf queue
#define RECV_BUF_MAX        12000
#define USE_RXEVENTCALLBACK 1
typedef void (*PacketArrived)(uint8_t *packet, uint16_t packetLen, void *arg);

typedef struct {
  UART_HandleTypeDef *uart;
  PacketArrived deliver_packet;
  void *arg;

  uint16_t FrameLength;
  uint8_t m_framePayload[RECV_BUF_MAX];
} LinkLayerHandler;

typedef struct {
  void *priv;
  LinkLayerHandler llhandler;
} UartMessageHandler;

void LLInit(LinkLayerHandler *self, UART_HandleTypeDef *uart);
void LLSendData(LinkLayerHandler *self, uint8_t *data, size_t len);
void UART_IDLE_Callback(LinkLayerHandler *self, UART_HandleTypeDef *huart);

void UartPacketHandlerInit(UartMessageHandler *msgHandler, UART_HandleTypeDef *uart, PacketArrived pHandlerPacket);

#endif /* UART_COMM_HELPER_H_ */
