#ifndef UART_H
#define UART_H
#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define UART_BUFFER_SIZE 512
#define USER_UART USART3
#define USER_UART_RX_BUFFER uart3_rx_buf
#define USER_UART_TX_BUFFER uart3_tx_buf
#define GPS_UART UART4
#define GPS_UART_RX_BUFFER uart4_rx_buf
#define GPS_UART_TX_BUFFER uart4_tx_buf

#define USER_UART_Read(d, s) UART_Read(&uart3_rx_buf, &huart3, d, s)
#define USER_UART_ReadLine(d, s) UART_ReadLine(&uart3_rx_buf, &huart3, d, s, false)
#define USER_UART_Write(d, s) UART_Send(&uart3_tx_buf, &huart3, d, s)
#define USER_UART_Print(s) UART_Send(&uart3_tx_buf, &huart3, (uint8_t *)s, uart_strlen((uint8_t *)s))
#define USER_UART_ReadByte() UART_ReadByte(&uart3_rx_buf, &huart3)
#define USER_UART_WriteByte(c) UART_SendByte(&uart3_tx_buf, &huart3, c)
#define USER_UART_GetBufferSize_Tx() UART_GetBufferSize(&uart3_tx_buf)
#define USER_UART_GetBufferSize_Rx() UART_GetBufferSize(&uart3_rx_buf)

#define GPS_UART_Read(d, s) UART_Read(&uart4_rx_buf, &huart4, d, s)
#define GPS_UART_ReadLine(d, s, l) UART_ReadLine(&uart4_rx_buf, &huart4, d, s, l)
#define GPS_UART_Write(d, s) UART_Send(&uart4_tx_buf, &huart4, d, s)
#define GPS_UART_Print(s) UART_Send(&uart4_tx_buf, &huart4, (uint8_t *)s, uart_strlen((uint8_t *)s))
#define GPS_UART_ReadByte() UART_ReadByte(&uart4_rx_buf, &huart4)
#define GPS_UART_WriteByte(c) UART_SendByte(&uart4_tx_buf, &huart4, c)
#define GPS_UART_GetBufferSize_Tx() UART_GetBufferSize(&uart4_tx_buf)
#define GPS_UART_GetBufferSize_Rx() UART_GetBufferSize(&uart4_rx_buf)

#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct
    {
        uint8_t data[UART_BUFFER_SIZE];
        uint16_t top;
        uint16_t bottom;
        uint16_t pending_size;
    } UART_Buffer;

    extern UART_Buffer uart3_rx_buf;
    extern UART_Buffer uart3_tx_buf;
    extern UART_Buffer uart4_rx_buf;
    extern UART_Buffer uart4_tx_buf;
    extern UART_HandleTypeDef huart3;
    extern UART_HandleTypeDef huart4;

    void UART_Init();
    void UART_TickTimer();
    int16_t UART_GetBufferSize(UART_Buffer *buf);
    int16_t UART_Send(UART_Buffer *buf, UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
    int16_t UART_Read(UART_Buffer *buf, UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
    int16_t UART_ReadLine(UART_Buffer *buf, UART_HandleTypeDef *huart, uint8_t *data, uint16_t size, bool remain_delimiter);
    int16_t UART_SendByte(UART_Buffer *buf, UART_HandleTypeDef *huart, uint8_t data);
    int16_t UART_ReadByte(UART_Buffer *buf, UART_HandleTypeDef *huart);
    int16_t UART_WriteBuffer(UART_Buffer *buf, uint8_t data);
    int16_t UART_ReadBuffer(UART_Buffer *buf);
    int16_t uart_strlen(uint8_t *data);

#ifdef __cplusplus
    extern "C"
    {
#endif

#ifdef __cplusplus
    }
#endif

#endif /* UART_H */
