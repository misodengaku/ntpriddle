#include "uart.h"
#include "cmsis_os.h"

UART_Buffer USER_UART_RX_BUFFER;
UART_Buffer USER_UART_TX_BUFFER;
UART_Buffer GPS_UART_RX_BUFFER;
UART_Buffer GPS_UART_TX_BUFFER;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_Buffer *tx_buffer;
    uint16_t len = 0;

    if (huart->Instance == USER_UART)
    {
        tx_buffer = &USER_UART_TX_BUFFER;
    }
    else if (huart->Instance == GPS_UART)
    {
        tx_buffer = &GPS_UART_TX_BUFFER;
    }
    else
    {
        return;
    }

    // Transmitted Size
    len = tx_buffer->pending_size;

    // Top
    tx_buffer->top = (tx_buffer->top + len) % UART_BUFFER_SIZE;

    // Transmit size
    len = UART_GetBufferSize(tx_buffer);
    if (len == 0)
    {
        // complete
        return;
    }
    if (tx_buffer->top + len > UART_BUFFER_SIZE)
    {
        len = UART_BUFFER_SIZE - tx_buffer->top;
    }

    if (HAL_UART_Transmit_DMA(huart, &(tx_buffer->data[tx_buffer->top]), len) == HAL_OK)
    {
        tx_buffer->pending_size = len;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_Buffer *rx_buffer;

    if (huart->Instance == USER_UART)
    {
        rx_buffer = &USER_UART_RX_BUFFER;
    }
    else if (huart->Instance == GPS_UART)
    {
        rx_buffer = &GPS_UART_RX_BUFFER;
    }
    else
    {
        return;
    }

    // Bottom
    rx_buffer->bottom = (rx_buffer->bottom + 1) % UART_BUFFER_SIZE;

    if (UART_GetBufferSize(rx_buffer) < (UART_BUFFER_SIZE - 1))
    {
        HAL_UART_Receive_IT(huart, &rx_buffer->data[rx_buffer->bottom], 1);
    }
}

void UART_TickTimer()
{
    UART_Buffer *rx_buffer;

    rx_buffer = &USER_UART_RX_BUFFER;
    if (UART_GetBufferSize(rx_buffer) < (UART_BUFFER_SIZE - 1))
    {
        HAL_UART_Receive_IT(&huart3, &rx_buffer->data[rx_buffer->bottom], 1);
    }

    rx_buffer = &GPS_UART_RX_BUFFER;
    if (UART_GetBufferSize(rx_buffer) < (UART_BUFFER_SIZE - 1))
    {
        HAL_UART_Receive_IT(&huart4, &rx_buffer->data[rx_buffer->bottom], 1);
    }
}

int16_t UART_GetBufferSize(UART_Buffer *buf)
{
    if (buf->bottom == buf->top)
    {
        return 0;
    }
    else if ((buf->bottom) < (buf->top))
    {
        return (UART_BUFFER_SIZE - buf->top) + buf->bottom;
    }
    return buf->bottom - buf->top;
}

int16_t UART_WriteBuffer(UART_Buffer *buf, uint8_t data)
{
    while (UART_GetBufferSize(buf) >= (UART_BUFFER_SIZE - 2))
    {
        osDelay(1);
    }

    __disable_irq();
    buf->data[buf->bottom] = data;
    buf->bottom = (buf->bottom + 1) % UART_BUFFER_SIZE;
    __enable_irq();
    return 1;
}

int16_t UART_ReadBuffer(UART_Buffer *buf)
{
    int16_t ret;

    if (UART_GetBufferSize(buf) == 0)
    {
        return -1;
    }

    __disable_irq();
    ret = buf->data[buf->top];
    buf->top = (buf->top + 1) % UART_BUFFER_SIZE;
    __enable_irq();
    return ret;
}

int16_t UART_SendByte(UART_Buffer *buf, UART_HandleTypeDef *huart, uint8_t data)
{
    int16_t ret, len;
    ret = UART_WriteBuffer(buf, data);

    if (huart->gState == HAL_UART_STATE_READY)
    {
        __disable_irq();
        // Transmit size
        len = UART_GetBufferSize(buf);
        if (buf->top + len > UART_BUFFER_SIZE)
        {
            len = UART_BUFFER_SIZE - buf->top;
        }
        // Transmit
        if (HAL_UART_Transmit_DMA(huart, &(buf->data[buf->top]), len) == HAL_OK)
        {
            buf->pending_size = len;
        }
        __enable_irq();
    }
    return ret;
}

int16_t UART_ReadByte(UART_Buffer *buf, UART_HandleTypeDef *huart)
{

    if (huart->RxState != HAL_UART_STATE_BUSY_RX)
    {
        HAL_UART_Receive_IT(huart, &buf->data[buf->bottom], 1);
    }
    return UART_ReadBuffer(buf);
}

int16_t UART_Read(UART_Buffer *buf, UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    int16_t ret;
    int16_t c;
    for (ret = 0; ret < size; ret++)
    {
        c = UART_ReadByte(buf, huart);
        if (c < 0)
        {
            return ret;
        }
        data[ret] = c;
    }
    return ret;
}

int16_t UART_ReadLine(UART_Buffer *buf, UART_HandleTypeDef *huart, uint8_t *data, uint16_t size, bool remain_delimiter)
{
    int16_t ret;
    int16_t c;
    data[0] = 0;
    for (ret = 0; ret < size; ret++)
    {
        // blockma
        while ((c = UART_ReadByte(buf, huart)) < 0)
            osDelay(1);
        data[ret] = c;
        if (c == '\n' || c == '\r')
        {
            if (ret == 0)
            {
                ret--;
                continue;
            }

            // 末端の改行文字を残す
            if (remain_delimiter)
            {
                data[ret + 1] = 0;
                return ret + 2;
            }

            data[ret] = 0;
            return ret + 1;
        }
    }
    data[ret - 1] = 0;
    return ret;
}

int16_t UART_Send(UART_Buffer *buf, UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    int16_t ret, len;
    for (ret = 0; ret < size; ret++)
    {
        UART_WriteBuffer(buf, data[ret]);
        if (huart->gState == HAL_UART_STATE_READY)
        {
            __disable_irq();
            // Transmit size
            len = UART_GetBufferSize(buf);
            if (buf->top + len > UART_BUFFER_SIZE)
            {
                len = UART_BUFFER_SIZE - buf->top;
            }
            // Transmit
            if (HAL_UART_Transmit_DMA(huart, &(buf->data[buf->top]), len) == HAL_OK)
            {
                buf->pending_size = len;
            }
            __enable_irq();
        }
    }
    return ret;
}

int16_t uart_strlen(uint8_t *data)
{
    uint16_t ret = 0;
    while (data[ret] != 0)
    {
        ret++;
    }
    return ret;
}

void UART_Init()
{
    USER_UART_RX_BUFFER.bottom = 0;
    USER_UART_RX_BUFFER.top = 0;
    USER_UART_TX_BUFFER.bottom = 0;
    USER_UART_TX_BUFFER.top = 0;
    GPS_UART_RX_BUFFER.bottom = 0;
    GPS_UART_RX_BUFFER.top = 0;
    GPS_UART_TX_BUFFER.bottom = 0;
    GPS_UART_TX_BUFFER.top = 0;

    HAL_UART_Receive_IT(&huart3, &USER_UART_RX_BUFFER.data[0], 1);
    HAL_UART_Receive_IT(&huart4, &GPS_UART_RX_BUFFER.data[0], 1);
}