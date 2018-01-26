#include <stm32plus/usart.h>
#include <stm32plus/strutil.h>

#include <string.h>
#include <inttypes.h>

static UART_HandleTypeDef	*g_huart = NULL;
static uint8_t				g_buffer[128];
static volatile uint16_t	g_txStart = 0, g_txCount = 0, g_chunkSize = 0;
static const uint16_t		g_size = sizeof(g_buffer);

////////////////////////////////////////////////////////////////////
static inline void EnableIrq(uint8_t wasEnabled) {
	if(wasEnabled) __enable_irq();
}

////////////////////////////////////////////////////////////////////
uint16_t FillTxBuffer(const uint8_t *buffer, uint16_t count)
{
	uint8_t				irqEnabled = __get_PRIMASK() == 0;
	HAL_StatusTypeDef	st;
	uint16_t   			free, freestart, tocopy, copied = 0;

	UNUSED(st);

	__disable_irq();
	freestart = g_txStart + g_txCount;
	free = g_size - g_txCount;
	EnableIrq(irqEnabled);

	freestart -= (freestart >= g_size) ? g_size : 0;
	if(count > free) count = free;
	tocopy = freestart + count > g_size ? g_size - freestart : count;

	memcpy(g_buffer + freestart, buffer, tocopy);

	__disable_irq();
	if(!g_txCount) {
		EnableIrq(irqEnabled);
		g_chunkSize = tocopy;
		st = HAL_UART_Transmit_IT(g_huart, g_buffer + freestart, tocopy);
	}
	copied = tocopy;
	count -= tocopy;
	__disable_irq();
	g_txCount += tocopy;
	EnableIrq(irqEnabled);

	if(!count)
		return copied;

	buffer += tocopy;
	memcpy(g_buffer, buffer, count);

	uint8_t schedule;
	__disable_irq();
	schedule = !g_txCount;
	g_txCount += count;
	EnableIrq(irqEnabled);

	if(schedule)	//	unlikely corner case
		st = HAL_UART_Transmit_IT(g_huart, g_buffer, count);

	return copied + count;
}

////////////////////////////////////////////////////////////////////
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef	st;
	UNUSED(st);

	if(huart == g_huart)
	{
		if(g_txCount) {
			g_txCount -= g_chunkSize;
			g_txStart += g_chunkSize;
			if(g_txStart >= g_size)
				g_txStart -= g_size;
			g_chunkSize = g_txStart + g_txCount > g_size ? g_size - g_txStart : g_txCount;
			if(g_chunkSize)
				st = HAL_UART_Transmit_IT(huart, g_buffer + g_txStart, g_chunkSize);
		}
	}
}

////////////////////////////////////////////////////////////////////
void UsartInit(UART_HandleTypeDef* huart)
{
	g_huart = huart;
}

////////////////////////////////////////////////////////////////////
uint16_t UsartSend(const void *buffer, uint16_t count, uint8_t block)
{
	uint16_t  sent = 0, copied;

	while(count) {
		while(g_txCount == g_size)
			if(!block)
				return sent;

		copied = FillTxBuffer((uint8_t*)buffer, count);
		buffer = (uint8_t*)buffer + copied;
		count -= copied;
		sent += copied;
	}

	return sent;
}

////////////////////////////////////////////////////////////////////
uint16_t UsartSendStr(const char *buffer, uint8_t block)
{
	uint16_t  count = 0;

	const char *tmp = buffer;
	while(*tmp++) ++count;

	return UsartSend(buffer, count, block);
}

////////////////////////////////////////////////////////////////////
uint16_t UsartPrintInt(int32_t data, uint8_t hex, uint8_t block)
{
	uint8_t	count = 0;
	char	buffer[12], *tmp = buffer;

	if(data < 0) {
		*tmp++ = '-';
		data = -data;
		++count;
	}
	count += hex ? uitohex(tmp, data, hex) : uitodec(tmp, data);

	return UsartSend(buffer, count, block);
}

////////////////////////////////////////////////////////////////////
uint16_t UsartPrintUint(uint32_t data, uint8_t hex, uint8_t block)
{
	uint8_t	count = 0;
	char	buffer[11];

	count = hex ? uitohex(buffer, data, hex) : uitodec(buffer, data);

	return UsartSend(buffer, count, block);
}

////////////////////////////////////////////////////////////////////
uint16_t	UsartPrintByte(uint8_t data, uint8_t hex, uint8_t block)
{
	uint8_t	count = 0;
	char	buffer[4];

	count = hex ? uitohex(buffer, data, hex) : uitodec(buffer, data);

	return UsartSend(buffer, count, block);
}

