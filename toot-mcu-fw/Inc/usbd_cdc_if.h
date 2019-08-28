#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
#define APP_RX_DATA_SIZE  1000
#define APP_TX_DATA_SIZE  1000

/** Received data over USB are stored in this buffer      */
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
/** Data to send over USB CDC are stored in this buffer   */
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
