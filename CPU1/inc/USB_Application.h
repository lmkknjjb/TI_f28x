/*
 * USB_Application.h
 *
 *  Created on: 2020. 10. 27.
 *      Author: Administrator
 */

#ifndef INC_USB_APPLICATION_H_
#define INC_USB_APPLICATION_H_

uint32_t RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
void ModeCallback(uint32_t ui32Index, tUSBMode eMode);



#endif /* INC_USB_APPLICATION_H_ */
