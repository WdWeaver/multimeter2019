#include "can.h"

int can_prepare(CAN_HandleTypeDef* _hcan) {
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(_hcan,&filter) != HAL_OK) {
        return ERROR;
    }
    return SUCCESS;
}

int can_enable_interrupt(CAN_HandleTypeDef* _hcan) {
    uint32_t can_interrupts = 
    (
        CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO0_FULL|CAN_IT_RX_FIFO0_OVERRUN|
        CAN_IT_RX_FIFO1_MSG_PENDING|CAN_IT_RX_FIFO1_FULL|CAN_IT_RX_FIFO1_OVERRUN|
        CAN_IT_ERROR_WARNING|CAN_IT_ERROR_PASSIVE|CAN_IT_BUSOFF|
        CAN_IT_ERROR|CAN_IT_LAST_ERROR_CODE
    );
    if(HAL_CAN_ActivateNotification(_hcan,can_interrupts) != HAL_OK) {
        return ERROR;
    }
    return SUCCESS;
}
