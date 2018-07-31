/*
 * CAN module object for ST STM32F334 microcontroller.
 *
 * @file        CO_driver.c
 * @author      Janez Paternoster
 * @author      Ondrej Netik
 * @author      Vijayendra
 * @author      Jan van Lienden
 * @author      Petteri Mustonen
 * @copyright   2013 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * CANopenNode is free and open source software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Following clarification and special exception to the GNU General Public
 * License is included to the distribution terms of CANopenNode:
 *
 * Linking this library statically or dynamically with other modules is
 * making a combined work based on this library. Thus, the terms and
 * conditions of the GNU General Public License cover the whole combination.
 *
 * As a special exception, the copyright holders of this library give
 * you permission to link this library with independent modules to
 * produce an executable, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting
 * executable under terms of your choice, provided that you also meet,
 * for each linked independent module, the terms and conditions of the
 * license of that module. An independent module is a module which is
 * not derived from or based on this library. If you modify this
 * library, you may extend this exception to your version of the
 * library, but you are not obliged to do so. If you do not wish
 * to do so, delete this exception statement from your version.
 */

/* Includes ------------------------------------------------------------------*/
//#include "stm32f30x.h"
#include "CO_driver.h"
#include "CO_Emergency.h"
#include <string.h>

#ifdef CO_USE_TX_QUEUE
#define TX_QUEUE_SIZE 16

static CO_CANtx_t CO_TxQueue[TX_QUEUE_SIZE];
static int CO_TxQueue_Head = 0;
static int CO_TxQueue_Len = 0;

#define CO_TxQueue_Enqueue(b)  CO_TxQueue[((CO_TxQueue_Head + CO_TxQueue_Len++) % TX_QUEUE_SIZE)] = *b
#define CO_TxQueue_Dequeue()   &CO_TxQueue[(CO_TxQueue_Len--, CO_TxQueue_Head++ % TX_QUEUE_SIZE)]
#define CO_TxQueue_IsFull()   (CO_TxQueue_Len >= TX_QUEUE_SIZE)
#define CO_TxQueue_IsEmpty()  (CO_TxQueue_Len == 0)
#define CO_TxQueue_Reset()    (CO_TxQueue_Len = 0, CO_TxQueue_Head = 0)

#endif

/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variable ----------------------------------------------------------*/
/* Private function ----------------------------------------------------------*/
static void CO_CANClkSetting (void);
static uint8_t CO_CANsendToModule(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer);
static HAL_StatusTypeDef  CO_CANsetBitrate(CAN_HandleTypeDef* hcan, uint16_t CANbitRate);

/*******************************************************************************
   Macro and Constants - CAN module registers
 *******************************************************************************/


/******************************************************************************/
void CO_CANsetConfigurationMode(int32_t CANbaseAddress){
    CAN_HandleTypeDef* hcan = (CAN_HandleTypeDef*)(CANbaseAddress);

    HAL_CAN_Stop(hcan);

	hcan->Init.Mode      = CAN_MODE_SILENT;

	HAL_CAN_Init( hcan );
    HAL_CAN_Start(hcan);
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    CAN_HandleTypeDef* hcan = CANmodule->hcan;

    HAL_CAN_Stop(hcan);

    CANmodule->CANnormal = true;
	hcan->Init.Mode      = CAN_MODE_NORMAL;

	HAL_CAN_Init( hcan );
	HAL_CAN_Start(hcan);
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    CAN_HandleTypeDef* hcan = CANmodule->hcan;

    HAL_CAN_Stop(hcan);

    hcan->Init.Mode      = CAN_MODE_SILENT;

    HAL_CAN_Init( hcan );
    HAL_CAN_Start(hcan);
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_setBitrate(
        CO_CANmodule_t         *CANmodule,
        uint16_t                CANbitRate)
{
    CAN_HandleTypeDef* hcan = CANmodule->hcan;
    if (CO_CANsetBitrate(hcan, CANbitRate) == HAL_OK)
        return CO_ERROR_NO;
    else
        return CO_ERROR_ILLEGAL_ARGUMENT;
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        int32_t                 CANbaseAddress,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    CAN_HandleTypeDef* hcan = (CAN_HandleTypeDef            *)CANbaseAddress;
    CAN_FilterTypeDef CAN_FilterInitStruct;

    int i;
    uint8_t result;

    /* verify arguments */
    if(hcan==NULL || rxArray==NULL || txArray==NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CANmodule->hcan = hcan;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false;
    CANmodule->bufferInhibitFlag = 0;
    CANmodule->firstCANtxMessage = 1;
#ifndef CO_USE_TX_QUEUE
    CANmodule->CANtxCount = 0;
#endif
    CANmodule->errOld = 0;
    CANmodule->em = 0;

    //HAL_CAN_ITConfig(CANmodule->CANbaseAddress, (CAN_IT_TME | CAN_IT_FMP0), DISABLE);

    for (i = 0; i < rxSize; i++) {
        CANmodule->rxArray[i].ident = 0;
        CANmodule->rxArray[i].mask = 0xFFFFU;
        CANmodule->rxArray[i].object = NULL;
        CANmodule->rxArray[i].pFunct = 0;
    }

    for (i = 0; i < txSize; i++) {
        CANmodule->txArray[i].bufferFull = 0;
    }

    /* Setting Clock of CAN HW */
    CO_CANClkSetting();

    /* Set bit rate */
    result = CO_CANsetBitrate(hcan, CANbitRate);
    if (result != HAL_OK) {
       return CO_ERROR_ILLEGAL_BAUDRATE;  /* CO- Return Init failed */
    }

    memset(&CAN_FilterInitStruct, 0, sizeof (CAN_FilterInitStruct));
    CAN_FilterInitStruct.SlaveStartFilterBank = 0;
    CAN_FilterInitStruct.FilterIdHigh = 0;
    CAN_FilterInitStruct.FilterIdLow = 0;
    CAN_FilterInitStruct.FilterMaskIdHigh = 0;
    CAN_FilterInitStruct.FilterMaskIdLow = 0;
    CAN_FilterInitStruct.FilterFIFOAssignment = 0; // pouzivame jen FIFO0
    CAN_FilterInitStruct.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStruct.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterInitStruct.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(hcan, &CAN_FilterInitStruct);

    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* interrupts from receiver */
    //NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_INTERRUPTS;
    //NVIC_Init(&NVIC_InitStructure);
    /* interrupts from transmitter */
    //NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_INTERRUPTS;
    //NVIC_Init(&NVIC_InitStructure);

    /* Can_init function of ST Driver puts the controller into the normal mode */

    //CAN_ITConfig(hcan, (CAN_IT_TME | CAN_IT_FMP0), ENABLE);
    HAL_CAN_Start(hcan);

    HAL_CAN_ActivateNotification(hcan,CAN_IT_TX_MAILBOX_EMPTY |
            CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN |
            CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);

    /* Get ready for RX in interrupt mode */
    //HAL_CAN_Receive_IT(hcan, CAN_FIFO0);

    return CO_ERROR_NO;
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        int8_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
    CO_CANrx_t *rxBuffer;
    uint16_t RXF, RXM;

    //safety
    if (!CANmodule || !object || !pFunct || index >= CANmodule->rxSize) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* buffer, which will be configured */
    rxBuffer = CANmodule->rxArray + index;

    /* Configure object variables */
    rxBuffer->object = object;
    rxBuffer->pFunct = pFunct;

    /* CAN identifier and CAN mask, bit aligned with CAN module registers */
    RXF = (ident & 0x07FF) << 2;
    if (rtr) RXF |= 0x02;
    RXM = (mask & 0x07FF) << 2;
    RXM |= 0x02;

    /* configure filter and mask */
    if (RXF != rxBuffer->ident || RXM != rxBuffer->mask) {
        rxBuffer->ident = RXF;
        rxBuffer->mask = RXM;
    }

    return CO_ERROR_NO;
}

/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        int8_t                  rtr,
        uint8_t                 noOfBytes,
        int8_t                  syncFlag)
{
    CO_CANtx_t *buffer;

    /* safety */
    if (!CANmodule || CANmodule->txSize <= index) return 0;

    /* get specific buffer */
    buffer = &CANmodule->txArray[index];

#if 0
	CAN_HandleTypeDef* hcan = CANmodule->hcan;
    /* CAN identifier, bit aligned with CAN module registers */
    TXF = ident << 21;
    TXF &= 0xFFE00000;
    if (rtr) TXF |= 0x02;
    /* write to buffer */
    buffer->ident = TXF;
#endif
    /* write to buffer */
    buffer->ident = ident;
    buffer->DLC = noOfBytes;
    buffer->bufferFull = 0;
    buffer->syncFlag = syncFlag ? 1 : 0;

    return buffer;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;
    uint8_t txRes;

#ifndef CO_USE_TX_QUEUE
    /* Verify overflow */
    if (buffer->bufferFull) {
        if(!CANmodule->firstCANtxMessage) /* don't set error, if bootup message is still on buffers */
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, 0);
        err = CO_ERROR_TX_OVERFLOW;
    }
#endif

    CO_LOCK_CAN_SEND();

    /* First try to transmit the message immediately if one mailbox is free. */
    CANmodule->bufferInhibitFlag = buffer->syncFlag;
    txRes = CO_CANsendToModule(CANmodule, buffer);

    /* No free mailbox -> use interrupt for transmission */
    if (txRes != HAL_OK) {
#ifdef CO_USE_TX_QUEUE
        if (CO_TxQueue_IsFull())
        {
            if(!CANmodule->firstCANtxMessage) /* don't set error, if bootup message is still on buffers */
                CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, 0);
            err = CO_ERROR_TX_OVERFLOW;
        }
        else
        {
            CO_TxQueue_Enqueue(buffer);
        }
#else
        buffer->bufferFull = 1;
        CANmodule->CANtxCount++;
#endif
    }
    CO_UNLOCK_CAN_SEND();

    return err;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    uint32_t tpdoDeleted = 0U;
    uint8_t state = 0;
    CAN_HandleTypeDef* hcan = CANmodule->hcan;

    CO_LOCK_CAN_SEND();

    /* This function will be called every time the SYNC window closes
     * (only if the sync window length in us is != 0)
     * with the purpose of clearing any pending SYNC message which
     * would be on the way.
     * In its original implementation, it would first check the global
     * bufferInhibitFlag (which would be set to true to mark that the
     * can message in the *SINGLE* hw mailbox was a sync one) and delete it from
     * there, if present (return value 1).
     * Then it would check the buffers in the TX Buffers array
     * and cancel the ones with syncFlag set.
     *
     * Notice how clearing the ones in the HW mailbox is still an open
     * issue, since we're using multiple mailboxes and we don't know
     * exactly which one we're actually using.
     */

    /* Abort message from CAN module, if there is synchronous TPDO. */
    state = HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX0);
    if((state == CAN_TXSTATUS_PENDING) && (CANmodule->bufferInhibitFlag)) {
        HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0);
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }

    state = HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX1);
    if((state == CAN_TXSTATUS_PENDING) && (CANmodule->bufferInhibitFlag)) {
        HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX1);
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }

    state = HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX2);
    if((state == CAN_TXSTATUS_PENDING) && (CANmodule->bufferInhibitFlag)) {
        HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX2);
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }

#ifdef CO_USE_TX_QUEUE
    if (!CO_TxQueue_IsEmpty())
    {
	// TODO: Invalidate all SYNC messages in the queue
    }
#else
    /* delete also pending synchronous TPDOs in TX buffers */
    if(CANmodule->CANtxCount != 0U){
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for(i = CANmodule->txSize; i > 0U; i--){
            if(buffer->bufferFull){
                if(buffer->syncFlag){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
#endif
    CO_UNLOCK_CAN_SEND();


    if(tpdoDeleted != 0U){
        CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_TPDO_OUTSIDE_WINDOW, CO_EMC_COMMUNICATION, tpdoDeleted);
    }
}

/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule)
{
    uint32_t err;
    CO_EM_t* em = (CO_EM_t*)CANmodule->em;
    CAN_HandleTypeDef* hcan = CANmodule->hcan;

    err = hcan->Instance->ESR;

    if(CANmodule->errOld != err) {
        CANmodule->errOld = err;

        /* CAN RX bus overflow */
        if(hcan->Instance->RF0R & 0x10) {
            CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
            hcan->Instance->RF0R &=~0x10; //clear bits
        }

        /* CAN TX bus off */
        if(err & 0x04) {
            CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
        }
        else {
            CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);
        }

        /* CAN TX or RX bus passive */
        if(err & 0x02) {
            if(!CANmodule->firstCANtxMessage) CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
        }
        else {
        // int16_t wasCleared;
        /* wasCleared = */CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
        /* if(wasCleared == 1) */CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
        }


        /* CAN TX or RX bus warning */
        if(err & 0x01) {
            CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
        }
        else {
            CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
        }
    }
}

/******************************************************************************/
/* Interrupt from receiver */
void CO_CANinterrupt_Rx(CO_CANmodule_t *CANmodule)
{
	CO_CANrxMsg_t CAN1_RxMsg;
	CAN_HandleTypeDef* hcan = CANmodule->hcan;
    static CAN_RxHeaderTypeDef pRxMsg = {};
    uint8_t payload[8];

    uint16_t index;
    uint8_t msgMatched = 0;
    CO_CANrx_t *msgBuff = CANmodule->rxArray;

    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);

	//CAN_Receive(CANmodule->CANbaseAddress, CAN_FilterFIFO0, &CAN1_RxMsg);
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxMsg, &payload[0]);

    for (index = 0; index < CANmodule->rxSize; index++) {
        uint16_t msg = (pRxMsg.StdId << 2) | (pRxMsg.RTR ? 2 : 0);

        if (((msg ^ msgBuff->ident) & msgBuff->mask) == 0) {
            msgMatched = 1;
            break;
        }
        msgBuff++;
    }

    /* Call specific function, which will process the message */
    if (msgMatched && msgBuff->pFunct) {
        /* Copy data from hcan buffer to local buffer */
        CAN1_RxMsg.ident = pRxMsg.StdId;
        CAN1_RxMsg.ExtId = pRxMsg.ExtId;
        CAN1_RxMsg.RTR = pRxMsg.RTR;
        CAN1_RxMsg.DLC = pRxMsg.DLC;
        memcpy(CAN1_RxMsg.data, payload, CAN1_RxMsg.DLC);
        CAN1_RxMsg.FMI = pRxMsg.FilterMatchIndex;
        msgBuff->pFunct(msgBuff->object, &CAN1_RxMsg);
    }

    /* Trigger next acquisition */
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/******************************************************************************/
/* Interrupt from trasmitter */
void CO_CANinterrupt_Tx(CO_CANmodule_t *CANmodule)
{
	CAN_HandleTypeDef* hcan = CANmodule->hcan;

    /* First CAN message (bootup) was sent successfully */
    CANmodule->firstCANtxMessage = 0;

    /* clear flag from previous message */
    CANmodule->bufferInhibitFlag = 0;

    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY |
            CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);

#ifdef CO_USE_TX_QUEUE
    if (!CO_TxQueue_IsEmpty())
    {
	/* Pop the first message in the queue and send it for transmission. */
	/* TODO: only check the ones with bufferFull and repeat until we find one. */
        CO_CANtx_t *buffer = CO_TxQueue_Dequeue();
        /* Copy message to CAN buffer */
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
        CO_CANsendToModule(CANmodule, buffer);
    }
#else
    /* Are there any new messages waiting to be send */
    if (CANmodule->CANtxCount > 0) {
        uint16_t i;             /* index of transmitting message */

        /* first buffer */
        CO_CANtx_t *buffer = CANmodule->txArray;
        /* search through whole array of pointers to transmit message buffers. */
        for(i = CANmodule->txSize; i > 0; i--) {
            /* if message buffer is full, send it. */
            if(buffer->bufferFull) {
                buffer->bufferFull = 0;
                CANmodule->CANtxCount--;

                /* Copy message to CAN buffer */
                CANmodule->bufferInhibitFlag = buffer->syncFlag;
                CO_CANsendToModule(CANmodule, buffer);
                break;                      /* exit for loop */
            }
            buffer++;
        }/* end of for loop */

        /* Clear counter if no more messages */
        if(i == 0) CANmodule->CANtxCount = 0;
    }

#endif
	/* Clear all Request Completed mailboxes flags in order to clear the
	 * Transmit interrupt ( CAN_IT_TME ) */
    hcan->Instance->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2;

    HAL_CAN_ActivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY);
}

void CO_CANinterrupt_Err(CO_CANmodule_t *CANmodule)
{
    CAN_HandleTypeDef* hcan = CANmodule->hcan;

    hcan->ErrorCode = HAL_CAN_ERROR_NONE;
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);
    HAL_CAN_Stop(hcan);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);

}
/** **************************************************************************
 ** @brief Function-line macros to convert bit quanta to bit settings
 ** *************************************************************************/

#define CAN_BS1_TQ(n) \
        (n ==  1) ? CAN_BS1_1TQ : \
        (n ==  2) ? CAN_BS1_2TQ : \
        (n ==  3) ? CAN_BS1_3TQ : \
        (n ==  4) ? CAN_BS1_4TQ : \
        (n ==  5) ? CAN_BS1_5TQ : \
        (n ==  6) ? CAN_BS1_6TQ : \
        (n ==  7) ? CAN_BS1_7TQ : \
        (n ==  8) ? CAN_BS1_8TQ : \
        (n ==  9) ? CAN_BS1_9TQ : \
        (n == 10) ? CAN_BS1_10TQ : \
        (n == 11) ? CAN_BS1_11TQ : \
        (n == 12) ? CAN_BS1_12TQ : \
        (n == 13) ? CAN_BS1_13TQ : \
        (n == 14) ? CAN_BS1_14TQ : \
        (n == 15) ? CAN_BS1_15TQ : \
        (n == 16) ? CAN_BS1_16TQ : 0


#define CAN_BS2_TQ(n) \
        (n ==  2) ? CAN_BS2_2TQ : \
        (n ==  3) ? CAN_BS2_3TQ : \
        (n ==  4) ? CAN_BS2_4TQ : \
        (n ==  5) ? CAN_BS2_5TQ : \
        (n ==  6) ? CAN_BS2_6TQ : \
        (n ==  7) ? CAN_BS2_7TQ : \
        (n ==  8) ? CAN_BS2_8TQ : 0

/** **************************************************************************
 ** @brief  Find suitable prescaler and bitquanta settings
 **         for the desidered bitrate, given the current clock frequency
 ** @param clkFreq  clock frequency in Hz
 ** @param bitrate  desired bitrate
 ** @param out prescaler  prescaler
 ** @param out bs1  BS1 time quanta
 ** @param out bs2  BS2 time quanta
 ** @return     1 if the function terminates correctly, 0 in case of errors
 ** *************************************************************************/
static int CAN_FindBitQuanta(uint32_t clkFreq, uint32_t bitrate,
        uint32_t *prescaler, uint8_t *bs1, uint8_t *bs2)
{
    int bq_settings[][2] = {
        { 14, 5 }, /* total 20, 15/20 sample point 75% */
        {  8, 3 }, /* total 12, 9/12  sample point 75% */
        { 11, 4 }, /* total 16, 12/16 sample point 75% */
        { 10, 4 }, /* total 15, 11/15 sample point 73.3% */
    };
    int ns = sizeof(bq_settings) / sizeof(bq_settings[0]);
    int i;

    for (i = 0; i < ns; i++)
    {
        uint8_t n1 = bq_settings[i][0];
        uint8_t n2 = bq_settings[i][1];
        uint32_t nt = n1 + n2 + 1;
        if (clkFreq % (nt*bitrate) == 0)
        {
            if (bs1) *bs1 = n1;
            if (bs2) *bs2 = n2;
            if (prescaler) *prescaler = clkFreq / (nt * bitrate);
            return 1;
        }
    }
    return 0;
}
/******************************************************************************/
static HAL_StatusTypeDef  CO_CANsetBitrate(CAN_HandleTypeDef* hcan, uint16_t CANbitRate)
{
    uint32_t prescaler;
    uint8_t bs1, bs2;
    if (CAN_FindBitQuanta(HAL_RCC_GetPCLK1Freq(), CANbitRate*1000, &prescaler, &bs1, &bs2) == 0)
    {
        return HAL_ERROR;
    }

    HAL_CAN_Stop(hcan);

    /* Configure CAN address relying on HAL */
    hcan->Init.Prescaler = prescaler;

    hcan->Init.Mode = CAN_MODE_NORMAL;
    hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan->Init.TimeSeg1 = CAN_BS1_TQ(bs1);
    hcan->Init.TimeSeg2 = CAN_BS2_TQ(bs2);

    /*********************************************************************************************/
    /*Please notice that starting from STM32HAL_L4 v1.11 this flag has opposite logic than before*/
    hcan->Init.AutoRetransmission = ENABLE;         // No Automatic retransmision
    /*********************************************************************************************/

    hcan->Init.TransmitFifoPriority = ENABLE;
	/* Enable automatic Bus-Off management (ABOM) so to automatically
	 * rejoin the bus once the error conditions have been cleared */
    hcan->Init.AutoBusOff = ENABLE;

    HAL_CAN_Init( hcan );
    return HAL_CAN_Start( hcan );

}
/******************************************************************************/
static uint8_t CO_CANsendToModule(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
	CAN_HandleTypeDef* hcan = CANmodule->hcan;
	static CAN_TxHeaderTypeDef pTxMsg = {};
	uint8_t payload[8];
	static uint32_t pTxMailbox;
	pTxMsg.IDE = CAN_ID_STD;
	pTxMsg.StdId = buffer->ident;
	pTxMsg.DLC = buffer->DLC;
	memcpy(payload, buffer->data, pTxMsg.DLC);

	/* Relies on HAL to transmit data.
	 * NOTE: This assumes HAL would return HAL_ERROR in case of no mailbox available,
	 *       which may not necessarily be the case!
	 */
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
	    HAL_CAN_ActivateNotification(hcan, CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);

	    HAL_CAN_AddTxMessage(hcan, &pTxMsg, &payload[0], &pTxMailbox);
	    if(hcan->ErrorCode != HAL_CAN_ERROR_NONE)
	    {
	        return HAL_ERROR;
	    }
	    else
	    {
	        return HAL_OK;
	    }
	}
	else
	{
	    return HAL_ERROR;
	}


}

/******************************************************************************/
static void CO_CANClkSetting (void)
{
    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

