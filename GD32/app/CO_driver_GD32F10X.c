/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "301/CO_driver.h"
#include "cmsis_os.h"
#include "gd32f10x.h"


static CO_CANmodule_t *CANModule_local = NULL;

typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
} CO_CANrxMsg_t;

osMailQDef(q_rx_msg_id, 10, CO_CANrxMsg_t);
static osMailQId q_rx_msg_id;
osMailQDef(q_tx_msg_id, 10, CO_CANrxMsg_t);
static osMailQId q_tx_msg_id;


__NO_RETURN static void CAN_Transmit_Thread(void const *arg)
{
    CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)arg;
    osEvent evt;
    CO_CANrxMsg_t *RxMSG;
    can_trasnmit_message_struct transmit_message;
    transmit_message.tx_ft = CAN_FT_DATA;
    transmit_message.tx_ff = CAN_FF_STANDARD;
    while (1) {
        evt = osMailGet(q_tx_msg_id, osWaitForever); // wait until a message
        if (evt.status == osEventMail) // Check for a valid message
        {
            RxMSG = (CO_CANrxMsg_t *)evt.value.p;

            can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &transmit_message);
            transmit_message.tx_sfid = RxMSG->ident;
            transmit_message.tx_ft = CAN_FT_DATA;
            transmit_message.tx_ff = CAN_FF_STANDARD;
            transmit_message.tx_dlen = RxMSG->DLC;
            for (uint8_t i = 0; i < 8; i++) {
                transmit_message.tx_data[i] = RxMSG->data[i];
            }
            can_message_transmit(*(uint32_t *)CANmodule->CANptr,
                                 &transmit_message);

            // ptrCAN->MessageSend(
            //     tx_obj_idx, &tx_msg_info, RxMSG->data, RxMSG->DLC);

            osMailFree(q_tx_msg_id, RxMSG);
        }
        osDelay(1U);
    }
}

__NO_RETURN static void CAN_Receive_Process_Thread(void const *arg)
{
    CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)arg;
    osEvent evt;
    CO_CANrxMsg_t *RxMSG;
    for (;;) {
        evt = osMailGet(q_rx_msg_id, osWaitForever); // wait until a message
        if (evt.status == osEventMail) // Check for a valid message
        {
            RxMSG = (CO_CANrxMsg_t *)evt.value.p;
            uint8_t msgMatched = 0;
            CO_CANrx_t *msgBuff = CANmodule->rxArray;
            for (uint8_t index = 0; index < CANmodule->rxSize; index++) {
                if ((((RxMSG->ident << 2) ^ msgBuff->ident) & msgBuff->mask) ==
                    0) {
                    msgMatched = 1;
                    break;
                }
                msgBuff++;
            }
            // Call specific function, which will process the message
            if (msgMatched && msgBuff->CANrx_callback) {
                msgBuff->CANrx_callback(msgBuff->object, RxMSG);
            }

            osMailFree(q_rx_msg_id, RxMSG);
        }
    }
}

osThreadDef(CAN_Transmit_Thread, osPriorityHigh, 1, 0);
osThreadDef(CAN_Receive_Process_Thread, osPriorityHigh, 1, 0);
/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr)
{
    /* Put CAN module in configuration mode */
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    /* Put CAN module in normal mode */
    can_working_mode_set(*(uint32_t *)CANmodule->CANptr, CAN_MODE_NORMAL);
    CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule,
                                   void *CANptr,
                                   CO_CANrx_t rxArray[],
                                   uint16_t rxSize,
                                   CO_CANtx_t txArray[],
                                   uint16_t txSize,
                                   uint16_t CANbitRate)
{
    uint16_t i;

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters =
        (rxSize <= 32U) ? true : false; /* microcontroller dependent */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for (i = 0U; i < rxSize; i++) {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (i = 0U; i < txSize; i++) {
        txArray[i].bufferFull = false;
    }
    CANModule_local = CANmodule;

    /* Configure CAN module registers */

    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOD);

    // configure CAN0 GPIO, CAN0_TX(PD1) and CAN0_RX(PD0)
    gpio_init(GPIOD, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    gpio_init(GPIOD, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_0);

    gpio_pin_remap_config(GPIO_CAN_FULL_REMAP, ENABLE);

    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);

    can_deinit(*(uint32_t *)CANptr);

    /* Configure CAN timing */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.no_auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_5TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_3TQ;
    //  baudrate 1Mbps
    can_parameter.prescaler = 6;
    can_init(*(uint32_t *)CANptr, &can_parameter);


    /* Configure CAN module hardware filters */
    can_filter.filter_number = 0;
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO1;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
    // if (CANmodule->useCANrxFilters) {
    //     /* CAN module filters are used, they will be configured with */
    //     /* CO_CANrxBufferInit() functions, called by separate CANopen */
    //     /* init functions. */
    //     /* Configure all masks so, that received message must match filter */
    // } else {
    //     /* CAN module filters are not used, all messages with standard 11-bit
    //     */
    //     /* identifier will be received */
    //     /* Configure mask 0 so, that all messages with standard identifier
    //     are
    //      * accepted */
    // }


    /* configure CAN interrupt registers */
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 0, 0);
    can_interrupt_enable(*(uint32_t *)CANptr, CAN_INT_RFNE0);

    q_rx_msg_id = osMailCreate(osMailQ(q_rx_msg_id), NULL);
    q_tx_msg_id = osMailCreate(osMailQ(q_tx_msg_id), NULL);

    osThreadCreate(osThread(CAN_Receive_Process_Thread), CANmodule);
    osThreadCreate(osThread(CAN_Transmit_Thread), CANmodule);

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != NULL) {
        /* turn off the module */
        can_deinit(*(uint32_t *)CANmodule->CANptr);
    }
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule,
                                    uint16_t index,
                                    uint16_t ident,
                                    uint16_t mask,
                                    bool_t rtr,
                                    void *object,
                                    void (*CANrx_callback)(void *object,
                                                           void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) &&
        (index < CANmodule->rxSize)) {
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different
         * on different microcontrollers. */
        buffer->ident = ident & 0x07FFU;
        if (rtr) {
            buffer->ident |= 0x0800U;
        }
        buffer->mask = (mask & 0x07FFU) | 0x0800U;

        can_filter_parameter_struct can_filter;
        can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters) {
            can_filter.filter_number = buffer->ident;
            can_filter.filter_mode = CAN_FILTERMODE_MASK;
            can_filter.filter_bits = CAN_FILTERBITS_32BIT;
            can_filter.filter_list_high = 0x0000;
            can_filter.filter_list_low = 0x0000;
            can_filter.filter_mask_high = buffer->mask >> 8;
            can_filter.filter_mask_low = buffer->mask & 0xff;
            can_filter.filter_fifo_number = CAN_FIFO1;
            can_filter.filter_enable = ENABLE;
            can_filter_init(&can_filter);
        }
    } else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(CO_CANmodule_t *CANmodule,
                               uint16_t index,
                               uint16_t ident,
                               bool_t rtr,
                               uint8_t noOfBytes,
                               bool_t syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit
         * buffer. Microcontroller specific. */
        buffer->ident = ((uint32_t)ident & 0x07FFU) |
                        ((uint32_t)(((uint32_t)noOfBytes & 0xFU) << 12U)) |
                        ((uint32_t)(rtr ? 0x8000U : 0U));

        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}


/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;
    CO_CANrxMsg_t *txMsg;

    /* Verify overflow */
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    /* if CAN TX buffer is free, copy message to it */
    if (1 && CANmodule->CANtxCount == 0) {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
        /* copy message and txRequest */

        txMsg = (CO_CANrxMsg_t *)osMailAlloc(q_tx_msg_id, osWaitForever);
        txMsg->ident = buffer->ident;
        txMsg->DLC = buffer->DLC;
        for (uint8_t i = 0; i < 8; i++) {
            txMsg->data[i] = buffer->data[i];
        }

        osMailPut(q_tx_msg_id, txMsg);
    }
    /* if no buffer is free, message will be sent by interrupt */
    else {
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag) {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount != 0U) {
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for (i = CANmodule->txSize; i > 0U; i--) {
            if (buffer->bufferFull) {
                if (buffer->syncFlag) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);


    if (tpdoDeleted != 0U) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}


/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
 * different way to determine errors. */
static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    uint32_t err;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors >= 256U) {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        } else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING |
                                CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING |
                                CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (overflow != 0) {
            /* CAN RX bus overflow */
            status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
    }
}


/******************************************************************************/


void CO_CANinterrupt(CO_CANmodule_t *CANmodule)
{

    /* receive interrupt */
    if (1) {
        CO_CANrxMsg_t *rcvMsg; /* pointer to received message in CAN module */
        uint16_t index;        /* index of received message */
        uint32_t rcvMsgIdent;  /* identifier of the received message */
        CO_CANrx_t *buffer =
            NULL; /* receive message buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false;

        rcvMsg = 0; /* get message from module here */
        rcvMsgIdent = rcvMsg->ident;
        if (CANmodule->useCANrxFilters) {
            /* CAN module filters are used. Message with known 11-bit identifier
             * has */
            /* been received */
            index = 0; /* get index of the received message here. Or something
                          similar */
            if (index < CANmodule->rxSize) {
                buffer = &CANmodule->rxArray[index];
                /* verify also RTR */
                if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
                    msgMatched = true;
                }
            }
        } else {
            /* CAN module filters are not used, message with any standard 11-bit
             * identifier */
            /* has been received. Search rxArray form CANmodule for the same
             * CAN-ID. */
            buffer = &CANmodule->rxArray[0];
            for (index = CANmodule->rxSize; index > 0U; index--) {
                if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
                    msgMatched = true;
                    break;
                }
                buffer++;
            }
        }

        /* Call specific function, which will process the message */
        if (msgMatched && (buffer != NULL) &&
            (buffer->CANrx_callback != NULL)) {
            buffer->CANrx_callback(buffer->object, (void *)rcvMsg);
        }

        /* Clear interrupt flag */
    }


    /* transmit interrupt */
    else if (0) {
        /* Clear interrupt flag */

        /* First CAN message (bootup) was sent successfully */
        CANmodule->firstCANtxMessage = false;
        /* clear flag from previous message */
        CANmodule->bufferInhibitFlag = false;
        /* Are there any new messages waiting to be send */
        if (CANmodule->CANtxCount > 0U) {
            uint16_t i; /* index of transmitting message */

            /* first buffer */
            CO_CANtx_t *buffer = &CANmodule->txArray[0];
            /* search through whole array of pointers to transmit message
             * buffers. */
            for (i = CANmodule->txSize; i > 0U; i--) {
                /* if message buffer is full, send it. */
                if (buffer->bufferFull) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;

                    /* Copy message to CAN buffer */
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                    /* canSend... */
                    break; /* exit for loop */
                }
                buffer++;
            } /* end of for loop */

            /* Clear counter if no more messages */
            if (i == 0U) {
                CANmodule->CANtxCount = 0U;
            }
        }
    } else {
        /* some other interrupt reason */
    }
}

void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    can_receive_message_struct receive_message;
    /* check the receive message */
    can_message_receive(
        *(uint32_t *)CANModule_local->CANptr, CAN_FIFO1, &receive_message);
    //  线程中通知
    CO_CANrxMsg_t *RxMSG;
    if (CAN_FF_STANDARD == receive_message.rx_ff) {
        RxMSG = (CO_CANrxMsg_t *)osMailAlloc(q_rx_msg_id, osWaitForever);
        RxMSG->ident = receive_message.rx_sfid;
        RxMSG->DLC = receive_message.rx_dlen;
        for (uint8_t i = 0U; i < 8U; i++) {
            RxMSG->data[i] = receive_message.rx_data[i];
        }
        osMailPut(q_rx_msg_id, RxMSG);
    }
}
