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

#include "Driver_CAN.h"
#include "cmsis_os.h"

typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
} CO_CANrxMsg_t;


// CAN Interface configuration -------------------------------------------------

#define CAN_CONTROLLER 1           // CAN Controller number
#define CAN_LOOPBACK 0             // 0 = no loopback, 1 = external loopback
#define CAN_BITRATE_NOMINAL 500000 // Nominal bitrate (125 kbit/s)

//------------------------------------------------------------------------------

#define _CAN_Driver_(n) Driver_CAN##n
#define CAN_Driver_(n) _CAN_Driver_(n)
extern ARM_DRIVER_CAN CAN_Driver_(CAN_CONTROLLER);
#define ptrCAN (&CAN_Driver_(CAN_CONTROLLER))

static uint32_t rx_obj_idx;
static uint32_t tx_obj_idx;


osMailQDef(q_rx_msg_id, 16, CO_CANrxMsg_t);
static osMailQId q_rx_msg_id;
osMailQDef(q_tx_msg_id, 16, CO_CANrxMsg_t);
static osMailQId q_tx_msg_id;

// #define CAN_TRANSMIT_THREAD_STK_SZ (1024)
// #define CAN_RECEIVE_THREAD_STK_SZ (1024)

// static uint64_t can_transmit_thread_stk[(CAN_TRANSMIT_THREAD_STK_SZ + 7) /
// 8]; static const osThreadAttr_t can_transmit_thread_attr = {
//     .stack_mem = can_transmit_thread_stk,
//     .stack_size = sizeof(can_transmit_thread_stk),
//     .priority = osPriorityHigh1};

// static uint64_t
//     can_receive_process_thread_stk[(CAN_RECEIVE_THREAD_STK_SZ + 7) / 8];
// static const osThreadAttr_t can_receive_process_thread_attr = {
//     .stack_mem = can_receive_process_thread_stk,
//     .stack_size = sizeof(can_receive_process_thread_stk),
//     .priority = osPriorityHigh1};

//------------------------------------------------------------------------------
//  CAN Interface Signal Unit Event Callback
//------------------------------------------------------------------------------
void CAN_SignalUnitEvent(uint32_t event)
{

    switch (event) {
    case ARM_CAN_EVENT_UNIT_ACTIVE:
        break;
    case ARM_CAN_EVENT_UNIT_WARNING:
        break;
    case ARM_CAN_EVENT_UNIT_PASSIVE:
        break;
    case ARM_CAN_EVENT_UNIT_BUS_OFF:
        break;
    }
}
__NO_RETURN static void ErrorHandler(void)
{
    for (;;)
        ;
}

__NO_RETURN static void CAN_Transmit_Thread(void *arg)
{
    CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)arg;
    osEvent evt;
    ARM_CAN_MSG_INFO tx_msg_info;
    CO_CANrxMsg_t *RxMSG;
    while (1) {
        evt = osMailGet(q_tx_msg_id, osWaitForever); // wait until a message
        if (evt.status == osEventMail) // Check for a valid message
        {
            RxMSG = (CO_CANrxMsg_t *)evt.value.p;

            tx_msg_info.rtr = 0;
            tx_msg_info.dlc = RxMSG->DLC;
            tx_msg_info.id = ARM_CAN_STANDARD_ID(RxMSG->ident);

            ptrCAN->MessageSend(
                tx_obj_idx, &tx_msg_info, RxMSG->data, RxMSG->DLC);

            osMailFree(q_tx_msg_id, RxMSG);
        }
        osDelay(1U);
    }
}

__NO_RETURN static void CAN_Receive_Process_Thread(void *arg)
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

//------------------------------------------------------------------------------
//  CAN Interface Signal Object Event Callback
//------------------------------------------------------------------------------
static void CAN_SignalObjectEvent(uint32_t obj_idx, uint32_t event)
{

    ARM_CAN_MSG_INFO rx_msg_info;
    CO_CANrxMsg_t *RxMSG;
    if (event == ARM_CAN_EVENT_RECEIVE) { // If receive event
        if (obj_idx == rx_obj_idx) {      // If receive object event

            RxMSG = (CO_CANrxMsg_t *)osMailAlloc(q_rx_msg_id, osWaitForever);

            ptrCAN->MessageRead(rx_obj_idx, &rx_msg_info, RxMSG->data, 8);
            RxMSG->ident = rx_msg_info.id;
            RxMSG->DLC = rx_msg_info.dlc;

            osMailPut(q_rx_msg_id, RxMSG);
            //        osDelay(1U);
        }
    }
    if (event == ARM_CAN_EVENT_SEND_COMPLETE) { // If send completed event
        if (obj_idx == tx_obj_idx) {            // If transmit object event
        }
    }
}
/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr)
{
    /* Put CAN module in configuration mode */
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    int32_t status =
        ptrCAN->SetMode(ARM_CAN_MODE_NORMAL); // Activate normal operation mode
    if (status != ARM_DRIVER_OK) {
        return;
    }
    /* Put CAN module in normal mode */

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
    ARM_CAN_CAPABILITIES can_cap;
    ARM_CAN_OBJ_CAPABILITIES can_obj_cap;
    int32_t status;
    uint32_t i, num_objects, clock;

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
    CANmodule->useCANrxFilters = true; /* microcontroller dependent */
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


    /* Configure CAN module registers */

    can_cap = ptrCAN->GetCapabilities(); // Get CAN driver capabilities
    num_objects = can_cap.num_objects;   // Number of receive/transmit objects

    status = ptrCAN->Initialize(CAN_SignalUnitEvent,
                                CAN_SignalObjectEvent); // Initialize CAN driver
    if (status != ARM_DRIVER_OK) {
        return false;
    }

    status = ptrCAN->PowerControl(ARM_POWER_FULL); // Power-up CAN controller
    if (status != ARM_DRIVER_OK) {
        return false;
    }

    status = ptrCAN->SetMode(
        ARM_CAN_MODE_INITIALIZATION); // Activate initialization mode
    if (status != ARM_DRIVER_OK) {
        return false;
    }

    /* Configure CAN timing */
    clock = ptrCAN->GetClock(); // Get CAN bas clock
    if ((clock % (8U * CAN_BITRATE_NOMINAL)) ==
        0U) { // If CAN base clock is divisible by 8 * nominal bitrate without
              // remainder
        status = ptrCAN->SetBitrate(
            ARM_CAN_BITRATE_NOMINAL, // Set nominal bitrate
            CAN_BITRATE_NOMINAL, // Set nominal bitrate to configured constant
                                 // value
            ARM_CAN_BIT_PROP_SEG(
                5U) | // Set propagation segment to 5 time quanta
                ARM_CAN_BIT_PHASE_SEG1(
                    1U) | // Set phase segment 1 to 1 time quantum (sample point
                          // at 87.5% of bit time)
                ARM_CAN_BIT_PHASE_SEG2(
                    1U) | // Set phase segment 2 to 1 time quantum (total bit is
                          // 8 time quanta long)
                ARM_CAN_BIT_SJW(1U)); // Resynchronization jump width is same as
                                      // phase segment 2
    } else if ((clock % (10U * CAN_BITRATE_NOMINAL)) ==
               0U) { // If CAN base clock is divisible by 10 * nominal bitrate
                     // without remainder
        status = ptrCAN->SetBitrate(
            ARM_CAN_BITRATE_NOMINAL, // Set nominal bitrate
            CAN_BITRATE_NOMINAL, // Set nominal bitrate to configured constant
                                 // value
            ARM_CAN_BIT_PROP_SEG(
                7U) | // Set propagation segment to 7 time quanta
                ARM_CAN_BIT_PHASE_SEG1(
                    1U) | // Set phase segment 1 to 1 time quantum (sample point
                          // at 90% of bit time)
                ARM_CAN_BIT_PHASE_SEG2(
                    1U) | // Set phase segment 2 to 1 time quantum (total bit is
                          // 10 time quanta long)
                ARM_CAN_BIT_SJW(1U)); // Resynchronization jump width is same as
                                      // phase segment 2
    } else if ((clock % (12U * CAN_BITRATE_NOMINAL)) ==
               0U) { // If CAN base clock is divisible by 12 * nominal bitrate
                     // without remainder
        status = ptrCAN->SetBitrate(
            ARM_CAN_BITRATE_NOMINAL, // Set nominal bitrate
            CAN_BITRATE_NOMINAL, // Set nominal bitrate to configured constant
                                 // value
            ARM_CAN_BIT_PROP_SEG(
                7U) | // Set propagation segment to 7 time quanta
                ARM_CAN_BIT_PHASE_SEG1(
                    2U) | // Set phase segment 1 to 2 time quantum (sample point
                          // at 83.3% of bit time)
                ARM_CAN_BIT_PHASE_SEG2(
                    2U) | // Set phase segment 2 to 2 time quantum (total bit is
                          // 12 time quanta long)
                ARM_CAN_BIT_SJW(2U)); // Resynchronization jump width is same as
                                      // phase segment 2
    } else {
        return false;
    }
    if (status != ARM_DRIVER_OK) {
        return false;
    }

    rx_obj_idx = 0xFFFFFFFFU;
    tx_obj_idx = 0xFFFFFFFFU;
    for (i = 0U; i < num_objects;
         i++) { // Find first available object for receive and transmit
        can_obj_cap =
            ptrCAN->ObjectGetCapabilities(i); // Get object capabilities
        if ((rx_obj_idx == 0xFFFFFFFFU) && (can_obj_cap.rx == 1U)) {
            rx_obj_idx = i;
        } else if ((tx_obj_idx == 0xFFFFFFFFU) && (can_obj_cap.tx == 1U)) {
            tx_obj_idx = i;
            break;
        }
    }
    if ((rx_obj_idx == 0xFFFFFFFFU) || (tx_obj_idx == 0xFFFFFFFFU)) {
        return false;
    }

    /* Configure CAN module hardware filters */
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
    status = ptrCAN->ObjectSetFilter(rx_obj_idx,
                                     ARM_CAN_FILTER_ID_MASKABLE_ADD,
                                     ARM_CAN_STANDARD_ID(0x000007FFUL),
                                     0U);
    if (status != ARM_DRIVER_OK) {
        return false;
    }

    status = ptrCAN->ObjectConfigure(
        tx_obj_idx, ARM_CAN_OBJ_TX); // Configure transmit object
    if (status != ARM_DRIVER_OK) {
        return false;
    }

    status = ptrCAN->ObjectConfigure(
        rx_obj_idx, ARM_CAN_OBJ_RX); // Configure receive object
    if (status != ARM_DRIVER_OK) {
        return false;
    }


#if (CAN_LOOPBACK == 1)
    if (can_cap.external_loopback != 1U) {
        return false;
    }
    status = ptrCAN->SetMode(
        ARM_CAN_MODE_LOOPBACK_EXTERNAL); // Activate loopback external mode
    if (status != ARM_DRIVER_OK) {
        return false;
    }
#else
    status =
        ptrCAN->SetMode(ARM_CAN_MODE_NORMAL); // Activate normal operation mode
    if (status != ARM_DRIVER_OK) {
        return false;
    }
#endif


    /* configure CAN interrupt registers */

    q_rx_msg_id = osMailCreate(osMailQ(q_rx_msg_id), NULL);
    q_tx_msg_id = osMailCreate(osMailQ(q_tx_msg_id), NULL);

    osThreadCreate(osThread(CAN_Receive_Process_Thread), CANmodule);
    osThreadCreate(osThread(CAN_Transmit_Thread), CANmodule);

    // if (osThreadNew(CAN_Receive_Process_Thread,
    //                 CANmodule,
    //                 &can_receive_process_thread_attr) == 0U) {
    //     ErrorHandler();
    // }
    // if (osThreadNew(
    //         CAN_Transmit_Thread, CANmodule, &can_transmit_thread_attr) ==
    //         0U) {
    //     ErrorHandler();
    // }

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    /* turn off the module */
    int32_t status =
        ptrCAN->PowerControl(ARM_POWER_OFF); // Power-up CAN controller
    if (status != ARM_DRIVER_OK) {
        return;
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
    int32_t status;

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

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters) {
            status = ptrCAN->ObjectSetFilter(rx_obj_idx,
                                             ARM_CAN_FILTER_ID_MASKABLE_ADD,
                                             ARM_CAN_STANDARD_ID(buffer->ident),
                                             buffer->mask);
            if (status != ARM_DRIVER_OK) {
                return false;
            }
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

        buffer->DLC = noOfBytes;
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

    CO_LOCK_CAN_SEND();
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
    // else {
    //     buffer->bufferFull = true;
    //     CANmodule->CANtxCount++;
    // }
    CO_UNLOCK_CAN_SEND();

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND();
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
    CO_UNLOCK_CAN_SEND();


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
