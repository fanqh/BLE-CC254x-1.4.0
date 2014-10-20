/**************************************************************************************************
  Filename:       simpleBLEperipheral.h
  Revised:        $Date: 2010-08-01 14:03:16 -0700 (Sun, 01 Aug 2010) $
  Revision:       $Revision: 23256 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  definitions and prototypes.

  Copyright 2010 - 2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef SIMPLEBLEPERIPHERAL_H
#define SIMPLEBLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif


//#define BUFF_LEN_MAX    256
//
//  
//typedef struct
//{
//    uint16 Sf;
//    uint16 Ss;
//    uint16 Sn;
//    uint16 size;
//    uint16 seq;
//}form_t;
//
//typedef
//{
//    uint16 uuid;
//    uint1  head;
//    uint8  data[BUFF_LEN_MAX];
//    uint8  tail;  
//}ble_data_t;
#define DATA_LEN_MAX   256
#define WINDOW_SIZE    3
#define SENDCOUNT_MAX  3
#define SEND_TIMEOUT    1000 
  
  typedef struct
{
  uint8 sf;
  uint8 ss;
  uint8 sn;
  uint8 size;
}form_t;

typedef struct
{       
   uint8 seq;
   uint8 size;
   uint8 data[DATA_LEN_MAX];
}frame_t;

typedef struct
{
  uint8 sendcount;
  frame_t frame;

}attribute_t;

typedef enum
{
  ACK,
  NAK
}type_t;

typedef struct
{
    type_t type;
    uint16 seq;
}ackornak_t;
/*********************************************************************
 * EXTERN
 */
  
extern uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// Simple BLE Peripheral Task Events
   
#define SBP_TIMEOUT0_EVT                              0x0001 
#define SBP_TIMEOUT1_EVT                              0x0002 
#define SBP_TIMEOUT2_EVT                              0x0004 
#define SBP_TIMEOUT3_EVT                              0x0008 
#define SBP_TIMEOUT4_EVT                              0x0010 
#define SBP_TIMEOUT5_EVT                              0x0020 
#define SBP_TIMEOUT6_EVT                              0x0040 
#define SBP_TIMEOUT7_EVT                              0x0080 
   
   
#define SBP_START_DEVICE_EVT                              0x0100
#define SBP_PERIODIC_EVT                                  0x0200
#define SBP_ADVER_TIMEOUT_EVT                             0x0400
#define SCAN_RESULT_MESSAGE                               0X0800

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void SimpleBLEPeripheral_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events );
//ack process
void ProcessACK(ackornak_t  a);
//ble send
void BLESend(uint8 *p, uint16 len);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEPERIPHERAL_H */
