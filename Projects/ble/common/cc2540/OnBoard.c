/**************************************************************************************************
  Filename:       OnBoard.c
  Revised:        $Date: 2008-03-18 15:14:17 -0700 (Tue, 18 Mar 2008) $
  Revision:       $Revision: 16604 $

  Description:    This file contains the UI and control for the
                  peripherals on the EVAL development board
  Notes:          This file targets the Chipcon CC2430DB/CC2430EB


  Copyright 2005-2013 Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OnBoard.h"
#include "OSAL.h"
#include "OnBoard.h"

#include "hal_led.h"
#include "hal_key.h"
#include "hal_drivers.h"
#include "hal_board_cfg.h"
#include "hal_adc.h"
#include "hal_uart.h"
 
#include "hal_led.h"
 #include "iic.h"
 #include "tps65721.h"
#include "simpleBLEPeripheral.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Task ID not initialized
#define NO_TASK_ID 0xFF
#define ACTIVING_TIMEOUT  100
#define DEACTIVE_TIMEOUT   1000 
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 OnboardKeyIntEnable;

void hal_initialising_state_enter(void); 
void hal_initialising_state_exit(void);
void hal_activing_state_exit(void);
void hal_activing_state_enter(void);
void hal_active_state_exit(void);
void hal_deactive_state_exit(void);

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern uint8 LL_PseudoRand( uint8 *randData, uint8 dataLen );

#if   defined FEATURE_ABL
#include "..\..\util\ABL\app\sbl_app.c"
#elif defined FEATURE_SBL
#include "..\..\util\SBL\app\sbl_app.c"
#elif defined FEATURE_EBL
#include "..\..\util\EBL\app\sbl_app.c"
#elif defined FEATURE_UBL_MSD
#include "..\..\util\UBL\soc_8051\usb_msd\app\ubl_app.c"
#else
void appForceBoot(void);
#endif



hal_state_t hal_state;

typedef enum
{
    HOLD,
    RELEASE
}hal_key_enum_t;

typedef struct 
{
    hal_key_enum_t  hal_key_enum;
    uint16 holdtime;
}hal_key_state_t;


hal_key_state_t power_key;
/*********************************************************************
 * LOCAL VARIABLES
 */
void HalPowerReleaseFun(void);
void HalPowerShortFun(void);
void HalPowerLongFun(void);
// Registered keys task ID, initialized to NOT USED.
static uint8 registeredKeysTaskID = NO_TASK_ID;

static halUARTCfg_t uartConfig;

barcode_t scan;

/*********************************************************************
 * @fn      InitBoard()
 * @brief   Initialize the CC2540DB Board Peripherals
 * @param   level: COLD,WARM,READY
 * @return  None
 */

static void uartCB(uint8 port, uint8 event) 
{

  bool success;
  uint16 len, l;
  uint8 buff[DATA_LEN_MAX];
  
  scanner_scanning_state_exit();
  scanner_ready_state_enter();
  osal_stop_timerEx( Hal_TaskID, SCANNER_TIMEOUT_EVENT);
  
  len = Hal_UART_RxBufLen(HAL_UART_PORT_0);
  if(len<=DATA_LEN_MAX)
  {
      HalLedBlink (HAL_BUZZ, 1, 50, 500);
      l = HalUARTRead ( HAL_UART_PORT_0, buff, len );
      success = barcode_fill_raw_bytes(&scan, buff, l);
      if(success)
      {
         if(barcode_is_terminated(&scan))
         {
            scan.bHaveSend = true;
            //SCAN_RESULT_MESSAGE
            osal_set_event (simpleBLEPeripheral_TaskID, SCAN_RESULT_MESSAGE);
         }
      }

  }
  
}

void InitBoard( uint8 level )
{

  if ( level == OB_COLD )
  {
    // Interrupts off
    osal_int_disable( INTS_ALL );
    // Turn all LEDs off
    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_OFF );
    // Check for Brown-Out reset
//    ChkReset();
  }
  else  // !OB_COLD
  {
    
      uartConfig.configured = TRUE;
      uartConfig.baudRate = HAL_UART_BR_9600;
      uartConfig.flowControl = FALSE;
      uartConfig.flowControlThreshold = 0;
      uartConfig.rx.maxBufSize = 256;
      uartConfig.tx.maxBufSize = 256;
      uartConfig.idleTimeout = 6;
      uartConfig.intEnable = TRUE;
      uartConfig.callBackFunc = uartCB;
      
      HalUARTOpen(HAL_UART_PORT_0, &uartConfig);
    
    /* Initialize Key stuff */
    OnboardKeyIntEnable = HAL_KEY_INTERRUPT_ENABLE;
    HalKeyConfig( OnboardKeyIntEnable, OnBoard_KeyCallback);
    
    hal_state = initialising;
    hal_set_state(initialising);
    IIC_Init();
    
    HalGpioInit();
  }
}

/*********************************************************************
 * @fn        Onboard_rand
 *
 * @brief    Random number generator
 *
 * @param   none
 *
 * @return  uint16 - new random number
 *
 *********************************************************************/
uint16 Onboard_rand( void )
{
  uint16 randNum;

  LL_PseudoRand( (uint8 *)&randNum, 2 );

  return ( randNum );
}

/*********************************************************************
 * @fn      _itoa
 *
 * @brief   convert a 16bit number to ASCII
 *
 * @param   num -
 *          buf -
 *          radix -
 *
 * @return  void
 *
 *********************************************************************/
void _itoa(uint16 num, uint8 *buf, uint8 radix)
{
  char c,i;
  uint8 *p, rst[5];

  p = rst;
  for ( i=0; i<5; i++,p++ )
  {
    c = num % radix;  // Isolate a digit
    *p = c + (( c < 10 ) ? '0' : '7');  // Convert to Ascii
    num /= radix;
    if ( !num )
      break;
  }

  for ( c=0 ; c<=i; c++ )
    *buf++ = *p--;  // Reverse character order

  *buf = '\0';
}

/*********************************************************************
 *                        "Keyboard" Support
 *********************************************************************/

/*********************************************************************
 * Keyboard Register function
 *
 * The keyboard handler is setup to send all keyboard changes to
 * one task (if a task is registered).
 *
 * If a task registers, it will get all the keys. You can change this
 * to register for individual keys.
 *********************************************************************/
uint8 RegisterForKeys( uint8 task_id )
{
  // Allow only the first task
  if ( registeredKeysTaskID == NO_TASK_ID )
  {
    registeredKeysTaskID = task_id;
    return ( true );
  }
  else
    return ( false );
}

/*********************************************************************
 * @fn      OnBoard_SendKeys
 *
 * @brief   Send "Key Pressed" message to application.
 *
 * @param   keys  - keys that were pressed
 *          state - shifted
 *
 * @return  status
 *********************************************************************/
uint8 OnBoard_SendKeys( uint8 keys, uint8 state )
{
  keyChange_t *msgPtr;

  if ( registeredKeysTaskID != NO_TASK_ID )
  {
    // Send the address to the task
    msgPtr = (keyChange_t *)osal_msg_allocate( sizeof(keyChange_t) );
    if ( msgPtr )
    {
      msgPtr->hdr.event = KEY_CHANGE;
      msgPtr->state = state;
      msgPtr->keys = keys;

      osal_msg_send( registeredKeysTaskID, (uint8 *)msgPtr );
    }
    return ( SUCCESS );
  }
  else
    return ( FAILURE );
}

/*********************************************************************
 * @fn      OnBoard_KeyCallback
 *
 * @brief   Callback service for keys
 *
 * @param   keys  - keys that were pressed
 *          state - shifted
 *
 * @return  void
 *********************************************************************/
void OnBoard_KeyCallback ( uint8 keys, uint8 state )
{
  uint8 shift;
  (void)state;

  // shift key (S1) is used to generate key interrupt
  // applications should not use S1 when key interrupt is enabled
  shift = (OnboardKeyIntEnable == HAL_KEY_INTERRUPT_ENABLE) ? false : ((keys & HAL_KEY_SW_6) ? true : false);

  if ( OnBoard_SendKeys( keys, shift ) != SUCCESS )
  {
    // Process SW1 here
    if ( keys & HAL_KEY_POWER_BUTTON )  // Switch 1
    {
        power_key.hal_key_enum = HOLD;
        power_key.holdtime = 0;
    }
    else
    {
        HalPowerReleaseFun();
    }
    // Process SW2 here
    if ( keys & HAL_KEY_SCAN_BUTTON )  // Switch 2
    {
      
        uint16 voltage;
         
        
        voltage = HalAdcRead (7, HAL_ADC_RESOLUTION_8);
        {
        IICread(SLAVE_DEVICE_ADDR, CHGCONFIG1_BASE, &state, 1);
        }
        HAL_TURN_ON_LED_RED();
       //HAL_TURN_ON_LED_BLUE();
        
        osal_set_event (Hal_TaskID, HAL_KEY_FUNCTION_EVENT);
       
    }
    
  }

  /* If any key is currently pressed down and interrupt
     is still enabled, disable interrupt and switch to polling */
  if( keys != 0 )
  {
    if( OnboardKeyIntEnable == HAL_KEY_INTERRUPT_ENABLE )
    {
      OnboardKeyIntEnable = HAL_KEY_INTERRUPT_DISABLE;
      HalKeyConfig( OnboardKeyIntEnable, OnBoard_KeyCallback);
    }
  }
  /* If no key is currently pressed down and interrupt
     is disabled, enable interrupt and turn off polling */
  else
  {
    if( OnboardKeyIntEnable == HAL_KEY_INTERRUPT_DISABLE )
    {
      OnboardKeyIntEnable = HAL_KEY_INTERRUPT_ENABLE;
      HalKeyConfig( OnboardKeyIntEnable, OnBoard_KeyCallback);
    }
  }
}

/*********************************************************************
 * @fn      Onboard_soft_reset
 *
 * @brief   Effect a soft reset.
 *
 * @param   none
 *
 * @return  none
 *
 *********************************************************************/
__near_func void Onboard_soft_reset( void )
{
  HAL_DISABLE_INTERRUPTS();
  asm("LJMP 0x0");
}

#if   defined FEATURE_ABL
#elif defined FEATURE_SBL
#elif defined FEATURE_EBL
#elif defined FEATURE_UBL_MSD
#else
/*********************************************************************
 * @fn      appForceBoot
 *
 * @brief   Common force-boot function for the HCI library to invoke.
 *
 * @param   none
 *
 * @return  void
 *********************************************************************/
void appForceBoot(void)
{
  // Dummy function for HCI library that cannot depend on the SBL build defines.
}


static void disablepower(void)
{
    P0_6 = 0;
}

static void enablepower(void)
{
    P0_6 = 1;
}

void HalPowerReleaseFun(void)
{}

void hal_initialising_state_exit(void)
{
    enablepower();
}

void hal_initialising_state_enter(void) 
{	
  
    hal_state = initialising;
	/** floating, if user release button and power cable unplugged, system stop **/
	disablepower();	
    scanner_ready_state_enter();
}

void HalGpioInit(void)
{
    /*MUC POWER PIN ,MOTO PIN,  USB SW_DETECT,BLUE LED PIN  */
    P0SEL &= (~BV(6)) & ( (~BV(4)) & (~BV(1)) );    
    /*SET GPIO AS OUTPUT*/
    P0DIR |= BV(6) | (BV(1)) ;
    
    /*SET GPIO AS INPUT*/
    P0DIR &= ~BV(4);
    
    SCAN_TRIG_SEL &= ~BV(0);
    SCAN_TRIG_DDR |= BV(0);
    SCAN_TRIG_SBIT = 1;
    
    
    
}

void PowerKeyStateInit(void)
{
    power_key.hal_key_enum =  RELEASE;  
    power_key.holdtime = 0;
    osal_clear_event( Hal_TaskID, HAL_KEY_SHORT_EVENT);
    osal_clear_event( Hal_TaskID, HAL_KEY_LONG_EVENT);
                                  
}



void PowerKeyHoldTimeCount(void)
{
    power_key.holdtime++;
    if(power_key.holdtime == 10)
        osal_set_event (Hal_TaskID, HAL_KEY_SHORT_EVENT);
    else if(power_key.holdtime==20)  ///need to add some to stop event continuously
        osal_set_event (Hal_TaskID, HAL_KEY_LONG_EVENT);
}


void HalPowerLongFun(void)
{
  
}



void hal_activing_state_exit(void)
{}


void hal_activing_state_enter(void)
{
    hal_state = activing;
    HalLedBlink (HAL_BUZZ, 2, 50, 500);
    HalLedBlink (HAL_MOTOR, 2, 50, 500);
    enablepower();
    
    HalLedBlink (HAL_LED_BLUE, 0, 50, 500);
    /**have a time if timeout is arrived , active in enter */
    osal_start_timerEx( Hal_TaskID, HAL_ACTIVING_TIMEOUT, ACTIVING_TIMEOUT );
}

void hal_active_state_enter(void)
{
  osal_start_timerEx( Hal_TaskID, HAL_ACTIVE_AUTO_SHUTDOWN_TIMEOUT,  AUTO_SHUTDOWN_TIMEOUT);  //10MIN
}
void hal_active_state_exit(void)
{
  osal_stop_timerEx( Hal_TaskID, HAL_ACTIVE_AUTO_SHUTDOWN_TIMEOUT );
}

void hal_deactive_state_enter(void)
{
     HalLedBlink (HAL_BUZZ, 2, 60, 300);
     osal_start_timerEx( Hal_TaskID, HAL_DEACTIVE_TIMEOUT, DEACTIVE_TIMEOUT );
     osal_set_event (simpleBLEPeripheral_TaskID, HAL_MESSAGE_SWITCCH_OFF);
}
void hal_deactive_state_exit(void)
{}

void hal_set_state(hal_state_t state)
{
  switch (hal_state)
  {  
        case initialising :
            hal_initialising_state_exit();
            break;
        case activing:
            hal_activing_state_exit();
            break;
        case active:
            hal_active_state_exit();
            break;
            
        case deactive:
            hal_deactive_state_exit();
            break;
        default:
            break; 
    }    

        hal_state = state;
        switch(state)  
        { 
            case initialising:
                hal_initialising_state_enter();
                break;
            case activing:
                hal_activing_state_enter();
                break;
            case active:
                hal_active_state_enter();
                break;
            case deactive:
                hal_deactive_state_enter();
                break;
            default:
            break;
        }
  
  
}
#endif

/*********************************************************************
*********************************************************************/
