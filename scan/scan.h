#ifndef _SCAN_H__
#define _SCAN_H__


#include "hal_types.h"
#include "simpleBLEPeripheral.h"


//SCAN_TRIG
#define SCAN_TRIG_BV           BV(0)
#define SCAN_TRIG_SBIT         P2_0
#define SCAN_TRIG_DDR          P2DIR
#define SCAN_TRIG_SEL          P2SEL
#define HAL_SCAN_DISABLE()       st( SCAN_TRIG_SBIT = 1; )
#define HAL_SCAN_ENABLE()        st( SCAN_TRIG_SBIT = 0; )


typedef struct {
  
	bool bHaveSend;
	uint16 length;
	uint16 cursor;	/** this is used in hid only **/
	uint8 code[DATA_LEN_MAX];
		
} barcode_t;

typedef struct {
	
	uint16 length;
	uint8 barcode[DATA_LEN_MAX];
	
} scanner_result_t;



typedef enum {
	
	SCANNER_READY,
	SCANNER_SCANNING,
	SCANNER_INVALID
	
} scanner_state_t;


typedef struct 
{
  
	scanner_state_t	 scan_state;
	
	barcode_t  barcode;
	
} scanner_task_t;



bool barcode_fill_raw_bytes(barcode_t* bcode, const uint8* src, uint16 len);
bool barcode_is_terminated(barcode_t* bcode);
void scanner_ready_state_enter(void);
void scanner_ready_state_exit(void);
void scanner_scanning_state_enter(void);
void scanner_scanning_state_exit(void);
uint8 RegisterForScanner( uint8 task_id );


#endif



