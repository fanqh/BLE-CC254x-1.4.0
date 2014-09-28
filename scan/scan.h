#ifndef _SCAN_H__
#define _SCAN_H__


#include "hal_types.h"
#define MAXIMUM_BARCODE_LENGTH  128



typedef struct {
  
	bool bHaveSend;
	uint16 length;
	uint16 cursor;	/** this is used in hid only **/
	uint8 code[MAXIMUM_BARCODE_LENGTH];
		
} barcode_t;

typedef struct {
	
	uint16 length;
	uint8 barcode[32];
	
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
void ready_state_enter(void);
void ready_state_exit(void);
void scanning_state_enter(void);
void scanning_state_exit(void);
uint8 RegisterForScanner( uint8 task_id );


#endif



