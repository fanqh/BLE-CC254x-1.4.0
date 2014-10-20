
#include "scan.h"
#include "hal_board_cfg.h"
#include "hal_drivers.h"
#include "OSAL_Timers.h"


scanner_state_t scanner_state;

static uint8 registeredScannerTaskID = 0xff;

uint8 RegisterForScanner( uint8 task_id )
{
  // Allow only the first task
  if ( registeredScannerTaskID == 0xff )
  {
    registeredScannerTaskID = task_id;
    return ( TRUE );
  }
  else
    return ( FALSE );
}

void scanner_ready_state_enter(void) 
{
  
    scanner_state = SCANNER_READY;
    HAL_SCAN_DISABLE();
	
}

void scanner_ready_state_exit(void) 
{
    scanner_state = SCANNER_INVALID;
}


void scanner_scanning_state_enter(void) 
{
   
    scanner_state = SCANNER_SCANNING;
    HAL_SCAN_ENABLE(); 
    osal_start_timerEx( registeredScannerTaskID, SCANNER_TIMEOUT_EVENT, 3000);
}

void scanner_scanning_state_exit(void) 
{
    HAL_SCAN_DISABLE();  
  
}

scanner_state_t Get_Scanner_State(void)
{
    return scanner_state;
}

/** tab is not included **/
static bool isDisplayableChar(uint8 chr) 
{
	
	if (chr >= ' ' && chr <='~') 
        {
		
		return TRUE;
	}
	
	return FALSE;
}

static bool isLineFeed(uint8 chr) 
{
	
	return (chr == 0x0A);
}

static bool isCarriageReturn(uint8 chr) 
{
	
	return (chr == 0x0D);
}


static bool fill_byte(barcode_t* bcode, uint8 chr) 
{
	
	uint16 len = bcode ->length;
	
	/** if overflow, refuse **/
	if ( len >= DATA_LEN_MAX ) 
        {
		
		return FALSE;
	}
	
	/** if first character **/
	if ( len == 0) 
        {
		
		if ( isDisplayableChar(chr) ) 
                 {	/** only displayable char is allowed for the first char **/
			
			bcode ->code[0] = chr;
			bcode ->length++;
			
			return TRUE;
		}
		
		return FALSE;
	}
	
	/**if ( isLineFeed(bcode ->code[len - 1]) ) 
    {	 if trailing char is LF, then no more char is allowed 0x0a  modied by fan 
		
		return FALSE;
	}
	else **/if ( isCarriageReturn(bcode ->code[len - 1]) ) 
    { /** if trailing char is CR, then only LF is allowed  0x0d**/
		
		if ( isLineFeed(chr) ) 
                {
			
			bcode ->code[len] = chr;
			bcode ->length++;
			
			return TRUE;
		}
		
		return FALSE;
	}
	else 
        { /** only displayable char or CR is allowed **/
		
		if ( isDisplayableChar(chr) || isCarriageReturn(chr )|| isLineFeed(chr) ) 
                {/*add by fan*/
			
			bcode ->code[len] = chr;
			bcode ->length++;
			
			return TRUE;
		}
		
		return FALSE;
	}
} 

bool barcode_fill_raw_bytes(barcode_t* bcode, const uint8* src, uint16 len) {
	
	uint16 i;
	
	if (len == 0) {
		
		return TRUE;
	}
	
	if (bcode ->length + len > DATA_LEN_MAX) {
		
		return FALSE;
	}
	
	for (i = 0; i < len; i++) {
		
		if (FALSE == fill_byte(bcode, src[i])) {
			
			return FALSE;
		}
	}
	
	return TRUE;
}

void barcode_clear(barcode_t* bcode) {
	
	if (bcode) {
		
		uint16 i;
		
		bcode ->length = 0;
		
		for (i = 0; i < DATA_LEN_MAX; i++) {
			
			bcode ->code[i] = 0;
		}
	}
}


bool barcode_is_terminated(barcode_t* bcode) {
	
	if (bcode ->length < 3) /** at least 3, one displayable char and CR/LF **/
	{
		return FALSE;
	}

	
	if ( isCarriageReturn(bcode ->code[bcode ->length - 2]) && isLineFeed(bcode ->code[bcode ->length - 1]) ) {
		
		return TRUE;
	}
	
	return FALSE;
}