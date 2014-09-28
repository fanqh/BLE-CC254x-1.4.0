#ifndef _IIC_H_
#define  _IIC_H_
#include "hal_types.h"


#define IIC_SEL        P1SEL
#define IIC_SDA_PORT   P1

//IIC  SDA
#define IIC_SDA_BV           BV(4)
#define IIC_SDA_SBIT         P1_4
#define IIC_DDR          P1DIR
#define IIC_INP         P1INP


//IIC  SCLK
#define IIC_SCLK_BV           BV(5)
#define IIC_SCLK_SBIT         P1_5

//#define SET_SDA_HIGH()        st( IIC_SDA_SBIT = 1; )
//#define SET_SDA_LOW()        st( IIC_SDA_SBIT = 0; )

#define IIC_SDA  IIC_SDA_SBIT

//#define READ_SDA  (IIC_SDA_PORT & IIC_SDA_BV)



//#define SET_SCLK_HIGH()        st( IIC_SCLK_SBIT = 1; )
//#define SET_SCLK_LOW()        st( IIC_SCLK_SBIT = 0; )
void IIC_Init(void);
bool IICread(uint8 sla, uint8 suba, uint8 *s, uint8 no);
bool IICreadExt(uint8 sla, uint8 *s, uint8 no);
bool IICwrite(uint8 sla, uint8 suba, uint8 *s, uint8 no);
bool IICwrite0(uint8 sla, uint8 suba, uint8 s);
bool IICwriteExt(uint8 sla, uint8 *s, uint8 no);

#endif

