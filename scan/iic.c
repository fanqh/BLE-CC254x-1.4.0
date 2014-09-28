#include "iic.h"
#include "hal_types.h"


#include "hal_defs.h"
#include "ioCC2540.h"

void IIC_Stop(void);


void SET_SDA_HIGH(void)
{
     IIC_SEL &= ~IIC_SDA_BV ;  /* Set pin function to GPIO */
   
     IIC_DDR  &= (~IIC_SDA_BV);  
     IIC_INP  |= (IIC_SDA_BV);
}
void SET_SDA_LOW(void)
{
    IIC_DDR  |= (IIC_SDA_BV);   
    IIC_SDA_SBIT = 0;
}

void SET_SCLK_HIGH(void)
{
    IIC_SEL &= ~IIC_SCLK_BV ;  /* Set pin function to GPIO */
    
    IIC_DDR  &= (~IIC_SCLK_BV);
    IIC_INP  |= (IIC_SCLK_BV);
}

void SET_SCLK_LOW(void)
{
    IIC_DDR  |= (IIC_SCLK_BV);   
    IIC_SCLK_SBIT = 0;    
}
uint8  READ_SDA(void)
{
    IIC_DDR  &= (~IIC_SDA_BV);
    IIC_INP  |= (IIC_SDA_BV);
    return (IIC_SDA_PORT & IIC_SDA_BV);
}

uint8 ack;

void delay_us(uint16 time)
{
        uint16 i;
        
        while( time--)
        {
          for(i= 1; i>0; i--)
            ;
        }
              
}

//----初始化IIC-------------------------------------------------------------------
void IIC_Init(void)
{		
  
        IIC_SEL &= (~IIC_SDA_BV) ;  /* Set pin function to GPIO */
        IIC_SEL &= (~IIC_SCLK_BV) ;  /* Set pin function to GPIO */
        /* set direction for GPIO input  */     
        IIC_DDR  &= (~IIC_SDA_BV);
        IIC_DDR  &= (~IIC_SCLK_BV);
        /*INIT SDA SCLK OUTPUT HIGH*/
        SET_SDA_HIGH();
        SET_SCLK_HIGH();
       // IIC_Stop();
       
}
//----产生IIC起始信号-------------------------------------------------------------
void IIC_Start(void) 
{
	SET_SDA_HIGH();	  	  
	SET_SCLK_HIGH();
	delay_us(9); //9
 	SET_SDA_LOW();//START:when CLK is high,DATA change form high to low 
	delay_us(9); //	9
	SET_SCLK_LOW();//钳住I2C总线，准备发送或接收数据 
}	  
//----产生IIC停止信号-------------------------------------------------------------
void IIC_Stop(void)
{
	SET_SCLK_LOW();
	SET_SDA_LOW();//STOP:when CLK is high DATA change form low to high
 	delay_us(9);	//9
	SET_SCLK_HIGH(); 
	delay_us(9);	//9						   	
	SET_SDA_HIGH();//发送I2C总线结束信号
	delay_us(9);	//9						   	
}
//----一个SCL时钟----------------------------------------------------
void IIC_Clock()
{
	//uint8 sample;
	
	delay_us(9);   //9
	SET_SCLK_HIGH();      		//置时钟线为高，通知被控器开始接收数据位
	delay_us(9);		//保证时钟高电平周期大于4μs   9
	//sample=READ_SDA();
	SET_SCLK_LOW(); 
	//return sample;
}


uint8 GetState_Clock()
{
	uint8 sample;
	
	delay_us(9);   //9
	SET_SCLK_HIGH();      		//置时钟线为高，通知被控器开始接收数据位
	delay_us(9);		//保证时钟高电平周期大于4μs   9
	sample=READ_SDA();
       // delay_us(9);
	SET_SCLK_LOW(); 
	return sample;
}
//----发送一个字节----------------------------------------------------
void IIC_SendByte(uint8 c)
{
	uint8 BitCnt;
       // uint8 state;
  
	//要传送的数据长度为8位,
	for(BitCnt=0; BitCnt<8; BitCnt++)  //上升沿发送
   	{
          if((c & 0x80) >> 7)
              SET_SDA_HIGH();
          else
              SET_SDA_LOW();
          //  IIC_SDA = (c & 0x80) >> 7;
	    c<<=1;
	    IIC_Clock();
	}
        
	if(GetState_Clock())
    	    ack=0;
	else 
    	    ack=1;
}
//----接收一个字节----------------------------------------------------
uint8 IIC_RcvByte()
{
	uint8 retc;
	uint8 BitCnt;
  
	retc=0; 
    for(BitCnt=0;BitCnt<8;BitCnt++)
    {
     	retc=retc<<1;
		if(GetState_Clock())
        	retc++;
    }
	return retc;
}
//----应答----------------------------------------------------------
void IIC_Ack(uint8 a)
{
        if(a)
            SET_SDA_HIGH();
        else
            SET_SDA_LOW(); 
   	//IIC_SDA = a;     	//在此发出应答或非应答信号 
	IIC_Clock();
	//IIC_SDA = 1;
        SET_SDA_HIGH();
}

///////////////////////////////////////////////////////////////////
//----写一个字节----------------------------------------------------
bool IICputc(uint8 sla, uint8 c)
{
	IIC_Start();          //启动总线
	IIC_SendByte(sla);		//发送器件地址
	if(ack==0)
    	return FALSE;

	IIC_SendByte(c);          //发送数据
	if(ack==0)            
    	return FALSE;

	IIC_Stop();           //结束总线 
	return TRUE;
}

//----写有子地址----------------------------------------------------
bool IICwrite0(uint8 sla, uint8 suba, uint8 s)
{

	IIC_Start();          //启动总线
	IIC_SendByte(sla);        //发送器件地址
	if(ack==0)           
        {
          IIC_Stop(); 
    	  return FALSE;
        }

	IIC_SendByte(suba);       //发送器件子地址
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }

    {   
    	IIC_SendByte(s);      //发送数据
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }
		s++;
    } 
	IIC_Stop();           //结束总线
	return TRUE;
}

//----写有子地址----------------------------------------------------
bool IICwrite(uint8 sla, uint8 suba, uint8 *s, uint8 no)
{
	uint8 i;

	IIC_Start();          //启动总线
	IIC_SendByte(sla);        //发送器件地址
	if(ack==0)           
        {
          IIC_Stop(); 
    	  return FALSE;
        }

	IIC_SendByte(suba);       //发送器件子地址
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }

    for(i=0;i<no;i++)
    {   
    	IIC_SendByte(*s);      //发送数据
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }
		s++;
    } 
	IIC_Stop();           //结束总线
	return TRUE;
}

//----写无子地址----------------------------------------------------
bool IICwriteExt(uint8 sla, uint8 *s, uint8 no)
{
	uint8 i;

	IIC_Start();          //启动总线
	IIC_SendByte(sla);        //发送器件地址
	if(ack==0)
    	return FALSE;

	for(i=0;i<no;i++)
    {   
    	IIC_SendByte(*s);      //发送数据
		if(ack==0)
		    return FALSE;
		s++;
    } 
	IIC_Stop();           //结束总线 
	return TRUE;
}
//----读一个字节----------------------------------------------------
bool IICgetc(uint8 sla, uint8 *c)
{
	IIC_Start();          //启动总线
	IIC_SendByte(sla+1);      //发送器件地址
	if(ack==0)
		return FALSE;

	*c=IIC_RcvByte();         //接收数据
	IIC_Ack(1);           //接收完，发送非应答位，结束总线
	IIC_Stop();           //结束总线 
	return TRUE;
}




//----读有子地址----------------------------------------------------
bool IICread(uint8 sla, uint8 suba, uint8 *s, uint8 no)
{
	uint8 i;

        
        //i = READ_SDA();
	IIC_Start();          //启动总线
	IIC_SendByte(sla);        //发送器件地址
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }

	IIC_SendByte(suba);       //发送器件子地址
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }
		
	IIC_Start();			//重新启动总线
  	IIC_SendByte(sla+1);
  	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }

	for(i=0;i<no-1;i++)   //先接收前(no-1)字节
	{   
    	*s=IIC_RcvByte();      //接收数据
     	IIC_Ack(0);        //还未接收完，发送应答位  
     	s++;
   	} 
	*s=IIC_RcvByte();        //接收第no字节
	IIC_Ack(1);          //接收完，发送非应答位
	IIC_Stop();          //结束总线 
	return TRUE;
}
//----读无子地址----------------------------------------------------
bool IICreadExt(uint8 sla, uint8 *s, uint8 no)
{
	uint8 i;

	IIC_Start();
	IIC_SendByte(sla+1);		//R/W选择位，为1时为读， 为0 时为写
	if(ack==0)
		return FALSE;

	for(i=0;i<no-1;i++)   //先接收前（no-1)个字节
   	{   
    	*s=IIC_RcvByte();      //接收数据
     	IIC_Ack(0);        //未读取完，发送应答位  
     	s++;
	} 
   	*s=IIC_RcvByte();        //接收第no字节
   	IIC_Ack(1);          //接收完，发送非应答位
   	IIC_Stop();          //结束总线 
   	return TRUE;
}



