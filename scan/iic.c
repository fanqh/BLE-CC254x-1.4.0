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

//----��ʼ��IIC-------------------------------------------------------------------
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
//----����IIC��ʼ�ź�-------------------------------------------------------------
void IIC_Start(void) 
{
	SET_SDA_HIGH();	  	  
	SET_SCLK_HIGH();
	delay_us(9); //9
 	SET_SDA_LOW();//START:when CLK is high,DATA change form high to low 
	delay_us(9); //	9
	SET_SCLK_LOW();//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//----����IICֹͣ�ź�-------------------------------------------------------------
void IIC_Stop(void)
{
	SET_SCLK_LOW();
	SET_SDA_LOW();//STOP:when CLK is high DATA change form low to high
 	delay_us(9);	//9
	SET_SCLK_HIGH(); 
	delay_us(9);	//9						   	
	SET_SDA_HIGH();//����I2C���߽����ź�
	delay_us(9);	//9						   	
}
//----һ��SCLʱ��----------------------------------------------------
void IIC_Clock()
{
	//uint8 sample;
	
	delay_us(9);   //9
	SET_SCLK_HIGH();      		//��ʱ����Ϊ�ߣ�֪ͨ��������ʼ��������λ
	delay_us(9);		//��֤ʱ�Ӹߵ�ƽ���ڴ���4��s   9
	//sample=READ_SDA();
	SET_SCLK_LOW(); 
	//return sample;
}


uint8 GetState_Clock()
{
	uint8 sample;
	
	delay_us(9);   //9
	SET_SCLK_HIGH();      		//��ʱ����Ϊ�ߣ�֪ͨ��������ʼ��������λ
	delay_us(9);		//��֤ʱ�Ӹߵ�ƽ���ڴ���4��s   9
	sample=READ_SDA();
       // delay_us(9);
	SET_SCLK_LOW(); 
	return sample;
}
//----����һ���ֽ�----------------------------------------------------
void IIC_SendByte(uint8 c)
{
	uint8 BitCnt;
       // uint8 state;
  
	//Ҫ���͵����ݳ���Ϊ8λ,
	for(BitCnt=0; BitCnt<8; BitCnt++)  //�����ط���
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
//----����һ���ֽ�----------------------------------------------------
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
//----Ӧ��----------------------------------------------------------
void IIC_Ack(uint8 a)
{
        if(a)
            SET_SDA_HIGH();
        else
            SET_SDA_LOW(); 
   	//IIC_SDA = a;     	//�ڴ˷���Ӧ����Ӧ���ź� 
	IIC_Clock();
	//IIC_SDA = 1;
        SET_SDA_HIGH();
}

///////////////////////////////////////////////////////////////////
//----дһ���ֽ�----------------------------------------------------
bool IICputc(uint8 sla, uint8 c)
{
	IIC_Start();          //��������
	IIC_SendByte(sla);		//����������ַ
	if(ack==0)
    	return FALSE;

	IIC_SendByte(c);          //��������
	if(ack==0)            
    	return FALSE;

	IIC_Stop();           //�������� 
	return TRUE;
}

//----д���ӵ�ַ----------------------------------------------------
bool IICwrite0(uint8 sla, uint8 suba, uint8 s)
{

	IIC_Start();          //��������
	IIC_SendByte(sla);        //����������ַ
	if(ack==0)           
        {
          IIC_Stop(); 
    	  return FALSE;
        }

	IIC_SendByte(suba);       //���������ӵ�ַ
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }

    {   
    	IIC_SendByte(s);      //��������
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }
		s++;
    } 
	IIC_Stop();           //��������
	return TRUE;
}

//----д���ӵ�ַ----------------------------------------------------
bool IICwrite(uint8 sla, uint8 suba, uint8 *s, uint8 no)
{
	uint8 i;

	IIC_Start();          //��������
	IIC_SendByte(sla);        //����������ַ
	if(ack==0)           
        {
          IIC_Stop(); 
    	  return FALSE;
        }

	IIC_SendByte(suba);       //���������ӵ�ַ
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }

    for(i=0;i<no;i++)
    {   
    	IIC_SendByte(*s);      //��������
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }
		s++;
    } 
	IIC_Stop();           //��������
	return TRUE;
}

//----д���ӵ�ַ----------------------------------------------------
bool IICwriteExt(uint8 sla, uint8 *s, uint8 no)
{
	uint8 i;

	IIC_Start();          //��������
	IIC_SendByte(sla);        //����������ַ
	if(ack==0)
    	return FALSE;

	for(i=0;i<no;i++)
    {   
    	IIC_SendByte(*s);      //��������
		if(ack==0)
		    return FALSE;
		s++;
    } 
	IIC_Stop();           //�������� 
	return TRUE;
}
//----��һ���ֽ�----------------------------------------------------
bool IICgetc(uint8 sla, uint8 *c)
{
	IIC_Start();          //��������
	IIC_SendByte(sla+1);      //����������ַ
	if(ack==0)
		return FALSE;

	*c=IIC_RcvByte();         //��������
	IIC_Ack(1);           //�����꣬���ͷ�Ӧ��λ����������
	IIC_Stop();           //�������� 
	return TRUE;
}




//----�����ӵ�ַ----------------------------------------------------
bool IICread(uint8 sla, uint8 suba, uint8 *s, uint8 no)
{
	uint8 i;

        
        //i = READ_SDA();
	IIC_Start();          //��������
	IIC_SendByte(sla);        //����������ַ
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }

	IIC_SendByte(suba);       //���������ӵ�ַ
	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }
		
	IIC_Start();			//������������
  	IIC_SendByte(sla+1);
  	if(ack==0)
        {
          IIC_Stop(); 
    	  return FALSE;
        }

	for(i=0;i<no-1;i++)   //�Ƚ���ǰ(no-1)�ֽ�
	{   
    	*s=IIC_RcvByte();      //��������
     	IIC_Ack(0);        //��δ�����꣬����Ӧ��λ  
     	s++;
   	} 
	*s=IIC_RcvByte();        //���յ�no�ֽ�
	IIC_Ack(1);          //�����꣬���ͷ�Ӧ��λ
	IIC_Stop();          //�������� 
	return TRUE;
}
//----�����ӵ�ַ----------------------------------------------------
bool IICreadExt(uint8 sla, uint8 *s, uint8 no)
{
	uint8 i;

	IIC_Start();
	IIC_SendByte(sla+1);		//R/Wѡ��λ��Ϊ1ʱΪ���� Ϊ0 ʱΪд
	if(ack==0)
		return FALSE;

	for(i=0;i<no-1;i++)   //�Ƚ���ǰ��no-1)���ֽ�
   	{   
    	*s=IIC_RcvByte();      //��������
     	IIC_Ack(0);        //δ��ȡ�꣬����Ӧ��λ  
     	s++;
	} 
   	*s=IIC_RcvByte();        //���յ�no�ֽ�
   	IIC_Ack(1);          //�����꣬���ͷ�Ӧ��λ
   	IIC_Stop();          //�������� 
   	return TRUE;
}



