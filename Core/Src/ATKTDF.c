#include "ATKTDF.h"
#include "MY_USART.h"
#include "usart.h"
#include "Steering.h"
extern int  rx_lenu4;
 ATKTDF_USART atktdf;
char ATKADD[]={0x51,0x0A,0xFF,0xFF,0x00,0x03,0x02,0x02,0x5E};//找地址
char ATKreset[]={0x51,0x0A,0x00,0x01,0x01,0x00,0x01,0x01,0x00,0x5F};//初始化发送
char ATKNORMAL[]= {0x51, 0x0A, 0x00, 0x01, 0x01, 0x0A, 0x01, 0x00, 0x00, 0x68 };
char ATKMOBE[]= {0x51, 0x0A, 0x00, 0x01, 0x01, 0x0A, 0x01, 0x01, 0x00 ,0x69};

char ATK_MOB_DATA[]={0x51,0x0A ,0x00 ,0x01,0x00,0x05 ,0x02 ,0x00,0x63}  ;
uint16_t atk_location_buffer[10];
char ATK_high_accuracy[]={0x51,0x0A,0x00,0x01,0x01,0x08,0x01,0x01,0x00 ,0x67};
char ATK_high_speed[]={   0x51,0x0A, 0x00,0x01,0x01,0x08,0x01,0x03,0x00 ,0x69};
char ATK_calibration[]={0x51,0x0A,0x00,0x01,0x01,0x09,0x01,0x04,0x00 ,0x6B};//校准

volatile int Init_atk=0;//开启状态位


void ATKTDF_DATA_analysis(void)
{

	if(Init_atk==0||Init_atk==1||Init_atk==2||Init_atk==3)
	{
	   
		
		  return;
    }
	
			
	
	if(rx_lenu4<=5){
	  for(int k=0;k<79;k++)
		   {
			 atktdf. RxBuffer[k]=0;
			   
		   }
	return ;}	
	
	
	
	
	
	
	
	
	
	
	
	
	 for(int i=0;i<=79;i++)
	{
	   if(atktdf.RxBuffer[i]==0x55&&atktdf.RxBuffer[i+1]==0x0A&&atktdf.RxBuffer[i+6]==0x05&&atktdf.RxBuffer[i+7]==0x02)
	   {
		atktdf. head_location= i;  //得到数据的是i+8，i+9
		  for(int k=0;k<=14;k++)
		   {
			 atktdf. middle_Buffer[k]=atktdf.RxBuffer[atktdf. head_location+k];
			   
		   }
	   
		  break;
	   }
		else if(atktdf.RxBuffer[i]==0x55&&atktdf.RxBuffer[i+1]==0x0A&&atktdf.RxBuffer[i+6]!=0x05&&atktdf.RxBuffer[i+7]!=0x02&&Init_atk==1)
		{
			  for(int k=0;k<=79;k++)
		   {
			 atktdf. RxBuffer[k]=0;
			   
		   }
		   return;
		
		
		}
	}
	
	
	for(int g=0;g<=15;g++)
	{
	      switch( atktdf. middle_Buffer[g])
		  {
			  case 0x55:
						if(atktdf. middle_Buffer[g]==0x55&& atktdf. middle_Buffer[g+1]==0x0A&&atktdf.ATK_STAUE==0)
						{
							atktdf.ATK_STAUE=1;
							continue;
						}
						else if(atktdf.ATK_STAUE!=0)
						{
							
						for(int i=0;i<=79;i++)
							{atktdf. middle_Buffer[i]=0;
							}							
						continue;
						
						
						}
						
						
			   case 0x05:
				   if(atktdf.ATK_STAUE==1)
				   {
					   if( atktdf. middle_Buffer[g]==0x05&& atktdf. middle_Buffer[g+1]==0x02)
					   {
					atk_location_buffer[0]=	 atktdf. middle_Buffer[g+2]; 
					atk_location_buffer[1]=	 atktdf. middle_Buffer[g+3];    
						 atktdf.LOCATION= (uint16_t)atk_location_buffer[0] << 8 | atk_location_buffer[1];
						    for(int i=0;i<=29;i++)
							{atktdf. middle_Buffer[i]=0;
							}
							for(int k=0;k<=79;k++)
							{
							  atktdf.RxBuffer[k]=0 ;
							
							}

							atktdf.ATK_STAUE=0;
							return;
						  
						   
						   
					   }
				   
				   
				   }


					   
		  }
	  }	
}


 void ATKTDF_Init(void)
 {
 
  //HAL_UART_Transmit_DMA(&huart4,(uint8_t *)ATKreset,sizeof(ATKreset));

	 if(Init_atk==0) 
	 {HAL_UART_Transmit_DMA(&huart4,(uint8_t *)ATKMOBE,sizeof(ATKMOBE));
  HAL_Delay(100);
 Init_atk++;	 }
if(Init_atk==1) 	
{  HAL_UART_Transmit_DMA(&huart4,(uint8_t *)ATK_high_speed,sizeof(ATK_high_speed));
  HAL_Delay(100);
 
  Init_atk++;  }


 

 
if(Init_atk==2)
{	for(int i=0;i<=79;i++)
			{
			   atktdf.RxBuffer[i]=0;
			}
 HAL_UART_Transmit_DMA(&huart4,(uint8_t *)ATK_calibration,sizeof(ATK_calibration));
	 
	 while(Init_atk==2)
	 {
		 for(int k=0;k<=79;k++)
		 {
			if(atktdf.RxBuffer[k]==0x43&&atktdf.RxBuffer[k+1]==0x61&&atktdf.RxBuffer[k+2]==0x6C&&atktdf.RxBuffer[k+3]==0x69&&atktdf.RxBuffer[k+4]==0x62&&atktdf.RxBuffer[k+5]==0x72&&atktdf.RxBuffer[k+6]==0x61&&atktdf.RxBuffer[k+10]==0x4F&&atktdf.RxBuffer[k+11]==0x6B)
			{
			 Init_atk=4;
			for(int i=0;i<=79;i++)
			{
			   atktdf.RxBuffer[i]=0;
			}	
		 	 break;
			}
		 }    }
	 
	 
	 }
 
 
 }
void ATKTDF_GET_add(void)
{

		HAL_UART_Transmit_DMA(&huart4,(uint8_t *)ATK_MOB_DATA,sizeof(ATK_MOB_DATA));
		

}
uint16_t  sa_ATK_fliter[20];
int ATKTDF_TIME=0;
void ATKTDF_DATA_FIFTER(void)
{   
	
	sa_ATK_fliter[ATKTDF_TIME]=atktdf.LOCATION;
	ATKTDF_TIME++;
	if(ATKTDF_TIME>=3)
	{ ATKTDF_TIME=0;
	  if(sa_ATK_fliter[0]>=sa_ATK_fliter[1]&&sa_ATK_fliter[1]>= sa_ATK_fliter[2] )
	  {   atktdf.LOCATION_fifle =sa_ATK_fliter[1];             
	  
	  
	  }
	   if(sa_ATK_fliter[2]>=sa_ATK_fliter[1]&&sa_ATK_fliter[1]>= sa_ATK_fliter[0] )
	  {   atktdf.LOCATION_fifle =sa_ATK_fliter[1];             
	  
	  
	  }
	   if(sa_ATK_fliter[1]>=sa_ATK_fliter[0]&&sa_ATK_fliter[0]>= sa_ATK_fliter[2] )
	  {   atktdf.LOCATION_fifle =sa_ATK_fliter[0];             
	  
	  
	  } if(sa_ATK_fliter[2]>=sa_ATK_fliter[0]&&sa_ATK_fliter[0]>= sa_ATK_fliter[1] )
	  {   atktdf.LOCATION_fifle =sa_ATK_fliter[0];             
	  
	  
	  }
	  if(sa_ATK_fliter[0]>=sa_ATK_fliter[2]&&sa_ATK_fliter[2]>= sa_ATK_fliter[1] )
	  {   atktdf.LOCATION_fifle =sa_ATK_fliter[2];             
	  
	  
	  }
	  if(sa_ATK_fliter[1]>=sa_ATK_fliter[2]&&sa_ATK_fliter[2]>= sa_ATK_fliter[0] )
	  {   atktdf.LOCATION_fifle =sa_ATK_fliter[2];             
	  
	  
	  }
	  
	  
	  
	  
	}
	
}


void ATK_tongs_COUNTER(void)
{
	if(atktdf.LOCATION_fifle<=70&&atktdf.LOCATION_fifle!=0)
	{
	    Steering_tongs_close();
	}	  
	if(atktdf.LOCATION_fifle>70||atktdf.LOCATION_fifle==0)	  
	{Steering_tongs_loosen();}
}

 

