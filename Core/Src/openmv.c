#include "OPENMV.h"
#include <stdlib.h>
User_USART__openmv OPEN_data; //将DMA的数据存入这个数组里

     char openmv_Data_rebuffer[40];
     char x_buffer[10]; //定义x与y的缓冲区 ，因为atio函数（用于将字符串数组变为整形）只能用与char类型数组，而 OPEN_data.RxBuffer[i]为uint8——t
		char y_buffer[10];
		char d_buffer[10];
		char staus_buffer[2];//是否有接收到数据的接收位中间数组
		     int x_buffer_test;
			 int x_buffer_111=0;
			 
			int x_index=0; //定义两个xy对应值用于一一对应值
			int y_index=0;
			int d_index=0;
			int x_static= 1; //定义遍历到了某一程度 ,遍历是由x开始
		    int y_static= 0;
			int d_static= 0;
			int y_static_middle=0;
			int d_static_middle=0;
//从open得到的数据格式是@开头+数据x+/r/n+数据y+/k
//x范围0-320，中间为160
//面积为五位数
//夹子为一位数
 extern  volatile uint8_t rx_len ;  //??????????????

  int  x_buffer_test_count=0;
void openmv_data_Init(void)
{
   OPEN_data.OPENMV_X=0;
   OPEN_data.area=0;
	OPEN_data.RX_staus=0;
	OPEN_data.duoji_FIFLTER =0;
}

int sa_switch_count=0;//次数
int sa_switch_satus=0;//状态
int sa_aite_count=0;//@位置
int sa_aite_count_error=0;
void openmv_data_get(void)
{
	 if(rx_len < 20) return;   //如果位数不对
//	 for(int k=0;k<=24;k++) //检测是否有多的@
//		{
//			if(OPEN_data.RxBuffer[k]=='@')
//			{   
//				sa_aite_count_error++;
//				if(sa_aite_count_error==2)
//				{
//					 for(int o=0;o<=24;o++) 
//					{    OPEN_data.RxBuffer[o]=0;
//					
//					
//					}
//					for(int o=0;o<=21;o++)
//					{
//					  openmv_Data_rebuffer[o]=0;
//					}
//					
//					
//					   OPEN_data.OPENMV_X=0;
//					   //return;
//				}
//				
//			
//			}
//			
//			if(k==23)
//			{
//				 sa_aite_count_error=0;
//			
//			}
//			
//		}	
	
	 for(int k=0;k<=24;k++)//
	{                 
          if(OPEN_data.RxBuffer[k]=='@'&&OPEN_data.RxBuffer[k+20]=='$')
		  {
		    
//			  if(k!=0)
//			  {
//				   
//				   
//					   OPEN_data.OPENMV_X=0;
//					 return;
//			  }
			      sa_aite_count= k;    //@位置
				 break;
			 }
		continue;
	}
	
			 
			 
	
			 
//	if(openmv_Data_rebuffer[0]!='@')
//	{
//		   for(int o=0;o<=21;o++)
//				   {
//					  openmv_Data_rebuffer[o]=0;
//				   
//				   }
//				    OPEN_data.OPENMV_X=0;
//					 return;
//		
//		
//		
//		
//	
//	}
			 
	
	for(int q=sa_aite_count;q<=20+sa_aite_count;q++)
		{
		openmv_Data_rebuffer[q]=	OPEN_data.RxBuffer[q-sa_aite_count];
				
		}			 
		//3.10	 
	for(int g=1;g<=25;g++)
	{
	      switch(openmv_Data_rebuffer[g])
		  {
			  case '/': 
				  if(openmv_Data_rebuffer[g]=='/'&&openmv_Data_rebuffer[g+1]=='r'&&sa_switch_satus==0)//则前面三个是x坐标
			  {
							sa_switch_satus=1;
			               for(int o=0;o<3;o++)
							{
				             x_buffer[o]=openmv_Data_rebuffer[g-3+o];//x_buffer将RXBUFF拆开
							}
//				           x_buffer[0]=Oopenmv_Data_rebuffer[g-3];//x_buffer将RXBUFF拆开
//                           x_buffer[1]=OPopenmv_Data_rebuffer[g-2];//x_buffer将RXBUFF拆开
//				           x_buffer[2]=openmv_Data_rebuffer[g-1];//x_buffer将RXBUFF拆开
				             OPEN_data.OPENMV_X=atoi(x_buffer);
			  
			  }
			 
		          if(openmv_Data_rebuffer[g]=='/'&&openmv_Data_rebuffer[g+1]=='k'&&	sa_switch_satus==1)//则前面三个是x坐标
		          {
					  	sa_switch_satus=2;
					  for(int o=0;o<8;o++)
					  {     y_buffer[o]=openmv_Data_rebuffer[g-8+o];}//x_buffer将RXBUFF拆开                                                        }
                       OPEN_data.area= atoi(y_buffer);  
				  }
				 break; 
			  case 'e':
				 if(openmv_Data_rebuffer[g]=='e'&&sa_switch_satus==2)//则前面三个是x坐标	
				 {
					 	sa_switch_satus++;
						d_buffer[0]= openmv_Data_rebuffer[g-2];
						d_buffer[1]= openmv_Data_rebuffer[g-1];
				        OPEN_data.duoji= atoi(d_buffer); 
						staus_buffer[0]=openmv_Data_rebuffer[g+1];
						OPEN_data.RX_staus=atoi(staus_buffer);
						
						
					 
					 
						
				 }
				  break; 
			  case '$':
					if(openmv_Data_rebuffer[g]=='$'&&sa_switch_satus==3&&openmv_Data_rebuffer[g-2]=='e')
					{	    for(int o=0;o<=59;o++)
						{
					 
					      OPEN_data.RxBuffer[o]=0;
					
						}
						  for(int o=0;o<=39;o++)
						{
						openmv_Data_rebuffer[o]=0;
				   
						}
						 for(int o=0;o<=9;o++)
						{
						x_buffer[o]=0;
				   
						}
						for(int o=0;o<=9;o++)
						{
						y_buffer[o]=0;
				   
						}

						for(int o=0;o<=9;o++)
						{
						d_buffer[o]=0;
				   
						}							
				
						
						sa_switch_satus=0;
						rx_len=0;
						return;
					}
					
					 break; 
				   
			  default:        break;
				  
				   
					
				   
//			  case '@':
//				  if(OPEN_data.RxBuffer[g]=='@'&&OPEN_data.RxBuffer[g+1]=='e')
//				for(int i=0;i<=22;i++)
//					{OPEN_data.RxBuffer[i]=0;
//					
//					
//					}			  
//					for(int i=0;i<=21;i++)
//					{openmv_Data_rebuffer[i]=0;
//					
//					
//					}		
		  
		  
		  }
	
	}
	 
//	  for(int o=0;o<=24;o++)
//				   {
//					  OPEN_data.RxBuffer[o]=0;
//				   
//				   
//				   }
				 
		

	
	

}
