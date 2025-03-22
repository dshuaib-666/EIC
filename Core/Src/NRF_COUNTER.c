#include "NRF_COUNTER.h"
#include "NRF.h"
#include "MOTOR.h"
#include "HWT101.h"
#include "math.h"
#include "PID_COUNTER.h"
#include "location.h"
#include "MY_USART.h"
 #define pi 	3.14159265358979
extern struct NRF_BUFFER NBF_BUFF;  //����һ���ṹ������������
extern  HWT101_USART HWT101; //���ս��սṹ��
extern uint8_t Receive[33];

 extern int sa_NRF_KEY6;
extern int  sa_NRF_KEY6_SATUS;
 extern float X_real,Y_real;
//�Խṹ����г�ʼ��
void NRF_BUFFER_Init(void)
{
	NBF_BUFF.one=2040;	
	NBF_BUFF.two=2040;	
	NBF_BUFF.start=0;
	for(int k=0;k<=10;k++)
	{
	     Receive[k]=102;
	
	}
	for(int k=20;k<=32;k++)
	{
	     Receive[k]=0;
	
	}
}

float yaw=0,pitch,roll;
 float yaw_Expect=0;
float sa_bete=0;
int sa14=0,sa13=0;
int sa_yaw1=0;
int sa_yaw2=0;
int sa_yaw3=0;
float NRF_YAW=0;
extern int sa_NRF_GUDINGYAW;
int sa_NRF_1=0;
float record_left_yaw=0.0f;
//����ƫ���ǽ�����xy������
 //���� NBF_BUFF.two���м�ֵ��Ϊ2500-1500 	  ǰ��
 //���� NBF_BUFF.one���м�ֵ��Ϊ2000-2100	 ԭ��ת��
//	uint16_t three;//���ҡ�����£��м���ȥ1990-1940��1960��
//	uint16_t four;//���ҡ�����ң��м�����Ϊ1800-1860��1840��
int motor_left=0,motor_right=0;
void NRF_COUNTER_RIGHT(void)//�ұ߶���
{
  NRF_dispose();//����ҡ�˵�����
	 
	  //old
  if ((NBF_BUFF.two>1500)&&(NBF_BUFF.two<2500)){NBF_BUFF.two=2040;}
  if ((NBF_BUFF.one>2000)&&(NBF_BUFF.one<2100)){NBF_BUFF.one=2040;}
  if ((NBF_BUFF.three>1940)&&(NBF_BUFF.three<1990)){NBF_BUFF.three=1960;}       
  if ((NBF_BUFF.four>1800)&&(NBF_BUFF.four<1860)){NBF_BUFF.four=1840;}
  
//   sa_yaw1= 90*(NBF_BUFF.three-1960)/2064;
//   sa_yaw2= 90*(NBF_BUFF.four-1840)/2184;
  
//	  yaw=(atan2(sa_yaw1,sa_yaw2)*180.0f / pi)-90.0f;
//	  if(yaw<-180&yaw>-270){yaw=yaw+360.0f;}
//	  if(yaw>180||yaw<-180){yaw=0.0f;}	
      //   motor_left=(NBF_BUFF.two-2040)*5;
  
  motor_left=(NBF_BUFF.two-2040)*7+(NBF_BUFF.four-1840)*4;
  motor_right=(NBF_BUFF.two-2040)*7-(NBF_BUFF.four-1840)*4;
  
    
  
  Motor_370_respectively(motor_left,motor_right); 
	  //straight_line(motor_left,motor_left,yaw+HWT101.angle,&sa14,&sa13);
	
}
 
void NRF_COUNTER_LEFT(void)
{
		NRF_dispose();//����ҡ�˵�����
		
	  //old
  if ((NBF_BUFF.two>1500)&&(NBF_BUFF.two<2500)){NBF_BUFF.two=2040;}
  if ((NBF_BUFF.one>2000)&&(NBF_BUFF.one<2100)){NBF_BUFF.one=2040;}
  if ((NBF_BUFF.three>1940)&&(NBF_BUFF.three<1990)){NBF_BUFF.three=1960;}       
  if ((NBF_BUFF.four>1800)&&(NBF_BUFF.four<1860)){NBF_BUFF.four=1840;}
		
		sa_yaw1= 90*(NBF_BUFF.three-1960)/2064;
	  	sa_yaw2= 90*(NBF_BUFF.four-1840)/2184;
	  yaw=(atan2(sa_yaw1,sa_yaw2)*180.0f / pi)-90.0f;
	  if(yaw<-180&yaw>-270){yaw=yaw+360.0f;}
	  if(yaw>180||yaw<-180){yaw=0.0f;}	
		
	  if(((NBF_BUFF.three-1960.0f)*(NBF_BUFF.three-1960.0f)+(NBF_BUFF.four-1840.0f)*(NBF_BUFF.four-1840.0f))>=3422500.0f)
	  {
	       record_left_yaw=   yaw;
	  
	  
	  }
	  else {
	  
	         yaw= record_left_yaw;
	  
	  }
	 motor_left=(NBF_BUFF.two-2040)*8;
	  
//		 sa_bete=atan2 (NBF_BUFF.three,NBF_BUFF.four);
		
//		motor_left= (((((NBF_BUFF.three-1960)/20)*(NBF_BUFF.three-1960)/20) +((NBF_BUFF.four-1840)/20)*((NBF_BUFF.four-1840)/20))*2)/3;
//        motor_right= (((((NBF_BUFF.three-1960)/20)*(NBF_BUFF.three-1960)/20) +((NBF_BUFF.four-1840)/20)*((NBF_BUFF.four-1840)/20))*2)/3;
//       if(motor_left<200){
//	         yaw=0;
//	   }
//       if(sa_NRF_GUDINGYAW==1&&sa_NRF_1==0)
//	   {
//		sa_NRF_1=1;                                                                          +
//	    NRF_YAW=yaw;
//	   }
//		 if(sa_NRF_GUDINGYAW==1&&sa_NRF_1==1)
//	   { 
//	    yaw=NRF_YAW;
//	   }
//        if(sa_NRF_GUDINGYAW==0&&sa_NRF_1==1)
//	   { 
//			sa_NRF_1=0;
//			NRF_YAW=0;	
//	   }
		
		
		
		
		straight_line(motor_left,motor_left,yaw,&sa14,&sa13);
	
		
		

}
 int sa22=0,sa23=0;
 void  NRF_KEY6(void)
 {
	 if(sa_NRF_KEY6_SATUS==0){return;}
	 if(sa_NRF_KEY6==1&&sa_NRF_KEY6_SATUS==1){
	 // Location_GO_EYSE(52,90,X_real,Y_real);
		 //straight_line(4000,4000,10,&sa22,&sa23);
	 }
     if(sa_NRF_KEY6_SATUS==1&&sa_NRF_KEY6==0){
	 
	        Motor_370_respectively(0,0);
	 
	 }
 
 
 
 
 }

extern int sa_NRF_START;
//�ж�״̬λ������������״̬
void NRF_COUNTER_START(void)
{
	NRF_dispose();
	NRF_KEY6();
	if(sa_NRF_START==6&&sa_NRF_KEY6_SATUS==0)
	{
		NRF_COUNTER_RIGHT();
	
	}
	else if(sa_NRF_START==7&&sa_NRF_KEY6_SATUS==0)
	{
	    NRF_COUNTER_LEFT();
	}
	
	
	else{return;}


}

