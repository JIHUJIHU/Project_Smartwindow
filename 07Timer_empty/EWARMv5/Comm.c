#include "stm32f10x_lib.h"
#include "platform_config.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_map.h"
#include "stm32f10x_tim.h"

#include "main.h"
#include "rtc.h"

u8 Com1_Send(u8 type, u8 comm_type);
u8 Com1_Measure_data(u8 type, u8 comm_type);

extern void USART1_SEND(u8 i);

extern u32 Product_ID;
extern u8 Firmware_Version[8];


extern u8 Buffer_USART1[100];
extern u8 USART1_rx_end, USART1_len;
extern u8 Timer3_TCP_Check;

extern u16 IP1_PORT; 
extern u16 IP2_PORT;
extern u16 IP3_PORT;

extern u8 Firmware_Version[8];
extern u8 Server_Version[8];

extern u8 TCP_Send_Buf[120];
extern u8 TCP_Send_TVOC1[5];
extern u8 TCP_Send_TVOC2[5];
extern u8 TCP_Send_TVOCA[5];

extern u8 LATITUDE[10];
extern u8 LONGITUDE[11];

extern u8 Timer1_1sec;

extern u32 Product_ID;


/********************************************

********************************************/
u8 Com1_Send(u8 type, u8 comm_type)
{
	u8 temp[7] ={0};
	u32 temp32 =0;
	u16 temp16 =0;
	u8 error=0;
	
	USART1_SEND(0x02);     			// STX
	
	USART1_SEND(0x31);       		//Length
	USART1_SEND(0x36);
	
	USART1_SEND(type/10+0x30);       	//type
	USART1_SEND(type%10+0x30);
	
	USART1_SEND(comm_type/10+0x30);    //Com_type
	USART1_SEND(comm_type%10+0x30);
	
	temp32 = Product_ID%1000000;	     // Product ID
	temp[1]  = temp32/100000;
	temp32 = Product_ID%100000;
	temp[2] = temp32/10000;
	temp32 = Product_ID%10000;
	temp[3] = temp32/1000;
	temp16 = temp32%1000;
	temp[4] = temp16/100;
	temp16 = temp32%100;
	temp[5] = temp16/10;
	temp[6] = Product_ID%10; 	
	
	USART1_SEND(temp[1]+0x30);		// Send Product ID 
	USART1_SEND(temp[2]+0x30);
	USART1_SEND(temp[3]+0x30);
	USART1_SEND(temp[4]+0x30);
	USART1_SEND(temp[5]+0x30);
	USART1_SEND(temp[6]+0x30);
	
	USART1_SEND(Firmware_Version[0]+0x30);		// Send Firmware Version
	USART1_SEND(Firmware_Version[1]+0x30);
	USART1_SEND(Firmware_Version[2]+0x30);
	USART1_SEND(Firmware_Version[3]+0x30);
	USART1_SEND(Firmware_Version[4]+0x30);
	USART1_SEND(Firmware_Version[5]+0x30);
	
	USART1_SEND(0x03);     			// ETX
	
	USART1_rx_end=0;
	USART1_len=0;
	
	Timer3_TCP_Check=0;
	
	while(1){
		
		GPIO_ResetBits(GPIOA, GPIO_Pin_3);
		GPIO_SetBits(GPIOA, GPIO_Pin_3);
		
		if(USART1_rx_end){
			USART1_rx_end=0;
			USART1_len=0;
			
			break;
		}
		if(Timer3_TCP_Check > 5){
			error=1;
			break;
		}
	}	
	return error;		
}

extern tm_struct timer;

u8 Com1_Measure_data(u8 type, u8 comm_type)
{
	u8 temp[7] ={0};
	u32 temp32 =0;
	u16 temp16 =0;
	u8 error=0, i8;
	
	u8 TCP_Send_Time[14];
	u8 s_year[4], s_month[2], s_date[2], s_hour[2], s_min[2], s_sec[2];
	u8 temp1, temp8;
	u8 day_chg=0;   // 일변화 감지 
	u8 Month_change=0;
	
	USART1_SEND(0x02);     			// STX
	
	USART1_SEND(0x31);       		//Length
	USART1_SEND(0x30);
	USART1_SEND(0x34);
	
	USART1_SEND(type/10+0x30);       	//type
	USART1_SEND(type%10+0x30);
	
	USART1_SEND(comm_type/10+0x30);    //Com_type
	USART1_SEND(comm_type%10+0x30);
	
	temp32 = Product_ID%1000000;	     // Product ID
	temp[1]  = temp32/100000;
	temp32 = Product_ID%100000;
	temp[2] = temp32/10000;
	temp32 = Product_ID%10000;
	temp[3] = temp32/1000;
	temp16 = temp32%1000;
	temp[4] = temp16/100;
	temp16 = temp32%100;
	temp[5] = temp16/10;
	temp[6] = Product_ID%10; 	
	
	USART1_SEND(temp[1]+0x30);		// Send Product ID 
	USART1_SEND(temp[2]+0x30);
	USART1_SEND(temp[3]+0x30);
	USART1_SEND(temp[4]+0x30);
	USART1_SEND(temp[5]+0x30);
	USART1_SEND(temp[6]+0x30);
	
	 // Firmware Version
	for(i8=0; i8<6; i8++){
		USART1_SEND(Firmware_Version[i8]+0x30);
	}	
	
	USART1_SEND('|');
	
	// DATE
	
	RTC_Get();
	
	s_year[0]= timer.w_year/1000+0x30;
	temp16=timer.w_year%1000;
	s_year[1]=temp16/100+0x30;
	temp8=timer.w_year%100;
	s_year[2]=temp8/10+0x30;
	s_year[3]=temp8%10+0x30;
	TCP_Send_Time[0]=s_year[0];
	TCP_Send_Time[1]=s_year[1];
	TCP_Send_Time[2]=s_year[2];
	TCP_Send_Time[3]=s_year[3];		
	
	temp8=timer.w_month%100;
	temp1=temp8/10;
	s_month[0]=temp1+0x30;
	temp1=temp8%10;
	s_month[1]=temp1+0x30;
	TCP_Send_Time[4]=s_month[0];	
	TCP_Send_Time[5]=s_month[1];		
		
	temp8=timer.w_date%100;
	temp1=temp8/10;
	s_date[0]=temp1+0x30;
	temp1=temp8%10;
	s_date[1]=temp1+0x30;
	TCP_Send_Time[6]=s_date[0];	
	TCP_Send_Time[7]=s_date[1];	
		
	temp8=timer.hour%100;
	temp1=temp8/10;
	s_hour[0]=temp1+0x30;
	temp1=temp8%10;
	s_hour[1]=temp1+0x30;
	TCP_Send_Time[8]=s_hour[0];	
	TCP_Send_Time[9]=s_hour[1];	
	
	if(Month_change==0){
		if(temp8==0)day_chg++;       // 00시이면 '1'
		else day_chg=0;
	}
	if(temp8>=1)Month_change=0;
	
	if(day_chg>=1){
		day_chg=0;
		Month_change=1;
	}
	
	temp8=timer.min%100;
	temp1=temp8/10;
	s_min[0]=temp1+0x30;
	temp1=temp8%10;
	s_min[1]=temp1+0x30;	
	TCP_Send_Time[10]=s_min[0];	
	TCP_Send_Time[11]=s_min[1];
	
	temp8=timer.sec%100;
	temp1=temp8/10;
	s_sec[0]=temp1+0x30;
	temp1=temp8%10;
	s_sec[1]=temp1+0x30;	
	TCP_Send_Time[12]=s_sec[0];	
	TCP_Send_Time[13]=s_sec[1];	
	
	for(i8=0; i8<14; i8++){
		USART1_SEND(TCP_Send_Time[i8]);
	}
	
	USART1_SEND(':');
	
	USART1_SEND('C');
	USART1_SEND('O');
	for(i8=0; i8<5; i8++){                    //  전압값 
		USART1_SEND(TCP_Send_TVOC1[i8]);
	}	
	USART1_SEND(':');
	
	USART1_SEND('C');
	USART1_SEND('O');
	USART1_SEND('2');
	for(i8=0; i8<5; i8++){                    // TVOC2 전압값 
		USART1_SEND(TCP_Send_TVOC2[i8]);
	}
	USART1_SEND(':');
	
	USART1_SEND('N');
	USART1_SEND('H');
	USART1_SEND('3');
	USART1_SEND(0x30);
	USART1_SEND(0x30);
	USART1_SEND(0x31);		
	USART1_SEND(0x33);	
	USART1_SEND(':');
	
	USART1_SEND('H');
	USART1_SEND('2');
	USART1_SEND('S');
	USART1_SEND(0x30);
	USART1_SEND(0x30);
	USART1_SEND(0x30);		
	USART1_SEND(0x35);	
	USART1_SEND(':');
	
	USART1_SEND('T');
	USART1_SEND('V');
	USART1_SEND('O');
	USART1_SEND('C');
	for(i8=0; i8<5; i8++){                    // TVOC 평균 전압값 
		USART1_SEND(TCP_Send_TVOCA[i8]);
	}		
	USART1_SEND(':');
	
	USART1_SEND('T');
	USART1_SEND(0x32);
	USART1_SEND(0x32);
	USART1_SEND('.');		
	USART1_SEND(0x34);	
	USART1_SEND(':');
	
	USART1_SEND('H');
	USART1_SEND(0x33);
	USART1_SEND(0x32);
	USART1_SEND(':');
	
	USART1_SEND('D');
	USART1_SEND(0x32);
	USART1_SEND(0x32);
	USART1_SEND(0x30);
	USART1_SEND('.');		
	USART1_SEND(0x35);	
	USART1_SEND(':');
	
	USART1_SEND('W');
	USART1_SEND('S');
	USART1_SEND(0x30);
	USART1_SEND(0x30);
	USART1_SEND(0x33);
	USART1_SEND('.');		
	USART1_SEND(0x35);	
	USART1_SEND(':');
	
	USART1_SEND('W');
	USART1_SEND('D');
	USART1_SEND(0x31);
	USART1_SEND(0x30);
	USART1_SEND(0x35);
	
	USART1_SEND(0x03);     			// ETX
	
	USART1_rx_end=0;
	USART1_len=0;
	
	Timer3_TCP_Check=0;
/*	
	while(1){
		
		GPIO_ResetBits(GPIOA, GPIO_Pin_3);
		GPIO_SetBits(GPIOA, GPIO_Pin_3);
		
		if(USART1_rx_end){
			USART1_rx_end=0;
			USART1_len=0;
			
			break;
		}
		if(Timer3_TCP_Check > 5){
			error=1;
			break;
		}
	}	
	*/
	return error;		
	
}
