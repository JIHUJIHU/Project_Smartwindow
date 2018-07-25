#include "stm32f10x_lib.h"
#include <stdio.h>

void putstr(char *str);
size_t __write(int handle, const unsigned char *buf, size_t bufSize);
size_t __read(int handle, unsigned char * buffer, size_t size);

void delay_us(u32 nCount);
void delay_ms(u32 nCount);


size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
     size_t nChars = 0;
 
     if(handle == -1) return 0;
     else if(handle != 1 && handle != 2) return -1;
 
     for(;bufSize> 0; --bufSize){
          while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET);
          USART_SendData(USART1, (u8)*buf);
          ++buf;
          ++nChars;
     }

     return nChars;
}
 
size_t __read(int handle, unsigned char * buffer, size_t size)
{
     size_t nChars = 0;
     unsigned char c;
 
     if (handle != 0) return -1;
 
     for (; size > 0; --size){
          while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == RESET);
          c = USART_ReceiveData(USART1);
 
          if (c == 0) break;
 
          *buffer++ = c;
          ++nChars;
     }
 
     return nChars;
}



void delay_us(u32 nCount)
{
     int i;
     for(; nCount !=0; nCount--){
          for(i=0; i<10; i++)   asm("NOP");
     }
}

void delay_ms(u32 nCount)
{
     for(; nCount !=0; nCount--)    delay_us(1000);
}
