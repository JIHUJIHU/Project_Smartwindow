/*

  GPIO_ResetBits(LCD, LCD_CS);   
  Delay(10);   
  GPIO_ResetBits(LCD, LCD_RES);     
  Delay(50000);  
  
  GPIO_SetBits(LCD, LCD_RES);     
  Delay(10);   
  GPIO_SetBits(LCD, LCD_CS);   
  Delay(10); 
  
  LcdCommand(0xa2);    // LCD bias=1/9
  Delay(10);  
  LcdCommand(0xa0);    // no flip on x-direction
  Delay(10);   
  LcdCommand(0xc8);    // flip on y-direction
  Delay(10);   
  LcdCommand(0x40);
  Delay(10);   
  LcdCommand(0x26);     
  Delay(50000);  
  Delay(50000);  
  Delay(50000);  
  Delay(50000);  
  Delay(50000);  
  Delay(50000);    
  LcdCommand(0x2e);
  Delay(50000);  
  Delay(50000);  
  Delay(50000);  
  Delay(50000);  
  Delay(50000);  
  Delay(50000);    
  LcdCommand(0x2f);    // 
  Delay(50000);  
  Delay(50000);  
  Delay(50000);  
  Delay(50000);  
  Delay(50000);  
  Delay(50000);    
  LcdCommand(0x25);    // turn on the internal voltage follower and wait until VOUT stable
  Delay(10);   
  LcdCommand(0x81);    // set reference voltage mode
  Delay(10);   
  LcdCommand(0x20);  
  Delay(10);   
  LcdCommand(0xaf);
  Delay(10);   
  LcdCommand(0xb0);
  Delay(10);   
  LcdCommand(0x10);  
  Delay(10);   
  LcdCommand(0x00);  
  Delay(10);   
  
  LcdCommand(0xa0);    // no flip on x-direction
  Delay(10);   
  LcdCommand(0xc8);    // flip on y-direction
  Delay(10);   
  LcdCommand(0xa2);
  Delay(10);   
  LcdCommand(0x2f);     
  Delay(10);    
  LcdCommand(0x81);    // set reference voltage mode
  Delay(10);   
  LcdCommand(0x29);  
  Delay(10);   
  LcdCommand(0x40);
  Delay(10);   
  LcdCommand(0xaf);
  Delay(10);  



	




























*/