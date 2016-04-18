/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder initializeing to be run in the main setup() function */
  void initEncoders() {
    /* Nothing to do here */
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }

  /* Wrap the encoder reset function */
  void resetEncoders() {
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
  }
  
#elif defined(ROBOGAIA_3_AXIS)
  /* Robogaia 3-axis encoder shield */

  //*****************************************************  
  void initEncoders()
  //*****************************************************   
  {
      pinMode(chipSelectPin1, OUTPUT);
      pinMode(chipSelectPin2, OUTPUT);
      pinMode(chipSelectPin3, OUTPUT);
  
      digitalWrite(chipSelectPin1, HIGH);
      digitalWrite(chipSelectPin2, HIGH);
      digitalWrite(chipSelectPin3, HIGH);
 
      LS7366_Init();
      
      delay(100); 
  }
  
  long readEncoder(int encoder)
  //*****************************************************
  {
      unsigned int count1Value, count2Value, count3Value, count4Value;
      long result;

      /* The 3-axis encoder uses a 1-based index */
      if (encoder == 2) encoder = 3;
      if (encoder == 1) encoder = 2;
      if (encoder == 0) encoder = 1;
      
      selectEncoder(encoder);
      
      SPI.transfer(0x60); // Request count
      count1Value = SPI.transfer(0x00); // Read highest order byte
      count2Value = SPI.transfer(0x00);
      count3Value = SPI.transfer(0x00);
      count4Value = SPI.transfer(0x00); // Read lowest order byte
      
      deselectEncoder(encoder);
     
      result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;
      
      return result;
  }//end func
  
  //*************************************************
  void selectEncoder(int encoder)
  //*************************************************
  {
    switch(encoder)
    {
       case 1:
          digitalWrite(chipSelectPin1,LOW);
          break;
       case 2:
         digitalWrite(chipSelectPin2,LOW);
         break;
       case 3:
         digitalWrite(chipSelectPin3,LOW);
         break;    
    }//end switch
    
  }//end func
  
  //*************************************************
  void deselectEncoder(int encoder)
  //*************************************************
  {
    switch(encoder)
    {
       case 1:
          digitalWrite(chipSelectPin1,HIGH);
          break;
       case 2:
         digitalWrite(chipSelectPin2,HIGH);
         break;
       case 3:
         digitalWrite(chipSelectPin3,HIGH);
         break;    
    }//end switch
    
  }//end func

  //*************************************************
  void resetEncoder(int encoder)
  //*************************************************
  {
    /* The 3-axis encoder uses a 1-based index */
    if (encoder == 2) encoder = 3;
    if (encoder == 1) encoder = 2;
    if (encoder == 0) encoder = 1;
    
    selectEncoder(encoder);
    SPI.transfer(CLR | CNTR);
    deselectEncoder(encoder);
  }

  /* Wrap the encoder reset function */
  void resetEncoders() {
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
  }
  
  // LS7366 Initialization and configuration
  //*************************************************
  void LS7366_Init(void)
  //*************************************************
  {
     // SPI initialization
     SPI.begin();
     SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
     delay(10);
     
     digitalWrite(chipSelectPin1,LOW);
     SPI.transfer(0x88); 
     SPI.transfer(0x03);
     digitalWrite(chipSelectPin1,HIGH); 
     
     
     digitalWrite(chipSelectPin2,LOW);
     SPI.transfer(0x88); 
     SPI.transfer(0x03);
     digitalWrite(chipSelectPin2,HIGH); 
     
     
     digitalWrite(chipSelectPin3,LOW);
     SPI.transfer(0x88); 
     SPI.transfer(0x03);
     digitalWrite(chipSelectPin3,HIGH); 
     
  }//end func
  
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;
        
	enc_last <<=2; //shift previous state two places
	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  
  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
        static uint8_t enc_last=0;
          	
	enc_last <<=2; //shift previous state two places
	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  /* Wrap the encoder initializeing to be run in the main setup() function */
  void initEncoders() {
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
  
    // Enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);
  
    // Tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // Tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
  
    // Enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }

  /* Wrap the encoder reset function */
  void resetEncoders() {
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
  }
#else
  #error A encoder driver must be selected!
#endif

#endif


