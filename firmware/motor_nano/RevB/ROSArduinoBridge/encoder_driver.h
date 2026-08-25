/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //LEFT encoder must be an adjacent PORTC bit pair (A on the lower bit);
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PC4  //pin A4
  #define LEFT_ENC_PIN_B PC5  //pin A5

  //RIGHT encoder must be an adjacent PORTD bit pair (A on the lower bit)
  #define RIGHT_ENC_PIN_A PD6 //pin 6
  #define RIGHT_ENC_PIN_B PD7 //pin 7
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

