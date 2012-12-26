

/* *************************************************************
   Encoder definitions
   
   Add a #ifdef block to this file to include support for
   a particular encoder board or library. Then add the appropriate #define
   near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef ROBOGAIA
  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#endif

