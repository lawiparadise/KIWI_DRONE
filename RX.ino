
#define FAILSAFE_DETECT_TRESHOLD  985

//RAW RC values will be store here
#if defined(SBUS)
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#elif defined(SPEKTRUM) || defined(SERIAL_SUM_PPM)
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#else
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#endif


#if defined(SERIAL_SUM_PPM) //Channel order for PPM SUM RX Configs
  static uint8_t rcChannel[RC_CHANS] = {SERIAL_SUM_PPM};
#elif defined(SBUS) //Channel order for SBUS RX Configs
  // for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiWii, but it should be easy to integrate them.
  static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13,14,15,16,17};
  static uint16_t sbusIndex=0;
#elif defined(SPEKTRUM)
  static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11};
#else // Standard Channel order
  static uint8_t rcChannel[RC_CHANS]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
  static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit
#endif

void configureReceiver() {
  /******************    Configure each rc pin for PCINT    ***************************/
  #if defined(STANDARD_RX)
    #if defined(MEGA)
      DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
    #endif
    // PCINT activation
    for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ // i think a for loop is ok for the init.
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }
    PCICR = PCIR_PORT_BIT;
    
    /*************    atmega328P's Specific Aux2 Pin Setup    *********************/
    #if defined(PROMINI)
     #if defined(RCAUXPIN)
        PCICR  |= (1 << 0) ; // PCINT activated also for PINS [D8-D13] on port B
        #if defined(RCAUXPIN8)
          PCMSK0 = (1 << 0);
        #endif
        #if defined(RCAUXPIN12)
          PCMSK0 = (1 << 4);
        #endif
      #endif
    #endif
    
    /***************   atmega32u4's Specific RX Pin Setup   **********************/
    #if defined(PROMICRO)
      //Trottle on pin 7
      DDRE &= ~(1 << 6); // pin 7 to input
      PORTE |= (1 << 6); // enable pullups
      EIMSK |= (1 << INT6); // enable interuppt
      EICRB |= (1 << ISC60);
      // Aux2 pin on PBO (D17/RXLED)
      #if defined(RCAUX2PIND17)
        DDRB &= ~(1 << 0); // set D17 to input 
      #endif
      // Aux2 pin on PD2 (RX0)
      #if defined(RCAUX2PINRXO)
        DDRD &= ~(1 << 2); // RX to input
        PORTD |= (1 << 2); // enable pullups
        EIMSK |= (1 << INT2); // enable interuppt
        EICRA |= (1 << ISC20);
      #endif
    #endif
    
  /*************************   Special RX Setup   ********************************/
  #endif
  // Init PPM SUM RX
  #if defined(SERIAL_SUM_PPM)
    PPM_PIN_INTERRUPT; 
  #endif
  // Init Sektrum Satellite RX
  #if defined (SPEKTRUM)
    SerialOpen(SPEK_SERIAL_PORT,115200);
  #endif
  // Init SBUS RX
  #if defined(SBUS)
    SerialOpen(1,100000);
  #endif
}


void computeRC() {
  static uint16_t rcData4Values[RC_CHANS][4], rcDataMean[RC_CHANS];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  #if !(defined(RCSERIAL) || defined(OPENLRSv2MULTI)) // dont know if this is right here
    #if defined(SBUS)
      readSBus();
    #endif
    rc4ValuesIndex++;
    if (rc4ValuesIndex == 4) rc4ValuesIndex = 0;
    for (chan = 0; chan < RC_CHANS; chan++) {
      #if defined(FAILSAFE)
        uint16_t rcval = readRawRC(chan);
        if(rcval>FAILSAFE_DETECT_TRESHOLD || chan > 3) {        // update controls channel only if pulse is above FAILSAFE_DETECT_TRESHOLD
          rcData4Values[chan][rc4ValuesIndex] = rcval;
        }
      #else
        rcData4Values[chan][rc4ValuesIndex] = readRawRC(chan);
      #endif
      rcDataMean[chan] = 0;
      for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
      rcDataMean[chan]= (rcDataMean[chan]+2)>>2;
      if ( rcDataMean[chan] < (uint16_t)rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;
      if ( rcDataMean[chan] > (uint16_t)rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;
    }
  #endif
}


uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  #if defined(SPEKTRUM)
    readSpektrum();
    if (chan < RC_CHANS) {
      data = rcValue[rcChannel[chan]];
    } else data = 1500;
  #else
    uint8_t oldSREG;
    oldSREG = SREG; cli(); // Let's disable interrupts
    data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
    SREG = oldSREG;        // Let's restore interrupt state
  #endif
  return data; // We return the value correctly copied when the IRQ's where disabled
}
