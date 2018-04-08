// Proc auto detection
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#define PROMINI
#endif
#if defined(__AVR_ATmega32U4__) || defined(TEENSY20)
#define PROMICRO
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
#define MEGA
#endif

#define MULTITYPE 3


/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/
#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(MPU6050) || defined(MMA8451Q) || defined(NUNCHUCK)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110)
  #define MAG 1
#else
  #define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050) || defined(MPU3050) || defined(WMP)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA)
  #define BARO 1
#else
  #define BARO 0
#endif

#if defined(GPS_PROMINI_SERIAL) && defined(PROMINI)
  #define GPS_SERIAL 0
  #define GPS_PROMINI
  #define GPS_BAUD   GPS_PROMINI_SERIAL
#endif

#if defined(GPS_SERIAL)  || defined(I2C_GPS) || defined(GPS_FROM_OSD) || defined(TINY_GPS)
  #define GPS 1
#else
  #define GPS 0
#endif

#if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235) || defined(TINY_GPS_SONAR)
  #define SONAR 1
#else
  #define SONAR 0
#endif

#if defined(HEX_NANO)
#define MPU6050
#define BMP085
#define HMC5883
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  -Y; magADC[YAW]  = -Z;}
#define MPU6050_EN_I2C_BYPASS // MAG connected to the AUX I2C bus of MPU6050 
#undef INTERNAL_I2C_PULLUPS
//#define MINTHROTTLE 1050
//#define MAXTHROTTLE 2000
#define EXT_MOTOR_RANGE
#endif

#if defined(SBUS)
#define RC_CHANS 18
#elif defined(SPEKTRUM) || defined(SERIAL_SUM_PPM)
#define RC_CHANS 12
#else
#define RC_CHANS 8
#endif


/**************************  atmega32u4 (Promicro)  ***********************************/
#if defined(PROMICRO)
#if defined(MICROWII)
#define A32U4ALLPINS
#endif
#if !defined(TEENSY20)
#define LEDPIN_PINMODE             //
#define LEDPIN_TOGGLE              PIND |= 1<<5;     //switch LEDPIN state (Port D5)
#if !defined(PROMICRO10)
#define LEDPIN_OFF                 PORTD |= (1<<5);
#define LEDPIN_ON                  PORTD &= ~(1<<5);
#else
#define LEDPIN_OFF                PORTD &= ~(1<<5);
#define LEDPIN_ON                 PORTD |= (1<<5);
#endif
#else
#define LEDPIN_PINMODE           DDRD |= (1<<6);
#define LEDPIN_OFF               PORTD &= ~(1<<6);
#define LEDPIN_ON                PORTD |= (1<<6);
#define LEDPIN_TOGGLE            PIND |= 1<<6;     //switch LEDPIN state (Port D6)  
#endif
#if defined(D8BUZZER)
#define BUZZERPIN_PINMODE          DDRB |= (1<<4);
#if defined PILOTLAMP
#define    PL_PIN_ON            PORTB |= 1<<4;
#define    PL_PIN_OFF           PORTB &= ~(1<<4);
#else
#define BUZZERPIN_ON               PORTB |= 1<<4;
#define BUZZERPIN_OFF              PORTB &= ~(1<<4);
#endif

#elif defined(A32U4ALLPINS)
#define BUZZERPIN_PINMODE          DDRD |= (1<<4);
#if defined PILOTLAMP
#define    PL_PIN_ON    PORTD |= 1<<4;
#define    PL_PIN_OFF   PORTD &= ~(1<<4);
#else
#define BUZZERPIN_ON               PORTD |= 1<<4;
#define BUZZERPIN_OFF              PORTD &= ~(1<<4);
#endif
#else
#define BUZZERPIN_PINMODE          DDRD |= (1<<3);
#if defined PILOTLAMP
#define    PL_PIN_ON    PORTD |= 1<<3;
#define    PL_PIN_OFF   PORTD &= ~(1<<3);
#else
#define BUZZERPIN_ON               PORTD |= 1<<3;
#define BUZZERPIN_OFF              PORTD &= ~(1<<3);
#endif
#endif
#define POWERPIN_PINMODE           //
#define POWERPIN_ON                //
#define POWERPIN_OFF               //
#define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;   // PIN 2&3 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
#define PINMODE_LCD                DDRD |= (1<<2);
#define LCDPIN_OFF                 PORTD &= ~1;
#define LCDPIN_ON                  PORTD |= 1;
#define STABLEPIN_PINMODE          ;
#define STABLEPIN_ON               ;
#define STABLEPIN_OFF              ;
#define PPM_PIN_INTERRUPT          DDRE &= ~(1 << 6);PORTE |= (1 << 6);EIMSK |= (1 << INT6);EICRB |= (1 << ISC61)|(1 << ISC60);
#if !defined(SPEK_SERIAL_PORT)
#define SPEK_SERIAL_PORT         1
#endif
#define USB_CDC_TX                 3
#define USB_CDC_RX                 2

//soft PWM Pins
#define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<4;
#define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<4);
#define SOFT_PWM_2_PIN_HIGH        PORTF |= 1<<5;
#define SOFT_PWM_2_PIN_LOW         PORTF &= ~(1<<5);
#if !defined(A32U4ALLPINS)
#define SOFT_PWM_3_PIN_HIGH        PORTF |= 1<<7;
#define SOFT_PWM_3_PIN_LOW         PORTF &= ~(1<<7);
#define SOFT_PWM_4_PIN_HIGH        PORTF |= 1<<6;
#define SOFT_PWM_4_PIN_LOW         PORTF &= ~(1<<6);
#define SW_PWM_P3                  A1
#define SW_PWM_P4                  A0
#else
#define SOFT_PWM_3_PIN_HIGH        PORTF |= 1<<4;
#define SOFT_PWM_3_PIN_LOW         PORTF &= ~(1<<4);
#define SOFT_PWM_4_PIN_HIGH        PORTF |= 1<<5;
#define SOFT_PWM_4_PIN_LOW         PORTF &= ~(1<<5);
#define SW_PWM_P3                  A2
#define SW_PWM_P4                  A3
#endif

// Servos
#define SERVO_1_PINMODE   DDRF |= (1<<7); // A0
#define SERVO_1_PIN_HIGH  PORTF|= 1<<7;
#define SERVO_1_PIN_LOW   PORTF &= ~(1<<7);
#define SERVO_2_PINMODE   DDRF |= (1<<6); // A1
#define SERVO_2_PIN_HIGH  PORTF |= 1<<6;
#define SERVO_2_PIN_LOW   PORTF &= ~(1<<6);
#define SERVO_3_PINMODE   DDRF |= (1<<5); // A2
#define SERVO_3_PIN_HIGH  PORTF |= 1<<5;
#define SERVO_3_PIN_LOW   PORTF &= ~(1<<5);
#if !defined(A32U4ALLPINS)
#define SERVO_4_PINMODE   DDRD |= (1<<4); // 4
#define SERVO_4_PIN_HIGH  PORTD |= 1<<4;
#define SERVO_4_PIN_LOW   PORTD &= ~(1<<4);
#else
#define SERVO_4_PINMODE   DDRF |= (1<<4); // A3
#define SERVO_4_PIN_HIGH  PORTF |= 1<<4;
#define SERVO_4_PIN_LOW   PORTF &= ~(1<<4);
#endif
#define SERVO_5_PINMODE   DDRC |= (1<<6); // 5
#define SERVO_5_PIN_HIGH  PORTC|= 1<<6;
#define SERVO_5_PIN_LOW   PORTC &= ~(1<<6);
#define SERVO_6_PINMODE   DDRD |= (1<<7); // 6
#define SERVO_6_PIN_HIGH  PORTD |= 1<<7;
#define SERVO_6_PIN_LOW   PORTD &= ~(1<<7);
#define SERVO_7_PINMODE   DDRB |= (1<<6); // 10
#define SERVO_7_PIN_HIGH  PORTB |= 1<<6;
#define SERVO_7_PIN_LOW   PORTB &= ~(1<<6);
#define SERVO_8_PINMODE   DDRB |= (1<<5); // 9
#define SERVO_8_PIN_HIGH  PORTB |= 1<<5;
#define SERVO_8_PIN_LOW   PORTB &= ~(1<<5);

//Standart RX
#define THROTTLEPIN                  3
#if defined(A32U4ALLPINS)
#define ROLLPIN                    6
#define PITCHPIN                   2
#define YAWPIN                     4
#define AUX1PIN                    5
#else
#define ROLLPIN                    4
#define PITCHPIN                   5
#define YAWPIN                     2
#define AUX1PIN                    6
#endif
#define AUX2PIN                      7
#define AUX3PIN                      1 // unused 
#define AUX4PIN                      0 // unused 
#if !defined(RCAUX2PIND17)
#define PCINT_PIN_COUNT          4
#define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4)
#else
#define PCINT_PIN_COUNT          5 // one more bit (PB0) is added in RX code
#define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4),(1<<0)
#endif
#define PCINT_RX_PORT                PORTB
#define PCINT_RX_MASK                PCMSK0
#define PCIR_PORT_BIT                (1<<0)
#define RX_PC_INTERRUPT              PCINT0_vect
#define RX_PCINT_PIN_PORT            PINB

#if !defined(A32U4ALLPINS) && !defined(TEENSY20)
#define V_BATPIN                  A3    // Analog PIN 3
#elif defined(A32U4ALLPINS)
#define V_BATPIN                  A4    // Analog PIN 4
#else
#define V_BATPIN                  A2    // Analog PIN 3
#endif
#if !defined(TEENSY20)
#define PSENSORPIN                A2    // Analog PIN 2 
#else
#define PSENSORPIN                A2    // Analog PIN 2 
#endif
#endif





#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(MPU6050) || defined(MMA8451Q) || defined(NUNCHUCK)
#define ACC 1
#else
#define ACC 0
#endif



#if defined(GIMBAL)
#define NUMBER_MOTOR     0
#define PRI_SERVO_FROM   1 // use servo from 1 to 2
#define PRI_SERVO_TO     2
#elif defined(FLYING_WING)
#define NUMBER_MOTOR     1
#define PRI_SERVO_FROM   1 // use servo from 1 to 2
#define PRI_SERVO_TO     2

#elif defined(SINGLECOPTER)
#define NUMBER_MOTOR     1
#define PRI_SERVO_FROM   4 // use servo from 4 to 7
#define PRI_SERVO_TO     7
#elif defined(DUALCOPTER)
#define NUMBER_MOTOR     2
#define PRI_SERVO_FROM   4 // use servo from 5 to 6
#define PRI_SERVO_TO     6

#elif defined(AIRPLANE)
#if defined (USE_THROTTLESERVO)
#define NUMBER_MOTOR     0
#else
#define NUMBER_MOTOR     1
#endif
#if defined(FLAPS)
#define PRI_SERVO_FROM   3 // use servo from 3 to 8    
#undef CAMTRIG             // Disable Camtrig on A2
#else
#define PRI_SERVO_FROM   4 // use servo from 4 to 8
#endif
#define PRI_SERVO_TO     8
#elif defined(BI)
#define NUMBER_MOTOR     2
#define PRI_SERVO_FROM   5 // use servo from 5 to 6
#define PRI_SERVO_TO     6
#elif defined(TRI)
#define NUMBER_MOTOR     3
#define PRI_SERVO_FROM   6 // use only servo 6
#define PRI_SERVO_TO     6
#elif defined(QUADP) || defined(QUADX) || defined(Y4)|| defined(VTAIL4)
#define NUMBER_MOTOR     4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X) || defined(HEX6H)
#define NUMBER_MOTOR     6
#elif defined(OCTOX8) || defined(OCTOFLATP) || defined(OCTOFLATX)
#define NUMBER_MOTOR     8
#elif defined(HELICOPTER)
#ifdef HELI_USE_SERVO_FOR_THROTTLE
#define NUMBER_MOTOR     0 // use servo to drive throttle output
#define PRI_SERVO_FROM   4 // use servo from 4 to 8
#define PRI_SERVO_TO     8
#else
#define NUMBER_MOTOR     1 // use 1 motor for throttle
#define PRI_SERVO_FROM   4 // use servo from 4 to 7
#define PRI_SERVO_TO     7
#endif
#endif
