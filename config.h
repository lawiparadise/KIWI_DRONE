#define VBATSCALE       131 // (*) change this value if readed Battery voltage is different than real voltage
#define VBATNOMINAL     126 // 12,6V full battery nominal voltage - only used for lcd.telemetry
#define VBATLEVEL_WARN1 107 // (*) 10,7V
#define VBATLEVEL_WARN2  99 // (*) 9.9V
#define VBATLEVEL_CRIT   93 // (*) 9.3V - critical condition: if vbat ever goes below this value, permanent alarm is triggered
#define NO_VBAT          16  // (*) Avoid beeping without any battery

#define QUADX

#define KIWI
#define HEX_NANO

#if defined(SBUS)
#define RC_CHANS 18
#elif defined(SPEKTRUM) || defined(SERIAL_SUM_PPM)
#define RC_CHANS 12
#else
#define RC_CHANS 8
#endif

/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500

#define SERIAL0_COM_SPEED 115200
#define SERIAL1_COM_SPEED 115200
#define SERIAL2_COM_SPEED 115200
#define SERIAL3_COM_SPEED 115200

#define MINCOMMAND  -1056

#define MINTHROTTLE 1000 // (*)
#define MAXTHROTTLE 2000
