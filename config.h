#define QUADX

#define MINTHROTTLE 1000
#define MAXTHROTTLE 2000
#define MINCOMMAND  -1056
#define I2C_SPEED 400000L

#define KIWI
#define HEX_NANO

#define YAW_DIRECTION 1
#define TRI_YAW_CONSTRAINT_MIN 1020
#define TRI_YAW_CONSTRAINT_MAX 2000
#define TRI_YAW_MIDDLE 1500 // (*) tail servo center pos. - use this for initial trim; later trim midpoint via LCD
#define BI_PITCH_DIRECTION -1
#define ALLOW_ARM_DISARM_VIA_TX_YAW

#define TILT_PITCH_MIN    1020    //servo travel min, don't set it below 1020
#define TILT_PITCH_MAX    2000    //servo travel max, max value=2000
#define TILT_PITCH_MIDDLE 1500    //servo neutral value
#define TILT_PITCH_PROP   10      //servo proportional (tied to angle) ; can be negative to invert movement
#define TILT_PITCH_AUX_CH AUX3    //AUX channel to overwrite CAM pitch (AUX1-AUX4), comment to disable manual input and free the AUX channel
#define TILT_ROLL_MIN     1020
#define TILT_ROLL_MAX     2000
#define TILT_ROLL_MIDDLE  1500
#define TILT_ROLL_PROP    10
#define TILT_ROLL_AUX_CH  AUX4    //AUX channel to overwrite CAM Roll (AUX1-AUX4), comment to disable manual input and free the AUX channel

#define CAM_SERVO_HIGH 2000  // the position of HIGH state servo
#define CAM_SERVO_LOW 1020   // the position of LOW state servo
#define CAM_TIME_HIGH 1000   // the duration of HIGH state servo expressed in ms
#define CAM_TIME_LOW 1000    // the duration of LOW state servo expressed in ms

#define PITCH_DIRECTION_L 1 // left servo - pitch orientation
#define PITCH_DIRECTION_R -1  // right servo - pitch orientation (opposite sign to PITCH_DIRECTION_L, if servos are mounted in mirrored orientation)
#define ROLL_DIRECTION_L 1 // left servo - roll orientation
#define ROLL_DIRECTION_R 1  // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)
#define WING_LEFT_MID  1500 // (*) left servo center pos. - use this for initial trim; later trim midpoint via LCD
#define WING_RIGHT_MID 1500 // (*) right servo center pos. - use this for initial trim; later trim midpoint via LCD
#define WING_LEFT_MIN  1020 // limit servo travel range must be inside [1020;2000]
#define WING_LEFT_MAX  2000 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MIN 1020 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MAX 2000 // limit servo travel range must be inside [1020;2000]

#define SERVO_OFFSET     {  0,   0,   0,  0,   0,   0,  0,   0 } // (*) Adjust Servo MID Offset & Swash angles

#define RCSERIAL

#define SERIAL0_COM_SPEED 115200
#define SERIAL1_COM_SPEED 115200
#define SERIAL2_COM_SPEED 115200
#define SERIAL3_COM_SPEED 115200

#define MPU6050_LPF_98HZ

#define AP_MODE 40  // Create a deadspan for GPS.

#define FAILSAFE                                // uncomment  to activate the failsafe function
#define FAILSAFE_DELAY     12                    // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAFE_OFF_DELAY 40                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAFE_THROTTLE  (MINTHROTTLE + 50)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case

#define VBATSCALE       131 // (*) change this value if readed Battery voltage is different than real voltage
#define VBATNOMINAL     126 // 12,6V full battery nominal voltage - only used for lcd.telemetry
#define VBATLEVEL_WARN1 107 // (*) 10,7V
#define VBATLEVEL_WARN2  99 // (*) 9.9V
#define VBATLEVEL_CRIT   93 // (*) 9.3V - critical condition: if vbat ever goes below this value, permanent alarm is triggered
#define NO_VBAT          16  // (*) Avoid beeping without any battery

#define FAILSAFE                                // uncomment  to activate the failsafe function
#define FAILSAFE_DELAY     12                    // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAFE_OFF_DELAY 40                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAFE_THROTTLE  (MINTHROTTLE + 50)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case

/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500



#define MAG_DECLINIATION  -2.34f



