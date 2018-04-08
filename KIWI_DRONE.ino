/*
   [KIWI DRONE] is modifed from FlexBot Project(http://flexbot.cc/wiki)
   [KIWI DRONE] is under the terms of the GNU General Public License
   [KIWI DRONE] is made for education of Software and Drone
   [KIWI DRONE] is made by WHIT-edu CodingBird(http://codingbird.kr)

   MultiWiiCopter by Alexandre Dubus www.multiwii.com March  2013     V2.2
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   any later version. see <http://www.gnu.org/licenses/>
*/

#include <avr/io.h> // AVR 기본 입출력 관련 헤더파일

#include "config.h" // 설정을 기록하기 위한 헤더 파일
#include "def.h" //정의를 기록하기 위한 헤더 파일

#include <avr/pgmspace.h> // PROGMEM을 사용하기 위한 헤더 파일
#define VERSION 220 // 버전 코드

#if defined(KIWI) // KIWI라는 키워드가 정의되어 있다면
volatile uint16_t serialRcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // 시리얼RC값 초기화
#endif

/*****RC alias*****/
enum rc { // rc라는 이름으로 열거형 선언(이름을 갖는 정수형의 상수 선언)
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

enum pid { // pid라는 이름으로 열거형 선언
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL,     // not used currently
  PIDITEMS
};

const char pidnames[] PROGMEM = // 데이터를 Flash메모리에 생성하는 키워드
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
  ;

enum box {
  BOXARM,
#if ACC //가속도 센서가 있으면
  BOXANGLE,
  BOXHORIZON,
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD)) // 기압 센서가 있으면
  BOXBARO,
#endif
#ifdef VARIOMETER // 승강 속도계(승강 속도를 측정하는 센서)가 있으면
  BOXVARIO,
#endif
#if MAG // 자기 센서가 있으면
  BOXMAG,
  BOXHEADFREE,
  BOXHEADADJ, // acquire heading for HEADFREE mode
#endif
#if defined(SERVO_TILT) || defined(GIMBAL)  || defined(SERVO_MIX_TILT) // 서보, 짐벌이 있으면
  BOXCAMSTAB,
#endif
#if defined(CAMTRIG) // CAMTRIG가 있으면
  BOXCAMTRIG,
#endif
#if GPS // GPS가 있으면
  BOXGPSHOME,
  BOXGPSHOLD,
#endif
#if defined(FIXEDWING) || defined(HELICOPTER) // 고정익이 있거나 프로펠러이면
  BOXPASSTHRU,
#endif
#if defined(BUZZER) // 부저가 있다면
  BOXBEEPERON,
#endif
#if defined(LED_FLASHER) // LED가 있다면
  BOXLEDMAX, // we want maximum illumination
  BOXLEDLOW, // low/no lights
#endif
#if defined(LANDING_LIGHTS_DDR) // 랜딩기어 LED가 있다면
  BOXLLIGHTS, // enable landing lights at any altitude
#endif
#ifdef INFLIGHT_ACC_CALIBRATION // ?
  BOXCALIB,
#endif
#ifdef GOVERNOR_P // ?
  BOXGOV,
#endif
#ifdef OSD_SWITCH // ?
  BOXOSD,
#endif
  CHECKBOXITEMS // ?
};

const char boxnames[] PROGMEM = // 역동적인 GUI설정을 위한 이름 설정
  "ARM;"
#if ACC
  "ANGLE;"
  "HORIZON;"
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  "BARO;"
#endif
#ifdef VARIOMETER
  "VARIO;"
#endif
#if MAG
  "MAG;"
  "HEADFREE;"
  "HEADADJ;"
#endif
#if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
  "CAMSTAB;"
#endif
#if defined(CAMTRIG)
  "CAMTRIG;"
#endif
#if GPS
  "GPS HOME;"
  "GPS HOLD;"
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
  "PASSTHRU;"
#endif
#if defined(BUZZER)
  "BEEPER;"
#endif
#if defined(LED_FLASHER)
  "LEDMAX;"
  "LEDLOW;"
#endif
#if defined(LANDING_LIGHTS_DDR)
  "LLIGHTS;"
#endif
#ifdef INFLIGHT_ACC_CALIBRATION
  "CALIB;"
#endif
#ifdef GOVERNOR_P
  "GOVERNOR;"
#endif
#ifdef OSD_SWITCH
  "OSD SW;"
#endif
  ;

const uint8_t boxids[] PROGMEM = {// boxes와 관련된 영구적인 ID. BOX함수를 특정짓기 위한 ID값을 의존? 사용?할 수 있다
  0, //"ARM;"
#if ACC
  1, //"ANGLE;"
  2, //"HORIZON;"
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  3, //"BARO;"
#endif
#ifdef VARIOMETER
  4, //"VARIO;"
#endif
#if MAG
  5, //"MAG;"
  6, //"HEADFREE;"
  7, //"HEADADJ;"
#endif
#if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
  8, //"CAMSTAB;"
#endif
#if defined(CAMTRIG)
  9, //"CAMTRIG;"
#endif
#if GPS
  10, //"GPS HOME;"
  11, //"GPS HOLD;"
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
  12, //"PASSTHRU;"
#endif
#if defined(BUZZER)
  13, //"BEEPER;"
#endif
#if defined(LED_FLASHER)
  14, //"LEDMAX;"
  15, //"LEDLOW;"
#endif
#if defined(LANDING_LIGHTS_DDR)
  16, //"LLIGHTS;"
#endif
#ifdef INFLIGHT_ACC_CALIBRATION
  17, //"CALIB;"
#endif
#ifdef GOVERNOR_P
  18, //"GOVERNOR;"
#endif
#ifdef OSD_SWITCH
  19, //"OSD_SWITCH;"
#endif
};
// 변수 설정
static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // loop문 한 번 도는 시간, PID loop를 고려하여 약간 차이가 있을 수 있음
static uint16_t calibratingA = 0;  // main loop에서 조절이 진행되고, 낮아지다가 0이 되면 normal mode 진입
static uint16_t calibratingB = 0;  // 기압 측정, 기압 초기화
static uint16_t calibratingG;
static uint16_t acc_1G;            // 1G에서 측정한 가속도
static uint16_t acc_25deg;
static int16_t  gyroADC[3], accADC[3], accSmooth[3], magADC[3];
static int16_t  heading, magHold, headFreeModeHold; // [-180;+180]
static uint8_t  vbat;                   // 배터리 전압 in 0.1V steps
static uint8_t  vbatMin = VBATNOMINAL;  // 최소 배터리 전압 in 0.1V steps
static uint8_t  rcOptions[CHECKBOXITEMS];
static int32_t  BaroAlt, EstAlt, AltHold; // in cm
static int16_t  BaroPID = 0;
static int16_t  errorAltitudeI = 0;
static int16_t  vario = 0;              // 승강계 in cm/s

#if defined(ARMEDTIMEWARNING)
static uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

static int16_t  debug[4];
static int16_t  sonarAlt; // ? to think about the unit

struct flags_struct {
  uint8_t OK_TO_ARM : 1 ;
  uint8_t ARMED : 1 ;
  uint8_t I2C_INIT_DONE : 1 ; // i2c GPS를 위해, 언제 i2c가 초기화 되는지 알아야 함. eeprom에서 인자값을 i2cgps에 업데이트 함(setup()에서 완료 됨)
  uint8_t ACC_CALIBRATED : 1 ;
  uint8_t NUNCHUKDATA : 1 ;
  uint8_t ANGLE_MODE : 1 ;
  uint8_t HORIZON_MODE : 1 ;
  uint8_t MAG_MODE : 1 ;
  uint8_t BARO_MODE : 1 ;
  uint8_t GPS_HOME_MODE : 1 ;
  uint8_t GPS_HOLD_MODE : 1 ;
  uint8_t HEADFREE_MODE : 1 ;
  uint8_t PASSTHRU_MODE : 1 ;
  uint8_t GPS_FIX : 1 ;
  uint8_t GPS_FIX_HOME : 1 ;
  uint8_t SMALL_ANGLES_25 : 1 ;
  uint8_t CALIBRATE_MAG : 1 ;
  uint8_t VARIO_MODE : 1;
} f;

//for log
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
static uint16_t cycleTimeMax = 0;       // 사이클 중 최댓값
static uint16_t cycleTimeMin = 65535;   // 사이클 중 최솟값
static uint16_t powerMax = 0;           // 전류 최댓값
static int32_t  BAROaltMax;         // 최댓값
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING)  || defined(LOG_PERMANENT)
static uint32_t armedTime = 0;
#endif

static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;

// 자동 ACC Offset 조정
#if defined(INFLIGHT_ACC_CALIBRATION)
static uint16_t InflightcalibratingA = 0;
static int16_t AccInflightCalibrationArmed;
static uint16_t AccInflightCalibrationMeasurementDone = 0;
static uint16_t AccInflightCalibrationSavetoEEProm = 0;
static uint16_t AccInflightCalibrationActive = 0;
#endif

//전력 측정
#if defined(POWERMETER)
#define PMOTOR_SUM 8                     // 합을 구하기 위한 pMeter[] 번호
static uint32_t pMeter[PMOTOR_SUM + 1];  // [0:7] 8개의 모터를 사용하고, 1개는 합을 위한 변수
static uint8_t pMeterV;                  // ConfigurationLoop()에서 paramStruct logic를 만족하기 위한 더미
static uint32_t pAlarm;                  // eeprom의 값을 [0:255]에서 이 값으로 스케일변경, pMeter[6]의 합과 비교
static uint16_t powerValue = 0;          // last known current
#endif
static uint16_t intPowerMeterSum, intPowerTrigger1;

//텔레메트리
#if defined(LCD_TELEMETRY)
static uint8_t telemetry = 0;
static uint8_t telemetry_auto = 0;
#endif
#ifdef LCD_TELEMETRY_STEP
static char telemetryStepSequence []  = LCD_TELEMETRY_STEP;
static uint8_t telemetryStepIndex = 0;
#endif

// rc 기능
#define MINCHECK 1100
#define MAXCHECK 1900

#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

static int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

static int16_t rcData[RC_CHANS];    // interval [1000;2000]
static int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
static int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE
static uint16_t rssi;               // range: [0;1023]

#if defined(SPEKTRUM)
volatile uint8_t  spekFrameFlags;
volatile uint32_t spekTimeLast;
#endif

#if defined(OPENLRSv2MULTI)
static uint8_t pot_P, pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
#endif

// 자이로+가속도 IMU(Inertial Measurement Unit, 관성 측정 장치)
static int16_t gyroData[3] = {0, 0, 0};
static int16_t gyroZero[3] = {0, 0, 0};
static int16_t angle[2]    = {0, 0}; // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

// 모터, 서보 기능
static int16_t axisPID[3];
static int16_t motor[NUMBER_MOTOR];
#if defined(SERVO)
static int16_t servo[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
#endif

// EEPROM
static uint8_t dynP8[3], dynD8[3];

//global_conf 정의
static struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE !
} global_conf;

//conf 정의
static struct {
  uint8_t checkNewConf;
  uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t angleTrim[2];
  uint16_t activate[CHECKBOXITEMS];
  uint8_t powerTrigger1;
#ifdef FLYING_WING
  uint16_t wing_left_mid;
  uint16_t wing_right_mid;
#endif
#ifdef TRI
  uint16_t tri_yaw_middle;
#endif
#if defined HELICOPTER || defined(AIRPLANE)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)
  int16_t servoTrim[8];
#endif
#if defined(GYRO_SMOOTHING)
  uint8_t Smoothing[3];
#endif
#if defined (FAILSAFE)
  int16_t failsafe_throttle;
#endif
#ifdef VBAT
  uint8_t vbatscale;
  uint8_t vbatlevel_warn1;
  uint8_t vbatlevel_warn2;
  uint8_t vbatlevel_crit;
  uint8_t no_vbat;
#endif
#ifdef POWERMETER
  uint16_t psensornull;
  uint16_t pleveldivsoft;
  uint16_t pleveldiv;
  uint8_t pint2ma;
#endif
#ifdef CYCLETIME_FIXATED
  uint16_t cycletime_fixated;
#endif
#ifdef MMGYRO
  uint8_t mmgyro;
#endif
#ifdef ARMEDTIMEWARNING
  uint16_t armedtimewarning;
#endif
  int16_t minthrottle;
#ifdef GOVERNOR_P
  int16_t governorP;
  int16_t governorD;
  int8_t  governorR;
#endif
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} conf;

//LOG_PERMANENT가 있다면 plog 스트럭트 구성
#ifdef LOG_PERMANENT
static struct {
  uint16_t arm;           // #arm events
  uint16_t disarm;        // #disarm events
  uint16_t start;         // #powercycle/reset/initialize events
  uint32_t armed_time ;   // copy of armedTime @ disarm
  uint32_t lifetime;      // sum (armed) lifetime in seconds
  uint16_t failsafe;      // #failsafe state @ disarm
  uint16_t i2c;           // #i2c errs state @ disarm
  uint8_t  running;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} plog;
#endif

//GPS 공통 변수
static int32_t  GPS_coord[2];
static int32_t  GPS_home[2];
static int32_t  GPS_hold[2];
static uint8_t  GPS_numSat;
static uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
static int16_t  GPS_directionToHome;                         // direction to home - unit: degree
static uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
static uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
static uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
static int16_t  GPS_angle[2] = { 0, 0 };                      // the angles that must be applied for GPS correction
static uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10
static uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
static uint8_t  GPS_Enable = 0;

#define LAT  0 // 위도
#define LON  1 // 경도
// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
static int16_t  nav[2];
static int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother


// default POSHOLD control gains
#define POSHOLD_P              .11
#define POSHOLD_I              0.0
#define POSHOLD_IMAX           20        // degrees

#define POSHOLD_RATE_P         2.0
#define POSHOLD_RATE_I         0.08      // Wind control
#define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX      20        // degrees

// default Navigation PID gains
#define NAV_P                  1.4
#define NAV_I                  0.20      // Wind control
#define NAV_D                  0.08      //
#define NAV_IMAX               20        // degrees

#define NAV_MODE_NONE          0
#define NAV_MODE_POSHOLD       1
#define NAV_MODE_WP            2
static uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode

static uint8_t alarmArray[16];           // array

#if BARO
static int32_t baroPressure;
static int32_t baroTemperature;
static int32_t baroPressureSum;
#endif

//Code합병하기
void annexCode() { // loop문에서 실행된다. 이 함수가 650ms보다 적은 시간동안 실행되면 루프문을 방해하지 않는다.
  static uint32_t calibratedAccTime;
  uint16_t tmp, tmp2;
  uint8_t axis, prop1, prop2;

#define BREAKPOINT 1500

  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if (rcData[THROTTLE] < BREAKPOINT) {
    prop2 = 100;
  } else {
    if (rcData[THROTTLE] < 2000) {
      prop2 = 100 - (uint16_t)conf.dynThrPID * (rcData[THROTTLE] - BREAKPOINT) / (2000 - BREAKPOINT);
    } else {
      prop2 = 100 - conf.dynThrPID;
    }
  }

  for (axis = 0; axis < 3; axis++) {
    tmp = min(abs(rcData[axis] - MIDRC), 500);
#if defined(DEADBAND)
    if (tmp > DEADBAND) {
      tmp -= DEADBAND;
    }
    else {
      tmp = 0;
    }
#endif
    if (axis != 2) { //ROLL & PITCH
      tmp2 = tmp / 100;
      rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
      prop1 = 100 - (uint16_t)conf.rollPitchRate * tmp / 500;
      prop1 = (uint16_t)prop1 * prop2 / 100;
    } else {      // YAW
      rcCommand[axis] = tmp;
      prop1 = 100 - (uint16_t)conf.yawRate * tmp / 500;
    }
    dynP8[axis] = (uint16_t)conf.P8[axis] * prop1 / 100;
    dynD8[axis] = (uint16_t)conf.D8[axis] * prop1 / 100;
    if (rcData[axis] < MIDRC) rcCommand[axis] = -rcCommand[axis];
  }

  tmp = constrain(rcData[THROTTLE], MINCHECK, 2000);
  tmp = (uint32_t)(tmp - MINCHECK) * 1000 / (2000 - MINCHECK); // [MINCHECK;2000] -> [0;1000]
  tmp2 = tmp / 100;
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100; // [0;1000] -> expo -> [conf.minthrottle;MAXTHROTTLE]

  if (f.HEADFREE_MODE) { //to optimize
    float radDiff = (heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
  }

  // 안 쓰임
#if defined(POWERMETER_HARD)
  uint16_t pMeterRaw;               // used for current reading
  static uint16_t psensorTimer = 0;
  if (! (++psensorTimer % PSENSORFREQ)) {
    pMeterRaw =  analogRead(PSENSORPIN);
    //lcdprint_int16(pMeterRaw); LCDcrlf();
    powerValue = ( conf.psensornull > pMeterRaw ? conf.psensornull - pMeterRaw : pMeterRaw - conf.psensornull); // do not use abs(), it would induce implicit cast to uint and overrun
    if ( powerValue < 333) {  // only accept reasonable values. 333 is empirical
#ifdef LCD_TELEMETRY
      if (powerValue > powerMax) powerMax = powerValue;
#endif
    } else {
      powerValue = 333;
    }
    pMeter[PMOTOR_SUM] += (uint32_t) powerValue;
  }
#endif

  // 안 쓰임
#if defined(BUZZER)
#if defined(VBAT)
  static uint8_t vbatTimer = 0;
  static uint8_t ind = 0;
  uint16_t vbatRaw = 0;
  static uint16_t vbatRawArray[8];
  if (! (++vbatTimer % VBATFREQ)) {
    vbatRawArray[(ind++) % 8] = analogRead(V_BATPIN);
    for (uint8_t i = 0; i < 8; i++) vbatRaw += vbatRawArray[i];
    vbat = (vbatRaw * 2) / conf.vbatscale; // result is Vbatt in 0.1V steps
  }
#endif
  alarmHandler(); // external buzzer routine that handles buzzer events globally now
#endif

  // 안 쓰임
#if defined(RX_RSSI)
  static uint8_t sig = 0;
  uint16_t rssiRaw = 0;
  static uint16_t rssiRawArray[8];
  rssiRawArray[(sig++) % 8] = analogRead(RX_RSSI_PIN);
  for (uint8_t i = 0; i < 8; i++) rssiRaw += rssiRawArray[i];
  rssi = rssiRaw / 8;
#endif

  if ( (calibratingA > 0 && ACC ) || (calibratingG > 0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {
      LEDPIN_OFF;
    }
    if (f.ARMED) {
      LEDPIN_ON;
    }
  }

  // 안 쓰임
#if defined(LED_RING)
  static uint32_t LEDTime;
  if ( currentTime > LEDTime ) {
    LEDTime = currentTime + 50000;
    i2CLedRingState();
  }
#endif

  // 안 쓰임
#if defined(LED_FLASHER)
  auto_switch_led_flasher();
#endif

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  //
#if !(defined(SPEKTRUM) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if Spektrum Sat in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
#if defined(GPS_PROMINI)
  if (GPS_Enable == 0) {
    serialCom();
  }
#else
  serialCom();
#endif
#endif

  //안 쓰임
#if defined(POWERMETER)
  intPowerMeterSum = (pMeter[PMOTOR_SUM] / conf.pleveldiv);
  intPowerTrigger1 = conf.powerTrigger1 * PLEVELSCALE;
#endif

  // 안 쓰임
#ifdef LCD_TELEMETRY_AUTO
  static char telemetryAutoSequence []  = LCD_TELEMETRY_AUTO;
  static uint8_t telemetryAutoIndex = 0;
  static uint16_t telemetryAutoTimer = 0;
  if ( (telemetry_auto) && (! (++telemetryAutoTimer % LCD_TELEMETRY_AUTO_FREQ) )  ) {
    telemetry = telemetryAutoSequence[++telemetryAutoIndex % strlen(telemetryAutoSequence)];
    LCDclear(); // make sure to clear away remnants
  }
#endif

  // 안 쓰임
#ifdef LCD_TELEMETRY
  static uint16_t telemetryTimer = 0;
  if (! (++telemetryTimer % LCD_TELEMETRY_FREQ)) {
#if (LCD_TELEMETRY_DEBUG+0 > 0)
    telemetry = LCD_TELEMETRY_DEBUG;
#endif
    if (telemetry) lcd_telemetry();
  }
#endif


#if GPS & defined(GPS_LED_INDICATOR)       // modified by MIS to use STABLEPIN LED for number of sattelites indication
  static uint32_t GPSLEDTime;              // - No GPS FIX -> LED blink at speed of incoming GPS frames
  static uint8_t blcnt;                    // - Fix and sat no. bellow 5 -> LED off
  if (currentTime > GPSLEDTime) {          // - Fix and sat no. >= 5 -> LED blinks, one blink for 5 sat, two blinks for 6 sat, three for 7 ...
    if (f.GPS_FIX && GPS_numSat >= 5) {
      if (++blcnt > 2 * GPS_numSat) blcnt = 0;
      GPSLEDTime = currentTime + 150000;
      if (blcnt >= 10 && ((blcnt % 2) == 0)) {
        STABLEPIN_ON;
      } else {
        STABLEPIN_OFF;
      }
    } else {
      if ((GPS_update == 1) && !f.GPS_FIX) {
        STABLEPIN_ON;
      } else {
        STABLEPIN_OFF;
      }
      blcnt = 0;
    }
  }
#endif

  // 안 쓰임
#if defined(LOG_VALUES) && (LOG_VALUES >= 2)
  if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
  if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
#endif

  if (f.ARMED)  {
#if defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
    armedTime += (uint32_t)cycleTime;
#endif
#if defined(VBAT)
    if ( (vbat > conf.no_vbat) && (vbat < vbatMin) ) vbatMin = vbat;
#endif
#ifdef LCD_TELEMETRY
#if BARO
    if ( (BaroAlt > BAROaltMax) ) BAROaltMax = BaroAlt;
#endif
#endif
  }
}

void setup() {
#if !defined(GPS_PROMINI)
  SerialOpen(0, SERIAL0_COM_SPEED);
#if defined(PROMICRO)
  SerialOpen(1, SERIAL1_COM_SPEED);
#endif
#endif

  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
}

void go_arm() {
  if (calibratingG == 0 && f.ACC_CALIBRATED
#if defined(FAILSAFE)
      // && failsafeCnt < 2
#endif
     ) {
    if (!f.ARMED) { // arm now!
      f.ARMED = 1;
      headFreeModeHold = heading;
#if defined(VBAT)
      if (vbat > conf.no_vbat) vbatMin = vbat;
#endif
#ifdef LCD_TELEMETRY // reset some values when arming
#if BARO
      BAROaltMax = BaroAlt;
#endif
#endif
#ifdef LOG_PERMANENT
      plog.arm++;           // #arm events
      plog.running = 1;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
      // write now.
      writePLog();
#endif
    }
  } else if (!f.ARMED) {
    blinkLED(2, 255, 1);
    alarmArray[8] = 1;
  }
}

void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
#ifdef LOG_PERMANENT
    plog.disarm++;        // #disarm events
    plog.armed_time = armedTime ;   // lifetime in seconds
    if (failsafeEvents) plog.failsafe++;      // #acitve failsafe @ disarm
    if (i2c_errors_count > 10) plog.i2c++;           // #i2c errs @ disarm
    plog.running = 0;       // toggle @ arm & disarm to monitor for clean shutdown vs. powercut
    // write now.
    writePLog();
#endif
  }
}




void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       // this hold sticks position for command combos
  uint8_t axis, i;
  int16_t error, errorAngle;
  int16_t delta, deltaSum;
  int16_t PTerm, ITerm, DTerm;
  int16_t PTermACC = 0 , ITermACC = 0 , PTermGYRO = 0 , ITermGYRO = 0;
  static int16_t lastGyro[3] = {0, 0, 0};
  static int16_t delta1[3], delta2[3];
  static int16_t errorGyroI[3] = {0, 0, 0};
  static int16_t errorAngleI[2] = {0, 0};
  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;

#if defined(SPEKTRUM)
  if (spekFrameFlags == 0x01) readSpektrum();
#endif
#if defined(OPENLRSv2MULTI)
  Read_OpenLRS_RC();
#endif

#if defined(HEX_NANO)
  serialCom();
#endif

  if (currentTime > rcTime ) { // 50Hz
    rcTime = currentTime + 20000;
    computeRC();
    // Failsafe routine - added by MIS
#if defined(HEX_NANO)
    rcData[0] = serialRcValue[0];
    rcData[1] = serialRcValue[1];
    rcData[2] = serialRcValue[2];
    rcData[3] = serialRcValue[3];
    rcData[4] = serialRcValue[4];
    rcData[5] = serialRcValue[5];
    rcData[6] = serialRcValue[6];
    rcData[7] = serialRcValue[7];
#endif

#if defined(FAILSAFE)
    if ( failsafeCnt > (5 * FAILSAFE_DELAY) && f.ARMED) {                // Stabilize, and set Throttle to specified level
      for (i = 0; i < 3; i++) rcData[i] = MIDRC;                          // after specified guard time after RC signal is lost (in 0.1sec)

      if (rcData[THROTTLE] < conf.failsafe_throttle) {
        rcData[THROTTLE] = MINTHROTTLE;
      }
      else {
        rcData[THROTTLE] = conf.failsafe_throttle;
      }

      if (failsafeCnt > 5 * (FAILSAFE_DELAY + FAILSAFE_OFF_DELAY)) {      // Turn OFF motors after specified Time (in 0.1sec)
        go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
        f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
      }
      failsafeEvents++;
    }
    if ( failsafeCnt > (5 * FAILSAFE_DELAY) && !f.ARMED) { //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
      go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
      f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
    }
    failsafeCnt++;
#endif
    // end of failsafe routine - next change is made with RcOptions setting

    // ------------------ STICKS COMMAND HANDLER --------------------
    // checking sticks positions
    uint8_t stTmp = 0;
    for (i = 0; i < 4; i++) {
      stTmp >>= 2;
      if (rcData[i] > MINCHECK) stTmp |= 0x80;     // check for MIN
      if (rcData[i] < MAXCHECK) stTmp |= 0x40;     // check for MAX
    }
    if (stTmp == rcSticks) {
      if (rcDelayCommand < 250) rcDelayCommand++;
    } else rcDelayCommand = 0;
    rcSticks = stTmp;

    // perform actions
    if (rcData[THROTTLE] <= MINCHECK) {            // THROTTLE at minimum
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      if (conf.activate[BOXARM] > 0) {             // Arming/Disarming via ARM BOX
        if ( rcOptions[BOXARM] && f.OK_TO_ARM ) go_arm(); else if (f.ARMED) go_disarm();
      }
    }
    if (rcDelayCommand == 20) {
      if (f.ARMED) {                  // actions during armed
#ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) go_disarm();    // Disarm via YAW
#endif
#ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
        if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO) go_disarm();    // Disarm via ROLL
#endif
      } else {                        // actions during not armed
        i = 0;
        if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {    // GYRO calibration
          calibratingG = 512;
#if GPS
          GPS_reset_home_position();
#endif
#if BARO
          calibratingB = 10; // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
#endif
        }
#if defined(INFLIGHT_ACC_CALIBRATION)
        else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI) {    // Inflight ACC calibration START/STOP
          if (AccInflightCalibrationMeasurementDone) {               // trigger saving into eeprom after landing
            AccInflightCalibrationMeasurementDone = 0;
            AccInflightCalibrationSavetoEEProm = 1;
          } else {
            AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
#if defined(BUZZER)
            if (AccInflightCalibrationArmed) alarmArray[0] = 2; else   alarmArray[0] = 3;
#endif
          }
        }
#endif
#ifdef MULTIPLE_CONFIGURATION_PROFILES
        if      (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO) i = 1;  // ROLL left  -> Profile 1
        else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE) i = 2;  // PITCH up   -> Profile 2
        else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI) i = 3;  // ROLL right -> Profile 3
        if (i) {
          global_conf.currentSet = i - 1;
          writeGlobalSet(0);
          readEEPROM();
          blinkLED(2, 40, i);
          alarmArray[0] = i;
        }
#endif
        if (rcSticks == THR_LO + YAW_HI + PIT_HI + ROL_CE) {            // Enter LCD config
#if defined(LCD_CONF)
          configurationLoop(); // beginning LCD configuration
#endif
          previousTime = micros();
        }
#ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) go_arm();      // Arm via YAW
#endif
#ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
        else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI) go_arm();      // Arm via ROLL
#endif
#ifdef LCD_TELEMETRY_AUTO
        else if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) {              // Auto telemetry ON/OFF
          if (telemetry_auto) {
            telemetry_auto = 0;
            telemetry = 0;
          } else
            telemetry_auto = 1;
        }
#endif
#ifdef LCD_TELEMETRY_STEP
        else if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) {              // Telemetry next step
          telemetry = telemetryStepSequence[++telemetryStepIndex % strlen(telemetryStepSequence)];
#ifdef OLED_I2C_128x64
          if (telemetry != 0) i2c_OLED_init();
#endif
          LCDclear();
        }
#endif
        else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) calibratingA = 512;   // throttle=max, yaw=left, pitch=min
        else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) f.CALIBRATE_MAG = 1;  // throttle=max, yaw=right, pitch=min
        i = 0;
        if      (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
          conf.angleTrim[PITCH] += 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
          conf.angleTrim[PITCH] -= 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
          conf.angleTrim[ROLL] += 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
          conf.angleTrim[ROLL] -= 2;
          i = 1;
        }
        if (i) {
          writeParams(1);
          rcDelayCommand = 0;    // allow autorepetition
#if defined(LED_RING)
          blinkLedRing();
#endif
        }
      }
    }
#if defined(LED_FLASHER)
    led_flasher_autoselect_sequence();
#endif

#if defined(INFLIGHT_ACC_CALIBRATION)
    if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > MINCHECK && !rcOptions[BOXARM] ) { // Copter is airborne and you are turning it off via boxarm : start measurement
      InflightcalibratingA = 50;
      AccInflightCalibrationArmed = 0;
    }
    if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
      if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone) {
        InflightcalibratingA = 50;
      }
    } else if (AccInflightCalibrationMeasurementDone && !f.ARMED) {
      AccInflightCalibrationMeasurementDone = 0;
      AccInflightCalibrationSavetoEEProm = 1;
    }
#endif

    uint16_t auxState = 0;
    for (i = 0; i < 4; i++)
      auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
    for (i = 0; i < CHECKBOXITEMS; i++)
      rcOptions[i] = (auxState & conf.activate[i]) > 0;

    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAFE_DELAY is always false
#if ACC
    if ( rcOptions[BOXANGLE] || (failsafeCnt > 5 * FAILSAFE_DELAY) ) {
      // bumpless transfer to Level mode
      if (!f.ANGLE_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.ANGLE_MODE = 1;
      }
    } else {
      // failsafe support
      f.ANGLE_MODE = 0;
    }
    if ( rcOptions[BOXHORIZON] ) {
      f.ANGLE_MODE = 0;
      if (!f.HORIZON_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.HORIZON_MODE = 1;
      }
    } else {
      f.HORIZON_MODE = 0;
    }
#endif

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
#if !defined(GPS_LED_INDICATOR)
    if (f.ANGLE_MODE || f.HORIZON_MODE) {
      STABLEPIN_ON;
    } else {
      STABLEPIN_OFF;
    }
#endif

#if BARO
#if (!defined(SUPPRESS_BARO_ALTHOLD))
    if (rcOptions[BOXBARO]) {
      if (!f.BARO_MODE) {
        f.BARO_MODE = 1;
        AltHold = EstAlt;
        initialThrottleHold = rcCommand[THROTTLE];
        errorAltitudeI = 0;
        BaroPID = 0;
      }
    } else {
      f.BARO_MODE = 0;
    }
#endif
#ifdef VARIOMETER
    if (rcOptions[BOXVARIO]) {
      if (!f.VARIO_MODE) {
        f.VARIO_MODE = 1;
      }
    } else {
      f.VARIO_MODE = 0;
    }
#endif
#endif
#if MAG
    if (rcOptions[BOXMAG]) {
      if (!f.MAG_MODE) {
        f.MAG_MODE = 1;
        magHold = heading;
      }
    } else {
      f.MAG_MODE = 0;
    }
    if (rcOptions[BOXHEADFREE]) {
      if (!f.HEADFREE_MODE) {
        f.HEADFREE_MODE = 1;
      }
    } else {
      f.HEADFREE_MODE = 0;
    }
    if (rcOptions[BOXHEADADJ]) {
      headFreeModeHold = heading; // acquire new heading
    }
#endif

#if GPS
    static uint8_t GPSNavReset = 1;
    if (f.GPS_FIX && GPS_numSat >= 5 ) {
      if (rcOptions[BOXGPSHOME]) {  // if both GPS_HOME & GPS_HOLD are checked => GPS_HOME is the priority
        if (!f.GPS_HOME_MODE)  {
          f.GPS_HOME_MODE = 1;
          f.GPS_HOLD_MODE = 0;
          GPSNavReset = 0;
#if defined(I2C_GPS)
          GPS_I2C_command(I2C_GPS_COMMAND_START_NAV, 0);       //waypoint zero
#else // SERIAL
          GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
          nav_mode    = NAV_MODE_WP;
#endif
        }
      } else {
        f.GPS_HOME_MODE = 0;
        if (rcOptions[BOXGPSHOLD] && abs(rcCommand[ROLL]) < AP_MODE && abs(rcCommand[PITCH]) < AP_MODE) {
          if (!f.GPS_HOLD_MODE) {
            f.GPS_HOLD_MODE = 1;
            GPSNavReset = 0;
#if defined(I2C_GPS)
            GPS_I2C_command(I2C_GPS_COMMAND_POSHOLD, 0);
#else
            GPS_hold[LAT] = GPS_coord[LAT];
            GPS_hold[LON] = GPS_coord[LON];
            GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
            nav_mode = NAV_MODE_POSHOLD;
#endif
          }
        } else {
          f.GPS_HOLD_MODE = 0;
          // both boxes are unselected here, nav is reset if not already done
          if (GPSNavReset == 0 ) {
            GPSNavReset = 1;
            GPS_reset_nav();
          }
        }
      }
    } else {
      f.GPS_HOME_MODE = 0;
      f.GPS_HOLD_MODE = 0;
#if !defined(I2C_GPS)
      nav_mode = NAV_MODE_NONE;
#endif
    }
#endif

#if defined(FIXEDWING) || defined(HELICOPTER)
    if (rcOptions[BOXPASSTHRU]) {
      f.PASSTHRU_MODE = 1;
    }
    else {
      f.PASSTHRU_MODE = 0;
    }
#endif

  } else { // not in rc loop
    static uint8_t taskOrder = 0; // never call all functions in the same loop, to avoid high delay spikes
    if (taskOrder > 4) taskOrder -= 5;
    switch (taskOrder) {
      case 0:
        taskOrder++;
#if MAG
        if (Mag_getADC()) break; // max 350 µs (HMC5883) // only break when we actually did something
#endif
      case 1:
        taskOrder++;
#if BARO
        if (Baro_update() != 0 ) break;
#endif
      case 2:
        taskOrder++;
#if BARO
        if (getEstimatedAltitude() != 0) break;
#endif
      case 3:
        taskOrder++;
#if GPS
        if (GPS_Enable) GPS_NewData();
        break;
#endif
      case 4:
        taskOrder++;
#if SONAR
        Sonar_update(); debug[2] = sonarAlt;
#endif
#ifdef LANDING_LIGHTS_DDR
        auto_switch_landing_lights();
#endif
#ifdef VARIOMETER
        if (f.VARIO_MODE) vario_signaling();
#endif
        break;
    }
  }

  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

#ifdef CYCLETIME_FIXATED
  if (conf.cycletime_fixated) {
    if ((micros() - timestamp_fixated) > conf.cycletime_fixated) {
    } else {
      while ((micros() - timestamp_fixated) < conf.cycletime_fixated) ; // waste away
    }
    timestamp_fixated = micros();
  }
#endif
  //***********************************
  //**** Experimental FlightModes *****
  //***********************************
#if defined(ACROTRAINER_MODE)
  if (f.ANGLE_MODE) {
    if (abs(rcCommand[ROLL]) + abs(rcCommand[PITCH]) >= ACROTRAINER_MODE ) {
      f.ANGLE_MODE = 0;
      f.HORIZON_MODE = 0;
      f.MAG_MODE = 0;
      f.BARO_MODE = 0;
      f.GPS_HOME_MODE = 0;
      f.GPS_HOLD_MODE = 0;
    }
  }
#endif

  //***********************************

#if MAG
  if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) {
    int16_t dif = heading - magHold;
    if (dif <= - 180) dif += 360;
    if (dif >= + 180) dif -= 360;
    if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif * conf.P8[PIDMAG] >> 5;
  } else magHold = heading;
#endif

#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  if (f.BARO_MODE) {
    static uint8_t isAltHoldChanged = 0;
#if defined(ALTHOLD_FAST_THROTTLE_CHANGE)
    if (abs(rcCommand[THROTTLE] - initialThrottleHold) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
      //errorAltitudeI = 0;
      isAltHoldChanged = 1;
      rcCommand[THROTTLE] += (rcCommand[THROTTLE] > initialThrottleHold) ? -ALT_HOLD_THROTTLE_NEUTRAL_ZONE : ALT_HOLD_THROTTLE_NEUTRAL_ZONE;
      // initialThrottleHold += (rcCommand[THROTTLE] > initialThrottleHold) ? -ALT_HOLD_THROTTLE_NEUTRAL_ZONE : ALT_HOLD_THROTTLE_NEUTRAL_ZONE;; //++hex nano
    } else {
      if (isAltHoldChanged) {
        AltHold = EstAlt;
        isAltHoldChanged = 0;
      }
      rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
    }
#else
    static int16_t AltHoldCorr = 0;
    if (abs(rcCommand[THROTTLE] - initialThrottleHold) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
      // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
      AltHoldCorr += rcCommand[THROTTLE] - initialThrottleHold;
      if (abs(AltHoldCorr) > 500) {
        AltHold += AltHoldCorr / 500;
        AltHoldCorr %= 500;
      }
      //errorAltitudeI = 0;
      isAltHoldChanged = 1;
    } else if (isAltHoldChanged) {
      AltHold = EstAlt;
      isAltHoldChanged = 0;
    }
    rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
#endif
  }
#endif
#if GPS
  if ( (f.GPS_HOME_MODE || f.GPS_HOLD_MODE) && f.GPS_FIX_HOME ) {
    float sin_yaw_y = sin(heading * 0.0174532925f);
    float cos_yaw_x = cos(heading * 0.0174532925f);
#if defined(NAV_SLEW_RATE)
    nav_rated[LON]   += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -NAV_SLEW_RATE, NAV_SLEW_RATE);
    nav_rated[LAT]   += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -NAV_SLEW_RATE, NAV_SLEW_RATE);
    GPS_angle[ROLL]   = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
    GPS_angle[PITCH]  = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
#else
    GPS_angle[ROLL]   = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
    GPS_angle[PITCH]  = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
#endif
  } else {
    GPS_angle[ROLL]  = 0;
    GPS_angle[PITCH] = 0;
  }
#endif

  //**** PITCH & ROLL & YAW PID ****
  int16_t prop;
  prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500); // range [0;500]

  for (axis = 0; axis < 3; axis++) {
    if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2 ) { // MODE relying on ACC
      // 50 degrees max inclination
      errorAngle = constrain((rcCommand[axis] << 1) + GPS_angle[axis], -500, +500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
      PTermACC = ((int32_t)errorAngle * conf.P8[PIDLEVEL]) >> 7;                      // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
      PTermACC = constrain(PTermACC, -conf.D8[PIDLEVEL] * 5, +conf.D8[PIDLEVEL] * 5);

      errorAngleI[axis]     = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp     //16 bits is ok here
      ITermACC              = ((int32_t)errorAngleI[axis] * conf.I8[PIDLEVEL]) >> 12;        // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    }
    if ( !f.ANGLE_MODE || f.HORIZON_MODE || axis == 2 ) { // MODE relying on GYRO or YAW axis
      if (abs(rcCommand[axis]) < 500) error =          (rcCommand[axis] << 6) / conf.P8[axis] ; // 16 bits is needed for calculation: 500*64 = 32000      16 bits is ok for result if P8>5 (P>0.5)
      else error = ((int32_t)rcCommand[axis] << 6) / conf.P8[axis] ; // 32 bits is needed for calculation

      error -= gyroData[axis];

      PTermGYRO = rcCommand[axis];

      errorGyroI[axis]  = constrain(errorGyroI[axis] + error, -16000, +16000);     // WindUp   16 bits is ok here
      if (abs(gyroData[axis]) > 640) errorGyroI[axis] = 0;
      ITermGYRO = ((errorGyroI[axis] >> 7) * conf.I8[axis]) >> 6;                  // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
    }
    if ( f.HORIZON_MODE && axis < 2) {
      PTerm = ((int32_t)PTermACC * (512 - prop) + (int32_t)PTermGYRO * prop) >> 9; // the real factor should be 500, but 512 is ok
      ITerm = ((int32_t)ITermACC * (512 - prop) + (int32_t)ITermGYRO * prop) >> 9;
    } else {
      if ( f.ANGLE_MODE && axis < 2) {
        PTerm = PTermACC;
        ITerm = ITermACC;
      } else {
        PTerm = PTermGYRO;
        ITerm = ITermGYRO;
      }
    }

    PTerm -= ((int32_t)gyroData[axis] * dynP8[axis]) >> 6; // 32 bits is needed for calculation

    delta          = gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = gyroData[axis];
    deltaSum       = delta1[axis] + delta2[axis] + delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;

    DTerm = ((int32_t)deltaSum * dynD8[axis]) >> 5;    // 32 bits is needed for calculation

    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

  mixTable();
  writeServos();
  writeMotors();
}

