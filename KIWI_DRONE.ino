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




void loop() {

}

