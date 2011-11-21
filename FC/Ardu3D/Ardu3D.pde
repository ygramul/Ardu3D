#define IDLE_THROTTLE 4
#define MAX_THROTTLE 250
#define PCB_DIRECTION 315 // Orientation of the flight control board in degrees (should be multible of 45)

#define PITCH_STICK_MIN 32
#define PITCH_STICK_MAX 240
#define ROLL_STICK_MIN 29
#define ROLL_STICK_MAX 233
#define YAW_STICK_MIN 26
#define YAW_STICK_MAX 233
#define THROTTLE_STICK_MIN 28
#define THROTTLE_STICK_MAX 233

#include <I2C.h>
#include <ITG3200.h>
#include <PID.h>
#include <MCMixer.h>
#include <RCTransform.h>
#include <EEPROM.h>
#include <Shell.h>
//#include <RcSuSiReceiver.h>
#include <RcReceiver.h>
#include <Kalman.h>

typedef union
{
  struct
  {
    int16_t para16;
  };
  struct
  {
    uint8_t low_param8;
    uint8_t hi_param8;
  };
} long_short_param_t;

Shell shell;
ITG3200 itg3200;
PID rollPid(1, 256);
PID pitchPid(1, 256);
PID yawPid(1, 256);
Kalman rollKalman;
Kalman pitchKalman;
Kalman yawKalman;
MCMixer mixer;
RCTransform rcTransform;
//RcSuSiReceiver receiver(RC_ROLL,RC_PITCH,RC_THROTTLE,RC_YAW,RC_AUX1,RC_AUX2,RC_CAMPITCH,RC_CAMROLL);
RcReceiver receiver;

static uint8_t config_version = 001;

static long_short_param_t pitch_prop;
static long_short_param_t pitch_int;
static long_short_param_t pitch_diff;
static long_short_param_t roll_prop;
static long_short_param_t roll_int;
static long_short_param_t roll_diff;
static long_short_param_t yaw_prop;
static long_short_param_t yaw_int;
static long_short_param_t yaw_diff;

static uint32_t currentTime;
static uint32_t blctrlTime;
static uint32_t serialTime;
static uint32_t currUsedTime;
static uint32_t usedTime;

static int16_t roll_filtered;
static int16_t roll_action;
static int16_t pitch_filtered;
static int16_t pitch_action;
static int16_t yaw_filtered;
static int16_t yaw_action;

static int16_t stick_throttle; // 28 down - 233 up
static int16_t stick_roll;     // -96 left - 108 right
static int16_t stick_pitch;    // -91 down - 115 up
static int16_t stick_yaw;      // -99 left - 108 right
static int8_t armed;          // "G" switch on RC
static int16_t transformed_roll;
static int16_t transformed_pitch;

#define POWERPIN_PINMODE           pinMode (12, OUTPUT);
#define POWERPIN_ON                PORTB |= 1<<4;
#define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12

// *************************
// motor and servo functions
// *************************

/*********** BL-Ctrl I2C Adresses *******/
uint8_t BLCTRL_I2C[4] = {
  0x58, 0x54, 0x52, 0x56};

static int16_t i2c_motor[4] = {
  0, 0, 0, 0};

void writeMotors() { // [1000;2000] => [125;250]
    for(uint8_t i=0;i<4;i++) {   // [1000;2000] => [0;255]
      i2c_rep_start(BLCTRL_I2C[i]); // Send command to Ctrl 
      i2c_write(i2c_motor[i]);   // Set speed of Ctrl 
    }
}

/*********** Pitch servo *******/
uint8_t pitch_pin[4] = {9,10,11,3};

static uint8_t pitch_value[4];

void set_all_pitch_servos() { 
    for(uint8_t i=0;i<4;i++) { 
      analogWrite(pitch_pin[i], pitch_value[i]);
    }
}

// *************************
// Persistence functions
// *************************

void readParameters() {
  uint8_t i,p=1;
  pitch_prop.low_param8 = EEPROM.read(p++);pitch_prop.hi_param8 = EEPROM.read(p++);
  pitch_int.low_param8 = EEPROM.read(p++);pitch_int.hi_param8 = EEPROM.read(p++);
  pitch_diff.low_param8 = EEPROM.read(p++);pitch_diff.hi_param8 = EEPROM.read(p++);
  roll_prop.low_param8 = EEPROM.read(p++);roll_prop.hi_param8 = EEPROM.read(p++);
  roll_int.low_param8 = EEPROM.read(p++);roll_int.hi_param8 = EEPROM.read(p++);
  roll_diff.low_param8 = EEPROM.read(p++);roll_diff.hi_param8 = EEPROM.read(p++);
  yaw_prop.low_param8 = EEPROM.read(p++);yaw_prop.hi_param8 = EEPROM.read(p++);
  yaw_int.low_param8 = EEPROM.read(p++);yaw_int.hi_param8 = EEPROM.read(p++);
  yaw_diff.low_param8 = EEPROM.read(p++);yaw_diff.hi_param8 = EEPROM.read(p++);
  for(uint8_t i=0;i<4;i++)
    pitch_value[i] = EEPROM.read(p++);
}

void writeParameter() {
  uint8_t i,p=1;
  EEPROM.write(0, config_version);
  EEPROM.write(p++,pitch_prop.low_param8);EEPROM.write(p++,pitch_prop.hi_param8);
  EEPROM.write(p++,pitch_int.low_param8);EEPROM.write(p++,pitch_int.hi_param8);
  EEPROM.write(p++,pitch_diff.low_param8);EEPROM.write(p++,pitch_diff.hi_param8);
  EEPROM.write(p++,roll_prop.low_param8);EEPROM.write(p++,roll_prop.hi_param8);
  EEPROM.write(p++,roll_int.low_param8);EEPROM.write(p++,roll_int.hi_param8);
  EEPROM.write(p++,roll_diff.low_param8);EEPROM.write(p++,roll_diff.hi_param8);
  EEPROM.write(p++,yaw_prop.low_param8);EEPROM.write(p++,yaw_prop.hi_param8);
  EEPROM.write(p++,yaw_int.low_param8);EEPROM.write(p++,yaw_int.hi_param8);
  EEPROM.write(p++,yaw_diff.low_param8);EEPROM.write(p++,yaw_diff.hi_param8);
  for(uint8_t i=0;i<4;i++)
    EEPROM.write(p++, pitch_value[i]);
  shell.stopActiveFunction();
  blinkLED(15,20,1);
}

void checkFirstTime() {
  if ( EEPROM.read(0) != config_version ) {
    pitch_prop.para16 = 60;
    pitch_int.para16 = 0;
    pitch_diff.para16 = 50;
    roll_prop.para16 = 60;
    roll_int.para16 = 0;
    roll_diff.para16 = 50;
    yaw_prop.para16 = 40;
    yaw_int.para16 = 0;
    yaw_diff.para16 = 30;
    for(uint8_t i=0;i<4;i++)
      pitch_value[i] = 127;
    writeParameter();
  }
}

void computeRC() {
    stick_throttle = map(receiver.readScaledRC(RC_THROTTLE), THROTTLE_STICK_MIN, THROTTLE_STICK_MAX, 0, 255);
    stick_roll     = map(receiver.readScaledRC(RC_ROLL), ROLL_STICK_MIN, ROLL_STICK_MAX, -127, 127);
    stick_pitch    = map(receiver.readScaledRC(RC_PITCH), PITCH_STICK_MIN, PITCH_STICK_MAX, -127, 127);
    stick_yaw      = map(receiver.readScaledRC(RC_YAW), YAW_STICK_MIN, YAW_STICK_MAX, -127, 127);
    if (receiver.isSignalAvailable()) {
      armed = receiver.readScaledRC(RC_AUX1) > 125 ? 1 : 0;
    }
}

void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
      PORTB |= 1;
      delay(wait);
      PORTB &= ~1;
    }
    delay(60);
  }
}

// *************************
// Startup / Initialisation
// *************************

void initShell() {
  shell.addCommand("rp", "pid proportional of roll", &roll_prop.para16);
  shell.addCommand("ri", "pid integrational of roll", &roll_int.para16);
  shell.addCommand("rd", "pid differential of roll", &roll_diff.para16);
  shell.addCommand("pp", "pid proportional of pitch", &pitch_prop.para16);
  shell.addCommand("pi", "pid integrational of pitch", &pitch_int.para16);
  shell.addCommand("pd", "pid differential of pitch", &pitch_diff.para16);
  shell.addCommand("yp", "pid proportional of yaw", &yaw_prop.para16);
  shell.addCommand("yi", "pid integrational of yaw", &yaw_int.para16);
  shell.addCommand("yd", "pid differential of yaw", &yaw_diff.para16);
  
  shell.addCommand("ser1", "pitch value for servo 1", &pitch_value[0]);
  shell.addCommand("ser2", "pitch value for servo 2", &pitch_value[1]);
  shell.addCommand("ser3", "pitch value for servo 3", &pitch_value[2]);
  shell.addCommand("ser4", "pitch value for servo 4", &pitch_value[3]);
  shell.addCommand("armed", "switch on/off motors, be carefull!", &armed);
  shell.setReadwrite();
  shell.addCommand("save", "save parameters", writeParameter);
  shell.addCommand("gyro", "debug gyro sensor values", debugGyro);
  shell.addCommand("rc", "debug scaled rc stick values", debugRC);
  shell.addCommand("rawrc", "debug 'raw' rc stick values", debugRawRC);
  shell.addCommand("motor", "debug motor values", debugMotors);
  shell.addCommand("pid", "debug pid control action", debugPidAction);
}

void setParametersToPid() {
  pitchPid.setProp(pitch_prop.para16);
  pitchPid.setInt(pitch_int.para16);
  pitchPid.setDiff(pitch_diff.para16);
  rollPid.setProp(roll_prop.para16);
  rollPid.setInt(roll_int.para16);
  rollPid.setDiff(roll_diff.para16);
  yawPid.setProp(yaw_prop.para16);
  yawPid.setInt(yaw_int.para16);
  yawPid.setDiff(yaw_diff.para16);
}

void setup() {
  Serial.begin(115200);
  
  pinMode (8, OUTPUT);
  armed = 0;
  
  Serial.println("Init i2c");
  // init wire. i2c_init(internal pullups disabled, 400kHz)
  i2c_init(false, true); 
  delay(100);
  
  // We power the ITG with a pin of the arduino, therefore activate the ITG
  POWERPIN_PINMODE
  POWERPIN_ON
  delay(100);

  Serial.println("Init itg callbacks");
  // make i2c access functions available to sensor methods
  itg3200.i2cStartCallback(i2c_rep_start);
  itg3200.i2cWriteCallback(i2c_write);
  itg3200.i2cAckCallback(i2c_readAck);
  itg3200.i2cNackCallback(i2c_readNak);
  Serial.println("Init itg sensor");
  itg3200.init(); // init sensor. Set working values
  Serial.println("Calibrate gyro");
  // gyro_calibration(); // get deviation of sensor values
  itg3200.calcAdcOffset();
  
  Serial.println("load pid values");
  checkFirstTime();
  readParameters();
  setParametersToPid();
  
  Serial.println("init Kalman filters");
  rollKalman.init1D(0.0625, 4.0, 0.469725, 0.0);
  pitchKalman.init1D(0.0625, 4.0, 0.469725, 0.0);
  yawKalman.init1D(0.0625, 4.0, 0.469725, 0.0);
  
  Serial.println("init motor mixer");
  mixer.setMotorRange(IDLE_THROTTLE, MAX_THROTTLE);
  
  Serial.println("configure receiver");
  stick_throttle = 0;
  stick_roll = 0;
  stick_pitch = 0;
  stick_yaw = 0;
  receiver.configureReceiver();
  
  Serial.println("Init shell");
  initShell();
  
  Serial.println("init done!");
  blinkLED(10,15,2);
}

// *************************
// Main controller loop.
// *************************

void loop() {
  if (currentTime > (blctrlTime + 2000) ) { // 1000000 / 2000 = 500 Hz Refresh rate of i2c motor control
    blctrlTime = currentTime;
    itg3200.readAdc();
    
    // adjust zero and do a low pass on sensor values
    roll_filtered = rollKalman.update1D(itg3200.getRollCalib() / 32);
    pitch_filtered = - pitchKalman.update1D(itg3200.getPitchCalib() / 32);
    yaw_filtered = - yawKalman.update1D(itg3200.getYawCalib() / 32);
    
    pitch_action = pitchPid.calculate(transformed_pitch, pitch_filtered);
    roll_action = rollPid.calculate(transformed_roll, roll_filtered);
    yaw_action = yawPid.calculate(stick_yaw * 6, yaw_filtered);
    
    mixer.mixQuadP(stick_throttle, pitch_action, roll_action, yaw_action);
    
    if (armed) {
      i2c_motor[0] = mixer.getMotor1();
      i2c_motor[1] = mixer.getMotor2();
      i2c_motor[2] = mixer.getMotor3();
      i2c_motor[3] = mixer.getMotor4();
    } else {
      i2c_motor[0] = 0;
      i2c_motor[1] = 0;
      i2c_motor[2] = 0;
      i2c_motor[3] = 0;
    }
    writeMotors();
    set_all_pitch_servos();
  }
  
  if (currentTime > (serialTime + 20000) ) { // 1000000 / 20000 = 50 Hz Refresh rate of serial input
    serialTime = currentTime;
          
    computeRC();
    
    transformed_pitch = rcTransform.linToExp(rcTransform.turnRcPitch(PCB_DIRECTION, stick_pitch, stick_roll), 640);
    transformed_roll = rcTransform.linToExp(rcTransform.turnRcRoll(PCB_DIRECTION, stick_pitch, stick_roll), 640);

    uint8_t c = 0;
    if (Serial.available() > 0) {
      c = Serial.read();
      if (c == 13 || c == 10) {
        Serial.println();
      }
      Serial.print(c);
    }  
    shell.parseCommand(c);
    setParametersToPid();
  }
  currUsedTime = micros() - currentTime;
  if (currUsedTime > usedTime) {
    usedTime = currUsedTime;
  }
  currentTime = micros();
}

// **************************
// debug output functions
// **************************

void debugPidAction() {
    Serial.print("R:");
    Serial.print(roll_action);
    Serial.print(" P:");
    Serial.print(pitch_action);
    Serial.print(" Y:");
    Serial.println(yaw_action);
}

void debugMotors() {
    Serial.print(i2c_motor[0]);
    Serial.print('\t');
    Serial.print(i2c_motor[1]);
    Serial.print('\t');
    Serial.print(i2c_motor[2]);
    Serial.print('\t');
    Serial.println(i2c_motor[3]);
}

void debugGyro() {
    Serial.print("R:");
    Serial.print(roll_filtered);
    Serial.print(" P:");
    Serial.print(pitch_filtered);
    Serial.print(" Y:");
    Serial.println(yaw_filtered);
}

void debugTansformedRC() {
  Serial.print(transformed_pitch);
}

void debugRC() {
    armed ? Serial.print("on") : Serial.print("off");
    Serial.print('\t');
    Serial.print(stick_throttle);
    Serial.print('\t');
    Serial.print(stick_yaw);
    Serial.print('\t');
    Serial.print(stick_pitch);
    Serial.print('\t');
    Serial.println(stick_roll);

}  

void debugRawRC() {
    Serial.print(receiver.readScaledRC(RC_THROTTLE), DEC);
    Serial.print('\t');
    Serial.print(receiver.readScaledRC(RC_ROLL), DEC);
    Serial.print('\t');
    Serial.print(receiver.readScaledRC(RC_PITCH), DEC);
    Serial.print('\t');
    Serial.print(receiver.readScaledRC(RC_YAW), DEC);
    Serial.print('\t');
    Serial.print(receiver.readScaledRC(RC_AUX1) > 125 ? 0 : 1, DEC);
    Serial.println();
}

void showUsedTime() {
    Serial.print("Max used time in main loop: ");
    Serial.println(usedTime);
    Serial.println(" microseconds");
    shell.stopActiveFunction();
}
