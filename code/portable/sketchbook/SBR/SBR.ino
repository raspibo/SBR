/*
 Self Balancing Roboto 
 The full project is in https://github.com/raspibo/SBR.git
  
 
 Documentation for PWM Configuration  http://www.righto.com/2009/07/secrets-of-arduino-pwm.html
*/




#include <Wire.h>   //SDA=A4 SCL=A5
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
//#include <TimerOne.h>
#include "MPU6050.h"



//#define F_Debug	//Uncomment to enable the "fast debug" use for event
#define S_Debug		//Uncomment to enable the schedule debug
#define TDebug		1000


// Arduino Pin Defintion
#define MPUInt	2
#define MotorR	3
#define WDOP	4
#define MotorEN 8
#define MotorL	11
#define LedR	13

#define VBat	A0


//---------LCD I2C Addres and pinout---
#define LCD_I2C_ADDR	0x20
#define BACKLIGHT_PIN	3
#define En_pin			2
#define Rw_pin			1
#define Rs_pin			0
#define D4_pin			4
#define D5_pin			5
#define D6_pin			6
#define D7_pin			7


char PWM_R = 0;
char PWM_L = 0;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

long TimeDeb = 0;

MPU6050 mpu;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);

void setup() {
  pinMode(MPUInt, INPUT);   //INPUT_PULLUP
  pinMode(LedR, OUTPUT);
  pinMode(MotorR, OUTPUT);
  pinMode(MotorL, OUTPUT);
   
  analogReference(DEFAULT);
  
  Serial.begin(115200);

  //Wire.begin();
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  
  
  lcd.begin(16, 2);
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.setCursor(4, 0);
  lcd.print("Msystem");
  lcd.setCursor(6, 1);
  lcd.print("SBR");
  //lcd.clear();
  //lcd.setBacklight(LOW);
  
  
#if defined F_Debug
  Serial.println(F("Initializing I2C devices..."));
#endif  
  mpu.initialize();
  
#if defined F_Debug  
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

// load and configure the DMP
  Serial.println(F("Initializing DMP..."));
#endif
  //devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
 
 // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    
	Serial.println(F("Enabling DMP..."));
    
	mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(MPUInt), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    //packetSize = mpu.dmpGetFIFOPacketSize();
  } 
 #if defined F_Debug   
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  Serial.println(F("Initializing PWM TMR2"));
#endif
 //Set TMR2 for PWM at 16 MHz, used for fan PWM control
  //TCCR1A – Timer/Counter1 Control Register A
  //COM2A1: COM2An: Compare Output Mode for Channel A on non-PWM mode (depend of  WGM2[2:0] bit). Clear OC2A on Compare Match in
  //COM2B1: Compare Output Mode for Channel B. Clear OC2B on Compare Match.
  //WGM20: Waveform Generation Mode for timer2
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  //TCCR2B – Timer/Counter2 Control Register B
  // 1:    = 0x01
  // 8:    = 0x02
  // 32:   = 0x03
  // 64:   = 0x04
  // 128:  = 0x05
  // 256:  = 0x06
  // 1024: = 0x7
  TCCR2B = TCCR2B & 0b11111000 | 0x02; // Prescale 16Mhz/8/256 7,8KHz
  
  PWM_R = PWM_L = 0; 
  OCR2B = map(PWM_R, 0, 100, 0, 255);
  OCR2A = map(PWM_L, 0, 100, 0, 255);
  digitalWrite(MotorEN, LOW);    
      
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


#if defined S_Debug
void Debug() {
 
  
}
#endif

void loop(){
//------------Time Debug Mng---------
#if defined S_Debug
  if (millis() > TimeDeb) {
    TimeDeb = (millis() + TDebug);
    Debug();
  }
#endif	
	
	
	
}
