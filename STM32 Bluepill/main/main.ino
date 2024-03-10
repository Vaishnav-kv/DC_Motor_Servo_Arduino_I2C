//HW-INFO//
//STM32F103C8T6 - Blue Pill//
//HEADERS//
#include <Wire.h>
#include <AS5047P.h>
//PIN DEFINITIONS//
#define LED_PIN LED_BUILTIN  //PC13//
#define HB_PWM_EN PA8        // is pwm capable //pin is 5v tolerant
//HardwareTimer timer(1);
#define IN1 PA14             // is not pwm capable  //pin is 5v tolerant
#define IN2 PA15             // is not pwm capable  //pin is 5v tolerant
/////////MAG-ENCODER_SETUP////////
#define AS5047P_CHIP_SELECT_PORT PA4  //pin is not 5v tolerant
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
// SCLK-PA5,MISO-PA6,MOSI-PA7
//////////////////////////////////
/////////VARIABLES////////
float target;  // idk if it need to be refreshed to "0"
long prevT = 0;
float eprev = 0;
float eintegral = 0;
//////////////////////////
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(HB_PWM_EN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.begin(9600);
/////////I2C_SETUP////////
#define I2C_DEV_ADDR 0x55  //id as per requirment
  Wire.begin();
  //SDA-PB7,SCL-PB6

  // initialize the AS5047P sensor and hold if sensor can't be initialized.
  while (!as5047p.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(1000);
  }
  //////////////////////////
}

void loop() {

  // set target position
  Wire.onReceive(receiveEvent);  // register event,target is recived via i2c
  //int target = 1200;  // manually set target
  //int target = 250 * sin(prevT / 1e6);
  Serial.println(target);
  // PID constants
  float kp = 1;
  float kd = 1;
  float ki = 1;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position
  int pos;         // idk if it need to be refreshed to "0"
  noInterrupts();  // disable interrupts temporarily while reading
  pos = as5047p.readAngleDegree();
  Serial.println(pos);
  interrupts();  // turn interrupts back on

  // error
  int e = pos - target;

  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;


  // motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, HB_PWM_EN, IN1, IN2);


  // store previous error
  eprev = e;
}
