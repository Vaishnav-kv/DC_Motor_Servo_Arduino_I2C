//HW-ESP32//
#include <analogWrite.h>
/////////MAG-ENCODER_SETUP////////
#include <AS5047P.h>
#define LED_PIN 2
#define AS5047P_CHIP_SELECT_PORT 2
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
//////////////////////////////////
/////////H-BRIDGE_SETUP?////////
const int PWM = 4;
const int IN1 = 16;
const int IN2 = 17;
////////////////////////////////
/////////I2C_SETUP////////
#include <Wire.h>
#define I2C_DEV_ADDR 0x55
//////////////////////////
/////////VARIABLES////////
long prevT = 0;
float eprev = 0;
float eintegral = 0;
//////////////////////////
void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Wire.begin();

  // initialize the AS5047P sensor and hold if sensor can't be initialized.
  while (!as5047p.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(1000);
  }
}

void loop() {

  // set target position
  //int target = 1200;
  int target = 250 * sin(prevT / 1e6);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = as5047p.readAngleDegree();
  interrupts(); // turn interrupts back on

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
  if ( pwr > 255 ) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, PWM, IN1, IN2);


  // store previous error
  eprev = e;

}
