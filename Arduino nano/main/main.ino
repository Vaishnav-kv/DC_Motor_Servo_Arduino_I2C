//HW-INFO//
//Arduino nano//
//HEADERS//
#include <Wire.h>
#include <AS5047P.h>
//PIN DEFINITIONS//
#define LED_PIN LED_BUILTIN  //13//
#define HB_PWM_EN 3        // is pwm capable //pin is 5v tolerant
#define IN1 8             //  is pwm capable //pin is 5v tolerant
#define IN2 7             //  is pwm capable //pin is 5v tolerant
/////////MAG-ENCODER_SETUP////////
#define AS5047P_CHIP_SELECT_PORT 10  //pin is not 5v tolerant
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
// SCLK-PA5,MISO-PA6,MOSI-PA7
//////////////////////////////////
/////////VARIABLES////////
float target_theta;  // idk if it need to be refreshed to "0"
float pos;         // idk if it need to be refreshed to "0"
long prevT = 0;
float eprev = 0;
float eintegral = 0;
// PID constants
float kp = 3.5;
float kd = 0;
float ki = 1;
//////////////////////////
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(HB_PWM_EN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.begin(9600);
  /////////I2C_SETUP////////
  //#define I2C_DEV_ADDR 8  //id as per requirment
  //Wire.begin(8);
  //Wire.onReceive(receiveEvent);  // register event,target is recived via i2c
  //SDA-PB7,SCL-PB6

  // initialize the AS5047P sensor and hold if sensor can't be initialized.
  while (!as5047p.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(1000);
  }
  //////////////////////////
}

void loop() {

  // Check for incoming serial data
  if (Serial.available() > 0) {
    // Read the incoming value as a string
    String inputString = Serial.readStringUntil('\n');

    // Convert the string to a float
    target_theta = inputString.toFloat();

    // Validate the input (optional)
    if (target_theta < 0.0 || target_theta > 360.0) {
      //Serial.println("Invalid input: Target theta must be between 0 and 360.");
      // Optionally, set target_theta back to a default value here
    } else {
      //Serial.print("Target theta updated to: ");
      //Serial.println(target_theta);
    }
  }
  serial_debug();
  // set target position
  // target_theta = 180;  // manually set target
  target_theta = 180 * sin(prevT / 1e6) + 180;
  // target_theta = fmod(250 * sin(prevT / 1e6) + 250, 360);


  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position
  noInterrupts();  // disable interrupts temporarily while reading
  pos = as5047p.readAngleDegree();
  interrupts();  // turn interrupts back on

  // error
  int e = pos - target_theta;

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

void serial_debug() {
  Serial.print("target_theta: ");
  Serial.print(target_theta);
  Serial.print(", Current_theta: ");
  Serial.println(pos);
  //delay(2000); // Delay between debug updates
}
