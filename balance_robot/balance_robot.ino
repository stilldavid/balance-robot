#include <Wire.h>

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <PID_v1.h>
#include <Encoder.h>

#include <TimedAction.h>

FreeSixIMU sixDOF = FreeSixIMU();

#define SERIAL_PORT_SPEED  115200

/***
MOTOR DRIVER STUFF
***/
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

// imu vars
float imuValues[6];
float ypr[3];
float roll;

// speed filter
const int AVERAGE_COUNT = 20;
float speed_values[AVERAGE_COUNT];
int filter_pos = 0;

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2]  = {2, 3}; // CS: Current sense ANALOG input
int enpin[2]  = {0, 1}; // EN: Status of switches output (Analog pin)

int statpin = 13;

Encoder left(2, 3);
Encoder right(18, 19);

// every 10s to start
int move = 0;

void setup() {

  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i=0; i<2; i++) {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++) {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }

  Serial.begin(SERIAL_PORT_SPEED);
  Serial2.begin(9600);
  Serial.println("BalBot Starting...");
  Wire.begin();

  Serial.println("Initializing IMU...");
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
  Serial.println("running!");
}

float angle, angle_velocity, last_angle;

long r_wheel, r_wheel_velocity, r_last_wheel;
long l_wheel, l_wheel_velocity, l_last_wheel;

long wheel_velocity, wheel;

int moving = 0;

int torque;
int ai, av;
int knob1, knob2, knob3, knob4;
int dk1, dk2, dk3, dk4;

void loop()
{
  if(Serial2.available()) {
    char dir = Serial2.read();
    Serial.println(dir);
    switch(dir) {
      case 'F':
        move++;
        break;
      case 'B':
        move--;
        break;
      case 'S':
        move = 0;
        break;
    }
  }

  sixDOF.getValues(imuValues);  
  sixDOF.getYawPitchRoll(ypr);
  angle = -1 * ypr[2];

  angle_velocity = angle - last_angle;
  last_angle = angle;

  av = angle_velocity * 10;
  ai = angle * 10;

  /*
  wheel = left.read() / 4;
  wheel_velocity = wheel - last_wheel;
  last_wheel = wheel;
  */

  // calculate speed
  l_wheel = left.read();
  r_wheel = right.read();

  l_wheel_velocity = l_wheel - l_last_wheel;
  l_last_wheel = l_wheel;
  r_wheel_velocity = r_wheel - r_last_wheel;
  r_last_wheel = r_wheel;

  float speed = ((l_wheel_velocity + r_wheel_velocity)) / 10;
  wheel += speed;

  wheel_velocity = speed;

  dk1 = analogRead(8) / 10;
  dk2 = analogRead(9) / 10;
  dk3 = analogRead(10) / 10;

  // will be overwritten below
  dk4 = analogRead(11) / 10;

  // if we're moving, dk4 needs to be 0;
  if(0 != move) {
    dk4 = 0;
    moving = true;
  } else {
    if(moving) {
      wheel = 0;
      moving = 0;
    }
    dk4 = 1;
  }

  // the guts of the matter
  torque = (av * dk1) + (ai * dk2) + ((wheel_velocity + move) * dk3) + (wheel * dk4);

  int go = torque / 15;

  if(go > 0) {
    if (go > 1023)
      go = 1023;
    //go = map(go, 0, 100, 0, 1023);
    motorGo(0, CW, go);
    motorGo(1, CW, go * 1.3);
  } else {
    go = abs(go);
    if (go > 1023)
      go = 1023;
    //go = map(go, 0, 100, 0, 1023);
    motorGo(0, CCW, go);
    motorGo(1, CCW, go * 1.3);
  }

  Serial2.print(move);
  Serial2.print(",");
  Serial2.print(dk1);
  Serial2.print(",");
  Serial2.print(dk2);
  Serial2.print(",");
  Serial2.print(dk3);
  Serial2.print(",");
  Serial2.print(dk4);
  Serial2.print(",");
  Serial2.print(ai);
  Serial2.print(",");
  Serial2.print(av);
  Serial2.print(",");
  Serial2.print(wheel);
  Serial2.print(",");
  Serial2.print(wheel_velocity);
  Serial2.print(",");
  Serial2.print(torque);
  Serial2.print(",");
  Serial2.print(go);
  Serial2.println();
  
  delay(40);
}

void motorOff(int motor) {
  // Initialize braked
  for (int i=0; i<2; i++) {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) {
  if (motor <= 1) {
    if (direct <=4) {

      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);

    }
  }
}

void movement() {
  // every 10s
  if(move == 0) {
    move = 2;
  } else if(move > 0) {
    move = -2;
  } else if(move < 0) {
    move = 0;
  }
}
