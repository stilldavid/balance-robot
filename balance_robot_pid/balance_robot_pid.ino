#include <Wire.h>

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <PID_v1.h>
#include <Encoder.h>

//Define Variables we'll be connecting to
double speedSetpoint = 0; // in a perfect world, 0 would be straight up and down
double speedInput;
double speedOutput;

double angleSetpoint = -2.8;
double angleInput;
double angleOutput;

float imuValues[6];
float ypr[3];
float roll;

const int AVERAGE_COUNT = 20;
float speed_values[AVERAGE_COUNT];
int filter_pos = 0;

FreeSixIMU sixDOF = FreeSixIMU();

//Specify the links and initial tuning parameters
PID speedPID(&speedInput, &speedOutput, &speedSetpoint, 0, 0, 0, DIRECT);
PID anglePID(&angleInput, &angleOutput, &angleSetpoint, 0, 0, 0, DIRECT);

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  115200

/**
 * MOTOR DRIVER STUFF
**/
#define BRAKEVCC 0
#define CW  1
#define CCW 2
#define BRAKEGND 3
#define CS_THRESHOLD 500

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

int counter = 0;

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

  for (int i=0; i<AVERAGE_COUNT; i++) {
    speed_values[i] = 0; // fill the array with 0's
  }
  

  Serial.begin(SERIAL_PORT_SPEED);
  Serial2.begin(9600);
  Serial.println("BalBot Starting...");

  Wire.begin();

  Serial.println("Initializing IMU...");
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);

  Serial.println("Initializing PIDs...");
  // turn the PIDs on
  anglePID.SetMode(AUTOMATIC);
  anglePID.SetSampleTime(10);
  anglePID.SetOutputLimits(-100, 100);
  anglePID.SetTunings(25, 10, 0);

  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(10);
  speedPID.SetOutputLimits(-300, 300); // -3 and +3 degrees, respectively
  speedPID.SetTunings(25, 50, 0);

  // test
  motorGo(0, CCW, 1024);
  motorGo(1, CCW, 1024);
  delay(1000);
  motorOff(0);
  motorOff(1);
}

long r_wheel, r_wheel_velocity, r_last_wheel;
long l_wheel, l_wheel_velocity, l_last_wheel;
float angle = 0;

void loop()
{
  // read knobs for angle pid
  double knob0 = analogRead(8);
  double knob1 = analogRead(9);
  double akp = knob0 / 200;
  double aki = knob1 / 200;

  // read knobs for speed pid
  double knob2 = analogRead(10);
  double knob3 = analogRead(11);
  double skp = knob2 / 200;
  double ski = knob3 / 200;

  anglePID.SetTunings(akp, aki, 0);
  speedPID.SetTunings(skp, ski, 0);

  // calculate speed
  l_wheel = left.read();
  r_wheel = right.read();

  l_wheel_velocity = l_wheel - l_last_wheel;
  l_last_wheel = l_wheel;
  r_wheel_velocity = r_wheel - r_last_wheel;
  r_last_wheel = r_wheel;

  int speed = ((l_wheel_velocity + r_wheel_velocity));

  // let's make an average speed for the last N smaples
  int out = 0;
  speed_values[filter_pos] = speed;
  filter_pos = (filter_pos + 1) % AVERAGE_COUNT;
  for (int i=0; i<AVERAGE_COUNT; i++) {
    out += speed_values[i];
  }
  speedInput = (out / AVERAGE_COUNT);

  // calculate angle setpoint
  speedPID.Compute();
  angleSetpoint = speedOutput / 100;
  //angleSetpoint = 1.3;

  sixDOF.getValues(imuValues);  
  sixDOF.getYawPitchRoll(ypr);
  angle = -1 * ypr[2];

  /*
  // panic!
  if(abs(angle) >= 20) {
    stop_robot(5000);
  }
  */

  angleInput = angle;
  anglePID.Compute();

  float go = angleOutput;

  if(go > 0) {
    int speed = map(go, 0, 100, 0, 1023);
    motorGo(0, CCW, speed);
    motorGo(1, CCW, speed * 1.2);
  } else {
    int speed = abs(go);
    speed = map(speed, 0, 100, 0, 1023);
    motorGo(0, CW, speed);
    motorGo(1, CW, speed * 1.2);
  }

  //if(0 == counter % 10) {
    Serial2.print(akp);
    Serial2.print(",");
    Serial2.print(aki);
    Serial2.print(",");
    Serial2.print(skp);
    Serial2.print(",");
    Serial2.print(ski);
    Serial2.print(",");
    Serial2.print(angleInput);
    Serial2.print(",");
    Serial2.print(angleOutput);
    Serial2.print(",");
    Serial2.print(angleSetpoint);
    Serial2.print(",");
    Serial2.print(speedInput);
    Serial2.print(",");
    Serial2.print(speedOutput / 100);
    Serial2.print(",");
    Serial2.print(go);

    Serial2.println();
  //}
  counter++;
}

void stop_robot(int duration) {
  motorOff(0);
  motorOff(1);
  delay(duration);
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

