#include <Encoder.h>

#include <Wire.h>
#include <SPI.h>
#include <FreeSixIMU.h>

// Motor stuffs
int STBY = 10; //standby

// Motor A
int PWMA = 6; //Speed control 
int AIN1 = 9; //Direction
int AIN2 = 8; //Direction

// Motor B
int PWMB = 5; //Speed control
int BIN1 = 16; //Direction
int BIN2 = 14; //Direction

FreeSixIMU sixDOF = FreeSixIMU();

const int can_move = 1;

Encoder left(0, 1);

// imu vars
float imuValues[6];
float ypr[3];
float roll;

void setup()
{
  Serial.begin(9600);
  
  // just wait a couple seconds
  delay(2000);

  Serial.println("Initializing motors...");
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  Serial.println("Motors initialized!");

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  
  Serial.println("Initializing IMU...");
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
  Serial.println("IMU initialized!");

}

int count = 0;

float angle, angle_velocity, last_angle;

long r_wheel, r_wheel_velocity, r_last_wheel;
long l_wheel, l_wheel_velocity, l_last_wheel;
long wheel_velocity, wheel;
long speed;

int torque;
int ai, av;
int dk1, dk2, dk3, dk4;

float other_angles[3];

float alpha = 0.8;
float oldCleanSignal;
float cleanSignal;

void loop()
{
  sixDOF.getValues(imuValues);  
  //sixDOF.getYawPitchRoll(ypr);
  sixDOF.getAngles(other_angles);

  /*
  Serial.print(imuValues[0]);
  Serial.print(" ");
  Serial.print(imuValues[1]);
  Serial.print(" ");
  Serial.print(imuValues[2]);
  Serial.print(" ");
  Serial.print(imuValues[3]);
  Serial.print(" ");
  Serial.print(imuValues[4]);
  Serial.print(" ");
  Serial.print(imuValues[5]);
  Serial.println();
  delay(100);
  return;
  */

  //angle = other_angles[2];
  angle = -1 * (imuValues[0] + 40);

  // very simple LPF
  oldCleanSignal = cleanSignal;
  cleanSignal = alpha * oldCleanSignal + (1 - alpha) * angle;
  angle = cleanSignal;

  angle_velocity = angle - last_angle;
  last_angle = angle;

  av = angle_velocity;
  ai = angle;

  // calculate speed
  l_wheel = left.read();  
  l_wheel_velocity = l_wheel - l_last_wheel;
  l_last_wheel = l_wheel;
  speed = l_wheel_velocity;

  wheel += speed;

  dk1 = analogRead(A0) / 20;
  dk2 = analogRead(A1) / 20;
  dk3 = analogRead(A2) / 20;
  dk4 = analogRead(A3) / 20;

  // the guts of the matter
  torque = (av * dk1) + (ai * dk2) + (speed * dk3) + (wheel * dk4);

  int go = torque;


  if(go > 0) {
    if (go > 255)
      go = 255;
    if(abs(angle) < 250) {
      move(1, go, 1);
      move(2, go, 1);
    }
  } else {
    go = abs(go);
    if (go > 255)
      go = 255;
    if(abs(angle) < 250) {
      move(1, go, 0);
      move(2, go, 0);
    }
  }

  Serial.print("0,");
  Serial.print(dk1);
  Serial.print(",");
  Serial.print(dk2);
  Serial.print(",");
  Serial.print(dk3);
  Serial.print(",");
  Serial.print(dk4);
  Serial.print(",");
  Serial.print(ai);
  Serial.print(",");
  Serial.print(av);
  Serial.print(",");
  Serial.print(wheel);
  Serial.print(",");
  Serial.print(speed);
  Serial.print(",");
  Serial.print(torque);
  Serial.print(",");
  Serial.print(go);
  Serial.println();
  
  delay(30);
  count++;
}


void move(int motor, int speed, int direction) {
  if(! can_move)
    return;
  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void stop() {
  //enable standby  
  digitalWrite(STBY, LOW); 
}
