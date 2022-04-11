/*Agregar detection sensores de choque
  Agregar deteccion sensor ultrasonico

  Agregar control manual
*/
#include <SoftwareSerial.h>

#define TEST_MOTORS 0
#define TEST_SENSORS 0

#define LINESENSOR1 A10
#define LINESENSOR2 A9

#define TOUCHSENSOR1 A8
#define TOUCHSENSOR2 A7

#define MOTORA_PWM 11
#define MOTORA_EN  31
#define MOTORA_IN1 33
#define MOTORA_IN2 32

#define MOTORB_PWM 12
#define MOTORB_EN  18
#define MOTORB_IN1 34
#define MOTORB_IN2 35

SoftwareSerial Bluetooth(13,14);

int moveSpeed = 90;
int lastSpeed;
int lastCase;

bool manualControl = false;
char val;

void setup() {
  pinMode(LINESENSOR1, INPUT);
  pinMode(LINESENSOR2, INPUT);

  pinMode(MOTORA_EN, OUTPUT);
  pinMode(MOTORB_EN, OUTPUT);

  pinMode(MOTORA_IN1, OUTPUT);
  pinMode(MOTORA_IN2, OUTPUT);
  pinMode(MOTORB_IN1, OUTPUT);
  pinMode(MOTORB_IN2, OUTPUT);

  digitalWrite(MOTORA_EN, HIGH);    //Enable motors
  digitalWrite(MOTORB_EN, HIGH);

  moveSpeed = 90;

  Serial.begin(9600);
  Bluetooth.begin(9600);

}

void loop() {

  if (TEST_MOTORS) {
    //Run MOTOR_A FORWARD
    runMotorA(moveSpeed);
    runMotorB(0);
    Serial.println("Run MotorA Forward");
    delay(1000);
    //Run MOTOR_A BACKWARD
    runMotorA(-moveSpeed);
    runMotorB(0);
    Serial.println("Run MotorA Backward");
    delay(1000);
    //Run MOTOR_B FORWARD
    runMotorA(0);
    runMotorB(moveSpeed);
    Serial.println("Run MotorB Forward");
    delay(1000);
    //Run MOTOR_B BACKWARD
    runMotorA(0);
    runMotorB(-moveSpeed);
    Serial.println("Run MotorB Backward");
    delay(1000);
  }

  if (TEST_SENSORS) {
    int sensorVal_1 = 0;
    int sensorVal_2 = 0;
    sensorVal_1 =  analogRead(LINESENSOR1);
    sensorVal_2 =  analogRead(LINESENSOR2);
    Serial.print("Sensor 1 = ");
    Serial.print(sensorVal_1);
    Serial.print("Sensor 2 = ");
    Serial.println(sensorVal_2);
  }

  if (Bluetooth.available()) {
    val = Bluetooth.read();
    Serial.println(val); // Print the reading value
    parseCommandControl(val);
    if (manualControl == true) parseCommandDirections(val);
    parseCommandCamera(val);
  }

  if (manualControl == false) {
    if (!detectObject())detectLine();
    else Bluetooth.print('L');    
  }


}

void detectLine(void) {
  uint8_t val;
  val = readSensors();
  switch (val)
  {
    case 0b00:
      Forward();
      lastCase = 0b00;
      break;

    case 0b01:
      TurnLeft();
      lastCase = 0b01;
      break;

    case 0b10:
      TurnRight();
      lastCase = 0b10;
      break;

    case 0b11:
      if (lastCase == 0b01) TurnLeft1();
      if (lastCase == 0b10) TurnRight1();
      break;
  }
}

uint8_t readSensors(void) {
  uint8_t state  = 0b00;
  bool s1State;
  bool s2State;
  if (analogRead(LINESENSOR1) > 600) s1State = 1;
  else s1State = 0;
  if (analogRead(LINESENSOR2) > 600) s2State = 1;
  else s2State = 0;
  state = ( (1 & s1State) << 1) | s2State;
  return (state);
}

bool detectObject(void) {
  if (digitalRead(TOUCHSENSOR1) || digitalRead(TOUCHSENSOR2)) return true;
  else return false;
}
void runMotorA(int16_t speed)
{
  speed = speed > 255 ? 255 : speed;
  speed = speed < -255 ? -255 : speed;

  if (lastSpeed != speed)
  {
    lastSpeed = speed;
  }
  else
  {
    return;
  }

  if (speed > 0)
  {
    digitalWrite(MOTORA_IN2, LOW);
    delayMicroseconds(5);
    digitalWrite(MOTORA_IN1, HIGH);
    analogWrite(MOTORA_PWM, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(MOTORA_IN1, LOW);
    delayMicroseconds(5);
    digitalWrite(MOTORA_IN2, HIGH);
    analogWrite(MOTORA_PWM, -speed);
  }
  else
  {
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORA_IN1, LOW);
    analogWrite(MOTORA_PWM, 0);
  }
}

void runMotorB(int16_t speed)
{
  speed = speed > 255 ? 255 : speed;
  speed = speed < -255 ? -255 : speed;

  if (lastSpeed != speed)
  {
    lastSpeed = speed;
  }
  else
  {
    return;
  }

  if (speed > 0)
  {
    digitalWrite(MOTORB_IN2, LOW);
    delayMicroseconds(5);
    digitalWrite(MOTORB_IN1, HIGH);
    analogWrite(MOTORB_PWM, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(MOTORB_IN1, LOW);
    delayMicroseconds(5);
    digitalWrite(MOTORB_IN2, HIGH);
    analogWrite(MOTORB_PWM, -speed);
  }
  else
  {
    digitalWrite(MOTORB_IN2, LOW);
    digitalWrite(MOTORB_IN1, LOW);
    analogWrite(MOTORB_PWM, 0);
  }
}
void Forward(void)
{
  runMotorA(moveSpeed);
  runMotorB(moveSpeed);
}
void Backward(void)
{
  runMotorA(-moveSpeed);
  runMotorB(-moveSpeed);
}
void BackwardAndTurnLeft(void)
{
  runMotorA(-moveSpeed);
  runMotorB(-moveSpeed / 4);
}
void BackwardAndTurnRight(void)
{
  runMotorA(-moveSpeed / 4);
  runMotorB(-moveSpeed);
}
void TurnLeft(void)
{
  runMotorA(moveSpeed / 2);
  runMotorB(moveSpeed);
}
void TurnRight(void)
{
  runMotorA(moveSpeed);
  runMotorB(moveSpeed / 2);
}
void TurnLeft1(void)
{
  runMotorA(-moveSpeed);
  runMotorB(moveSpeed);
}
void TurnRight1(void)
{
  runMotorA(moveSpeed);
  runMotorB(-moveSpeed);
}
void Stop(void)
{
  runMotorA(0);
  runMotorB(0);
}
void parseCommandControl(char input) {
  switch (input) {
    case '$':
      manualControl = true; 
      break;
    case '&':
      manualControl = false; 
      break;    
  }
}

void parseCommandDirections(char input) {
  switch (input) {
    case '1':
      TurnLeft1();   
      break;
    case '2':
      Forward();
      break;
    case '3':
      TurnRight1();
      break;
    case '4':
      Backward();
      break;
  }
}

void parseCommandCamera(char input) {
  switch (input) {
    case '5':
      //TurnCameraLeft();   
      break;
    case '7':
      //TurnCameraRight();
      break;
  }
}
