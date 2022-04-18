#define TEST_MOTORS   0
#define TEST_SENSORS  0

#define LINESENSOR1   A10
#define LINESENSOR2   A9

#define TOUCHSENSOR1  A8
#define TOUCHSENSOR2  A7

#define ECHOSENSOR    A11
#define TRIGSENSOR    A12

#define MOTORA_PWM 11
#define MOTORA_EN  31
#define MOTORA_IN1 33
#define MOTORA_IN2 32

#define MOTORB_PWM 12
#define MOTORB_EN  18
#define MOTORB_IN1 34
#define MOTORB_IN2 35

int moveSpeed = 90;
int sensorDistance = 50;
int lastSpeed;
int lastCase;

bool manualControl = false;
char val;

void setup() {
  pinMode(LINESENSOR1, INPUT);
  pinMode(LINESENSOR2, INPUT);

  pinMode(TOUCHSENSOR1, INPUT);
  pinMode(TOUCHSENSOR2, INPUT);

  pinMode(MOTORA_EN, OUTPUT);
  pinMode(MOTORB_EN, OUTPUT);

  pinMode(MOTORA_IN1, OUTPUT);
  pinMode(MOTORA_IN2, OUTPUT);
  pinMode(MOTORB_IN1, OUTPUT);
  pinMode(MOTORB_IN2, OUTPUT);

  pinMode(ECHOSENSOR, INPUT);
  pinMode(TRIGSENSOR, OUTPUT);
  digitalWrite(TRIGSENSOR, LOW);

  digitalWrite(MOTORA_EN, HIGH);    //Enable motors
  digitalWrite(MOTORB_EN, HIGH);

  moveSpeed = 90;

  Serial.begin(9600);
  Serial3.begin(9600);

}

void loop() {

  if (TEST_MOTORS) {
    //Run MOTOR_A FORWARD
    runMotorA(moveSpeed);
    runMotorB(0);
    Serial.println("Run MotorA Forward");
    delay(2000);
    //Run MOTOR_A BACKWARD
    runMotorA(-moveSpeed);
    runMotorB(0);
    Serial.println("Run MotorA Backward");
    delay(2000);
    //Run MOTOR_B FORWARD
    runMotorA(0);
    runMotorB(moveSpeed);
    Serial.println("Run MotorB Forward");
    delay(2000);
    //Run MOTOR_B BACKWARD
    runMotorA(0);
    runMotorB(-moveSpeed);
    Serial.println("Run MotorB Backward");
    delay(2000);
  }

  if (TEST_SENSORS) {
    int sensorVal_1 = 0;
    int sensorVal_2 = 0;
    int sensorVal_3 = 0;
    int sensorVal_4 = 0;
    sensorVal_1 =  analogRead(LINESENSOR1);
    sensorVal_2 =  analogRead(LINESENSOR2);
    sensorVal_3 =  digitalRead(TOUCHSENSOR1);
    sensorVal_4 =  digitalRead(TOUCHSENSOR2);
    Serial.print("Sensor 1 = ");
    Serial.print(sensorVal_1);
    Serial.print("\tSensor 2 = ");
    Serial.print(sensorVal_2);
    Serial.print("\tSensor 3 = ");
    Serial.print(sensorVal_3);
    Serial.print("\tSensor 4 = ");
    Serial.println(sensorVal_4);
  }

  if (Serial3.available()) {
    val = Serial3.read();
    Serial.print("Bluetooth read = ");
    Serial.println(val); // Print the reading value
    parseCommandControl(val);
    if (manualControl == true) parseCommandDirections(val);
    parseCommandCamera(val);
  }

  if (manualControl == false) {
    if (!detectObject())detectLine();
    else {
      Serial3.print('L');
      Serial.println("Object detected");
      delay(100);
    }    
  }

  delay(10);
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

  long timeTravel; //tiempo que demora en llegar el eco
  long distanceTravel; //distancia en centimetros  
  
  digitalWrite(TRIGSENSOR, LOW);
  delayMicroseconds(2);           //Clear the Trigger pin
  digitalWrite(TRIGSENSOR, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(TRIGSENSOR, LOW);

  timeTravel = pulseIn(ECHOSENSOR, HIGH, 2000); //obtenemos el ancho del pulso
  timeTravel = timeTravel == 0 ? 9999 : timeTravel; //Si el tiempo es 0 asignar 9999
  distanceTravel = timeTravel / 59; //escalamos el tiempo a una distancia en cm

  if (distanceTravel < sensorDistance){
    Serial.println("Object detected by ultrasonic sensor");
    //return true;
  }
  if (digitalRead(TOUCHSENSOR1) || digitalRead(TOUCHSENSOR2)) {
    Serial.println("Object detected by impact sensors");
    //return true;
  }

  return false;
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
    Serial.println("Run MotorA Forward");
  }
  else if (speed < 0)
  {
    digitalWrite(MOTORA_IN1, LOW);
    delayMicroseconds(5);
    digitalWrite(MOTORA_IN2, HIGH);
    analogWrite(MOTORA_PWM, -speed);
    Serial.println("Run MotorA Backward");
  }
  else
  {
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORA_IN1, LOW);
    analogWrite(MOTORA_PWM, 0);
    Serial.println("Stop MotorA");
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
    Serial.println("Run MotorB Forward");
  }
  else if (speed < 0)
  {
    digitalWrite(MOTORB_IN1, LOW);
    delayMicroseconds(5);
    digitalWrite(MOTORB_IN2, HIGH);
    analogWrite(MOTORB_PWM, -speed);
    Serial.println("Run MotorB Backward");
  }
  else
  {
    digitalWrite(MOTORB_IN2, LOW);
    digitalWrite(MOTORB_IN1, LOW);
    analogWrite(MOTORB_PWM, 0);
    Serial.println("Stop MotorB");
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
    case 'A':
      Serial.println("Manual Control ON");
      manualControl = true;
      break;
    case 'D':
      Serial.println("Manual Control OFF");
      manualControl = false;
      break;
  }
}

void parseCommandDirections(char input) {
  switch (input) {
    case '1':
      Forward();
      break;
    case '2':
      TurnRight1();
      break;
    case '3':
      Backward();
      break;
    case '4':
      TurnLeft1();
      break;
    case '0':
      Stop();
      break;
  }
}

void parseCommandCamera(char input) {
  switch (input) {
    case '8':
      //TurnCameraLeft();
      break;
    case '6':
      //TurnCameraRight();
      break;
    case '9':
      //StopCamera();
      break;
  }
}
