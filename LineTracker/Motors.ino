
int16_t last_speed;

void enableMotors(void)
{
  digitalWrite(MOTORA_EN, HIGH);
  digitalWrite(MOTORB_EN, HIGH);
}

void runMotorA(int16_t speed)
{
  speed = speed > 255 ? 255 : speed;
  speed = speed < -255 ? -255 : speed;

  if(last_speed != speed)
  {
    last_speed = speed;
  }
  else
  {
    return;
  }
 
  if(speed > 0)
  {
    digitalWrite(MOTORA_IN2, LOW);
    delayMicroseconds(5);
    digitalWrite(MOTORA_IN1, HIGH);
    analogWrite(MOTORA_PWM,speed);
  }
  else if(speed < 0)
  {
    digitalWrite(MOTORA_IN1, LOW);
    delayMicroseconds(5);
    digitalWrite(MOTORA_IN2, HIGH);
    analogWrite(MOTORA_PWM,-speed);
  }
  else
  {
    digitalWrite(MOTORA_IN2, LOW);
    digitalWrite(MOTORA_IN1, LOW);
    analogWrite(MOTORA_PWM,0);
  }
}

void runMotorB(int16_t speed)
{
  speed = speed > 255 ? 255 : speed;
  speed = speed < -255 ? -255 : speed;

  if(last_speed != speed)
  {
    last_speed = speed;
  }
  else
  {
    return;
  }
 
  if(speed > 0)
  {
    digitalWrite(MOTORB_IN2, LOW);
    delayMicroseconds(5);
    digitalWrite(MOTORB_IN1, HIGH);
    analogWrite(MOTORB_PWM,speed);
  }
  else if(speed < 0)
  {
    digitalWrite(MOTORB_IN1, LOW);
    delayMicroseconds(5);
    digitalWrite(MOTORB_IN2, HIGH);
    analogWrite(MOTORB_PWM,-speed);
  }
  else
  {
    digitalWrite(MOTORB_IN2, LOW);
    digitalWrite(MOTORB_IN1, LOW);
    analogWrite(MOTORB_PWM,0);
  }
}
