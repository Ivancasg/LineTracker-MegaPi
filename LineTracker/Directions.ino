
int16_t moveSpeed = 180;

void setSpeed(int16_t speed)
{
  moveSpeed=speed;
}
void Forward(void)
{
  runMotorA(moveSpeed);
  runMotorB(-moveSpeed);
}
void Backward(void)
{
  runMotorA(-moveSpeed);
  runMotorB(moveSpeed);
}
void BackwardAndTurnLeft(void)
{
  runMotorA(-moveSpeed/4);
  runMotorB(moveSpeed);
}
void BackwardAndTurnRight(void)
{
  runMotorA(-moveSpeed);
  runMotorB(moveSpeed/4);
}
void TurnLeft(void)
{
  runMotorA(moveSpeed);
  runMotorB(-moveSpeed/2);
}
void TurnRight(void)
{
  runMotorA(moveSpeed/2);
  runMotorB(-moveSpeed);
}
void TurnLeft1(void)
{
  runMotorA(moveSpeed);
  runMotorB(moveSpeed);
}
void TurnRight1(void)
{
  runMotorA(-moveSpeed);
  runMotorB(-moveSpeed);
}
void Stop(void)
{
  runMotorA(0);
  runMotorB(0);
}
