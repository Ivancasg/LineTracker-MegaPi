
#define S1_IN_S2_IN   (0x00)
#define S1_IN_S2_OUT  (0x01)
#define S1_OUT_S2_IN  (0x02)
#define S1_OUT_S2_OUT (0x03)

int16_t LineFollowFlag=0;

void detectLine(void) {
  uint8_t val;
  val = readSensors();  
  switch (val)
  {
    case S1_IN_S2_IN:
      Forward();
      LineFollowFlag=10;
      break;

    case S1_IN_S2_OUT:
       Forward();
      if (LineFollowFlag>1) LineFollowFlag--;
      break;

    case S1_OUT_S2_IN:
      Forward();
      if (LineFollowFlag<20) LineFollowFlag++;
      break;

    case S1_OUT_S2_OUT:
      if(LineFollowFlag==10) Backward();
      if(LineFollowFlag<10) TurnLeft1();
      if(LineFollowFlag>10) TurnRight1();
      break;
  }
}

uint8_t readSensors(void) {
  uint8_t state  = S1_IN_S2_IN;
  bool s1State = digitalRead(LINESENSOR1);
  bool s2State = digitalRead(LINESENSOR2);
  state = ( (1 & s1State) << 1) | s2State;
  return(state);  
}
