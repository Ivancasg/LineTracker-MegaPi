#define LINESENSOR1 A9
#define LINESENSOR2 A10

#define MOTORA_PWM 11
#define MOTORA_EN  31
#define MOTORA_IN1 33
#define MOTORA_IN2 32

#define MOTORB_PWM 12
#define MOTORB_EN  18
#define MOTORB_IN1 34
#define MOTORB_IN2 35

void setup() {
  pinMode(LINESENSOR1,INPUT);
  pinMode(LINESENSOR2,INPUT);

  pinMode(MOTORA_EN,OUTPUT);
  pinMode(MOTORB_EN,OUTPUT);
  
  pinMode(MOTORA_IN1,OUTPUT);
  pinMode(MOTORA_IN2,OUTPUT);
  pinMode(MOTORB_IN1,OUTPUT);
  pinMode(MOTORB_IN2,OUTPUT);

  setSpeed(120);
  enableMotors();

}

void loop() {
  detectLine();
}
