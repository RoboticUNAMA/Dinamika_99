//motor kanan
#define R_maju_R     5//pin D2
#define L_mundur_R   6 //pin D3
#define PWM_R     7 //Pin D4
//motor kiri
#define R_maju_L    8//pin D6
#define L_mundur_L   9 //pin D7
#define PWM_L     10 //pin D8

void setup()
{
  Serial.begin(9600);
  pinMode(R_maju_R, OUTPUT);
  pinMode(L_mundur_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(R_maju_L, OUTPUT);
  pinMode(L_mundur_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  motor_stop();
}

void loop()
{
  motor_stop();   delay(1000);
  motor_cw();     delay(5000);
  motor_stop();   delay(1000);
  motor_ccw();     delay(5000);
}

void motor_cw()//belok kanan
{
  analogWrite(PWM_R ,512); //pwm kanan
  digitalWrite(R_maju_R, HIGH);
  digitalWrite(L_mundur_R, LOW);
  analogWrite(PWM_L ,512); //pwm kiri
  digitalWrite(R_maju_L, HIGH);
  digitalWrite(L_mundur_L, LOW);
}

void motor_ccw()  //belok kiri
{
  analogWrite(PWM_R ,512);  //pwm kanan
  digitalWrite(R_maju_R, LOW);
  digitalWrite(L_mundur_R, HIGH);
  analogWrite(PWM_L ,512);  //pwm kiri
  digitalWrite(R_maju_L, LOW);
  digitalWrite(L_mundur_L, HIGH);
}

void motor_stop()
{
  analogWrite(PWM_R ,0); //pwm kanan
  digitalWrite(R_maju_R, LOW);
  digitalWrite(L_mundur_R, LOW);
  analogWrite(PWM_L ,0); //pwm kiri
  digitalWrite(R_maju_L, LOW);
  digitalWrite(L_mundur_L, LOW);
}
