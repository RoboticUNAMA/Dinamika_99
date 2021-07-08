// PWM MOTOR
#define kanan_depan_pwm         3
#define kiri_depan_pwm          9
#define kanan_belakang_pwm      10
#define kiri_belakang_pwm       5

// MOTOR KANAN DEPAN
#define kanan_depan_maju        6
#define kanan_depan_mundur      7

// MOTOR KIRI DEPAN
#define kiri_depan_maju         2
#define kiri_depan_mundur       8


// MOTOR KANAN BELAKANG
#define kanan_belakang_maju     11
#define kanan_belakang_mundur   4


// MOTOR KIRI BELAKANG
#define kiri_belakang_maju      12
#define kiri_belakang_mundur    13


int cepat = 120;
int sedang = 80;
int lambat = 50;
int count = 0;
String data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(kanan_depan_maju, OUTPUT);
  pinMode(kanan_depan_mundur, OUTPUT);
  pinMode(kiri_depan_maju, OUTPUT);
  pinMode(kiri_depan_mundur, OUTPUT);
  pinMode(kanan_depan_pwm, OUTPUT);
  pinMode(kiri_depan_pwm, OUTPUT);

  pinMode(kanan_belakang_maju, OUTPUT);
  pinMode(kanan_belakang_mundur, OUTPUT);
  pinMode(kiri_belakang_maju, OUTPUT);
  pinMode(kiri_belakang_mundur, OUTPUT);
  pinMode(kanan_belakang_pwm, OUTPUT);
  pinMode(kiri_belakang_pwm, OUTPUT);


  motor_stop();
}

void loop() {
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    Serial.println(data);
  }
//  data = "#M|DKA|120|0|1";

  String header = getValue(data, '|', 0);

  String idmotor = getValue(data, '|', 1);

  String nPwm = getValue(data, '|', 2);

  String polaritas1 = getValue(data, '|', 3);

  String polaritas2 = getValue(data, '|', 4);

//Serial.println(idmotor);
  if (idmotor == "DKI") {
    mtrDepanKanan(nPwm.toInt(), polaritas1.toInt(), polaritas2.toInt());
//    Serial.println("ada");
  }
  if (idmotor == "DKA") {
    mtrDepanKiri(nPwm.toInt(), polaritas1.toInt(), polaritas2.toInt());
  }
  if (idmotor == "BKI") {
    mtrBlkKanan(nPwm.toInt(), polaritas1.toInt(), polaritas2.toInt());
  }
  if (idmotor == "BKA") {
    mtrBlkKiri(nPwm.toInt(), polaritas1.toInt(), polaritas2.toInt());
  }






}

void mtrDepanKanan(int spd, int nilai1 , int nilai2)
{
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, nilai1);
  digitalWrite(kanan_depan_mundur, nilai2);
}

void mtrDepanKiri(int spd, int nilai1 , int nilai2)
{
  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, nilai1);
  digitalWrite(kiri_depan_mundur, nilai2);
}

void mtrBlkKanan(int spd, int nilai1 , int nilai2)
{
  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, nilai1);
  digitalWrite(kanan_belakang_mundur, nilai2);
}

void mtrBlkKiri(int spd, int nilai1 , int nilai2)
{
  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, nilai1);
  digitalWrite(kanan_belakang_mundur, nilai2);
}


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void motor_stop() {
  //  analogWrite(kanan_depan_pwm, 0);
  //  digitalWrite(kanan_depan_maju, HIGH);
  //  digitalWrite(kanan_depan_mundur, HIGH);
  //
  //  analogWrite(kiri_depan_pwm, 0);
  //  digitalWrite(kiri_depan_maju, HIGH);
  //  digitalWrite(kiri_depan_mundur, HIGH);
  //
  //  analogWrite(kanan_belakang_pwm, 0);
  //  digitalWrite(kanan_belakang_maju, HIGH);
  //  digitalWrite(kanan_belakang_mundur, HIGH);
  //
  //  analogWrite(kiri_belakang_pwm, 0);
  //  digitalWrite(kiri_belakang_maju, HIGH);
  //  digitalWrite(kiri_belakang_mundur, HIGH);
  //
  //  delay(100);
  //
  analogWrite(kanan_depan_pwm, 0);
  digitalWrite(kanan_depan_maju, LOW);
  digitalWrite(kanan_depan_mundur, LOW);

  analogWrite(kiri_depan_pwm, 0);
  digitalWrite(kiri_depan_maju, LOW);
  digitalWrite(kiri_depan_mundur, LOW);

  analogWrite(kanan_belakang_pwm, 0);
  digitalWrite(kanan_belakang_maju, LOW);
  digitalWrite(kanan_belakang_mundur, LOW);

  analogWrite(kiri_belakang_pwm, 0);
  digitalWrite(kiri_belakang_maju, LOW);
  digitalWrite(kiri_belakang_mundur, LOW);
}
