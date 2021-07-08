// PWM MOTOR
#define kanan_depan_pwm         3
#define kiri_depan_pwm          9
#define kanan_belakang_pwm      10
#define kiri_belakang_pwm       5

// MOTOR KANAN DEPAN
#define kanan_depan_maju        6
#define kanan_depan_mundur      7

// MOTOR KIRI DEPAN
#define kiri_depan_maju         8
#define kiri_depan_mundur       2


// MOTOR KANAN BELAKANG
#define kanan_belakang_maju     11
#define kanan_belakang_mundur   4


// MOTOR KIRI BELAKANG
#define kiri_belakang_maju      13
#define kiri_belakang_mundur    12

float Kp = 5;
float Kd = 5;
float Ki = 0;

float error, error_sebelumnya, jumlah_error, selisih_error;
float P, I, D;
float PID;
int speed_awal = 180;



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

  error = 0;
  error_sebelumnya = 0;
  jumlah_error = 0;
  selisih_error = 0;
  motor_stop();
}

void loop() {
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    Serial.println(data);
  }
  //  data = "#M|RUN|120|120|120|120";

  String header = getValue(data, '|', 0);

  String cmd = getValue(data, '|', 1);

  String mtr1 = getValue(data, '|', 2);

  String mtr2 = getValue(data, '|', 3);

  String mtr3 = getValue(data, '|', 4);

  String mtr4 = getValue(data, '|', 5);

  //Serial.println(idmotor);

  if (cmd == "RUN") {
    mtrDepanKiri(mtr1.toInt());
    mtrDepanKanan(mtr2.toInt());
    mtrBlkKanan(mtr3.toInt());
    mtrBlkKiri(mtr4.toInt());
  }

  if (cmd == "STP") {
    motor_stop();
  }

  //Serial.println(idmotor);
  //   mtrDepanKiri(100);
  //   mtrDepanKanan(-40);
  //   mtrBlkKanan(100);
  //   mtrBlkKiri(50);
}

void mtrDepanKiri(int spd)
{
  if (spd > 255) {
    spd = 255;
  }

  if (spd < -255) {
    spd = -255;
  }

  if (spd >= 0 ) {
    analogWrite(kiri_depan_pwm, spd);
    digitalWrite(kiri_depan_maju, HIGH);
    digitalWrite(kiri_depan_mundur, LOW);
  }
  else {
    analogWrite(kiri_depan_pwm, -spd);
    digitalWrite(kiri_depan_maju, LOW);
    digitalWrite(kiri_depan_mundur, HIGH);
  }

}

void mtrBlkKanan(int spd)
{

  if (spd > 255) {
    spd = 255;
  }
  if (spd < -255) {
    spd = -255;
  }

  if (spd >= 0 ) {
    analogWrite(kanan_belakang_pwm, spd);
    digitalWrite(kanan_belakang_maju, HIGH);
    digitalWrite(kanan_belakang_mundur, LOW);
  }
  else {
    analogWrite(kanan_belakang_pwm, -spd);
    digitalWrite(kanan_belakang_maju, LOW);
    digitalWrite(kanan_belakang_mundur, HIGH);
  }


}

void mtrBlkKiri(int spd)
{

  if (spd > 255) {
    spd = 255;
  }

  if (spd < -255) {
    spd = -255;
  }

  if (spd >= 0 ) {
    analogWrite(kiri_belakang_pwm, spd);
    digitalWrite(kiri_belakang_maju, HIGH);
    digitalWrite(kiri_belakang_mundur, LOW);
  }
  else {
    analogWrite(kiri_belakang_pwm, -spd);
    digitalWrite(kiri_belakang_maju, LOW);
    digitalWrite(kiri_belakang_mundur, HIGH);
  }
}

void mtrDepanKanan(int spd)
{

  if (spd > 255) {
    spd = 255;
  }

  if (spd < -255) {
    spd = -255;
  }

  if (spd >= 0 ) {
    analogWrite(kanan_depan_pwm, spd);
    digitalWrite(kanan_depan_maju, HIGH);
    digitalWrite(kanan_depan_mundur, LOW);
  }
  else {
    analogWrite(kanan_depan_pwm, -spd);
    digitalWrite(kanan_depan_maju, LOW);
    digitalWrite(kanan_depan_mundur, HIGH);
  }
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
