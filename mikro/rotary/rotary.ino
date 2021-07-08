// ROTARY

// MOTOR KANAN DEPAN
#define kanan_depan_maju        A0
#define kanan_depan_mundur      A1

// MOTOR KIRI DEPAN
#define kiri_depan_maju         A2
#define kiri_depan_mundur       A3


// MOTOR KANAN BELAKANG
#define kanan_belakang_maju     A4
#define kanan_belakang_mundur   A5


// MOTOR KIRI BELAKANG
#define kiri_belakang_maju      A6
#define kiri_belakang_mundur    A7

// DRIBBLE
#define dbKanan_cw    7
#define dbKanan_ccw   8
#define dbKanan_pwm   5

#define dbKiri_cw     2
#define dbKiri_ccw    3
#define dbKiri_pwm    6

int spd_db = 512;
String data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(kanan_depan_maju, INPUT);
  pinMode(kanan_depan_mundur, INPUT);
  pinMode(kiri_depan_maju, INPUT);
  pinMode(kiri_depan_mundur, INPUT);

  pinMode(kanan_belakang_maju, INPUT);
  pinMode(kanan_belakang_mundur, INPUT);
  pinMode(kiri_belakang_maju, INPUT);
  pinMode(kiri_belakang_mundur, INPUT);

  pinMode(dbKanan_cw, OUTPUT);
  pinMode(dbKanan_ccw, OUTPUT);
  pinMode(dbKiri_cw, OUTPUT);
  pinMode(dbKiri_ccw, OUTPUT);
  pinMode(dbKanan_pwm, OUTPUT);
  pinMode(dbKiri_pwm, OUTPUT);
}

void loop() {
//put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    Serial.println(data);
  }
  if (data == "DB ON") {
    db_on(255);
  }
  else if (data == "DB OFF") {
    db_off();
  }
  //analogWrite(6, 255);
  //digitalWrite(2, LOW);
  //digitalWrite(3, HIGH);
  //
  //analogWrite(5, 255);
  //digitalWrite(7, HIGH);
  //digitalWrite(8, LOW);
  //db_on(512);
  //  delay(5000);
  //  db_off();
  //  delay(1000);
}

void db_on(int spd) {
  analogWrite(6, spd);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);

  analogWrite(5, spd);
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
}

void db_off() {
  analogWrite(6, 0);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);

  analogWrite(5, 0);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
}
