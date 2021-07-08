// ROTARY

//// MOTOR KANAN DEPAN
//#define kanan_depan_maju        A0
//#define kanan_depan_mundur      A1
//
//// MOTOR KIRI DEPAN
//#define kiri_depan_maju         A2
//#define kiri_depan_mundur       A3
//
//
//// MOTOR KANAN BELAKANG
//#define kanan_belakang_maju     A4
//#define kanan_belakang_mundur   A5
//
//
//// MOTOR KIRI BELAKANG
//#define kiri_belakang_maju      A6
//#define kiri_belakang_mundur    A7

// DRIBBLE
#define db_kanan_cw    2
#define db_kanan_ccw   4
#define db_kanan_pwm   3

#define db_kiri_cw     13
#define db_kiri_ccw    12
#define db_kiri_pwm    11

// KICKER
#define tendang_cw  8
#define tendang_ccw 7
#define tendang_pwm 10

int spd_db = 255;
String data;
bool kick = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //  pinMode(kanan_depan_maju, INPUT);
  //  pinMode(kanan_depan_mundur, INPUT);
  //  pinMode(kiri_depan_maju, INPUT);
  //  pinMode(kiri_depan_mundur, INPUT);
  //
  //  pinMode(kanan_belakang_maju, INPUT);
  //  pinMode(kanan_belakang_mundur, INPUT);
  //  pinMode(kiri_belakang_maju, INPUT);
  //  pinMode(kiri_belakang_mundur, INPUT);

  pinMode(db_kanan_cw, OUTPUT);
  pinMode(db_kanan_ccw, OUTPUT);
  pinMode(db_kiri_cw, OUTPUT);
  pinMode(db_kiri_ccw, OUTPUT);
  pinMode(db_kanan_pwm, OUTPUT);
  pinMode(db_kiri_pwm, OUTPUT);

  pinMode(tendang_cw, OUTPUT);
  pinMode(tendang_ccw, OUTPUT);
  pinMode(tendang_pwm, OUTPUT);
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
  else if (data == "TENDANG" && kick == false) {
    tendang(255);
    delay(50);
    init_tendang(35);
    delay(380);
    off_tendang();
    delay(100);
    kick = true;
    reset();
  }

  else if (data == "PASSING" && kick == false) {
    tendang(100);
    delay(150);
    init_tendang(35);
    delay(380);
    off_tendang();
    delay(100);
    kick = true;
    reset();
  }

  else if (data == "RESET" && kick == true) {
    reset();
  }
}

void reset() {
  data = "RESET";
  db_off();
  init_tendang(35);
  delay(300);
  off_tendang();
  delay(100);
  kick = false;
}

void tendang(int spd) {
  analogWrite(tendang_pwm, spd);
  digitalWrite(tendang_cw, LOW);
  digitalWrite(tendang_ccw, HIGH);
}

void init_tendang(int spd) {
  analogWrite(tendang_pwm, spd);
  digitalWrite(tendang_cw, HIGH);
  digitalWrite(tendang_ccw, LOW);
}

void off_tendang() {
  analogWrite(tendang_pwm, 0);
  digitalWrite(tendang_cw, LOW);
  digitalWrite(tendang_ccw, LOW);
}

void db_on(int spd) {
  analogWrite(db_kanan_pwm, spd);
  digitalWrite(db_kanan_cw, LOW);
  digitalWrite(db_kanan_ccw, HIGH);

  analogWrite(db_kiri_pwm, spd);
  digitalWrite(db_kiri_cw, HIGH);
  digitalWrite(db_kiri_ccw, LOW);
}

void db_off() {
  analogWrite(db_kanan_pwm, 0);
  digitalWrite(db_kanan_cw, LOW);
  digitalWrite(db_kanan_ccw, LOW);

  analogWrite(db_kiri_pwm, 0);
  digitalWrite(db_kiri_cw, LOW);
  digitalWrite(db_kiri_ccw, LOW);
}
