// ROTARY KICKER
#define kicker_A      A1
#define kicker_B      A0
#define kicker_maju   13
#define kicker_mundur 12
#define kicker_pwm    11

// MOTOR KIRI BELAKANG
#define kiri_belakang_maju         A2
#define kiri_belakang_mundur       A3


// MOTOR KANAN BELAKANG
#define kanan_belakang_maju     A4
#define kanan_belakang_mundur   A5

// DRIBBLE
#define dbKanan_cw    7
#define dbKanan_ccw   8
#define dbKanan_pwm   5

#define dbKiri_cw     3
#define dbKiri_ccw    2
#define dbKiri_pwm    6

int val;
int kicker_pos = 0;
int kicker_last = LOW;
int n = LOW;
int limit = 20;
int spd_db = 255;
String data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(kicker_A, INPUT);
  pinMode(kicker_B, INPUT);

  pinMode(kanan_belakang_maju, INPUT);
  pinMode(kanan_belakang_mundur, INPUT);
  pinMode(kiri_belakang_maju, INPUT);
  pinMode(kiri_belakang_mundur, INPUT);

  pinMode(kicker_maju, OUTPUT);
  pinMode(kicker_mundur, OUTPUT);
  pinMode(kicker_pwm, OUTPUT);
  pinMode(dbKanan_cw, OUTPUT);
  pinMode(dbKanan_ccw, OUTPUT);
  pinMode(dbKiri_cw, OUTPUT);
  pinMode(dbKiri_ccw, OUTPUT);
  pinMode(dbKanan_pwm, OUTPUT);
  pinMode(dbKiri_pwm, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
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
  else if(data == "TENDANG"){
    tendang();
  }
  else if(data == "PASSING"){
    passing();
  }

  //ROTARY
  //    n = digitalRead(kicker_A);
  //    if ((kicker_last == LOW) && (n == HIGH)) {
  //      if (digitalRead(kicker_B) == LOW) {
  //        kicker_pos--;
  //      } else {
  //        kicker_pos++;
  //      }
  //      Serial.println(kicker_pos);
  //    }
  //    kicker_last = n;
}

void tendang() {
  analogWrite(kicker_pwm, 255);
  digitalWrite(kicker_maju, HIGH);
  digitalWrite(kicker_mundur, LOW);
  delay(100);

  analogWrite(kicker_pwm, 50);
  digitalWrite(kicker_maju, LOW);
  digitalWrite(kicker_mundur, HIGH);
  delay(500);
  
  analogWrite(kicker_pwm, 0);
  digitalWrite(kicker_maju, LOW);
  digitalWrite(kicker_mundur, LOW);
  delay(1500);
}

void passing(){
  analogWrite(kicker_pwm, 100);
  digitalWrite(kicker_maju, HIGH);
  digitalWrite(kicker_mundur, LOW);
  delay(100);

  analogWrite(kicker_pwm, 50);
  digitalWrite(kicker_maju, LOW);
  digitalWrite(kicker_mundur, HIGH);
  delay(500);

  analogWrite(kicker_pwm, 0);
  digitalWrite(kicker_maju, LOW);
  digitalWrite(kicker_mundur, LOW);
  delay(1500);
}

void db_on(int spd) {
  analogWrite(dbKanan_pwm, spd);
  digitalWrite(dbKanan_cw, HIGH);
  digitalWrite(dbKanan_ccw, LOW);

  analogWrite(dbKiri_pwm, spd);
  digitalWrite(dbKiri_cw, HIGH);
  digitalWrite(dbKiri_ccw, LOW);
}

void db_off() {
  analogWrite(dbKanan_pwm, 0);
  digitalWrite(dbKanan_cw, LOW);
  digitalWrite(dbKanan_ccw, LOW);

  analogWrite(dbKiri_pwm, 0);
  digitalWrite(dbKiri_cw, LOW);
  digitalWrite(dbKiri_ccw, LOW);
}
