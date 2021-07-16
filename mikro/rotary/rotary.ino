// DRIBBLE
#define dribble_cw      4
#define dribble_ccw     2
#define dribble_pwm     3
#define dribble_limit   A0

// KICKER
#define tendang_cw      6
#define tendang_ccw     7
#define tendang_pwm     5
#define tendang_limit   A1

int spd_db = 255;
String data;
bool kick = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(dribble_cw, OUTPUT);
  pinMode(dribble_ccw, OUTPUT);
  pinMode(dribble_pwm, OUTPUT);

  pinMode(tendang_cw, OUTPUT);
  pinMode(tendang_ccw, OUTPUT);
  pinMode(tendang_pwm, OUTPUT);

  pinMode(dribble_limit, INPUT_PULLUP);
  pinMode(tendang_limit, INPUT_PULLUP);
}

void loop() {
//  if(digitalRead(tendang_limit) == HIGH){
//    db_off();
//  }
//  else if(digitalRead(tendang_limit) == LOW){
//    db_on(255);
//  }
//  Serial.println(digitalRead(tendang_limit));
//  delay(1000);
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
  analogWrite(dribble_pwm, spd);
  digitalWrite(dribble_cw, HIGH);
  digitalWrite(dribble_ccw, LOW);
}

void db_off() {
  analogWrite(dribble_pwm, 0);
  digitalWrite(dribble_cw, LOW);
  digitalWrite(dribble_ccw, LOW);
}
