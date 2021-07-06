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


int count = 0, spd = 0;
String data_lama, data_baru;

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
    data_baru = Serial.readStringUntil('\n');
    Serial.println(data_baru);
    if (data_lama != data_baru) {
      data_lama = data_baru;
      while (spd > 50) {
        spd -= 2;
        if (spd <= 50);
        spd = 50;
        break;
      }
    }
    else if (data_lama == data_baru) {
      spd += 2;
    }

    if (data_baru == "BERHENTI") {
      spd = 0;
    }
  }

  if (data_baru == "MAJU") {
    motor_maju(spd);
  }
  else if (data_baru == "MUNDUR") {
    motor_mundur(spd);
  }
  else if (data_baru == "BELOK KIRI") {
    motor_belok_kiri(spd);
  }
  else if (data_baru == "BELOK KANAN") {
    motor_belok_kanan(spd);
  }
  else if (data_baru == "PUTAR KIRI") {
    motor_putar_kiri(spd);
  }
  else if (data_baru == "PUTAR KANAN") {
    motor_putar_kanan(spd);
  }
  else if (data_baru == "GESER KIRI") {
    motor_geser_kiri(spd);
  }
  else if (data_baru == "GESER KANAN") {
    motor_geser_kanan(spd);
  }
  else if (data_baru == "SERONG KANAN MAJU") {
    motor_serong_kanan_maju(spd);
  }
  else if (data_baru == "SERONG KIRI MAJU") {
    motor_serong_kiri_maju(spd);
  }
  else if (data_baru == "SERONG KANAN MUNDUR") {
    motor_serong_kanan_mundur(spd);
  }
  else if (data_baru == "SERONG KIRI MUNDUR") {
    motor_serong_kiri_mundur(spd);
  }
  else if (data_baru == "BERHENTI") {
    motor_stop();
  }
}

void motor_mundur(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, LOW);
  digitalWrite(kanan_depan_mundur, HIGH);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, LOW);
  digitalWrite(kiri_depan_mundur, HIGH);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, LOW);
  digitalWrite(kanan_belakang_mundur, HIGH);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, LOW);
  digitalWrite(kiri_belakang_mundur, HIGH);
}

void motor_maju(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, HIGH);
  digitalWrite(kanan_depan_mundur, LOW);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, HIGH);
  digitalWrite(kiri_depan_mundur, LOW);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, HIGH);
  digitalWrite(kanan_belakang_mundur, LOW);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, HIGH);
  digitalWrite(kiri_belakang_mundur, LOW);
}

void motor_belok_kiri(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, HIGH);
  digitalWrite(kanan_depan_mundur, LOW);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, LOW);
  digitalWrite(kiri_depan_mundur, LOW);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, HIGH);
  digitalWrite(kanan_belakang_mundur, LOW);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, LOW);
  digitalWrite(kiri_belakang_mundur, LOW);
}

void motor_belok_kanan(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, LOW);
  digitalWrite(kanan_depan_mundur, LOW);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, HIGH);
  digitalWrite(kiri_depan_mundur, LOW);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, LOW);
  digitalWrite(kanan_belakang_mundur, LOW);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, HIGH);
  digitalWrite(kiri_belakang_mundur, LOW);
}

void motor_putar_kiri(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, HIGH);
  digitalWrite(kanan_depan_mundur, LOW);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, LOW);
  digitalWrite(kiri_depan_mundur, HIGH);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, HIGH);
  digitalWrite(kanan_belakang_mundur, LOW);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, LOW);
  digitalWrite(kiri_belakang_mundur, HIGH);
}

void motor_putar_kanan(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, LOW);
  digitalWrite(kanan_depan_mundur, HIGH);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, HIGH);
  digitalWrite(kiri_depan_mundur, LOW);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, LOW);
  digitalWrite(kanan_belakang_mundur, HIGH);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, HIGH);
  digitalWrite(kiri_belakang_mundur, LOW);
}

void motor_geser_kiri(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, HIGH);
  digitalWrite(kanan_depan_mundur, LOW);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, LOW);
  digitalWrite(kiri_depan_mundur, HIGH);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, HIGH);
  digitalWrite(kanan_belakang_mundur, LOW);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, LOW);
  digitalWrite(kiri_belakang_mundur, HIGH);
}

void motor_geser_kanan(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, LOW);
  digitalWrite(kanan_depan_mundur, HIGH);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, HIGH);
  digitalWrite(kiri_depan_mundur, LOW);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, HIGH);
  digitalWrite(kanan_belakang_mundur, LOW);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, LOW);
  digitalWrite(kiri_belakang_mundur, HIGH);
}

void motor_serong_kanan_maju(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, LOW);
  digitalWrite(kanan_depan_mundur, LOW);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, HIGH);
  digitalWrite(kiri_depan_mundur, LOW);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, HIGH);
  digitalWrite(kanan_belakang_mundur, LOW);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, LOW);
  digitalWrite(kiri_belakang_mundur, LOW);
}

void motor_serong_kiri_maju(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, HIGH);
  digitalWrite(kanan_depan_mundur, LOW);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, LOW);
  digitalWrite(kiri_depan_mundur, LOW);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, LOW);
  digitalWrite(kanan_belakang_mundur, LOW);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, HIGH);
  digitalWrite(kiri_belakang_mundur, LOW);
}

void motor_serong_kanan_mundur(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, LOW);
  digitalWrite(kanan_depan_mundur, LOW);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, LOW);
  digitalWrite(kiri_depan_mundur, HIGH);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, LOW);
  digitalWrite(kanan_belakang_mundur, HIGH);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, LOW);
  digitalWrite(kiri_belakang_mundur, LOW);
}

void motor_serong_kiri_mundur(int spd) {
  analogWrite(kanan_depan_pwm, spd);
  digitalWrite(kanan_depan_maju, LOW);
  digitalWrite(kanan_depan_mundur, HIGH);

  analogWrite(kiri_depan_pwm, spd);
  digitalWrite(kiri_depan_maju, LOW);
  digitalWrite(kiri_depan_mundur, LOW);

  analogWrite(kanan_belakang_pwm, spd);
  digitalWrite(kanan_belakang_maju, LOW);
  digitalWrite(kanan_belakang_mundur, LOW);

  analogWrite(kiri_belakang_pwm, spd);
  digitalWrite(kiri_belakang_maju, LOW);
  digitalWrite(kiri_belakang_mundur, HIGH);
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
