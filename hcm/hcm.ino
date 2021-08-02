
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// DRIBBLE
#define dribble_cw    2
#define dribble_ccw   4
#define dribble_pwm   3

// TENDANGAN
#define tendang_cw    6
#define tendang_ccw   7
#define tendang_pwm   5

//LIMIT
#define limit_db      A0
#define limit_tendang A1

String data;
int compass = 0;
int bola = 0;

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  //  Serial.println("------------------------------------");
  //  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  //  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  //  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  //  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  //  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  //  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  //  Serial.println("------------------------------------");
  //  Serial.println("");
  delay(500);
}

void setup(void)
{
  Serial.begin(9600);
  //Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  pinMode(dribble_cw, OUTPUT);
  pinMode(dribble_ccw, OUTPUT);
  pinMode(dribble_pwm, OUTPUT);

  pinMode(tendang_cw, OUTPUT);
  pinMode(tendang_ccw, OUTPUT);
  pinMode(tendang_pwm, OUTPUT);

  pinMode(limit_db, INPUT_PULLUP);
  pinMode(limit_tendang, INPUT_PULLUP);

  /* Initialise the sensor */
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    //Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

  /* Display some basic information on this sensor */
  //displaySensorDetails();
}

void loop(void)
{
  int dapat_bola = cekLimitDB(); // jika 0 = bola dapat, jika 1 bola tidak dapat
  int lmtd = cekLimitTendang();
  init_tendang(lmtd);
  
  //int lm_tendang = cekLimitTendang(); // jika 0 = dapat limit
  if (dapat_bola == 0 && bola == 0) {
    Serial.println("Dapat Bola");
    bola = 1;
  }
  else if (dapat_bola == 1 && bola == 1{
    Serial.println("Tidak Dapat Bola");
    bola = 0;
  }
  delay(10);


  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    //Serial.println(data);
  }
  if (data == "DB ON") {
    db_on(255);
  }
  else if (data == "DB OFF") {
    db_off();
  }

  if (data == "COMPASS ON") {
    compass = 1;
  }
  else if (data == "COMPASS OFF") {
    compass = 0;
  }

  else if (data == "TEND1") {
    data = "";
    tendang_keras();

  }

  else if (data == "TEND2") {
    data = "";
    db_off();
    tendang_operan();

  }

  if (compass == 1) {
    /* Get a new sensor event */
    sensors_event_t event;
    mag.getEvent(&event);

    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    //  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    //  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    //  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(event.magnetic.y, event.magnetic.x);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.22;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if (heading < 0)
      heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
      heading -= 2 * PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180 / M_PI;

    Serial.print("Heading:  "); Serial.print(headingDegrees); Serial.print(" degrees");
    Serial.println();

    delay(200);
  }
}


void db_on(int spd) {
  analogWrite(dribble_pwm, spd);
  digitalWrite(dribble_cw, LOW);
  digitalWrite(dribble_ccw, HIGH);
}

void db_off() {
  analogWrite(dribble_pwm, 0);
  digitalWrite(dribble_cw, LOW);
  digitalWrite(dribble_ccw, LOW);
}

int cekLimitDB() {
  int lm = digitalRead(limit_db);
  return lm;
}

int cekLimitTendang() {
  int lm = digitalRead(limit_tendang);
  return lm;
}


void tendang_keras()
{
  int lm = cekLimitTendang();
  analogWrite(tendang_pwm, 255);
  digitalWrite(tendang_cw, LOW);
  digitalWrite(tendang_ccw, HIGH);

  delay(90);

  analogWrite(tendang_pwm, 0);
  digitalWrite(tendang_cw, LOW);
  digitalWrite(tendang_ccw, LOW);
}

void tendang_operan()
{
  int lm = cekLimitTendang();
  analogWrite(tendang_pwm, 225);
  digitalWrite(tendang_cw, LOW);
  digitalWrite(tendang_ccw, HIGH);

  delay(90);

  analogWrite(tendang_pwm, 0);
  digitalWrite(tendang_cw, LOW);
  digitalWrite(tendang_ccw, LOW);
}
void init_tendang(int limit){
  if(limit == 1){
    analogWrite(tendang_pwm, 30);
    digitalWrite(tendang_cw, HIGH);
    digitalWrite(tendang_ccw, LOW);
  }
  else{
    analogWrite(tendang_pwm, 0);
    digitalWrite(tendang_cw, LOW);
    digitalWrite(tendang_ccw, LOW);
  }
    
}
