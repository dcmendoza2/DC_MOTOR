#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// TinkerCAD Example: https://www.tinkercad.com/things/fDbfQagj7va

MPU6050 mpu;

int16_t gx, gy, gz;

double valgx; 
double valgy;
double valgz;

double gyroRange;

int motorPin = 3;
void setup() {
  pinMode(motorPin, OUTPUT);
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  // Note we can set the Gyro Range by using setFullScaleGyroRange(range) where range can be [0,1,2,3]
  // mpu.setFullScaleGyroRange(3);
  switch(mpu.getFullScaleGyroRange())
  {
    case 0:
    Serial.println("Gyro range: +/- 250 degrees/sec");
    gyroRange = 250;
    break;
    case 1:
    Serial.println("Gyro range: +/- 500 degrees/sec");
    gyroRange = 500;
    break;
    case 2:
    Serial.println("Gyro range: +/- 1000 degrees/sec");
    gyroRange = 1000;
    break;
    case 3:
    Serial.println("Gyro range: +/- 2000 degrees/sec");
    gyroRange = 2000;
    break;
  }
  Serial.println("--------------------");
  mpu.setXGyroOffset(5);
  mpu.setYGyroOffset(54);
  mpu.setZGyroOffset(36);
}


void loop() {
  // put your main code here, to run repeatedly:
  mpu.getRotation(&gx, &gy, &gz);
  
  valgx = map2(gx, -32768, 32768, -gyroRange, gyroRange);
  valgy = map2(gy, -32768, 32768, -gyroRange, gyroRange);
  valgz = map2(gz, -32768, 32768, -gyroRange, gyroRange);

  double angDispX = integrateX(valgx);
  double angDispY = integrateY(valgy);
  double angDispZ = integrateZ(valgz);

  double angAccelX = derivativeX(valgx);
  double angAccelY = derivativeY(valgy);
  double angAccelZ = derivativeZ(valgz);
  
  // PWM frequency is determined by 1/(2*k*10e-6)
  int k = 1000;
  double multiplier = k/180.0;
  double f = 1000000.0/(2*k);
  Serial.print("PWM Frequency: ");
  Serial.print(f);
  Serial.print("Hz Duty: ");
  Serial.println((k + angDispX*multiplier)/(2.0*k));
  digitalWrite(motorPin, HIGH);
  delayMicroseconds(k + angDispX * multiplier);
  digitalWrite(motorPin, LOW);
  delayMicroseconds(k - angDispX * multiplier);
  /*
  if (angDispX > 0){
    
  }
  else{
    digitalWrite(motorPin, LOW);
    delayMicroseconds(2*k);
  }
  // delay(100);
  */
}


double integrateX(float x) {
  static float last_x = 0;
  static unsigned long last_t = 0;
  static double Ix = 0;

  unsigned long t = millis();

  long dt = t - last_t;
  last_t = t;

  Ix = Ix + (x + last_x) * dt / 2000.0;
  last_x = x;

  return Ix;
}

double integrateY(float x) {
  static float last_x = 0;
  static unsigned long last_t = 0;
  static long Ix = 0;

  unsigned long t = millis();

  long dt = t - last_t;
  last_t = t;

  Ix = Ix + (x + last_x) * dt / 2000.0;
  last_x = x;

  return Ix;
}

double integrateZ(float x) {
  static float last_x = 0;
  static unsigned long last_t = 0;
  static long Ix = 0;

  unsigned long t = millis();

  long dt = t - last_t;
  last_t = t;

  Ix = Ix + (x + last_x) * dt / 2000.0;
  last_x = x;

  return Ix;
}

double derivativeX(float x) {
  static float last_x = 0;
  static unsigned long last_t = 0;

  unsigned long t = millis();
  long dt = t - last_t;
  last_t = t;

  double Dx = (x - last_x) * 1000 / dt;
  return Dx;
}

double derivativeY(float x) {
  static float last_x = 0;
  static unsigned long last_t = 0;

  unsigned long t = millis();
  long dt = t - last_t;
  last_t = t;

  double Dx = (x - last_x) * 1000 / dt;
  return Dx;
}

double derivativeZ(float x) {
  static float last_x = 0;
  static unsigned long last_t = 0;

  unsigned long t = millis();
  long dt = t - last_t;
  last_t = t;

  double Dx = (x - last_x) * 1000 / dt;
  return Dx;
}

double map2(double  x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
