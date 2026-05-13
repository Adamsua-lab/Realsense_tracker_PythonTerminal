#include <Adafruit_ISM330DHCX.h>

Adafruit_ISM330DHCX ism330dhcx;

const int sensorPin = 2;
volatile unsigned long lastPulse = 0;
volatile unsigned long period = 0;
volatile bool newPulse = false;

float rpm_latest = 0.0f;
float gain = 2.3f;  // ajustar segun medicion real

// 60 Hz timing
const uint32_t frequency = 60;
const uint32_t loop_period = 1000000UL / frequency;
uint32_t data_report_timestamp = 0;

void detectInterrupt() 
{
  unsigned long now = micros();
  if (now - lastPulse > 100) 
  {
    period = now - lastPulse;
    lastPulse = now;
    newPulse = true;
  }
}

void setup(void) 
{
  Serial.begin(115200);
  Serial.println("timestamp_ms,rpm,ax,ay,az,gx,gy,gz");
  pinMode(sensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin), detectInterrupt, FALLING);
  if (!ism330dhcx.begin_I2C()) 
  {
    while (1) 
    {
      delay(10);
    }
  }
  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  ism330dhcx.setAccelDataRate(LSM6DS_RATE_104_HZ);
  ism330dhcx.setGyroDataRate(LSM6DS_RATE_104_HZ);
}

void loop() 
{
  // RPM - siempre, sin esperar el timing
  if (newPulse && period > 0) 
  {
    noInterrupts();
    unsigned long revolutionTime = period;
    newPulse = false;
    interrupts();
    rpm_latest = gain * (60.0f * 1.0e6f / revolutionTime);
    if (rpm_latest > 8500.0f) rpm_latest = 8500.0f;
  }

  if (micros() - lastPulse > 2000000) 
  {
    rpm_latest = 0.0;
  }

  // Timing 60Hz - leer IMU y enviar datos
  uint32_t now = micros();
  if (data_report_timestamp == 0) 
  {
    data_report_timestamp = now;
  }
  int32_t difference = (int32_t)(now - data_report_timestamp);
  if (difference < 0) 
  {
    return;
  }
  data_report_timestamp += loop_period;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);

  Serial.print(millis());                      Serial.print(",");
  Serial.print(rpm_latest, 2);                 Serial.print(",");
  Serial.print(accel.acceleration.x, 4);       Serial.print(",");
  Serial.print(accel.acceleration.y, 4);       Serial.print(",");
  Serial.print(accel.acceleration.z, 4);       Serial.print(",");
  Serial.print(gyro.gyro.x, 4);               Serial.print(",");
  Serial.print(gyro.gyro.y, 4);               Serial.print(",");
  Serial.println(gyro.gyro.z, 4);
}