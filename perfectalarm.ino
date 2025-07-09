#include <Wire.h>
#include <RTClib.h>
#include <MPU6050.h>
#include <math.h>

RTC_DS3231 rtc;
MPU6050 mpu(0x69);

const int BUZZER_PIN = 9;
const int PULSE_PIN = A0;

enum SleepStage { AWAKE, REM_SLEEP, LIGHT_SLEEP, DEEP_SLEEP };

DateTime alarmTime;
DateTime monitoringStartTime;
bool monitoringStarted = false;
bool monitoringInfoPrinted = false;
bool napScenario = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println("Enter wake-up time (HH MM format):");
  while (Serial.available() == 0);
  int alarmHour = Serial.parseInt();
  int alarmMinute = Serial.parseInt();

  DateTime now = rtc.now();
  alarmTime = DateTime(now.year(), now.month(), now.day(), alarmHour, alarmMinute, 0);
  monitoringStartTime = alarmTime - TimeSpan(0, 1, 55, 0);

  if (alarmTime < now) {
    alarmTime = alarmTime + TimeSpan(1, 0, 0, 0);
    monitoringStartTime = monitoringStartTime + TimeSpan(1, 0, 0, 0);
  }

  if (monitoringStartTime < now) {
    napScenario = true;
    Serial.println("Nap scenario: Monitoring disabled");
    Serial.print("Alarm will ring at ");
    printDateTime(alarmTime);
  }

  if (!napScenario) {
    Serial.print("Next alarm at: ");
    printDateTime(alarmTime);
    Serial.print("Monitoring starts at: ");
    printDateTime(monitoringStartTime);
  }

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  DateTime now = rtc.now();

  if (now.hour() == alarmTime.hour() && 
      now.minute() == alarmTime.minute() &&
      now.day() == alarmTime.day()) {
    Serial.println("WAKE UP TIME REACHED!");
    triggerBuzzer();
    while (1);
  }

  if (napScenario) {
    delay(1000);
    return;
  }

  if (!monitoringStarted) {
    if (now >= monitoringStartTime) {
      monitoringStarted = true;
      Serial.println("Starting sleep stage monitoring...");
    } else if (!monitoringInfoPrinted) {
      TimeSpan remaining = monitoringStartTime - now;
      Serial.print("Monitoring starts in ");
      Serial.print(remaining.hours());
      Serial.print("h ");
      Serial.print(remaining.minutes());
      Serial.println("m");
      monitoringInfoPrinted = true;
    }
  }

  if (monitoringStarted) {
    static int lastMinute = -1;
    if (now.minute() != lastMinute) {
      lastMinute = now.minute();

      float heartRate = getHeartRate();
      float heartRateStd = getHeartRateStd();
      float accVal = 0.0, gyroVal = 0.0;
      getMPUData(accVal, gyroVal);

      SleepStage stage = classifySleepStage(heartRate, heartRateStd, accVal, gyroVal);

      Serial.print("Sleep Stage: ");
      switch (stage) {
        case AWAKE:
          Serial.println("AWAKE");
          break;
        case REM_SLEEP:
          Serial.println("REM SLEEP");
          break;
        case LIGHT_SLEEP:
          Serial.println("LIGHT SLEEP → Triggering alarm");
          triggerBuzzer();
          while (1);
          break;
        case DEEP_SLEEP:
          Serial.println("DEEP SLEEP");
          break;
      }
    }
  }

  delay(1000);
}

void printDateTime(DateTime dt) {
  char buf[] = "YYYY-MM-DD HH:MM";
  sprintf(buf, "%04d-%02d-%02d %02d:%02d", 
          dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute());
  Serial.println(buf);
}

SleepStage classifySleepStage(float hr, float hrStd, float acc, float gyro) {
  if (acc > 0.3 || gyro > 10.0) return AWAKE;
  if (hr > 80) return AWAKE;
  if (hr < 65) return DEEP_SLEEP;
  return (hrStd >= 5.0) ? REM_SLEEP : LIGHT_SLEEP;
}

float getHeartRate() {
  return map(analogRead(PULSE_PIN), 0, 1023, 50, 100);
}

float getHeartRateStd() {
  return 4.0;
}

void getMPUData(float &accVal, float &gyroVal) {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accVal = sqrt((long)ax * ax + (long)ay * ay + (long)az * az) / 16384.0;
  gyroVal = sqrt((long)gx * gx + (long)gy * gy + (long)gz * gz) / 131.0;
}

void triggerBuzzer() {
  tone(BUZZER_PIN, 2000);
  delay(8000);
  noTone(BUZZER_PIN);
}
