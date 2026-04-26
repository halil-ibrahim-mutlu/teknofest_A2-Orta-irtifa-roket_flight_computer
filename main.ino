#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>

#include <TinyGPSPlus.h>

// =====================================================
// A2 Orta Irtifa - Pico + BNO055 + BMP280 + L86 GPS + LoRa E32 + SD
// LoRa E32 ayarlari yapilmaz. Sadece UART uzerinden veri gonderilir.
// GPS: Quectel L86, NMEA UART
// =====================================================

// -------------------- I2C pinleri --------------------
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// -------------------- LoRa E32 UART pinleri --------------------
#define LORA_TX_PIN 0   // Pico TX -> E32 RX
#define LORA_RX_PIN 1   // Pico RX <- E32 TX

// -------------------- GPS UART pinleri --------------------
#define GPS_TX_PIN 8    // Pico TX -> GPS RX
#define GPS_RX_PIN 9    // Pico RX <- GPS TX

// -------------------- SD kart SPI pinleri --------------------
#define SD_MISO_PIN 16
#define SD_CS_PIN   17
#define SD_SCK_PIN  18
#define SD_MOSI_PIN 19

// -------------------- Cikis pinleri --------------------
#define DEPLOY_PRIMARY_PIN   10
#define DEPLOY_SECONDARY_PIN 11
#define BUZZER_PIN           12
#define FLASH_LED_PIN        13

// -------------------- Basinc ayari --------------------
// Atis alaninda guncel deniz seviyesi basinci ile guncellenmeli.
#define SEA_LEVEL_PRESSURE_HPA 1012.50

// -------------------- Ucus esikleri --------------------
#define MAIN_DEPLOY_ALTITUDE_M 500.0
#define LAUNCH_ACCEL_THRESHOLD 18.0
#define LAUNCH_ALT_THRESHOLD_M 30.0

#define LANDED_ALT_THRESHOLD_M 20.0
#define LANDED_VEL_THRESHOLD_MPS 1.0
#define LANDED_CONFIRM_TIME_MS 10000

#define TELEMETRY_PERIOD_MS 200
#define SENSOR_PERIOD_MS    50
#define SD_LOG_PERIOD_MS    200
#define DEPLOY_PULSE_MS     1000

// -------------------- UART hizlari --------------------
#define LORA_BAUD 9600
#define GPS_BAUD  9600

// -------------------- Sensor nesneleri --------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP280 bmp;
TinyGPSPlus gps;

// -------------------- SD kart --------------------
File logFile;
bool sdReady = false;
const char *LOG_FILENAME = "flight.csv";

// -------------------- Ucus durumlari --------------------
enum FlightState {
  STATE_PAD = 0,
  STATE_ASCENT = 1,
  STATE_APOGEE_DETECTED = 2,
  STATE_DROGUE_DEPLOYED = 3,
  STATE_MAIN_DEPLOYED = 4,
  STATE_LANDED = 5
};

FlightState flightState = STATE_PAD;

// -------------------- BMP280 verileri --------------------
float pressure_hPa = 0.0;
float temperature_C = 0.0;

float rawAltitude_m = 0.0;
float altitudeFiltered_m = 0.0;
float groundAltitude_m = 0.0;
float relativeAltitude_m = 0.0;
float previousRelativeAltitude_m = 0.0;

float verticalVelocity_mps = 0.0;
float verticalVelocityFiltered_mps = 0.0;
float maxAltitude_m = 0.0;

// -------------------- BNO055 verileri --------------------
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float accelMagnitude = 0.0;

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

// -------------------- GPS verileri --------------------
bool gpsValid = false;
double gpsLat = 0.0;
double gpsLon = 0.0;
double gpsAlt_m = 0.0;
double gpsSpeed_mps = 0.0;
int gpsSatellites = 0;
double gpsHdop = 0.0;

// -------------------- Durum bayraklari --------------------
bool primaryDeployed = false;
bool secondaryDeployed = false;

// -------------------- Zamanlayicilar --------------------
unsigned long lastSensorMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastSdLogMs = 0;
unsigned long lastBlinkMs = 0;

unsigned long primaryDeployStartMs = 0;
unsigned long secondaryDeployStartMs = 0;

unsigned long landedCandidateStartMs = 0;

// -------------------- Filtre katsayilari --------------------
const float ALT_ALPHA = 0.18;
const float VEL_ALPHA = 0.25;

// =====================================================
// Yardimci fonksiyonlar
// =====================================================

String stateName(FlightState s) {
  switch (s) {
    case STATE_PAD: return "PAD";
    case STATE_ASCENT: return "ASCENT";
    case STATE_APOGEE_DETECTED: return "APOGEE";
    case STATE_DROGUE_DEPLOYED: return "DROGUE";
    case STATE_MAIN_DEPLOYED: return "MAIN";
    case STATE_LANDED: return "LANDED";
    default: return "UNKNOWN";
  }
}

void errorBlink(int delayMs) {
  while (1) {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(delayMs);
  }
}

// =====================================================
// Ayrilma sinyal cikislari
// =====================================================

void startPrimaryDeploySignal() {
  if (!primaryDeployed) {
    primaryDeployed = true;
    primaryDeployStartMs = millis();
    digitalWrite(DEPLOY_PRIMARY_PIN, HIGH);
  }
}

void startSecondaryDeploySignal() {
  if (!secondaryDeployed) {
    secondaryDeployed = true;
    secondaryDeployStartMs = millis();
    digitalWrite(DEPLOY_SECONDARY_PIN, HIGH);
  }
}

void updateDeployPulses() {
  unsigned long now = millis();

  if (primaryDeployed && digitalRead(DEPLOY_PRIMARY_PIN) == HIGH) {
    if (now - primaryDeployStartMs >= DEPLOY_PULSE_MS) {
      digitalWrite(DEPLOY_PRIMARY_PIN, LOW);
    }
  }

  if (secondaryDeployed && digitalRead(DEPLOY_SECONDARY_PIN) == HIGH) {
    if (now - secondaryDeployStartMs >= DEPLOY_PULSE_MS) {
      digitalWrite(DEPLOY_SECONDARY_PIN, LOW);
    }
  }
}

// =====================================================
// Buzzer ve LED
// =====================================================

void updateBuzzerAndFlash() {
  if (flightState == STATE_LANDED) {
    digitalWrite(BUZZER_PIN, HIGH);

    if (millis() - lastBlinkMs >= 300) {
      lastBlinkMs = millis();
      digitalWrite(FLASH_LED_PIN, !digitalRead(FLASH_LED_PIN));
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(FLASH_LED_PIN, LOW);
  }
}

// =====================================================
// GPS okuma
// =====================================================

void updateGPS() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  gpsValid = gps.location.isValid();

  if (gps.location.isValid()) {
    gpsLat = gps.location.lat();
    gpsLon = gps.location.lng();
  }

  if (gps.altitude.isValid()) {
    gpsAlt_m = gps.altitude.meters();
  }

  if (gps.speed.isValid()) {
    gpsSpeed_mps = gps.speed.mps();
  }

  if (gps.satellites.isValid()) {
    gpsSatellites = gps.satellites.value();
  }

  if (gps.hdop.isValid()) {
    gpsHdop = gps.hdop.hdop();
  }
}

// =====================================================
// Sensor okuma
// =====================================================

void readSensors() {
  unsigned long now = millis();
  float dt = (now - lastSensorMs) / 1000.0;
  if (dt <= 0.0) dt = SENSOR_PERIOD_MS / 1000.0;

  pressure_hPa = bmp.readPressure() / 100.0;
  temperature_C = bmp.readTemperature();

  rawAltitude_m = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);

  previousRelativeAltitude_m = relativeAltitude_m;

  altitudeFiltered_m =
    ALT_ALPHA * rawAltitude_m + (1.0 - ALT_ALPHA) * altitudeFiltered_m;

  relativeAltitude_m = altitudeFiltered_m - groundAltitude_m;

  verticalVelocity_mps =
    (relativeAltitude_m - previousRelativeAltitude_m) / dt;

  verticalVelocityFiltered_mps =
    VEL_ALPHA * verticalVelocity_mps +
    (1.0 - VEL_ALPHA) * verticalVelocityFiltered_mps;

  if (relativeAltitude_m > maxAltitude_m) {
    maxAltitude_m = relativeAltitude_m;
  }

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  accelX = accel.x();
  accelY = accel.y();
  accelZ = accel.z();

  accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyroX = gyro.x();
  gyroY = gyro.y();
  gyroZ = gyro.z();

  lastSensorMs = now;
}

// =====================================================
// Ucus algoritmasi
// =====================================================

void updateFlightState() {
  // Kalkis tespiti:
  // Kriter 1: ivme esigi
  // Kriter 2: irtifa artisi
  if (flightState == STATE_PAD) {
    bool accelLaunch = accelMagnitude > LAUNCH_ACCEL_THRESHOLD;
    bool altitudeLaunch = relativeAltitude_m > LAUNCH_ALT_THRESHOLD_M;

    if (accelLaunch || altitudeLaunch) {
      flightState = STATE_ASCENT;
    }
  }

  // Apogee tespiti:
  // Kriter 1: filtrelenmis dikey hiz negatif
  // Kriter 2: maksimum irtifadan dusus baslamis
  if (flightState == STATE_ASCENT) {
    bool velocityFalling = verticalVelocityFiltered_mps < -2.0;
    bool altitudeDroppedFromMax = (maxAltitude_m - relativeAltitude_m) > 8.0;
    bool minimumAltitudeReached = relativeAltitude_m > 100.0;

    if (minimumAltitudeReached && velocityFalling && altitudeDroppedFromMax) {
      flightState = STATE_APOGEE_DETECTED;
      startPrimaryDeploySignal();
    }
  }

  if (flightState == STATE_APOGEE_DETECTED) {
    flightState = STATE_DROGUE_DEPLOYED;
  }

  // Ikincil parasut:
  // A2 icin 400-600 m araliginda acilmali. Burada hedef 500 m.
  if (flightState == STATE_DROGUE_DEPLOYED) {
    bool belowMainAltitude = relativeAltitude_m <= MAIN_DEPLOY_ALTITUDE_M;
    bool descending = verticalVelocityFiltered_mps < -1.0;

    if (belowMainAltitude && descending) {
      flightState = STATE_MAIN_DEPLOYED;
      startSecondaryDeploySignal();
    }
  }

  // Yere inis tespiti:
  if (flightState == STATE_MAIN_DEPLOYED) {
    bool nearGround = relativeAltitude_m < LANDED_ALT_THRESHOLD_M;
    bool slowVerticalSpeed = abs(verticalVelocityFiltered_mps) < LANDED_VEL_THRESHOLD_MPS;

    if (nearGround && slowVerticalSpeed) {
      if (landedCandidateStartMs == 0) {
        landedCandidateStartMs = millis();
      }

      if (millis() - landedCandidateStartMs >= LANDED_CONFIRM_TIME_MS) {
        flightState = STATE_LANDED;
      }
    } else {
      landedCandidateStartMs = 0;
    }
  }
}

// =====================================================
// Telemetri paketi
// =====================================================

String makeTelemetryPacket() {
  String packet = "";

  packet += "T=" + String(millis());
  packet += ",STATE=" + stateName(flightState);

  packet += ",P_hPa=" + String(pressure_hPa, 2);
  packet += ",TEMP_C=" + String(temperature_C, 2);

  packet += ",ALT_M=" + String(relativeAltitude_m, 2);
  packet += ",MAX_ALT_M=" + String(maxAltitude_m, 2);
  packet += ",VEL_MPS=" + String(verticalVelocityFiltered_mps, 2);

  packet += ",ACC_X=" + String(accelX, 2);
  packet += ",ACC_Y=" + String(accelY, 2);
  packet += ",ACC_Z=" + String(accelZ, 2);
  packet += ",ACC_MAG=" + String(accelMagnitude, 2);

  packet += ",GYRO_X=" + String(gyroX, 2);
  packet += ",GYRO_Y=" + String(gyroY, 2);
  packet += ",GYRO_Z=" + String(gyroZ, 2);

  packet += ",GPS_FIX=" + String(gpsValid ? 1 : 0);
  packet += ",GPS_LAT=" + String(gpsLat, 6);
  packet += ",GPS_LON=" + String(gpsLon, 6);
  packet += ",GPS_ALT_M=" + String(gpsAlt_m, 2);
  packet += ",GPS_SPEED_MPS=" + String(gpsSpeed_mps, 2);
  packet += ",GPS_SATS=" + String(gpsSatellites);
  packet += ",GPS_HDOP=" + String(gpsHdop, 2);

  packet += ",PRIMARY=" + String(primaryDeployed ? 1 : 0);
  packet += ",SECONDARY=" + String(secondaryDeployed ? 1 : 0);

  packet += ",SD=" + String(sdReady ? 1 : 0);

  return packet;
}

void sendTelemetry() {
  String packet = makeTelemetryPacket();

  Serial.println(packet);   // USB Serial Monitor
  Serial1.println(packet);  // LoRa E32'ye gonderme burasi
}

// =====================================================
// SD karta kayit
// =====================================================

void writeCsvHeader() {
  if (!sdReady) return;

  logFile = SD.open(LOG_FILENAME, FILE_WRITE);

  if (logFile) {
    logFile.println(
      "time_ms,state,"
      "pressure_hPa,temperature_C,"
      "altitude_m,max_altitude_m,vertical_velocity_mps,"
      "accel_x,accel_y,accel_z,accel_mag,"
      "gyro_x,gyro_y,gyro_z,"
      "gps_fix,gps_lat,gps_lon,gps_alt_m,gps_speed_mps,gps_sats,gps_hdop,"
      "primary_deployed,secondary_deployed"
    );
    logFile.close();
  }
}

void logDataToSD() {
  if (!sdReady) return;

  logFile = SD.open(LOG_FILENAME, FILE_WRITE);

  if (logFile) {
    logFile.print(millis());
    logFile.print(",");
    logFile.print(stateName(flightState));
    logFile.print(",");

    logFile.print(pressure_hPa, 2);
    logFile.print(",");
    logFile.print(temperature_C, 2);
    logFile.print(",");

    logFile.print(relativeAltitude_m, 2);
    logFile.print(",");
    logFile.print(maxAltitude_m, 2);
    logFile.print(",");
    logFile.print(verticalVelocityFiltered_mps, 2);
    logFile.print(",");

    logFile.print(accelX, 2);
    logFile.print(",");
    logFile.print(accelY, 2);
    logFile.print(",");
    logFile.print(accelZ, 2);
    logFile.print(",");
    logFile.print(accelMagnitude, 2);
    logFile.print(",");

    logFile.print(gyroX, 2);
    logFile.print(",");
    logFile.print(gyroY, 2);
    logFile.print(",");
    logFile.print(gyroZ, 2);
    logFile.print(",");

    logFile.print(gpsValid ? 1 : 0);
    logFile.print(",");
    logFile.print(gpsLat, 6);
    logFile.print(",");
    logFile.print(gpsLon, 6);
    logFile.print(",");
    logFile.print(gpsAlt_m, 2);
    logFile.print(",");
    logFile.print(gpsSpeed_mps, 2);
    logFile.print(",");
    logFile.print(gpsSatellites);
    logFile.print(",");
    logFile.print(gpsHdop, 2);
    logFile.print(",");

    logFile.print(primaryDeployed ? 1 : 0);
    logFile.print(",");
    logFile.println(secondaryDeployed ? 1 : 0);

    logFile.close();
  }
}

// =====================================================
// Yer irtifasi kalibrasyonu
// =====================================================

void calibrateGroundAltitude() {
  Serial.println("Yer irtifasi kalibre ediliyor...");

  const int sampleCount = 100;
  float sum = 0.0;

  for (int i = 0; i < sampleCount; i++) {
    updateGPS();
    float a = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
    sum += a;
    delay(30);
  }

  groundAltitude_m = sum / sampleCount;

  rawAltitude_m = groundAltitude_m;
  altitudeFiltered_m = groundAltitude_m;
  relativeAltitude_m = 0.0;
  previousRelativeAltitude_m = 0.0;
  verticalVelocityFiltered_mps = 0.0;
  maxAltitude_m = 0.0;

  Serial.print("Yer irtifasi: ");
  Serial.print(groundAltitude_m);
  Serial.println(" m");
}

// =====================================================
// setup
// =====================================================

void setup() {
  pinMode(DEPLOY_PRIMARY_PIN, OUTPUT);
  pinMode(DEPLOY_SECONDARY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FLASH_LED_PIN, OUTPUT);

  digitalWrite(DEPLOY_PRIMARY_PIN, LOW);
  digitalWrite(DEPLOY_SECONDARY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  // LoRa E32 UART
  Serial1.setTX(LORA_TX_PIN);
  Serial1.setRX(LORA_RX_PIN);
  Serial1.begin(LORA_BAUD);

  // GPS UART
  Serial2.setTX(GPS_TX_PIN);
  Serial2.setRX(GPS_RX_PIN);
  Serial2.begin(GPS_BAUD);

  // I2C
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  // BNO055 baslatma
  if (!bno.begin()) {
    Serial.println("BNO055 bulunamadi.");
    Serial1.println("ERROR,BNO055_NOT_FOUND");
    errorBlink(100);
  }

  bno.setExtCrystalUse(true);

  // BMP280 baslatma
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 0x76 adresinde bulunamadi, 0x77 deneniyor...");

    if (!bmp.begin(0x77)) {
      Serial.println("BMP280 bulunamadi.");
      Serial1.println("ERROR,BMP280_NOT_FOUND");
      errorBlink(500);
    }
  }

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_1
  );

  // SD kart SPI
  SPI.setRX(SD_MISO_PIN);
  SPI.setTX(SD_MOSI_PIN);
  SPI.setSCK(SD_SCK_PIN);
  SPI.begin();

  if (SD.begin(SD_CS_PIN)) {
    sdReady = true;
    Serial.println("SD kart hazir.");
    writeCsvHeader();
  } else {
    sdReady = false;
    Serial.println("SD kart baslatilamadi. Ucus devam eder ama kayit yok.");
    Serial1.println("WARNING,SD_NOT_READY");
  }

  delay(1000);

  calibrateGroundAltitude();

  lastSensorMs = millis();
  lastTelemetryMs = millis();
  lastSdLogMs = millis();

  Serial.println("Sistem hazir.");
  Serial1.println("SYSTEM_READY");
}

// =====================================================
// loop
// =====================================================

void loop() {
  unsigned long now = millis();

  // GPS surekli okunmali, yoksa NMEA verisi kacabilir.
  updateGPS();

  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    readSensors();
    updateFlightState();
  }

  updateDeployPulses();
  updateBuzzerAndFlash();

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }

  if (now - lastSdLogMs >= SD_LOG_PERIOD_MS) {
    lastSdLogMs = now;
    logDataToSD();
  }
}
