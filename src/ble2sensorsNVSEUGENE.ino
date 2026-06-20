#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <GyverFilters.h>
#include <InterpolationLib.h>
#include <Adafruit_MCP4725.h>
#include <Update.h>

#define FW_VERSION_BASE "2.7.7"
#ifdef CONFIG_IDF_TARGET_ESP32S3
  #define FW_VERSION FW_VERSION_BASE "-S3"
#else
  #define FW_VERSION FW_VERSION_BASE "-C3"
#endif

// ── Фильтры шума ──
GFilterRA analogHall;
GFilterRA analog0;
GFilterRA analog1;
GFilterRA analog2;
GFilterRA analog3;

// ── DAC (MCP4725) — broadcast на все адреса ──
static const uint8_t DAC_ADDRS[4] = {0x60, 0x61, 0x62, 0x63};
Adafruit_MCP4725 dacs[4];

void initDacs() {
  for (uint8_t i = 0; i < 4; i++) dacs[i].begin(DAC_ADDRS[i]);
}

void dacSetVoltage(uint16_t value) {
  for (uint8_t i = 0; i < 4; i++) dacs[i].setVoltage(value, false, 400000);
}

// ── Кривые педалей ──
double x[]    = { 0,1,2,3,4,5,6,7,8,9,10 };
double y[]    = { 0,6,14,30,54,71,81,86,90,94,100 };
double z[]    = { 0,18,28,37,45,53,61,70,80,90,100 };
int numValues = sizeof(x) / sizeof(x[0]);

// ── BLE характеристики ──
BLEServer*         pServer                  = NULL;
BLECharacteristic* commandCharacteristic    = NULL;
BLECharacteristic* sensorCharacteristic     = NULL;
BLECharacteristic* sensorA1Characteristic   = NULL;
BLECharacteristic* coefsCharacteristic      = NULL;
BLECharacteristic* defCoefsCharacteristic   = NULL;
// v2 — Flywheel
BLECharacteristic* fwRiseLoadedChar    = NULL;
BLECharacteristic* fwFallLoadedChar    = NULL;
BLECharacteristic* fwRiseUnloadedChar  = NULL;
BLECharacteristic* fwFallUnloadedChar  = NULL;
BLECharacteristic* fwHangDurChar       = NULL;
BLECharacteristic* fwHangThrChar       = NULL;
BLECharacteristic* fwSnapBoostChar     = NULL;
BLECharacteristic* fwSnapThrChar       = NULL;
BLECharacteristic* fwSnapFallChar      = NULL;
BLECharacteristic* emergencyModeChar   = NULL;
BLECharacteristic* fwVersionChar       = NULL;
BLECharacteristic* otaCtrlChar         = NULL;
BLECharacteristic* otaDataChar         = NULL;

// ── Пины ──
int Hall          = A0;
int G1            = A2;
const int numReadings = 1;

// ── Калибровка ──
int   fullyPressedNVSValueA0  = 0;
int   fullyReleasedNVSValueA0 = 0;
int   fullyPressedNVSValueA1  = 500;
int   fullyReleasedNVSValueA1 = 2000;
float readingsA0[numReadings];
float readingsA1[numReadings];
bool  deviceConnected = false;

// ── Коэффициенты шумового фильтра (фиксированные) ──
static const float FILTER_CLUTCH      = 0.8f;   // сцепление — быстро, только шум
static const float FILTER_GAS_FAST    = 0.3f;   // газ — быстрая компонента
static const float FILTER_GAS_SLOW    = 0.15f;  // газ — медленная компонента

// ── Кривые педалей ──
double defaultcoefficients1[11] = { 0,6,8,13,22,37,62,78,88,94,100 };
double defaultcoefficients2[11] = { 0,10,20,30,40,50,60,70,80,90,100 };
double coefficients1[11]        = { 0,6,8,13,22,37,62,78,88,94,100 };
double coefficients2[11]        = { 0,10,20,30,40,50,60,70,80,90,100 };

// ── Flywheel параметры (% в секунду для rate limiter) ──
float riseLoaded         = 250.0f;  // нарастание под нагрузкой
float fallLoaded         = 250.0f;  // спад под нагрузкой
float riseUnloaded       = 500.0f;  // нарастание без нагрузки
float fallUnloaded       = 500.0f;  // спад без нагрузки
float hangDuration       = 500.0f;  // мс зависания оборотов
float hangThreshold      = 15.0f;   // скорость выжима для срабатывания
float snapBoost          = 1.32f;   // множитель всплеска при броске
float snapThreshold      = 378.0f;  // скорость отпускания сцепления для срабатывания snap
float snapDuration       = 500.0f;  // мс — как долго спадает от snapBoost до 1.0

// ── Аварийный режим ──
bool emergencyMode = false;

// ── OTA ──
bool     otaInProgress       = false;
size_t   otaExpectedSize      = 0;
// Отложенный notify — нельзя вызывать notify() внутри onWrite() callback
volatile bool otaNotifyPending = false;
String        otaNotifyMsg     = "";

// ── Rate limiter состояние ──
float    currentOutput = 0.0f;
float    prevClutchPct = 0.0f;
float    snapLevel     = 1.0f;   // текущий уровень snap (1.0 = нет эффекта)
uint32_t hangTimer     = 0;
bool     hangActive    = false;
uint32_t lastLoopTime  = 0;
uint8_t  bleNotifySkip  = 0;     // счётчик пропуска BLE notify
uint8_t  serialPrintSkip = 0;   // счётчик пропуска Serial print

// ════════════════════════════════════════════════════════
//  NVS
// ════════════════════════════════════════════════════════

void initNVS() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }
}

void writeIntToNVS(const char* key, int value) {
  nvs_handle_t h;
  nvs_open("storage", NVS_READWRITE, &h);
  nvs_set_i32(h, key, (int32_t)value);
  nvs_commit(h);
  nvs_close(h);
}

int readIntFromNVS(const char* key) {
  nvs_handle_t h;
  int32_t value = 0;
  nvs_open("storage", NVS_READWRITE, &h);
  nvs_get_i32(h, key, &value);
  nvs_close(h);
  return (int)value;
}

void writeArrayToNVS(const char* key, const double* array, size_t size) {
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_blob(h, key, array, size * sizeof(double));
    nvs_commit(h);
    nvs_close(h);
  }
}

void readArrayFromNVS(const char* key, double* array, size_t size) {
  nvs_handle_t h;
  size_t required_size = size * sizeof(double);
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    esp_err_t err = nvs_get_blob(h, key, array, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) memset(array, 0, size * sizeof(double));
    nvs_close(h);
  }
}

// ── Flywheel NVS ──
struct FlywheelParams {
  float riseLoaded, fallLoaded, riseUnloaded, fallUnloaded;
  float hangDuration, hangThreshold;
  float snapBoost, snapDuration;
  float snapThreshold;
};

void saveFlywheelParams() {
  FlywheelParams p = { riseLoaded, fallLoaded, riseUnloaded, fallUnloaded,
                       hangDuration, hangThreshold, snapBoost, snapDuration, snapThreshold };
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_blob(h, "flywheelP", &p, sizeof(p));
    nvs_commit(h);
    nvs_close(h);
    Serial.println("[NVS] Flywheel params saved.");
  }
}

void loadFlywheelParams() {
  FlywheelParams p = { riseLoaded, fallLoaded, riseUnloaded, fallUnloaded,
                       hangDuration, hangThreshold, snapBoost, snapDuration, snapThreshold };
  nvs_handle_t h;
  if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
    size_t size = sizeof(p);
    if (nvs_get_blob(h, "flywheelP", &p, &size) == ESP_OK) {
      riseLoaded   = p.riseLoaded;   fallLoaded   = p.fallLoaded;
      riseUnloaded = p.riseUnloaded; fallUnloaded = p.fallUnloaded;
      hangDuration  = p.hangDuration;  hangThreshold = p.hangThreshold;
      snapBoost     = p.snapBoost;     snapDuration  = p.snapDuration;
      if (size >= sizeof(p)) snapThreshold = p.snapThreshold;
      Serial.println("[NVS] Flywheel params loaded.");
    } else {
      Serial.println("[NVS] No flywheel params, using defaults.");
    }
    nvs_close(h);
  }
}

// ════════════════════════════════════════════════════════
//  Фильтры
// ════════════════════════════════════════════════════════

void initFilters() {
  analogHall.setCoef(FILTER_CLUTCH);
  analog0.setCoef(FILTER_GAS_FAST);
  analog1.setCoef(FILTER_GAS_SLOW);
  analog2.setCoef(0.01f);
  analog3.setCoef(0.008f);
  analogHall.setStep(1);
  analog0.setStep(1);
  analog1.setStep(1);
}

void loadDefaultCoefficientsIfNeeded() {
  double s1[11], s2[11];
  readArrayFromNVS("coefficients1", s1, 11);
  readArrayFromNVS("coefficients2", s2, 11);
  bool e1 = true, e2 = true;
  for (int i = 0; i < 11; i++) {
    if (s1[i] != 0.0) e1 = false;
    if (s2[i] != 0.0) e2 = false;
  }
  if (e1) { memcpy(coefficients1, defaultcoefficients1, sizeof(defaultcoefficients1)); writeArrayToNVS("coefficients1", coefficients1, 11); }
  else     { memcpy(coefficients1, s1, sizeof(s1)); }
  if (e2) { memcpy(coefficients2, defaultcoefficients2, sizeof(defaultcoefficients2)); writeArrayToNVS("coefficients2", coefficients2, 11); }
  else     { memcpy(coefficients2, s2, sizeof(s2)); }
}

int readAverageVoltage(int pin, float* readings) {
  int total = 0;
  for (int i = 0; i < numReadings; i++) {
    readings[i] = analogReadMilliVolts(pin);
    total += readings[i];
    delay(1);
  }
  return total / numReadings;
}

// ════════════════════════════════════════════════════════
//  BLE Callbacks
// ════════════════════════════════════════════════════════

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*)    { deviceConnected = true;  Serial.println("[BLE] Connected."); }
  void onDisconnect(BLEServer*) { deviceConnected = false; Serial.println("[BLE] Disconnected."); BLEDevice::startAdvertising(); }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    std::string value = std::string(c->getValue().c_str());
    if (value == "Fully Pressed A0") {
      fullyPressedNVSValueA0 = readAverageVoltage(Hall, readingsA0);
      writeIntToNVS("fullyPressedA0", fullyPressedNVSValueA0);
      sensorCharacteristic->setValue("Fully Pressed A0 Saved");
      sensorCharacteristic->notify();
    }
    if (value == "Fully Released A0") {
      fullyReleasedNVSValueA0 = readAverageVoltage(Hall, readingsA0);
      writeIntToNVS("fullyReleasedA0", fullyReleasedNVSValueA0);
      sensorCharacteristic->setValue("Fully Released A0 Saved");
      sensorCharacteristic->notify();
    }
    if (value == "Fully Pressed A1") {
      fullyPressedNVSValueA1 = readAverageVoltage(G1, readingsA1);
      writeIntToNVS("fullyPressedA1", fullyPressedNVSValueA1);
      sensorA1Characteristic->setValue("Fully Pressed A1 Saved");
      sensorA1Characteristic->notify();
    }
    if (value == "Fully Released A1") {
      fullyReleasedNVSValueA1 = readAverageVoltage(G1, readingsA1);
      writeIntToNVS("fullyReleasedA1", fullyReleasedNVSValueA1);
      sensorA1Characteristic->setValue("Fully Released A1 Saved");
      sensorA1Characteristic->notify();
    }
  }
};

class CoefficientsCallbacks : public BLECharacteristicCallbacks {
  static constexpr size_t N = 22, H = 11;
  double buf[22];

  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() != N * sizeof(double)) return;
    memcpy(buf, c->getData(), N * sizeof(double));
    writeArrayToNVS("coefficients1", buf,     H);
    writeArrayToNVS("coefficients2", buf + H, H);
    memcpy(coefficients1, buf,     H * sizeof(double));
    memcpy(coefficients2, buf + H, H * sizeof(double));
  }
  void onRead(BLECharacteristic* c) override {
    readArrayFromNVS("coefficients1", buf,     H);
    readArrayFromNVS("coefficients2", buf + H, H);
    c->setValue((uint8_t*)buf, N * sizeof(double));
  }
};

class DefaultCoefficientsCallbacks : public BLECharacteristicCallbacks {
  double buf[22];
  void onRead(BLECharacteristic* c) override {
    memcpy(buf,      defaultcoefficients1, 11 * sizeof(double));
    memcpy(buf + 11, defaultcoefficients2, 11 * sizeof(double));
    c->setValue((uint8_t*)buf, 22 * sizeof(double));
  }
};


// ── Emergency mode NVS ──
void saveEmergencyMode() {
  writeIntToNVS("emergencyMode", emergencyMode ? 1 : 0);
}
void loadEmergencyMode() {
  emergencyMode = readIntFromNVS("emergencyMode") != 0;
}

class EmergencyModeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() < 1) return;
    emergencyMode = c->getData()[0] != 0;
    saveEmergencyMode();
    Serial.printf("[BLE] Emergency mode: %s\n", emergencyMode ? "ON" : "OFF");
  }
  void onRead(BLECharacteristic* c) override {
    uint8_t val = emergencyMode ? 1 : 0;
    c->setValue(&val, 1);
  }
};

// Универсальный callback для float-параметров маховика
class FloatParamCallbacks : public BLECharacteristicCallbacks {
  float* param;
public:
  FloatParamCallbacks(float* p) : param(p) {}
  void onWrite(BLECharacteristic* c) override {
    if (c->getLength() != sizeof(float)) return;
    *param = *((float*)c->getData());
    saveFlywheelParams();
    Serial.printf("[BLE] Flywheel param updated: %.2f\n", *param);
  }
  void onRead(BLECharacteristic* c) override {
    c->setValue((uint8_t*)param, sizeof(float));
  }
};

class OtaCtrlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string val = std::string(c->getValue().c_str());
    if (val.rfind("SIZE:", 0) == 0) {
      otaExpectedSize = (size_t)atoi(val.c_str() + 5);
      if (!Update.begin(otaExpectedSize)) {
        Serial.println("[OTA] begin() failed — check partition scheme");
        otaNotifyMsg = "ERROR:begin failed"; otaNotifyPending = true;
        return;
      }
      otaInProgress = true;
      Serial.printf("[OTA] Started, %d bytes\n", (int)otaExpectedSize);
      otaNotifyMsg = "READY"; otaNotifyPending = true;
    } else if (val == "APPLY") {
      otaInProgress = false;
      if (Update.end(true)) {
        Serial.println("[OTA] Done, rebooting...");
        otaNotifyMsg = "DONE"; otaNotifyPending = true;
      } else {
        Serial.printf("[OTA] end() error: %s\n", Update.errorString());
        otaNotifyMsg = String("ERROR:") + Update.errorString();
        otaNotifyPending = true;
      }
    }
  }
};

class OtaDataCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (!otaInProgress || c->getLength() == 0) return;
    if (Update.write(c->getData(), c->getLength()) != c->getLength()) {
      otaInProgress = false;
      otaCtrlChar->setValue("ERROR:write failed");
      otaCtrlChar->notify();
    }
  }
};

// ════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  pinMode(Hall, INPUT);
  pinMode(G1,   INPUT);

  initDacs();

  initNVS();
  initFilters();
  loadFlywheelParams();
  loadEmergencyMode();
  loadDefaultCoefficientsIfNeeded();

  // BLE
  BLEDevice::init("Stark_Clutch");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEDevice::setMTU(512);

  BLEService* pService = pServer->createService(
    BLEUUID("19b10000-e8f2-537e-4f6c-d104768a1214"), 64, 1);

  // Существующие характеристики
  commandCharacteristic = pService->createCharacteristic(
    "19b10002-e8f2-537e-4f6c-d104768a1214", BLECharacteristic::PROPERTY_WRITE);
  commandCharacteristic->setCallbacks(new CommandCallbacks());

  sensorCharacteristic = pService->createCharacteristic(
    "19b10001-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  sensorCharacteristic->addDescriptor(new BLE2902());

  sensorA1Characteristic = pService->createCharacteristic(
    "19b10003-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  sensorA1Characteristic->addDescriptor(new BLE2902());

  coefsCharacteristic = pService->createCharacteristic(
    "19b10004-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  coefsCharacteristic->setCallbacks(new CoefficientsCallbacks());

  defCoefsCharacteristic = pService->createCharacteristic(
    "19b10005-e8f2-537e-4f6c-d104768a1214", BLECharacteristic::PROPERTY_READ);
  defCoefsCharacteristic->setCallbacks(new DefaultCoefficientsCallbacks());

  // Flywheel v2 характеристики
  fwRiseLoadedChar = pService->createCharacteristic(
    "19b1000a-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwRiseLoadedChar->setCallbacks(new FloatParamCallbacks(&riseLoaded));

  fwFallLoadedChar = pService->createCharacteristic(
    "19b1000b-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwFallLoadedChar->setCallbacks(new FloatParamCallbacks(&fallLoaded));

  fwRiseUnloadedChar = pService->createCharacteristic(
    "19b1000c-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwRiseUnloadedChar->setCallbacks(new FloatParamCallbacks(&riseUnloaded));

  fwFallUnloadedChar = pService->createCharacteristic(
    "19b1000d-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwFallUnloadedChar->setCallbacks(new FloatParamCallbacks(&fallUnloaded));

  fwHangDurChar = pService->createCharacteristic(
    "19b10011-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwHangDurChar->setCallbacks(new FloatParamCallbacks(&hangDuration));

  fwHangThrChar = pService->createCharacteristic(
    "19b10012-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwHangThrChar->setCallbacks(new FloatParamCallbacks(&hangThreshold));

  fwSnapBoostChar = pService->createCharacteristic(
    "19b10013-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwSnapBoostChar->setCallbacks(new FloatParamCallbacks(&snapBoost));

  fwSnapThrChar = pService->createCharacteristic(
    "19b10014-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwSnapThrChar->setCallbacks(new FloatParamCallbacks(&snapThreshold));

  fwSnapFallChar = pService->createCharacteristic(
    "19b10015-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  fwSnapFallChar->setCallbacks(new FloatParamCallbacks(&snapDuration));

  emergencyModeChar = pService->createCharacteristic(
    "19b10016-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  emergencyModeChar->setCallbacks(new EmergencyModeCallbacks());

  // Версия прошивки + OTA
  fwVersionChar = pService->createCharacteristic(
    "19b10020-e8f2-537e-4f6c-d104768a1214", BLECharacteristic::PROPERTY_READ);
  fwVersionChar->setValue(FW_VERSION);

  otaCtrlChar = pService->createCharacteristic(
    "19b10021-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  otaCtrlChar->addDescriptor(new BLE2902());
  otaCtrlChar->setCallbacks(new OtaCtrlCallbacks());

  otaDataChar = pService->createCharacteristic(
    "19b10022-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_WRITE);
  otaDataChar->setCallbacks(new OtaDataCallbacks());

  pService->start();
  BLEDevice::startAdvertising();

  // Загружаем калибровку из NVS
  fullyPressedNVSValueA0  = readIntFromNVS("fullyPressedA0");
  fullyReleasedNVSValueA0 = readIntFromNVS("fullyReleasedA0");
  fullyPressedNVSValueA1  = readIntFromNVS("fullyPressedA1");
  fullyReleasedNVSValueA1 = readIntFromNVS("fullyReleasedA1");

  lastLoopTime = millis();

  Serial.printf("[OK] Stark Clutch v%s ready.\n", FW_VERSION);
  Serial.printf("  Clutch:   pressed=%d mV  released=%d mV\n", fullyPressedNVSValueA0, fullyReleasedNVSValueA0);
  Serial.printf("  Throttle: pressed=%d mV  released=%d mV\n", fullyPressedNVSValueA1, fullyReleasedNVSValueA1);
}

// ════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════

void loop() {

  // ── dt ──
  uint32_t now = millis();
  float dt = (now - lastLoopTime) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f)  dt = 0.05f;   // защита от скачков при BLE операциях
  lastLoopTime = now;

  // Отложенный OTA notify — вызываем из loop(), не из BLE callback
  if (otaNotifyPending) {
    otaCtrlChar->setValue(otaNotifyMsg.c_str());
    otaCtrlChar->notify();
    otaNotifyPending = false;
    // После DONE — перезагрузка
    if (otaNotifyMsg == "DONE") { delay(300); ESP.restart(); }
  }

  // Во время OTA не трогаем DAC и аналоговые входы
  if (otaInProgress) { delay(10); return; }

  // ── 1. Чтение и шумовой фильтр ──
  int rawA0  = analogReadMilliVolts(Hall);
  int rawA1  = analogReadMilliVolts(G1);

  int filtA0  = analogHall.filteredTime(rawA0);
  int filtA11 = analog0.filteredTime(rawA1);
  int filtA12 = analog1.filteredTime(rawA1);
  int filtA1  = max(filtA11, filtA12);

  // ── 2. Нормализация 0–100% ──
  int rangeA0 = fullyPressedNVSValueA0 - fullyReleasedNVSValueA0;
  int rangeA1 = fullyPressedNVSValueA1 - fullyReleasedNVSValueA1;

  float clutchPct = (rangeA0 != 0) ?
    constrain((float)(filtA0 - fullyReleasedNVSValueA0) / rangeA0 * 100.0f, 0.0f, 100.0f) : 0.0f;
  float gasPct = (rangeA1 != 0) ?
    constrain((float)(filtA1 - fullyReleasedNVSValueA1) / rangeA1 * 100.0f, 0.0f, 100.0f) : 0.0f;

  // ── АВАРИЙНЫЙ РЕЖИМ: сцепление → газ, плавно, макс 20% ──
  float voltage3;
  float outputFinal;
  if (emergencyMode) {
    float emergencyTarget = clutchPct * 0.20f;  // 0..20%
    if (emergencyTarget > currentOutput)
      currentOutput = min(currentOutput + 5.0f * dt, emergencyTarget);   // 5%/сек нарастание
    else
      currentOutput = max(currentOutput - 100.0f * dt, emergencyTarget); // быстрое падение
    outputFinal = currentOutput;
    voltage3 = 1000.0f + 3000.0f * (outputFinal / 100.0f);
    dacSetVoltage((uint16_t)(voltage3 * 0.79f));

  } else {
    // ── 3. Кривые педалей (сплайн) ──
    float splineClutch = constrain(clutchPct / 10.0f, 0.0f, 10.0f);
    float splineGas    = constrain(gasPct    / 10.0f, 0.0f, 10.0f);

    double clutchCurve = Interpolation::ConstrainedSpline(x, coefficients1, numValues, splineClutch);
    double gasCurve    = Interpolation::ConstrainedSpline(x, coefficients2, numValues, splineGas);

    float gasTarget = (float)gasCurve;

    // ── 4. Скорость движения рычага сцепления ──
    float clutchDelta = clutchPct - prevClutchPct;
    prevClutchPct = clutchPct;

    // ── 5. Зависание оборотов ──
    if (clutchDelta > hangThreshold) {
      hangTimer = now + (uint32_t)hangDuration;
      hangActive = true;
    }
    if (hangActive && (int32_t)(now - hangTimer) >= 0) hangActive = false;

    // ── 6. Бросок сцепления ──
    if (-clutchDelta > snapThreshold && currentOutput > 10.0f) {
      snapLevel = snapBoost;
    }

    // ── 7. Rate limiter ──
    float t          = clutchPct / 100.0f;
    float actualRise = riseLoaded  + (riseUnloaded  - riseLoaded)  * t;
    float actualFall = fallLoaded  + (fallUnloaded  - fallLoaded)  * t;

    if (hangActive) actualFall *= 0.05f;

    if (gasTarget > currentOutput)
      currentOutput = min(currentOutput + actualRise * dt, gasTarget);
    else
      currentOutput = max(currentOutput - actualFall * dt, gasTarget);

    // ── 8. Snap boost ──
    if (snapLevel > 1.0f) {
      float decayRate = (snapBoost - 1.0f) / (snapDuration / 1000.0f);
      snapLevel -= decayRate * dt;
      if (snapLevel < 1.0f) snapLevel = 1.0f;
    }
    outputFinal = min(currentOutput * snapLevel, 100.0f);

    // ── 9. DAC ──
    float clutchEffect = (100.0f - (float)clutchCurve) / 100.0f;
    voltage3 = 1000.0f + 3000.0f * (outputFinal / 100.0f) * clutchEffect;
    dacSetVoltage((uint16_t)(voltage3 * 0.79f));
  }

  // ── 10. BLE notify ──
  // S3: BLE на Core 0 — notify() не блокирует управление, отправляем каждый цикл (~200Hz)
  // C3: одно ядро — раз в 20 циклов (~10Hz) чтобы не тормозить управление
#ifdef CONFIG_IDF_TARGET_ESP32S3
  if (deviceConnected) {
#else
  if (deviceConnected && ++bleNotifySkip >= 20) {
    bleNotifySkip = 0;
#endif
    String data = String(filtA0)  + "," + String(fullyPressedNVSValueA0) + "," +
                  String(fullyReleasedNVSValueA0) + "," +
                  String(filtA1)  + "," + String(fullyPressedNVSValueA1) + "," +
                  String(fullyReleasedNVSValueA1) + "," +
                  String(outputFinal, 1);
    sensorCharacteristic->setValue(data.c_str());
    sensorCharacteristic->notify();
    sensorA1Characteristic->setValue(data.c_str());
    sensorA1Characteristic->notify();
  }

  // ── 11. Serial Plotter — раз в 10 циклов (~20Hz), не блокирует управление ──
  if (++serialPrintSkip >= 10) {
    serialPrintSkip = 0;
    Serial.print("A0:");        Serial.print(filtA0);
    Serial.print(",A0_max:");   Serial.print(fullyPressedNVSValueA0);
    Serial.print(",A0_min:");   Serial.print(fullyReleasedNVSValueA0);
    Serial.print(",A1:");       Serial.print(filtA1);
    Serial.print(",A1_max:");   Serial.print(fullyPressedNVSValueA1);
    Serial.print(",A1_min:");   Serial.print(fullyReleasedNVSValueA1);
    int dacScaled = fullyReleasedNVSValueA1 +
      (int)((fullyPressedNVSValueA1 - fullyReleasedNVSValueA1) * (outputFinal / 100.0f));
    Serial.print(",DAC:");      Serial.print(dacScaled);
    Serial.print(",FW:");       Serial.println(FW_VERSION);
  }

  delay(5);
}
