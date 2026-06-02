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

GFilterRA analogHall;
GFilterRA analog0;
GFilterRA analog1;
GFilterRA analog2;
GFilterRA analog3;

Adafruit_MCP4725 dac;

double x[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
double y[] = { 0, 6, 14, 30, 54, 71, 81, 86, 90, 94, 100 };
double z[] = { 0, 18, 28, 37, 45, 53, 61, 70, 80, 90, 100 };
int numValues = sizeof(x) / sizeof(x[0]);

BLEServer* pServer = NULL;
BLECharacteristic* commandCharacteristic = NULL;
BLECharacteristic* sensorCharacteristic = NULL;
BLECharacteristic* sensorA1Characteristic = NULL;
BLECharacteristic* coefsCharacteristic = NULL;
BLECharacteristic* defCoefsCharacteristic = NULL;
BLECharacteristic* analogCoef1Characteristic = NULL;
BLECharacteristic* analogCoef2Characteristic = NULL;
BLECharacteristic* analogStep1Characteristic = NULL;
BLECharacteristic* analogStep2Characteristic = NULL;


int Hall = A0;
int G1 = A2; 
int G2 = A2;
const int numReadings = 1;
int fullyPressedNVSValueA0 = 0;
int fullyReleasedNVSValueA0 = 0;
int fullyPressedNVSValueA1 = 500;
int fullyReleasedNVSValueA1 = 2000;
float readingsA0[numReadings];
float readingsA1[numReadings];
bool deviceConnected = false;

float analogCoefHall = 0.8;
float analogCoef1 = 0;
float analogCoef2 = 0;
float analogCoef3 = 0.01;
uint16_t analogStepHall = 1;
uint16_t analogStep1 = 1;
uint16_t analogStep2 = 80;
uint16_t analogStep3 = 1;
float analogStepH = 0.8;



// Массив коэффициентов по умолчанию для Clutch
double defaultcoefficients1[11] = { 0, 6, 14, 30, 54, 71, 81, 86, 90, 94, 100  };
// Массив коэффициентов по умолчанию для Gas
double defaultcoefficients2[11] = { 0, 18, 28, 37, 45, 53, 61, 70, 80, 90, 100 };
double coefficients1[11] = { 0, 6, 14, 30, 54, 71, 81, 86, 90, 94, 100  };
double coefficients2[11] = { 0, 18, 28, 37, 45, 53, 61, 70, 80, 90, 100 };


// Инициализация NVS
void initNVS() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }
}

// Запись значения в NVS
void writeIntToNVS(const char* key, int value) {
  nvs_handle_t nvs_handle;
  nvs_open("storage", NVS_READWRITE, &nvs_handle);
  nvs_set_i32(nvs_handle, key, (int32_t)value);
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
}

// Чтение значения из NVS
int readIntFromNVS(const char* key) {
  nvs_handle_t nvs_handle;
  int32_t value = 0;  // Используем int32_t для совместимости
  nvs_open("storage", NVS_READWRITE, &nvs_handle);
  nvs_get_i32(nvs_handle, key, &value);
  nvs_close(nvs_handle);
  return (int)value;
}

void writeArrayToNVS(const char* key, const double* array, size_t size) {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  if (err == ESP_OK) {
    err = nvs_set_blob(nvs_handle, key, array, size * sizeof(double));
    if (err == ESP_OK) {
      nvs_commit(nvs_handle);
      Serial.println("Array successfully saved to NVS.");
    } else {
      Serial.printf("Error saving array to NVS: %s\n", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
  } else {
    Serial.printf("Error opening NVS: %s\n", esp_err_to_name(err));
  }
}

void readArrayFromNVS(const char* key, double* array, size_t size) {
  nvs_handle_t nvs_handle;
  size_t required_size = size * sizeof(double);
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  if (err == ESP_OK) {
    err = nvs_get_blob(nvs_handle, key, array, &required_size);
    if (err == ESP_OK) {
      Serial.println("Array successfully read from NVS.");
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
      Serial.println("No data found in NVS, filling array with zeros.");
      memset(array, 0, size * sizeof(double)); // Если данных нет, заполняем нулями
    } else {
      Serial.printf("Error reading array from NVS: %s\n", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
  } else {
    Serial.printf("Error opening NVS: %s\n", esp_err_to_name(err));
  }
}

// Функция для вычисления среднего значения с датчика
int readAverageVoltage(int sensorPin, float* readings) {
  int total = 0;
  for (int i = 0; i < numReadings; i++) {
    readings[i] = analogReadMilliVolts(sensorPin);  // Преобразование значения в милливольты
    total += readings[i];
    delay(1);  // Задержка для стабильности чтения
  }
  return total / numReadings;  // Возвращаем среднее значение
}


void setAnalogCoefs() {
  analogHall.setCoef(0.01*analogStep2);
  analog0.setCoef(0.31-analogCoef1);
  analog1.setCoef(0.31-analogCoef2);
  analog2.setCoef(0.01);
  analog3.setCoef(0.008);
  analogHall.setStep(analogStepHall);
  analog0.setStep(1);
  analog1.setStep(1);
  analog2.setStep(1);
  analog3.setStep(1);
}

void loadAnalogCoefs() {
  // Define a structure that matches the one used for saving the coefficients
  struct AnalogCoefs {
    float coef1;
    float coef2;
    uint16_t step1;
    uint16_t step2;
  };

  AnalogCoefs analogCoefs;

  // Open NVS for reading
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);

  if (err == ESP_OK) {
    // Try to read the blob from NVS
    size_t required_size = sizeof(analogCoefs);
    err = nvs_get_blob(nvs_handle, "analogCoefs", &analogCoefs, &required_size);

    if (err == ESP_OK) {
      // Successfully read the data, now set the coefficients
      analogCoef1 = analogCoefs.coef1;
      analogCoef2 = analogCoefs.coef2;
      analogStep1 = analogCoefs.step1;
      analogStep2 = analogCoefs.step2;

      // Call the function to set the loaded coefficients
      Serial.println("Analog coefficients loaded from NVS.");
    } else {
      // Handle error when reading the coefficients
      Serial.printf("Error reading analog coefficients from NVS: %s\n", esp_err_to_name(err));
    }

    // Close NVS handle
    nvs_close(nvs_handle);
  } else {
    // Handle error when opening NVS
    Serial.printf("Error opening NVS: %s\n", esp_err_to_name(err));
  }
  setAnalogCoefs();
}

void saveAnalogCoefs() {
  // Create a structure to hold all the coefficients
  struct AnalogCoefs {
    float coef1;
    float coef2;
    uint16_t step1;
    uint16_t step2;
  };

  // Initialize the structure with the current values
  AnalogCoefs analogCoefs = {analogCoef1, analogCoef2, analogStep1, analogStep2};

  // Open NVS for writing
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  
  if (err == ESP_OK) {
    // Write the structure to NVS
    err = nvs_set_blob(nvs_handle, "analogCoefs", &analogCoefs, sizeof(analogCoefs));
    
    if (err == ESP_OK) {
      nvs_commit(nvs_handle);
      Serial.println("Analog coefficients saved to NVS.");
    } else {
      Serial.printf("Error saving analog coefficients to NVS: %s\n", esp_err_to_name(err));
    }
    
    // Close NVS handle
    nvs_close(nvs_handle);
  } else {
    Serial.printf("Error opening NVS: %s\n", esp_err_to_name(err));
  }
}


void loadDefaultCoefficientsIfNeeded() {
  double storedCoefficients1[11];
  double storedCoefficients2[11];

  // Attempt to read coefficients from NVS
  readArrayFromNVS("coefficients1", storedCoefficients1, 11);
  readArrayFromNVS("coefficients2", storedCoefficients2, 11);

  // Check if the arrays in NVS are filled with valid data
  bool coefficients1Empty = true;
  bool coefficients2Empty = true;

  for (int i = 0; i < 11; i++) {
    if (storedCoefficients1[i] != 0.0) coefficients1Empty = false;
    if (storedCoefficients2[i] != 0.0) coefficients2Empty = false;
  }

  // Load defaults if arrays are empty
  if (coefficients1Empty) {
    Serial.println("No coefficients1 found in NVS, loading defaults.");
    memcpy(coefficients1, defaultcoefficients1, sizeof(defaultcoefficients1));
    writeArrayToNVS("coefficients1", coefficients1, 11);
  } else {
    Serial.println("Coefficients1 found in NVS.");
    memcpy(coefficients1, storedCoefficients1, sizeof(storedCoefficients1));
  }

  if (coefficients2Empty) {
    Serial.println("No coefficients2 found in NVS, loading defaults.");
    memcpy(coefficients2, defaultcoefficients2, sizeof(defaultcoefficients2));
    writeArrayToNVS("coefficients2", coefficients2, 11);
  } else {
    Serial.println("Coefficients2 found in NVS.");
    memcpy(coefficients2, storedCoefficients2, sizeof(storedCoefficients2));
  }
}

// Класс для обработки команд записи через BLE
class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = std::string(pCharacteristic->getValue().c_str());  // Получаем команду от клиента

    // Если команда "Fully Pressed" для A0, сохраняем значение в NVS
    if (value == "Fully Pressed A0") {
      fullyPressedNVSValueA0 = readAverageVoltage(Hall, readingsA0);  // Считываем среднее значение с A0
      writeIntToNVS("fullyPressedA0", fullyPressedNVSValueA0);               // Записываем в NVS
      Serial.print("Fully Pressed Value A0 Saved to NVS: ");
      Serial.println(fullyPressedNVSValueA0);

      // Подтверждаем запись через BLE
      sensorCharacteristic->setValue("Fully Pressed A0 Saved");
      sensorCharacteristic->notify();
    }

    // Если команда "Fully Released" для A0, сохраняем значение в NVS
    if (value == "Fully Released A0") {
      fullyReleasedNVSValueA0 = readAverageVoltage(Hall, readingsA0);  // Считываем среднее значение с A0
      writeIntToNVS("fullyReleasedA0", fullyReleasedNVSValueA0);              // Записываем в NVS
      Serial.print("Fully Released Value A0 Saved to NVS: ");
      Serial.println(fullyReleasedNVSValueA0);

      // Подтверждаем запись через BLE
      sensorCharacteristic->setValue("Fully Released A0 Saved");
      sensorCharacteristic->notify();
    }

    // Если команда "Fully Pressed" для A1, сохраняем значение в NVS
    if (value == "Fully Pressed A1") {
      fullyPressedNVSValueA1 = readAverageVoltage(G1, readingsA1);  // Считываем среднее значение с A1
      writeIntToNVS("fullyPressedA1", fullyPressedNVSValueA1);               // Записываем в NVS
      Serial.print("Fully Pressed Value A1 Saved to NVS: ");
      Serial.println(fullyPressedNVSValueA1);

      // Подтверждаем запись через BLE
      sensorA1Characteristic->setValue("Fully Pressed A1 Saved");
      sensorA1Characteristic->notify();
    }

    // Если команда "Fully Released" для A1, сохраняем значение в NVS
    if (value == "Fully Released A1") {
      fullyReleasedNVSValueA1 = readAverageVoltage(G1, readingsA1);  // Считываем среднее значение с A1
      writeIntToNVS("fullyReleasedA1", fullyReleasedNVSValueA1);              // Записываем в NVS
      Serial.print("Fully Released Value A1 Saved to NVS: ");
      Serial.println(fullyReleasedNVSValueA1);

      // Подтверждаем запись через BLE
      sensorA1Characteristic->setValue("Fully Released A1 Saved");
      sensorA1Characteristic->notify();
    }
  }
};

// Обработчик событий соединения
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;  // Устанавливаем флаг соединения
    Serial.println("Device connected.");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;  // Сбрасываем флаг при разрыве соединения
    Serial.println("Device disconnected. Restarting advertisement...");
    BLEDevice::startAdvertising();  // Начинаем снова рекламу после разрыва соединения
  }
};

class CoefficientsCallbacks : public BLECharacteristicCallbacks {
private:
    static constexpr size_t ARRAY_SIZE = 22;
    static constexpr size_t HALF_ARRAY_SIZE = ARRAY_SIZE / 2;

    double coefficients[ARRAY_SIZE]; // Array to hold coefficients

    void handleWriteCommand(const uint8_t* value, size_t size) {
        // Ensure the received data size matches the expected size
        if (size != ARRAY_SIZE * sizeof(double)) {
            Serial.printf("Invalid data size: expected %zu bytes, received %zu bytes\n", ARRAY_SIZE * sizeof(double), size);
            return;
        }

        // Copy the data into the coefficients array
        memcpy(coefficients, value, size);
        Serial.println("Received all coefficients:");
        printArrayToSerial(coefficients, ARRAY_SIZE);

        // Split and save the coefficients to NVS
        writeArrayToNVS("coefficients1", coefficients, HALF_ARRAY_SIZE);
        writeArrayToNVS("coefficients2", coefficients + HALF_ARRAY_SIZE, HALF_ARRAY_SIZE);
        memcpy(coefficients1, coefficients, HALF_ARRAY_SIZE * sizeof(double));
        memcpy(coefficients2, coefficients + HALF_ARRAY_SIZE, HALF_ARRAY_SIZE * sizeof(double));
        Serial.println("Coefficients saved to NVS.");
    }

    void handleReadCommand(BLECharacteristic* pCharacteristic) {
        // Load coefficients from NVS into the array
        readArrayFromNVS("coefficients1", coefficients, HALF_ARRAY_SIZE);
        readArrayFromNVS("coefficients2", coefficients + HALF_ARRAY_SIZE, HALF_ARRAY_SIZE);

        // Send all coefficients as a single chunk
        Serial.println("Sending coefficients as a single chunk:");
        printArrayToSerial(coefficients, ARRAY_SIZE);
        pCharacteristic->setValue((uint8_t*)coefficients, ARRAY_SIZE * sizeof(double));
    }

    // Utility function to print an array to Serial
    void printArrayToSerial(const double* array, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            Serial.print(array[i]);
            if (i < size - 1) Serial.print(", ");
        }
        Serial.println();
    }

public:
    void onWrite(BLECharacteristic* pCharacteristic) override {
        const uint8_t* value = pCharacteristic->getData();
        size_t size = pCharacteristic->getLength();

        handleWriteCommand(value, size);
    }

    void onRead(BLECharacteristic* pCharacteristic) override {
        handleReadCommand(pCharacteristic);
    }
};

class DefaultCoefficientsCallbacks : public BLECharacteristicCallbacks {
private:
    static constexpr size_t ARRAY_SIZE = 22;
    static constexpr size_t HALF_ARRAY_SIZE = ARRAY_SIZE / 2;

    double coefficients[ARRAY_SIZE]; // Array to hold coefficients

    void handleReadCommand(BLECharacteristic* pCharacteristic) {
        // Load coefficients from NVS into the array
        memcpy(coefficients, defaultcoefficients1, HALF_ARRAY_SIZE * sizeof(double));
        memcpy(coefficients + HALF_ARRAY_SIZE, defaultcoefficients2, HALF_ARRAY_SIZE * sizeof(double));

        // Send all coefficients as a single chunk
        Serial.println("Sending coefficients as a single chunk:");
        printArrayToSerial(coefficients, ARRAY_SIZE);
        pCharacteristic->setValue((uint8_t*)coefficients, ARRAY_SIZE * sizeof(double));
    }

    // Utility function to print an array to Serial
    void printArrayToSerial(const double* array, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            Serial.print(array[i]);
            if (i < size - 1) Serial.print(", ");
        }
        Serial.println();
    }

public:
    void onRead(BLECharacteristic* pCharacteristic) override {
        handleReadCommand(pCharacteristic);
    }
};

class AnalogCoefCallbacks1 : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* pCharacteristic) override {
        const uint8_t* value = pCharacteristic->getData();
        size_t size = pCharacteristic->getLength();
        if (size != sizeof(float)) {
          Serial.println("Size of analog coef is not right");
          Serial.println(size);
          return;
        }

        analogCoef1 = *((float*)value);
        saveAnalogCoefs();
        setAnalogCoefs();
    }

    void onRead(BLECharacteristic* pCharacteristic) override {
        pCharacteristic->setValue((uint8_t*)&analogCoef1, sizeof(float));
    }
};

class AnalogCoefCallbacks2 : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* pCharacteristic) override {
        const uint8_t* value = pCharacteristic->getData();
        size_t size = pCharacteristic->getLength();
        if (size != sizeof(float)) {
          Serial.println("Size of analog coef is not right");
          Serial.println(size);
          return;
        }

        analogCoef2 = *((float*)value);
        saveAnalogCoefs();
        setAnalogCoefs();
    }

    void onRead(BLECharacteristic* pCharacteristic) override {
        pCharacteristic->setValue((uint8_t*)&analogCoef2, sizeof(float));
    }
};

class AnalogStepCallbacks1 : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* pCharacteristic) override {
        const uint8_t* value = pCharacteristic->getData();
        size_t size = pCharacteristic->getLength();
        if (size != sizeof(uint16_t)) {
          Serial.println("Size of analog coef is not right");
          return;
        }

        analogStep1 = *((uint16_t*)value);
        saveAnalogCoefs();
        setAnalogCoefs();
    }

    void onRead(BLECharacteristic* pCharacteristic) override {
        pCharacteristic->setValue((uint8_t*)&analogStep1, sizeof(uint16_t));
    }
};

class AnalogStepCallbacks2 : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* pCharacteristic) override {
        const uint8_t* value = pCharacteristic->getData();
        size_t size = pCharacteristic->getLength();
        if (size != sizeof(uint16_t)) {
          Serial.println("Size of analog coef is not right");
          return;
        }

        analogStep2 = *((uint16_t*)value);
        
        saveAnalogCoefs();
        setAnalogCoefs();
    }

    void onRead(BLECharacteristic* pCharacteristic) override {
        pCharacteristic->setValue((uint8_t*)&analogStep2, sizeof(uint16_t));
    }
};

void setup() {
  Serial.begin(115200);         // Инициализация серийного порта
  pinMode(Hall, INPUT);  // Устанавливаем пин A0 как вход
  pinMode(G1, INPUT);  // Устанавливаем пин A1 как вход

  dac.begin(0x60);
 // dac.begin(0x60); //для старой версии
 // dac.begin(0x62); // первая партия 9129350a
 // dac.begin(0x60); //вторая партия

  // Инициализация NVS
  initNVS();

  loadAnalogCoefs();

  // Load default coefficients if needed
  loadDefaultCoefficientsIfNeeded();
  // Инициализация BLE
  BLEDevice::init("Stark_Clutch");
  pServer = BLEDevice::createServer();
  BLEDevice::setMTU(512); 
  // Создание BLE сервиса
  BLEService* pService = pServer->createService(BLEUUID("19b10000-e8f2-537e-4f6c-d104768a1214"), 128, 1);

  // Создание BLE характеристики для команд
  commandCharacteristic = pService->createCharacteristic(
    "19b10002-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_WRITE);
  commandCharacteristic->setCallbacks(new CommandCallbacks());  // Устанавливаем обработчик команд

  // Создание BLE характеристики для данных с A0
  sensorCharacteristic = pService->createCharacteristic(
    "19b10001-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  sensorCharacteristic->addDescriptor(new BLE2902());  // Добавляем дескриптор для уведомлений

  // Создание BLE характеристики для данных с A1
  sensorA1Characteristic = pService->createCharacteristic(
    "19b10003-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  sensorA1Characteristic->addDescriptor(new BLE2902());  // Добавляем дескриптор для уведомлений

  // Создание BLE характеристики для данных с A1
  coefsCharacteristic = pService->createCharacteristic(
    "19b10004-e8f2-537e-4f6c-d104768a1214",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
   coefsCharacteristic->setCallbacks(new CoefficientsCallbacks());

  defCoefsCharacteristic = pService->createCharacteristic(
  "19b10005-e8f2-537e-4f6c-d104768a1214",
  BLECharacteristic::PROPERTY_READ);
  defCoefsCharacteristic->setCallbacks(new DefaultCoefficientsCallbacks());

  analogCoef1Characteristic = pService->createCharacteristic(
  "19b10006-e8f2-537e-4f6c-d104768a1214",
  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  analogCoef1Characteristic->setCallbacks(new AnalogCoefCallbacks1());

  analogStep1Characteristic = pService->createCharacteristic(
  "19b10007-e8f2-537e-4f6c-d104768a1214",
  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  analogStep1Characteristic->setCallbacks(new AnalogStepCallbacks1());

  analogCoef2Characteristic = pService->createCharacteristic(
  "19b10008-e8f2-537e-4f6c-d104768a1214",
  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  analogCoef2Characteristic->setCallbacks(new AnalogCoefCallbacks2());

  analogStep2Characteristic = pService->createCharacteristic(
  "19b10009-e8f2-537e-4f6c-d104768a1214",
  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  analogStep2Characteristic->setCallbacks(new AnalogStepCallbacks2());

  pService->start();                               // Запускаем сервис
  pServer->setCallbacks(new MyServerCallbacks());  // Добавляем обработчик для соединений
  BLEDevice::startAdvertising();                   // Начинаем рекламу устройства через BLE

  // Чтение значений из NVS
  fullyPressedNVSValueA0 = readIntFromNVS("fullyPressedA0");
  fullyReleasedNVSValueA0 = readIntFromNVS("fullyReleasedA0");
  fullyPressedNVSValueA1 = readIntFromNVS("fullyPressedA1");
  fullyReleasedNVSValueA1 = readIntFromNVS("fullyReleasedA1");

  Serial.println("Initialization complete.");

  // Вывод значений из NVS
  Serial.print("Fully Pressed Value A0 (NVS): ");
  Serial.println(fullyPressedNVSValueA0);
  Serial.print("Fully Released Value A0 (NVS): ");
  Serial.println(fullyReleasedNVSValueA0);
  Serial.print("Fully Pressed Value A1 (NVS): ");
  Serial.println(fullyPressedNVSValueA1);
  Serial.print("Fully Released Value A1 (NVS): ");
  Serial.println(fullyReleasedNVSValueA1);
}

int prevRawVoltageA0 = 0;
int prevRawVoltageA1 = 0;
int rawVoltageA0 = 0;
int rawVoltageA1 = 0;
void loop() {

  // Чтение текущего напряжения с датчика A0 и A1
  //int currentVoltageA0 = analogHall.filteredTime(analogReadMilliVolts(Hall));
  prevRawVoltageA0 = rawVoltageA0;
  rawVoltageA0 = analogReadMilliVolts(Hall);
  int currentVoltageA0 = analogHall.filteredTime(rawVoltageA0);
  prevRawVoltageA1 = rawVoltageA1;
  rawVoltageA1 = analogReadMilliVolts(G1);
  int currentVoltageA11 = analog0.filteredTime(rawVoltageA1);
  int currentVoltageA12 = analog1.filteredTime(rawVoltageA1);
  int currentVoltageA1 = (analogCoef1 < analogCoef2) ? max(currentVoltageA11, currentVoltageA12) : min(currentVoltageA11, currentVoltageA12);//analog1.filteredTime(rawVoltageA1);


  // Формируем строку с данными для отправки на веб-страницу
  String dataToSend = String(currentVoltageA0) + "," + String(fullyPressedNVSValueA0) + "," + String(fullyReleasedNVSValueA0) + ",";
  dataToSend += String(currentVoltageA1) + "," + String(fullyPressedNVSValueA1) + "," + String(fullyReleasedNVSValueA1);

  // Отправка данных через BLE, если устройство подключено
  if (deviceConnected) {
    sensorCharacteristic->setValue(dataToSend.c_str());    // Устанавливаем значение для характеристики A0
    sensorCharacteristic->notify();                        // Отправляем уведомление для A0
    sensorA1Characteristic->setValue(dataToSend.c_str());  // Устанавливаем значение для характеристики A1
    sensorA1Characteristic->notify();                      // Отправляем уведомление для A1
  }
  

  // Map sensor value to percentage range
  float percentage = (float)(currentVoltageA0 - fullyReleasedNVSValueA0) / (fullyPressedNVSValueA0 - fullyReleasedNVSValueA0) * 11;
  percentage = constrain(percentage, 0, 10);  // Ensure percentage stays within bounds
  float percentage2 = ((float)(currentVoltageA1 - fullyReleasedNVSValueA1) / (fullyPressedNVSValueA1 - fullyReleasedNVSValueA1)) * 11;
  //float percentage2 = ((float)(currentVoltageA0 - fullyReleasedNVSValueA0) / (fullyPressedNVSValueA0 - fullyReleasedNVSValueA0)) * 11;
  percentage2 = constrain(percentage2, 0, 10);  // Ensure percentage stays within bounds


  // Perform spline interpolation
  double result = Interpolation::ConstrainedSpline(x, coefficients1, numValues, percentage);
  double result2 = Interpolation::ConstrainedSpline(x, coefficients2, numValues, percentage2);
  //double result2 = Interpolation::ConstrainedSpline(x, coefficients2, numValues, percentage);
  float coefficient = 100 - result;
  float coefficient2 = result2;

  // Optional: Output result to DAC
  
  float add = (3000*coefficient2)/100;
  float voltage3 = 1000 + (add)*coefficient/100;
  dac.setVoltage(voltage3 * 0.79, false, 400000);
  analogStepH = 0.01*analogStep2;
/*
*/
  // Данные для Serial Plotter (только числа)
  Serial.print(currentVoltageA0);
  Serial.print(", ");
  Serial.print(rawVoltageA0);
  Serial.print(", ");
  Serial.print(currentVoltageA1);
  Serial.print(", ");
  Serial.print(currentVoltageA11);
  Serial.print(", ");
  Serial.print(currentVoltageA12);
  Serial.print(", ");

  Serial.print(fullyPressedNVSValueA0);
  Serial.print(", ");
  Serial.print(fullyReleasedNVSValueA0);
  Serial.print(", ");
  Serial.print(fullyPressedNVSValueA1);
  Serial.print(", ");
  Serial.print(fullyReleasedNVSValueA1);
  Serial.println();
// Обязательно println для завершения строки
/*
*/
  // Данные для Serial Monitor (числа и текст)
  Serial.print("voltage3: ");
  Serial.print(voltage3);
  Serial.print(", add: ");
  Serial.print(add);
  Serial.print(", Percentage: ");
  Serial.print(percentage);
  Serial.print(", Percentage2: ");
  Serial.print(percentage2);
  Serial.print(", GAS: ");
  Serial.print(coefficient2, 2);
  Serial.print(", Clutch: ");
  Serial.print(result, 2); 
  
  Serial.print(", analogCoef1: ");
  Serial.print(0.31-analogCoef1, 2);
  Serial.print(", analogStep2: ");
  Serial.print(analogStep2);
  Serial.print(", analogCoef2: ");
  Serial.println(0.31-analogCoef2, 2);
  // Завершаем строку



  delay(5);
}
