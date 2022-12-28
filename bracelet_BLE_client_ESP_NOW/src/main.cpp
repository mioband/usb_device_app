#include <Arduino.h>
#include "BLEDevice.h"


#define NRF_DEBUG

#define bleServerName "LARS_Bracelet"
#define LED_RESOLUTION 10
#define LED_FREQ       1
#define LED_1_CH       0
#define LED_2_CH       1
#define LED_1          GPIO_NUM_18 // red
#define LED_2          GPIO_NUM_32 // green


void indicating(void);
bool connectToServer(void);
static void bleStringNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);


static bool doConnect = false; // переменная, используемая для определения того, нужно ли начинать подключение или завершено ли подключение
volatile bool connected = false;
#ifdef NRF_DEBUG
volatile uint8_t board_mode = 0xFF;
#endif

static BLEUUID bleServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); // UUID сервиса
static BLEUUID bleStringCharacteristicUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // UUID для (данные одной строчки)
static BLERemoteCharacteristic* bleStringCharacteristic; // характеристики, данные которых необходимо считать
BLEClient* pClient = BLEDevice::createClient();
static BLEAdvertisedDevice* bracelet_server;


class ClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {}

    void onDisconnect(BLEClient* pclient) {
        connected = false;
    }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    /** @brief Метод обратного вызова, который будет вызван при получении оповещения от другого устройства
     */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.getName() == bleServerName) {
            connected = true;
            BLEDevice::getScan()->stop();
            bracelet_server = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true; // задаем индикатор, дающий понять, что мы готовы подключиться
        }
    }
};

typedef struct {
    uint8_t idx;
    int16_t a_x;
    int16_t a_y;
} imu_data_5B_s;

typedef struct {
    uint8_t idx;
    int16_t a_x;
    int16_t a_y;
    int16_t a_z;
    int16_t w_x;
    int16_t w_y;
    int16_t w_z;
} imu_data_13B_s;


void setup() {
    char input_buf[3];
    Serial.begin(115200);
    while (!Serial);

    ledcSetup(LED_1_CH, LED_FREQ, LED_RESOLUTION);
    ledcSetup(LED_2_CH, LED_FREQ, LED_RESOLUTION);

    ledcAttachPin(LED_1, LED_1_CH);
    ledcAttachPin(LED_2, LED_2_CH);
    ledcWrite(LED_1_CH, 240);
    ledcWrite(LED_2_CH, 512);

#ifdef NRF_DEBUG
    while (board_mode == 0xFF) {
        if (Serial.read() == 0x7E) { // '~'
            Serial.readBytes(input_buf, 3);
            if (!strcmp(input_buf, "DBG")) {
                indicating();
                board_mode = 1;
            } else if (!strcmp(input_buf, "NRM")) {
                indicating();
                board_mode = 0;
            }
        }
    }
#endif

    ledcWrite(LED_1_CH, 240);
    ledcWrite(LED_2_CH, 512);

    // инициализируем BLE-устройство
    BLEDevice::init("ESP32_Client");
    /* Setting the new tx power */
    if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9) == ESP_FAIL) {
        Serial.println("Tx power set failed");
    }

    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    pBLEScan->start(5, false);
}


void loop() {
    if (doConnect) {
        if (connectToServer()) { // подключение к серверу        
            ledcWrite(LED_2_CH, 1024);
            ledcWrite(LED_1_CH, 0);
        } else {
            Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
            //  "Подключиться к серверу не получилось.
            ledcWrite(LED_1_CH, 160);
            ledcWrite(LED_2_CH, 160);
        }
        doConnect = false;
    } else if (!connected) {
        ledcWrite(LED_1_CH, 240);
        ledcWrite(LED_2_CH, 512);
        BLEDevice::getScan()->start(0);
    }
}
/****************************************************************************************************************************/

static void bleStringNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
#ifdef NRF_DEBUG
    if (!board_mode) {
#endif
        switch ((*pData) & 0xFE) {
            case 0x50: // info about charge
                Serial.print(*pData); // device id + message id
                Serial.print(',');
                Serial.print(*(++pData)); // value
                Serial.println(',');
                break;
            case 0x90: { // info about buttons and gestures
                    uint8_t gesture_buttons = *(++pData);
                    Serial.print(gesture_buttons & 0x01); Serial.print(',');
                    Serial.print(gesture_buttons & 0x02); Serial.print(',');
                    Serial.print(gesture_buttons & 0x04); Serial.println(',');
                }
                break;
            case 0x30: { // for only two accelerations
                    imu_data_5B_s tmp_struct;
                    memcpy(&tmp_struct, pData, sizeof(tmp_struct));
                    Serial.print(tmp_struct.idx); Serial.print(',');
                    Serial.print(tmp_struct.a_x); Serial.print(',');
                    Serial.print(tmp_struct.a_y); Serial.println(',');
                }
                break;
            case 0x34: { // for all mpu data
                    imu_data_13B_s tmp_struct;
                    memcpy(&tmp_struct, pData, sizeof(tmp_struct));
                    Serial.print(tmp_struct.idx); Serial.print(',');
                    Serial.print(tmp_struct.a_x); Serial.print(',');
                    Serial.print(tmp_struct.a_y); Serial.print(',');
                    Serial.print(tmp_struct.a_z); Serial.print(',');
                    Serial.print(tmp_struct.w_x); Serial.print(',');
                    Serial.print(tmp_struct.w_y); Serial.print(',');
                    Serial.print(tmp_struct.w_z); Serial.println(',');
                }
        }
#ifdef NRF_DEBUG
    } else if (board_mode == 1) {
        char tmp[50];
        memset(tmp, 0, sizeof(tmp));
        memcpy(tmp, pData, length);
        Serial.print(tmp);
    }
#endif
}

void indicating(void) {
    ledcWrite(LED_1_CH, 1024);
    ledcWrite(LED_2_CH, 1024);
    delay(1100);
}

/**
 * @brief Подключение к BLE-серверу
 * @retval Статус подключения
 */
bool connectToServer(void) {
    pClient->setClientCallbacks(new ClientCallback());

    pClient->connect(bracelet_server);
    BLERemoteService* pRemoteService = pClient->getService(bleServiceUUID); // считываем UUID искомого сервиса

    if (pRemoteService == nullptr) {
        return false;
    }

    bleStringCharacteristic = pRemoteService->getCharacteristic(bleStringCharacteristicUUID); // считываем UUID искомых характеристик

    if (bleStringCharacteristic == nullptr) {
        Serial.print("Failed to find our characteristic UUID");
        return false;
    }

    bleStringCharacteristic->registerForNotify(bleStringNotifyCallback);
    return true;
}
