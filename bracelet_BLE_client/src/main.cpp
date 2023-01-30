/*
 * For games testing with TWO bracelets
*/
#include <Arduino.h>
#include "BLEDevice.h"


#define LED_RESOLUTION 10
#define LED_FREQ       1
#define LED_1_CH       0
#define LED_2_CH       1
#define LED_1          GPIO_NUM_18 // red
#define LED_2          GPIO_NUM_32 // green


bool connect_to_ble_server(void);
static void bleStringNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
void BLE_enable(void);


/* BLE stuff */
static bool doConnect = false; // переменная, используемая для определения того, нужно ли начинать подключение или завершено ли подключение
volatile bool connected[] = {false, false};
char ble_server_name[2][11] = {"Bracelet_1", "Bracelet_2"};


static BLEUUID bleServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); // UUID сервиса
static BLEUUID bleStringCharacteristicUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // UUID для (данные одной строчки)
static BLERemoteCharacteristic* bleStringCharacteristic; // характеристики, данные которых необходимо считать
BLEClient* pClient[] = {BLEDevice::createClient(), BLEDevice::createClient()};
static BLEAdvertisedDevice* bracelet_server[2];
volatile uint8_t ble_connected_devices_count = 0;


class ClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {}

    void onDisconnect(BLEClient* pclient) {
        if (pclient == pClient[0]) {
            ble_connected_devices_count = 0;
            connected[0] = false;
            ledcWrite(LED_1_CH, 240);
        } else if (pclient == pClient[1]) {
            connected[1] = false;
            ledcWrite(LED_2_CH, 512);
        }
    }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    /** @brief Метод обратного вызова, который будет вызван при получении оповещения от другого устройства
     */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.getName() == ble_server_name[ble_connected_devices_count]) {
            connected[ble_connected_devices_count] = true;
            BLEDevice::getScan()->stop();
            bracelet_server[ble_connected_devices_count] = new BLEAdvertisedDevice(advertisedDevice);
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
    Serial.begin(115200);
    while (!Serial);

    ledcSetup(LED_1_CH, LED_FREQ, LED_RESOLUTION);
    ledcSetup(LED_2_CH, LED_FREQ, LED_RESOLUTION);

    ledcAttachPin(LED_1, LED_1_CH);
    ledcAttachPin(LED_2, LED_2_CH);
    ledcWrite(LED_1_CH, 240);
    ledcWrite(LED_2_CH, 512);

    BLE_enable();
}


void loop() {
    if (doConnect) {
        if (connect_to_ble_server()) { // подключение к серверу
            switch (ble_connected_devices_count) {
                case 0:
                    ledcWrite(LED_1_CH, 1024);
                    break;
                case 1:
                    ledcWrite(LED_2_CH, 1024);
                    break;
            }
        } else {
            // Подключиться к BLE-серверу не получилось
            Serial.println("Connection failed");
        }
        doConnect = false; 
    } else if ((connected[0] & connected[1]) != true) BLEDevice::getScan()->start(0);
}


/****************************************************************************************************************************/

static void bleStringNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
        switch ((*pData) & 0xFE) {
            case 0x50: // info about charge
                Serial.print(*pData); // device id + message id
                Serial.print(',');
                Serial.print(*(++pData)); // value
                Serial.println(',');
                break;
            case 0x90: { // info about buttons and gestures
                    Serial.print(*pData); Serial.print(','); // idx
                    uint8_t gesture_buttons = *(++pData);
                    Serial.print(gesture_buttons & 0x01); Serial.print(',');
                    Serial.print((gesture_buttons & 0x02) >> 1); Serial.print(',');
                    Serial.print((gesture_buttons & 0x04) >> 2); Serial.println(',');
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
}

/**
 * @brief Подключение к BLE-серверу
 * @retval Статус подключения
 */
bool connect_to_ble_server(void) {
    BLEClient* ptr_client_bracelet;

    ptr_client_bracelet = pClient[ble_connected_devices_count];
    ptr_client_bracelet->setClientCallbacks(new ClientCallback());
    ptr_client_bracelet->connect(bracelet_server[ble_connected_devices_count]);

    if (++ble_connected_devices_count > 1) ble_connected_devices_count = 1;

    BLERemoteService* pRemoteService = ptr_client_bracelet->getService(bleServiceUUID); // считываем UUID искомого сервиса

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

/**
 * @brief Инициализация BLE-клиента, сканирование для подключения к BLE-серверу
 */
void BLE_enable(void) {
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
