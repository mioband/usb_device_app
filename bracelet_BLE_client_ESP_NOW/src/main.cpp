/*
 * Connecting to the TWO bracelets v.3
* [ 37418][E][BLERemoteCharacteristic.cpp:289] retrieveDescriptors(): esp_ble_gattc_get_all_descr: Unknown - COMMENTED
*/
#include <Arduino.h>
#include "BLEDevice.h"
#include <esp_now.h>
#include <WiFi.h>

#define MAX_BLE_SERVER_NAME_LEN 100
#define LED_RESOLUTION          10
#define LED_FREQ                1
#define LED_1_CH                0
#define LED_2_CH                1
#define LED_1                   GPIO_NUM_18 // red
#define LED_2                   GPIO_NUM_32 // green


bool connect_to_ble_server(void);
static void bleStringNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
void WiFi_enable(void);
void connect_to_platform(void);
void BLE_enable(void);
bool BLE_address_config(void);
void esp_now_input_handler(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
static void ble_connection_monitorning(void);
static void processing_serial_commands(void);
static void retransmiting_robot_data(void);


/* BLE stuff */
static bool do_ble_connect = false; // переменная, используемая для определения того, нужно ли начинать подключение или завершено ли подключение
volatile bool two_devices_were_connected = false;
volatile bool reconnect[] = {false, false};
char ble_server_name[2][MAX_BLE_SERVER_NAME_LEN] = {"temp_name_0", "temp_name_1"};
char bracelets_hands[] = {'0', '1'};

static BLEUUID bleServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); // UUID сервиса
static BLEUUID bleStringCharacteristicUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // UUID для (данные одной строчки)
static BLERemoteCharacteristic* bleStringCharacteristic; // характеристики, данные которых необходимо считать
BLEClient* pClient[] = {BLEDevice::createClient(), BLEDevice::createClient()};
static BLEAdvertisedDevice* bracelet_server[2];
volatile uint8_t ble_connected_devices_count = 0;

/* ESP_Now stuff */
uint8_t broadcast_platfrom_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peer_info; // creating peer interface
volatile bool platfrom_is_connected = false;
uint8_t us_ir_received_data[11] = {0,};
uint8_t rfid_received_data[6] = {0,};


class ClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {}

    void onDisconnect(BLEClient* pclient) {
        if (pclient == pClient[0]) {
            reconnect[0] = true;
            ble_connected_devices_count = 0;
            ledcWrite(LED_1_CH, 240);
        } else if (pclient == pClient[1]) {
            ledcWrite(LED_2_CH, 512);
            reconnect[1] = true;
        }
    }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    /** @brief Метод обратного вызова, который будет вызван при получении оповещения от другого устройства
     */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.getName() == ble_server_name[ble_connected_devices_count - 1]) {
            reconnect[ble_connected_devices_count - 1] = false;
            BLEDevice::getScan()->stop();
            bracelet_server[ble_connected_devices_count - 1] = new BLEAdvertisedDevice(advertisedDevice);
            do_ble_connect = true; // задаем индикатор, дающий понять, что мы готовы подключиться
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
    uint8_t connection_cmd_flag = 0;
    memset(ble_server_name, '\0', sizeof(ble_server_name));

    Serial.begin(115200);
    while (!Serial);

    ledcSetup(LED_1_CH, LED_FREQ, LED_RESOLUTION);
    ledcSetup(LED_2_CH, LED_FREQ, LED_RESOLUTION);

    ledcAttachPin(LED_1, LED_1_CH);
    ledcAttachPin(LED_2, LED_2_CH);
    ledcWrite(LED_1_CH, 240);
    ledcWrite(LED_2_CH, 512);

    // инициализируем BLE-устройство
    BLEDevice::init("ESP32_Client");
    /* Setting the new tx power */
    if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9) == ESP_FAIL) {
        Serial.println("Tx power set failed");
    }

    WiFi_enable();

    while (!connection_cmd_flag) {
        if (Serial.read() == 0x7E) { // '~'
            switch (Serial.read()) {
                case 0x48: // 'H'
                    connect_to_platform();
                    connection_cmd_flag = 1;
                    break;
                case 0x47: // 'G'
                    if (BLE_address_config()) connection_cmd_flag = 2;
                    break;
            }
        }
    }

    if (connection_cmd_flag == 2) BLE_enable();
}


void loop() {
    // Serial.println(temperatureRead());
    ble_connection_monitorning();
    processing_serial_commands();
    retransmiting_robot_data();
}


/****************************************************************************************************************************/

static void ble_connection_monitorning(void) {
    if (do_ble_connect) {
        if (connect_to_ble_server()) { // подключение к серверу
            Serial.println("GOK");
            switch (ble_connected_devices_count) {
                case 1:
                    ledcWrite(LED_1_CH, 1024);
                    break;
                case 2:
                    ledcWrite(LED_2_CH, 1024);
                    break;
            }
        } else {
            // Подключиться к BLE-серверу не получилось
            Serial.println("GNO");
        }
        do_ble_connect = false;
    } /*else if ((reconnect[0] || reconnect[1]) && two_devices_were_connected) {
        BLEDevice::getScan()->start(0);
    }*/
}

static void processing_serial_commands(void) {
    if (Serial.read() == 0x7E) {
        switch (Serial.read()) {
            case 0x48: // 'H' - connecting to the robotic platform with esp_now interface
                connect_to_platform();
                break;
            case 0x47: // 'G' - connecting to the bracelet via BLE
                if (BLE_address_config()) BLE_enable();
                break;
            /*case 0x49: // 'I' - querring US data from robotic platfrom
                if (platfrom_is_connected) {
                    uint8_t us_number = Serial.read();
                    if (us_number > 0 && us_number < 6) {
                        uint8_t us_querry[2] = {'U', us_number};
                        esp_now_send(broadcast_platfrom_address, us_querry, 2);
                    }

                    delay(3000);
                    if (us_received_data != 0xFF) {
                        char tmp_buf[8];
                        sprintf(tmp_buf, "G,%i,%i", us_number, us_received_data);
                        us_received_data = 0xFF;
                        us_number = 0;
                        Serial.println(tmp_buf);
                    } else {
                        Serial.println("G0");
                    }
                }
                break;*/
            case 0x4A: // changing light state of the robotic platform
                if (platfrom_is_connected) {
                    uint8_t light_state = Serial.read();
                    if (light_state <= 3) {
                        uint8_t light_cmd[] = {0x4A, light_state};
                        esp_now_send(broadcast_platfrom_address, light_cmd, 2);
                    }
                }
                break;
            case 0x4D: // setting the movement direction of the robot
                if (platfrom_is_connected) {
                    uint8_t mov_dir = Serial.read();
                    if (mov_dir == 0x3E || mov_dir == 0x3C) {
                        uint8_t mov_dir_cmd[] = {0x4D, mov_dir};
                        esp_now_send(broadcast_platfrom_address, mov_dir_cmd, 2);
                    }
                }
                break;
            case 0x52: // setting the rotation direction
                if (platfrom_is_connected) {
                    uint8_t rot_dir = Serial.read();
                    if (rot_dir == 0x3E || rot_dir == 0x3C) {
                        uint8_t rotating_cmd[] = {0x52, rot_dir};
                        esp_now_send(broadcast_platfrom_address, rotating_cmd, 2);
                    }
                }
                break;
            case 0x4B: // setting the moving direction with rotation
                if (platfrom_is_connected) {
                    uint8_t dir = Serial.read();
                    if (dir <= 4) {
                        uint8_t moving_cmd[] = {0x4B, dir};
                        esp_now_send(broadcast_platfrom_address, moving_cmd, 2);
                    }
                }
                break;
            case 0x53: // stop command
                if (platfrom_is_connected) {
                    uint8_t stp[] = {0x53, 0x00};
                    esp_now_send(broadcast_platfrom_address, stp, 1);
                }
                break;
            case 0x51: // setting speed of the wheel pairs
                if (platfrom_is_connected) {
                    uint8_t wheels_speed_cmd[3] = {0x51, 0x00, 0x00};
                    Serial.readBytes(&wheels_speed_cmd[1], 2);
                    esp_now_send(broadcast_platfrom_address, wheels_speed_cmd, 3);
                }
                break;
        }
    }
}

static void retransmiting_robot_data(void) {
    if (us_ir_received_data[0] == 0x49) {
        char tmp_buf[45];
        sprintf(tmp_buf, "%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i", us_ir_received_data[0], us_ir_received_data[1], us_ir_received_data[2],
        us_ir_received_data[3], us_ir_received_data[4], us_ir_received_data[5], us_ir_received_data[6], us_ir_received_data[7],
        us_ir_received_data[8], us_ir_received_data[9], us_ir_received_data[10]);
        us_ir_received_data[0] = 0xFF;
        Serial.println(tmp_buf);
    } else if (rfid_received_data[0] == 0x46) {
        char tmp_buf[30];
        sprintf(tmp_buf, "%i,%i,%i,%i,%i,%i", rfid_received_data[0], rfid_received_data[1], rfid_received_data[2],
        rfid_received_data[3], rfid_received_data[4], rfid_received_data[5]);
        rfid_received_data[0] = 0xFF;
        Serial.println(tmp_buf);
    }
}

static void bleStringNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    uint8_t msg_id = *pData;
    char bracelet_id = '\0';

    BLERemoteService* tmp_service = pBLERemoteCharacteristic->getRemoteService();
    if (tmp_service->getClient() == pClient[0]) {
        bracelet_id = bracelets_hands[0];
    } else if (tmp_service->getClient() == pClient[1]) {
        bracelet_id = bracelets_hands[1];
    }

    switch (msg_id) {
        case 0x50: // info about charge
            Serial.print(msg_id); Serial.print(','); // message id
            Serial.print(bracelet_id); Serial.print(','); // device id
            Serial.print(*(++pData)); Serial.println(','); // value
            break;
        case 0x90: { // info about buttons and gestures
                Serial.print(msg_id); Serial.print(','); // message id
                Serial.print(bracelet_id); Serial.print(','); // bracelet id
                uint8_t gesture_buttons = *(++pData);
                Serial.print(gesture_buttons & 0x01); Serial.print(',');
                Serial.print((gesture_buttons & 0x02) >> 1); Serial.print(',');
                Serial.print((gesture_buttons & 0x04) >> 2); Serial.println(',');
            }
            break;
        case 0x30: { // for only two accelerations
                imu_data_5B_s tmp_struct;
                memcpy(&tmp_struct, pData, sizeof(tmp_struct));
                Serial.print(msg_id); Serial.print(',');
                Serial.print(bracelet_id); Serial.print(',');
                Serial.print(tmp_struct.a_x); Serial.print(',');
                Serial.print(tmp_struct.a_y); Serial.println(',');
            }
            break;
        case 0x34: { // for all mpu data
                imu_data_13B_s tmp_struct;
                memcpy(&tmp_struct, pData, sizeof(tmp_struct));
                Serial.print(msg_id); Serial.print(',');
                Serial.print(bracelet_id); Serial.print(',');
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
    switch (ble_connected_devices_count) {
        case 1:
            ptr_client_bracelet = pClient[0];
            break;
        case 2:
            ptr_client_bracelet = pClient[1];
            break;
    }

    ptr_client_bracelet->setClientCallbacks(new ClientCallback());

    switch (ble_connected_devices_count) {
        case 1:
            ptr_client_bracelet->connect(bracelet_server[0]);
            break;
        case 2:
            ptr_client_bracelet->connect(bracelet_server[1]);
            break;
    }

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

void WiFi_enable(void) {
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) while(1);

    memcpy(peer_info.peer_addr, broadcast_platfrom_address, sizeof(broadcast_platfrom_address));
    if (esp_now_add_peer(&peer_info) != ESP_OK) while(1);

    if (WiFi.setTxPower(WIFI_POWER_2dBm) != 1) while(1);

    esp_now_register_recv_cb(esp_now_input_handler);
    WiFi.disconnect();
}

void connect_to_platform(void) {
    platfrom_is_connected = false;
    char input_buf[6];
    Serial.readBytes(input_buf, 6);
    memcpy(broadcast_platfrom_address, input_buf, sizeof(input_buf));
    
    if (esp_now_del_peer(peer_info.peer_addr) != ESP_OK) while(1);

    memcpy(peer_info.peer_addr, broadcast_platfrom_address, sizeof(broadcast_platfrom_address));
    if (esp_now_add_peer(&peer_info) != ESP_OK) while(1);
    delay(100);

    uint8_t esp_now_querry;
    esp_now_querry = 0x48;
    esp_now_send(broadcast_platfrom_address, &esp_now_querry, 1);

    delay(3000);
    if (platfrom_is_connected) {
        Serial.println("HOK");
    } else {
        Serial.println("HNO");
    }
}

bool BLE_address_config(void) {
    uint8_t name_end_pose = 0;
    char input_buf[MAX_BLE_SERVER_NAME_LEN];
    size_t check_len = Serial.readBytesUntil('\n', input_buf, MAX_BLE_SERVER_NAME_LEN);
    
    if (check_len > 0) {
        while (input_buf[name_end_pose] != '$' && name_end_pose < MAX_BLE_SERVER_NAME_LEN) name_end_pose++;

        if (name_end_pose < (MAX_BLE_SERVER_NAME_LEN - 1)) { // if '$' is found and there is the symbol 'L' or 'R' after it
            if (++ble_connected_devices_count >= 2) {
                ble_connected_devices_count = 2;
                two_devices_were_connected = true;
            }

            switch (input_buf[name_end_pose + 1]) {
                case 'L':
                    bracelets_hands[ble_connected_devices_count - 1] = '0';
                    break;
                case 'R':
                    bracelets_hands[ble_connected_devices_count - 1] = '1';
                    break;
            }

            memcpy(ble_server_name[ble_connected_devices_count - 1], input_buf, name_end_pose);

            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

void BLE_enable(void) {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    pBLEScan->start(5, false);
}

// callback function that will be executed when data is received
void esp_now_input_handler(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (!memcmp(mac_addr, broadcast_platfrom_address, 6)) {
        switch (*incomingData) {
            case 0x68: // 'h'
                platfrom_is_connected = true;
                break;
            case 0x46:
                memcpy(rfid_received_data, incomingData, 6);
                break;
            case 0x49: // US and IR data
                memcpy(us_ir_received_data, incomingData, 11);
                break;
        }
    }
}
