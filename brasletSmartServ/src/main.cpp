#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <string>
#include <iterator>
#include <numeric>
#include <iterator>
#include <cstdint>


#define LED_3 GPIO_NUM_18
#define LED_4 GPIO_NUM_32

void init_esp_now(void);
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

// Structure to receive data, must match the sender structure
typedef struct {
    int id;
    int x;
    int y;
    int t;
    int q;
    int w;
} struct_message;


// Create a struct_message called myData
struct_message myData;

int buff_char[6];


void setup() {
    setCpuFrequencyMhz(80);
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    pinMode(LED_3, OUTPUT);
    pinMode(LED_4, OUTPUT);

    init_esp_now();

    Serial.println("Receiver is starting");
    esp_now_register_recv_cb(OnDataRecv);
}


void loop() {

}
/***********************************************************/

void init_esp_now(void) {
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error in initializing ESP-NOW");
        while (1) {
            digitalWrite(LED_3, 1);
            digitalWrite(LED_4, 1);
            delay(500);
            digitalWrite(LED_3, 0);
            digitalWrite(LED_4, 0);
            delay(500);
        }
    } else {
        for (uint8_t i = 0; i < 7; i++) {
            digitalWrite(LED_3, 1);
            digitalWrite(LED_4, 0);
            delay(100);
            digitalWrite(LED_3, 0);
            digitalWrite(LED_4, 1);
            delay(100);
        }
        digitalWrite(LED_3, 0);
        digitalWrite(LED_4, 0);
    }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (len == 2) {
        Serial.print('\%');
        Serial.print(*incomingData);
        Serial.print(',');
        Serial.println(*(++incomingData));
    } else {
        memcpy(&myData, incomingData, sizeof(myData));
        buff_char[0] = myData.x;
        buff_char[1] = myData.y;
        buff_char[2] = myData.t;
        buff_char[3] = myData.q;
        buff_char[4] = myData.w;
        buff_char[5] = myData.id;

        for (uint8_t i = 0; i < 6; ++i) {
            Serial.print(buff_char[i]);
            Serial.print(',');
        }
        Serial.println();
    }
}
