#include <esp_now.h>
#include <WiFi.h>

// Replace with the MAC address of your receiver ESP32
uint8_t receiverMacAddress[] = {0xF8, 0xB3, 0xB7, 0x47, 0xF8, 0x7C};

// Joystick pins
#define VRX_PIN  A0  // Arduino pin connected to VRx pin of joystick
#define VRY_PIN  A1  // Arduino pin connected to VRy pin of joystick
#define SW_PIN   2   // Arduino pin connected to SW pin of joystick

// Variables to store joystick readings
int xValue = 0;
int yValue = 0;
int bValue = 0;

struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte switchPressed;
};
PacketData data;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Success");
  } else {
    Serial.println("Delivery Fail");
  }
}

void setup() {
  Serial.begin(115200);

  // Set joystick switch pin as input with pull-up
  pinMode(SW_PIN, INPUT_PULLUP);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Print MAC address
  Serial.print("Transmitter MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  } else {
    Serial.println("Success: Added peer");
  }
}

void loop() {
  // Read joystick values
  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);
  bValue = digitalRead(SW_PIN);

  // Map analog values to byte range (0-254)
  data.xAxisValue = map(xValue, 0, 4095, 0, 254); // ESP32 ADC is 12-bit (0-4095)
  data.yAxisValue = map(yValue, 0, 4095, 0, 254);
  data.switchPressed = (bValue == LOW) ? 1 : 0;   // Button is pressed when LOW

  // Debug output
  Serial.print("X: ");
  Serial.print(data.xAxisValue);
  Serial.print(" Y: ");
  Serial.print(data.yAxisValue);
  Serial.print(" Button: ");
  Serial.println(data.switchPressed);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &data, sizeof(data));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  delay(100); // Send data every 100ms
}
