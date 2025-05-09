#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// CAN module pins
#define CAN_CS_PIN 5
#define CAN_INT_PIN 4

// CAN speed (500 kbps)
#define CAN_SPEED CAN_500KBPS
#define CAN_CRYSTAL MCP_8MHZ

// Create MCP2515 instance
MCP2515 mcp2515(CAN_CS_PIN);

// CAN message structure
struct can_frame canMsg;

// Function to initialize CAN controller
bool initCAN() {
  mcp2515.reset();
  MCP2515::ERROR result = mcp2515.setBitrate(CAN_SPEED, CAN_CRYSTAL);
  
  if (result == MCP2515::ERROR_OK) {
    mcp2515.setNormalMode();
    return true;
  }
  
  return false;
}

// Function to send CAN message
void sendCANMessage(uint32_t id, uint8_t dlc, uint8_t* data) {
  canMsg.can_id = id;
  canMsg.can_dlc = dlc;
  
  for (uint8_t i = 0; i < dlc && i < 8; i++) {
    canMsg.data[i] = data[i];
  }
  
  mcp2515.sendMessage(&canMsg);
}

// Function to check for received CAN messages
bool receiveCANMessage() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    return true;
  }
  return false;
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("CAN Controller Initialization");
  
  // Initialize SPI for CAN communication
  SPI.begin();
  
  // Initialize CAN controller
  if (initCAN()) {
    Serial.println("CAN initialized successfully");
  } else {
    Serial.println("CAN initialization failed");
  }
  
  // Setup interrupt pin for received messages
  pinMode(CAN_INT_PIN, INPUT_PULLUP);
}

void loop() {
  // Check if CAN message is received
  if (digitalRead(CAN_INT_PIN) == LOW) {
    if (receiveCANMessage()) {
      // Process received message
      Serial.print("Message ID: ");
      Serial.println(canMsg.can_id, HEX);
      
      Serial.print("Data Length: ");
      Serial.println(canMsg.can_dlc);
      
      Serial.print("Data: ");
      for (int i = 0; i < canMsg.can_dlc; i++) {
        Serial.print(canMsg.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  
  // Example: Send a CAN message every 1 second
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 1000) {
    uint8_t data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    sendCANMessage(0x123, 8, data);
    lastSendTime = millis();
    Serial.println("CAN message sent");
  }
}