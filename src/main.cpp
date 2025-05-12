#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MCP2515.h>
#include <RS03Motor.h>
#include <Adafruit_NeoPixel.h>

// CAN module pins
#define CAN_CS_PIN 19
#define CAN_INT_PIN 22

// CAN speed
#define CAN_BAUDRATE 250000

// Motor configuration
#define MOTOR_ID 1
#define MASTER_ID 0

// NeoPixel configuration
#define NEOPIXEL_PIN 16
#define NUM_PIXELS 1

// Create instances
Adafruit_MCP2515 mcp(CAN_CS_PIN);
RS03Motor motor(mcp, MOTOR_ID, MASTER_ID);
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// LED states
enum LEDState {
  LED_OFF,           // No communication
  LED_INIT,          // Initializing
  LED_BLUE,          // Blue
  LED_COMM_OK,       // Communication OK
  LED_COMM_ERROR,    // Communication error
  LED_MOTOR_ACTIVE,  // Motor is active
  LED_RX_OVERRUN,    // RX buffer overrun
  LED_TX_ERROR,      // TX error
  LED_RX_ERROR,      // RX error
  LED_BUS_ERROR      // Bus error
};

LEDState currentLEDState = LED_OFF;
unsigned long lastLEDUpdate = 0;
unsigned long lastCommTime = 0;
unsigned long lastErrorCheck = 0;
unsigned long connectionAttempts = 0;
const unsigned long MAX_CONNECTION_ATTEMPTS = 50000;
const unsigned long ERROR_CHECK_INTERVAL = 100; // Check for errors every 100ms

// Test state definitions
enum TestState {
  STATE_INIT,
  STATE_POSITION_TEST,
  STATE_VELOCITY_TEST,
  STATE_CURRENT_TEST,
  STATE_SINUSOIDAL_TEST,
  STATE_FINISHED
};

TestState currentState = STATE_INIT;
unsigned long stateStartTime = 0;
unsigned long statusPrintTime = 0;

// Function to initialize CAN communication
bool initCAN() {
    Serial.println("Starting CAN initialization...");
    
    // Initialize SPI
    Serial.println("Initializing SPI...");
    SPI.begin();
    delay(100);  // Give the CAN module time to stabilize

    // Initialize CAN
    Serial.println("Initializing MCP2515...");
    if (!mcp.begin(CAN_BAUDRATE)) {
        Serial.println("Error initializing MCP2515");
        return false;
    }
    Serial.println("MCP2515 initialized successfully");
    delay(100);

    return true;
}

// Function to set LED color based on CAN state
void setCANStateLED(uint8_t state) {
    switch (state) {
        case 0:  // Initializing
            pixels.setPixelColor(0, pixels.Color(255, 165, 0));  // Orange
            break;
        case 1:  // Communication OK
            pixels.setPixelColor(0, pixels.Color(0, 255, 0));    // Green
            break;
        case 2:  // Error
            pixels.setPixelColor(0, pixels.Color(255, 0, 0));    // Red
            break;
        default:
            pixels.setPixelColor(0, pixels.Color(0, 0, 0));      // Off
    }
    pixels.show();
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("Starting...");

    // Initialize NeoPixel
    pixels.begin();
    pixels.setBrightness(50);
    setCANStateLED(0);  // Initializing state

    // Initialize CAN
    if (!initCAN()) {
        Serial.println("Failed to initialize CAN");
        setCANStateLED(2);  // Error state
        while (1) delay(1000);
    }

    // Initialize motor
    if (!motor.begin()) {
        Serial.println("Failed to initialize motor");
        setCANStateLED(2);  // Error state
        while (1) delay(1000);
    }

    setCANStateLED(1);  // Communication OK state
    Serial.println("Initialization complete");
}

void loop() {
    // Your existing loop code
    delay(100);
}