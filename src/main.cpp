#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <RS03Motor.h>

// CAN module pins
#define CAN_CS_PIN 5
#define CAN_INT_PIN 4

// CAN speed (1 Mbps for RS03 motors)
#define CAN_SPEED CAN_1000KBPS
#define CAN_CRYSTAL MCP_8MHZ

// Motor configuration
#define MOTOR_ID 1
#define MASTER_ID 0

// Create MCP2515 instance
MCP2515 mcp2515(CAN_CS_PIN);

// Create RS03Motor instance
RS03Motor motor(mcp2515, MOTOR_ID, MASTER_ID);

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

// Function to print motor status
void printMotorStatus() {
  RS03MotorFeedback feedback = motor.getFeedback();
  
  Serial.println("\n----- Motor Status -----");
  Serial.print("Position: ");
  Serial.print(feedback.position);
  Serial.println(" rad");
  
  Serial.print("Velocity: ");
  Serial.print(feedback.velocity);
  Serial.println(" rad/s");
  
  Serial.print("Torque: ");
  Serial.print(feedback.torque);
  Serial.println(" Nm");
  
  Serial.print("Temperature: ");
  Serial.print(feedback.temperature);
  Serial.println(" °C");
  
  Serial.print("Mode: ");
  Serial.println(feedback.mode);
  
  if (motor.hasFault()) {
    RS03MotorFault fault = motor.getFault();
    Serial.print("Fault: ");
    Serial.println(fault.fault_str);
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("RS03 Motor Test");
  
  // Initialize SPI for CAN communication
  SPI.begin();
  
  // Initialize CAN controller
  if (initCAN()) {
    Serial.println("CAN initialized successfully");
  } else {
    Serial.println("CAN initialization failed");
    while (1); // Halt on error
  }
  
  // Setup interrupt pin for received messages
  pinMode(CAN_INT_PIN, INPUT_PULLUP);
  
  // Initialize the motor
  Serial.println("Initializing motor...");
  if (motor.begin()) {
    Serial.println("Motor initialized successfully");
  } else {
    Serial.println("Motor initialization failed");
    while (1); // Halt on error
  }
  
  // Start the state machine
  stateStartTime = millis();
  statusPrintTime = millis();
}

void loop() {
  // Process incoming CAN messages
  if (digitalRead(CAN_INT_PIN) == LOW) {
    struct can_frame frame;
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      motor.processCANMessage(frame);
    }
  }
  
  // Print status every 2 seconds
  if (millis() - statusPrintTime >= 2000) {
    printMotorStatus();
    statusPrintTime = millis();
  }
  
  // State machine for testing different motor functions
  switch (currentState) {
    case STATE_INIT:
      if (millis() - stateStartTime >= 1000) {
        // Enable the motor
        Serial.println("Enabling motor...");
        if (motor.enable()) {
          Serial.println("Motor enabled successfully");
          currentState = STATE_POSITION_TEST;
          stateStartTime = millis();
        } else {
          Serial.println("Failed to enable motor");
        }
      }
      break;
      
    case STATE_POSITION_TEST:
      if (millis() - stateStartTime == 0) {
        // Start position test
        Serial.println("\n----- Position Test -----");
        Serial.println("Setting to position mode (CSP)");
        motor.setMode(MODE_POSITION_CSP);
        Serial.println("Moving to position 1.0 rad");
        motor.setPosition(1.0);
      }
      else if (millis() - stateStartTime >= 5000) {
        // Move to next test
        currentState = STATE_VELOCITY_TEST;
        stateStartTime = millis();
      }
      break;
      
    case STATE_VELOCITY_TEST:
      if (millis() - stateStartTime == 0) {
        // Start velocity test
        Serial.println("\n----- Velocity Test -----");
        Serial.println("Setting to velocity mode");
        motor.setMode(MODE_VELOCITY);
        Serial.println("Setting velocity to 2.0 rad/s");
        motor.setVelocity(2.0);
      }
      else if (millis() - stateStartTime >= 5000) {
        // Stop the motor and move to next test
        Serial.println("Stopping motor");
        motor.setVelocity(0.0);
        currentState = STATE_CURRENT_TEST;
        stateStartTime = millis();
      }
      break;
      
    case STATE_CURRENT_TEST:
      if (millis() - stateStartTime == 0) {
        // Start current test
        Serial.println("\n----- Current Test -----");
        Serial.println("Setting to current mode");
        motor.setMode(MODE_CURRENT);
        Serial.println("Setting current to 1.0 A");
        motor.setCurrent(1.0);
      }
      else if (millis() - stateStartTime >= 3000) {
        // Stop the motor and move to next test
        Serial.println("Stopping motor");
        motor.setCurrent(0.0);
        currentState = STATE_SINUSOIDAL_TEST;
        stateStartTime = millis();
      }
      break;
      
    case STATE_SINUSOIDAL_TEST:
      if (millis() - stateStartTime == 0) {
        // Start sinusoidal movement test
        Serial.println("\n----- Sinusoidal Movement Test -----");
        Serial.println("Starting sinusoidal movement");
        motor.testSinusoidalMovement(0.5, 0.5, 10000);
      }
      else if (millis() - stateStartTime >= 11000) {
        // Test finished
        Serial.println("\nAll tests completed!");
        motor.disable();
        currentState = STATE_FINISHED;
      }
      break;
      
    case STATE_FINISHED:
      // Tests are done, just continue monitoring status
      break;
  }
}