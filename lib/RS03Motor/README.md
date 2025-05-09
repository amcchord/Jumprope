# RS03Motor Library

A comprehensive Arduino library for controlling RS03 motors over CAN bus. This library provides a complete interface for all control modes of the RS03 motor, including position and velocity control.

## Features

- Full support for all RS03 motor control modes:
  - Position control (CSP mode)
  - Position control with trajectory (PP mode)
  - Velocity control
  - Current (torque) control
  - Operation control mode
- Detailed error handling and fault reporting
- Multiple motor control support
- Test functions for motor evaluation
- Easy-to-use API with Arduino compatibility

## Hardware Requirements

- A microcontroller compatible with Arduino (Tested with Raspberry Pi Pico)
- MCP2515 CAN controller module
- RS03 motor(s)

## Dependencies

- [MCP2515 Library](https://github.com/autowp/arduino-mcp2515) for CAN communication

## Installation

1. Download the library as a ZIP file
2. In the Arduino IDE, go to Sketch > Include Library > Add .ZIP Library... and select the downloaded file
3. Alternatively, extract the ZIP file into your Arduino libraries folder

## Wiring

Connect your MCP2515 module to your Arduino board:

| MCP2515 Pin | Arduino Pin |
|-------------|-------------|
| VCC         | 5V or 3.3V  |
| GND         | GND         |
| CS          | GPIO5 (default, configurable) |
| MOSI        | SPI MOSI    |
| MISO        | SPI MISO    |
| SCK         | SPI SCK     |
| INT         | GPIO4 (default, configurable) |

Connect the CAN bus wires to your RS03 motor:

| CAN Module | RS03 Motor |
|------------|------------|
| CAN_H      | CAN_H      |
| CAN_L      | CAN_L      |

## Usage

### Basic Usage

```cpp
#include <SPI.h>
#include <mcp2515.h>
#include <RS03Motor.h>

// CAN module setup
#define CAN_CS_PIN 5
#define CAN_INT_PIN 4
#define CAN_SPEED CAN_1000KBPS
#define CAN_CRYSTAL MCP_8MHZ

// Create MCP2515 instance
MCP2515 mcp2515(CAN_CS_PIN);

// Create RS03Motor instance (motor ID 1, master ID 0)
RS03Motor motor(mcp2515, 1, 0);

void setup() {
  Serial.begin(115200);
  SPI.begin();
  
  // Initialize CAN controller
  mcp2515.reset();
  mcp2515.setBitrate(CAN_SPEED, CAN_CRYSTAL);
  mcp2515.setNormalMode();
  
  // Setup interrupt pin
  pinMode(CAN_INT_PIN, INPUT_PULLUP);
  
  // Initialize the motor
  if (motor.begin()) {
    Serial.println("Motor initialized successfully");
    
    // Enable the motor
    motor.enable();
    
    // Set position mode
    motor.setMode(MODE_POSITION_CSP);
    
    // Move to position 1.0 radians
    motor.setPosition(1.0);
  } else {
    Serial.println("Motor initialization failed");
  }
}

void loop() {
  // Process incoming CAN messages
  if (digitalRead(CAN_INT_PIN) == LOW) {
    struct can_frame frame;
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      motor.processCANMessage(frame);
    }
  }
  
  // Print motor status every second
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 1000) {
    RS03MotorFeedback feedback = motor.getFeedback();
    
    Serial.print("Position: ");
    Serial.print(feedback.position);
    Serial.print(" rad, Velocity: ");
    Serial.print(feedback.velocity);
    Serial.print(" rad/s, Torque: ");
    Serial.print(feedback.torque);
    Serial.print(" Nm, Temperature: ");
    Serial.print(feedback.temperature);
    Serial.println(" °C");
    
    if (motor.hasFault()) {
      RS03MotorFault fault = motor.getFault();
      Serial.print("Fault: ");
      Serial.println(fault.fault_str);
    }
    
    lastStatus = millis();
  }
}
```

### Position Control

```cpp
// Set CSP mode (Continuous position control)
motor.setMode(MODE_POSITION_CSP);

// Move to position 2.0 radians
motor.setPosition(2.0);
```

### Position Control with Trajectory (PP Mode)

```cpp
// Set PP mode (Point-to-point position control)
motor.setMode(MODE_POSITION_PP);

// Move to position 2.0 radians with max velocity 5.0 rad/s and acceleration 10.0 rad/s²
motor.setPositionWithParams(2.0, 5.0, 10.0);
```

### Velocity Control

```cpp
// Set velocity mode
motor.setMode(MODE_VELOCITY);

// Rotate at 3.0 rad/s
motor.setVelocity(3.0);

// Set acceleration to 5.0 rad/s²
motor.setAcceleration(5.0);
```

### Current (Torque) Control

```cpp
// Set current mode
motor.setMode(MODE_CURRENT);

// Set current to 2.0 Amps
motor.setCurrent(2.0);
```

### Operation Control Mode

```cpp
// Set operation mode
motor.setMode(MODE_OPERATION);

// Set position, velocity, torque, kp, and kd
motor.setOperationControl(1.0, 2.0, 5.0, 30.0, 0.5);
```

### Motor Settings

```cpp
// Set torque limit
motor.setTorqueLimit(10.0);

// Set mechanical zero position
motor.setZeroPosition();

// Set new CAN ID
motor.setCanId(2);

// Save parameters to non-volatile memory
motor.saveParameters();
```

## Fault Handling

```cpp
// Check if motor has fault
if (motor.hasFault()) {
  // Get detailed fault information
  RS03MotorFault fault = motor.getFault();
  
  // Print fault description
  Serial.print("Fault: ");
  Serial.println(fault.fault_str);
  
  // Clear the fault
  motor.clearFault();
}
```

## Examples

The library comes with several examples:

- **RS03Motor_Test**: Basic test of motor functionality
- **RS03MultiMotor**: Control multiple motors with synchronized patterns

## Reference

### Constructor

- `RS03Motor(MCP2515 &mcp2515_instance, uint8_t motor_id = 1, uint8_t master_id = 0)`

### Methods

- `bool begin()`: Initialize the motor
- `bool enable()`: Enable the motor
- `bool disable(bool clear_fault = false)`: Disable the motor
- `bool setMode(uint8_t mode)`: Set motor operation mode
- `bool setPosition(float position)`: Set position (CSP mode)
- `bool setPositionWithParams(float position, float velocity, float acceleration)`: Set position with parameters (PP mode)
- `bool setVelocity(float velocity)`: Set velocity
- `bool setAcceleration(float acceleration)`: Set acceleration for velocity mode
- `bool setCurrent(float current)`: Set current (torque)
- `bool setOperationControl(float position, float velocity, float torque, float kp, float kd)`: Set operation control parameters
- `bool setTorqueLimit(float torque_limit)`: Set torque limit
- `bool setZeroPosition()`: Set mechanical zero position
- `bool setCanId(uint8_t new_id)`: Set new CAN ID for motor
- `bool saveParameters()`: Save parameters to non-volatile memory
- `bool setActiveReporting(bool enable, uint16_t interval_ms = 10)`: Set active reporting mode
- `bool setZeroFlag(uint8_t zero_flag)`: Set zero flag status
- `bool readParameter(uint16_t param_index, float *value)`: Read a parameter from the motor
- `bool writeParameter(uint16_t param_index, float value)`: Write a parameter to the motor
- `bool processCANMessage(struct can_frame &frame)`: Process received CAN message
- `RS03MotorFeedback getFeedback()`: Get last received feedback from motor
- `RS03MotorFault getFault()`: Get detailed fault information
- `bool clearFault()`: Clear motor fault
- `bool hasFault()`: Check if motor is in fault state
- `String getFaultDescription(uint32_t fault_code)`: Get human-readable description of fault
- `bool testPositionControl(float position)`: Test motor position control
- `bool testVelocityControl(float velocity, uint32_t duration_ms)`: Test motor velocity control
- `bool testSinusoidalMovement(float amplitude, float frequency, uint32_t duration_ms)`: Test motor sinusoidal movement

## License

This library is released under the MIT License.

## Acknowledgments

This library is based on the RS03 motor instruction manual and protocol documentation. 