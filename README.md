# Jumprope
This is the code that will enable a new combat robot.

## Hardware Platform

This project targets the [Adafruit RP2040 CAN Bus Feather](https://learn.adafruit.com/adafruit-rp2040-can-bus-feather), which features:

- RP2040 32-bit Cortex M0+ dual core microcontroller running at ~133 MHz
- 8 MB of QSPI FLASH for code storage
- 264 KB of RAM
- Built-in MCP25625 CAN controller (MCP2515 with integrated transceiver)
- USB-C connector for programming and power
- LiPo battery connector and charging circuit
- STEMMA QT connector for I2C devices

### CAN Bus Interface

The onboard CAN controller (MCP25625) connects to the RP2040 via SPI with the following connections:
- SPI interface for data communication
- Chip Select (CS) pin
- Interrupt (IRQ) pin

The CAN controller supports:
- Standard and extended CAN message formats
- Data rates up to 1 Mbps
- Built-in 5V charge pump for the CAN transceiver
- Onboard 120-ohm termination resistor (removable via jumper)
- 3.5mm terminal block for CAN High, CAN Low, and GND connections

## Pinouts

### CAN Controller Pins
The MCP25625 CAN controller uses the main SPI port and the following GPIO pins:

- **GPIO14** - SCK (SPI Clock)
- **GPIO15** - MOSI (SPI Data Out)
- **GPIO8** - MISO (SPI Data In)
- **GPIO16** - CAN Standby pin (available as `CAN_STANDBY` in CircuitPython)
- **GPIO17** - CAN Transmit pin (available as `CAN_TX0_RTS` in CircuitPython)
- **GPIO18** - CAN Reset pin (available as `CAN_RESET` in CircuitPython)
- **GPIO19** - CAN Chip Select pin (available as `CAN_CS` in CircuitPython)
- **GPIO22** - CAN Interrupt pin (available as `CAN_INTERRUPT` in CircuitPython)
- **GPIO23** - CAN Receive pin (available as `CAN_RX0_BF` in CircuitPython)

### CAN Bus Terminal Block
The 3.5mm terminal block provides:
- **L** - CAN Low signal
- **GND** - Common ground
- **H** - CAN High signal

### Board Features
- **Terminator Jumper**: The board includes a 120 ohm termination resistor between CAN H and CAN L that can be disabled by cutting the "Term" jumper.
- **STEMMA QT**: For easy I2C device connections (uses RP2040's I2C1 peripheral)
- **RGB NeoPixel** on GPIO21 with power control on GPIO20
- **Red LED** on GPIO13
- **Boot button** on GPIO7
- **Reset button**
- **4 Analog pins**: A0-A3 (GPIO26-29)
