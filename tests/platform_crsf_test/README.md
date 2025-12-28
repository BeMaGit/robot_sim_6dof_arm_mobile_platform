# Platform CRSF Test (BetaFPV ELRS)

This sketch allows you to control the mobile platform using a BetaFPV ELRS Micro Receive via the CRSF protocol.

## Wiring (Serial2)
| ELRS Receiver | Arduino Mega |
|---------------|--------------|
| **TX (Ch2)**  | **RX2 (Pin 17)** |
| **RX (Ch3)**  | **TX2 (Pin 16)** |
| **5V**        | **5V**       |
| **GND**       | **GND**      |

**Note**: CRSF is a non-inverted protocol, so no hardware inverter is needed.

## Configuration (IMPORTANT)
The Arduino Mega cannot accurately handle the default CRSF baud rate of 416,666.

1.  **Open ExpressLRS Configurator (Web UI or Lua Script)** for your Receiver.
2.  Go to **Model** -> **Output 2 (TX)** / **Output 3 (RX)** settings.
3.  Set the **Serial Baud Rate** to **115200**.
4.  Set **Protocol** to **CRSF** (or Serial).

## Dependencies
- **AlfredoCRSF** library (Install via Arduino Library Manager).

## Usage
1. Open `platform_crsf_test.ino` in Arduino IDE.
2. Install the **AlfredoCRSF** library.
3. Connect the receiver to Serial2 (Pins 16/17).
4. Upload the sketch.
5. Open Serial Monitor at **115200** baud.
