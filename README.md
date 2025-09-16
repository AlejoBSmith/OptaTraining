# Smart Campus UTP - Machine Vibration Analysis

To run inferences, download the EdgeImpulse deployment model from: https://studio.edgeimpulse.com/public/779726/live

Important: working with BLE and Ethernet simultaneously in the Arduino Opta has proven to be problematic. This code has been tested with:
Core Opta (arduino:mbed_opta): 4.2.1
ArduinoBLE: 1.3.7

Code has proven to fail (resulting in red 4/4 blinking LED in the Opta) if used with:
Core Opta 4.3.x and 4.4.x
ArduinoBLE 1.4.0 / 1.4.1 (not fully tested)
