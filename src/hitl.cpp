#include <Arduino.h>
#include <communication.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial7.begin(115200);
}

void loop() {
  if (receive_message()) {
        // Print received data to Serial7 for debugging
        Serial7.println("Message received successfully:");

        // Example: Print data fields
        if (msg.has_gains) {
            Serial7.print("Gain Altitude: ");
            Serial7.println(msg.gains.alt);

            Serial7.print("Gain Roll: ");
            Serial7.println(msg.gains.roll);
        }

        if (msg.has_sensors) {
            Serial7.print("Sensor Roll: ");
            Serial7.println(msg.sensors.roll);

            Serial7.print("Sensor Pitch: ");
            Serial7.println(msg.sensors.pitch);
        }

        if (msg.has_controls) {
            Serial7.print("Control Torque X: ");
            Serial7.println(msg.controls.torque_x);

            Serial7.print("Total Force: ");
            Serial7.println(msg.controls.total_force);
        }
    } else {
        // Optional: You can print an error or debug message here
        Serial7.println("No valid message received.");
    }
    delay(10);

    // Small delay to avoid overwhelming the Serial port
}