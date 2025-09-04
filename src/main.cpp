#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina(0x40);  // default INA219 address

// ====== PINS ======
const int RELAY_PIN = 16;           // change if needed
const bool RELAY_ACTIVE_LOW = true; // set false if your relay is active-HIGH
const int STATUS_LED = 2;           // onboard LED (optional)

// ====== LOGGING ======
const uint16_t SAMPLE_HZ = 20;      // lines per second (20 is plenty for EI time series)
const bool PRINT_HEADER = true;     // print CSV header once

// ====== LABEL (you change this from Serial) ======
String label = "unset";  // e.g., normal_off, normal_on, theft_off, theft_on

// ====== HELP ======
void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  on      : relay ON");
  Serial.println("  off     : relay OFF");
  Serial.println("  l <txt> : set label (e.g., l normal_on)");
  Serial.println("  ?       : show help");
  Serial.println();
  Serial.println("CSV columns: ts_ms,relay_on,V,I,P,label");
  Serial.println("Tip: File > Save output in Serial Monitor to .csv for Edge Impulse upload.");
  Serial.println();
}

void relayWrite(bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  else                  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

bool relayIsOn() {
  int v = digitalRead(RELAY_PIN);
  if (RELAY_ACTIVE_LOW) return (v == LOW);
  else                  return (v == HIGH);
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("on")) {
    relayWrite(true);
    Serial.println("# relay: ON");
  } else if (line.equalsIgnoreCase("off")) {
    relayWrite(false);
    Serial.println("# relay: OFF");
  } else if (line.startsWith("l ")) {
    label = line.substring(2);
    label.trim();
    Serial.print("# label: ");
    Serial.println(label);
  } else if (line == "?" || line.equalsIgnoreCase("help")) {
    printHelp();
  } else {
    Serial.println("# Unknown. Type '?' for help.");
  }
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  relayWrite(false);                 // start safe: OFF
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  Serial.begin(115200);
  delay(200);

  Wire.begin(21, 22);                // SDA, SCL (ESP32 default pins)
  if (!ina.begin()) {
    Serial.println("# ERROR: INA219 not found. Check wiring/address.");
    while (1) { delay(500); }
  }

  Serial.println("# ESP32 + INA219 CSV Logger (Edge Impulse)");
  printHelp();

  if (PRINT_HEADER) {
    Serial.println("ts_ms,relay_on,V,I,P,label");
  }

}

void loop() {
  static unsigned long last = 0;
  unsigned long now = millis();

  // sample at SAMPLE_HZ
  if (now - last >= (1000 / SAMPLE_HZ)) {
    last = now;

    // read INA219
    float V = ina.getBusVoltage_V();          // volts
    float I = ina.getCurrent_mA() / 1000.0f;  // amps
    float P = V * I;                           // watts

    // status LED on when relay ON
    digitalWrite(STATUS_LED, relayIsOn() ? HIGH : LOW);

    // print CSV line
    Serial.print(now);                 Serial.print(',');
    Serial.print(relayIsOn() ? 1 : 0); Serial.print(',');
    Serial.print(V, 3);                Serial.print(',');
    Serial.print(I, 3);                Serial.print(',');
    Serial.print(P, 3);                Serial.print(',');
    Serial.println(label);
  }

  // handle user commands
  handleSerial();
}
