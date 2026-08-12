#include <Wire.h>

constexpr int I2C_SDA = 32;
constexpr int I2C_SCL = 33;

void printLineState(const char *label) {
  Serial.printf("%s: SDA=%d SCL=%d\n",
                label, digitalRead(I2C_SDA), digitalRead(I2C_SCL));
}

void scanBus(uint32_t frequency) {
  Serial.printf("\nScanning I2C on SDA=%d SCL=%d at %lu Hz\n",
                I2C_SDA, I2C_SCL, static_cast<unsigned long>(frequency));

  Wire.end();
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(20);
  printLineState("Before Wire.begin");

  Wire.begin(I2C_SDA, I2C_SCL, frequency);
  Wire.setTimeOut(50);
  delay(100);
  printLineState("After Wire.begin");

  int found = 0;
  int nack = 0;
  int timeout = 0;
  int otherErrors = 0;

  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("FOUND: 0x%02X", address);
      if (address == 0x68 || address == 0x69) {
        Serial.print("  <- possible ICM20948");
      } else if (address == 0x40 || address == 0x41) {
        Serial.print("  <- possible INA219");
      } else if (address == 0x3C || address == 0x3D) {
        Serial.print("  <- possible OLED");
      }
      Serial.println();
      ++found;
    } else if (error == 2 || error == 3) {
      ++nack;
    } else if (error == 5) {
      ++timeout;
    } else {
      ++otherErrors;
    }
  }

  if (found == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.printf("Total devices: %d\n", found);
  }
  Serial.printf("Errors: NACK=%d TIMEOUT=%d OTHER=%d\n",
                nack, timeout, otherErrors);
  printLineState("After scan");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nUGV02 I2C diagnostic scanner");
  Serial.println("This sketch does not drive the motors.");

  scanBus(100000);
  scanBus(400000);
}

void loop() {
  delay(5000);
  scanBus(100000);
}
