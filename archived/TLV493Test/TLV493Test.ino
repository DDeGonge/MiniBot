#include <Tlv493d.h>
#include <Wire.h>

#define SDA_PIN 25
#define SCL_PIN 26

void print_mag(Tlv493d);

void print_mag(Tlv493d MagSensor)
{
  MagSensor.updateData();
  Serial.printf("%.2f,%.2f,%.2f\n", MagSensor.getX(), MagSensor.getY(), MagSensor.getZ());
  // Serial.printf("T: %d\tAZ: %.2f\tMAG: %.2f\n", micros(), MagSensor.getAzimuth(), MagSensor.getAmount());
}

void setup() {
  Serial.begin(921600);
  while(!Serial);
}
 
void loop() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(10000000);
  Tlv493d MagSensor = Tlv493d();
  MagSensor.begin(Wire);
  MagSensor.setAccessMode(MagSensor.FASTMODE); // LOWPOWERMODE FASTMODE
  // MagSensor.disableTemp();
  Serial.println("x,y,z");
//  MagSensor.enableInterrupt();
  // Serial.println("SENSOR STARTED");

  while(true) {
    print_mag(MagSensor);
    delay(100);
  }
}
