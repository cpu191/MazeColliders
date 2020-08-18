//This will be used on the Arduino UNO + LCD Shield

#include <LiquidCrystal.h>

double  incomingByte;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


void PrintMessage(String message)
{
  Serial.print(message);
  Serial.write(13); //carriage return character (ASCII 13, or '\r')
  Serial.write(10); //newline character (ASCII 10, or '\n')
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);
}
void loop() {

  PrintMessage("CMD_ACT_LAT_1_2 ");  // Move Foward 2m
  delay(100);
  PrintMessage("CMD_ACT_ROT_1_30 "); // Rotate Clockwise 30Deg
  delay(100);
  PrintMessage("CMD_ACT_LAT_0_1 ");  // Move Backward 1m
  delay(100);
  PrintMessage("CMD_SEN_ROT_180");    // Rotate the sensor to the 45Deg position
  delay(100);
  PrintMessage("CMD_SEN_IR");        // Query the sensor reading
  String incomingByte = Serial.readString(); // Read the incoming value
  lcd.print(incomingByte);           // Print the incoming byte to the lcd shield
  PrintMessage("CMD_SEN_ROT_0");    // Rotate the sensor to the 0Deg position
  for (int i = 0; i <=18; i++) {
    PrintMessage("CMD_ACT_LAT_1_1");  // Move Foward 0.5m
    PrintMessage("CMD_ACT_ROT_0_20"); // Rotate Counter Clockwise 30Deg
  }
  PrintMessage("CLOSE");             // Stop the program and clear the variable

}
