//This will be used on the Arduino UNO + LCD Shield


#include <LiquidCrystal.h>

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5
double  incomingByte;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


// read the buttons
int read_LCD_buttons()
{
  int adc_key_in = analogRead(0);      // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  delay(150);// debounce
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 195)  return btnUP;
  if (adc_key_in < 380)  return btnDOWN;
  if (adc_key_in < 555)  return btnLEFT;
  if (adc_key_in < 790)  return btnSELECT;

  return btnNONE;  // when all others fail, return this...
}

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
  lcd.autoscroll();
  delay(2000);
  PrintMessage("CMD_START");
}
long unsigned int x = 0;
void loop() {

  // send data only when you receive data:
  //  if (Serial.available() > 0) {
  //    // read the incoming byte:

  

  int lcd_key = read_LCD_buttons();  // read the buttons

  
  switch (lcd_key)               // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:
      {
        PrintMessage("CMD_ACT_ROT_1_10");
        break;
      }
    case btnLEFT:
      {
        PrintMessage("CMD_ACT_ROT_0_10");
        break;
      }
    case btnUP:
      {
        PrintMessage("CMD_ACT_LAT_1_1");
        break;
      }
    case btnDOWN:
      {
        PrintMessage("CMD_SEN_PING");
        
        String incomingByte = Serial.readString();
//        Serial.println(incomingByte);
        lcd.println(incomingByte);
        
        break;
      }
    case btnSELECT:
      {
        PrintMessage("CMD_CLOSE");// SLCT
        break;
      }
    case btnNONE:
      {
        //PrintMessage("CMD_SEN_PING");
       // String incomingByte = Serial.readString();
        //Serial.println(incomingByte);
        break;
      }

  }

}
