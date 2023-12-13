/**	ANCHOR: DW1000 ANCHOR FOR RANGING
 *  AUTHOR:     Ayush Chinmay
 *  CREATED:    10 Nov 2023
 *	MODIFIED:   7 Nov 2023
 *
 *  BRIEF:      UWB based proximity alert system for the visually impaired.
 *
 *  BOM:
 *      -   ESP32 UWB PRO (Decawave DWM100)
 *      -   SSD1306 OLED Display (128x64)
 *      -   Piezo Buzzer (3V)
 *
 * 	NOTES:
 * 		-
 *
 *	CHANGELOG:
 **     -   28 November 2023
 *                      - Updated BuzzAlert function to beep at intervals
 *                      - Beep intervals are now determined by distance
 **		-	11 November 2023
 *						- Code Clean Up & Documentation
 *
 **		-	10 November 2023
 *						- Read Two-Way Ranging (TWR) data from the UWB module
 *						- Print TWR data to the OLED display
 *
 *	TODO:
 *		[-]		Pin UWB ranging function to a thread to allow for concurrent execution
 *		[-]		...
 */


/** ==========[ LIBRARIES ]========== **/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "DW1000Ranging.h"

/** ==========[ CONSTANTS ]========== **/
// UWB PINS
#define PIN_SS 4
#define DW_CS 21
#define PIN_RST 27
#define PIN_IRQ 34
// SPI PINS
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
// BUZZER PIN
#define BUZZ_PIN 13
// OLED DISPLAY
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
// BUZZER VARIABLES
unsigned long prevBuzz = 0;
int interval = 50;
const int alert_high = 622;
const int alert_med = 440;
const int alert_low = 110;


/** ==========[ OBJECTS ]========== **/
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/** ==========[ VARIABLES ]========== **/
/// UWB VARIABLES
uint16_t uwbID;
float distance = 0.0;
float powerDBm = 0.0;

/** ==========[ SETUP ]========== **
 *  Setup the UWB module as an anchor
 *  Setup the OLED display
 */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  initOLED();
  initUWB();
  pinMode(BUZZ_PIN, OUTPUT);
  delay(800);
}

/** ==========[ LOOP ]========== **
 *  Main Loop
 */
void loop() {
  DW1000Ranging.loop();
}

/** ==========[ FUNCTIONS ]========== **/
void initUWB()
{
  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
//   DW1000Ranging.useRangeFilter(true);
//   DW1000Ranging.setRangeFilterValue(2);

  //we start the module as a tag
  DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  display.println("TAG: INIT SUCCESS");
  display.println("\n7D:00:22:EA:82:60:3B:9C");
  display.display();
  delay(800);
}

void initOLED()
{
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.display();
  delay(500);
  display.clearDisplay();
  display.display();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void printOLED() {
  display.clearDisplay();
  display.setCursor(16, 0);
  display.setTextSize(1);
  display.print("TAG --> "); display.println(uwbID, HEX);
  display.print("\nDIST: ");
  display.setTextSize(2);

  if (distance >= 1) {
	display.print(distance); display.println(" m");
	// noTone(BUZZ_PIN);
  } else {
	display.print(int(distance*100)); display.println(" cm");
	// tone(BUZZ_PIN, alert_high, duration);
  }

  display.println();
  display.setTextSize(1);
  display.print("PWR:  "); display.print(powerDBm); display.println(" dBm");
  display.display();
}

/** ==========[ ALERT BUZZER ]========== **
 *  Function to determine alert priority and trigger alert
 *  Triggers the buzzer at intervals determined from distance
 */
void alertBuzz() {
    if (prevBuzz == 0 || millis()-prevBuzz > interval) {
        prevBuzz = millis();

        if (distance >= 5.0) {
            noTone(BUZZ_PIN);
            Serial.println("\tNO BUZZ");
            interval = 50;

        } else if (distance < 4.0 && distance > 2.0) {
            interval = 50;
            tone(BUZZ_PIN, 400, 50);
            Serial.println("\tLOW BUZZ");
            interval = 1000;

        } else if (distance < 2.0 && distance > 1.0) {
            interval = 75;
            tone(BUZZ_PIN, 400, 50);
            Serial.println("\tMED BUZZ");
            interval = 500;

        } else if ( distance < 1.0) {
            tone(BUZZ_PIN, 400, 50);
            Serial.println("\tHIGH BUZZ");
            interval = 100;
        }
    }
}

/** ==========[ NEW RANGE ]========== **
 *  Callback function for new range data: Asymmetric Two-Way Ranging
 *  Prints the range data to the serial monitor
 */
void newRange() {
  uwbID = DW1000Ranging.getDistantDevice()->getShortAddress();
  distance = DW1000Ranging.getDistantDevice()->getRange();
  powerDBm = DW1000Ranging.getDistantDevice()->getRXPower();
  printOLED();
  alertBuzz();
  Serial.print("from: "); Serial.print(uwbID, HEX);
  Serial.print("\t Range: "); Serial.print(distance); Serial.print(" m");
  Serial.print("\t RX power: "); Serial.print(powerDBm); Serial.println(" dBm");
}

/** ==========[ NEW DEVICE ]========== **
 *  Callback function for new device data
 *  Prints the blink data to the serial monitor
 */
void newDevice(DW1000Device* device) {
  Serial.print("ranging init; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

/** ==========[ INACTIVE DEVICE ]========== **
 *  Callback function for inactive device data
 *  Prints the inactive device data to the serial monitor
 */
void inactiveDevice(DW1000Device* device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

