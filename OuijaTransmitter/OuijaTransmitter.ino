/*
   Default code for use with the Ouija Board Access Control receiver, as shown here: http://riverpeople.pl/new-project-a-smart-ouija-board/
   This code was developed using bits of example code from Arduino and developers of the NRF24 library.
   Parameters commented in ALL CAPS are there to be configured by the user according to their application.
   By The River People, 2016. This code is released on LGPL licence.
*/

/*
  pin D8-----------CE    (chip enable in)
  SS pin D10----------CSN   (chip select in)
  SCK pin D13----------SCK   (SPI clock in)
  MOSI pin D11----------SDI   (SPI Data in)
  MISO pin D12----------SDO   (SPI data out)

*/

#include <SPI.h>
#include <RH_NRF24.h>
// Singleton instance of the radio driver
RH_NRF24 nrf24;

#define REL_TIME 5000 //HOW LONG IN MS SHOULD THE RELAY STAY ENERGIZED AFTER THE CORRECT PASSWORD IS GIVEN?

#define PIN_BTN 2 //YOUR AUXILLARY MICROSWITCH PIN
#define PIN_REL 9 //YOUR RELAY PIN
#define PIN_LED_H 3 //YOUR GREEN LED PIN
#define PIN_LED_B 6 //YOUR YELLOW LED PIN

long int beats = 0;

//handle a hello message from the planchette.
void hello() {
  Serial.println("controller wakes up");
  digitalWrite(PIN_LED_B, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(PIN_LED_H, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);
  digitalWrite(PIN_LED_B, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(PIN_LED_H, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(100);
  digitalWrite(PIN_LED_B, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(PIN_LED_H, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);
  digitalWrite(PIN_LED_B, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(PIN_LED_H, LOW);   // turn the LED on (HIGH is the voltage level)
}

//handle the heartbeat.
void heartbeat() {
  digitalWrite(PIN_LED_H, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);
  digitalWrite(PIN_LED_H, LOW);   // turn the LED on (HIGH is the voltage level)
  Serial.print("heartbeat: ");
  Serial.println(beats++);
}

//handle a battery warning.
void batt_warn() {
  digitalWrite(PIN_LED_B, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("battery low!");
}

//handle an "access granted" message
void trig() {
  digitalWrite(PIN_REL, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("door opens.");
  delay(REL_TIME);
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);

  if (!nrf24.init())
    Serial.println("RF init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");

  // initialize pins.
  pinMode(PIN_REL, OUTPUT);
  pinMode(PIN_LED_H, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  digitalWrite(PIN_LED_H, LOW);
  digitalWrite(PIN_LED_B, LOW);
  Serial.println("RX init done.");
}

// the loop function runs over and over again forever
void loop() {

  if (nrf24.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (nrf24.recv(buf, &len))
    {
      // Send a reply if you get an "access granted" message
      uint8_t data[] = "ack";
      if (buf[0] == 't') {
        nrf24.send(data, sizeof(data));
        nrf24.waitPacketSent();
        Serial.println("Sent a reply");
      }
      //handle each message with an appropriate function.
      switch (buf[0]) {
        case 'h':
          heartbeat();
          break;
        case 'b':
          batt_warn();
          break;
        case 't':
          trig();
          break;
        case 'H':
          hello();
          break;
      }
    }
    else
    {
      Serial.println("recv failed");
    }
  }

  //clear the relay's state back to unenergized, also enable overriding this clearing with the manual microswitch.
  if (digitalRead(PIN_BTN) == LOW) {
    digitalWrite(PIN_REL, HIGH);
  }
  else digitalWrite(PIN_REL, LOW);
}
