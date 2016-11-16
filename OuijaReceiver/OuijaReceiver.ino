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

#define PIN_BTN 2
#define PIN_REL 9
#define PIN_LED_H 3
#define PIN_LED_B 6

long int beats = 0;

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

void heartbeat() {
  digitalWrite(PIN_LED_H, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);
  digitalWrite(PIN_LED_H, LOW);   // turn the LED on (HIGH is the voltage level)
  Serial.print("heartbeat: ");
  Serial.println(beats++);
}

void batt_warn() {
  digitalWrite(PIN_LED_B, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("battery low!");
}

void trig() {
  digitalWrite(PIN_REL, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("door opens.");
  delay(5000);
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

  // initialize digital pin 13 as an output.
  pinMode(PIN_REL, OUTPUT);
  pinMode(PIN_LED_H, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  digitalWrite(PIN_LED_H, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(PIN_LED_B, LOW);   // turn the LED on (HIGH is the voltage level)
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
      //      NRF24::printBuffer("request: ", buf, len);
      
      //Serial.print("RF RX:");
      //Serial.println((char*)buf);

      // Send a reply
      uint8_t data[] = "ack";
      if (buf[0] == 't') {
      nrf24.send(data, sizeof(data));
      nrf24.waitPacketSent();
      Serial.println("Sent a reply");
      }
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

  if (digitalRead(PIN_BTN) == LOW) {
    digitalWrite(PIN_REL, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  //delay(5000);              // wait for a second
  else digitalWrite(PIN_REL, LOW);    // turn the LED off by making the voltage LOW
  //delay(5000);              // wait for a second
}
