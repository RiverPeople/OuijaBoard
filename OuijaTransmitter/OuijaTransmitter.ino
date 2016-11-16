/*
        pin D8-----------CE    (chip enable in)
  SS pin D10----------CSN   (chip select in)
  SCK pin D13----------SCK   (SPI clock in)
  MOSI pin D11----------SDI   (SPI Data in)
  MISO pin D12----------SDO   (SPI data out)

*/

#include <SPI.h>
#include <RH_NRF24.h>

#define PIN_BATT A0
#define PIN_MOTOR 9 //A1

#define BATT_THRES 680
#define TAG_SIZE 14
#define TAG_N 13

char tags[13][32] = {"1500363D1709", "1500368DA608", "1500370E6A46", "1500368B77DF", "15003738F3E9", "150037294B40", "150036423352", "150036BFCF53", "1500364E3A57", "150037523D4D", "150036CD11FF", "1500374F711C", "150036347661"};
int tag_index = 0;

unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 5000;
char buffer[32]; // buffer array for data receive over serial port
int count = 0;   // counter for buffer array

char last_tag[32]; //storing the last tag we read

// Singleton instance of the radio driver
RH_NRF24 nrf24;
// RH_NRF24 nrf24(8, 7); // use this to be electrically compatible with Mirf
// RH_NRF24 nrf24(8, 10);// For Leonardo, need explicit SS pin
// RH_NRF24 nrf24(8, 7); // For RFM73 on Anarduino Mini

void clearBufferArray()              // function to clear buffer array
{
  for (int i = 0; i < count; i++)
  {
    buffer[i] = NULL;
  }                  // clear all index of array with command NULL
}

int rf_send(uint8_t* data) {
  //data = uint8_t (data);
  nrf24.send(data, sizeof(data));

  nrf24.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (data[0] == 't') {
    if (nrf24.waitAvailableTimeout(500))
    {
      // Should be a reply message for us now
      if (nrf24.recv(buf, &len))
      {
        if (strcmp((char*)buf, "ack") == 0) {
          Serial.println("acknowledged");
          return 0;
        } else {
          Serial.print("got reply: ");
          Serial.println((char*)buf);
          return 1;
        }
      }
      else
      {
        Serial.println("recv failed");
        return -1;
      }
    }
    else
    {
      Serial.println("nack");
      return -2;
    }
  }
}


void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");

  pinMode(PIN_MOTOR, OUTPUT);
  digitalWrite(PIN_MOTOR, HIGH);
  delay(400);
  digitalWrite(PIN_MOTOR, LOW);

  uint8_t data[1] = {'H'};
  rf_send(data);
}


void loop()
{

  if (Serial.available())              // if date is coming from software serial port ==> data is coming from SoftSerial shield
  {
    while (Serial.available())         // reading data into char array
    {
      buffer[count++] = Serial.read();   // writing data into array
      delay(3);
      if (count == TAG_SIZE) {
        Serial.print("rxed tag");
        break;
      }
    }

    Serial.print("comparing:");           // if no data transmission ends, write buffer to hardware serial port
    Serial.print(buffer);           // if no data transmission ends, write buffer to hardware serial port
    Serial.print(";");           // if no data transmission ends, write buffer to hardware serial port
    Serial.println(tags[tag_index]);           // if no data transmission ends, write buffer to hardware serial port


    if (strcmp(buffer, last_tag)) {
      digitalWrite(PIN_MOTOR, HIGH);
      delay(200);
      digitalWrite(PIN_MOTOR, LOW);

      if (strcmp(buffer, tags[tag_index]) == 0) {
        Serial.print(tag_index);
        Serial.println(". tag correct");
        tag_index++;
      } else if (strcmp(buffer, tags[0]) == 0) {
        Serial.println("tag 0 found, starting over");
        tag_index = 1;
      } else if (strcmp(buffer, tags[tag_index - 1]) != 0) {
        Serial.print(tag_index);
        Serial.println(". tag wrong, start over");
        tag_index = 0;
        //digitalWrite(PIN_MOTOR, HIGH);
        //delay(200);
        //digitalWrite(PIN_MOTOR, LOW);
      }
    }
    strncpy(last_tag, buffer, 64);

    if (tag_index == TAG_N) { //wysylamy sygnal, gdy trafimy wszystkie tagi
      Serial.println("ALL TAGS CORRECT");
      uint8_t data2[1] = {'t'}; //trigger
      while (rf_send(data2)) { //wysylaj trigger, poki nie dostaniesz ack od odbiornika
        Serial.println("no ack, resending trigger");
        delay(200);
      }
      tag_index = 0;
    }
    clearBufferArray();              // call clearBufferArray function to clear the stored data from the array
    count = 0;                       // set counter of while loop to zero

  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // przesylaj heartbeat co okreslony czas bez delay
    previousMillis = currentMillis;

    uint8_t data[1] = {'h'}; //heartbeat
    rf_send(data);
    if (analogRead(PIN_BATT) < BATT_THRES) {
      Serial.println(analogRead(PIN_BATT));
      uint8_t data3[1] = {'b'}; //malo energii
      rf_send(data3);
    }
  }
}

