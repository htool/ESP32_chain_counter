bool wifi_enabled = true;
bool n2k_enabled  = false;

#include <Arduino.h>
#include "WiFi.h"
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <EEPROM.h>             // include library to read and write from flash memory
#include <Wire.h>

// EEPROM to save chain length
const uint8_t EEPROM_SIZE = 6;
enum EEP_ADDR
{
    EEP_CHAIN_PULSE_COUNT = 0x00,
    EEP_CHAIN_DPP = 0x02    
};

// N2K
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32
#include "N2kMessages.h"
#include <NMEA2000_esp32.h>
#define DEV_CHAIN 0 // 60-140
tNMEA2000* nmea2000;
tN2kMsg N2kMsg;
tN2kMsg N2kMsgReply;
int DEVICE_ID = 51;
int SID;

// LED
#define LED_BUILTIN 2

// WIFI
const char* ssid = "Lepelaar";
const char* password =  "bladiebla";
const char* websockets_server_host = "192.168.3.1"; // Enter server address
const char* websockets_uri = "/signalk/v1/stream?subscribe=none";   // Websocket uri
const uint16_t websockets_server_port = 3000; // Enter server port

unsigned long t_next, u_next, e_next;             // Timers
String deltaMsg;
char delta[255];

// ----- Arduino pin definitions
byte intPin = 2;                                                    // Interrupt pin (not used) ... 2 and 3 are the Arduinos ext int pins

bool showOutput = true;

char InputChar;                   // incoming characters stored here

// ----- software timer
unsigned long Timer1 = 500000L;   // 500mS loop ... used when sending data to to Processing
unsigned long Stop1;              // Timer1 stops when micros() exceeds this value

// Chain variables
bool ChainUp, ChainDown, ChainSensor, ChainMoved, ChainSensorLastState;
float ChainLength;
float distancePerPulse = 0.1675;
int ChainDirection = 1;    // 1 = down, -1 = up
int pulseCount;
bool sendUpdate = true;
#define DOWN_PIN 36
#define UP_PIN 39
#define SENSOR_PIN 35

using namespace websockets;
WebsocketsClient wsClient;

String subscribeMsg = "{\"context\": \"*\",\"subscribe\":[{\"path\":\"winches.windlass.distanceperpulse\"},{\"path\":\"winches.windlass.pulseCount\"}]}";

void onMessageCallback(WebsocketsMessage message) {
  if (String(message.data()).indexOf("winches.windlass.distanceperpulse") > 0) {
    int colon = message.data().lastIndexOf(":");
    // Serial.println(message.data());
    float value = message.data().substring(colon+1,colon+6).toFloat();
    Serial.printf("Got distancePerPulse value: %1.5f \n", value);
    distancePerPulse = value;
    ChainLength = distancePerPulse * pulseCount;
  }
  if (String(message.data()).indexOf("winches.windlass.pulseCount") > 0) {
    int colon = message.data().lastIndexOf(":");
    // Serial.println(message.data());
    float value = message.data().substring(colon+1,colon+6).toFloat();
    Serial.printf("Got pulseCount value: %1.5f \n", value);
    pulseCount = value;
    ChainLength = distancePerPulse * pulseCount;
  }
}

void onEventsCallback(WebsocketsEvent event, String data) {
    if(event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("Websocket connnection opened");
        wsClient.send(subscribeMsg);
    } else if(event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("Websocket connnection closed");
        sleep(5);
        connectToSk();
    } else if(event == WebsocketsEvent::GotPing) {
        // Serial.println("Got a Ping!");
    } else if(event == WebsocketsEvent::GotPong) {
        // Serial.println("Got a Pong!");
    }
}

void connectToSk() {
  Serial.println("Connecting to SignalK websocket");
  while (!wsClient.connect(websockets_server_host, websockets_server_port, websockets_uri)) {
    Serial.print('.');
    sleep(1);
  }
  Serial.println("\nWebsocket connected!");
}
    
void setup()
{
  Serial.begin(115200);
  while (!Serial);

  // Setup LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup websocket Callbacks
  wsClient.onMessage(onMessageCallback);
  wsClient.onEvent(onEventsCallback);
    
  // Setup chain sensors
  pinMode(UP_PIN, INPUT);
  // digitalWrite(UP_PIN, HIGH);       // turn on pullup resistors
  pinMode(DOWN_PIN, INPUT);
  // digitalWrite(DOWN_PIN, HIGH);       // turn on pullup resistors
  pinMode(SENSOR_PIN, INPUT);
  // digitalWrite(SENSOR_PIN, HIGH);       // turn on pullup resistors

  Serial.println("\n\nQuick chain counter starting...");
  Serial.printf("Wifi: %d\n", int(wifi_enabled));
  Serial.printf("N2k : %d\n", int(n2k_enabled));
  Serial.println("");
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        Serial.println("EEPROM start failed");
    } else {
        Serial.println("EEPROM started");
    }
  
  Serial.print("Pulse count from EEPROM: ");
  pulseCount = EEPROM.readInt(EEP_CHAIN_PULSE_COUNT);
  if (pulseCount < 10) {
    // Auto reset to 0 when we can assume anchor is fully up
    pulseCount = 0;        
  }
  Serial.printf("%d\n", pulseCount);

  Serial.print("Chain distance per pulse from EEPROM: ");
  distancePerPulse = float(EEPROM.readFloat(EEP_CHAIN_DPP)) / 1000.0;
  Serial.printf("%1.4fm\n", distancePerPulse);
  ChainLength = pulseCount * distancePerPulse;
  Serial.printf("Chain length: %4.1fm\n", ChainLength);

  if (wifi_enabled) {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.print(" connected with ");
    Serial.println(WiFi.localIP());

    connectToSk();
  }

  sleep(1);
  
  // Initialise canbus
  Serial.println("Set up NMEA2000 device");
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);
  
  // nmea2000->SetDeviceCount(1);
  // nmea2000->SetInstallationDescription1("");
  // nmea2000->SetInstallationDescription2("");
  nmea2000->SetProductInformation("107018103", // Manufacturer's Model serial code
                                 13233, // Manufacturer's product code
                                 "Chain counter",  // Manufacturer's Model ID
                                 "0.0.1",  // Manufacturer's Software version code
                                 "", // Manufacturer's Model version
                                 1,  // load equivalency *50ma
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_CHAIN
                                );
 
  // Set device information
  nmea2000->SetDeviceInformation(1048278, // Unique number. Use e.g. Serial number.
                                140, // Device function=Temperature See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                60, // Device class=Sensor Communication Interface. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                275, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4,
                                DEV_CHAIN
                               );
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, DEVICE_ID);
  nmea2000->EnableForward(false);
  nmea2000->SetForwardStream(&Serial);
  nmea2000->SetForwardType(tNMEA2000::fwdt_Text);
  // nmea2000->ExtendTransmitMessages(TransmitMessagesCompass, DEV_CHAIN);
  nmea2000->SetN2kCANMsgBufSize(20);
  nmea2000->SetMsgHandler(HandleNMEA2000Msg);
  nmea2000->Open();
  
  Serial.println("Press 'o' to enable serial output.");
  t_next = 0;
  u_next = 0;
  e_next = 0;
}

void loop()
{
  if (t_next + 10 <= millis())                  // Set timer for 10ms
  {
    t_next = millis();
    readSensors();
    if (ChainUp) {
      ChainDirection = -1;
    } else if (ChainDown) {
      ChainDirection = 1;
    }

    if (ChainMoved) {
      pulseCount += ChainDirection;
      ChainLength = pulseCount * distancePerPulse;
      // Can't go negative
      if (pulseCount < 0) { pulseCount = 0; }
      ChainMoved = false;
      sendUpdate = true;
    }
    
    if (e_next + 5000 <= millis()) {    // Update EEPROM if needed every 5 seconds
      e_next = millis();
      // Store pusle count
      writeIntToEEPROM(EEP_CHAIN_PULSE_COUNT, pulseCount);
      writeFloatToEEPROM(EEP_CHAIN_DPP, distancePerPulse * 1000);
      wsClient.ping();                    // And ping SignalK
    }
    
    if (u_next + 2000 <= millis()) {
      // Send at least a packet every 2 seconds even if there's no update.
      u_next = millis();
      sendUpdate = true;
    }
         
    if (sendUpdate) {
      // Send 130824 performance packet
      if (n2k_enabled && SetN2kPGN130824(N2kMsg)) {
        nmea2000->SendMsg(N2kMsg, DEV_CHAIN);
        SID++; if (SID > 253) {
          SID = 1;
        }
      }
      if (wifi_enabled) {
        generateDeltaMsg();
        wsClient.send(deltaMsg);
      }
      
      sendUpdate = false;
      printValues();
    }
        
    // Check for commands
    if (Serial.available()) {
      InputChar = Serial.read();
      if (InputChar == 'o') {
        showOutput = !showOutput;
      }
    }
    ToggleLed();                              // Toggle led
  }
  nmea2000->ParseMessages();
  wsClient.poll();
}

int num_n2k_messages = 0;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  // N2kMsg.Print(&Serial);
  // Serial.printf("PGN: %u\n", (unsigned int)N2kMsg.PGN);

  if (!n2k_enabled) {
    n2k_enabled = true;
    Serial.println("Enabling sending over NMEA2000");
  }
  /* switch (N2kMsg.PGN) {
    case 130845L:
      uint16_t Key, Command, Value;
      if (ParseN2kPGN130845(N2kMsg, Key, Command, Value)) {
        // Serial.printf("Key: %d Command: %d Value: %d\n", Key, Command, Value);
      }
      break;
  }
  */
  num_n2k_messages++;
  // Serial.printf("Message count: %d\n", num_n2k_messages);
  // ToggleLed();
}

bool SetN2kPGN130824(tN2kMsg &N2kMsg) {
  N2kMsg.SetPGN(130824L);
  N2kMsg.Priority=3;
  N2kMsg.AddByte(0x7d); // Reserved
  N2kMsg.AddByte(0x99); // Reserved
  N2kMsg.AddByte(0x1c); // 1c,21 Chain length
  N2kMsg.AddByte(0x21); //
  N2kMsg.Add2ByteDouble(ChainLength, 0.01);    
  return true;
}

void readSensors () {
  ChainUp = !digitalRead(UP_PIN);
  ChainDown = !digitalRead(DOWN_PIN);
  ChainSensor = digitalRead(SENSOR_PIN);

  if (ChainSensor != ChainSensorLastState) {
    ChainMoved = true;
    ChainSensorLastState = ChainSensor;
  }
}

void generateDeltaMsg () {
  sprintf(delta, "{\"updates\": [{\"source\": {\"label\": \"Chain counter\"},\"values\": [{\"path\": \"winches.windlass.rode\",\"value\": %f}, {\"path\": \"winches.windlass.pulseCount\",\"value\": %d}]}]}", ChainLength, pulseCount);
  deltaMsg = String(delta);
}

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

void printValues() {
  // ----- send the results to the Serial Monitor
  if (showOutput) {  
    Serial.printf("Up: %1d  Down: %1d  Direction: %d  Sensor: %1d  pulseCount: %d  dpp: %5.3f  Length: %6.1fm  n2k: %1d\n", 
     ChainUp, ChainDown, ChainDirection, ChainSensor, pulseCount, distancePerPulse, ChainLength, n2k_enabled);
  } 
}

void writeFloatToEEPROM (int8_t address, float value) {
  if (float(EEPROM.readFloat(address)) != value) {
    Serial.printf("Writing new EEPROM value: %f != %f\n", float(EEPROM.readFloat(address)), value);
    EEPROM.writeFloat(address, value);
    EEPROM.commit();
  }
}

void writeIntToEEPROM(int8_t address, int value) {
  if (int(EEPROM.readInt(address)) != value) {
    Serial.printf("Writing new EEPROM value: %d != %d\n", int(EEPROM.readInt(address)), value);
    EEPROM.writeInt(address, value);
    EEPROM.commit();
  }  
}


void clearCalibration()
{
    for (size_t i = 0; i < EEPROM_SIZE; ++i) {
      EEPROM.writeByte(i, 0x0);
    }
    EEPROM.commit();
}
