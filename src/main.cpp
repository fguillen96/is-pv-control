// *******************************************************************************************************
// DESCRIPTION: MQTT monitoring and control system 
// AUTOR: Fran Guillén
// BOARD: M-Duino 50RRA (IndustrialShields)
// *****************************************************************************************************

#define DEBUGGING
//#define PRINT_DATA


#include <debug.h>
#include <Arduino.h>
#include <Ethernet.h>           // Ethernet communication
#include <SPI.h>                // SPI protocol (Ethernet shield)
#include <PubSubClient.h>       // MQTT library
#include <ArduinoJson.h>        // JSON library
#include <TimerOne.h>           // TimerOne library (interrupt timer)
#include <ModbusRTUMaster.h>    //
#include <RS485.h>
#include <secrets.h>



// ---------- DEFAULT SYSTEM CONFIGURATION ----------
#define SAMPLE_TIME 100    // Sample time (ms)

// ---------- ANALOG INPUTS ----------
#define AI_IRRADIANCE     I0_5
#define AI_TEMPERATURE    I0_4
#define AI_VOLTAGE        I1_5
#define AI_CURRENT        I1_4
#define AI_FREQUENCY      I1_3

// ---------- DIGITAL OUTPUTS ----------
#define DO_RUN            R0_1      // Pin to run signal to VFD


// ---------- MODBUS CONFIG ---------
#define INV_ID              1
#define INV_ADDR_POWER      42  
#define INV_ADDR_CONTROL    1000

#define INV_START_CMD       6
#define INV_STOP_CMD        5


// ---------- ETHERNET AND MQTT CONNECTION ----------
#define ETHERNET_MAC      0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED      // MAC
#define ETHERNET_IP       192, 168, 1, 225                        // Shield IP
#define CLIENT_ID         "mduino_LAB"                            // MQTT client ID
#define MQTT_SERVER       SECRET_MQTT_SERVER                      // MQTT server IP
#define MQTT_PORT         SECRET_MQTT_PORT                   // MQTT port
#define MQTT_USERNAME     SECRET_MQTT_USERNAME                    // MQTT server username
#define MQTT_PASSWORD     SECRET_MQTT_PASSWORD                    // MQTT server password


// ---------- MQTT TOPICS ----------
#define TOPIC_PUB_PARAM               "lab/param"
#define TOPIC_PUB_STATUS              "lab/status"
#define TOPIC_SUB_CONFIG              "lab/config"
#define TOPIC_SUB_RUN                 "lab/control"
#define TOPIC_SUB_INVERTER_CONTROL    "lab/inverter/control"




// ********************************************************************
//                     GLOBAL VARIABLES
// ********************************************************************
// ---------- ETHERNET AND MQTT CLIENT ----------
byte mac[] = {ETHERNET_MAC};
IPAddress ip(ETHERNET_IP);
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

// --------- MODBUS ---------
ModbusRTUMaster master(RS485);
const uint32_t baudrate = 9600UL;

// ---------- DATA PARAMETERS -----------
volatile int irradiance_adc = 0;
volatile int cell_temp_adc = 0;
volatile int voltage_adc = 0;
volatile int current_adc = 0;
volatile int frequency_adc = 0;
int inv_power = 0;

// --------- INTERRUPT FLAG --------
volatile bool interrupt_flag = false;


// ---------- TIME CONTROL VARIABLES ---------
volatile unsigned long millis_value = 0;
unsigned long previous_time = 0;
unsigned long timestamp = 0;


// --------- OPERATION VARIABLES ---------
bool run = false;
bool save_mode = true;
int sample_time = SAMPLE_TIME;

bool modbus_control_request_send = false;
uint16_t modbus_data[2];

// ********************************************************************
//                     FUNCTION PROTOTYPES
// ********************************************************************
bool ConnectMQTT();
void ReadData();
void UpdateStatus();
unsigned long GetEpochTime();




// ********************************************************************
//                      INTERRUPTS
// ********************************************************************
void ReadData() {
  // ------- ANALOG INPUTS ---------
  current_adc = analogRead(AI_CURRENT);
  frequency_adc = analogRead(AI_FREQUENCY);
  irradiance_adc = analogRead(AI_IRRADIANCE);
  cell_temp_adc = analogRead(AI_TEMPERATURE);
  voltage_adc = analogRead(AI_VOLTAGE);

  // ---------- MODBUS ---------
  master.readInputRegisters(INV_ID, INV_ADDR_POWER, 1);

  // ---------- INTERRUPT FLAG ----------
  interrupt_flag = true;
}




// ********************************************************************
//                      CALLBACKS
// ********************************************************************
void OnReceiveMQTT(char *topic, byte *payload, unsigned int length) {
  DEBUGLN("Received!");
  // ---------- FILTERING BY TOPIC ----------
  if (strcmp(topic, TOPIC_SUB_RUN) == 0) {
    DEBUGLN("Run signal changed");
    if (payload[0] == '1' or payload[0] == 't')
      run = true;
    else
      run = false;

    digitalWrite(DO_RUN, run); // Stop and start frequency drive
    UpdateStatus();
  }


  if (strcmp(topic, TOPIC_SUB_INVERTER_CONTROL) == 0) {
    StaticJsonDocument<100> doc;
    deserializeJson(doc, payload, length);
    JsonObject obj = doc.as<JsonObject>();

    if (obj.containsKey("max_power")) {
      uint16_t max_power = doc["max_power"];
      modbus_data[0] = 8;
      modbus_data[1] = max_power * 6.5534;
      modbus_control_request_send = true;
    }

    if (obj.containsKey("run")) {
      bool run = doc["run"];
      if (run) {
        modbus_data[0] = {INV_START_CMD};
      }
      else {
        modbus_data[0] = {INV_STOP_CMD};
      }
      modbus_control_request_send = true;
    }
    UpdateStatus();
  }

  else if (strcmp(topic, TOPIC_SUB_CONFIG) == 0) {

    // --------- MQTT VARIABLES ----------
    StaticJsonDocument<100> doc;
    deserializeJson(doc, payload, length);
    JsonObject obj = doc.as<JsonObject>();

    // ---------- CHANGING SAMPLE TIME ---------
    if (obj.containsKey("sample_time")) {
      DEBUGLN("Sample time changed");
      sample_time = doc["sample_time"];  
      Timer1.setPeriod(sample_time * 1000L);
      UpdateStatus();
    }

    // ---------- ENABLE SENDING DATA OR NOT ----------
    if (obj.containsKey("save_mode")) {
      save_mode = doc["save_mode"];
      UpdateStatus(); 
    }

    // ---------- RESET TIMESTAMP ----------
    if (obj.containsKey("timestamp")) {
      timestamp = doc["timestamp"];
    }
  }

}

// ********************************************************************
//                     BOARD SETUP
// ********************************************************************
void setup()
{
  pinMode(DO_RUN, OUTPUT);

  Serial.begin(9600);

  // ---------- TIMER ONE INTERRUPT CONFIG ---------
  Timer1.initialize(SAMPLE_TIME * 1000L);
  Timer1.attachInterrupt(ReadData);

  // --------- INTERNET AND SERVER CONFIG ---------
  Ethernet.begin(mac, ip);
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(OnReceiveMQTT);

  // -------- MODBUS CONFIGURATION -------
  RS485.begin(baudrate, HALFDUPLEX, SERIAL_8N1);
  master.begin(baudrate);

}

// ********************************************************************
//                           LOOP
// ********************************************************************
void loop() {
  // ---------- MANAGE MQTT CONNECTION (NON BLOCKING) ----------
  static unsigned long lastReconnectAttempt = 0;
  if (!mqttClient.connected()) {
    if (millis() - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = millis();
      if (ConnectMQTT()) {
        lastReconnectAttempt = 0;
        UpdateStatus();
      }
    }
  }


  // ---------- MANAGE MODBUS REQUEST AND RESPONSE ----------
  static bool data_response_flag = false;
  static bool data_write_ok = false;
  ModbusResponse response(0, nullptr);
  if (master.isWaitingResponse()) {
    response = master.available();
    if (response) {
      if (!response.hasError()) {
        if (response.getFC() == ModbusDevice::FunctionCodes::ReadInputRegisters) {
          inv_power = response.getRegister(0);
          data_response_flag = true;
        }
        else if (response.getFC() == ModbusDevice::FunctionCodes::WriteMultipleRegisters) {
          data_write_ok = true;
          DEBUGLN("Data write OK");
        }
      }  
      else DEBUGLN("Response Error");
    }
  }
  else if(modbus_control_request_send) {
    modbus_control_request_send = false;
    DEBUG("Request OK: ");
    DEBUGLN(master.writeMultipleRegisters(INV_ID, INV_ADDR_CONTROL, modbus_data, 2));
  }
  
  if (interrupt_flag && data_response_flag) {
    interrupt_flag = false;
    data_response_flag = false;

    // ---------- SEND DATA ---------
    const int capacity = JSON_OBJECT_SIZE(7);
    StaticJsonDocument<capacity> doc;
    char buffer[110];
    unsigned long aux = 0;

    // Timestamp
    if (save_mode) {
      unsigned long increment = (millis_value - previous_time);
      if (increment < SAMPLE_TIME) increment = SAMPLE_TIME;
      timestamp = timestamp + increment;
      previous_time = millis_value;
      doc["t"] = timestamp;
    }

    // Irradiance
    aux = irradiance_adc * 1164L;
    aux = aux / 511;
    doc["G"] = aux;
    DATA("G: ");
    DATALN(aux);

    // Temperature
    float cell_temp = roundf(cell_temp_adc * 29.35 - 2000.0) / 100.0;
    doc["T"] = cell_temp;
    DATA("T: ");
    DATALN(cell_temp);

    // Voltage
    aux = voltage_adc * 500L;
    aux = aux / 511;
    doc["V"] = aux;
    DATA("V: ");
    DATALN(aux);

    // Current
    float current = roundf(current_adc * 331.0 / 511.0) / 100.0;
    doc["I"] = current;
    DATA("I: ");
    DATALN(current);

    // Frequency
    float frequency = roundf(frequency_adc * 5000.0 / 511.0) / 100.0;
    doc["f"] = frequency;
    DATA("f: ");
    DATALN(frequency);
    DATALN();

    // Inversor power
    doc["Pi"] = inv_power;
    DATA("Pi: ");
    DATALN(inv_power);
    DATALN();

    //Send data to MQTT
    serializeJsonPretty(doc, buffer);
    mqttClient.publish(TOPIC_PUB_PARAM, buffer, false);
  }

  // ---------- CLIENT LOOP ----------
  mqttClient.loop();
}



// ********************************************************************
//                      LOCAL FUNCTIONS
// ********************************************************************


void UpdateStatus() {
    // Local variables to send data
    const int capacity = JSON_OBJECT_SIZE(3);
    StaticJsonDocument<capacity> doc;
    char buffer[80];

    doc["run_signal"] = run;
    doc["save_mode"] = save_mode;
    doc["sample_time"] = sample_time;

    //Send data to MQTT (RETAIN FLAG TO TRUE)
    serializeJsonPretty(doc, buffer);
    mqttClient.publish(TOPIC_PUB_STATUS, buffer, true);
}

bool ConnectMQTT() {
  String clientId = CLIENT_ID;

  // Attempt to connect
  if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
    DEBUGLN("Connected to MQTT server");
    mqttClient.subscribe(TOPIC_SUB_CONFIG, 1); // Subscribe to topic
    mqttClient.subscribe(TOPIC_SUB_RUN, 1);
    mqttClient.subscribe(TOPIC_SUB_INVERTER_CONTROL, 1);
    UpdateStatus();
    }

  return mqttClient.connected();
}