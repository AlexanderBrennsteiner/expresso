//Include Libraries
#include <OneWire.h> 
#include <DallasTemperature.h>

//Variables
int Poti_raw;
int Poti_processed;
int Current_raw;
int Current_mV = 0;
int Current_processed;
int Current_Offset = 0;
String inString;         // incoming serial byte
String test = String("start");
//Temperatursensor DS18B20 Variablen
uint32_t requestMillis = 0;
bool requestStarted = false;
const uint8_t bitResolution = 12;
const uint16_t ConvWait = 750;
int buttonState;

//Inputs
int pot_pin = A0;
int current_pin = A1;
int pushButton = 53;

//Outputs
int MOSFET_pin = 40;
int LED_pin = 41;





//Defines
#define CURRENT_SCALE_LEM 12.5
#define CURRENT_SCALE_ACS 66
#define CURRENT_SENSOR_WINDINGS_LEM 2
// Data wire is plugged into pin 2 on the Arduino 
#define ONE_WIRE_BUS 52 
//#define LEM_CURRENTSENSOR


// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
/********************************************************************/ 



void setup() {
  pinMode(LED_pin, OUTPUT);
  pinMode(MOSFET_pin, OUTPUT);
  pinMode(pushButton, INPUT_PULLUP); 
  Serial.begin(9600);

  // Start up the library 
  sensors.begin(); 
  sensors.setResolution(bitResolution); // die Aufloesung festlegen
  sensors.setWaitForConversion(false);  // den Sensor asynchron (nicht blockierend) abfrage

  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }  
}


void loop() {
  //--------------------------------------------------------------------
  //Read Sensor Inputs
  //Reading from potentiometer
  Poti_raw = analogRead(pot_pin);
  //reading from Current Sensor
  Current_raw = analogRead(current_pin);
  //Read the Pushbutton
  buttonState = digitalRead(pushButton);

  //Read the Temperatur
  if (!requestStarted) {                      // wenn noch kein Request ausgeloest wurde, dann...
    sensors.requestTemperatures();            // Request ausloesen
    requestStarted = true;                    // Variable zum merken, dass der Request ausgeloest wurde
    requestMillis = millis();                 // die Millisekunden merken
    //digitalWrite(LED_pin, HIGH);
  } else {
    //digitalWrite(LED_pin, LOW);
  }

  //--------------------------------------------------------------------


  //--------------------------------------------------------------------
  //Read from the serial COM Port
  if (Serial.available() > 0) {
    inString = Serial.readString();
  }
  //--------------------------------------------------------------------



  //--------------------------------------------------------------------#TODO make own Function
  //Zero Current Offset of the first Byte is "s"
  if (inString[0] == test[0]) {
    inString = ""; //Delete input after action
    Current_Offset = Current_mV;
    Serial.print("Offset Raw in mV = ");
    Serial.println(Current_Offset);
    
  }
  //--------------------------------------------------------------------


  
  //--------------------------------------------------------------------
  //Calculate Values
  //Current Sensor
  Current_mV = map(Current_raw, 0, 1023, 0, 5000);
  
  #ifdef LEM_CURRENTSENSOR
    //LEM Current Sensor 
    Current_processed = ((Current_mV-Current_Offset)/CURRENT_SCALE_LEM)/CURRENT_SENSOR_WINDINGS_LEM;
  #else
    Current_processed = (Current_mV-Current_Offset)/CURRENT_SCALE_ACS;
    

  #endif

  
  
  //Mapping the Values Poti between 0 to 255 because we can give output
  //from 0 - 255 using the analogwrite funtion
  Poti_processed = map(Poti_raw, 0, 1021, 0, 255);


  //--------------------------------------------------------------------
  //Write Ouputs  
  //Write Signal for MOSFET 
  digitalWrite(LED_pin, !buttonState);
  digitalWrite(MOSFET_pin, !buttonState);
 
  //--------------------------------------------------------------------


  
  //--------------------------------------------------------------------
  //Write Data Back to PC when a new Temperatur value is ready
    if (millis() - requestMillis >= ConvWait) { // wenn die Konvertierungszeit erreicht ist, dann...
      Serial.print("Current: ");
      Serial.println(Current_processed);
      //Serial.println(Current_raw);
      float temp = sensors.getTempCByIndex(0);  // die Temperatur auslesen
      Serial.print("Temperature: ");            // und anzeigen
      Serial.println(temp);
      requestStarted = false;                   // die Variable zuruecksetzen, damit der naechste Request ausgeloest werden kann
  }
  //--------------------------------------------------------------------


  delay(1);

}
