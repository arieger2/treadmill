//HUGE APP (3mb NO OTA/1MB SPIFFS)

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>

/// BLE 1 start:  variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
bool        is_inst_stride_len_present = 1;                                 /**< True if Instantaneous Stride Length is present in the measurement. */
bool        is_total_distance_present = 1;                                  /**< True if Total Distance is present in the measurement. */
bool        is_running = 1;                                                 /**< True if running, False if walking. */
uint16_t    inst_speed = 40;                                                 /**< Instantaneous Speed. */
uint8_t     inst_cadence = 1;                                               /**< Instantaneous Cadence. */
uint16_t    inst_stride_length = 1;                                         /**< Instantaneous Stride Length. */
uint32_t    total_distance = 10;
int delayint = 500;
volatile unsigned int counter;
float rpm = 0;
float mpsOffset = 0;                                                        //used for BLE
volatile long gx_lastRiseTime = 0; 
volatile long mz_lastRiseTime = 0; 

volatile long firstButtonPress = 0;
///float kmph;
float mps = 0;
///float kmphStatic;
byte fakePos[1] = {1};
bool _BLEClientConnected = false;

#define RSCService BLEUUID((uint16_t)0x1814)
BLECharacteristic RSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A53), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);

BLEDescriptor RSCDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};
/// BLE 1 end  

//> Tacho Stuff1 start  Copied from "sketch_ESP32_RPR220_TACHO_130720_v6"
#include <Wire.h>
#include <TimeLib.h>  //https://playground.arduino.cc/Code/Time/

// constants won't change. They're used here to set pin numbers:
// External interupt pin for sensor
#define interruptPin 3      // the number of the IR Receiver pin

const int ledpin2 =  2;      // the number of the LED  pin 
int ledState = LOW;             // ledState used to set the LED

long lastUpdate = 0;  // for timing display updates
volatile long accumulator4 = 0;  // sum of last 4 rpm times over 4 seconds

volatile long accumulatorInterval = 0;  // time sum between display during intervals
float rpmaccumulatorInterval = 0;

volatile unsigned long startTime = 0; // start of revolution in microseconds
volatile unsigned int revCount = 0; // number of revolutions since last display update
volatile unsigned int Totalrevcount = 0; // number of revolutions since last display update
volatile unsigned long longpauseTime = 0; // revolution time with no relection
volatile long belt_distance = 142; //mm  Belt was measured to be 2.11m between belt markings. (or Diameter of belt roller).  Back roller calculated to be 0.120471m circumference
volatile long workout_distance = 0; //Running tally of workout distance
volatile unsigned long workout_startTime = 0; // start of Workout
// Tacho Stuff1 end


///Taco Stuff 4 start
//-------------------------------------
// Interrupt Handler
// IR reflective sensor - target passed
// Calculate revolution time
//-------------------------------------
void IRAM_ATTR tach_interrupt()
{
  //Serial.println("Interupt");
  // calculate the microseconds since the last interrupt.  Interupt activates on falling edge. (IR detected)
  long usNow = micros();
  long test_elapsed = usNow - startTime;

  if(test_elapsed<300){  //assume this is button bounce if interupt activated less than 30000 microseconds since last interupt.  
    return;
  }

  if(test_elapsed>1000000){  //assume the treadmill isn't spinning in 1 second.
    startTime = usNow;       // reset the clock;
    longpauseTime = 0;  
    return;
  }
  if(test_elapsed > longpauseTime/2){  //acts as a debounce, don't looking for interupts soon after the first hit.  
      //Serial.println(test_elapsed); //Serial.println(" Counted");
      startTime = usNow;  // reset the clock
      long elapsed = test_elapsed ;
      longpauseTime = test_elapsed ;

      revCount++;
      workout_distance += belt_distance; //Running tally of workout distance
      Totalrevcount += 1;
      accumulatorInterval += elapsed;

  }
  
}
///Taco Stuff 4 end

void InitBLE() {
  BLEDevice::init("Treadmil_Rieger");
  // CBLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Do some BLE Setup
  BLEService *pRSC = pServer->createService(RSCService);

  pRSC->addCharacteristic(&RSCMeasurementCharacteristics);
  RSCDescriptor.setValue("Send all your RCSM rubbish here");
  RSCMeasurementCharacteristics.addDescriptor(&RSCDescriptor);
  RSCMeasurementCharacteristics.addDescriptor(new BLE2902());

  pRSC->addCharacteristic(&sensorPositionCharacteristic);
  pServer->getAdvertising()->addServiceUUID(RSCService);
  pRSC->start();
  pServer->getAdvertising()->start();
}

void InitTacho(){
  //pinMode(ledIRTransmit, OUTPUT);     // initialize the IR Transmitter pin as an output:
  //digitalWrite(ledIRTransmit, HIGH);  // turn on IR Transmitter 
 
  // Enable the pullup resistor and attach the interrupt. Pin is high by default unless the IR receiver detects light.
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), tach_interrupt, FALLING);

  //Timer
  setTime(0,0,0,0,0,0);                   //setTime(hr,min,sec,day,month,yr); // Another way to set time.  
}


///////////////////////////////////
///////////////////////////////////
#include "WiFi.h"
#include "ESPAsyncWebServer.h"

const char *ssid = "chicago";
const char *password = "Neukirch20211!";
//float h = 0;  //temp

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
String readSpeed() {
    return String(mps*3.6);
}
String readDistance() {
    return String(workout_distance/1000);
}
String readRpm() {
    return String(int(rpm));
}
String readHour() {
    return String(hour());
}
String readMinute() {
    return String(minute());
}
String readSecond() {
    return String(second());
}
String readZoffset() {
    return String(mpsOffset*3.6);
}
String readTotalrevcount() {
    return String(Totalrevcount);
}
String readZoffsetup() {
    mpsOffset += 0.139;
    Serial.print("UP * "); Serial.print(mpsOffset*3.6); Serial.println("km/h");
    return String(mpsOffset);
}
String readZoffsetdown() {
       if (mpsOffset < 0.139){
          mpsOffset = 0;
       }
       if (mpsOffset >= 0.139){
          mpsOffset -= 0.139;
       }

       Serial.print("Down * "); Serial.print(mpsOffset*3.6); Serial.println("km/h");
   
    return String(mpsOffset);
}
String readReset() {
    //reset all variables
    workout_distance = 0;
    setTime(0,0,0,0,0,0);                   //setTime(hr,min,sec,day,month,yr); // Another way to set time.  
    Totalrevcount = 0;
    return String(mpsOffset);
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
  <style>
    html {
     font-family: Arial;
     display: inline-block;
     margin: 0px auto;
     text-align: center;
    }
    h2 { font-size: 3.0rem; }
    p { font-size: 3.0rem; }
    .units { font-size: 1.2rem; }
    .dht-labels{
      font-size: 1.5rem;
      vertical-align:middle;
      padding-bottom: 15px;
    }
    
.button {
  border: none;
  color: white;
  padding: 15px 32px;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 16px;
  margin: 4px 2px;
  cursor: pointer;
}
.button1 {background-color: #4CAF50;} /* Green */
.button2 {background-color: #008CBA;} /* Blue */
.button3 {background-color: #cc0000;} /* Red */
  </style>
</head>
<body>
   <p>
    <i class="fas fa-running" style="color:#059e8a;"></i> 
    <!-- <span class="dht-labels">Speed</span> -->
    <span id="speed">%SPEED%</span>
    <sup class="units">kph</sup>
  </p>
    <p>
    <i class="fas fa-shoe-prints" style="color:#2d0000;"></i> 
    &nbsp;&nbsp;
    <!-- <span class="dht-labels">Distance</span> -->
    <span id="distance">%DISTANCE%</span>
    <sup class="units">M</sup>
  </p>
  <p>
   <i class="fas fa-stopwatch" style="color:#059e8a;"></i> 
    <!-- <span class="dht-labels">Time</span> --> 
    <span id="hour">%HOUR%</span>
    :
    <span id="minute">%MINUTE%</span>
    :
    <span id="second">%SECOND%</span>
  </p>
   <p>
    <i class="fas fa-arrows-alt-h" style="color:#059e8a;"></i> 
    <!-- <span class="dht-labels">Zoffset</span> --> 
    <span id="zoffset">%ZOFFSET%</span>
    <sup class="units">kph</sup>
  <br>
    <a href=/zoffsetup\><button class="button button1">&nbsp;Up&nbsp;</button></a>    
    <a href=/><button class="button button2">Down</button></a>
  </p>
  <p>
    <i class="fas fa-tachometer-alt" style="color:#00add6;"></i> 
    <!-- <span class="dht-labels">Rpm</span> --> 
    <span id="rpm">%RPM%</span>
    <sup class="units">rpm</sup>
  </p>
    <p>
    <i class="fas fa-tachometer-alt" style="color:#00add6;"></i> 
    <!-- <span class="dht-labels">Totalrevcount</span> --> 
    <span id="totalrevcount">%TOTALREVCOUNT%</span>
    <sup class="units">total</sup>
  </p>
  <p>
  <a href=/reset\><button class="button button3">&nbsp;Reset&nbsp;</button></a> 
  </p>
  <h4>Treadmil Rieger</h4>
   
</body>
<script>
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("speed").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/speed", true);
  xhttp.send();
}, 1000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("distance").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/distance", true);
  xhttp.send();
}, 1000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("hour").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/hour", true);
  xhttp.send();
}, 1000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("minute").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/minute", true);
  xhttp.send();
}, 1000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("second").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/second", true);
  xhttp.send();
}, 1000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("zoffset").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/zoffset", true);
  xhttp.send();
}, 1000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("rpm").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/rpm", true);
  xhttp.send();
}, 1000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("totalrevcount").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/totalrevcount", true);
  xhttp.send();
}, 1000 ) ;

</script>
</html>)rawliteral";

// Replaces placeholder with DHT values
String processor(const String& var){
  //Serial.println(var);
  if(var == "RPM"){
    return readRpm();
  }
  //else if(var == "HUMIDITY"){  return readDHTHumidity();  }
  else if(var == "SPEED"){
    return readSpeed();
  }
  else if(var == "DISTANCE"){
    return readDistance();
  }
  else if(var == "HOUR"){
    return readHour();
  }
  else if(var == "MINUTE"){
    return readMinute();
  }
  else if(var == "SECOND"){
    return readSecond();
  }
  else if(var == "ZOFFSET"){
    return readZoffset();
  }
  else if(var == "TOTALREVCOUNT"){
    return readTotalrevcount();
  }
  return String();
}
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////


void InitAsync_Webserver(){
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  pinMode(ledpin2, OUTPUT);     // initialize LED pin 2 as an output:
  ledState = HIGH;
  digitalWrite(ledpin2, ledState);  // turn on the onboard LED when wifi is connected 
    
  // Route for root / web page  This function is the main page, as well as the Zoffsetdown button call function
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    readZoffsetdown();
    request->send_P(200, "text/html", index_html, processor);
  });
    server.on("/speed", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readSpeed().c_str());
  });
    server.on("/distance", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDistance().c_str());
  });
  server.on("/rpm", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readRpm().c_str());
  });
    server.on("/hour", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readHour().c_str());
  });
    server.on("/minute", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readMinute().c_str());
  });
    server.on("/second", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readSecond().c_str());
  });
    server.on("/zoffset", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readZoffset().c_str());
    
  });
    server.on("/totalrevcount", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readTotalrevcount().c_str());
  });
    // Send a GET request to <IP>/sensor/<number>/action/<action>
    server.on("/zoffsetup", HTTP_GET, [] (AsyncWebServerRequest *request) {
    readZoffsetup();
    request->send_P(200, "text/html", index_html, processor);
    });
    // Send a GET request to <IP>/sensor/<number>/action/<action>
    server.on("/reset", HTTP_GET, [] (AsyncWebServerRequest *request) {
    readReset();
    request->send_P(200, "text/html", index_html, processor);
    }); 
  //server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
  //  request->send_P(200, "text/plain", readDHTHumidity().c_str());
  //});

  // Start server
  server.begin();
}

void Calculate_RPM(){
   // divide number of microseconds in a minute, by the average interval.
    if (revCount > 0){    //confirm there was at least one spin in the last second
      // rpmaccumulatorInterval = 60000000/(accumulatorInterval/revCount);
      rpm = 60000000/(accumulatorInterval/revCount);
      
      accumulatorInterval = 0;

      //Test = Calculate average from last 4 rpms - response too slow
      accumulator4 -= (accumulator4 >> 2);
      accumulator4 += rpm;
    }
    else{
      rpm = 0;
      rpmaccumulatorInterval=0;
      accumulator4 = 0;  //average rpm of last 4 samples
    }
    revCount = 0;   
}
void SerialMonitor_Run_Metrics() {
    Serial.print("RPM:"); Serial.print(int(rpm));
    Serial.print("  Speed(MPS):"); 
    //Treadmill belt was measured in mm.    Speed =mm/rev * rev/min * min/secs * meters/mm = meters/secs
    mps = belt_distance*(rpm)/(60*1000);
    Serial.print(mps);    //mps
    Serial.print("  Speed(KPH):");  Serial.print(mps*3.6);  
    Serial.print("  Distance:");  Serial.print(workout_distance/1000);  Serial.print("m  ");
    Serial.print("  Timer:"); Serial.print(hour()); Serial.print(":");Serial.print(minute());Serial.print(":");Serial.print(second());
    Serial.print("  mpsOffset:");     Serial.print(mpsOffset); 
    //Serial.print("  t_rpmaccum:"); Serial.print(int(rpmaccumulatorInterval));
    //Serial.print("  t_speed(KPH):"); Serial.println(3.6*belt_distance*(rpmaccumulatorInterval)/(60*1000));
    Serial.print("  t_speed(KPH):"); Serial.println(round(10*3.6*belt_distance*(accumulator4/4)/(60*1000))/10);  //rounded, not used anywhere else
 
    }

 void send_BLE(){
  inst_speed = (mps+mpsOffset)*256;            //formated for BLE
  total_distance = (workout_distance/100);  //formated for BLE
  ///Taco Stuff 3 end
  //Create the bytearray to send to Zwift via BLE
  byte charArray[10] = {
      3,
      (unsigned byte)inst_speed, (unsigned byte)(inst_speed >> 8),
      (unsigned byte)inst_cadence,
      (unsigned byte)inst_stride_length, (unsigned byte)(inst_stride_length >> 8),
      (unsigned byte)total_distance, (unsigned byte)(total_distance >> 8), (unsigned byte)(total_distance >> 16), (unsigned byte)(total_distance >> 24)};

  RSCMeasurementCharacteristics.setValue(charArray,10);
  RSCMeasurementCharacteristics.notify();
  sensorPositionCharacteristic.setValue(fakePos, 1);
}

void setup() {
  // initialize the LED pin as an output:
  //pinMode(ledPin, OUTPUT);
  // initialize the Serial Monitor @ 9600
  Serial.begin(115200);
  
  Serial.println("RESET");
  InitBLE();
  InitTacho();
  InitAsync_Webserver();
  delay(2000);
}

void loop(){
   if (millis() - lastUpdate > 1000){
    lastUpdate = millis();
    Calculate_RPM();
    SerialMonitor_Run_Metrics();
   send_BLE();


// Connect to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
 } 
}
