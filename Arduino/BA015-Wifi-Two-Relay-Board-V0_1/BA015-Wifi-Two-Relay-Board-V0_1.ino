/*
 *  This sketch is running a web server for configuring WiFI if can't connect or for controlling of one GPIO to switch a light/LED
 *  Also it supports to change the state of the light via MQTT message and gives back the state after change.
 *  The push button has to switch to ground. It has following functions: Normal press less than 1 sec but more than 50ms-> Switch light. Restart press: 3 sec -> Restart the module. Reset press: 20 sec -> Clear the settings in EEPROM
 *  While a WiFi config is not set or can't connect:
 *    http://server_ip will give a config page with 
 *  While a WiFi config is set:
 *    http://server_ip/gpio -> Will display the GIPIO state and a switch form for it
 *    http://server_ip/gpio?state_led=0 -> Will change the GPIO14 to Low (triggering the SSR) 
 *    http://server_ip/gpio?state_led=1 -> Will change the GPIO14 to High (triggering the SSR)
 *    http://server_ip/gpio?state_sw=0 -> Will change the GPIO13 to Low (triggering the TRIAC) 
 *    http://server_ip/gpio?state_sw=1 -> Will change the GPIO13 to High ( triggering the TRIAC) 
 *    http://server_ip/gpio?state_dimmer=value -> value has to be a number between 0-90 example  http://server_ip/gpio?state_dimmer=80 (triggering the TRIAC)
 *    http://server_ip/cleareeprom -> Will reset the WiFi setting and rest to configure mode as AP
 *  server_ip is the IP address of the ESP8266 module, will be 
 *  printed to Serial when the module is connected. (most likly it will be 192.168.4.1)
 * To force AP config mode, press button 20 Secs!
 *  For several snippets used, the credit goes to:
 *  - https://github.com/esp8266
 *  - https://github.com/chriscook8/esp-arduino-apboot
 *  - https://github.com/knolleary/pubsubclient
 *  - https://github.com/vicatcu/pubsubclient <- Currently this needs to be used instead of the origin
 *  - https://gist.github.com/igrr/7f7e7973366fc01d6393
 *  - http://www.esp8266.com/viewforum.php?f=25
 *  - http://www.esp8266.com/viewtopic.php?f=29&t=2745
 *  Dimmer Code with timer from https://github.com/nassir-malik/IOT-Light-Dimmer
 *  - And the whole Arduino and ESP8266 comunity 1C-B9-C4-30-99-A8
 */

#define DEBUG
#ifdef DEBUG
  #define Debug(x)    Serial.print(x)
  #define Debugln(x)  Serial.println(x)
  #define Debugf(...) Serial.printf(__VA_ARGS__)
  #define Debugflush  Serial.flush
#else
  #define Debug(x)    {}
  #define Debugln(x)  {}
  #define Debugf(...) {}
  #define Debugflush  {}
#endif
 
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <Hash.h>
//#include <EEPROM.h>
#include <Ticker.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "HLW8012.h"

extern "C" {
  #include "user_interface.h" //Needed for the reset command
}
/*##################################### GPIO PIN MAP ##########################################################*/
#define OUTPIN_1                         5 //Output pin of first Relay
#define OUTPIN_2                         4 //Output pin of first Relay
#define INPIN                            0 //Input pin 
#define INPIN_1                         12 //Input pin Switch 1 
#define INPIN_2                         16 //Input pin Switch 2
   
/*HLW8012.h     GPIOS                      */
#define SEL_PIN                         15
#define CF1_PIN                         13
#define CF_PIN                          14

/*########################################## Other Defines #####################################################*/

#define RESTARTDELAY                     3 //minimal time in sec for button press to reset
#define HUMANPRESSDELAY                 50 // the delay in ms untill the press should be handled as a normal push by human. Button debounce. !!! Needs to be less than RESTARTDELAY & RESETDELAY!!!
#define RESETDELAY                      20 //Minimal time in sec for button press to reset all settings and boot to config mode

#define TIME_OUT_COUNT                 100 //number of times it tries to connect to wifi before hosting an ap 
#define PASS_VALUE                      20
#define FAIL_VALUE                      10

// Check values every 2 seconds
#define UPDATE_TIME                     2000
#define STATUS_UPDATE_TIME              200
// Set SEL_PIN to HIGH to sample current
// This is the case for Itead's Sonoff POW, where a
// the SEL_PIN drives a transistor that pulls down
// the SEL pin in the HLW8012 when closed
#define CURRENT_MODE                    HIGH

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.005
#define VOLTAGE_RESISTOR_UPSTREAM       ( 5 * 470000 ) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

/*########################################## Object instances #####################################################*/

MDNSResponder mdns;
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WiFiClient wifiClient;
PubSubClient mqttClient;
Ticker btn_timer;
Ticker otaTickLoop;
HLW8012 hlw8012;




//##### Flags ##### They are needed because the loop needs to continue and cant wait for long tasks!
int rstNeed=0;   // Restart needed to apply new settings
int toPub_1=0; // determine if state should be published.
int toPub_2=0; // determine if state should be published.
int configToClear=0; // determine if config should be cleared.
int otaFlag=0;
boolean inApMode=0;
//##### Global vars ##### 
int webtypeGlob;
int otaCount=300; //imeout in sec for OTA mode
int current; //Current state of the button
int switch_status_1; //Physical state of the switch 1
int switch_status_2; //Physical state of the switch 2
int state_out_1;   ////webpage mqtt state of the switch 1
int state_out_2;   ////webpage mqtt state of the switch 2
int iotMode=0; //IOT mode: 0 = Web control, 1 = MQTT (No const since it can change during runtime)
float P,V;
String Values_HLW8021;
String Values_HLW8021_ActivePower;
String Values_HLW8021_Voltage;
String Values_HLW8021_Current;
String Values_HLW8021_ApparentPower;
String Values_HLW8021_Powerfactor;
String Values_HLW8021_Energy;
double setmulPower=0;
double setmulCurrent=0;
double setmulVoltage=0;
int calflag =0;
unsigned long count = 0; //Button press time counter
unsigned long count_regulator = 0; //Button press time counter
String st; //WiFi Stations HTML list
String hostName ="Armtronix"; //The MQTT ID -> MAC adress will be added to make it kind of unique
String hostsaved; //Added on 15/11/2019
uint8_t mac_address[8];
char buf[40]; //For MQTT data recieve
char* host; //The DNS hostname
//To be read from Config file
String esid="";
String epass = "";
String pubTopic;
String pubTopic_HLW8021;
String subTopic;
String mqttServer = "";
String mqtt_user = "";     //added on 28/07/2018
String mqtt_passwd = "";   //added on 28/07/2018
String mqtt_will_msg_on_disconnecting = "disconnected"; //added on 28/07/2018
String mqtt_will_msg_on_connecting = "connected"; //added on 28/07/2018
String mqtt_port = "";  //added on 14/02/2019
String echannel = "" ;
const char* ebssid = "" ;
String ebssid_string = "" ;
uint8_t bssid_bytes[6];
String o1_state_pub; //State of relay one for mqtt
String o2_state_pub; //State of relay two for mqtt
const char* otaServerIndex = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
String javaScript,XML;

/*###############################################################################################*/
// When using interrupts we have to call the library entry point
// whenever an interrupt is triggered
void ICACHE_RAM_ATTR hlw8012_cf1_interrupt() 
{
    hlw8012.cf1_interrupt();
}
void ICACHE_RAM_ATTR hlw8012_cf_interrupt() {
    hlw8012.cf_interrupt();
}
/*###############################################################################################*/
// Library expects an interrupt on both edges
void setInterrupts() {
    attachInterrupt(CF1_PIN, hlw8012_cf1_interrupt, CHANGE);
    attachInterrupt(CF_PIN, hlw8012_cf_interrupt, CHANGE);
}

/*###############################################################################################*/
void calibrate_sensor(float power_value,float voltage_value) 
{
    // Let's first read power, current and voltage
    // with an interval in between to allow the signal to stabilise:
    digitalWrite(OUTPIN_1,HIGH);
    digitalWrite(OUTPIN_2,HIGH);
    
    Serial.println(power_value);
    Serial.println(voltage_value);
    
     delay(100);
     unsigned long timeout = millis();
    while ((millis() - timeout) < 10000) {
        delay(1);
    }
    // Calibrate using a 60W bulb (pure resistive) on a 230V line
    setmulPower=hlw8012.expectedActivePower(power_value);
    setmulVoltage=hlw8012.expectedVoltage(voltage_value);
    setmulCurrent=hlw8012.expectedCurrent(power_value / voltage_value);
    
    hlw8012.setCurrentMultiplier(setmulCurrent);
    hlw8012.setVoltageMultiplier(setmulVoltage);
    hlw8012.setPowerMultiplier(setmulPower);
    
    // Show corrected factors
    Serial.print("[HLW] New current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
    Serial.print("[HLW] New voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
    Serial.print("[HLW] New power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
    Serial.println();
    digitalWrite(OUTPIN_1,LOW);
    digitalWrite(OUTPIN_2,LOW);
    

}
/*###############################################################################################*/

void setup() 
{
  Serial.begin(115200);
  
  pinMode(OUTPIN_1, OUTPUT);
  pinMode(OUTPIN_2, OUTPUT);
  pinMode(INPIN, INPUT_PULLUP);
  pinMode(INPIN_1, INPUT_PULLUP);
  pinMode(INPIN_2, INPUT_PULLUP);
  btn_timer.attach(0.05, btn_handle);
  
  Debugln("DEBUG: Entering loadConfig()");
  if (!SPIFFS.begin()) 
  {
    Serial.println("Failed to mount file system");
  }
  
  WiFi.macAddress(mac_address);
  hostsaved = hostName+"_"+macToStr_2(mac_address); //Added on 15/11/2019
  host = (char*) hostsaved.c_str();
  loadConfig();
  //loadConfigOld();
  Debugln("DEBUG: loadConfig() passed");
  
  // Connect to WiFi network
  Debugln("DEBUG: Entering initWiFi()");
  initWiFi();
  Debugln("DEBUG: initWiFi() passed");
  Debug("iotMode:");
  Debugln(iotMode);
  Debug("webtypeGlob:");
  Debugln(webtypeGlob);
  Debug("otaFlag:");
  Debugln(otaFlag);
  Debugln("DEBUG: Starting the main loop");

  hlw8012.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, true);
  
  hlw8012.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);
  hlw8012.setCurrentMultiplier(setmulCurrent);
  hlw8012.setVoltageMultiplier(setmulVoltage);
  hlw8012.setPowerMultiplier(setmulPower);
    
  Serial.print("[HLW] Default current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
  Serial.print("[HLW] Default voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
  Serial.print("[HLW] Default power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
  Serial.println();
  setInterrupts();
    
}

/*###############################################################################################*/
void switch_status_handle()
{
  if(!digitalRead(INPIN_1))
 {
 switch_status_1=1;
 }
 else
 {
 switch_status_1=0; 
 }

if(!digitalRead(INPIN_2))
 {
 switch_status_2=1;
 }
 else
 {
 switch_status_2=0;
 }
}

/*###############################################################################################*/
void output_status_handle()
{
int A=state_out_1;
int B=switch_status_1;
int C=state_out_2;
int D=switch_status_2;

if((((!A)&&(B))||((A)&&(!B))))
 {
  digitalWrite(OUTPIN_1,HIGH);
  if(o1_state_pub !="R01ISON")
  {
   o1_state_pub="R01ISON";
   toPub_1 = 1;
  }
 }
else
 {
  digitalWrite(OUTPIN_1,LOW);
  if(o1_state_pub !="R01ISOFF")
  {
   o1_state_pub="R01ISOFF";
   toPub_1 = 1;
  }
 }

if(((!C)&&(D))||((C)&&(!D)))
 {
  digitalWrite(OUTPIN_2,HIGH);
  if(o2_state_pub !="R02ISON")
  {
   o2_state_pub="R02ISON";
   toPub_2 = 1;
  }
 }
else
 {
  digitalWrite(OUTPIN_2,LOW);
 if(o2_state_pub !="R02ISOFF")
  {
   o2_state_pub="R02ISOFF";
   toPub_2 = 1;
  }
 }
}

/*###############################################################################################*/
void btn_handle()
{
  if(!digitalRead(INPIN))
  {
    ++count; // one count is 50ms   
  } 
  else 
  {
    if (count > 1 && count < HUMANPRESSDELAY/5) 
    {
      //push between 50 ms and 1 sec      
      Serial.print("button pressed "); 
      Serial.print(count*0.05); 
      Serial.println(" Sec."); 
   
    } else if (count > (RESTARTDELAY/0.05) && count <= (RESETDELAY/0.05))
    { 
      //pressed 3 secs (60*0.05s)
      Serial.print("button pressed "); 
      Serial.print(count*0.05); 
      Serial.println(" Sec. Restarting!"); 
      setOtaFlag(!otaFlag);      
      ESP.reset();
    } else if (count > (RESETDELAY/0.05)){ //pressed 20 secs
      Serial.print("button pressed "); 
      Serial.print(count*0.05); 
      Serial.println(" Sec."); 
      Serial.println(" Clear settings and resetting!");       
      configToClear=1;
      }
    count=0; //reset since we are at high
  }
}

/*###############################################################################################*/
void loop() {
    static unsigned long last = millis();
    
    // This UPDATE_TIME should be at least twice the interrupt timeout (2 second by default)
    if ((millis() - last) > UPDATE_TIME) 
    {

        last = millis();
        
        Values_HLW8021_ActivePower = String(hlw8012.getActivePower());
        Values_HLW8021_Voltage = String(hlw8012.getVoltage());
        Values_HLW8021_Current = String(hlw8012.getCurrent());
        Values_HLW8021_ApparentPower = String(hlw8012.getApparentPower());
        Values_HLW8021_Powerfactor = String((int) (100 * hlw8012.getPowerFactor()));
        Values_HLW8021_Energy = String(hlw8012.getEnergy());
        
        Serial.print("[HLW] Active Power (W)    : "); Serial.println(Values_HLW8021_ActivePower);
        Serial.print("[HLW] Voltage (V)         : "); Serial.println(Values_HLW8021_Voltage);
        Serial.print("[HLW] Current (A)         : "); Serial.println(Values_HLW8021_Current);
        Serial.print("[HLW] Apparent Power (VA) : "); Serial.println(Values_HLW8021_ApparentPower);
        Serial.print("[HLW] Power Factor (%)    : "); Serial.println(Values_HLW8021_Powerfactor);
        Serial.print("[HLW] Agg. energy (Ws)    : "); Serial.println(Values_HLW8021_Energy);
        Serial.println();
        
        Values_HLW8021= "P:"+Values_HLW8021_ActivePower+"W"+", V:"+Values_HLW8021_Voltage+"V"+", C:"+Values_HLW8021_Current+"A"+", E:"+Values_HLW8021_Energy+"Ws";
        pubTopic_HLW8021=pubTopic+"/HLW8021";
         if (mqttClient.connected()) 
         {
         mqttClient.publish((char*)pubTopic_HLW8021.c_str(), (char*)Values_HLW8021.c_str());
         mqttClient.loop();
         }
    }

   if(calflag ==1)
   { 
    calibrate_sensor(P,V);
    calflag =0; 
    saveConfig()? Serial.println("sucessfully.") : Serial.println("not succesfully!");;
    ESP.wdtFeed();
    delay(100);
    WiFi.forceSleepBegin(); wdt_reset(); 
    ESP.restart(); 
    while(1)wdt_reset();
   }
   switch_status_handle();
   output_status_handle();
   
   webSocket.loop();
  //Debugln("DEBUG: loop() begin");
  
  if(configToClear==1){
    //Debugln("DEBUG: loop() clear config flag set!");
    clearConfig()? Serial.println("Config cleared!") : Serial.println("Config could not be cleared");
   // delay(1000);
    ESP.reset();
  }
  //Debugln("DEBUG: config reset check passed");  
  if (WiFi.status() == WL_CONNECTED && otaFlag){
    if(otaCount<=1) {
      Serial.println("OTA mode time out. Reset!"); 
      setOtaFlag(0);
      ESP.reset();
     // delay(100);
    }
    server.handleClient();
   // delay(1);
  } else if (WiFi.status() == WL_CONNECTED || webtypeGlob == 1){
    //Debugln("DEBUG: loop() wifi connected & webServer ");
    if (iotMode==0 || webtypeGlob == 1){
      //Debugln("DEBUG: loop() Web mode requesthandling ");
      server.handleClient();
     // delay(1);
       if(esid != "" && WiFi.status() != WL_CONNECTED) //wifi reconnect part
      {
        Scan_Wifi_Networks();
      }
    } else if (iotMode==1 && webtypeGlob != 1 && otaFlag !=1){
          //Debugln("DEBUG: loop() MQTT mode requesthandling ");
          if (!connectMQTT()){
            //  delay(200);          
          }                    
          if (mqttClient.connected()){            
              //Debugln("mqtt handler");
              mqtt_handler();
          } else {
              Debugln("mqtt Not connected!");
          }
    }
  } else{
    Debugln("DEBUG: loop - WiFi not connected");  
    delay(1000);
    initWiFi(); //Try to connect again
  }
  ESP.wdtFeed();
  //Debugln("DEBUG: loop() end");
  delay(1);
}
