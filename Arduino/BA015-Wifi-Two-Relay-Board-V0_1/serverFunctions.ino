
// -- BEGIN mainpage.html
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name = "viewport" content = "width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
<title>ARMtronix Technologies LLP</title>
<style>
body { background: #ffffff repeat; font-family: Arial, Helvetica, Sans-Serif; Color:  #ff0000; }
#container { width: 100%; max-width: 400px; margin: auto; }
.bulb { display: block; clear: none; width: 32px; height: 32px; padding-bottom: 0.5em; background-attachment: fixed; background-position: center; background-repeat: no-repeat; }
.bulbOn { background: transparent url('http://sasakaranovic.com/iotdimmer/bulbON.png') top left no-repeat; float: right; }
.bulbOff{ background: transparent url('http://sasakaranovic.com/iotdimmer/bulbOFF.png') top left no-repeat; float: left; }
.image {font-size: 0; text-align: center; width: 400px;  height: 150px; margin-left: 0; margin-right: 0;}
img {display: inline-block; vertical-align: middle; height: 150px;margin-left: 0; margin-right: 0;}
.trick {display: inline-block; vertical-align: middle; height: 150px; margin-left: 0; margin-right: 0; }

h1 {  display: block; font-size: 2em; margin-top: 0.10em; margin-bottom: 0.10em; width: 400px; margin-left: 0; margin-right: 0; font-weight: bold; text-align: center; }
input[type=text] {
  width: 40%;
  padding: 8px 20px;
  margin: 8px 0;
  font-weight: bold
  box-sizing: border-box;
}
input[type=password] {
  width: 40%;
  padding: 8px 20px;
  margin: 8px 0;
  font-weight: bold
  box-sizing: border-box;
}
input[type=submit]{
  background-color: #ff0000;
  border: none;
  border-radius: 4px;
  font-weight: bold
  cursor: pointer;
  font-weight: bold; 
  box-sizing: border-box;
}
.slidecontainer {width: 400px; }
.slider {width: 100%; margin: 0 0 2em 0; }
.buttonOff { background-color: #ff0000;margin-top: 0.1em; margin-bottom: 0.1em }
.buttonOn{ background-color: #4CAF50;margin-top: 0.1em; margin-bottom: 0.1em }
  a { 
  background-color: #212121;
    border: none;
    color: white;
    padding: 15px 20px;
  margin-right: 1em;
    text-align: center;
    text-decoration: none;
    display: inline-block;
    font-size: 16px;
  border-radius: 4px;
  }
    b { 
  background-color: #212121;
    border: none;
    color: white;
    padding: 15px 20px;
    margin-top: 0.10em; 
    margin-bottom: 0.10em; 
     margin-left: 0; 
    margin-right: 0; 
    font-weight: bold; 
    text-align: center;
    text-decoration: none;
    display: inline-block;
    font-size: 16px;
    border-radius: 4px;
  }
</style>
<script>
var websock;
var xmlHttp=createXmlHttpObject();

function start() {
websock = new WebSocket('ws://' + window.location.hostname + ':81/');
websock.onopen = function(evt) { console.log('websock open'); };
websock.onclose = function(evt) { console.log('websock close'); };
websock.onerror = function(evt) { console.log(evt); };
websock.onmessage = function(evt) 
{
console.log(evt);
var ch1 = document.getElementById('ch1');
var ch2 = document.getElementById('ch2');
var ch_values = JSON.parse(evt.data);
ch1.value = ch_values.ch1;
ch2.value = ch_values.ch2;
};
process();
}
function setCh1(val) 
{
var dat = 'ch1:'+ val
websock.send(dat);
}
function setCh2(val) 
{
var dat = 'ch2:'+ val
websock.send(dat);
}
function setPV(val1,val2) 
{
var dat = 'PV:'+ val1+','+val2
websock.send(dat);
}
function updateSlider(e) 
{
var dat = e.id +':'+ e.value
websock.send(dat);
}

function createXmlHttpObject()
{
 if(window.XMLHttpRequest)
   {
    xmlHttp=new XMLHttpRequest();
   }
 else
   {
    xmlHttp=new ActiveXObject('Microsoft.XMLHTTP');
   }
 return xmlHttp;
}

function process()
{
 if(xmlHttp.readyState==0 || xmlHttp.readyState==4)
   {
    xmlHttp.open('PUT','xml',true);
    xmlHttp.onreadystatechange=handleServerResponse;
    xmlHttp.send(null);
   }
 setTimeout('process()',500);
}
 
function handleServerResponse()
{
 if(xmlHttp.readyState==4 && xmlHttp.status==200)
   {
    xmlResponse=xmlHttp.responseXML;
    xmldoc1 = xmlResponse.getElementsByTagName('activepower');
    xmldoc2 = xmlResponse.getElementsByTagName('voltage');
    xmldoc3 = xmlResponse.getElementsByTagName('current');
    xmldoc4 = xmlResponse.getElementsByTagName('powerfactor');
    xmldoc5 = xmlResponse.getElementsByTagName('energy');
    xmldoc6 = xmlResponse.getElementsByTagName('t1');
    xmldoc7 = xmlResponse.getElementsByTagName('t2');
    message1 = xmldoc1[0].firstChild.nodeValue;
    message2 = xmldoc2[0].firstChild.nodeValue;
    message3 = xmldoc3[0].firstChild.nodeValue;
    message4 = xmldoc4[0].firstChild.nodeValue;
    message5 = xmldoc5[0].firstChild.nodeValue;
    message6 = xmldoc6[0].firstChild.nodeValue;
    message7 = xmldoc7[0].firstChild.nodeValue;
    document.getElementById('runtime1').innerHTML=message1;
    document.getElementById('runtime2').innerHTML=message2;
    document.getElementById('runtime3').innerHTML=message3;
    document.getElementById('runtime4').innerHTML=message4;
    document.getElementById('runtime5').innerHTML=message5;
    document.getElementById('runtime6').innerHTML=message6;
    document.getElementById('runtime7').innerHTML=message7;
   }
}
</script>
</head>
<body onload="javascript:start();">
<div id="container">
<div class="image">
<div class="trick"></div>
  <img src='http://armtronix.in/img/logo.png'/>
</div>
<h1>Wifi Two Relay Board</h1>
<div class="slidecontainer" style="text-align: center">
<a href="#t1"  class="buttonOn"  id='runtime6' onClick="setCh1(100)"></a> <a href="#t2" class="buttonOn"  id='runtime7' onClick="setCh2(100)"></a>
</div>
<div class="slidecontainer" style="text-align: center">
<a class="buttonOff"<label> Active Power:</label></a>
<a class="buttonOn" id='runtime1'></a>
</div>
<div class="slidecontainer" style="text-align: center">
<a class="buttonOff"<label>   Voltage:  </label></a>
<a class="buttonOn" id='runtime2'></a>
</div>
<div class="slidecontainer" style="text-align: center">
<a class="buttonOff"<label>   Current:  </label></a>
<a class="buttonOn" id='runtime3'></a>
</div>
<div class="slidecontainer" style="text-align: center">
<a class="buttonOff"<label> Power Factor:</label></a>
<a class="buttonOn" id='runtime4'></a>
</div>
<div class="slidecontainer" style="text-align: center">
<a class="buttonOff"<label>Energy in Sec:</label></a>
<a class="buttonOn" id='runtime5'></a>
</div>
<div class="slidecontainer" style="text-align: center">
<label><strong>Enter Voltage: </strong></label></a><input type="text" id='enteredvoltage' length=32>
</div>
<div class="slidecontainer" style="text-align: center">
<label><strong>Enter Power: </strong></label><input type="text" id='enteredpower' length=32>
</div>
<div class="slidecontainer" style="text-align: center">
<b href="#c2" onClick="setPV(enteredpower.value,enteredvoltage.value)"<label>Calibrate</label></b>
</div>
</div>
</body>
</html>
)rawliteral";
// ------------------------------------ END Index.html -----------------------------------------------------------------------------------------------------------------------------------
//url('https://litumiot.com/wp-content/uploads/2016/10/page-center-background-image-iot-2.jpg')

// ------------------------------------  MAIN.html -----------------------------------------------------------------------------------------------------------------------------------

static const char PROGMEM MAIN_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name = "viewport" content = "width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
<title>ARMtronix Technologies LLP</title>
<style>
body { background: #ffffff  repeat; font-family: Arial, Helvetica, Sans-Serif; Color:  #f44336; }
#container { width: 100%; max-width: 400px; margin: auto; }
.image {font-size: 0; text-align: center; width: 400px;  height: 150px;}
img {display: inline-block; vertical-align: middle; max-height: 100%; max-width: 100%;}
.trick {display: inline-block; vertical-align: middle; height: 150px;}
h1 {  display: block; font-size: 2em; margin-top: 0.67em; margin-bottom: 0.67em; margin-left: 0; margin-right: 0; font-weight: bold; text-align: center; color: #f44336; }
  a { 
  background-color: #ff0000;
    border: none;
    color: white;
    padding: 15px 32px;
  margin-top: 0.67em; 
  margin-bottom: 0.67em;
  margin-right: 0em;
  margin-left: 0em; 
    text-align: center;
    text-decoration: none;
    display: block;
    font-size: 18px;
    font-weight: bold;
  border-radius: 4px;
  }
</style>
</head>
<body>
<div id="container">
<div class="image">
<div class="trick"></div>
  <img src='http://armtronix.in/img/logo.png'/>
</div>
<h1>Wifi Two Relay Board</h1>
<a href=\gpio>Control Outputs</a><br />
<a href=\cleareeprom>Clear settings an boot into Configmode</a><br />
</div>
</body>
</html>
)rawliteral";

// ------------------------------------ END MAIN.html -----------------------------------------------------------------------------------------------------------------------------------

static const char PROGMEM CONFIG_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name = "viewport" content = "width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
<title>ARMtronix Technologies LLP</title>
</head>
<style>
body { background: #ffffff  repeat; font-family: Arial, Helvetica, Sans-Serif; font-weight: bold; Color:  #f44336; }
h1 {  display: block; font-size: 1.80em; margin-top: 0.67em; margin-bottom: 0.67em; margin-left: 0; margin-right: 0; font-weight: bold; text-align: center; color: #f44336; }
input[type=text] {
  width: 100%;
  padding: 8px 20px;
  margin: 8px 0;
  box-sizing: border-box;
}
input[type=password] {
  width: 100%;
  padding: 8px 20px;
  margin: 8px 0;
  box-sizing: border-box;
}
input[type=submit]{
  background-color: #ff0000;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-weight: bold; 
  box-sizing: border-box;
}
</style>
<body>
<h1>Wifi Two Relay Board</h1>
<form method='get' action='a'>
<label>SSID: </label><input type="text" name='ssid' length=32><label> Pass: </label><input name='pass' type='password' length=64></br>
<label>Channel:</label><input type="text" name='channel' length=32><label>BSSID: </label><input type="text" name='bssid' length=32></br>
<label>IOT Mode: </label><input type='radio' name='iot' value='0'>HTTP <input type='radio' name='iot' value='1' checked> MQTT</br>
<label>MQTT Broker IP/DNS: </label><input type="text" name='host' length=15></br>
<label>MQTT User Name: </label><input name='mqtt_user' length=15></br>
<label>MQTT Password: </label><input name='mqtt_passwd' type='password' length=15></br>
<label>MQTT Port: </label><input name='mqtt_port' value='1883' length=15></br>
<label>MQTT Publish topic: </label><input type="text" name='pubtop' length=64></br>
<label>MQTT Subscribe topic: </label><input type="text" name='subtop' length=64></br>
<input type='submit'>
</form>
</body>
</html>
)rawliteral";


void initWiFi()
{

Serial.println("Running initWiFI().....Startup");
  
if(esid!="")
    {
     if ( testWifi_bssid() == PASS_VALUE )
       {
        launchWeb(0);
        return;
       }
      else if ( testWifi_ssid() == PASS_VALUE )
       {
        launchWeb(0);
        return;
       }
      else
       {
        Serial.println("Failed to connect using BSSID and SSID");
       }
    }
else
    {
    Serial.println("No BSSID or SSID");
    }
Serial.println("Opening AP");
setupAP();   
}

/*###############################################################################################*/
int testWifi_bssid(void) {
  int c = 0;
  Debugln("Wifi test...");
  ebssid=ebssid_string.c_str();
  Serial.print("Connecting to WiFi using BSSID ");
  Serial.println(ebssid);
  Serial.print(" with ssid: ");
  Serial.println(esid);
  Serial.print(" with Channel: ");
  Serial.println(echannel);
  parseBytes(ebssid, '-', bssid_bytes, 6, 16);
  //Serial.println(macToStr_2(bssid_bytes));
  
  
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA);
  //Serial.print("Connecting to WiFi using BSSID");
  //Serial.println(esid);
  //Debugln(epass);
  WiFi.begin((char*)esid.c_str(), (char*)epass.c_str(), echannel.toInt(), (uint8_t*) bssid_bytes);
  //WiFi.begin((char*)esid.c_str(), (char*)epass.c_str(), (int) echannel, (uint8_t*) ebssid); //old used before 12/04/2018
  //earlier this part was initwifi()

  while ( c < TIME_OUT_COUNT ) //changed by naren on 28/09/2017
  {
    if (WiFi.status() == WL_CONNECTED) {
      return (PASS_VALUE);
    }
    delay(500);
    Serial.print(".");
    c++;
  }
  Serial.println("WiFi Connect timed out!");
  return (FAIL_VALUE);
}

/*###############################################################################################*/

int testWifi_ssid(void) {
  int c = 0;
  Debugln("Wifi test...");

  // test esid //earlier this part was initwifi()
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to WiFi using SSID ");
  Serial.println(esid);
  Debugln(epass);
  WiFi.begin((char*)esid.c_str(), (char*)epass.c_str());
  //earlier this part was initwifi()

  while ( c < TIME_OUT_COUNT ) //changed by naren on 28/09/2017
  {
    if (WiFi.status() == WL_CONNECTED) {
      return (PASS_VALUE);
    }
    delay(500);
    Serial.print(".");
    c++;
  }
  Serial.println("WiFi Connect timed out!");
  return (FAIL_VALUE);
}

/*###############################################################################################*/

void setupAP(void) {
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0){
    Serial.println("no networks found");
    st ="<b>No networks found:</b>";
  } else {
    Serial.print(n);
    Serial.println(" Networks found");
    st = "<ul>";
    for (int i = 0; i < n; ++i)
     {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" (OPEN)":"*");
      
      // Print to web SSID and RSSI for each network found
      st += "<li>";
      st +=i + 1;
      st += ": ";
      st += WiFi.SSID(i);
      st += " (";
      st += WiFi.RSSI(i);
      st += ")";
      st += (WiFi.encryptionType(i) == ENC_TYPE_NONE)?" (OPEN)":"*";
      st += "</li>";
      //delay(10);
     }
    st += "</ul>";
  }
  Serial.println(""); 
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_AP);

  
  WiFi.softAP((char*)hostsaved.c_str());
  WiFi.begin((char*)hostsaved.c_str()); // not sure if need but works
  Serial.print("Access point started with name ");
  Serial.println(hostsaved);
  inApMode=1;
  launchWeb(1);
}

void launchWeb(int webtype) {
    Serial.println("");
    Serial.println("WiFi connected");    
    //Start the web server or MQTT
    if(otaFlag==1 && !inApMode){
      Serial.println("Starting OTA mode.");    
      Serial.printf("Sketch size: %u\n", ESP.getSketchSize());
      Serial.printf("Free size: %u\n", ESP.getFreeSketchSpace());
      MDNS.begin((char*)hostsaved.c_str());
      server.on("/", HTTP_GET, [](){
        server.sendHeader("Connection", "close");
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/html", otaServerIndex);
      });
      server.on("/update", HTTP_POST, [](){
        server.sendHeader("Connection", "close");
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
        setOtaFlag(0); 
        ESP.restart();
      },[](){
        HTTPUpload& upload = server.upload();
        if(upload.status == UPLOAD_FILE_START){
          //Serial.setDebugOutput(true);
          WiFiUDP::stopAll();
          Serial.printf("Update: %s\n", upload.filename.c_str());
          otaCount=300;
          uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
          if(!Update.begin(maxSketchSpace)){//start with max available size
            Update.printError(Serial);
          }
        } else if(upload.status == UPLOAD_FILE_WRITE){
          if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
            Update.printError(Serial);
          }
        } else if(upload.status == UPLOAD_FILE_END){
          if(Update.end(true)){ //true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
          } else {
            Update.printError(Serial);
          }
          Serial.setDebugOutput(false);
        }
        yield();
      });
      server.begin();
      
      Serial.printf("Ready! Open http://%s.local in your browser\n", (char*)hostsaved.c_str());
      MDNS.addService("http", "tcp", 80);
      MDNS.addService("ws", "tcp", 81);  //added by naren on 02/01/2019
//      otaTickLoop.attach(1, otaCountown);
    } else { 
      //setOtaFlag(1); 
      if (webtype==1 || iotMode==0){ //in config mode or WebControle
          if (webtype==1) {           
            webtypeGlob == 1;
            Serial.println(WiFi.softAPIP());
            server.on("/", webHandleConfig);
            server.on("/a", webHandleConfigSave);
            server.on("/gpio", webHandleGpio);//naren          
          } else {
             webSocket.begin();  //added by naren on 02/01/2019
             webSocket.onEvent(webSocketEvent);  //added by naren on 02/01/2019
             Serial.println("setting up MDNS responder now!");
            //setup DNS since we are a client in WiFi net
            if (!MDNS.begin((char*)hostsaved.c_str())) {
              Serial.println("Error setting up MDNS responder!");
            } else {
              Serial.println("mDNS responder started");
              MDNS.addService("http", "tcp", 80);
            }          
            Serial.println(WiFi.localIP());
            server.on("/", webHandleRoot);  
            server.on("/cleareeprom", webHandleClearRom);
            server.on("/gpio", webHandleGpio);
            server.on("/xml",handleXML);
          }
          //server.onNotFound(webHandleRoot);
          server.begin();
          Serial.println("Web server started");   
          webtypeGlob=webtype; //Store global to use in loop()
        } else if(webtype!=1 && iotMode==1){ // in MQTT and not in config mode     
          //mqttClient.setBrokerDomain((char*) mqttServer.c_str());//naren
          //mqttClient.setPort(1883);//naren
          mqttClient.setServer((char*) mqttServer.c_str(),mqtt_port.toInt());
          mqttClient.setCallback(mqtt_arrived);
          mqttClient.setClient(wifiClient);
          if (WiFi.status() == WL_CONNECTED){
            if (!connectMQTT()){
               // delay(2000);
                if (!connectMQTT()){                            
                  Serial.println("Could not connect MQTT.");
                  Serial.println("Starting web server instead.");
                  iotMode=0;
                  launchWeb(0);
                  webtypeGlob=webtype;
                }
              }                    
            }
      }
    }
}

void webHandleConfig(){
  IPAddress ip = WiFi.softAPIP();
  String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
  String s;
 
  s = "Configuration of " + hostName + " at ";
  s += ipStr;
  s += "<p><a href=\"/gpio\">Control GPIO</a><br />";
  s += st;
  s += "<form method='get' action='a'>";
  s += "<label>SSID: </label><input name='ssid' length=32><label> Pass: </label><input name='pass' type='password' length=64><label>Channel:</label><input name='channel' length=32><label>BSSID: </label><input name='bssid' length=32></br>";
 // s += "<label>SSID: </label><input name='ssid' length=32><label> Pass: </label><input name='pass' type='password' length=64></br>";
  s += "The following is not ready yet!</br>";
  s += "<label>IOT mode: </label><input type='radio' name='iot' value='0'> HTTP<input type='radio' name='iot' value='1' checked> MQTT</br>";
  s += "<label>MQTT Broker IP/DNS: </label><input name='host' length=15></br>";
  s += "<label>MQTT User Name: </label><input name='mqtt_user' length=15></br>";
  s += "<label>MQTT Password: </label><input name='mqtt_passwd' type='password' length=15></br>";
  s += "<label>MQTT Port: </label><input name='mqtt_port' value='1883' length=15></br>";
  s += "<label>MQTT Publish topic: </label><input name='pubtop' length=64></br>";
  s += "<label>MQTT Subscribe topic: </label><input name='subtop' length=64></br>";
  s += "<input type='submit'></form></p>";
  s += "\r\n\r\n";
  Serial.println("Sending 200");  
 // server.send(200, "text/html", s); 
  server.send_P(200, "text/html", CONFIG_HTML);
}

void webHandleConfigSave(){
  // /a?ssid=blahhhh&pass=poooo
  String s;
  s = "<p>Settings saved to eeprom and reset to boot into new settings</p>\r\n\r\n";
  server.send(200, "text/html", s); 
  Serial.println("clearing EEPROM.");
  clearConfig();
  String qsid; 
  qsid = server.arg("ssid");
  qsid.replace("%2F","/");
  Serial.println("Got SSID: " + qsid);
  esid = (char*) qsid.c_str();
  
  String qpass;
  qpass = server.arg("pass");
  qpass.replace("%2F","/");
  Serial.println("Got pass: " + qpass);
  epass = (char*) qpass.c_str();

  String qchannel;
  qchannel = server.arg("channel");
  qchannel.replace("%2F", "/");
  Serial.println("Got channel: " + qchannel);
  echannel = (char*) qchannel.c_str();

  String qbssid;
  qbssid = server.arg("bssid");
  qbssid.replace("%2F", "/");
  Serial.println("Got bssid: " + qbssid); 
  ebssid_string = (char*) qbssid.c_str();
  
  String qiot;
  qiot= server.arg("iot");
  Serial.println("Got iot mode: " + qiot);
  qiot=="0"? iotMode = 0 : iotMode = 1 ;

  mqttServer = (char*) server.arg("host").c_str();
  Serial.print("Got mqtt Server: ");
  Serial.println(mqttServer);

  String qmqtt_user;//qmqtt_user
  qmqtt_user = server.arg("mqtt_user");
  qmqtt_user.replace("%2F", "/");
  Serial.println("Got mqtt_user: " + qmqtt_user);
  mqtt_user = (char*) qmqtt_user.c_str();

  String qmqtt_passwd;//qmqtt_passwd
  qmqtt_passwd = server.arg("mqtt_passwd");
  qmqtt_passwd.replace("%2F", "/");
  Serial.println("Got mqtt_passwd: " + qmqtt_passwd);
  mqtt_passwd = (char*) qmqtt_passwd.c_str();

  String qmqtt_port;//qmqtt_port
  qmqtt_port = server.arg("mqtt_port");
  qmqtt_port.replace("%2F", "/");
  Serial.println("Got mqtt_port: " + qmqtt_port);
  mqtt_port = (char*) qmqtt_port.c_str();
  
  String qsubTop;
  qsubTop = server.arg("subtop");
  qsubTop.replace("%2F","/");
  Serial.println("Got subtop: " + qsubTop);
  subTopic = (char*) qsubTop.c_str();
  
  String qpubTop;
  qpubTop = server.arg("pubtop");
  qpubTop.replace("%2F","/");
  Serial.println("Got pubtop: " + qpubTop);
  pubTopic = (char*) qpubTop.c_str();
  
  double voltage_reg = (VOLTAGE_RESISTOR_UPSTREAM +VOLTAGE_RESISTOR_DOWNSTREAM) / VOLTAGE_RESISTOR_DOWNSTREAM;
  setmulPower=( 1000000.0 * 128 * V_REF * V_REF * voltage_reg / CURRENT_RESISTOR/ 48.0 / F_OSC );
  setmulVoltage=( 1000000.0 * 512 * V_REF * voltage_reg / 2.0 / F_OSC );
  setmulCurrent=( 1000000.0 * 512 * V_REF /CURRENT_RESISTOR / 24.0 / F_OSC );
  
  Serial.println("Got setmulPower: " + String(setmulPower));
  Serial.println("Got setmulVoltage: " + String(setmulVoltage));
  Serial.println("Got setmulCurrent: " + String(setmulCurrent));
  
  Serial.print("Settings written ");
  saveConfig()? Serial.println("sucessfully.") : Serial.println("not succesfully!");;
  Serial.println("Restarting!"); 
  //delay(1000);
  ESP.reset();
}

void webHandleRoot(){
  String s;
  s = "<p>BA015 WIFI TWO RELAY BOARD";
  s += "</p>";
  s += "<a href=\"/gpio\">Controle GPIO</a><br />";
  s += "<a href=\"/cleareeprom\">Clear settings and boot into Config mode</a><br />";
  s += "\r\n\r\n";
  //Serial.println("Sending 200");  
  //server.send(200, "text/html", s); 
  server.send_P(200, "text/html",MAIN_HTML);
}

void webHandleClearRom(){
  String s;
  s = "<p>Clearing the config and reset to configure new wifi<p>";
  s += "</html>\r\n\r\n";
  Serial.println("Sending 200"); 
  server.send(200, "text/html", s); 
  Serial.println("clearing config");
  clearConfig();
  delay(10);
  Serial.println("Done, restarting!");
  ESP.wdtDisable();
  ESP.restart();
}


void webHandleGpio(){
  String s;
   // Set GPIO according to the request
    if (server.arg("state_out_1")=="1" || server.arg("state_out_1")=="0" ) 
    {
      state_out_1 = server.arg("state_out_1").toInt();
      Serial.print("Light switched via web request to  ");      
      Serial.println(state_out_1);      
    }
    if (server.arg("state_out_2")=="1" || server.arg("state_out_2")=="0" ) 
    {
      state_out_2 = server.arg("state_led").toInt();
      Serial.print("Light switched via web request to  ");      
      Serial.println(state_out_2);      
    }
    if(server.arg("reboot")=="1")
    {
     ESP.reset(); 
    }
     
//    s = "TRIAC is now ";
//    s += (digitalRead(OUTPIN_1))?"ON":"OFF";
//    s += "<p>Change to <form action='gpio'><input type='radio' name='state_sw' value='1' ";
//    s += (digitalRead(OUTPIN_1))?"checked":"";
//    s += ">TRIAC_ON<input type='radio' name='state_sw' value='0' ";
//    s += (digitalRead(OUTPIN_1))?"":"checked";
//    s += ">TRIAC_OFF <input type='submit' value='Submit'></form></p>";  
//
//    s += "SSR is now ";
//    s += (digitalRead(OUTPIN_2))?"ON":"OFF";
//    s += "<p>Change to <form action='gpio'><input type='radio' name='state_led' value='1' ";
//    s += (digitalRead(OUTPIN_2))?"checked":"";
//    s += ">SSR_ON <input type='radio' name='state_led' value='0' ";
//    s += (digitalRead(OUTPIN_2))?"":"checked";
//    s += ">SSR_OFF <input type='submit' value='Submit'></form></p>"; 
//    
//    s += "<p>Change to <form action='gpio' name='state' method='GET' oninput='showValue(state_dimmer.value)'><input type='range' name='state_dimmer' id='state_dimmer' min='0' max='90' step='5' value='0'  >" ; 
//    s += "</form></p>";
//    s += "<script type='text/javascript'>";
//    s += "function showValue(dimming)";
//    s += "{  document.querySelector('#state_dimmer').value=dimming ;";
//    s += "   document.forms['state'].submit();";
//    s += "}";
//    s += "</script>";
//    s +="<p><a href=\"gpio?reboot=1\">Reboot</a></p>";
    //server.send(200, "text/html", "<html><head><script>var connection = new WebSocket('ws://'+location.hostname+':81/', ['arduino']);connection.onopen = function () {  connection.send('Connect ' + new Date()); }; connection.onerror = function (error) {    console.log('WebSocket Error ', error);};connection.onmessage = function (e) {  console.log('Server: ', e.data);};function sendRGB() {  var r = parseInt(document.getElementById('r').value).toString(16);  var g = parseInt(document.getElementById('g').value).toString(16);  var b = parseInt(document.getElementById('b').value).toString(16);  if(r.length < 2) { r = '0' + r; }   if(g.length < 2) { g = '0' + g; }   if(b.length < 2) { b = '0' + b; }   var rgb = '#'+r+g+b;    console.log('RGB: ' + rgb); connection.send(rgb); }</script></head><body>LED Control:<br/><br/>R: <input id=\"r\" type=\"range\" min=\"0\" max=\"255\" step=\"1\" oninput=\"sendRGB();\" /><br/>G: <input id=\"g\" type=\"range\" min=\"0\" max=\"255\" step=\"1\" oninput=\"sendRGB();\" /><br/>B: <input id=\"b\" type=\"range\" min=\"0\" max=\"255\" step=\"1\" oninput=\"sendRGB();\" /><br/></body></html>");
    //server.send(200, "text/html", s);
     server.send_P(200, "text/html", INDEX_HTML);    
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
 
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            //Serial.println("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
 
            // send message to client
            webSocket.sendTXT(num, "Connected");
        }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);
            Serial.write((const char *)payload);
            Serial.println();
            if(payload[0]=='P'&& payload[1]=='V')
            {
              String tempString_http=(const char *)payload;
              int ind1; 
              int ind2;
              ind1=tempString_http.indexOf(':');
              ind2=tempString_http.indexOf(',');
              P=tempString_http.substring(ind1+1,ind2+1).toFloat();
              V=tempString_http.substring(ind2+1).toFloat();
              calflag=1;
            }
            if(payload[2]=='1')
            { 
              String dum1=(const char *)payload;
              if(dum1.substring(4).toInt()==100)
              toggle_o1();
            }
           if(payload[2]=='2')
            {
              String dum2=(const char *)payload;
              if(dum2.substring(4).toInt()==100)
              toggle_o2();
            }
            break;
    }
 
}

void toggle_o1()
{
 state_out_1=state_out_1^1;
 Serial.print("R_1 switched via web request to ");
 Serial.println(((state_out_1)));  
}

void toggle_o2()
{
  state_out_2=state_out_2^1;
  Serial.print("R_2 switched via web request to ");
  Serial.println(((state_out_2))); 

}


void buildXML(){
  XML="<?xml version='1.0'?>";
  XML="<Title>";
  XML+="<activepower>";
  XML+=Values_HLW8021_ActivePower;
  XML+="</activepower>";
  XML+="<voltage>";
  XML+=Values_HLW8021_Voltage;
  XML+="</voltage>";
  XML+="<current>";
  XML+=Values_HLW8021_Current;
  XML+="</current>";
  XML+="<powerfactor>";
  XML+=Values_HLW8021_Powerfactor;
  XML+="</powerfactor>";
  XML+="<energy>";
  XML+=Values_HLW8021_Energy;
  XML+="</energy>";
  XML+="<t1>";
  XML+="Load 1:"+Status_output_one();
  XML+="</t1>";
  XML+="<t2>";
  XML+="Load 2:"+Status_output_two();
  XML+="</t2>";
  XML+="</Title>";
}

String Status_output_one()
{
  String ostatus_1;
if(digitalRead(OUTPIN_1))
{
  ostatus_1="ON";
}
else
{
  ostatus_1="OFF";
}
return ostatus_1;
}

String Status_output_two()
{
  String ostatus_2;
if(digitalRead(OUTPIN_2))
{
  ostatus_2="ON";
}
else 
{
  ostatus_2="OFF";
}
return ostatus_2;
}


void handleXML(){
  buildXML();
  server.send(200,"text/xml",XML);
}

