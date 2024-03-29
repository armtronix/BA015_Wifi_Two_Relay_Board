bool loadConfig() {
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());

  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }

  int otaFlagC = json["otaFlag"];
  String esidC = json["esid"];
  String epassC = json["epass"];
  String echannelC = json["echannel"];  //added by naren on 11/11/17
  String ebssidC =json["ebssid"];   //added by naren on 11/11/17
  int iotModeC = json["iotMode"];
  String mqttServerC = json["mqttServer"];
  String mqtt_userC = json["mqtt_user"];      //added on 28/07/2018
  String mqtt_passwdC = json["mqtt_passwd"];  //added on 28/07/2018
  String mqtt_portC = json["mqtt_port"];  //added on 28/07/2018
  String pubTopicC = json["pubTopic"];
  String subTopicC = json["subTopic"];
  String tempsetmulCurrentC=json["setmulCurrentJson"];
  String tempsetmulVoltageC=json["setmulVoltageJson"];
  String tempsetmulPowerC= json["setmulPowerJson"];
  double setmulCurrentC = tempsetmulCurrentC.toFloat();
  double setmulVoltageC = tempsetmulVoltageC.toFloat();
  double setmulPowerC = tempsetmulPowerC.toFloat();

  // Real world application would store these values in some variables for
  // later use.
  otaFlag = otaFlagC;
  esid = esidC;
  epass = epassC;
  echannel=echannelC;  //added by naren on 11/11/17
  ebssid_string=ebssidC; //added by naren on 21/11/17
  iotMode = iotModeC;
  mqttServer = mqttServerC;
  mqtt_user = mqtt_userC; //added on 28/07/2018
  mqtt_passwd = mqtt_passwdC; //added on 28/07/2018
  mqtt_port = mqtt_portC; //added on 28/07/2018
  pubTopic = pubTopicC;
  subTopic = subTopicC;
  setmulCurrent =setmulCurrentC;
  setmulVoltage = setmulVoltageC;
  setmulPower = setmulPowerC;
  
  Serial.print("otaFlag: "); 
  Serial.println(otaFlag);
  Serial.print("esid: ");
  Serial.println(esid);
  Serial.print("epass: ");
  Serial.println(epass);
  Serial.print("echannel: "); //added by naren on 11/11/17
  Serial.println(echannel);   //added by naren on 11/11/17
  Serial.print("ebssid: ");  //added by naren on 11/11/1
  Serial.println(ebssid_string);    //added by naren on 11/11/17
  Serial.print("iotMode: ");
  Serial.println(iotMode);
  Serial.print("mqttServer: ");
  Serial.println(mqttServer);
  Serial.print("mqtt_user: ");  //added on 28/07/2018
  Serial.println(mqtt_user);    //added on 28/07/2018
  Serial.print("mqtt_passwd: "); //added on 28/07/2018
  Serial.println(mqtt_passwd);  //added on 28/07/2018
  Serial.print("mqtt_port: "); //added on 28/07/2018
  Serial.println(mqtt_port);  //added on 28/07/2018
  Serial.print("esid: ");
  Serial.println(esid);
  Serial.print("pubTopic: ");
  Serial.println(pubTopic);
  Serial.print("subTopic: ");
  Serial.println(subTopic);
  Serial.print("setmulCurrent: ");
  Serial.println(setmulCurrent);
  Serial.print("setmulVoltage: ");
  Serial.println(setmulVoltage);
  Serial.print("setmulPower: ");
  Serial.println(setmulPower);
  return true;
}

bool saveConfig() {
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["otaFlag"] = otaFlag;
  json["esid"] = esid;
  json["epass"] = epass;
  json["echannel"] = echannel;
  json["ebssid"]= ebssid_string;//added on 21/11/2017
  json["iotMode"] = iotMode;
  json["mqttServer"] = mqttServer;
  json["mqtt_user"] = mqtt_user; //added on 28/07/2018
  json["mqtt_passwd"] = mqtt_passwd; //added on 28/07/2018 
  json["mqtt_port"] = mqtt_port; //added on 28/07/2018 
  json["pubTopic"] = pubTopic;
  json["subTopic"] = subTopic;

  json["setmulCurrentJson"] = String(setmulCurrent);
  json["setmulVoltageJson"] = String(setmulVoltage);
  json["setmulPowerJson"] = String(setmulPower);

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  json.printTo(configFile);
  return true;
}


void setOtaFlag(int intOta){
  otaFlag=intOta;
  saveConfig();
  yield();
}

bool clearConfig(){
    Debugln("DEBUG: In config clear!");
    return SPIFFS.format();  
}

