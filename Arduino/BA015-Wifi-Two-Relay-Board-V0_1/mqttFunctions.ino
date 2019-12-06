boolean connectMQTT() {
  if (mqttClient.connected()) {
    return true;
  }

  Serial.print("Connecting to MQTT server ");
  Serial.print(mqttServer);
  Serial.print(" as ");
  Serial.println(hostsaved);
  if(mqttClient.connect((char*)hostsaved.c_str(), (char*)mqtt_user.c_str(), (char*)mqtt_passwd.c_str(), (char*)pubTopic.c_str(), 0, 0, (char*)mqtt_will_msg_on_disconnecting.c_str())) //changed host to hostsaved  on 15/11/2019
  {
     mqttClient.loop();
    Serial.println("Connected to MQTT broker");
    if (mqttClient.subscribe((char*)subTopic.c_str())) {
      Serial.println("Subsribed to topic.");
    } else {
      Serial.println("NOT subsribed to topic!");
    }
    mqttClient.publish((char*)pubTopic.c_str(), (char*) mqtt_will_msg_on_connecting.c_str());
    return true;
  }
  else if (mqttClient.connect((char*)hostsaved.c_str())) 
  {
     mqttClient.loop();
    Serial.println("Connected to MQTT broker");
    if (mqttClient.subscribe((char*)subTopic.c_str())) {
      Serial.println("Subsribed to topic.");
    } else {
      Serial.println("NOT subsribed to topic!");
    }
    return true;
  }
  else 
  {
    Serial.println("MQTT connect failed! ");
    return false;
  }
}

void disconnectMQTT() 
{
  mqttClient.disconnect();
}

void mqtt_handler() 
{
  mqttClient.loop();
  delay(1); //let things happen in background
  if (toPub_1 == 1 || toPub_2 == 1 ) {
    Debugln("DEBUG: Publishing state via MQTT");
    if (pubState_1()) 
    {
      toPub_1 = 0; 
    }
    if (pubState_2()) 
    {
      toPub_2 = 0;  
    }
 }
  delay(1); //let things happen in background
}

void mqtt_arrived(char* subTopic, byte* payload, unsigned int length) { // handle messages arrived
  int i = 0;
  Serial.print("MQTT message arrived:  topic: " + String(subTopic));
  // create character buffer with ending null terminator (string)

  for (i = 0; i < length; i++) {
    buf[i] = payload[i];
  }
  buf[i] = '\0';
  String msgString = String(buf);
  Serial.println(" message: " + msgString);
  if (msgString == "R01_ON" && !digitalRead(OUTPIN_1)) 
  {
    Serial.print("Output1 is ");
    Serial.println(digitalRead(OUTPIN_1));
    Serial.print("Switching Output1 to ");
    Serial.println("high");
    state_out_1=state_out_1^1;
    //digitalWrite(OUTPIN_1, HIGH);
  } 
  else if (msgString == "R01_OFF" && digitalRead(OUTPIN_1)) 
  {
    Serial.print("Output1 is ");
    Serial.println(digitalRead(OUTPIN_1));
    Serial.print("Switching Output1 to ");
    Serial.println("low");
    state_out_1=state_out_1^1;
    //digitalWrite(OUTPIN_1, LOW);
  }
  else if (msgString == "R02_ON" && !digitalRead(OUTPIN_2)) 
  {
    Serial.print("Output2 is ");
    Serial.println(digitalRead(OUTPIN_2));
    Serial.print("Switching Output2 to ");
    Serial.println("high");
    state_out_2 =state_out_2^1;
    //digitalWrite(OUTPIN_2, HIGH);
  } 
  else if (msgString == "R02_OFF" && digitalRead(OUTPIN_2)) 
  {
    Serial.print("Output2 is ");
    Serial.println(digitalRead(OUTPIN_2));
    Serial.print("Switching Output2 to ");
    Serial.println("low");
    state_out_2 =state_out_2^1;
    //digitalWrite(OUTPIN_2, LOW);
  }
   else if (msgString.substring(0,6) == "SetPV:") 
  {
  String  tempString_mqtt =msgString;
  //float P,V;
  int ind1; 
  int ind2;
  ind1=tempString_mqtt.indexOf(':');
  ind2=tempString_mqtt.indexOf(',');
  P=tempString_mqtt.substring(ind1+1,ind2+1).toFloat();
  V=tempString_mqtt.substring(ind2+1).toFloat();
  //Serial.println(P);
  //Serial.println(V);
  calflag=1;
  
  //calibrate_sensor(P,V) ;
  }
   else if (msgString == "Reset") 
  {
    clearConfig();
    delay(10);
    Serial.println("Done, restarting!");
    ESP.wdtDisable();
    ESP.restart();
  }
 } 
 
    

boolean pubState_1() 
{
  if (!connectMQTT()) 
  {
    delay(100);
    if (!connectMQTT) 
    {
      Serial.println("Could not connect MQTT.");
      Serial.println("Publish state NOK");
      return false;
    }
  }
  if (mqttClient.connected()) 
  {
    Serial.println("To publish state " + o1_state_pub);
    String pubTopic_R1=pubTopic+"/R01";
    
    if(toPub_1==1)
    {
     if (mqttClient.publish((char*)pubTopic_R1.c_str(), (char*) o1_state_pub.c_str())) 
     {
      Serial.println("Publish state OK");
      return true;
     } 
     else 
     {
      Serial.println("Publish state NOK");
      return false;
     }
   } 
  }
  else 
  {
    Serial.println("Publish state NOK");
    Serial.println("No MQTT connection.");
  }
}

boolean pubState_2() 
{ 
  if (!connectMQTT()) 
  {
    delay(100);
    if (!connectMQTT) 
    {
      Serial.println("Could not connect MQTT.");
      Serial.println("Publish state NOK");
      return false;
    }
  }
  if (mqttClient.connected()) 
  {
    Serial.println("To publish state "+o2_state_pub);
    String pubTopic_R2=pubTopic+"/R02";
    if(toPub_2==1)
    {
     if (mqttClient.publish((char*)pubTopic_R2.c_str(), (char*) o2_state_pub.c_str())) 
     {
      Serial.println("Publish state OK");
      return true;
     } 
     else 
     {
      Serial.println("Publish state NOK");
      return false;
     }
   } 
  }
  else 
  {
    Serial.println("Publish state NOK");
    Serial.println("No MQTT connection.");
  }
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {
    0, -1
  };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

