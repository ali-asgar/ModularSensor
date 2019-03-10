#include <dht.h>

#include <SoftwareSerial.h>
#define RX 10
#define TX 11
String AP = "Ali";       // CHANGE ME
String PASS = "12345678"; // CHANGE ME
String API = "AR37FUEYFMWSVUYM";   // CHANGE ME
String HOST = "api.thingspeak.com";
String PORT = "80";
String field = "field1";
int countTrueCommand;
int countTimeCommand; 
boolean found = false; 
int valSensor = 1;
SoftwareSerial esp8266(RX,TX); 
#include "dht.h"
#define dht_apin A0 // Analog Pin sensor is connected to
 
dht DHT;
  
void setup() {
  Serial.begin(9600);
  esp8266.begin(115200);
  sendCommand("AT",5,"OK");
  sendCommand("AT+CWMODE=3",5,"OK");
  sendCommand("AT+CWJAP=\""+ AP +"\",\""+ PASS +"\"",20,"OK");
}
void loop() {
    DHT.read11(dht_apin);
    delay(100);
    Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.print("% \n");
    Serial.print("Current temperature = ");
    Serial.print(DHT.temperature); 
    Serial.println("C \n");
    delay(1000);
    int sensorValueOne = analogRead(A1);
    delay(100);
    Serial.print("Current lightOne = ");
    Serial.print(sensorValueOne);
    Serial.print("Lux \n");
    delay(1000);
    int sensorValueTwo = analogRead(A2);
    delay(100);
    Serial.print("Current lightTwo = ");
    Serial.print(sensorValueTwo);
    Serial.print("Lux \n");
    delay(1000);
    int sensorValue = analogRead(A3);
    delay(100);
    Serial.print("Current soil moisture = ");
    Serial.print(sensorValue);
    Serial.print(" m3 water/m3 soil \n");
    delay(1000);  
 String getData = "GET /update?api_key="+ API +"&field1="+String(DHT.temperature)+"&field2="+String(DHT.humidity)+"&field3="+String(sensorValueOne)+"&field4="+String(sensorValueTwo)+"&field5="+String(sensorValue); 
 sendCommand("AT+CIPMUX=1",5,"OK");
 sendCommand("AT+CIPSTART=0,\"TCP\",\""+ HOST +"\","+ PORT,15,"OK");
 sendCommand("AT+CIPSEND=0," +String(getData.length()+4),4,">");
 esp8266.println(getData);delay(1500);countTrueCommand++;
 sendCommand("AT+CIPCLOSE=0",5,"OK");
}

void sendCommand(String command, int maxTime, char readReplay[]) {
  Serial.print(countTrueCommand);
  Serial.print(". at command => ");
  Serial.print(command);
  Serial.print(" ");
  while(countTimeCommand < (maxTime*1))
  {
    esp8266.println(command);//at+cipsend
    if(esp8266.find(readReplay))//ok
    {
      found = true;
      break;
    }
  
    countTimeCommand++;
  }
  
  if(found == true)
  {
    Serial.println("OYI");
    countTrueCommand++;
    countTimeCommand = 0;
  }
  
  if(found == false)
  {
    Serial.println("Fail");
    countTrueCommand = 0;
    countTimeCommand = 0;
  }
  
  found = false;
 }
