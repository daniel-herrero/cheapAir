#include <MQ7.h>

#include <Wire.h>
#include <UnoWiFiDevEd.h>


static unsigned long timer;

int ledCO = 8;
int ledNO2 = 9;
int ledO3 = 10;
int ledG = 11;

float noDangerCOLimit = 50;
float dangerCOLimit= 100;

int brightness = 0;
int warnstep = 5;

const int sensorCO = A1;
const int sensorNO2 = A2;
const int sensorO3 = A3;

int valueCO = 0;
int valueNO2 = 0;
int valueO3 = 0;

void setup() {
  // put your setup code here, to run once:
  timer = millis() + 1000;
  Serial.println(timer);
  
  Wifi.begin();
  Wifi.println("Web Server is up");

  pinMode(ledCO, OUTPUT);
  pinMode(ledNO2, OUTPUT);
  pinMode(ledO3, OUTPUT);
  pinMode(ledG, OUTPUT);
  Serial.begin(9600);
}

float readCO(){
  MQ7 mq7(A1, 5.0); 
  float valueCO = mq7.getPPM();
  return valueCO;
}

int readNO2(){
  valueNO2 = analogRead(sensorNO2);
  return valueNO2;
}

int readO3(){
  valueO3 = analogRead(sensorO3);
  return valueO3;
}

void shutdownLed(int led){
  analogWrite(led, 0);
}

void lightLedHalf(int led){
  analogWrite(led, 30);
}

void lightLedFull(int led){
  analogWrite(led, 150);
}

void ledCOoutput(float ppmCOValue){
   if (ppmCOValue > dangerCOLimit) {
    lightLedFull(ledNO2);
   } else if (ppmCOValue > noDangerCOLimit) {
    lightLedHalf(ledNO2);
   } else {
    shutdownLed(ledNO2);
   }
}

void ledNO2output(int v){
   int w = v/5;
   analogWrite(ledNO2, w);
   
}

void ledO3output(int v){
   int w = v/6;
   analogWrite(ledO3, w);
   
}

void ledGoutput(int u, int v, int w){ 
    int g = (u + v + w)/50;
    analogWrite(ledG, g);
    
}


void process(WifiData client, int co, int no2, int o3, int g) {
  // read the command
  String command = client.readStringUntil('/');

  if (command == "webserver3") {
    WebServer(client, co, no2, o3, g);
  }
}

void WebServer(WifiData client, int co, int no2, int o3, int g) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println("Refresh: 20");  // refresh the page automatically every  sec
  client.println();
  client.println("<html>");
  client.println("<head> <title>UNO WIFI Example</title> </head>");
  client.print("<meta http-equiv=\"refresh\" content=\"0\">");
  client.print("<body>");

  client.print("CO = ");
  client.print(co);
  client.print(" ppm");
  client.print("<br/>");

  client.print("NO2 = ");
  client.print(no2);
  client.print(" ppm");
  client.print("<br/>");

  client.print("O3 = ");
  client.print(o3);
  client.print(" ppm");
  client.print("<br/>");
  
  client.print("G = ");
  client.println(g);
  client.print(" ppm");
  client.print("<br/>");

  client.print("</body>");
  client.println("</html>");
  client.print(DELIMITER); // very important to end the communication !!!
}

void loop() {
  // put your main code here, to run repeatedly:

  int co = readCO();
  int no2 = readNO2();
  int o3 = readO3();
  int g = (co + no2 + o3)/100;
//
//  int co = 1000;
//  int no2 = 1000;
//  int o3 = 1000;
//  int g = 1000;

  while(Wifi.available()){
    process(Wifi, co, no2, o3, g);
  }

 if( (long)(millis()-timer) >= 0) {
//    shutdownLeds();
    ledCOoutput(co);
    //ledNO2output(150);
    ledO3output(30);
    ledGoutput(1,1,1);

    timer += 1000;
  }
  
  Serial.print("CO = ");
  Serial.print(co);
  Serial.print("\t NO2 = ");
  Serial.print(no2);
  Serial.print("\t O3 = ");
  Serial.print(o3);
  Serial.print("\t G = ");
  Serial.println(g);
}
