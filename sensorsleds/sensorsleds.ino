int ledCO = 8;
int ledNO2 = 9;
int ledO3 = 10;
int ledG = 11;

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

  pinMode(ledCO, OUTPUT);
  pinMode(ledNO2, OUTPUT);
  pinMode(ledO3, OUTPUT);
  pinMode(ledG, OUTPUT);
  Serial.begin(9600);

}

int readCO(){
  valueCO = analogRead(sensorCO);
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

void ledCOoutput(int v){
   int w = v/2;
   analogWrite(ledCO, w);
   
}

void ledNO2output(int v){
   int w = v/2;
   analogWrite(ledNO2, w);
   
}

void ledO3output(int v){
   int w = v/2;
   analogWrite(ledO3, w);
   
}

void ledGoutput(int u, int v, int w){ 
    int g = (u + v + w)/50;
    analogWrite(ledG, g);
    
}

void shutdownLeds(){
  analogWrite(ledCO, 0);
  analogWrite(ledNO2, 0);
  analogWrite(ledO3, 0);
  analogWrite(ledG, 0);
  
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
  
  ledCOoutput(co);
  ledNO2output(no2);
  ledO3output(o3);
  ledGoutput(co,no2,o3);

  
  Serial.print("CO = ");
  Serial.print(co);
  Serial.print("\t NO2 = ");
  Serial.print(no2);
  Serial.print("\t O3 = ");
  Serial.print(o3);
  Serial.print("\t G = ");
  Serial.println(g);
  delay(2000);
  shutdownLeds();


}
