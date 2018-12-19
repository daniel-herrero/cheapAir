/****************************************************************************
 CheapAir pollution sensor by Daniel Herrero & Pablo Ruiz @ Kaleidos
 XV PIWEEK project
*****************************************************************************/

// MQ7 related
#include <MQ7.h>

#include <Wire.h>
#include <UnoWiFiDevEd.h>


int ledCO = 8;
float COnoDangerLimit = 50; //50 ppm
float CODangerLimit = 100; // 100 ppm

const int sensorCO = A1; //MQ-7
int valueCO = 0;

float readCO(){
  MQ7 mq7(A1, 5.0);
  float valueCO = mq7.getPPM();
  return valueCO;
}

void ledCOoutput(float ppmCOValue){
   if (ppmCOValue > CODangerLimit) {
    lightLedFull(ledCO);
  } else if (ppmCOValue > COnoDangerLimit) {
    lightLedHalf(ledCO);
   } else {
    shutdownLed(ledCO);
   }
}

//------------------------------------------------

// MQ131 related


int ledNO2 = 9;
int ledO3 = 10;

const int sensorO3NO2 = A2; //MQ-131

int valueNO2 = 0;
int valueO3 = 0;

float NO2noDangerLimit = 50; //50 ppm
float NO2DangerLimit = 100; // 100 ppm

float O3noDangerLimit = 50; //50 ppm
float O3DangerLimit = 100; // 100 ppm

// relevant setup values
int RL_VALUE=5;                                     //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=11.0;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                    //which is derived from the chart in datasheet

// macros
int CALIBARAION_SAMPLE_TIMES=50;                    //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=500;                //define the time interal(in milisecond) between each samples in the
                                                    //cablibration phase
int READ_SAMPLE_INTERVAL=50;                        //define how many samples you are going to take in normal operation
int READ_SAMPLE_TIMES=5;                            //define the time interal(in milisecond) between each samples in
                                                    //normal operation

#define         GAS_NOx           0
#define         GAS_O3            1

// sensitivity adjustments
float           NOxCurve[3]  =  {0.698,0.954,-0.40};   //two points are taken from the curve.
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
float           O3Curve[3]  =  {0.698,0.903,-0.45};    //two points are taken from the curve.
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)

float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms


int readNO2(){
  int iPPM_NOx = MQGetGasPercentage(MQRead(sensorO3NO2)/Ro,GAS_NOx);
  return iPPM_NOx;
}

int readO3(){
  int iPPM_O3 = MQGetGasPercentage(MQRead(sensorO3NO2)/Ro,GAS_O3);
  return iPPM_O3;
}

void ledNO2output(int ppmNO2Value){
  if (ppmNO2Value > NO2DangerLimit) {
   lightLedFull(ledNO2);
 } else if (ppmNO2Value > NO2noDangerLimit) {
   lightLedHalf(ledNO2);
  } else {
   shutdownLed(ledNO2);
  }

}

void ledO3output(int ppmO3Value){
  if (ppmO3Value > O3DangerLimit) {
   lightLedFull(ledO3);
 } else if (ppmO3Value > O3noDangerLimit) {
   lightLedHalf(ledO3);
  } else {
   shutdownLed(ledO3);
  }
}
//------------------------------------------------

// WIFI SERVER



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



// General

int ledG = 11;

float GnoDangerLimit = 50; //50 ppm
float GDangerLimit = 100; // 100 ppm

static unsigned long timer;

int brightness = 0;

void shutdownLed(int led){
  analogWrite(led, 0);
}

void lightLedHalf(int led){
  analogWrite(led, 30);
}

void lightLedFull(int led){
  analogWrite(led, 150);
}

void ledGoutput(int u, int v, int w){
  int uvw = u+v+w;
  if (uvw > GDangerLimit) {
   lightLedFull(ledG);
 } else if (uvw > GnoDangerLimit) {
   lightLedHalf(ledG);
  } else {
   shutdownLed(ledG);
  }

}


// SETUP

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
  Serial.print("Calibrating...");                        //serial display

  Ro = MQCalibration(sensorO3NO2);                         //Calibrating the sensor. Please make sure the sensor is in clean air

  Serial.println("done!");                                 //serial display
  Serial.print("Ro= ");
  Serial.print(Ro);
  Serial.println("kohm");
}


// LOOP

void loop() {
  // put your main code here, to run repeatedly:


  int co = readCO();
  int no2 = readNO2();
  int o3 = readO3();
  int g = co + no2 + o3;
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
    ledGoutput(co,no2,o3);

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







// MQ-131 section

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   sensorO3NO2 - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  return val;                                                      //according to the chart in the datasheet

}

/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
long MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_NOx ) {
     return MQGetPercentage(rs_ro_ratio,NOxCurve);
  } else if ( gas_id == GAS_O3 ) {
     return MQGetPercentage(rs_ro_ratio,O3Curve);

  }

  return 0;
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
long  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
