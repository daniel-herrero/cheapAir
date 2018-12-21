

/****************************************************************************
 CheapAir pollution sensor by Daniel Herrero & Pablo Ruiz @ Kaleidos
 XV PIWEEK project
*****************************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <SPI.h>
#include <WiFi101.h>
#include <WiFiMDNSResponder.h>

// OLED related


Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);


void displayText(char txt[]){
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(txt);
  display.display();
  
}


// MQ7 related
#include <MQ7.h>



int ledCO = 11;
float COnoDangerLimit = 15; //50 ppm
float CODangerLimit = 20; // 100 ppm

const int sensorCO = A3; //MQ-7
float valueCO = 0;

float readCO(){
  MQ7 mq7(sensorCO, 3.3);
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


//MQ4 related

int ledCH4 = 10;
float CH4noDangerLimit = 30; //50 ppm
float CH4DangerLimit = 40; // 100 ppm
const int sensorch4 = A2; //MQ-4

float m = -0.318; //Slope
float b = 1.133; //Y-Intercept
float R0 = 11.820; //Sensor Resistance in fresh air from previous code



float readCH4(){
  float valueCH4 = analogRead(sensorch4);
  float sensor_volt = valueCH4 * (3.3 / 1023.0); //Convert analog values to voltage
  float RS_gas = ((3.3 * 10.0) / sensor_volt) - 10.0; //Get value of RS in a gas
  float ratio = RS_gas / R0;   // Get ratio RS_gas/RS_air

  double ppm_log = (log10(ratio) - b) / m; //Get ppm value in linear scale according to the the ratio value
  float ppmCH4 = pow(10, ppm_log); //Convert ppm value to log scale
  return valueCH4;
}

void ledCH4output(float ppmCH4Value){
   if (ppmCH4Value > CH4DangerLimit) {
    lightLedFull(ledCH4);
  } else if (ppmCH4Value > CH4noDangerLimit) {
    lightLedHalf(ledCH4);
   } else {
    shutdownLed(ledCH4);
   }
}

//------------------------------------------------

// MQ131 related


int ledNO2 = 10;
int ledO3 = 12;

const int sensorO3NO2 = A2; //MQ-131

int valueNO2 = 0;
int valueO3 = 0;

float NO2noDangerLimit = 0; //50 ppm
float NO2DangerLimit = 1; // 100 ppm

float O3noDangerLimit = 40; //50 ppm
float O3DangerLimit = 50; // 100 ppm

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

char mdnsName[] = "cheapair"; // the MDNS name that the board will respond to
                             // after WiFi settings have been provisioned
// Note that the actual MDNS name will have '.local' after
// the name above, so "wifi101" will be accessible on
// the MDNS name "wifi101.local".

WiFiServer server(80);

// Create a MDNS responder to listen and respond to MDNS name requests.
WiFiMDNSResponder mdnsResponder;

void WIFISetup(){

    WiFi.setPins(8,7,4,2);
      // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // Start in provisioning mode:
  //  1) This will try to connect to a previously associated access point.
  //  2) If this fails, an access point named "wifi101-XXXX" will be created, where XXXX
  //     is the last 4 digits of the boards MAC address. Once you are connected to the access point,
  //     you can configure an SSID and password by visiting http://wifi101/
  WiFi.beginProvision();

  server.begin();

  // Setup the MDNS responder to listen to the configured name.
  // NOTE: You _must_ call this _after_ connecting to the WiFi network and
  // being assigned an IP address.
  if (!mdnsResponder.begin(mdnsName)) {
    Serial.println("Failed to start MDNS responder!");
    while(1);
  }

  Serial.print("Server listening at http://");
  Serial.print(mdnsName);
  Serial.println(".local/");

  // you're connected now, so print out the status:
  printWiFiStatus();
    
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void WebServer(WiFiClient client, int co, int no2, int o3, int g){


    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
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
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
}



// General

int ledG = 13;

float GnoDangerLimit = 80; //50 ppm
float GDangerLimit = 90; // 100 ppm

static unsigned long timer;

int brightness = 0;

void shutdownLed(int led){
  analogWrite(led, 0);
}

void lightLedHalf(int led){
  analogWrite(led, 150);
}

void lightLedFull(int led){
  analogWrite(led, 255);
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

//pinMode(sensorch4, INPUT);
//pinMode(sensorCO, INPUT);
 
  //WIFISetup();

  pinMode(ledCO, OUTPUT);
  pinMode(ledNO2, OUTPUT);
  pinMode(ledO3, OUTPUT);
  pinMode(ledG, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setTextSize(1);
  display.setTextColor(WHITE);

  Serial.begin(9600);
  Serial.print("Calibrating...");                        //serial display
  displayText("Calibrating...");

  Ro = MQCalibration(sensorch4);                         //Calibrating the sensor. Please make sure the sensor is in clean air

  Serial.println("done!");                                 //serial display
  Serial.print("Ro= ");
  Serial.print(Ro);
  Serial.println("kohm");
  display.clearDisplay();
  

}


// LOOP

void loop() {
  // put your main code here, to run repeatedly:


  float co = readCO();
//  int no2 = readNO2();
//  int o3 = readO3();
  float ch4 = readCH4();
  float o3 = ch4;
  int g = co + ch4 + o3;
//
//  int co = 1000;
//  int no2 = 1000;
//  int o3 = 1000;
//  int g = 1000;


 if( (long)(millis()-timer) >= 0) {
    ledCOoutput(co);
    ledCH4output(ch4);
    ledO3output(o3);
    ledGoutput(co,ch4,o3);

    timer += 1000;
  }

  Serial.print("CO = ");
  Serial.print(co);
  Serial.print("\t CH4 = ");
  Serial.print(ch4);
  Serial.print("\t O3 = ");
  Serial.print(o3);
  Serial.print("\t G = ");
  Serial.println(g);

  display.setCursor(0,0);
  display.print("CO ppm:");
  display.println(co);
  display.print("CH4 ppm:");
  display.println(ch4);
  display.print("O3 ppm:");
  display.println(o3);
  display.print("General:");
  display.println(g);
  display.setCursor(0,0);
  display.display(); // actually display all of the above
  delay(10);
   display.clearDisplay();
  yield();
  delay(1000);
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
