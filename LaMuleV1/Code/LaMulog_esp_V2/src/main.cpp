// Importation des librairies
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <Arduino_JSON.h>
#include <FS.h> //spiff file system

// Déclaration des constantes
#define ALT_ACTIVATION 1
#define DESCENTE_DECLENCHEMENT 1
#define Switch_1  D0
#define Switch_1_On  (!digitalRead(Switch_1))
#define WS2812_PIN D5

// Déclaration des variables
int pos = 0;    // variable to store the servo position
bool descente=false;  // Variable d'état. Vrai lorsque en mode descente
bool ready=false;
bool log2SD=true;
int phase_vol=0;
// Phase vol:
// 0: Inconnue
// 1: Mise a jour de la pression de reference donc altitude=0
// 10: Durablement stable
// 20: Décollage
// 30: Vol ascendant
// 40: Apogée
// 50: Vol descendant
// 60: Contact sol
double baselinePressure;  // Pression de référence au démarage du code
double alt_activation=ALT_ACTIVATION;  // Altitude minimum en dessous de laquelle l'ouverture de la coiffe ne peut se déclencher lors d'une decente
double lambda=0.1;  // Coefficient de filtration pour calculer l'altitude lissée à partir de l'altitude instantanée
double altitude_lissee=0;  // Altitude lissée
double ref_alt_10=0, ref_alt_20=0, ref_alt_30=0;  // Reference altitudes at t -5, -10 and -15 seconds. Used to detect stable state and regularly recalibrate altimeter.
double max_altitude=0;  // Altitude maximale atteinte depuis démarrage du secondes
double descente_declenchement=DESCENTE_DECLENCHEMENT; // Descente minimum requise pour déclencher l'ouverture de la coiffe
int max_altitude_dm;  // Is stored in EEPROM.
float accel_divisor=16384.0;  // Default divisor to convert MPU6050 accelerometer outputs into g
float gyro_divisor=131.0;  // Default divisor to convert MPU6050 gyrometer outputs into deg/s
float AccX, AccY, AccZ, Tmp, GyroX, GyroY, GyroZ; //These will be the raw data from the MPU6050.
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;  //Variables used for zeroing of errors from accel and zero
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime, ref_time=0;
int c = 0;
double CurrentTemp, CurrentPres, CurrentAlt;
uint8_t pin_led = 2;
uint16_t brightness=0;
uint16_t color=0;
String wifi_status = "Disconnected";

// Replace with your network credentials
const char* ssid = "tiffinanou";
const char* password = "WGrjLQxP8mWeTcbE";

char* mySsid = "lamulog";
// Type your own IP address and gateway - This not the address in use on project ;-)
IPAddress local_ip(192,168,168,168);
IPAddress gateway(192,168,168,1);
IPAddress netmask(255,255,255,0);
String current_ip, current_ssid;

//Création des objets
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;  
unsigned long timerDelay = 500;

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp; // I2C

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);

void ToggleLogging2SD(){
  log2SD=!log2SD;
  digitalWrite(pin_led,!digitalRead(pin_led));
  String datalogging_state = !digitalRead(pin_led) ? "Oui" : "Non";
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void wifiConnect()
{
  //reset networking
  //WiFi.softAPdisconnect(true);
  //WiFi.disconnect();
  //delay(1000);
  if (!Switch_1_On){  //Try to connect to known network first unless if button is pressed - in this case go to access point mode immediately
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
      if ((unsigned long)(millis() - startTime) >= 10000) break;
    }
    current_ssid=WiFi.SSID();
    current_ip=WiFi.localIP().toString();
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_ip, gateway, netmask);
    WiFi.softAP(mySsid, NULL);  // Creating a WiFi network with no password
    Serial.println("Unable to connect to a known wifi network - Starting ESP as Access Point.");
    wifi_status="Access Point";
    current_ssid=mySsid;
    current_ip="192.168.168.168";
  }
  else {Serial.println(""); Serial.println("ESP now connected in STAtion mode to known network.");Serial.println(WiFi.localIP().toString());wifi_status = "Station";}
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  
  while (c < 200) {
    mpu.getEvent(&a, &g, &temp);

    AccX = a.acceleration.x;
    AccY = a.acceleration.y;
    AccZ = a.acceleration.z;

    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));

    GyroX = g.gyro.x;
    GyroY = g.gyro.y;
    GyroZ = g.gyro.z;

    // Sum all readings
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;

    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;

  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

  // Print the error values on the Serial Monitor
  /*
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
  */
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Get Sensor Readings and return JSON object
String getSensorReadings(){
  readings["altitude"] = String(altitude_lissee);
  readings["max_altitude"] = String(max_altitude);
  String jsonString = JSON.stringify(readings);
  return jsonString;
}

// Initialize LittleFS
void initFS() {
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  else{
   Serial.println("LittleFS mounted successfully");
  }
}

void notifyClients(String sensorReadings) {
  ws.textAll(sensorReadings);
  //Serial.println("NotifyClient");
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    //data[len] = 0;
    //String message = (char*)data;
    // Check if the message is "getReadings"
    //if (strcmp((char*)data, "getReadings") == 0) {
      //if it is, send current sensor readings
      String sensorReadings = getSensorReadings();
      //Serial.print(sensorReadings);
      notifyClients(sensorReadings);
      //Serial.println("NotifyClient in handleWebSocketMessage");
    //}
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      if (!log2SD){Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());}
      break;
    case WS_EVT_DISCONNECT:
      if (!log2SD){Serial.printf("WebSocket client #%u disconnected\n", client->id());}
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup(void) {
  delay(100);

  pinMode (LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(Switch_1, INPUT_PULLUP);  // It seems that there is no pull-up on the D0 pin used in first board. Doesn't work as intented.

  strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'
  
  colorWipe(strip.Color(0, 255, 0), 50);
  if (log2SD){delay(15000);}  // Wait to ensure datalogger is up and running

  Serial.begin(115200);
  while (!Serial)
  delay(500); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Starting!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

if (!bmp.begin(0x76)) { 
Serial.println("Could not find a valid BMP280 sensor, check wiring!");
while (1);
}

bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
  

delay(1000);

colorWipe(strip.Color(0, 0, 255), 50);

  //get_alt_values(CurrentTemp, CurrentPres);
  baselinePressure = bmp.readPressure()/100.0;  // Get the baselinePressure pressure:

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(100);

  calculate_IMU_error();

  if (!log2SD){Serial.println("(5) Setup Wifi.");}
  wifiConnect();
  //initWiFi();
  initFS();
  initWebSocket();
  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");

  // Start server
  server.begin();

  randomSeed( analogRead( A0 ) );


  //delay(8000); // Wait so the data logger can start
  // Write header
 
  Serial.println("millis,roll,pitch,yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,CurrentTemp,CurrentPres,altitude,altitude_lissee,max_altitude,phase_vol");
  delay(500);
  if (wifi_status=="Access Point"){colorWipe(strip.Color(255, 0, 0), 50);}
  if (wifi_status=="Station"){colorWipe(strip.Color(0, 255, 255), 50);}
}

void loop() {

  delay(1);



  // === Read acceleromter data === //
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = a.acceleration.x;
  AccY = a.acceleration.y;
  AccZ = a.acceleration.z;
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) -AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) -AccErrorY; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  GyroX = g.gyro.x;
  GyroY = g.gyro.y;
  GyroZ = g.gyro.z;
  // Correct the outputs with the calculated error values
  GyroX = GyroX -GyroErrorX;
  GyroY = GyroY -GyroErrorY;
  GyroZ = GyroZ -GyroErrorZ;
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
      
  // collect altimeter data
  CurrentTemp=bmp.readTemperature();
  CurrentPres=bmp.readPressure();
  CurrentAlt=bmp.readAltitude(baselinePressure);
  altitude_lissee=lambda*CurrentAlt+(1-lambda)*altitude_lissee;

/*
  // Regularly reset reference altitude to zero if it is stable for 30 seconds
  if(millis()-ref_time>=10000){
    ref_time=millis();
    ref_alt_30=ref_alt_20;
    ref_alt_20=ref_alt_10;
    ref_alt_10=altitude_lissee;      
    if (((phase_vol==0)||(phase_vol==10))&(abs(altitude_lissee-ref_alt_10)<alt_activation)&(abs(altitude_lissee-ref_alt_20)<alt_activation)&(abs(altitude_lissee-ref_alt_30)<alt_activation)){
      baselinePressure = CurrentPres/100.0;  // Set baselinePressure pressure again:
      phase_vol=1;
    }
  }
*/

  if (altitude_lissee>max_altitude){max_altitude=altitude_lissee;}
  if((max_altitude-altitude_lissee>=descente_declenchement)&&(altitude_lissee>alt_activation)&&(descente==false)){
    descente=true;
    max_altitude_dm=int(max_altitude*10);
    //Serial.print("max_altitude_dm now updated to: ");Serial.println(max_altitude_dm);
    //EEPROM.write(1, highByte(max_altitude_dm));
    //EEPROM.write(2, lowByte(max_altitude_dm));
  }
  // Write to serial
  if (log2SD){
    Serial.print(millis());
    Serial.print(",");Serial.print(roll);
    Serial.print(",");Serial.print(pitch);
    Serial.print(",");Serial.print(yaw);
    Serial.print(",");Serial.print(AccX);
    Serial.print(",");Serial.print(AccY);
    Serial.print(",");Serial.print(AccZ);
    Serial.print(",");Serial.print(GyroX);
    Serial.print(",");Serial.print(GyroY);
    Serial.print(",");Serial.print(GyroZ);
    Serial.print(",");Serial.print(CurrentTemp);
    Serial.print(",");Serial.print(CurrentPres);
    Serial.print(",");Serial.print(CurrentAlt);
    Serial.print(",");Serial.print(altitude_lissee);
    Serial.print(",");Serial.print(max_altitude);
    Serial.print(",");Serial.print(phase_vol);
    //Serial.print(",");Serial.print(Switch_1_On);
    Serial.println(" ");
  }
  if (phase_vol==1){phase_vol=10;}  // Altimeter zeroing has been recorded, now switch back to stable mode

  if ((millis() - lastTime) > timerDelay) {
    String sensorReadings = getSensorReadings();
    //Serial.print(sensorReadings);
    notifyClients(sensorReadings);

  lastTime = millis();

  }

  ws.cleanupClients();
  //delay(5);

}