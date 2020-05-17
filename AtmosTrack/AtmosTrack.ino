#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <analogWrite.h>
#include "BluetoothSerial.h"

WiFiClient client;
WiFiServer server(80);
/*
 * References:
 * https://www.youtube.com/watch?v=MicAM_A0_lU (ESP32CAM)
 * https://www.youtube.com/watch?v=a_PxAT8M-58&t=288s (ESP32CAM)
 * 
 * https://create.arduino.cc/projecthub/Arca_Ege/using-dht11-b0f365 (DHT 11)
 * https://www.youtube.com/watch?v=aQcJ4uHdQEA ( BLUETOOTH)
 * http://www.circuitstoday.com/interfacing-mq5-lpg-sensor-to-arduino (GAS SENSOR)
 * https://www.youtube.com/watch?v=3U8AEEQuyhY&t=838s Sensor to App
 * https://www.youtube.com/watch?v=a_PxAT8M-58&list=PLrHcdCn6WAiydEnRxgmOZojoeygAY_6eV&index=16 ESP32 CAM
 * 
 */

#define CAMERA_MODEL_AI_THINKER

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


#define led 2
#include <dht11.h>
#define DHT11PIN 25
#define Sensor 4

dht11 DHT11;
int sensor = 21;
int gasValue;


/* receives data from application */
String  data =""; 

/* define L298N motor control pins */
int leftMotorForward = 5;     /* GPIO2(D4) -> IN3   */
int rightMotorForward = 2;   /* GPIO15(D8) -> IN1  */
int leftMotorBackward = 18;    /* GPIO0(D3) -> IN4   */
int rightMotorBackward = 4;  /* GPIO13(D7) -> IN2  */


/* define L298N enable pins */
int rightMotorENB = 22; /* GPIO14(D5) -> Motor-A Enable */
int leftMotorENB = 23;  /* GPIO12(D6) -> Motor-B Enable */

BluetoothSerial SerialBT;

#include "camera_pins.h"

const char* ssid = "Hi";
const char* password = "password";

void startCameraServer();

void setup() {
  pinMode(sensor,INPUT);
  pinMode(led, OUTPUT);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println();


  //  Serial.begin(115200);
  connectWiFi();
  /* initialize motor control pins as output */
  pinMode(leftMotorForward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT); 
  pinMode(leftMotorBackward, OUTPUT);  
  pinMode(rightMotorBackward, OUTPUT);

  /* initialize motor enable pins as output */
  pinMode(leftMotorENB, OUTPUT); 
  pinMode(rightMotorENB, OUTPUT);

  /* start server communication */
  server.begin();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect"); 
}

void loop() {
  digitalWrite(led, HIGH);
  int chk = DHT11.read(DHT11PIN);

  Serial.print("Humidity: ");
  Serial.println((float)DHT11.humidity, 1);

  Serial.print("Temperature: ");
  Serial.println((float)DHT11.temperature, 1);

  gasValue = digitalRead(sensor);
  Serial.print("Gas Value: ");
  Serial.println(gasValue);
  
  SerialBT.print("Humidity: ");
  SerialBT.println((float)DHT11.humidity, 1);

  SerialBT.print("Temperature: ");
  SerialBT.println((float)DHT11.temperature, 1);

  SerialBT.print("Gas Value: ");
  SerialBT.println(gasValue);
  
  
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(1000);

  client = server.available();
  if (!client) return; 
  data = checkClient ();
  

/************************ Run the function according to incoming data from application *************************/

    /* If the incoming data is "forward", run the "MotorForward" function */
  if (data == "up") MotorForward();
    /* If the incoming data is "backward", run the "MotorBackward" function */
  else if (data == "down") MotorBackward();
    /* If the incoming data is "left", run the "TurnLeft" function */
  else if (data == "left") TurnLeft();
    /* If the incoming data is "right", run the "TurnRight" function */
  else if (data == "right") TurnRight();
    /* If the incoming data is "stop", run the "MotorStop" function */
  else if (data == "stop") MotorStop();  
  
}



void MotorForward(void)   
{
  digitalWrite(leftMotorENB,HIGH);
  digitalWrite(rightMotorENB,HIGH);
  digitalWrite(leftMotorForward,HIGH);
  digitalWrite(rightMotorForward,HIGH);
  digitalWrite(leftMotorBackward,LOW);
  digitalWrite(rightMotorBackward,LOW);
}


void MotorBackward(void)   
{
  digitalWrite(leftMotorENB,HIGH);
  digitalWrite(rightMotorENB,HIGH);
  digitalWrite(leftMotorBackward,HIGH);
  digitalWrite(rightMotorBackward,HIGH);
  digitalWrite(leftMotorForward,LOW);
  digitalWrite(rightMotorForward,LOW);
}


void TurnLeft(void)   
{
  digitalWrite(leftMotorENB,HIGH);
  digitalWrite(rightMotorENB,HIGH); 
  digitalWrite(leftMotorForward,LOW);
  digitalWrite(rightMotorForward,HIGH);
  digitalWrite(rightMotorBackward,LOW);
  digitalWrite(leftMotorBackward,HIGH);  
}


void TurnRight(void)   
{
  digitalWrite(leftMotorENB,HIGH);
  digitalWrite(rightMotorENB,HIGH);
  digitalWrite(leftMotorForward,HIGH);
  digitalWrite(rightMotorForward,LOW);
  digitalWrite(rightMotorBackward,HIGH);
  digitalWrite(leftMotorBackward,LOW);
}


void MotorStop(void)   
{
  digitalWrite(leftMotorENB,LOW);
  digitalWrite(rightMotorENB,LOW);
  digitalWrite(leftMotorForward,LOW);
  digitalWrite(leftMotorBackward,LOW);
  digitalWrite(rightMotorForward,LOW);
  digitalWrite(rightMotorBackward,LOW);
}


/********************************** RECEIVE DATA FROM the APP ******************************************/
String checkClient (void)
{
  while(!client.available()) delay(1); 
  String request = client.readStringUntil('\r');
  request.remove(0, 5);
  request.remove(request.length()-9,9);
  return request;
}


void connectWiFi()
{
  Serial.println("Connecting to WIFI");
  WiFi.begin(ssid, password);
  while ((!(WiFi.status() == WL_CONNECTED)))
  {
    delay(300);
    Serial.print("..");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("NodeMCU Local IP is : ");
  Serial.print((WiFi.localIP()));
}
