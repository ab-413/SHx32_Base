#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <DallasTemperature.h>
#include <NewPing.h>
#include <OneWire.h>
#include <Bounce2.h>
#include <SPI.h>
#include <RF24Network.h>
#include <RF24.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>

#define MAX_DIST_W1 80
#define PIN_W1_T 12
#define PIN_W1_E 13

#define PUMP_0_BTN_PIN 32
#define PUMP_1_BTN_PIN 33
#define PIN_RALAY_0 25
#define PIN_RALAY_1 26

#define PIN_DS 27
#define PIN_DHT 14

#define RF_CE 4
#define RF_CSN 5

#define DHTTYPE DHT22

//----------------------------------Создание объектов-------------------------------
OneWire ds(PIN_DS);
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(PIN_DHT, DHTTYPE);
DallasTemperature ds_sensor(&ds);
NewPing w1_sens(PIN_W1_T, PIN_W1_E, MAX_DIST_W1);
Bounce btn_pump0 = Bounce();
Bounce btn_pump1 = Bounce();
RF24 radio(RF_CE, RF_CSN);
RF24Network network(radio);

//----------------------------------Переменные-------------------------------
boolean PUMP_0_STATE = false;     // Состояние реле 0 (Насос внизу)
boolean OLD_PUMP_0_STATE = false; // Старое остояние реле 0 (Насос внизу)
boolean PUMP_1_STATE = false;     // Состояние реле 1 (Насос давление)
boolean OLD_PUMP_1_STATE = false; // Старое остояние реле 1 (Насос давление)
boolean w2Full = true;            // флаг уровня воды в нижнем баке (1-полный(>50%) 0-пустой)
int W1_MAX_LVL = 95;              // Макс уровень воды в верхнем баке
int W1_MIN_LVL = 10;              // Мин уровень воды в верхнем баке

//----------------------------------Типа многозадачность-------------------------------
unsigned long SENS_prevMillis = 0;
const long SENS_interval = 6000; // Частота опроса датчиков
unsigned long DISP_prevMillis = 0;
const long DISP_interval = 5000; // Отправка данных на дисплей
unsigned long W_prevMillis = 0;
const long W_interval = 2000; // Частота измерения уровня воды
unsigned long Net_prevMillis = 0;
const long Net_interval = 300000; // Частота измерения уровня воды

//----------------------------------Адреса DS18b20-------------------------------
DeviceAddress ffloor = {0x28, 0xFF, 0xE3, 0xA9, 0x0, 0x16, 0x2, 0x1C};
DeviceAddress garage = {0x28, 0xFF, 0x14, 0x93, 0x0, 0x16, 0x2, 0xB8};
DeviceAddress outdoor = {0x28, 0xFF, 0x6C, 0x67, 0x0, 0x16, 0x1, 0x76};

//----------------------------------Адреса узлов с сети-------------------------------
const uint16_t this_node = 00;
const uint16_t disp_node = 01;
const uint16_t cont_node = 02;

//----------------------------------Параметры для Wi-Fi-------------------------------
const char *ssid = "********";
const char *serverName = "***********************";
String apiKeyValue = "**********";

//----------------------------------Структуры с данными-------------------------------
struct DATA_STRUCTURE
{
  uint8_t w1;
  float t1;
  float t2;
  float t3;
  float t4;
  float t5;
  uint8_t humidity;
  boolean d_pump;
  boolean u_pump;
} __attribute__((packed));
DATA_STRUCTURE data;

struct CONTAINER_STRUCTURE
{
  float temp;
  float cur_floor_temp;
  float trgt_floor_temp;
} __attribute__((packed));
CONTAINER_STRUCTURE container;

//----------------------------------Данные на дисплей-------------------------------
void display_handler()
{
  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(data.w1);
  lcd.print(" ");
  lcd.print(data.t1, 1);
  lcd.print(" ");
  lcd.print(data.t2, 1);
  lcd.setCursor(0, 1);
  lcd.print(data.t3, 1);
  lcd.print(" ");
  lcd.print(data.t4, 1);
  lcd.print(" ");
  lcd.print(container.temp, 1);
  if (PUMP_0_STATE)
  {
    lcd.setCursor(15, 0);
    lcd.print("@");
  }
  else
  {
    lcd.setCursor(15, 0);
    lcd.print(" ");
  }
  
  if (PUMP_1_STATE)
  {
    lcd.setCursor(15, 1);
    lcd.print("@");
  }
  else
  {
    lcd.setCursor(15, 1);
    lcd.print(" ");
  }
  
}

//----------------------------------Получение данных по RF24-------------------------------
void net_receive_handler()
{    
  while (network.available())
  {
    RF24NetworkHeader receiver;
    network.peek(receiver);
    if (receiver.type == 'T')
    {      
      network.read(receiver, &data, sizeof(data));

      PUMP_0_STATE = data.d_pump;
      OLD_PUMP_0_STATE = PUMP_0_STATE;
      PUMP_1_STATE = data.u_pump;
      OLD_PUMP_1_STATE = PUMP_1_STATE;      
    }
    if (receiver.type == 'N')
    {      
      network.read(receiver, &container, sizeof(container));

      data.t5 = container.temp;
    }
  }
}

//----------------------------------Отправка данных по RF24-------------------------------
void net_send_handler()
{ 
  RF24NetworkHeader transmitter(disp_node, 'T');
  boolean ok = network.write(transmitter, &data, sizeof(data)); 
  if (ok) {
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
  }  
}

// void net_send(void *pvParameter){
//     while(1){
//         network.update();

//         RF24NetworkHeader transmitter(disp_node, 'T');
//         boolean ok = network.write(transmitter, &data, sizeof(data));
//         if (ok) {
//           digitalWrite(BUILTIN_LED, HIGH);
//           delay(100);
//           digitalWrite(BUILTIN_LED, LOW);
//         }     
//         vTaskDelay(15000 / portTICK_PERIOD_MS);
//     }
// }

//----------------------------------Обработка реле-------------------------------
void relay_handler()
{
  if (data.w1 <= 5)
    PUMP_1_STATE = false;
  if (data.w1 >= W1_MAX_LVL || w2Full == false)
    PUMP_0_STATE = false;

  if (PUMP_0_STATE == true)
  {
    digitalWrite(PIN_RALAY_0, LOW);
  }
  if (PUMP_0_STATE == false)
  {
    digitalWrite(PIN_RALAY_0, HIGH);
  }
  if (PUMP_1_STATE == true)
  {
    digitalWrite(PIN_RALAY_1, LOW);
  }
  if (PUMP_1_STATE == false)
  {
    digitalWrite(PIN_RALAY_1, HIGH);
  }
}

//----------------------------------Обновление данных в базе-------------------------------
void net_update_handler()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;

    http.begin(serverName);

    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String httpRequestData = "api_key=" + apiKeyValue + "&w1=" + data.w1 + "&t1=" + data.t1 + "&t2=" + data.t2 + "&t3=" + data.t3 + "&t4=" + data.t4 + "&t5=" + data.t5 + "&hum=" + data.humidity + "";

    http.POST(httpRequestData);

    digitalWrite(BUILTIN_LED, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED, LOW);
    delay(50);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED, LOW);

    http.end();
  }
}

//----------------------------------ОБработка кнопок-------------------------------
void button_handler()
{
  if (btn_pump0.fell())
  {
    PUMP_0_STATE = !PUMP_0_STATE;
    data.d_pump = PUMP_0_STATE;
    RF24NetworkHeader transmitter(disp_node, 'T');
    boolean ok = network.write(transmitter, &data, sizeof(data));
    if (ok) {
      digitalWrite(BUILTIN_LED, HIGH);
      delay(100);
      digitalWrite(BUILTIN_LED, LOW);
    }     
    
    OLD_PUMP_0_STATE = PUMP_0_STATE;
  }
  if (btn_pump1.fell())
  {
    PUMP_1_STATE = !PUMP_1_STATE;
    data.u_pump = PUMP_1_STATE;
    RF24NetworkHeader transmitter(disp_node, 'T');
    boolean ok = network.write(transmitter, &data, sizeof(data));
    if (ok) {
      digitalWrite(BUILTIN_LED, HIGH);
      delay(100);
      digitalWrite(BUILTIN_LED, LOW);
    }     
    
    OLD_PUMP_1_STATE = PUMP_1_STATE;
  }
}

//////////////////////////////////////////////////////////

void setup()
{
  /* Инициализация */
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  dht.begin();
  ds_sensor.begin();

  SPI.begin();
  radio.begin();
  network.begin(70, this_node);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid);

  /* Инициализация кнопок */
  btn_pump0.attach(PUMP_0_BTN_PIN);
  btn_pump0.interval(50);
  btn_pump1.attach(PUMP_1_BTN_PIN);
  btn_pump1.interval(50);

  pinMode(PIN_RALAY_0, OUTPUT);
  digitalWrite(PIN_RALAY_0, HIGH);
  pinMode(PIN_RALAY_1, OUTPUT);
  digitalWrite(PIN_RALAY_1, HIGH);
  pinMode(PUMP_0_BTN_PIN, INPUT_PULLUP);
  pinMode(PUMP_1_BTN_PIN, INPUT_PULLUP);
  pinMode(BUILTIN_LED, OUTPUT);

  /* OTA */
  ArduinoOTA.setHostname("AlexESP32");

  ArduinoOTA.setPassword("fktrc");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  data.t1 = 0;
  data.t2 = 0;
  data.t3 = 0;
  data.t4 = 0;
  data.t5 = 0;
  data.w1 = 50;
  data.humidity = 0;
  data.u_pump = 0;
  data.d_pump = 0;

  container.temp = 0;

  //xTaskCreate(&net_send, "net_send", 2048, NULL, 1, NULL);
}

//////////////////////////////////////////////////////////

void loop()
{
  ArduinoOTA.handle();
  unsigned long currentMillis = millis();
  network.update();
  btn_pump0.update();
  btn_pump1.update(); 

  //---------------------------------- Прием данных 2.4GHz wireless -------------------------------
  net_receive_handler(); 

  //----------------------------------Обработка кнопок-------------------------------

  button_handler();

  //----------------------------------Обработка датчиков-------------------------------
  if (currentMillis - SENS_prevMillis >= SENS_interval)
  {
    SENS_prevMillis = currentMillis;
    ds_sensor.requestTemperatures();

    data.t1 = ds_sensor.getTempC(ffloor);
    data.t3 = ds_sensor.getTempC(garage);
    data.t4 = ds_sensor.getTempC(outdoor);
    data.t2 = dht.readTemperature();
    data.humidity = dht.readHumidity();
  }
  if (currentMillis - W_prevMillis >= W_interval)
  {
    W_prevMillis = currentMillis;
    data.w1 = map(w1_sens.ping_cm(), 70, 10, 0, 100);
    if (data.w1 < 0)
      data.w1 = 0;
    if (data.w1 > 100)
      data.w1 = 100;
  }
  //---------------------------------- Дисплей -------------------------------
  display_handler();

  //----------------------------------Отправка данных 2.4GHz wireless-------------------------------
  if (currentMillis - DISP_prevMillis >= DISP_interval)
  {
    DISP_prevMillis = currentMillis;
    net_send_handler();
  }

  //----------------------------------Отправка данных в облако-------------------------------
  if (currentMillis - Net_prevMillis >= Net_interval){
    Net_prevMillis = currentMillis;
    net_update_handler();
  }

  //----------------------------------Управление реле-------------------------------

  relay_handler();

  //-----------------------------------------------------------------
  //delay(50);
}
