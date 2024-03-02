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
#include <config.h>
#include <PromLokiTransport.h>
#include <PrometheusArduino.h>

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

PromLokiTransport transport;
PromClient client(transport);

WriteRequest req(8);

TimeSeries ts1(5, "ambient_temp_celsius", "{host=\"esp32\",city=\"olekminsk\",location=\"top30\",user=\"alex\"}");
TimeSeries ts2(5, "ambient_humidity_percents", "{host=\"esp32\",city=\"olekminsk\",location=\"top30\",user=\"alex\"}");
TimeSeries ts3(5, "water_level_percents", "{host=\"esp32\",city=\"olekminsk\",location=\"top30\",user=\"alex\"}");
TimeSeries ts4(5, "inside_temp_celsius", "{host=\"esp32\",city=\"olekminsk\",location=\"top30\",place=\"first_floor\",user=\"alex\"}");
TimeSeries ts5(5, "inside_temp_celsius", "{host=\"esp32\",city=\"olekminsk\",location=\"top30\",place=\"second_floor\",user=\"alex\"}");
TimeSeries ts6(5, "inside_temp_celsius", "{host=\"esp32\",city=\"olekminsk\",location=\"top30\",place=\"garage\",user=\"alex\"}");
TimeSeries ts7(5, "btn_switch_state", "{host=\"esp32\",city=\"olekminsk\",location=\"top30\",description=\"up_pump\",user=\"alex\"}");
TimeSeries ts8(5, "btn_switch_state", "{host=\"esp32\",city=\"olekminsk\",location=\"top30\",description=\"down_pump\",user=\"alex\"}");

//----------------------------------Переменные-------------------------------
boolean PUMP_0_STATE = false;     // Состояние реле 0 (Насос внизу)
boolean OLD_PUMP_0_STATE = false; // Старое остояние реле 0 (Насос внизу)
boolean PUMP_1_STATE = false;     // Состояние реле 1 (Насос давление)
boolean OLD_PUMP_1_STATE = false; // Старое остояние реле 1 (Насос давление)
boolean w2Full = true;            // флаг уровня воды в нижнем баке (1-полный(>50%) 0-пустой)
int W1_MAX_LVL = 95;              // Макс уровень воды в верхнем баке
int W1_MIN_LVL = 10;              // Мин уровень воды в верхнем баке

//----------------------------------Типа многозадачность-------------------------------
unsigned long sens_prevMillis = 0;
const long sens_interval = 6000; // Частота опроса датчиков
unsigned long disp_prevMillis = 0;
const long disp_interval = 5000; // Отправка данных на дисплей
unsigned long water_prevMillis = 0;
const long water_interval = 2000; // Частота измерения уровня воды
unsigned long cloud_prevMillis = 0;
const long cloud_interval = 60000; // Частота отправки данных

//----------------------------------Адреса DS18b20-------------------------------
DeviceAddress ffloor = {0x28, 0xFF, 0xE3, 0xA9, 0x0, 0x16, 0x2, 0x1C};
DeviceAddress garage = {0x28, 0xFF, 0x14, 0x93, 0x0, 0x16, 0x2, 0xB8};
DeviceAddress outdoor = {0x28, 0xFF, 0x6C, 0x67, 0x0, 0x16, 0x1, 0x76};

//----------------------------------Адреса узлов с сети-------------------------------
const uint16_t this_node = 00;
const uint16_t disp_node = 01;

//----------------------------------Структуры с данными-------------------------------
struct DATA_STRUCTURE
{
  uint8_t w1;
  float t1;
  float t2;
  float t3;
  float t4;
  uint8_t humidity;
  boolean d_pump;
  boolean u_pump;
} __attribute__((packed));
DATA_STRUCTURE data;

//----------------------------------Данные на дисплей-------------------------------
void display_handler()
{
  // lcd.clear();
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
void rf24_receive_handler()
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
  }
}

//----------------------------------Отправка данных по RF24-------------------------------
void rf24_send_handler()
{
  RF24NetworkHeader transmitter(disp_node, 'T');
  boolean ok = network.write(transmitter, &data, sizeof(data));
  if (ok)
  {
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
  }
}

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
void cloud_update_handler(int64_t unix_time)
{
  ts1.addSample(unix_time, data.t4);
  ts2.addSample(unix_time, data.humidity);
  ts1.addSample(unix_time, data.w1);
  ts2.addSample(unix_time, data.t1);
  ts1.addSample(unix_time, data.t2);
  ts2.addSample(unix_time, data.t3);
  ts1.addSample(unix_time, data.u_pump);
  ts2.addSample(unix_time, data.d_pump);

  PromClient::SendResult res = client.send(req);

  if (!res == PromClient::SendResult::SUCCESS)
  {
    digitalWrite(BUILTIN_LED, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED, LOW);
    delay(50);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED, LOW);
    delay(50);
    digitalWrite(BUILTIN_LED, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED, LOW);
  }

  ts1.resetSamples();
  ts2.resetSamples();
  ts3.resetSamples();
  ts4.resetSamples();
  ts5.resetSamples();
  ts6.resetSamples();
  ts7.resetSamples();
  ts8.resetSamples();
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
    if (ok)
    {
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
    if (ok)
    {
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
  // Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  dht.begin();
  ds_sensor.begin();

  SPI.begin();
  radio.begin();
  radio.setChannel(70);
  network.begin(this_node);

  transport.setWifiSsid(WIFI_SSID);
  transport.setWifiPass(WIFI_PASSWORD);
  transport.begin();

  client.setUrl(URL);
  client.setPath((char *)PATH);
  client.setPort(PORT);
  client.setUser(USER);
  client.setPass(PASS);
  client.begin();

  req.addTimeSeries(ts1);
  req.addTimeSeries(ts2);
  req.addTimeSeries(ts3);
  req.addTimeSeries(ts4);
  req.addTimeSeries(ts5);
  req.addTimeSeries(ts6);
  req.addTimeSeries(ts7);
  req.addTimeSeries(ts8);

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

  data.t1 = 0;
  data.t2 = 0;
  data.t3 = 0;
  data.t4 = 0;
  data.w1 = 50;
  data.humidity = 0;
  data.u_pump = 0;
  data.d_pump = 0;
}

//////////////////////////////////////////////////////////

void loop()
{
  unsigned long currentMillis = millis();
  int64_t time;
  time = transport.getTimeMillis();
  network.update();
  btn_pump0.update();
  btn_pump1.update();

  //---------------------------------- Прием данных 2.4GHz wireless -------------------------------
  rf24_receive_handler();

  //----------------------------------Обработка кнопок-------------------------------

  button_handler();

  //----------------------------------Обработка датчиков-------------------------------
  if (currentMillis - sens_prevMillis >= sens_interval)
  {
    sens_prevMillis = currentMillis;
    ds_sensor.requestTemperatures();

    data.t1 = ds_sensor.getTempC(ffloor);
    data.t3 = ds_sensor.getTempC(garage);
    data.t4 = ds_sensor.getTempC(outdoor);
    data.t2 = dht.readTemperature();
    data.humidity = dht.readHumidity();
  }
  if (currentMillis - water_prevMillis >= water_interval)
  {
    water_prevMillis = currentMillis;
    data.w1 = map(w1_sens.ping_cm(), 70, 10, 0, 100);
    if (data.w1 < 0)
      data.w1 = 0;
    if (data.w1 > 100)
      data.w1 = 100;
  }
  //---------------------------------- Дисплей -------------------------------
  display_handler();

  //----------------------------------Отправка данных 2.4GHz wireless-------------------------------
  if (currentMillis - disp_prevMillis >= disp_interval)
  {
    disp_prevMillis = currentMillis;
    rf24_send_handler();
  }

  //----------------------------------Отправка данных в облако-------------------------------
  if (currentMillis - cloud_prevMillis >= cloud_interval)
  {
    cloud_prevMillis = currentMillis;
    cloud_update_handler(time);
  }

  //----------------------------------Управление реле-------------------------------

  relay_handler();

  //-----------------------------------------------------------------
  // delay(50);
}
