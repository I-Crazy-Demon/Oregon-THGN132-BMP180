/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CrazyDemon
// Отсылка данных с датчиков по MQTT на домашний HomeAssistant и по TCP на сайт narodmon.ru
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Скетч для передачи показаний с беспроводных датчиков Oregon Scientific на сервис “Народный Мониторинга” (narodmon.ru)
//с помощью Arduino-совместимых плат на основе ESP8266 (Wemos D1, NodeMCU).
// https://github.com/invandy - библиотека и исходник скетча
//
//Для подключения необходимы:
//- Сам датчик Oregon Scientific THN132N, THGN132N, THGN123 и т.п.,
//- Плата микроконтроллера на основе ESP8266 (Wemos D1 или NodeMCU),
//- Приёмник OOK 433Мгц (Питание 3В, подключается к D7 платы микроконтроллера),
//- WiFi подключение к Интернет
//- Arduino IDE с установленной поддержкой ESP8266-совместимых устройств и библиотекой Oregon_NR
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Oregon_NR.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

ESP8266WebServer server(80); // включение сервера на порту:80

#define TEST_MODE        0   //Режим отладки (данные на narodmon.ru не отсылаются, изменить на 1 для отсылки)

//Кол-во датчиков различных типов используемых в системе. 
//Для экономии памяти можно обнулить неиспользуемые типы датчиков
#define NOF_132     2     //THGN132
#define NOF_500     0     //THGN500
#define NOF_968     0     //BTHR968
#define NOF_129     0     //BTHGN129
#define NOF_318     0     //RTGN318
#define NOF_800     0     //THGR810
#define NOF_THP     0     //THP

#define SEND_INTERVAL 300000            //Интервал отсылки данных на сервер, мс
//#define SEND_INTERVAL 10000            //Интервал отсылки данных на сервер, мс
#define CONNECT_TIMEOUT 10000           //Время ожидания  соединения, мс
#define DISCONNECT_TIMEOUT 10000        //Время ожидания отсоединения, мс

#define mac       "CrazyDemon463E52"  //МАС-адрес на narodmon.ru
#define ssid      "wifi"                //Параметры входа в WiFi, земенить на свои
#define password  "1234567890"

// MQTT config
#define AIO_SERVER      "192.168.1.3"
#define AIO_SERVERPORT  1883                   // def. 1883, use 8883 for SSL
#define AIO_USERNAME    "meteo"
#define AIO_KEY         "kjdsf894jkr3"
#define AIO_TOPIC       "1234567890"

// MQTT
/*
void callback(const MQTT::Publish& pub) {
  // handle message arrived
}
*/

WiFiClient wclient;
PubSubClient client(wclient, AIO_SERVER);

#define BLUE_LED 2      //Индикация подключения к WiFi
#define GREEN_LED 14    //Индикатор успешной доставки пакета а народмон
#define RED_LED 255     //Индикатор ошибки доставки пакета на народмон

//Параметоы соединения с narodmon:
char nardomon_server[] = "narodmon.ru"; 
int port=8283;

const unsigned long postingInterval = SEND_INTERVAL; 
unsigned long lastConnectionTime = 0;                   
boolean lastConnected = false;                          
unsigned long cur_mark;

//Анемометр
#define WIND_CORRECTION 0     //Коррекция севера на флюгере в градусах (используется при невозможности сориентировать датчик строго на север)
#define NO_WINDDIR      4     //Кол-во циклов передачи, необходимое для накопления данных для о направлении ветра

#define  N_OF_THP_SENSORS NOF_132 + NOF_500 + NOF_968 + NOF_129 + NOF_318 + NOF_800 + NOF_THP
//****************************************************************************************

Oregon_NR oregon(13, 13, 2, true); // Приёмник 433Мгц подключён к D7 (GPIO13), Светодиод на D2 подтянут к +пит.

//****************************************************************************************
//Структура для хранения полученных данных от термогигрометров:
struct BTHGN_sensor
{
  bool  isreceived = 0;           //Флаг о том, что по данному каналу приходил хоть один правильный пакет и данные актуальны
  byte  number_of_receiving = 0;  //сколько пакетов получено в процессе сбора данных
  unsigned long rcv_time = 7000000;// времена прихода последних пакетов
  byte  chnl;                     //канал передачи
  word  type;                     //Тип датчика
  float temperature;              //Температура
  float humidity;                 //Влажность. 
  float pressure;                 //Давление в мм.рт.ст.
  bool  battery;                  //Флаг батареи
  float voltage;                  //Напряжение батареи
};

BTHGN_sensor t_sensor[N_OF_THP_SENSORS];
//****************************************************************************************
//Структура для хранения полученных данных от анемометра:
struct WGR800_sensor
{
  bool  isreceived = 0;           //Флаг о том, что по данному каналу приходил хоть один правильный пакет и данные актуальны
  byte  number_of_receiving = 0;  //сколько пакетов получено в процессе сбора данных
  byte  number_of_dir_receiving = 0;  //сколько пакетов получено в процессе сбора данных
  unsigned long rcv_time = 7000000;// времена прихода последних пакетов
  
  float midspeed;                 //Средняя скорость ветра
  float maxspeed;                 //Порывы ветра
  float direction_x;              // Направление ветра
  float direction_y;
  byte dir_cycle = 0;             //Кол-во циклов накопления данных
  float dysp_wind_dir = -1;
  bool  battery;                 //Флаг батареи
};

WGR800_sensor wind_sensor;

//****************************************************************************************
//Структура для хранения УФ-индекса:
struct UVN800_sensor
{
  bool  isreceived = 0;           //Флаг о том, что по данному каналу приходил хоть один правильный пакет и данные актуальны
  byte  number_of_receiving = 0;  //сколько пакетов получено в процессе сбора данных
  unsigned long rcv_time = 7000000;// времена прихода последних пакетов
  
  float  index;                     //УФ-индекс
  bool battery;                    //Флаг батареи
};

UVN800_sensor uv_sensor;
//****************************************************************************************
//Структура для хранения полученных данных от счётчика осадков:
struct PCR800_sensor
{
  bool  isreceived = 0;           //Флаг о том, что по данному каналу приходил хоть один правильный пакет и данные актуальны
  byte  number_of_receiving = 0;  //сколько пакетов получено в процессе сбора данных
  unsigned long rcv_time = 7000000;// времена прихода последних пакетов
  
  float  rate;                    //Интенсивность осадков
  float  counter;                 //счётчик осадков
  bool battery;                   //Флаг батареи
};

PCR800_sensor rain_sensor;
//****************************************************************************************

///////////////////////////////////////////////////////////////////////////////////////////
// ВебСервер. Основная страница
void headroot(void) // Процедура обработки запроса GET , "/"
 {
  String  web ="", pref;

  web +="<html>";
  web +="<head>";
  web +="<meta charset='utf-8'>";
  web +="<title>Home Weather Station</title>";
  web +="<meta http-equiv='refresh' content='10'>";
  web +="<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  web +="<link href='https://fonts.googleapis.com/css?family=Open+Sans:300,400,600' rel='stylesheet'>";
  web +="<style>";
  web +="html { font-family: 'Open Sans', sans-serif; display: block; margin: 0px auto; text-align: center;color: #444444;}";
  web +="body{margin: 0px;} ";
  web +="h1 {margin: 50px auto 30px;} ";
  web +=".side-by-side{display: table-cell;vertical-align: middle;position: relative;}";
  web +=".text{font-weight: 600;font-size: 19px;width: 200px;}";
  web +=".reading{font-weight: 300;font-size: 50px;padding-right: 25px;}";
  web +=".temperature .reading{color: #F29C1F;}";
  web +=".humidity .reading{color: #3B97D3;}";
  web +=".pressure .reading{color: #26B99A;}";
  web +=".altitude .reading{color: #955BA5;}";
  web +=".superscript{font-size: 17px;font-weight: 600;position: absolute;top: 10px;}";
  web +=".data{padding: 10px;}";
  web +=".container{display: table;margin: 0 auto;}";
  web +=".icon{width:65px}";
  web +="</style>";
  web +="</head>";
  web +="<body>";
  web +="<h1>Home Weather Station</h1>";
  web +="<div class='container'>";
  web +="<div class='data pressure'>";
  web +="<div class='side-by-side icon'>";
  web +="<svg enable-background='new 0 0 40.542 40.541'height=40.541px id=Layer_1 version=1.1 viewBox='0 0 40.542 40.541'width=40.542px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M34.313,20.271c0-0.552,0.447-1,1-1h5.178c-0.236-4.841-2.163-9.228-5.214-12.593l-3.425,3.424";
  web +="c-0.195,0.195-0.451,0.293-0.707,0.293s-0.512-0.098-0.707-0.293c-0.391-0.391-0.391-1.023,0-1.414l3.425-3.424";
  web +="c-3.375-3.059-7.776-4.987-12.634-5.215c0.015,0.067,0.041,0.13,0.041,0.202v4.687c0,0.552-0.447,1-1,1s-1-0.448-1-1V0.25";
  web +="c0-0.071,0.026-0.134,0.041-0.202C14.39,0.279,9.936,2.256,6.544,5.385l3.576,3.577c0.391,0.391,0.391,1.024,0,1.414";
  web +="c-0.195,0.195-0.451,0.293-0.707,0.293s-0.512-0.098-0.707-0.293L5.142,6.812c-2.98,3.348-4.858,7.682-5.092,12.459h4.804";
  web +="c0.552,0,1,0.448,1,1s-0.448,1-1,1H0.05c0.525,10.728,9.362,19.271,20.22,19.271c10.857,0,19.696-8.543,20.22-19.271h-5.178";
  web +="C34.76,21.271,34.313,20.823,34.313,20.271z M23.084,22.037c-0.559,1.561-2.274,2.372-3.833,1.814";
  web +="c-1.561-0.557-2.373-2.272-1.815-3.833c0.372-1.041,1.263-1.737,2.277-1.928L25.2,7.202L22.497,19.05";
  web +="C23.196,19.843,23.464,20.973,23.084,22.037z'fill=#26B999 /></g></svg>";
  web +="</div>";
  web +="<div class='side-by-side text'>Давление</div>";
  web +="<div class='side-by-side reading'>";
  web +=bmp.readPressure()/133.3-0.1;
  web +="<span class='superscript'>мм.рт.ст.</span></div>";
  web +="</div>";

  web +=sendOregonData(1);

  web +="<div class='data temperature'>";
  web +="<div class='side-by-side icon'>";
  web +="<svg enable-background='new 0 0 19.438 54.003'height=54.003px id=Layer_1 version=1.1 viewBox='0 0 19.438 54.003'width=19.438px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g>";
  web +="<path d='M11.976,8.82v-2h4.084V6.063C16.06,2.715,13.345,0,9.996,0H9.313C5.965,0,3.252,2.715,3.252,6.063v30.982";
  web +="C1.261,38.825,0,41.403,0,44.286c0,5.367,4.351,9.718,9.719,9.718c5.368,0,9.719-4.351,9.719-9.718";
  web +="c0-2.943-1.312-5.574-3.378-7.355V18.436h-3.914v-2h3.914v-2.808h-4.084v-2h4.084V8.82H11.976z M15.302,44.833";
  web +="c0,3.083-2.5,5.583-5.583,5.583s-5.583-2.5-5.583-5.583c0-2.279,1.368-4.236,3.326-5.104V24.257C7.462,23.01,8.472,22,9.719,22";
  web +="s2.257,1.01,2.257,2.257V39.73C13.934,40.597,15.302,42.554,15.302,44.833z'fill=#F29C21 /></g></svg>";
  web +="</div>";
  web +="<div class='side-by-side text'>Температура станции</div>";
  web +="<div class='side-by-side reading'>";
  web +=bmp.readTemperature();
  web +="<span class='superscript'>&deg;C</span></div>";
  web +="</div>";

/*
  web +="<div class='data altitude'>";
  web +="<div class='side-by-side icon'>";
  web +="<svg enable-background='new 0 0 58.422 40.639'height=40.639px id=Layer_1 version=1.1 viewBox='0 0 58.422 40.639'width=58.422px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M58.203,37.754l0.007-0.004L42.09,9.935l-0.001,0.001c-0.356-0.543-0.969-0.902-1.667-0.902";
  web +="c-0.655,0-1.231,0.32-1.595,0.808l-0.011-0.007l-0.039,0.067c-0.021,0.03-0.035,0.063-0.054,0.094L22.78,37.692l0.008,0.004";
  web +="c-0.149,0.28-0.242,0.594-0.242,0.934c0,1.102,0.894,1.995,1.994,1.995v0.015h31.888c1.101,0,1.994-0.893,1.994-1.994";
  web +="C58.422,38.323,58.339,38.024,58.203,37.754z'fill=#955BA5 /><path d='M19.704,38.674l-0.013-0.004l13.544-23.522L25.13,1.156l-0.002,0.001C24.671,0.459,23.885,0,22.985,0";
  web +="c-0.84,0-1.582,0.41-2.051,1.038l-0.016-0.01L20.87,1.114c-0.025,0.039-0.046,0.082-0.068,0.124L0.299,36.851l0.013,0.004";
  web +="C0.117,37.215,0,37.62,0,38.059c0,1.412,1.147,2.565,2.565,2.565v0.015h16.989c-0.091-0.256-0.149-0.526-0.149-0.813";
  web +="C19.405,39.407,19.518,39.019,19.704,38.674z'fill=#955BA5 /></g></svg>";
  web +="</div>";
  web +="<div class='side-by-side text'>Высота над уровнем моря</div>";
  web +="<div class='side-by-side reading'>";
  web +=bmp.readAltitude();
  web +="<span class='superscript'>м</span></div>";
  web +="</div>";
*/

  // уровень WIFI сигнала
  int WIFIRSSI=constrain(((WiFi.RSSI()+100)*2),0,100);
  web +="<div class='data pressure'>";
  web +="<div class='side-by-side icon'>";
  web +="<svg version='1.1' id='Layer_1' xmlns='http://www.w3.org/2000/svg' xmlns:xlink='http://www.w3.org/1999/xlink' x='0px' y='0px'";
  web +="viewBox='0 0 424.264 424.264' style='enable-background:new 0 0 424.264 424.264;' xml:space='preserve'>";
  web +="<path style='fill:#2488FF;' d='M212.132,32.132C131.999,32.132,56.663,63.337,0,120l28.284,28.284";
  web +="c49.107-49.107,114.399-76.152,183.848-76.152s134.74,27.045,183.848,76.152L424.264,120";
  web +="C367.601,63.337,292.265,32.132,212.132,32.132z'/>";
  web +="<path style='fill:#2488FF;' d='M56.568,176.568l28.284,28.284c33.998-33.998,79.2-52.721,127.279-52.721";
  web +="s93.282,18.723,127.279,52.721l28.284-28.284c-41.553-41.553-96.799-64.437-155.563-64.437S98.121,135.016,56.568,176.568z'/>";
  web +="<path style='fill:#2488FF;' d='M113.137,233.137l28.284,28.284c38.99-38.989,102.432-38.989,141.422,0l28.284-28.284";
  web +="C256.541,178.551,167.723,178.551,113.137,233.137z'/>";
  web +="<path style='fill:#2488FF;' d='M152.132,332.132c0,33.084,26.916,60,60,60v-120C179.048,272.132,152.132,299.048,152.132,332.132z'/>";
  web +="<path style='fill:#005ECE;' d='M212.132,272.132v120c33.084,0,60-26.916,60-60S245.216,272.132,212.132,272.132z'/>";
  web +="</g></svg>";
  web +="</div>";
  web +="<div class='side-by-side text'>Уровень WI-FI (";
  web +=WiFi.SSID();
  web +=")</div>";
  web +="<div class='side-by-side reading'>";
  web +=WIFIRSSI;
  web +="<span class='superscript'>dBm</span></div>";
  web +="</div>";
  web +="</div>";

  uint32_t sec = millis() / 1000ul;
  int timeHours = (sec / 3600ul);
  int timeMins = (sec % 3600ul) / 60ul;
  int timeSecs = (sec % 3600ul) % 60ul;
  web +="<br /><br /><h3>Время работы станции: ";
  web +=timeHours;
  web +=":";
  web +=timeMins;
  web +=":";
  web +=timeSecs;
  web +="</h3>";

  web +="<br /><br /><h4><a href='/reboot'>";
  web +="Reboot";
  web +="</a></h4>";
  
  web +="</body>";
  web +="</html>"; 

 server.send(200, "text/html" , web );
 }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SETUP//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(BLUE_LED, OUTPUT);        
  pinMode(GREEN_LED, OUTPUT);        
  pinMode(RED_LED, OUTPUT);        

  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  while (1) {}
  }
  
  /////////////////////////////////////////////////////
  //Запуск Serial-ов
  Serial.begin(115200);
  Serial.println("Start Server");
  Serial.println("");
  
  if (TEST_MODE) Serial.println("TEST MODE");
  
/////////////////////////////////////////////////////
//Запуск Wifi
  wifi_connect();
/////////////////////////////////////////////////////

  digitalWrite(BLUE_LED, HIGH);    
  if (test_narodmon_connection()){
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
  }
  else {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
  }
  //вкючение прослушивания радиоканала  
  oregon.start(); 
  oregon.receiver_dump = 0;

//////////////////////////////////////////////////////////////////////
/* Вебсервер */
  server.on("/",headroot); // Ответ сервера на запрос главной страницы
  server.on("/reboot", handle_reboot); // Кнопка ребута
  server.begin(); //Запуск сервера

//////////////////////////////////////////////////////////////////////
// MQTT;
// client.set_callback(callback);
}

//////////////////////////////////////////////////////////////////////
//LOOP//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void loop() 
{
  //////////////////////////////////////////////////////////////////////
  //Защита от подвисаний/////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////  
  if  (micros() > 0xFFF00000) while ( micros() < 0xFFF00000); //Висим секунду до переполнения
  if  (millis() > 0xFFFFFC0F) while ( millis() < 0xFFFFFC0F); //Висим секунду до переполнения


  //////////////////////////////////////////////////////////////////////
  //Проверка полученных данных,/////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////  
  bool is_a_data_to_send = false;
  for (int i = 0; i < N_OF_THP_SENSORS; i++){
    if (t_sensor[i].number_of_receiving) is_a_data_to_send = 1;                 // Есть ли данные для отправки?
  }
   if (wind_sensor.number_of_receiving) is_a_data_to_send = 1;                 // Есть ли данные для отправки?
   if (rain_sensor.number_of_receiving) is_a_data_to_send = 1;                 // Есть ли данные для отправки?
   if (uv_sensor.number_of_receiving) is_a_data_to_send = 1;                 // Есть ли данные для отправки?
  //////////////////////////////////////////////////////////////////////
  //Отправка данных на narodmon.ru/////////////////////////////////////
  //////////////////////////////////////////////////////////////////////  
  
  if (millis() - lastConnectionTime > postingInterval && is_a_data_to_send)  {

    if (is_a_data_to_send)
    {
    //Обязательно отключить прослушивание канала
    oregon.stop();

//////////////////////////////////////////////////////////////////////////////////////////////////
// Отправляем MQTT
   mqtt_send();
//////////////////////////////////////////////////////////////////////////////////////////////////    

    digitalWrite(BLUE_LED, HIGH);    
    if (send_data()){
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
    }
    else {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
    }

    oregon.start();
    oregon.receiver_dump = 0;    
    }
    else Serial.println("No data to send");
  }

  //////////////////////////////////////////////////////////////////////
  //Захват пакета,//////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////  
  oregon.capture(0);
  //
  //Захваченные данные годны до следующего вызова capture
  //////////////////////////////////////////////////////////////////////
  //ОБработка полученного пакета
  //////////////////////////////////////////////
  if (oregon.captured)  
  {
    yield();
    //Вывод информации в Serial
    Serial.print ((float) millis() / 1000, 1); //Время
    Serial.print ("s\t\t");
    //Версия протокола
    if (oregon.ver == 2) Serial.print("  ");
    if (oregon.ver == 3) Serial.print("3 ");
    
    //Информация о восстановлени пакета
    if (oregon.restore_sign & 0x01) Serial.print("s"); //восстановлены одиночные такты
    else  Serial.print(" ");
    if (oregon.restore_sign & 0x02) Serial.print("d"); //восстановлены двойные такты
    else  Serial.print(" ");
    if (oregon.restore_sign & 0x04) Serial.print("p "); //исправленна ошибка при распознавании версии пакета
    else  Serial.print("  ");
    if (oregon.restore_sign & 0x08) Serial.print("r "); //собран из двух пакетов (для режима сборки в v.2)
    else  Serial.print("  ");

    //Вывод полученного пакета.
    for (int q = 0;q < oregon.packet_length; q++)
      if (oregon.valid_p[q] == 0x0F) Serial.print(oregon.packet[q], HEX);
      else Serial.print(" ");
        
    //Время обработки пакета
    Serial.print("  ");
    Serial.print(oregon.work_time);
    Serial.print("ms ");
    
    if ((oregon.sens_type == THGN132 ||
    (oregon.sens_type & 0x0FFF) == RTGN318 ||
    (oregon.sens_type & 0x0FFF) == RTHN318 ||
    oregon.sens_type == THGR810 ||
    oregon.sens_type == THN132 ||
    oregon.sens_type == THN800 ||
    oregon.sens_type == BTHGN129 ||
    oregon.sens_type == BTHR968 ||
    oregon.sens_type == THGN500) && oregon.crc_c)
    {
      Serial.print("\t");
      if (oregon.sens_type == THGN132) Serial.print("THGN132N");
      if (oregon.sens_type == THGN500) Serial.print("THGN500 ");
      if (oregon.sens_type == THGR810) Serial.print("THGR810 ");
      if ((oregon.sens_type & 0x0FFF) == RTGN318) Serial.print("RTGN318 ");
      if ((oregon.sens_type & 0x0FFF) == RTHN318) Serial.print("RTHN318 ");
      if (oregon.sens_type == THN132 ) Serial.print("THN132N ");
      if (oregon.sens_type == THN800 ) Serial.print("THN800  ");
      if (oregon.sens_type == BTHGN129 ) Serial.print("BTHGN129");
      if (oregon.sens_type == BTHR968 ) Serial.print("BTHR968 ");

      if (oregon.sens_type != BTHR968 && oregon.sens_type != THGN500)
      {
        Serial.print(" CHNL: ");
        Serial.print(oregon.sens_chnl);
      }
      else Serial.print("        ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print("ID: ");
      Serial.print(oregon.sens_id, HEX);
      
      if (oregon.sens_tmp >= 0 && oregon.sens_tmp < 10) Serial.print(" TMP:  ");
      if (oregon.sens_tmp < 0 && oregon.sens_tmp >-10) Serial.print(" TMP: ");
      if (oregon.sens_tmp <= -10) Serial.print(" TMP:");
      if (oregon.sens_tmp >= 10) Serial.print(" TMP: ");
      Serial.print(oregon.sens_tmp, 1);
      Serial.print("C ");
      if (oregon.sens_type == THGN132 ||
          oregon.sens_type == THGR810 ||
          oregon.sens_type == BTHGN129 ||
          oregon.sens_type == BTHR968 ||
          (oregon.sens_type & 0x0FFF) == RTGN318 ||
          oregon.sens_type == THGN500 ) {
        Serial.print("HUM: ");
        Serial.print(oregon.sens_hmdty, 0);
        Serial.print("%");
      }
      else Serial.print("        ");

      if (oregon.sens_type == BTHGN129 ||  oregon.sens_type == BTHR968)
      {
      Serial.print(" PRESS: ");
      Serial.print(oregon.get_pressure(), 1);
      Serial.print("Hgmm ");
      }

      if (oregon.sens_type == THGN132 && oregon.sens_chnl > NOF_132) {Serial.println(); return;}
      if (oregon.sens_type == THGN500 && NOF_500 == 0) {Serial.println(); return;}
      if (oregon.sens_type == BTHR968 && NOF_968 == 0) {Serial.println(); return;}
      if (oregon.sens_type == THGR810 && oregon.sens_chnl > NOF_800) {Serial.println(); return;}
      if (oregon.sens_type == BTHGN129 && oregon.sens_chnl > NOF_129) {Serial.println(); return;}
      if ((oregon.sens_type & 0x0FFF) == RTGN318 && oregon.sens_chnl > NOF_318) {Serial.println(); return;}
      
      byte _chnl = oregon.sens_chnl - 1;
           
      if (oregon.sens_type == THGN500) _chnl = NOF_132;
      if (oregon.sens_type == BTHR968 ) _chnl = NOF_132 + NOF_500; 
      if (oregon.sens_type == BTHGN129 ) _chnl = NOF_132 + NOF_500 + NOF_968; 
      if (oregon.sens_type == THGR810 || oregon.sens_type == THN800) _chnl = NOF_132 + NOF_500 + NOF_968 + NOF_129;
      if ((oregon.sens_type & 0x0FFF) == RTGN318 || (oregon.sens_type & 0x0FFF) == RTHN318) _chnl  = NOF_132 + NOF_500 + NOF_968 + NOF_129 + NOF_800;

      t_sensor[ _chnl].chnl = oregon.sens_chnl;
      t_sensor[ _chnl].number_of_receiving++;
      t_sensor[ _chnl].type = oregon.sens_type;
      t_sensor[ _chnl].battery = oregon.sens_battery;
      t_sensor[ _chnl].pressure = t_sensor[ _chnl].pressure * ((float)(t_sensor[ _chnl].number_of_receiving - 1) / (float)t_sensor[ _chnl].number_of_receiving) + oregon.get_pressure() / t_sensor[ _chnl].number_of_receiving;
      t_sensor[ _chnl].temperature = t_sensor[ _chnl].temperature * ((float)(t_sensor[ _chnl].number_of_receiving - 1) / (float)t_sensor[ _chnl].number_of_receiving) + oregon.sens_tmp / t_sensor[ _chnl].number_of_receiving;
      t_sensor[ _chnl].humidity = t_sensor[ _chnl].humidity * ((float)(t_sensor[ _chnl].number_of_receiving - 1) / (float)t_sensor[ _chnl].number_of_receiving) + oregon.sens_hmdty / t_sensor[ _chnl].number_of_receiving;
      t_sensor[ _chnl].rcv_time = millis();         
    }
    
    if (oregon.sens_type == PCR800 && oregon.crc_c)
    {
      Serial.print("\tPCR800  ");
      Serial.print("        ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print(" ID: ");
      Serial.print(oregon.sens_id, HEX);
      Serial.print("   TOTAL: ");
      Serial.print(oregon.get_total_rain(), 1);
      Serial.print("mm  RATE: ");
      Serial.print(oregon.get_rain_rate(), 1);
      Serial.print("mm/h");
      rain_sensor.number_of_receiving++;
      rain_sensor.battery = oregon.sens_battery;
      rain_sensor.rate = oregon.get_rain_rate();
      rain_sensor.counter = oregon.get_total_rain();
      rain_sensor.rcv_time = millis();         
    }    
    
  if (oregon.sens_type == WGR800 && oregon.crc_c){
      Serial.print("\tWGR800  ");
      Serial.print("        ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print("ID: ");
      Serial.print(oregon.sens_id, HEX);
      
      Serial.print(" AVG: ");
      Serial.print(oregon.sens_avg_ws, 1);
      Serial.print("m/s  MAX: ");
      Serial.print(oregon.sens_max_ws, 1);
      Serial.print("m/s  DIR: "); //N = 0, E = 4, S = 8, W = 12
      switch (oregon.sens_wdir)
      {
      case 0: Serial.print("N"); break;
      case 1: Serial.print("NNE"); break;
      case 2: Serial.print("NE"); break;
      case 3: Serial.print("NEE"); break;
      case 4: Serial.print("E"); break;
      case 5: Serial.print("SEE"); break;
      case 6: Serial.print("SE"); break;
      case 7: Serial.print("SSE"); break;
      case 8: Serial.print("S"); break;
      case 9: Serial.print("SSW"); break;
      case 10: Serial.print("SW"); break;
      case 11: Serial.print("SWW"); break;
      case 12: Serial.print("W"); break;
      case 13: Serial.print("NWW"); break;
      case 14: Serial.print("NW"); break;
      case 15: Serial.print("NNW"); break;
      }

      wind_sensor.battery = oregon.sens_battery;
      wind_sensor.number_of_receiving++;
      wind_sensor.number_of_dir_receiving++;
            
      //Средняя скорость
      wind_sensor.midspeed = wind_sensor.midspeed * (((float)wind_sensor.number_of_receiving - 1) / (float)wind_sensor.number_of_receiving) + oregon.sens_avg_ws / wind_sensor.number_of_receiving;
      
      //Порывы
      if (oregon.sens_max_ws > wind_sensor.maxspeed || wind_sensor.number_of_receiving == 1) wind_sensor.maxspeed = oregon.sens_max_ws;
      
      //Направление
      //Вычисляется вектор - его направление и модуль.
      if (wind_sensor.number_of_dir_receiving == 1 && (wind_sensor.direction_x != 0 || wind_sensor.direction_x != 0))
      {
        float wdiv = sqrt((wind_sensor.direction_x * wind_sensor.direction_x) + (wind_sensor.direction_y * wind_sensor.direction_y));
        wind_sensor.direction_x /= wdiv;
        wind_sensor.direction_y /= wdiv;
      }
      
      float wind_module = 1;
      if (oregon.sens_wdir == 0) {
        wind_sensor.direction_x += 1 * wind_module;
      }
      if (oregon.sens_wdir == 1) {
        wind_sensor.direction_x += 0.92 * wind_module;
        wind_sensor.direction_y -= 0.38 * wind_module;
      }
      if (oregon.sens_wdir == 2) {
        wind_sensor.direction_x += 0.71 * wind_module;
        wind_sensor.direction_y -= 0.71 * wind_module;
      }
      if (oregon.sens_wdir == 3) {
        wind_sensor.direction_x += 0.38 * wind_module;
        wind_sensor.direction_y -= 0.92 * wind_module;
      }
      if (oregon.sens_wdir == 4) {
        wind_sensor.direction_y -= 1 * wind_module;
      }
      
      if (oregon.sens_wdir == 5) {
        wind_sensor.direction_x -= 0.38 * wind_module;
        wind_sensor.direction_y -= 0.92 * wind_module;
      }
      if (oregon.sens_wdir == 6) {
        wind_sensor.direction_x -= 0.71 * wind_module;
        wind_sensor.direction_y -= 0.71 * wind_module;
      }
      if (oregon.sens_wdir == 7) {
        wind_sensor.direction_x -= 0.92 * wind_module;
        wind_sensor.direction_y -= 0.38 * wind_module;
      }
      if (oregon.sens_wdir == 8) {
        wind_sensor.direction_x -= 1 * wind_module;
      }
      if (oregon.sens_wdir == 9) {
        wind_sensor.direction_x -= 0.92 * wind_module;
        wind_sensor.direction_y += 0.38 * wind_module;
      }
      if (oregon.sens_wdir == 10) {
        wind_sensor.direction_x -= 0.71 * wind_module;
        wind_sensor.direction_y += 0.71 * wind_module;
      
      }
      if (oregon.sens_wdir == 11) {
        wind_sensor.direction_x -= 0.38 * wind_module;
        wind_sensor.direction_y += 0.92 * wind_module;
      }
      if (oregon.sens_wdir == 12) {
        wind_sensor.direction_y += 1 * wind_module;
      }
      if (oregon.sens_wdir == 13) {
        wind_sensor.direction_x += 0.38 * wind_module;
        wind_sensor.direction_y += 0.92 * wind_module;
      }
      if (oregon.sens_wdir == 14) {
        wind_sensor.direction_x += 0.71 * wind_module;
        wind_sensor.direction_y += 0.71 * wind_module;
      }
      if (oregon.sens_wdir == 15) {
        wind_sensor.direction_x += 0.92 * wind_module;
        wind_sensor.direction_y += 0.38 * wind_module;
      }
        wind_sensor.rcv_time = millis();
    }    

    if (oregon.sens_type == UVN800 && oregon.crc_c)
    {
      Serial.print("\tUVN800  ");
      Serial.print("        ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print("ID: ");
      Serial.print(oregon.sens_id, HEX);
      
      Serial.print(" UV IDX: ");
      Serial.print(oregon.UV_index);
      
      uv_sensor.number_of_receiving++;
      uv_sensor.battery = oregon.sens_battery;
      uv_sensor.index = uv_sensor.index * ((float)(uv_sensor.number_of_receiving - 1) / (float)uv_sensor.number_of_receiving) + oregon.UV_index / uv_sensor.number_of_receiving;
      uv_sensor.rcv_time = millis();         
    }    

    if (oregon.sens_type == RFCLOCK && oregon.crc_c){
      Serial.print("\tRF CLOCK");
      Serial.print(" CHNL: ");
      Serial.print(oregon.sens_chnl);
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print("ID: ");
      Serial.print(oregon.sens_id, HEX);
      Serial.print(" TIME: ");
      Serial.print(oregon.packet[6] & 0x0F, HEX);
      Serial.print(oregon.packet[6] & 0xF0 >> 4, HEX);
      Serial.print(':');
      Serial.print(oregon.packet[5] & 0x0F, HEX);
      Serial.print(oregon.packet[5] & 0xF0 >> 4, HEX);
      Serial.print(':');
      Serial.print(':');
      Serial.print(oregon.packet[4] & 0x0F, HEX);
      Serial.print(oregon.packet[4] & 0xF0 >> 4, HEX);
      Serial.print(" DATE: ");
      Serial.print(oregon.packet[7] & 0x0F, HEX);
      Serial.print(oregon.packet[7] & 0xF0 >> 4, HEX);
      Serial.print('.');
      if (oregon.packet[8] & 0x0F ==1 || oregon.packet[8] & 0x0F ==3)   Serial.print('1');
      else Serial.print('0');
      Serial.print(oregon.packet[8] & 0xF0 >> 4, HEX);
      Serial.print('.');
      Serial.print(oregon.packet[9] & 0x0F, HEX);
      Serial.print(oregon.packet[9] & 0xF0 >> 4, HEX);
      
    }    

    if (oregon.sens_type == PCR800 && oregon.crc_c){
      Serial.print("\tPCR800  ");
      Serial.print("        ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print(" ID: ");
      Serial.print(oregon.sens_id, HEX);
      Serial.print("   TOTAL: ");
      Serial.print(oregon.get_total_rain(), 1);
      Serial.print("mm  RATE: ");
      Serial.print(oregon.get_rain_rate(), 1);
      Serial.print("mm/h");
      
    }    
    
#if ADD_SENS_SUPPORT == 1
      if ((oregon.sens_type & 0xFF00) == THP && oregon.crc_c) {
      Serial.print("\tTHP     ");
      Serial.print(" CHNL: ");
      Serial.print(oregon.sens_chnl);
      Serial.print(" BAT: ");
      Serial.print(oregon.sens_voltage, 2);
      Serial.print("V");
      if (oregon.sens_tmp > 0 && oregon.sens_tmp < 10) Serial.print(" TMP:  ");
      if (oregon.sens_tmp < 0 && oregon.sens_tmp > -10) Serial.print(" TMP: ");
      if (oregon.sens_tmp <= -10) Serial.print(" TMP:");
      if (oregon.sens_tmp >= 10) Serial.print(" TMP: ");
      Serial.print(oregon.sens_tmp, 1);
      Serial.print("C ");
      Serial.print("HUM: ");
      Serial.print(oregon.sens_hmdty, 1);
      Serial.print("% ");
      Serial.print("PRESS: ");
      Serial.print(oregon.sens_pressure, 1);
      Serial.print("Hgmm");
      yield();

      if (oregon.sens_chnl > NOF_THP - 1) {Serial.println(); return;}
      
      byte _chnl = oregon.sens_chnl  + NOF_132 + NOF_500 + NOF_968 + NOF_129 + NOF_800 + NOF_318;
      t_sensor[ _chnl].chnl = oregon.sens_chnl + 1;
      t_sensor[ _chnl].number_of_receiving++;
      t_sensor[ _chnl].type = oregon.sens_type;
      t_sensor[ _chnl].pressure = t_sensor[ _chnl].pressure * ((float)(t_sensor[ _chnl].number_of_receiving - 1) / (float)t_sensor[ _chnl].number_of_receiving) + oregon.sens_pressure / t_sensor[ _chnl].number_of_receiving;
      t_sensor[ _chnl].temperature = t_sensor[ _chnl].temperature * ((float)(t_sensor[ _chnl].number_of_receiving - 1) / (float)t_sensor[ _chnl].number_of_receiving) + oregon.sens_tmp / t_sensor[ _chnl].number_of_receiving;
      t_sensor[ _chnl].humidity = t_sensor[ _chnl].humidity * ((float)(t_sensor[ _chnl].number_of_receiving - 1) / (float)t_sensor[ _chnl].number_of_receiving) + oregon.sens_hmdty / t_sensor[ _chnl].number_of_receiving;
      t_sensor[ _chnl].voltage = t_sensor[ _chnl].voltage * ((float)(t_sensor[ _chnl].number_of_receiving - 1) / (float)t_sensor[ _chnl].number_of_receiving) + oregon.sens_voltage / t_sensor[ _chnl].number_of_receiving;
      t_sensor[ _chnl].rcv_time = millis();         
    }
#endif
    Serial.println();
  }
  yield();
/*/////////////////////////////////////////////////////////////////////
// ВебСервер. обработка текущих входящих HTTP-запросов  *//////////////
  server.handleClient(); 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//***************************************************************************************************************************************
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void wifi_connect() {
  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);
  unsigned long cur_mark = millis();
  bool blink = 0;
  //WiFi.config(ip, gateway, subnet); 
  WiFi.mode(WIFI_STA); // Задаем режим работы WIFI_STA (клиент)
  WiFi.begin(ssid, password);
  do {
      while (WiFi.status() != WL_CONNECTED) {
      if (blink) {
        digitalWrite(BLUE_LED, LOW);
      }
      else {
        digitalWrite(BLUE_LED, HIGH);
      }
      blink = !blink;
      delay(500);
      Serial.print(".");
      //Подключаемся слишком долго. Переподключаемся....
      if ((millis() - cur_mark) > CONNECT_TIMEOUT){
        blink = 0; 
        digitalWrite(BLUE_LED, HIGH);
        WiFi.disconnect();
        delay(3000);
        cur_mark = millis();
        WiFi.begin(ssid, password);
      }
    }
  } while (WiFi.status() != WL_CONNECTED);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool test_narodmon_connection() {
  if (TEST_MODE) return true;
  if (wclient.connect(nardomon_server, port)) {  
    wclient.println("##");
    cur_mark = millis();
    do {
      wait_timer(10);
      if ((millis() - cur_mark) > CONNECT_TIMEOUT) {
        Serial.println("narodmon.ru is not responding");
        wclient.stop();
        return 0;
      }
    } while (!wclient.connected());
    Serial.println("narodmon.ru is attainable");
    wclient.stop();
    return 1;
  } 
  else {
    Serial.println("connection to narodmon.ru failed");
    wclient.stop();
    return 0;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Отсылка данных на narodmon
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool send_data() {

  wind_sensor.dir_cycle++;   //Накапливаем циклы для расчёта направления ветра
  //Если соединения с сервером нет, по переподключаемся
  if (WiFi.status() != WL_CONNECTED) wifi_connect();

  bool what_return = false;
  bool is_connect = true;
  if (!TEST_MODE) is_connect = wclient.connect(nardomon_server, port);
  
  if (is_connect) {  
    //Отправляем MAC-адрес
    Serial.println(' ');
    String s = "#";
           s += mac;
    Serial.println(s);
    if (!TEST_MODE) wclient.println(s);
    //Отправляем данные Oregon
    sendOregonData(2);
    // Отправляем mqtt
        
    //Завершаем передачу
    if (!TEST_MODE) wclient.println("##");
    Serial.println("##");
    //Ждём отключения клиента
    cur_mark = millis();
    if (!TEST_MODE)
    {
      do 
      {
        yield();
        if (millis() > cur_mark + DISCONNECT_TIMEOUT) break;
      }
      while (!wclient.connected());
    } 
     
    Serial.println(' ');
    if (!TEST_MODE) wclient.stop();
    what_return = true;
  } 
  else {
    Serial.println("connection to narodmon.ru failed");
    if (!TEST_MODE) wclient.stop();
  }
  lastConnectionTime = millis();
  
///////////////////////////////////////////////////////////////////////////
  //Обнуляем флаги полученных данных
  for (int i = 0; i < N_OF_THP_SENSORS; i++) 
    t_sensor[i].number_of_receiving = 0;
////////////////////////////////////////////////////////////////////////////

  if (wind_sensor.dir_cycle >= NO_WINDDIR)
  {
    wind_sensor.number_of_dir_receiving = 0;  
    wind_sensor.dir_cycle = 0;
  }
    
  rain_sensor.number_of_receiving = 0;
  wind_sensor.number_of_receiving = 0;
  uv_sensor.number_of_receiving = 0;
  wind_sensor.number_of_receiving = 0; 
  
  
  return what_return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
String sendOregonData(int flag) 
{
  String s = "", pref;
  String  web1 ="";
  String chanel ="";
  String chanel1 ="";
  
  
  for (byte i = 0; i < N_OF_THP_SENSORS; i++)
  {
    if (t_sensor[i].number_of_receiving > 0) 
    {
      if (t_sensor[i].type == BTHGN129) pref = "20";
      if (t_sensor[i].type == THGN132 ||t_sensor[i].type == THN132) pref = "30";
      if ((oregon.sens_type & 0x0FFF) == RTGN318 || (oregon.sens_type & 0x0FFF) == RTHN318) pref = "40";
      if (t_sensor[i].type == THGN500) pref = "50";
      if ((t_sensor[i].type & 0xFF00) == THP) pref = "70";
      if (t_sensor[i].type == THGR810 ||t_sensor[i].type == THN800) pref = "80";
      if (t_sensor[i].type == BTHR968) pref = "90";
      
      s += "#TEMPC";
      s += pref;
      s += t_sensor[i].chnl;
      s += "#";
      s += t_sensor[i].temperature;
      s += "\n";

      if(t_sensor[i].chnl == 1) chanel = " в спальне";
      if(t_sensor[i].chnl == 2) chanel = " на улице";

      web1 +="<div class='data temperature'>";
      web1 +="<div class='side-by-side icon'>";
      web1 +="<svg enable-background='new 0 0 19.438 54.003'height=54.003px id=Layer_1 version=1.1 viewBox='0 0 19.438 54.003'width=19.438px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M11.976,8.82v-2h4.084V6.063C16.06,2.715,13.345,0,9.996,0H9.313C5.965,0,3.252,2.715,3.252,6.063v30.982";
      web1 +="C1.261,38.825,0,41.403,0,44.286c0,5.367,4.351,9.718,9.719,9.718c5.368,0,9.719-4.351,9.719-9.718";
      web1 +="c0-2.943-1.312-5.574-3.378-7.355V18.436h-3.914v-2h3.914v-2.808h-4.084v-2h4.084V8.82H11.976z M15.302,44.833";
      web1 +="c0,3.083-2.5,5.583-5.583,5.583s-5.583-2.5-5.583-5.583c0-2.279,1.368-4.236,3.326-5.104V24.257C7.462,23.01,8.472,22,9.719,22";
      web1 +="s2.257,1.01,2.257,2.257V39.73C13.934,40.597,15.302,42.554,15.302,44.833z'fill=#F29C21 /></g></svg>";
      web1 +="</div>";
      web1 +="<div class='side-by-side text'>Температура";
      web1 +=chanel;
      web1 +="</div>";
      web1 +="<div class='side-by-side reading'>";
      web1 +=t_sensor[i].temperature;
      web1 +="<span class='superscript'>&deg;C</span></div>";
      web1 +="</div>";

      if (t_sensor[i].humidity > 0 && t_sensor[i].humidity <= 100  &&
      (t_sensor[i].type == THGN132 || 
      t_sensor[i].type == THGN500 ||
      t_sensor[i].type == THGR810 ||
      (t_sensor[i].type & 0x0FFF) == RTGN318) ||
      (t_sensor[i].type == BTHGN129  ||
      #if ADD_SENS_SUPPORT == 1
      (t_sensor[i].type & 0xFF00) == THP  ||
      #endif
      t_sensor[i].type == BTHR968))
      {
        s += "#HUMMRH";
        s += pref;
        s += t_sensor[i].chnl;
        s += "#";
        s += t_sensor[i].humidity;
        s += "\n";

            if(t_sensor[i].chnl == 1) { chanel = " спальне"; }
            if(t_sensor[i].chnl == 2) {chanel = " на улице"; }

        web1 +="<div class='data humidity'>";
        web1 +="<div class='side-by-side icon'>";
        web1 +="<svg enable-background='new 0 0 29.235 40.64'height=40.64px id=Layer_1 version=1.1 viewBox='0 0 29.235 40.64'width=29.235px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><path d='M14.618,0C14.618,0,0,17.95,0,26.022C0,34.096,6.544,40.64,14.618,40.64s14.617-6.544,14.617-14.617";
        web1 +="C29.235,17.95,14.618,0,14.618,0z M13.667,37.135c-5.604,0-10.162-4.56-10.162-10.162c0-0.787,0.638-1.426,1.426-1.426";
        web1 +="c0.787,0,1.425,0.639,1.425,1.426c0,4.031,3.28,7.312,7.311,7.312c0.787,0,1.425,0.638,1.425,1.425";
        web1 +="C15.093,36.497,14.455,37.135,13.667,37.135z'fill=#3C97D3 /></svg>";
        web1 +="</div>";
        web1 +="<div class='side-by-side text'>Влажность";
        web1 +=chanel;
        web1 +="</div>";
        web1 +="<div class='side-by-side reading'>";
        web1 +=t_sensor[i].humidity;
        web1 +="<span class='superscript'>%</span></div>";
        web1 +="</div>";

      }

      if ((t_sensor[i].type == BTHGN129  || 
      #if ADD_SENS_SUPPORT == 1
      (t_sensor[i].type & 0xFF00) == THP  ||
      #endif
      t_sensor[i].type == BTHR968))
      {
        s += "#P";
        s += pref;
        s += t_sensor[i].chnl;
        s += "#";
        s += t_sensor[i].pressure;
        s += "\n";
      }
      #if ADD_SENS_SUPPORT == 1
      if ((t_sensor[i].type & 0xFF00) == THP)
      {
        s += "#V";
        s += pref;
        s += t_sensor[i].chnl;
        s += "#";
        s += t_sensor[i].voltage;
        s += "\n";
      }
    #endif
    }
  }
  //Отправляем данные WGR800
  if (wind_sensor.number_of_receiving > 0)
  {
    s += "#WSMID#";
    s += wind_sensor.midspeed;
    s += '\n';
    s += "#WSMAX#";
    s += wind_sensor.maxspeed;
    s += '\n';    
  } 
  
  if (wind_sensor.number_of_dir_receiving > 0 && wind_sensor.dir_cycle >= NO_WINDDIR) 
  {
    s += "#DIR#";
    s += calc_wind_direction(&wind_sensor);
    s += '\n';      
  }
    
    
  //Отправляем данные PCR800
  if (rain_sensor.isreceived > 0)
  {
    s += "#RAIN#";
    s += rain_sensor.counter;
    s += '\n';
  }
    
  //Отправляем данные UVN800
  if (uv_sensor.isreceived > 0)
  {
    s += "#UV#";
    s += uv_sensor.index;
    s += '\n';
  }

  // Данные с BMP180
  //показания температуры
  s += "#TEMPC#";
  s += bmp.readTemperature();
  s += "\n";
  //показания давления
  s += "#PRESS#";
  s += bmp.readPressure();
  s += "#Датчик давления BMP180\n";
  // уровень WIFI сигнала
  int WIFIRSSI=constrain(((WiFi.RSSI()+100)*2),0,100);
  s += "#WIFI#";
  s += WIFIRSSI;
  s += "#Уровень WI-FI ";
  s += WiFi.SSID();
  s += "\n"; 

  if (flag != 1)Serial.print(s);
//  if (flag == 1)Serial.print(web1);
//  if (flag == 1)Serial.println(F("Дёрнул страницу!"));
  if (!TEST_MODE & flag ==2) wclient.print(s);
  return web1;
}

/////////////////////////////
// Отсылка данных по MQTT
/////////////////////////////
void mqtt_send() {

  if (client.connect(MQTT::Connect( mac )
       .set_auth(AIO_USERNAME, AIO_KEY))) {
//  client.publish("outTopic","hello world");

  int WIFIRSSI=constrain(((WiFi.RSSI()+100)*2),0,100);

          Serial.print(F("Sending childtemp "));
          Serial.print(bmp.readTemperature());
        if (! client.publish( AIO_TOPIC "/sensor/child/bmp180/temp", String(bmp.readTemperature()))) {
          Serial.println(F(" Failed"));
          } else {
            Serial.println(F(" OK!"));
          }
          
          Serial.print(F("Sending childpress "));
          Serial.print(bmp.readPressure());
        if (! client.publish( AIO_TOPIC "/sensor/child/bmp180/press", String(bmp.readPressure()/133.3-0.1))) {
          Serial.println(F(" Failed"));
          } else {
            Serial.println(F(" OK!"));
          }
          
          Serial.print(F("Sending childwifi "));
          Serial.print(WIFIRSSI);
        if (! client.publish( AIO_TOPIC "/sensor/child/bmp180/wifi", String(WIFIRSSI))) {
          Serial.println(F(" Failed"));
          } else {
            Serial.println(F(" OK!"));
          }
  
  for (byte i = 0; i < N_OF_THP_SENSORS; i++)
  {
    if (t_sensor[i].number_of_receiving > 0) 
    {
      // MQTT
      if( t_sensor[i].chnl == 1 ) {
          Serial.print(F("Sending bedroomtemp "));
          Serial.print(t_sensor[i].temperature);
        if (! client.publish( AIO_TOPIC "/sensor/bedroom/oregon/temp", String(t_sensor[i].temperature))) {
          Serial.println(F(" Failed"));
          } else {
            Serial.println(F(" OK!"));
          }
      }
      
      if( t_sensor[i].chnl == 2 ) {
          Serial.print(F("Sending outdortemp "));
          Serial.print(t_sensor[i].temperature);
        if (! client.publish( AIO_TOPIC "/sensor/outdor/oregon/temp", String(t_sensor[i].temperature))) {
          Serial.println(F(" Failed"));
          } else {
            Serial.println(F(" OK!"));
          }
      }
    
      if (t_sensor[i].humidity > 0 && t_sensor[i].humidity <= 100  &&
      (t_sensor[i].type == THGN132 || 
      t_sensor[i].type == THGN500 ||
      t_sensor[i].type == THGR810 ||
      (t_sensor[i].type & 0x0FFF) == RTGN318) ||
      (t_sensor[i].type == BTHGN129  ||
      #if ADD_SENS_SUPPORT == 1
      (t_sensor[i].type & 0xFF00) == THP  ||
      #endif
      t_sensor[i].type == BTHR968))
      {
        // MQTT
        if( t_sensor[i].chnl == 1 ) {
          Serial.print(F("Sending bedroomhumm "));
          Serial.print(t_sensor[i].humidity);
          if (! client.publish( AIO_TOPIC "/sensor/bedroom/oregon/humm", String(t_sensor[i].humidity))) {
            Serial.println(F(" Failed"));
            } else {
              Serial.println(F(" OK!"));
            }
        }
        if( t_sensor[i].chnl == 2 ) {
          Serial.print(F("Sending outdorhumm "));
          Serial.print(t_sensor[i].humidity);
          if (! client.publish( AIO_TOPIC "/sensor/outdor/oregon/humm", String(t_sensor[i].humidity))) {
            Serial.println(F(" Failed"));
            } else {
              Serial.println(F(" OK!"));
            }
        }
      }
    }
  }
 }

client.disconnect();
}


////////////////////////////////////////////////////////////////////////////////////////
// Рассчёт направления ветра
////////////////////////////////////////////////////////////////////////////////////////
float calc_wind_direction(WGR800_sensor* wdata)
{
 if (wdata->direction_x == 0) wdata->direction_x = 0.01;
 float otn = abs(wdata->direction_y / wdata->direction_x);
 float angle = (asin(otn / sqrt(1 + otn * otn))) * 180 / 3.14;
  
 //Определяем направление
 if (wdata->direction_x > 0 && wdata->direction_y < 0) otn = angle; 
 if (wdata->direction_x < 0 && wdata->direction_y < 0) otn = 180 - angle;
 if (wdata->direction_x < 0 && wdata->direction_y >= 0) otn = 180 + angle;
 if (wdata->direction_x > 0 && wdata->direction_y >= 0) otn = 360 - angle;
  
 angle = otn + WIND_CORRECTION; // Если маркер флюгера направлен не на север
 if (angle >= 360) angle -= 360;
 
 return angle;
}

////////////////////////////////////////////////////////////////////////////////////////
// ЗАМЕНА DELAY, которая работает и не приводит к вылету...
////////////////////////////////////////////////////////////////////////////////////////
void wait_timer(int del){
  unsigned long tm_marker = millis();
  while (millis() - tm_marker < del) yield();
  return;
}

////////////////////////////////////////////////////////////////////////////////////////
// ВебСервер. Кнопка перезагрузки.
////////////////////////////////////////////////////////////////////////////////////////
void handle_reboot() {
    Serial.println("Reset..");
    ESP.restart();
  server.send(200, "text/html", "Rebooting"); 
}
