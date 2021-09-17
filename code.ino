//Project created by Yeskov Yegor

/*
 Источники:
 https://alexgyver.ru/meteoclock/
 https://www.2150692.ru/faq/87-co2-mhz19-arduino
 https://www.youtube.com/watch?v=obB6lKffV90&ab_channel=ArduinoTV
 https://habr.com/ru/post/391157/
 */

/*
 ПОДКЛЮЧЕНИЯ

 BME:
 vcc - power
 gnd - power
 scl - A5
 sda - A4

 Display:
 gnd - power
 vcc - power
 sda - A4
 scl - A5

 MH-Z19:
 vcc - power
 gnd - power
 rx - A1
 tx - A0

 RGB light:
 gnd - power
 r - D11
 g - D12
 b - D13
 */

//Библеотеки
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>;

//defines Для светодиода 
#define RED 11 //присваиваем имя RED для пина 11
#define GRN 12 //присваиваем имя GRN для пина 12
#define BLU 13 //присваиваем имя BLU для пина 13


//Софт для датчика MH-Z19
SoftwareSerial mySerial(A0, A1); // A0 - к TX сенсора, A1 - к RX

byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
unsigned char response[9];

//I2C start 
Adafruit_BME280 bme; // I2C

//Переменные

//Для монитора
unsigned long delayTime = 1000; // Частота вывода в сериал монитор



//Параметры дисплея
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
    //устанавливаем параметры для rgb светодиода
    pinMode(RED, OUTPUT);
    pinMode(GRN, OUTPUT);
    pinMode(BLU, OUTPUT);
    
    //9600 бод
    Serial.begin(9600);

    //Serial для MH-Z19
    mySerial.begin(9600);

    //BME280 проверка
    Serial.println(F("BME280 test"));

    bool status;
    
    // Здесь вводим адрес устройства
    status = bme.begin(0x76);  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    bme.setSampling();
    Serial.println("-- Default Test --");
    Serial.println();


    //Инициализация дисплея
    lcd.init();                      
    lcd.init();
    lcd.backlight();
}


void loop()
{
  
  //Переменная для подсчёта концентрации CO2
  unsigned int ppm;

  //Подсчёт самого значения в ppm
  mySerial.write(cmd, 9);
  memset(response, 0, 9);
  mySerial.readBytes(response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc+=response[i];
  crc = 255 - crc;
  crc++;

  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error: " + String(crc) + " / "+ String(response[8]));
  } else {
    unsigned int responseHigh = (unsigned int) response[2];
    unsigned int responseLow = (unsigned int) response[3];
    ppm = (256*responseHigh) + responseLow;
    //Значений на мониторе порта не будет, так как ардуино не подключена к компьютеру
    Serial.println(ppm);
  }
  delay(10000);
  
  //Переменные, где храняться показания с датчиков

  //Для BME280
  float temp = bme.readTemperature();  // get temperature
  float humidity    = bme.readHumidity();     // get humidity
  float pressure    = bme.readPressure();     // get pressure

  //Вывод их на монитор порта (в нашем случае их не будет, потому что ардуино питается от power банка)
  Serial.print(temp);
  Serial.print(humidity);
  Serial.print(pressure);
  
  delay(10000);

  //если уровень СО2 400 - 800 ppm
  if (ppm > 0 && ppm < 800) {
    digitalWrite(RED, LOW);
    digitalWrite(GRN, HIGH);
    digitalWrite(BLU, LOW);
  } 
  //есле уровень СО2 800 - 1200 ppm
  else if (ppm > 800 && ppm < 1200) {
    digitalWrite(RED, HIGH);
    digitalWrite(GRN, HIGH);
    digitalWrite(BLU, LOW);
  } 
  //если уровень СО2 больше 1200  ppm
  else if (ppm > 1200) {
    digitalWrite(RED, HIGH);
    digitalWrite(GRN, LOW);
    digitalWrite(BLU, LOW);
  }

 

  //Непосредственно, вывод на дисплей самих значений с датчиков

  //Вывод температуры
  lcd.setCursor(0, 0);
  lcd.print("Temperature: ");
  lcd.setCursor(13, 0);
  lcd.print(bme.readTemperature());
  lcd.setCursor(19, 0);
  lcd.print("C");

  //Вывод влажности
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.setCursor(10, 1);
  lcd.print(bme.readHumidity());
  lcd.setCursor(16, 1);
  lcd.print("%");

  //Вывод атмосферного давления
  lcd.setCursor(0, 2);
  lcd.print("Pressure: ");
  lcd.setCursor(10, 2);
  lcd.print(round(bme.readPressure()/ 133.3F));
  lcd.setCursor(14, 2);
  lcd.print("mm.rt");
  
  //Вывод уровня углекислого газа
  lcd.setCursor(0, 3);
  lcd.print("CO2: ");
  lcd.setCursor(5, 3);
  lcd.print(ppm);
  lcd.setCursor(9, 3);
  lcd.print("ppm");
   
}
