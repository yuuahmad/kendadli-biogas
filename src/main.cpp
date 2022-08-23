#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>

// sensor bmp280
// #include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)
// #define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP280 bmp; // I2C
// sensor tds
#include "GravityTDS.h"
// current sensor
#include "ACS712.h"
ACS712 ACS(A3, 5.0, 1023, 66); // konek sensor acs ke a3
// rtc
#include "RTClib.h"
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"min", "sen", "sel", "rab", "kam", "jum", "sab"};
// sensor ph
int samples = 10;
// sensor tds
GravityTDS gravityTds;
float temperature = 25, tdsValue = 0;
// lcd i2c
LiquidCrystal_I2C lcd(0x27, 20, 4);
// sensor voltage
float vOUT = 0.0;
float vIN = 0.0;
int analogVolt = 0;

// data untuk motor
int kecepatanMotor, keadaanMotor;

// untuk motor driver
const int enKanan = 7;
const int enKiri = 8;
const int pwmKanan = 6;
const int pwmKiri = 9;

// TERIMA DADTA
bool parsing = false;
String sData, data[10];

void setup()
{
  Serial.begin(115200);

  // sensor bmp280
  unsigned status;
  // status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status)
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // buat pinmode dan buat jadi aktif
  // ketika 2 en motor high, artinya motor pada keadaan aktif (standby)
  // ketika 2 en motor low, artinya motor nonaktif (mati total)
  pinMode(enKanan, OUTPUT);
  pinMode(enKiri, OUTPUT);
  pinMode(pwmKanan, OUTPUT);
  pinMode(pwmKiri, OUTPUT);
  digitalWrite(enKanan, LOW);
  digitalWrite(enKiri, LOW);
  analogWrite(pwmKanan, 0);
  analogWrite(pwmKiri, 0);

  // i2c lcd
  lcd.init(); // initialize the lcd
  lcd.backlight();

  // untuk sensor arus
  ACS.autoMidPoint();

  // untuk rtc
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1)
      delay(10);
  }
  if (rtc.lostPower())
  {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // sensor tds
  gravityTds.setPin(A0);        // konek tds ke a0
  gravityTds.setAref(5.0);      // reference voltage on ADC
  gravityTds.setAdcRange(1024); // 1024 => 10bit;4096 => 12bit ADC
  gravityTds.begin();           // initialization

  // tampilkan data
  lcd.setCursor(0, 0);
  lcd.print("starting       ");
  lcd.setCursor(0, 1);
  lcd.print("smart biogas...");
  delay(2000);
  lcd.setCursor(0, 0);
  lcd.print("PKM-KC         ");
  lcd.setCursor(0, 1);
  lcd.print("ITERA JAYA     ");
  delay(5000);
}

float ph(float voltage)
{
  return 7 + ((2.5 - voltage) / 0.18);
}

void loop()
{
  // terima data dari blynk
  while (Serial.available())
  {
    char inChar = Serial.read();
    sData += inChar;
    if (inChar == '$')
      parsing = true;
    if (parsing)
    {
      int q = 0;
      for (int i = 0; i < sData.length(); i++)
      {
        if (sData[i] == '#')
        {
          q++;
          data[q] = "";
        }
        else
          data[q] += sData[i];
      }
      kecepatanMotor = data[1].toInt();
      keadaanMotor = data[2].toInt();
      parsing = false;
      sData = "";
    }
  }
  // untuk sensor arus
  int mA = ACS.mA_DC();
  // untk rtc
  DateTime now = rtc.now();
  // untuk sensor ph
  int measurings = 0;
  for (int i = 0; i < samples; i++)
  {
    measurings += analogRead(A1);
    delay(10);
  }
  float voltageph = 5 / 1024.0 * measurings / samples;
  voltageph = voltageph - 0.02;
  // sensor tds
  gravityTds.setTemperature(temperature); // temperature compensation
  gravityTds.update();                    // sample and calculate
  tdsValue = gravityTds.getTdsValue();    // dapatkan nilai dalam ppm

  // voltage sensor
  analogVolt = analogRead(A2); // konek voltage sensor ke a2
  vOUT = (analogVolt * 5.0) / 1024.0;
  vIN = vOUT / 0.2;

  // perintah mengontrol motor
  if (keadaanMotor == 1)
  {
    digitalWrite(enKanan, HIGH);
    digitalWrite(enKiri, HIGH);
    analogWrite(pwmKanan, kecepatanMotor);
    analogWrite(pwmKiri, 0);
  }
  else
  {
    digitalWrite(enKanan, LOW);
    digitalWrite(enKiri, LOW);
    analogWrite(pwmKanan, 0);
    analogWrite(pwmKiri, 0);
  }

  // tampilkan semua data ke lcd
  lcd.setCursor(0, 0);
  lcd.print(now.year());
  lcd.print('/');
  lcd.print(now.month());
  lcd.print('/');
  lcd.print(now.day());
  lcd.print(' ');
  lcd.print(now.hour());
  lcd.print(':');
  lcd.print(now.minute());
  lcd.print(':');
  lcd.print(now.second());
  lcd.print("  ");

  lcd.setCursor(0, 1);
  lcd.print("pH:");
  lcd.print(ph(voltageph));
  lcd.print("  ");
  lcd.setCursor(9, 1);
  lcd.print("TDS:");
  lcd.print(tdsValue);
  lcd.print("  ");

  lcd.setCursor(0, 2);
  lcd.print("bV:");
  lcd.print(vIN);
  lcd.print("  ");
  lcd.setCursor(9, 2);
  lcd.print("APV:");
  lcd.print(mA);
  lcd.print("  ");

  lcd.setCursor(0, 3);
  lcd.print("km:");
  lcd.print(keadaanMotor);
  lcd.print("/");
  lcd.print(kecepatanMotor);
  lcd.print("  ");
  lcd.setCursor(9, 3);
  lcd.print(bmp.readPressure() / 1000, 1); // default pa, ganti kpa
  lcd.print("/");
  lcd.print(bmp.readTemperature(), 1);
  lcd.print(" ");

  // kirim semua data ke blynk
  Serial.print("#");
  Serial.print(ph(voltageph));
  Serial.print("#");
  Serial.print(tdsValue);
  Serial.print("#");
  Serial.print(vIN);
  Serial.print("#");
  Serial.print(mA);
  Serial.print("#");
  Serial.print(bmp.readPressure());
  Serial.print("#");
  Serial.print(bmp.readTemperature());
  Serial.println("#$");
}