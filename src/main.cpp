#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>

// sensor tds
#include "GravityTDS.h"
// current sensor
#include "ACS712.h"
ACS712 ACS(A3, 5.0, 1023, 66); // konek sensor acs ke a3
// rtc
#include "RTClib.h"
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"minggu", "senin", "selasa", "rabu", "kamis", "jumat", "sabtu"};
// sensor ph
int samples = 10;
// sensor tds
GravityTDS gravityTds;
float temperature = 25, tdsValue = 0;
// lcd i2c
LiquidCrystal_I2C lcd(0x27, 16, 2);
// sensor voltage
float vOUT = 0.0;
float vIN = 0.0;
int analogVolt = 0;

// untuk motor driver
const int enKanan = 7;
const int enKiri = 8;
const int pwmKanan = 6;
const int pwmKiri = 9;

void setup()
{
  Serial.begin(115200);

  // buat pinmode dan buat jadi aktif
  // ketika 2 en motor high, artinya motor pada keadaan aktif (standby)
  // ketika 2 en motor low, artinya motor nonaktif (mati total)
  pinMode(enKanan, OUTPUT);
  pinMode(enKiri, OUTPUT);
  pinMode(pwmKanan, OUTPUT);
  pinMode(pwmKiri, OUTPUT);
  digitalWrite(enKanan, HIGH);
  digitalWrite(enKiri, HIGH);
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
  gravityTds.setAref(5.0);      // reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024); // 1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();           // initialization

  // beri delay 1 detik
  // Serial.println("mulai komputasi");
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
  int mA = ACS.mA_DC();
  // untk rtc
  DateTime now = rtc.now();
  // Serial.print(now.year());
  // Serial.print('/');
  // Serial.print(now.month());
  // Serial.print('/');
  // Serial.print(now.day());
  // Serial.print(" (");
  // Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  // Serial.print(") ");
  // Serial.print(now.hour());
  // Serial.print(':');
  // Serial.println(now.minute());
  // lcd
  lcd.setCursor(0, 0);
  lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);
  lcd.print('/');
  lcd.print(now.year());
  lcd.print('/');
  lcd.print(now.month());
  lcd.print('/');
  lcd.print(now.day());
  lcd.print("               ");

  lcd.setCursor(0, 1);
  // lcd.print(' ');
  lcd.print(now.hour());
  lcd.print('/');
  lcd.print(now.minute());
  lcd.print('/');
  lcd.print(now.second());
  lcd.print("               ");

  // untuk sensor ph
  int measurings = 0;
  for (int i = 0; i < samples; i++)
  {
    measurings += analogRead(A1);
    delay(10);
  }
  float voltageph = 5 / 1024.0 * measurings / samples;
  voltageph = voltageph - 0.02;
  // Serial.print("pH= ");
  // Serial.print(ph(voltageph));

  // sensor tds
  gravityTds.setTemperature(temperature); // set the temperature and execute temperature compensation
  gravityTds.update();                    // sample and calculate
  tdsValue = gravityTds.getTdsValue();    // then get the value
  // Serial.print(" ppm= ");
  // Serial.println(tdsValue, 0);

  // voltage sensor
  analogVolt = analogRead(A2); // konek voltage sensor ke a2
  vOUT = (analogVolt * 5.0) / 1024.0;
  vIN = vOUT / 0.2;
  // Serial.print("voltage = ");
  // Serial.println(vIN);

  // sensor arus
  // Serial.print("arus (mA) = ");
  // Serial.println(mA);
  // Serial.print("arus (A) = ");
  // Serial.println(mA / 1000);
  // delay(1000);

  // // ini adalah perintah sederhana mengontrol motor
  //   for (size_t i = 0; i < 225; i++)
  //   {
  //     analogWrite(pwmKanan, i);
  //     analogWrite(pwmKiri, 0);
  //     delay(50);
  //   }
  //   analogWrite(pwmKanan, 0);
  //   analogWrite(pwmKiri, 0);
  //   delay(3000);

  // kirim dan terima data dari blynk
  Serial.print("#");
  Serial.print(ph(voltageph));
  Serial.print("#");
  Serial.print(tdsValue);
  Serial.print("#");
  Serial.print(vIN);
  Serial.print("#");
  Serial.print(mA);
  Serial.println("#$");
}