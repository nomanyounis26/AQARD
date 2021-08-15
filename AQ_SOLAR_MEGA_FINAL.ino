//#define BLYNK_PRINT Serial
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>
#include <Wire.h>
#include "WiFiEsp.h"
#include "DHT.h"
#include "ThingSpeak.h"
#include <SDS011-select-serial.h>

char auth[] = "r4YLcXZ2tUnLBIWEDx9mdCl-ZFy3Jukc";
char ssid[] = "Zone-X";
char pass[] = "o3004404321";
unsigned long myChannelNumber = 1270079;
const char * myWriteAPIKey = "S8A84C7GW9NRMBYE";

#define EspSerial Serial1
ESP8266 wifi(&EspSerial);
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);
BlynkTimer timer;
Adafruit_INA219 ina219;
WiFiEspClient client;
SDS011 my_sds(Serial2);

#define SOL_VOL A0 // Solar panel side voltage divider
#define SOL_ADC A2 // ACS 712 for SOL_CUR
#define AIR_SEN A5 //mq135 air quality monitoring sensor
#define TMP A6 //NTC_10k for temp of box

#define DHTPIN 7
#define LDR 5
#define PIR 49
#define REED 11
#define SOL_LED 23
#define BAT_RED_LED 25
#define BAT_GREEN_LED 29
#define BAT_BLUE_LED 27
#define RELAY_LED 30
#define BUZZER 31
#define ARD_LED 13
#define PWR_MSFT 4
#define MSFT1 46
#define MSFT2 44
#define RLY1 39
#define RLY2 37
#define MOTOR 45

#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);

float vf = 5.05; //Arduino Mega 5v voltage
float scale = vf/1024.0;
const int AVG_NUM = 10;    
#define BAT_MIN 6.3  
#define BAT_MAX 8.0  

byte solar[8] = {0b11111,0b10101,0b11111,0b10101,0b11111,0b10101,0b11111,0b00000};
byte battery[8] = {0b01110,0b11011,0b10001,0b10001,0b10001,0b10001,0b10001,0b11111};
byte pm25_icon[8] = {0b10111,0b10101,0b11101,0b00000,0b00001,0b11100,0b10101,0b10111};
byte pm10_icon[8] = {0b11111,0b11111,0b10001,0b11111,0b11111,0b00000,0b11111,0b11111};
byte charge[8] = {0b01010,0b11111,0b10001,0b10001,0b10001,0b01110,0b00100,0b00100,};
byte not_charge[8]= {0b00000,0b10001,0b01010,0b00100,0b01010,0b10001,0b00000,0b00000,};
byte wifi_icon[8] = {B00000, B11111, B11111, B01110, B01110, B00100, B00100, B00000};

int aq = 0; 
int moisture = 0; //stores percentage of soil moist.
bool PIRstate = 0;
bool LDRstate = 0;
bool Reedstate = 0;
float solar_volt=0.00;
float solar_current=0.00;
//int sol_pwr = 0;
float shuntvoltage = 0.00;
float busvoltage = 0.00;
float current_mA = 0.00;
float loadvoltage = 0.00;
//float power = 0.00;
float t=0.00, h=0.00, hic = 0.00;
int disconnects = 0;
bool light = 0;
bool charging = false;
bool charger = 0;
bool pump = 0;
bool online;
bool dark = 0;
float p10,p25;

unsigned long currentMillis = 0;    // stores the value of millis() in each iteration of loop()
unsigned long previousMillis = 0;   // for sensors
unsigned long relay_counter = 0;  //to count time lapsed since relay on
unsigned long ts_previous = 0;    //when last time uploaded to thinkspeak
unsigned long charge_timer = 0;    //to count time lapsed since charging
unsigned long connection_time = 0;    //to count time lapsed since charging
unsigned long pump_time = 0;    //for water pump

WidgetLED led1(V20); //solar led
WidgetLED led2(V21); //battery led
WidgetLED led3(V22); //relay led
WidgetLED led4(V23); //water pump

void(* resetFunc) (void) = 0;

void setup() { 
  pinMode(22, INPUT_PULLUP);
  //Serial.begin(9600);
  EspSerial.println("AT+RST");
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Loading Please Wait");
  lcd.setCursor(0,1);
  lcd.print("Connecting to ");
  lcd.print(ssid);
  
  if(digitalRead(22)==1) {
    online = true;
  } else {
    online = false;
  }
  if(online) {
    EspSerial.begin(9600);
    WiFi.init(&Serial1);
    WiFi.begin(ssid, pass);
    Blynk.begin(auth, wifi, ssid, pass);
    ThingSpeak.begin(client);  // Initialize ThingSpeak
      
    lcd.setCursor(0,2);
    lcd.print("CONNECTED");
    timer.setInterval(10000L, get_sensor_data);
   } else {
      lcd.setCursor(0,2);
      lcd.print("OFFLINE MODE");
   }
  Serial2.begin(9600);
  ina219.begin();
  dht.begin();
  
  //ina219.setCalibration_32V_1A();
  //ina219.setCalibration_16V_400mA();
  
  uint32_t currentFrequency;

  pinMode(AIR_SEN, INPUT);
  pinMode(SOL_ADC, INPUT);
  pinMode(SOL_VOL, INPUT);
  pinMode(REED, INPUT);
  pinMode(PIR,INPUT);
  pinMode(LDR,INPUT);
  pinMode(TMP,INPUT);
  pinMode(BUZZER,OUTPUT);
  pinMode(SOL_LED,OUTPUT);
  pinMode(BAT_GREEN_LED,OUTPUT);
  pinMode(BAT_BLUE_LED,OUTPUT);
  pinMode(BAT_RED_LED,OUTPUT);
  pinMode(RELAY_LED, OUTPUT);
  pinMode(ARD_LED, OUTPUT);
  pinMode(PWR_MSFT, OUTPUT);
  pinMode(RLY1,OUTPUT);
  pinMode(RLY2, OUTPUT);
  pinMode(MSFT1,OUTPUT);
  pinMode(MSFT2, OUTPUT);
  pinMode(A8, INPUT); //capacitative soil moisture sensor
  pinMode(MOTOR, OUTPUT);
 
  digitalWrite(RLY1, HIGH);
  digitalWrite(RLY2, HIGH);  
  digitalWrite(PWR_MSFT, LOW);
  digitalWrite(MSFT1, LOW);
  digitalWrite(MSFT2, LOW);
  digitalWrite(MOTOR, LOW);

  tone(BUZZER, 1000, 1000);   

  lcd.createChar(1, solar);
  lcd.createChar(2, battery);
  lcd.createChar(3, pm25_icon);
  lcd.createChar(4, pm10_icon);
  lcd.createChar(5, wifi_icon);
  lcd.createChar(6, charge);
  lcd.createChar(7, not_charge);
  lcd.clear();
  led1.off();
  led2.on();
  led3.off();
  led4.off();
  get_sensor_data();
}

void loop() {
  currentMillis = millis();
  motion();
  if(online) {   
    Blynk.run();
    timer.run();
  } else {
    get_sensor_data();
  }
}

void motion(void) {
  LDRstate = digitalRead(LDR);
  PIRstate = digitalRead(PIR);
  Reedstate = digitalRead(REED);

  if(!light) {
    if (LDRstate && PIRstate || LDRstate && Reedstate) {
      light = 1;
      lightfunc();
    }
  }
      
  if(light) {
    lightfunc();
  }
}

void get_sensor_data() {   
  int cur_temp = read_adc(SOL_ADC) - 12; 
  solar_current = (((cur_temp * scale)-(vf*0.5))/0.185)*1000; //mA
  solar_volt = read_adc(SOL_VOL)*scale*11; //V
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000); // V
  int soil = analogRead(A8); //Soil sensor at AnalogPin08(A8)
  moisture = map(soil, 239, 595, 100, 0); //wet=239, dry=595 
  if (moisture<0) { moisture = 50; }
            
  if(solar_current > 300) {
    charging = true;
    led1.on();
  } else {
    charging = false;
    led1.off();
    loadvoltage = loadvoltage + 0.11; //+0.21
    if(solar_volt < 1) {
      solar_volt = 0;
      solar_current = 0;
    }
  }
  
  if (loadvoltage < 1.2) loadvoltage = 0;
  if(current_mA < 2 ) { current_mA = 0; }
 
  aq = read_adc(AIR_SEN);
  h = dht.readHumidity();
  t = dht.readTemperature();
  hic = dht.computeHeatIndex(t, h, false);
        
  my_sds.read(&p25,&p10);

  led_indication();
  lcd_display();
        
  if ((currentMillis - ts_previous >= 3600000) && online) {
    thingspeak_update();
    ts_previous = currentMillis;
  }
  
  if(moisture <= 30 && pump == 0) {
    pump = 1;
    pump_time = currentMillis;
  }
  if(pump == 1) { check_pump(); }
  
  if(charger == 1) { charge_relay(); }
}

void led_indication(void) {
   digitalWrite(SOL_LED, charging);
   
   if(loadvoltage >= BAT_MAX) {   
      digitalWrite(BAT_GREEN_LED, HIGH);  
      digitalWrite(BAT_RED_LED, LOW);
      Blynk.setProperty(V21, "color", "#0C990C");
   } else if(loadvoltage <= BAT_MIN) {
      digitalWrite(BAT_GREEN_LED,LOW);  
      digitalWrite(BAT_RED_LED, HIGH);
      Blynk.setProperty(V21, "color", "#D3435C");
      charger = 1;
      charge_timer = currentMillis;
      tone(BUZZER, 1000, 1000);
   } else {
      Blynk.setProperty(V21, "color", "#92990C");
      digitalWrite(BAT_GREEN_LED,LOW);  
      digitalWrite(BAT_RED_LED, LOW);
   }
}

void lcd_display() {
  lcd.setCursor(0, 0);
  lcd.print("AQ    ");
  lcd.setCursor(3, 0);
  lcd.print(aq);
  lcd.setCursor(7, 0);
  lcd.write(3);
  lcd.print ("    ");
  lcd.setCursor(9, 0);
  lcd.print(p25, 0);
  lcd.setCursor(13, 0);
  lcd.write(4);
  lcd.print("    ");
  lcd.setCursor(15, 0);
  lcd.print(p10, 0);
  lcd.setCursor(0, 1);
  lcd.print("T     ");
  lcd.setCursor(2, 1);
  lcd.print(t, 1);
  lcd.setCursor(7, 1);
  lcd.print("HI     ");
  lcd.setCursor(10, 1);
  lcd.print(hic, 1);
  lcd.setCursor(15, 1);
  lcd.print("H    ");
  lcd.setCursor(17, 1);
  lcd.print(h, 0);
  lcd.print("%");
  lcd.setCursor(0, 2);
  lcd.write(1);
  lcd.setCursor(2, 2);
  lcd.print("     ");
  lcd.setCursor(2, 2);
  lcd.print(solar_volt, 1);
  lcd.print("V");
  
  if (solar_current > 1000) {
    Blynk.virtualWrite(V8, String((solar_current / 1000), 1) + String(" A") );
    lcd.setCursor(8, 2);
    lcd.print("     ");
    lcd.setCursor(8, 2);
    lcd.print((solar_current / 1000), 2);
    lcd.print("A");
  } else {
    Blynk.virtualWrite(V8, String(solar_current, 0) + String(" mA") );
    lcd.setCursor(8, 2);
    lcd.print("     ");
    lcd.setCursor(8, 2);
    lcd.print(solar_current, 0);
    lcd.print("mA");
  }   

  lcd.setCursor(0, 3);
  lcd.write(2);
  lcd.setCursor(2, 3);
  lcd.print(loadvoltage, 2);
  lcd.print("V");

  if (current_mA > 1000) {
    Blynk.virtualWrite(V6, String((current_mA / 1000), 1) + String(" A") );
    lcd.setCursor(8, 3);
    lcd.print("     ");
    lcd.setCursor(8, 3);
    lcd.print((current_mA / 1000), 2);
    lcd.print("A");
  } else {
    Blynk.virtualWrite(V6, String(current_mA, 0) + String(" mA"));
    lcd.setCursor(8, 3);
    lcd.print("     ");
    lcd.setCursor(8, 3);
    lcd.print(current_mA, 0);
    lcd.print("mA");
  }   

  lcd.setCursor(15, 3);
  lcd.print("    ");
  lcd.setCursor(15, 3);
  int percentage = ((loadvoltage-6.4)/2.0)*100;
  lcd.print(percentage);
  lcd.print("%"); 
  Blynk.virtualWrite(V9, percentage); 
  if (percentage >= 50){Blynk.setProperty(V9, "color", "#0C990C");}
  if (percentage> 20 && percentage < 50){Blynk.setProperty(V9, "color", "#d69e04");}  
  if (percentage <= 20){Blynk.setProperty(V9, "color", "#e80606");}
  
  lcd.setCursor(14, 2); 
  lcd.write(2);
  if(charging) {
    lcd.setCursor(15, 2);
    lcd.write(6);
  } else {
    lcd.setCursor(15, 2);
    lcd.write(7);
  }
  lcd.setCursor(17, 2);
  if(Blynk.connected() && online == true) {
    lcd.write(5);
    if(disconnects > 0) {
      disconnects = 0;
      lcd.setCursor(18, 2);
      lcd.print("  ");  
    }
  } else {
    lcd.print("X");
    if(online) {
      check_connection();
    }
  }

  Blynk.virtualWrite(V0, aq);
  Blynk.virtualWrite(V1, p25);
  Blynk.virtualWrite(V2, p10);
  Blynk.virtualWrite(V3, t);
  Blynk.virtualWrite(V4, h);
  Blynk.virtualWrite(V5, loadvoltage);
  Blynk.virtualWrite(V7, solar_volt); 
  Blynk.virtualWrite(V16, moisture); 
}

void lightfunc() {
  if(light && dark){ //light is the signal to turn light on, dark is the state of darkness
    digitalWrite(RLY1, LOW);
    digitalWrite(RLY2, LOW);
    digitalWrite(RELAY_LED, HIGH);
    relay_counter = currentMillis;
    dark = 0;
    led3.on();
  }

  if(Reedstate) {
    relay_counter = currentMillis;
  }

  if(currentMillis - relay_counter >= 90000 || !light) {
      digitalWrite(RLY1, HIGH);
      digitalWrite(RLY2, HIGH);
      digitalWrite(RELAY_LED, LOW);
      dark = 1;
      light = 0;
      led3.off();
  }
}

int read_adc(int adc_parameter) {
  int sum = 0;
  int sample ;
  for (int i=0; i<AVG_NUM; i++) {                                        
    sample = analogRead(adc_parameter);      
    sum += sample;                        
    delayMicroseconds(50);                
  }
  return(sum / AVG_NUM);                
}

void thingspeak_update(void) {
  ThingSpeak.setField(1, aq);
  ThingSpeak.setField(2, t);
  ThingSpeak.setField(3, h);
  ThingSpeak.setField(4, loadvoltage);
  ThingSpeak.setField(5, moisture);
  ThingSpeak.setField(7, p25);
  ThingSpeak.setField(8, p10);
  if(charging) {
 // ThingSpeak.setField(5, solar_volt);
    ThingSpeak.setField(6, solar_current);    
  }
  ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
}

void check_connection (void) {
  if (currentMillis - connection_time >= 5000) {
    disconnects++;
    lcd.setCursor(18, 2);
    lcd.print("  ");
    lcd.setCursor(18, 2);
    lcd.print(disconnects);
    Blynk.connect(3333);
    connection_time = currentMillis;
  } 
/*  if ((WiFi.status() == WL_CONNECTED) && (disconnects > 10)) {
    Blynk.begin(auth, wifi, ssid, pass);
  } */
  if ((WiFi.status() == WL_CONNECTED) && (disconnects > 20)) {
    tone(BUZZER, 450);
    delay(5000);
    noTone(BUZZER);
    resetFunc();
  }
}

BLYNK_WRITE(V10) {
  int pinValue = param.asInt();
  light = pinValue;
  if (pinValue == 1) {
    dark = 1;
    lightfunc();  
  } else if (pinValue == 0) {
    lightfunc();
  }
}

void charge_relay(void) {
  if (charger == 1 && (currentMillis - charge_timer <= 3600000)) {
    digitalWrite(PWR_MSFT, HIGH);
  } else {
    digitalWrite(PWR_MSFT, LOW);
    charger = 0;
  }
}

void check_pump(void){
  if(pump == 1 && (currentMillis - pump_time <= 60000)) {
    digitalWrite(MOTOR, HIGH);
    led4.on();
  } else {
    pump = 0;
    digitalWrite(MOTOR, LOW); 
    led4.off();
  }  
}

BLYNK_WRITE(V11) {
  int pinValue_2 = param.asInt();
  charger = pinValue_2;
  if(pinValue_2 == 1) {
    charge_timer = currentMillis;
  } else if (pinValue_2 == 0) {
    charge_relay();
  }
}

BLYNK_WRITE(V12) {
  int pinValue_4 = param.asInt();
  if(pinValue_4 == 1) {
    thingspeak_update();
    pinValue_4 = 0;
  }
}

BLYNK_WRITE(V13) {
  int pinValue_3 = param.asInt();
  if (pinValue_3 == 1) {
    tone(BUZZER, 450);
    delay(500);
    noTone(BUZZER);
    resetFunc();
  }
}

BLYNK_READ(V14) {
  int val = analogRead(TMP);
  float mv = (val/1023.0)*5000;
  float cel = mv/10;
  Blynk.virtualWrite(V14, cel); //sending to Blynk
}

BLYNK_WRITE(V15) {
  int pinValue_5 = param.asInt();
  pump = pinValue_5;
  if (pinValue_5 == 1) {
     pump_time = currentMillis;
  } else if (pinValue_5 == 0) {
    check_pump();
  }
}

BLYNK_WRITE(V17) {
  int pinValue_6 = param.asInt();
  if (pinValue_6 == 1) {
     get_sensor_data();
  }
}
