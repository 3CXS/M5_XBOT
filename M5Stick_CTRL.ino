/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////
M5STICK-CPLUS
DISPLAY LCD 135 x 240
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
#include <M5StickCPlus.h>
#include "AXP192.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA 0
#define I2C_SCL 26

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

TFT_eSprite Sprite1 = TFT_eSprite(&M5.Lcd);
TFT_eSprite Sprite2 = TFT_eSprite(&M5.Lcd); 

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////*/

int batcapacity = 390; // battery capacity mAh
bool BtnA_State = false ;
bool BtnB_State = false ;
int color[] = {0xfd79, 0xe8e4, 0xfbe4, 0xff80, 0x2589, 0x51d, 0x3a59, 0xa254, 0x7bef, 0x00, 0xffff};
            /* 0-PINK 1-RED 2-ORANGE 3-YELLOW 4-GREEN 5-BLUE 6-DBLUE 7-VIOLET 8-GREY 9-BLACK 10-WHITE */

long Inter1 = 240;
unsigned long PrevTime1 = millis();
long Inter2 = 500;
unsigned long PrevTime2 = millis();

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;
        
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void setup() {
  M5.begin();
  M5.Axp.EnableCoulombcounter();
  M5.Axp.ScreenBreath(9);
  M5.Lcd.fillScreen(BLACK); 
  M5.Imu.Init();

  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

  
}

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void loop() {
  CTRL(); 
  IMU();  
  BAT();
  SERVO();
}

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void SERVO() {
  // Drive each servo one at a time using setPWM()
  //Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);

  servonum++;
  if (servonum > 7) servonum = 0; // Testing the first 8 servo channels
}


void IMU() {
  unsigned long CurTime1 = millis();
  Sprite1.createSprite(135, 160);
  Sprite1.pushSprite(4, 52);
  if(CurTime1 - PrevTime1 > Inter1) {
    PrevTime1 = CurTime1;
    M5.IMU.getAhrsData(&pitch,&roll,&yaw);
    Sprite1.setCursor(6, 16, 2);
    Sprite1.printf("%04.0f  %04.0f  %04.0f", pitch, roll, yaw);
    static float temp = 0;
    M5.IMU.getTempData(&temp);
    Sprite1.setCursor(60, 120, 2);
    Sprite1.printf("%.1f", temp);
    Sprite1.drawCircle(90, 121, 2, color[10]);
    Sprite1.setCursor(95, 120, 2);
    Sprite1.print("C");
  }
}

void CTRL() { 
  if(M5.Axp.GetBtnPress() == 0x02) {  // 0x01 long press(1s), 0x02 press
  esp_restart();
  } 
  if(M5.BtnA.wasPressed()) {
    if (BtnA_State == false) {
      M5.Axp.SetLDO2(false); // close tft voltage output
      BtnA_State = true;
    }
    else {
      M5.Axp.SetLDO2(true); 
      BtnA_State = false;
    }
  }
  else if(M5.BtnB.wasPressed()){
    if (BtnB_State == false) {
      BtnB_State = true;
    }
    else {
      BtnB_State = false;
    }
  }
  M5.update(); 
} 

 void BAT() {
  unsigned long CurTime2 = millis();
  Sprite2.createSprite(135, 32);
  Sprite2.pushSprite(4, 18);
  if(CurTime2 - PrevTime2 > Inter2) {
    PrevTime2 = CurTime2;
    int coulomb;
    int batlevel;
    int statecolor;
    coulomb = abs(M5.Axp.GetCoulombData());
    batlevel = round(map(coulomb, 0, batcapacity, 100, 0 ));  
    M5.Lcd.setCursor(92, 2, 2);
    M5.Lcd.printf("%3u", batlevel);
    M5.Lcd.setCursor(118, 2, 2);
    M5.Lcd.print((char) 37);
    if ( M5.Axp.GetBatCurrent() > 0) {
      statecolor = color[7];
      BtnB_State = true;
      Sprite2.setCursor(0, 16, 2);   
      Sprite2.printf("USB %.1f", M5.Axp.GetVBusVoltage());
      Sprite2.setCursor(54, 16, 2); 
      Sprite2.print("V");
      Sprite2.setCursor(81, 16, 2); 
      Sprite2.printf("%4.0f", M5.Axp.GetVBusCurrent());
      Sprite2.setCursor(112, 16, 2);
      Sprite2.print("mA");
    }
    else {
      Sprite2.fillSprite(BLACK); 
      if (batlevel < 25) {
        statecolor = color[1];
      }
      else {
      statecolor = color[10];
      }
    }
    M5.Lcd.drawRect(53, 4, 32, 12, WHITE);
    M5.Lcd.fillRect(56, 7, map(batlevel, 0, 100, 0, 24)+2, 6, statecolor);
    //M5.Axp.SetCoulombClear();
    if (BtnB_State == true) {
      Sprite2.setCursor(0, 0, 2); 
      Sprite2.printf("%.0f", M5.Axp.GetTempInAXP192());
      Sprite2.drawCircle(20, 3, 2, color[10]);
      Sprite2.setCursor(25, 0, 2);
      Sprite2.print("C");
      Sprite2.setCursor(48, 0, 2); 
      Sprite2.printf("%04u", coulomb);
      Sprite2.setCursor(82, 0, 2);
      Sprite2.printf("%4.0f", M5.Axp.GetBatCurrent());
      Sprite2.setCursor(112, 0, 2);
      Sprite2.print("mA"); 
      //Sprite1.printf("BAT %.1f V", M5.Axp.GetBatVoltage()); 
    }
    else {
      Sprite2.fillSprite(BLACK); 
    }
  }  
}
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////
  pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);

  pwm.setPWMFreq(1000)

  pwm.setPWM(15, 1024, 3072)

  void IIC_Servo_Init() {           //sda  0   scl  26    
     Wire.begin(0, 26, 100000UL);
  }


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define I2C_SDA 33
#define I2C_SCL 32

#define SEALEVELPRESSURE_HPA (1013.25)

TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;

unsigned long delayTime;

void setup() {
  Serial.begin(115200);
  Serial.println(F("BME280 test"));
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76, &I2CBME);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;

  Serial.println();
}

void loop() { 
  printValues();
  delay(delayTime);
}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *      d, i     signed int
 *      u        unsigned int
 *      ld, li   signed long
 *      lu       unsigned long
 *      f        double
 *      c        char
 *      s        string
 *      %        '%'
///////////////////////////////////////////////////////////////////////////////////////////////////////////*/
