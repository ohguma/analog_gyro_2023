/**
   Analog出力ジャイロ
   2023-12-09

   D5がLOWなら、アナログ出力を0-1/2[Vref]でかえす。

   AvrCopter_6050_328p_4726_without_oled の改良版
   2019-05-12
   参考
   https://github.com/rpicopter/ArduinoMotionSensorExample

   Analog出力ジャイロのキャリブレーションと合体
   2021-12-27 by ohguma
   センサーの使い方(ジャイロ編)
   http://blog.livedoor.jp/revolution_include/archives/2815979.html

   ▼パーツ
   DAC          MCP4726 秋月
   ジャイロセンサ MPU6050 Amazonなど
   OLEDなし

   ▼ピン設定
   入力  基準向きリセットSW   A3
   出力  向き表示LED1,2,3    D11,D12,D13
*/

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT(x) Serial.print(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

#include <EEPROM.h>
#include <Wire.h>
#include "freeram.h"
#include "mpu.h"
#include "I2Cdev.h" //Fastwireを使うため自前のI2Cdev.hを使う。
//MP6050の重複をさける
#define I2CDEVLIB_MPU6050_TYPEDEF
//自前のI2Cdev.hを使うために、自前の.hと.cppを使う。
#include "MPU6050_6Axis_MotionApps20.h"

//DACアドレス
#define MCP4726_ADDR 0x60
//PIN設定
//状態LED
#define pinLedRight 13
#define pinLedCenter 12
#define pinLedLeft 11
//基準向きリセットSW
#define pinBtn A3
//DAC出力電圧のハーフ化設定ピン
#define pinHalf 5
//方向リセットを始めるスイッチ長押し時間[ms]
#define MS_RESET 50
//キャリブレーションを始めるスイッチ長押し時間[ms]
#define MS_CALIB 3000
//キャリブレーション時のオフセット値を保存するEEPROMアドレス
#define POS_GYRO_X  0x00
#define POS_GYRO_Y  0x02
#define POS_GYRO_Z  0x04
#define POS_ACCEL_Z 0x06

//長押し用タイマー変数。押された瞬間のmillis()を保存する
unsigned long tm_reset = 0;
unsigned long tm_calib = 0;


float myinitdeg[3] = {0, 0, 0};
float mydeg[3] = {0, 0, 0};
long past = 0;
int ret;
bool is_half = false;

void setup()
{
  Fastwire::setup(400, 0);
  Serial.begin(9600);
  //ピン設定
  pinMode(pinLedRight, OUTPUT);
  pinMode(pinLedCenter, OUTPUT);
  pinMode(pinLedLeft, OUTPUT);
  pinMode(pinBtn, INPUT_PULLUP);
  pinMode(pinHalf, INPUT_PULLUP);
  if (digitalRead(pinHalf) == LOW) {
    is_half = true;
  }
  //出力初期化
  digitalWrite(pinLedRight, LOW);
  digitalWrite(pinLedCenter, LOW);
  digitalWrite(pinLedLeft, LOW);
  //起動サイン
  Serial.println(F("Analog OUTPUT Gyro"));
  Serial.println(F("Ver.2023-12-09"));
  if (is_half) {
    Serial.println(F("Half mode(D5 connect GND).Center:1/4[Vref]"));
  } else {
    Serial.println(F("Not half mode(D5 not connect GND). Center:1/2[Vref]"));
  }
  Serial.print("Free mem: ");
  Serial.println(freeRam());
  //起動サイン(LED)
  digitalWrite(pinLedLeft, HIGH);
  delay(50);
  digitalWrite(pinLedLeft, LOW);
  digitalWrite(pinLedCenter, HIGH);
  delay(50);
  digitalWrite(pinLedCenter, LOW);
  digitalWrite(pinLedRight, HIGH);
  delay(50);
  digitalWrite(pinLedRight, LOW);
  //ジャイロセンサ初期化
  init_gyro();
  ret = mympu_open(200);
  Serial.print(F("MPU init: "));
  Serial.println(ret);
  //基準向きセット
  attachOfset();
}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

void loop()
{
  if (digitalRead(pinBtn) == HIGH) {
    //キーが押されていないデフォルト状態
    tm_reset = 0;
    tm_calib = 0;
    //ジャイロ値更新
    ret = mympu_update();
    switch (ret) {
      case 0: c++; break;
      case 1: np++; return;
      case 2: err_o++; return;
      case 3: err_c++; return;
      default:
        digitalWrite(pinLedLeft, HIGH);
        digitalWrite(pinLedCenter, HIGH);
        digitalWrite(pinLedRight, HIGH);
        Serial.print("READ ERR");
        Serial.println(ret);
        return;
    }
    for (int i = 0; i < 3; i++) {
      mydeg[i] = mympu.ypr[i] - myinitdeg[i];
      if (mydeg[i] < -180) {
        mydeg[i] += 360;
      } else if (mydeg[i] > 180) {
        mydeg[i] -= 360;
      }
    }
    //for DAC
    int val = 0;
    int vmax = 4095;
    if (is_half) {
      vmax = 2047;
    }
    val = constrain(map(mydeg[0], -180, 180, vmax, 0), 0, vmax); //-180～180のデータを0～4095(2^12-1)のデータに換算する
    Wire.beginTransmission(MCP4726_ADDR);
    Wire.write((uint8_t) ((val >> 8) & 0x0F));   // MSB: (D11, D10, D9, D8)
    Wire.write((uint8_t) (val));  // LSB: (D7, D6, D5, D4, D3, D2, D1, D0)
    Wire.endTransmission();

    if (mydeg[0] >= -3.0 && mydeg[0] <= -1.0 ) {
      digitalWrite(pinLedLeft, HIGH);
    } else {
      digitalWrite(pinLedLeft, LOW);
    }
    if (mydeg[0] >= -1.0 && mydeg[0] <= 1.0 ) {
      digitalWrite(pinLedCenter, HIGH);
    } else {
      digitalWrite(pinLedCenter, LOW);
    }
    if (mydeg[0] >= 1.0 && mydeg[0] <= 3.0 ) {
      digitalWrite(pinLedRight, HIGH);
    } else {
      digitalWrite(pinLedRight, LOW);
    }
  } else if (tm_reset == 0 && tm_calib == 0) {
    //スイッチ押し時間計測開始
    tm_reset = millis();
    tm_calib = millis();
  } else if (tm_reset > 0 && millis() - tm_reset > MS_RESET) {
    //ローカル角度のリセット（現時点の向きを基準向きとする）
    tm_reset = 0;
    attachOfset();

  } else if (tm_calib > 0 && millis() - tm_calib > MS_CALIB) {
    //キャリブレーション開始
    tm_calib = 0;
    digitalWrite(pinLedRight, HIGH);
    digitalWrite(pinLedCenter, HIGH);
    digitalWrite(pinLedLeft, HIGH);
    action_calibration();
    digitalWrite(pinLedRight, LOW);
    digitalWrite(pinLedCenter, LOW);
    digitalWrite(pinLedLeft, LOW);
    //基準向きセット
    attachOfset();
  }
}

void attachOfset()
{
  for (int i = 0; i < 3; i++) {
    myinitdeg[i] = mympu.ypr[i];
  }
  past = millis();
}


//＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
//ジャイロセンサ角度計算用
MPU6050_6Axis_MotionApps20 mpu;
int16_t gyro = 0;             //ジャイロセンサ角度
static uint8_t mpuIntStatus;
static bool dmpReady = false; // set true if DMP init was successful
static uint16_t packetSize;   // expected DMP packet size (default is 42 bytes)

//ジャイロセンサ初期化
void init_gyro()
{
  int val;
  mpu.initialize();
  while (!mpu.testConnection()) {
  }
  if (mpu.testConnection() != true) {
    DEBUG_PRINTLN("MPU disconection");
    while (true) {}
  }
  if (mpu.dmpInitialize() != 0) {
    DEBUG_PRINTLN("MPU break");
    while (true) {}
  }
  EEPROM.get(POS_GYRO_X, val);
  mpu.setXGyroOffset(val);
  EEPROM.get(POS_GYRO_Y, val);
  mpu.setYGyroOffset(val);
  EEPROM.get(POS_GYRO_Z, val);
  mpu.setZGyroOffset(val);
  EEPROM.get(POS_ACCEL_Z, val);
  mpu.setZAccelOffset(val);
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
  //少し待つ
  delay(100);
}

//＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
// 以下は長押し時のキャリブレーション用
MPU6050_Base accelgyro;
int state = 0;
int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

//キャリブレーション処理
void action_calibration()
{
  state = 0;
  DEBUG_PRINTLN("\nMPU6050 Calibration");
  //delay(2000);
  DEBUG_PRINTLN("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  //delay(3000);
  // verify connection
  DEBUG_PRINTLN(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);
  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.
  accelgyro.initialize();
  while (1) {
    if (state == 0) {
      DEBUG_PRINTLN("\nReading sensors for first time...");
      meansensors();
      state++;
      delay(1000);
    }
    if (state == 1) {
      DEBUG_PRINTLN("\nCalculating offsets...");
      calibration();
      state++;
      delay(1000);
    }
    if (state == 2) {
      meansensors();
      DEBUG_PRINTLN("\nFINISHED!");
      DEBUG_PRINT("\nSensor readings with offsets:\t");
      DEBUG_PRINT(mean_ax);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(mean_ay);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(mean_az);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(mean_gx);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(mean_gy);
      DEBUG_PRINT("\t");
      DEBUG_PRINTLN(mean_gz);
      DEBUG_PRINT("Your offsets:\t");
      DEBUG_PRINT(ax_offset);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(ay_offset);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(az_offset);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(gx_offset);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(gy_offset);
      DEBUG_PRINT("\t");
      DEBUG_PRINTLN(gz_offset);
      DEBUG_PRINTLN("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
      EEPROM.put(POS_GYRO_X, gx_offset);
      EEPROM.put(POS_GYRO_Y, gy_offset);
      EEPROM.put(POS_GYRO_Z, gz_offset);
      EEPROM.put(POS_ACCEL_Z, az_offset);
      return;
    }
  }
}

void meansensors()
{
  long i = 0;
  long buff_ax = 0, buff_ay = 0, buff_az = 0;
  long buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration()
{
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);
    meansensors();
    DEBUG_PRINTLN("...");
    if (abs(mean_ax) <= acel_deadzone) {
      ready++;
    } else {
      ax_offset = ax_offset - mean_ax / acel_deadzone;
    }
    if (abs(mean_ay) <= acel_deadzone) {
      ready++;
    } else {
      ay_offset = ay_offset - mean_ay / acel_deadzone;
    }
    if (abs(16384 - mean_az) <= acel_deadzone) {
      ready++;
    } else {
      az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
    }
    if (abs(mean_gx) <= giro_deadzone) {
      ready++;
    } else {
      gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);
    }
    if (abs(mean_gy) <= giro_deadzone) {
      ready++;
    } else {
      gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);
    }
    if (abs(mean_gz) <= giro_deadzone) {
      ready++;
    } else {
      gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);
    }
    if (ready == 6) break;
  }
}
