/*
  SumoRobot UTC 2023
  Stated: 30/03/2023
  LastUpdate: 3AM 17/04/2023

  An, Trung, Bình
*/

#include <math.h>
#include <Arduino.h>

#include <MPU6050_light.h>
#include <VL53L0X.h>

#include <Servo.h>
Servo motorL;
Servo motorR;

#include <Wire.h>
#include <SPI.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Setup MPU class
MPU6050 mpu(Wire);
long timer1 = 0;

long a = 0;

// #include <reading.h>

// Khởi tạo theo thư viện
VL53L0X FL;
VL53L0X FF;
VL53L0X FR;
VL53L0X RR;
VL53L0X BR;
VL53L0X BL;
VL53L0X RL;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// set the pins to shutdown for all 7 sensors

#define SHT_L0X_FL 22 // XSHUT Pin laser 1
#define SHT_L0X_FF 24 // XSHUT Pin laser 2
#define SHT_L0X_FR 26 // XSHUT Pin laser 3
#define SHT_L0X_RR 28 // XSHUT Pin laser 4
#define SHT_L0X_BR 30 // XSHUT Pin laser 5
#define SHT_L0X_BL 32 // XSHUT Pin laser 6
#define SHT_L0X_RL 34 // XSHUT Pin laser 7

#define IR_FL 3  // Chân kết nối IR sensor FL
#define IR_FR 2  // Chân kết nối IR sensor FR
#define IR_BR 19 // Chân kết nối IR sensor BR
#define IR_BL 18 // Chân kết nối IR sensor BL

#define LPWM1 4 // Chân kết nối đến chân IN1 của động cơ A
#define RPWM1 5 // Chân kết nối đến chân IN2 của động cơ A
#define LPWM2 6 // Chân kết nối đến chân IN3 của động cơ B
#define RPWM2 7 // Chân kết nối đến chân IN4 của động cơ B

#define PWM_L 8 // Chân kết nối đến chân Enable của động cơ A
#define PWM_R 9 // Chân kết nối đến chân Enable của động cơ B

#define analogKey A9

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// address we will assign for all 7 sensor
#define L0X_FL_ADDRESS 0x28
#define L0X_FF_ADDRESS 0x30
#define L0X_FR_ADDRESS 0x31
#define L0X_RR_ADDRESS 0x32
#define L0X_BR_ADDRESS 0x33
#define L0X_BL_ADDRESS 0x34
#define L0X_RL_ADDRESS 0x35

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// cờ interrupt
boolean flagInterrupt0 = false;
boolean flagInterrupt1 = false;
boolean flagInterrupt4 = false;
boolean flagInterrupt5 = false;

int count = 0;

int sensorValue[8]; // Mảng giá trị cảm biến
int mark[8];        // Mảng đánh dấu số thứ tự của laser khi sắp xếp

// ADKeyboard Module
int adc_key_val[5] = {30, 70, 110, 150, 600};
int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;
int keyValue; // lưu nút đã bấm

void (*func)(int);  // con trỏ trỏ đến hàm tương ứng với nút đã bấm
void (*plan)(void); // con trỏ trỏ đến hàm chiến thuật tương ứng với nút đã bấm

void (*resetFunc)(void) = 0; // declare reset function at address 0

void scanI2CAddress()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(500); // wait 5 seconds for next scan
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ESC(int speedL, int speedR)
{
  if (speedL < 0)
  {
    speedL = map(abs(speedL), 0, 100, 90, 0);
  }
  else if (speedL > 0)
  {
    speedL = map(speedL, 0, 100, 110, 180);
  }
  else if (speedL == 0)
  {
    speedL = 100;
  }

  if (speedR < 0)
  {
    speedR = map(abs(speedR), 0, 100, 90, 0);
  }
  else if (speedR > 0)
  {
    speedR = map(speedR, 0, 100, 110, 180);
  }
  else if (speedR == 0)
  {
    speedR = 100;
  }

  motorL.write(speedL);
  motorR.write(speedR);

  Serial.print(speedL);
  Serial.print("      ");
  Serial.println(speedR);
}

// Xuất xung điều khiển động cơ: Truyền giá trị dương đi tiến, giá trị âm đi lùi
void PWM(int pwmL, int pwmR)
{
  // Serial.println(pwmL);
  // Serial.println(pwmR);

  if (pwmL > 0) // L tiến
  {
    analogWrite(LPWM1, pwmL);
    analogWrite(RPWM1, 0);
  }
  else if (pwmL < 0) // L lùi
  {
    analogWrite(LPWM1, 0);
    analogWrite(RPWM1, abs(pwmL));
  }
  else if (pwmL == 0) // L đứng yên
  {
    analogWrite(LPWM1, 0);
    analogWrite(RPWM1, 0);
  }

  if (pwmR > 0) // R tiến
  {
    analogWrite(LPWM2, pwmR);
    analogWrite(RPWM2, 0);
  }
  else if (pwmR < 0) // R lùi
  {
    analogWrite(LPWM2, 0);
    analogWrite(RPWM2, abs(pwmR));
  }
  else if (pwmR == 0) // R đứng yên
  {
    analogWrite(LPWM2, 0);
    analogWrite(RPWM2, 0);
  }
}

// Hàm khi kích hoạt ngắt sẽ treo cho đến khi thoát hiểm thành công
void standby()
{
  int time = 700; // tạo biến để chỉnh thời gian chạy thoát hiểm cho dễ

  // treo
  while (flagInterrupt0 && millis() - timer1 < time)
    ;

  while (flagInterrupt1 && millis() - timer1 < time)
    ;

  while (flagInterrupt4 && millis() - timer1 < time)
    ;
  while (flagInterrupt5 && millis() - timer1 < time)
    ;

  // reset hết cờ
  flagInterrupt0 = false;
  flagInterrupt1 = false;
  flagInterrupt4 = false;
  flagInterrupt5 = false;

  // dừng
  // PWM(0, 0);
  // Serial.println(a++);
}

// FrontLeft
void Interrupt_0()
{
  PWM(-150, -150);       // thoát hiểm ngay
  flagInterrupt0 = true; // bật cờ
  timer1 = millis();     // lấy mốc thời gian lúc bắt được thoát hiểm

  // tắt các cờ khác để luôn chỉ có 1 thoát hiểm
  flagInterrupt1 = false;
  flagInterrupt4 = false;
  flagInterrupt5 = false;
  // standby();
}

// FrontRight
void Interrupt_1()
{
  PWM(-150, -150);
  flagInterrupt1 = true;
  timer1 = millis();

  flagInterrupt0 = false;
  flagInterrupt4 = false;
  flagInterrupt5 = false;
  // standby();
}

// BackLeft
void Interrupt_4()
{
  PWM(150, 150);
  flagInterrupt4 = true;
  timer1 = millis();

  flagInterrupt0 = false;
  flagInterrupt1 = false;
  flagInterrupt5 = false;
  // standby();
}

// BackRight
void Interrupt_5()
{
  PWM(150, 150);
  flagInterrupt5 = true;
  timer1 = millis();

  flagInterrupt0 = false;
  flagInterrupt1 = false;
  flagInterrupt4 = false;
  // standby();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Convert ADC value to key number
int get_key(unsigned int inputValue)
{
  int k;
  for (k = 0; k < NUM_KEYS; k++)
  {
    if (inputValue < adc_key_val[k])
    {
      return k;
    }
  }
  if (k >= NUM_KEYS)
    k = -1; // No valid key pressed
  return k;
}

// Đọc nút cho đến khi phát hiện bấm nút
int readKeyLoop()
{
  boolean flagPressButton = false;
  adc_key_in = analogRead(0); // read the value from the sensor pin A0
  key = get_key(adc_key_in);  // convert into key press

  if (key != oldkey) // if keypress is detected
  {
    delay(50);                  // wait for debounce time
    adc_key_in = analogRead(0); // read the value from the sensor
    key = get_key(adc_key_in);  // convert into key press
    if (key != oldkey)
    {
      oldkey = key;
      if (key >= 0)
      {
        switch (key)
        {
        case 0:
          keyValue = 0;
          break;
        case 1:
          keyValue = 1;
          break;
        case 2:
          keyValue = 2;
          break;
        case 3:
          keyValue = 3;
          break;
        case 4:
          keyValue = 4;
          break;
        }

        flagPressButton = true;
      }
    }
  }

  if (!flagPressButton) // chưa bấm
  {
    readKeyLoop();
  }
  else
  {
    return 0; // thoát hàm
  }
}

// Đọc nút reset
int readResetKey()
{
  adc_key_in = analogRead(0); // read the value from the sensor pin A0
  key = get_key(adc_key_in);  // convert into key press

  if (key != oldkey) // if keypress is detected
  {
    delay(25);                  // wait for debounce time
    adc_key_in = analogRead(0); // read the value from the sensor
    key = get_key(adc_key_in);  // convert into key press
    if (key != oldkey)
    {
      oldkey = key;
      if (key >= 0)
      {
        switch (key)
        {
        case 0:
          keyValue = 0;
          break;
        case 1:
          keyValue = 1;
          break;
        case 2:
          keyValue = 2;
          break;
        case 3:
          keyValue = 3;
          break;
        case 4:
          keyValue = 4;
          break;
        }

        if (keyValue == 1) // Nút B
        {
          resetFunc(); // Lệnh reset
        }
      }
    }
  }
}

// Trả về giá trị góc quay theo trục Z
float readMPU()
{
  // int i = 1;
  float zAngle = 0;

  mpu.update();

  zAngle = mpu.getAngleZ();
  return zAngle;
}

void setupL298()
{
  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM2, OUTPUT);
}

// Hàm này giống hệt readLaserSensor(), khác mỗi cái có ghi ra serial để xem giá trị
void readLaserTest()
{
  int i = 0;

  sensorValue[++i] = FL.readRangeContinuousMillimeters();
  Serial.print(sensorValue[i]);

  Serial.print("   ");

  sensorValue[++i] = FF.readRangeContinuousMillimeters();
  Serial.print(sensorValue[i]);

  Serial.print("   ");

  sensorValue[++i] = FR.readRangeContinuousMillimeters();
  Serial.print(sensorValue[i]);

  Serial.print("   ");

  sensorValue[++i] = RR.readRangeContinuousMillimeters();
  Serial.print(sensorValue[i]);

  Serial.print("   ");

  sensorValue[++i] = BR.readRangeContinuousMillimeters();
  Serial.print(sensorValue[i]);

  Serial.print("   ");

  sensorValue[++i] = BL.readRangeContinuousMillimeters();
  Serial.print(sensorValue[i]);

  Serial.print("   ");

  sensorValue[++i] = RL.readRangeContinuousMillimeters();
  Serial.print(sensorValue[i]);

  Serial.println("   ");
}

// Đọc laser (không ghi ra serial)
void readLaserSensor()
{
  int i = 0;

  // dùng ++i khù khoằm này để copy patse cho nhanh, đỡ phải gõ
  sensorValue[++i] = FL.readRangeContinuousMillimeters();

  sensorValue[++i] = FF.readRangeContinuousMillimeters();

  sensorValue[++i] = FR.readRangeContinuousMillimeters();

  sensorValue[++i] = RR.readRangeContinuousMillimeters();

  sensorValue[++i] = BR.readRangeContinuousMillimeters();

  sensorValue[++i] = BL.readRangeContinuousMillimeters();

  sensorValue[++i] = RL.readRangeContinuousMillimeters();
}

void setupLCD()
{
  lcd.init();
  lcd.backlight();
}

// truyền xâu vào đây để ghi lên LCD
void printLCD(String s)
{
  lcd.setCursor(0, 0);
  lcd.print(s);
}

// bật từng laser lên để cài địa chỉ bằng chân XSHUT
void setupLaserSensor()
{
  int i;
  pinMode(SHT_L0X_FL, OUTPUT);
  pinMode(SHT_L0X_FF, OUTPUT);
  pinMode(SHT_L0X_FR, OUTPUT);
  pinMode(SHT_L0X_RR, OUTPUT);
  pinMode(SHT_L0X_BR, OUTPUT);
  pinMode(SHT_L0X_BL, OUTPUT);
  pinMode(SHT_L0X_RL, OUTPUT);

  Serial.println(i++);
  // Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_L0X_FL, LOW);
  digitalWrite(SHT_L0X_FF, LOW);
  digitalWrite(SHT_L0X_FR, LOW);
  digitalWrite(SHT_L0X_RR, LOW);
  digitalWrite(SHT_L0X_BR, LOW);
  digitalWrite(SHT_L0X_BL, LOW);
  digitalWrite(SHT_L0X_RL, LOW);

  Serial.println(i++);

  // all unreset
  digitalWrite(SHT_L0X_FL, HIGH);
  digitalWrite(SHT_L0X_FF, HIGH);
  digitalWrite(SHT_L0X_FR, HIGH);
  digitalWrite(SHT_L0X_RR, HIGH);
  digitalWrite(SHT_L0X_BR, HIGH);
  digitalWrite(SHT_L0X_BL, HIGH);
  digitalWrite(SHT_L0X_RL, HIGH);
  delay(10);

  Serial.println(i++);

  // activating L0X_L and reseting L0X_F
  digitalWrite(SHT_L0X_FL, HIGH);
  digitalWrite(SHT_L0X_FF, LOW);
  digitalWrite(SHT_L0X_FR, LOW);
  digitalWrite(SHT_L0X_RR, LOW);
  digitalWrite(SHT_L0X_BR, LOW);
  digitalWrite(SHT_L0X_BL, LOW);
  digitalWrite(SHT_L0X_RL, LOW);

  Serial.println(i++);

  delay(10);
  FL.setAddress(L0X_FL_ADDRESS);

  Serial.println(i++);

  digitalWrite(SHT_L0X_FF, HIGH);
  delay(10);
  FF.setAddress(L0X_FF_ADDRESS);

  Serial.println(i++);

  digitalWrite(SHT_L0X_FR, HIGH);
  delay(10);
  FR.setAddress(L0X_FR_ADDRESS);

  Serial.println(i++);

  digitalWrite(SHT_L0X_RR, HIGH);
  delay(10);
  RR.setAddress(L0X_RR_ADDRESS);

  Serial.println(i++);

  digitalWrite(SHT_L0X_BR, HIGH);
  delay(10);
  BR.setAddress(L0X_BR_ADDRESS);

  Serial.println(i++);

  digitalWrite(SHT_L0X_BL, HIGH);
  delay(10);
  BL.setAddress(L0X_BL_ADDRESS);

  Serial.println(i++);

  digitalWrite(SHT_L0X_RL, HIGH);
  delay(10);
  RL.setAddress(L0X_RL_ADDRESS);

  Serial.println(i++);

  delay(10);

  FL.init();
  FF.init();
  FR.init();
  RR.init();
  BR.init();
  BL.init();
  RL.init();

  Serial.println(i++);

  // Gắn cờ lỗi
  FL.setTimeout(500);
  FF.setTimeout(500);
  FR.setTimeout(500);
  RR.setTimeout(500);
  BR.setTimeout(500);
  BL.setTimeout(500);
  RL.setTimeout(500);

  Serial.println(i++);

  // // reduce timing budget to 20 ms (default is about 33 ms)
  // FL.setMeasurementTimingBudget(20);
  // FF.setMeasurementTimingBudget(20);
  // FR.setMeasurementTimingBudget(20);
  // RR.setMeasurementTimingBudget(20);
  // BR.setMeasurementTimingBudget(20);
  // BL.setMeasurementTimingBudget(20);
  // RL.setMeasurementTimingBudget(20);

  FL.startContinuous(10);
  FF.startContinuous(10);
  FR.startContinuous(10);
  RR.startContinuous(10);
  BR.startContinuous(10);
  BL.startContinuous(10);
  RL.startContinuous(10);

  Serial.println(i++);

  // scanI2CAddress();
}
void setupMPU()
{
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
  {
  } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true); // gyro and accelero
  Serial.println("Done!\n");
}

void setupIR()
{
  attachInterrupt(digitalPinToInterrupt(IR_FL), Interrupt_0, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_FR), Interrupt_1, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_BR), Interrupt_4, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_BL), Interrupt_5, RISING);
}

// đổi giá trị 2 biến cho nhau sử dụng con trỏ
void swap(int *pointer1, int *pointer2)
{
  int x;
  x = *pointer1;
  *pointer1 = *pointer2;
  *pointer2 = x;
}

// sắp xếp mảng giá trị đo từ 7 laser
void sort()
{
  // int array[8];
  boolean haveSwap = false; // khởi tạo biến kiểm tra sự đổi chỗ

  // readLaserSensor();

  for (int i = 1; i < 8; i++)
  {
    mark[i] = i;
  }

  for (int i = 1; i < 7; i++) // sắp xếp sủi bọt
  {
    haveSwap = false;
    for (int j = 1; j < 8 - i; j++)
    {
      if (sensorValue[j] > sensorValue[j + 1])
      {
        swap(&sensorValue[j], &sensorValue[j + 1]);
        swap(&mark[j], &mark[j + 1]);
        haveSwap = true; // đánh dấu đã đổi trong lần lặp này
      }
    }

    if (!haveSwap) // đã đổi, thoát vòng lặp lần này (vòng lặp biến i)
    {
      break;
    }
  }

  // đoạn từ đây đến hết hàm dùng để check thuật toán sắp xếp, hàng 1 là giá trị sensor, hàng 2 là số thứ tự của sensor

  // for (int i = 1; i < 8; i++)
  // {
  //   Serial.print(sensorValue[i]);
  //   Serial.print("    ");
  // }

  // Serial.println();

  // for (int i = 1; i < 8; i++)
  // {
  //   Serial.print(mark[i]);
  //   Serial.print("    ");
  // }

  // Serial.println();
  // Serial.println();
  // delay(300);
}

int minSensorValue()
{
  // delay(100);
  uint32_t minValue = 100000;
  uint16_t upperBlock = 500; // Chặn trên khoảng cách đo được của laser
  uint8_t markedSensor = 0;

  readLaserSensor();

  for (uint8_t i = 1; i <= 7; i++)
  {
    if (sensorValue[i] < minValue)
    {
      minValue = sensorValue[i];
      markedSensor = i;
    }
  }

  if (minValue < upperBlock)
  {
    return markedSensor;
  }
  else
  {
    return 0;
  }
}

// trả về số thứ tự của sensor phát hiện gần nhất
int searchNearest()
{
  const int upperBlock = 600;

  readLaserSensor();
  sort();
  // Serial.print(mark[1]);
  // Serial.print("    ");
  // Serial.println(sensor[1]);

  // check chặn trên
  if (sensorValue[1] < upperBlock) //<- thay đổi chặn trên thay vào số ở đây
  {
    return mark[1];
  }
  else
  {
    return 0;
  }
}

void search1_An(int target)
{
  int t = 5;

  if (target == 0) // Không tìm thấy
  {
    // PWM(255, -255); // Quay tại chỗ để tìm
    Serial.println("None");
    // printLCD("None        ");
    delay(t);
  }
  if (target == 2) // Laser FF
  {
    PWM(255, 255); // Đâm bằng đầu xe
    Serial.println("FF");
    // printLCD("FF              ");
    delay(t);
  }
  if (target == 1) // Laser FL
  {
    PWM(-50, 50); // Quay trái
    Serial.println("FL");
    // printLCD("FL                ");
    delay(t);
  }
  if (target == 3) // Laser FR
  {
    PWM(50, -50); // Quay phải
    Serial.println("FR");
    // printLCD("FR                    ");
    delay(t);
  }
  if (target == 4) // Laser RR
  {
    PWM(-50, 50); // Quay trai
    Serial.println("RR");
    // printLCD("RR                        ");
    delay(t);
  }
  if (target == 7) // Laser RL
  {
    PWM(70, -70); // Quay phải
    Serial.println("RL");
    // printLCD("RL                        ");
    delay(t);
  }
  if (target == 5 || target == 6) // Laser BR or BL
  {
    PWM(-255, -255); // Đâm bằng đuôi xe
    Serial.println("BR or BL");
    // printLCD("BR or BL            ");
    delay(t);
  }
}

void search2_An(int target)
{
  int t = 5;

  if (target == 0) // Không tìm thấy
  {
    PWM(-50, 50);
    // PWM(255, -255); // Quay tại chỗ để tìm
    Serial.println("None");
    // printLCD("None        ");
    delay(t);
  }
  if (target == 2) // Laser FF
  {
    // PWM(255, 255); // Đâm bằng đầu xe
    Serial.println("FF");
    // printLCD("FF              ");
    if (sensorValue[target] < 100)
    {
      PWM(255, 255);
    }
    else
    {
      PWM(120, 120);
    }
    delay(t);
  }
  if (target == 1) // Laser FL
  {
    // PWM(-50, 50); // Quay trái
    Serial.println("FL");
    // printLCD("FL                ");
    if (sensorValue[target] < 30)
    {
      PWM(-255, 255);
    }
    else
    {
      PWM(-50, 50);
    }
    delay(t);
  }
  if (target == 3) // Laser FR
  {
    // PWM(50, -50); // Quay phải
    Serial.println("FR");
    // printLCD("FR                    ");
    if (sensorValue[target] < 30)
    {
      PWM(255, -255);
    }
    else
    {
      PWM(50, -50);
    }
    delay(t);
  }
  if (target == 4) // Laser RR
  {
    // PWM(-50, 50); // Quay trai
    Serial.println("RR");
    // printLCD("RR                        ");
    if (sensorValue[target] < 35)
    {
      PWM(-255, 255);
    }
    else
    {
      PWM(-60, 60);
    }
    delay(t);
  }
  if (target == 7) // Laser RL
  {
    // PWM(70, -70); // Quay phải
    Serial.println("RL");
    // printLCD("RL                        ");
    if (sensorValue[target] < 35)
    {
      PWM(255, -255);
    }
    else
    {
      PWM(60, -60);
    }
    delay(t);
  }
  if (target == 5 || target == 6) // Laser BR or BL
  {
    // PWM(-255, -255); // Đâm bằng đuôi xe
    Serial.println("BR or BL");
    // printLCD("BR or BL            ");
    if (sensorValue[target] < 100)
    {
      PWM(-255, -255); // Đâm bằng đầu xe
    }
    else
    {
      PWM(-120, -120);
    }
    delay(t);
  }
}

void search_Trung(uint8_t target)
{

  switch (target)
  {
  case 0:
    PWM(-255, 255);
    printLCD("0");
    break;
    ///////////////
  case 1:
    PWM(-200, 200);
    printLCD("1");
    break;
    ///////////////
  case 2:
    PWM(255, 255);
    printLCD("2");
    break;
    ///////////////
  case 3:
    PWM(200, -200);
    printLCD("3");
    break;
    ///////////////
  case 4:
    PWM(255, -255);
    printLCD("4");
    break;
    ///////////////
  case 5:
    PWM(255, -255);
    printLCD("5");
    break;
    ///////////////
  case 6:
    PWM(-255, 255);
    printLCD("6");
    break;
    ///////////////
  case 7:
    PWM(-255, 255);
    printLCD("7");
    break;

    // default:
    //   PWM(150, 80);
    //   printLCD("d");
    //   break;
  }
}

// nút C
void plan1()
{
}

// nút D
void plan2()
{
  PWM(80, 160);
  delay(500);

  // PWM(0, 0);
}

// nút E
void plan3()
{
  PWM(160, 120);
  delay(500);

  // PWM(0, 0);
}

void plan4()
{

 PWM(200, 100);
 delay(200);
 PWM(80, 200);
 delay(300);
 PWM(0,0);
}

////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  setupL298();
  PWM(0, 0);

  pinMode(13, OUTPUT);
  digitalWrite(13, 0);

  // setupMPU();

  // scanI2CAddress();

  setupLaserSensor();

  // scanI2CAddress();

  setupLCD();
  printLCD("Standby");

  // setupIR();

  readKeyLoop(); // đọc nút
  switch (keyValue)
  {
  case 0:
    func = &search2_An;
    plan = &plan4;
    break;
  case 2: // nút C
    func = &search2_An;
    plan = &plan1;
    // printLCD("Select AN");
    break;
  case 3: // nút D
    func = &search2_An;
    plan = &plan2;
    break;
  case 4:
    func = &search2_An;
    plan = &plan3;
    break;
  }

  digitalWrite(13, 1);
  delay(2500); // Theo luật thi đấu, cơ mà giờ đang test nên không cần
  // printLCD("              ");
  // setupIR();

  plan();
  // PWM(0, 0);
}

void loop()
{
  standby();

  func(minSensorValue()); // Gọi hàm được trỏ

  readResetKey(); // Quét nút
}