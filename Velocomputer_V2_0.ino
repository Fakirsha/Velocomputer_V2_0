/*                                                                  VeloPC V2.0 by Fakirsha_pa 2024 
КОД НЕДОПИЛЕН ДО КОНЦА, НЕКОТОРЫЕ ФУНКЦИИ РАБОТАЮТ НЕКОРРЕКТНО ПОЭТОМУ БЫЛИ ЗАКОММЕНТИРОВАНЫ, ПО ПОВОДУ БАГОВ, ЛОГИЧЕСКИХ ПРОБЛЕМ И ДРУГИХ НЕИСПРАВНОСТЯХ ПИСАТЬ МНЕ СЮДА: ТГ КАНАЛ @falivekirsha (чат канала), эл почта: VeloPCfeedback@gmail.com
*/

/* БИБЛИОТЕКИ */
#include <TFT.h>
#include <EEPROM.h>
#include <GyverPower.h>
#include <GyverButton.h>
#include <AHT10.h> 

/*
// на это положите болт, тут я пытаюсь температуру взять из процессора и у меня получается кал
#define T1_ADC    320 // Значение АЦП при температуре Т1
#define T2_ADC    352 // Значение АЦП при температуре Т2
#define T1_TEMP   26  // Фактическая температура Т1
#define T2_TEMP   63  // Фактическая температура Т2
*/

/* ПИНЫ ПОДКЛЮЧЕНИЯ */
#define TFT_CS     10       
#define TFT_RST    9             // Пины для подключения дисплея
#define TFT_DC     8  

#define BATTERY_PIN A0      // Пин аккумулятора
#define BTN_PIN   4         // Пин кнопки 
#define BTN_POWER   7       // Пин кнопки питания
#define HALL_SENSOR_PIN 2   // Пин для цифрового датчика Холла
/* ПИНЫ ПОДКЛЮЧЕНИЯ */

/* НАСТРОЙКА ПОРОГОВ И ДРУГИХ ЗНАЧЕНИЙ */
#define WHEEL_DIAMETER_INCHES 27.5  // Диаметр колеса в дюймах (ставьте ваш диаметр колеса в дюймах)
#define ReferenceVoltage 5.18       //
#define batteryLowVol 3.30          // Напряжение ниже которого срабатывает защита (рекомендую ставить примерно от 3.25 до 3.4 чтобы акб не сдох)
#define upperThreshold 4.10         // Верхний порог вольтажа (для циклов зарядки, после этого напряжения будет считатся что акб прошел полный цикл зарядки но см. ниже)
#define lowerThreshold 3.70         // Нижний порог вольтажа (если акб не разрядился до зарядки ниже этого напряжения то при зарядке цикл не прибавится, рекомендую ставить от 3.70 до 3.90)
#define LockButtSpd 25              // Срабатывание блокировки кнопок, скорость в км/ч(чтобы на сккорости не было ложных нажатий и переключений режимов)
#define MinBatteryVol 330           //
#define MaxBatteryVol 419           //
#define MinMidSpdCol 25             //
#define MaxSpdCol 40                //
#define PI 3.14159265359            // Значение числа Пи


/* ПЕРЕМЕННЫЕ */
volatile unsigned long lastTime;
volatile unsigned long currentTime;
volatile unsigned long deltaTime; 
volatile float wheelCircumference;
volatile float speedKPH;
volatile float LastSpeed;
volatile float LastMaxSpeed;
int MaxSpeed;
int pwm;
float humidity;
float temperature;
volatile float SyncDist;
int Voltage;
float batteryVoltage = 0;
int batteryPercent = 0;
float LastbatteryVoltage;
int LastbatteryPercent;
volatile float distance;

volatile float SaveRideDist;
volatile float SaveRideMaxSPD;
volatile float SaveRideMidSPD;

volatile float SaveRideDist1;
volatile float SaveRideMaxSPD1;
volatile float SaveRideMidSPD1;

volatile float SaveRideDist2;
volatile float SaveRideMaxSPD2;
volatile float SaveRideMidSPD2;

volatile float Dist = 0.0;
volatile float DistForAcc = 0.0;
volatile float DistAcc = 0.0;
volatile float dist15 = 0.0;
volatile float dist30 = 0.0;
volatile float dist40 = 0.0;
unsigned long lastOdoSaveTime = 0;
int EEPROM_ADDRESS = 1;
int Function = 1;
int LastFunction;
int BatteryColor;
int SpeedColor;
float averageSpeed = 0;
int brightness;
int8_t SyncOn = 0;
int8_t PrintText = 1;
int8_t OnlySpeed = 0;
int8_t i = 0;
int SaveTripint1 = 0;
int SaveTripint2 = 0;
int SaveTripint3 = 0;
int8_t tripsButt = 0;
int temp;
int8_t AccTime = 0;
bool ResultAcc = false;
int8_t startAccTime = 0;
bool measurementAcc = false;
bool results = false;
bool flag1 = true;
bool flag2 = true;
float elapsedTime = 0.0;
unsigned long previousMillis = 0;
unsigned long currentMillis;
bool batterychrgplus = 1;
unsigned long wheelrevolutions;
int chargeCycles;
bool hasDischarged = false;
bool servmode = 0;

/* ОСНОВНОЙ КОД */
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
GButton button(BTN_PIN);                                                 // Имена для экрана, кнопки и AHT10
GButton butpow(BTN_POWER);
AHT10 aht;

void setup() {
interrupts();
pinMode(A2, OUTPUT);
digitalWrite(A2, HIGH);
power.autoCalibrate(); // Калибровка питания
power.setSystemPrescaler(PRESCALER_1);
power.setSleepMode(POWERDOWN_SLEEP);
pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);   // Подтяжка пина датчика холла( Без подтяжки у вас будет скорость 3489567345678378678 км/ч )
aht.begin();                              // Инициализация датчика температуры и влажности воздуха
tft.initR(INITR_BLACKTAB);                // Инициализация дисплея
tft.invertDisplay(true);                  // Инициализация дисплея
tft.fillScreen(ST7735_BLACK);             // Инициализация дисплея
tft.setRotation(3);                       // Ориентация дисплея
attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, RISING);   // прерывание для счета скорости
Serial.begin(9600);

/* ЧТЕНИЕ ДАННЫХ ИЗ EEPROM (энергонезависимой памяти) */
EEPROM.get(EEPROM_ADDRESS, distance);
EEPROM.get(10, brightness);
EEPROM.get(60, chargeCycles);
EEPROM.get(65, wheelrevolutions);

EEPROM.get(15, SaveRideDist);
EEPROM.get(20, SaveRideMaxSPD);
EEPROM.get(25, SaveRideMidSPD);

EEPROM.get(30, SaveRideDist1);
EEPROM.get(35, SaveRideMaxSPD1);
EEPROM.get(40, SaveRideMidSPD1);

EEPROM.get(45, SaveRideDist2);
EEPROM.get(50, SaveRideMaxSPD2);
EEPROM.get(55, SaveRideMidSPD2);
/* ЧТЕНИЕ ДАННЫХ ИЗ EEPROM (энергонезависимой памяти) */

wheelCircumference = WHEEL_DIAMETER_INCHES * 0.0254 * PI;    // Рассчет длины окружности колеса в метрах

button.setType(LOW_PULL);     // Подключение кнопки, HIGH_PULL - Кнопка подключена к GND, LOW_PULL - Кнопка подключена к VIN
button.setDebounce(20);        // Антидребезг
button.setTimeout(500);       // Таймаут
button.setDirection(NORM_OPEN); // NORM_OPEN - Нормально открытая кнопка (кнопка без фиксации), NORM_CLOSE - Нормально закрытая кнопка (кнопка с фиксацией)
button.setTickMode(MANUAL);     // Нужно для кнопки, тут не нужно трогать
butpow.setType(HIGH_PULL);     // Подключение кнопки, HIGH_PULL - Кнопка подключена к GND, LOW_PULL - Кнопка подключена к VIN
butpow.setDebounce(70);        // Антидребезг
butpow.setTimeout(2000);       // Таймаут
butpow.setDirection(NORM_OPEN); // NORM_OPEN - Нормально открытая кнопка (кнопка без фиксации), NORM_CLOSE - Нормально закрытая кнопка (кнопка с фиксацией)
butpow.setTickMode(MANUAL);     // Нужно для кнопки, тут не нужно трогать

LastFunction = Function;

humidity = aht.readHumidity();          // Измеряем влажность
temperature = aht.readTemperature();    // Измеряем температуру
 
/* ЗАСТАВКА В НАЧАЛЕ И ИНИЦИАЛИЗАЦИЯ КОМПОНЕНТОВ */
tft.fillScreen(ST7735_BLACK);
tft.setCursor(1, 27);
tft.setTextColor(ST7735_WHITE);
tft.setTextSize(2);
tft.print("Velocomputer ");
delay(300);
tft.setCursor(1, 61);
tft.print("  initial");
tft.print("    ");
delay(350);
tft.print(".");
delay(40);
tft.print(".");
delay(40);
tft.print(".");
delay(40);
tft.print(".");
delay(40);
  
tft.print(".");
delay(40);
tft.print(".");
delay(40);
tft.print(".");
delay(40);
tft.print(".");
delay(40);
tft.print(".");
delay(40);
if(batteryVoltage < batteryLowVol) {
LowCharge();
}
tft.print(".");
delay(40);
tft.print(".");
delay(40);
tft.print(".");
delay(40);
tft.print(".");
delay(40);
if(button.isHold()) {
 servmode = 1;
}
tft.fillScreen(ST7735_BLACK);
tft.setCursor(1, 27);
tft.setTextColor(ST7735_WHITE);
tft.setTextSize(2);
tft.print("WheelDiameter  " + String(WHEEL_DIAMETER_INCHES) + "'");
delay(1300);
/* ЗАСТАВКА В НАЧАЛЕ И ИНИЦИАЛИЗАЦИЯ КОМПОНЕНТОВ */
}

void loop() {
if(AccTime == 0) {
  if(speedKPH > 35) {
    OnlySpeed = 1;
    brightness = 4;
 }
  else {
    brightness = 9;
    OnlySpeed = 0;
    Function = LastFunction;
  }
}
if(servmode == 0) {
if(startAccTime == 0) {
if(AccTime == 0 ) {
 if(i == 0) {
  if(OnlySpeed == 0) {
  if(Function == 1) {
  displaySpeed();   // Показываем скорость на дисплее
  } 

  if(Function == 2) {
  displayWheatherandbattery();   // Показываем погоду и заряд аккумулятора на дисплее
  } 

  if(Function == 3) {
  odometer();   // Показываем пройденную дистанцию за все время
  } 

  if(Function == 4) {
  LastRides();   // Показываем сохраненную поездку
  } 
}
  if(OnlySpeed == 1) {
    OnlySpeedKPH();   // Показываем только скорость увеличивая шрифт и яркость

  } 
}
}
}
}

  if(servmode == 1) {
    Service();
  }

  if(startAccTime == 1) {
    startAccelerationTime();
  }

  if(AccTime > 0) {
    startAccTime = 2;
    brightness = 4;
    RunAccTime();
  }

  if(SaveTripint1 == 1) {
    SaveTrip1();
  }

  if(SaveTripint2 == 1) {
    SaveTrip2();
  }

  if(SaveTripint3 == 1) {
    SaveTrip3();
  }

  if(batteryVoltage < batteryLowVol) {
   LowCharge();
  }

  if(batteryVoltage > 3.90){
    BatteryColor = ST7735_GREEN;
  }

  if(batteryVoltage < 3.90 ){
    BatteryColor = ST7735_MAGENTA;
  }

  if(batteryVoltage < 3.60){
    BatteryColor = ST7735_BLUE;
  }

  EEPROM.put(EEPROM_ADDRESS, distance);   // Запихиваем пройденную дистанцию в энергонезависимую память
  EEPROM.put(10, brightness);
  EEPROM.put(60, chargeCycles);
  EEPROM.put(65, wheelrevolutions);
  LastbatteryPercent = batteryPercent;
  LastFunction = Function;

  humidity = aht.readHumidity();          // Измеряем влажность
  temperature = aht.readTemperature();    // Измеряем температуру


  // Проверяем, достиг ли аккумулятор верхнего порога
    if (batteryVoltage >= upperThreshold && hasDischarged) {
        chargeCycles++;
        hasDischarged = false; // Сбрасываем флаг разряда
    }
    // Проверяем, достиг ли аккумулятор нижнего порога
    if (batteryVoltage <= lowerThreshold) {
        hasDischarged = true; // Устанавливаем флаг разряда
    }
    
    digitalWrite(A2, HIGH);
    delay(1000); // Задержка (в развитии)
}

void yield() {                  // Тут идет обновление кнопки и цвета скорости без задержек, иначе delay съест кнопку с функциями и они не будут работать

if(measurementAcc == true) {
   currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {
     previousMillis = currentMillis; // Сохранение времени последнего обновления
     elapsedTime += 0.01; // Прибавление 0.01 секунды к общему времени
  }
}

  if(speedKPH < LockButtSpd) {
  button.tick();
  butpow.tick();
  }

if(tripsButt == 1){
  if(button.isClick()) {
    SaveTripint1++;
  }

  if(button.isDouble()) {
    SaveTripint2++;
  }

  if(button.isTriple()) {
    SaveTripint3++;
  }
}

   if(butpow.isHold()) {
    tft.fillScreen(ST7735_BLACK);
    digitalWrite(3, LOW);
    speedKPH = 0;
    MaxSpeed = 0;
    Dist = 0;
    power.hardwareDisable(PWR_ALL);
    power.sleep(SLEEP_FOREVER); // спим
  }

  if(butpow.isDouble()) {
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(1, 27);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.print("Do you really reboot?");
    delay(3000);
    if(butpow.isClick()) {
    noInterrupts();
    asm volatile ("JMP 0x00");
    }
  }

  if(butpow.isTriple()) {
    tft.fillScreen(ST7735_BLACK);
    digitalWrite(3, LOW);
    power.hardwareDisable(PWR_ALL);
    power.sleep(SLEEP_FOREVER); // спим
  }

  if(SyncOn == 1) {
    SyncData();
  }

  if(SyncOn == 2) {
    SyncOn = 0;
  }

  temp = temperature;

  batteryVoltage = analogRead(BATTERY_PIN) * (ReferenceVoltage  / 1023); // Конвертируем входящие значения в вольты, значение 5.18 это напряжение на контактах +5V и GND, у вас оно может быть другое, измерьте и впишите значение
  Voltage = batteryVoltage * 100;
  batteryPercent = map(Voltage, MinBatteryVol, MaxBatteryVol, 0, 100);
  batteryPercent = constrain(batteryPercent, 0, 100);

  pwm = map(brightness, 5, 0, -425, -1023);
  analogWrite(3, pwm);

  if(Function > 4) {
    Function = 1;
  }

  if(brightness == 11) {
    brightness = 1;
  }

  if(speedKPH < MinMidSpdCol){
    SpeedColor = ST7735_GREEN;
  }

  if(speedKPH > MinMidSpdCol){
    SpeedColor = ST7735_MAGENTA;
  }

  if(speedKPH > MaxSpdCol){
    SpeedColor = ST7735_BLUE;
  }

}

void hallSensorISR() {

  //Пробуждение по прерыванию и включение периферии
  power.wakeUp();
  power.hardwareEnable(PWR_ALL);
  // Обновляем время и скорость при каждом обороте колеса
  lastTime = currentTime;
  currentTime = millis();
  deltaTime = currentTime - lastTime;
  speedKPH = (((wheelCircumference / deltaTime) * 3600) - 1); // Скорость в км/ч

  if(speedKPH < 0 ) {
    speedKPH = 0;
  }

  if(speedKPH > LastSpeed + 20) {
    speedKPH = LastSpeed;
  }
  
  if(speedKPH > MaxSpeed){
    MaxSpeed = speedKPH;
  }
  
  if(MaxSpeed > LastMaxSpeed + 20) {
    MaxSpeed = LastMaxSpeed;
  }

  unsigned long currentTime = millis();
  float currentSpeedKmH = (float(digitalRead(HALL_SENSOR_PIN)) * PI * WHEEL_DIAMETER_INCHES * 3.6) / 2.0;
  Dist += wheelCircumference / 1051.993355;  // Увеличение значения пройденного расстояния
  lastTime = currentTime;
  distance += wheelCircumference / 1051.993355;
  averageSpeed = (Dist / 0); // Средняя скорость
  LastSpeed = speedKPH;
  LastMaxSpeed = MaxSpeed;
  wheelrevolutions++;
}

void displaySpeed() {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(2);
  tft.print("Speed:");
  tft.setTextSize(3);
  tft.setTextColor(SpeedColor);
  tft.print(speedKPH, 0);                                  // Вывод показаний текущей и максимальной скорости, а также пройденной дистанции 
  tft.setCursor(1, 57);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_BLUE);
  tft.println("MaxSpeed:" + String(MaxSpeed));
  tft.setCursor(1, 85);
  tft.setTextColor(ST7735_WHITE);
  tft.print("Dist:" + String(Dist) + "Km");

  if(button.isClick()){
    Function++;
  }

  if(button.isHold()){
    startAccTime = 1;
  }

}

void OnlySpeedKPH() {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 44);
  tft.setTextSize(6);
  tft.setTextColor(ST7735_WHITE);
  tft.print(" ");
  tft.print(speedKPH, 0);   
}

void displayWheatherandbattery() {
  aht.begin();   // Инициализация датчика температуры и влажности воздуха
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(2);
  tft.print("Temp:" + String(temperature) + "C");                     // Вывод показаний температуры и влажности воздуха и заряд аккумулятора в процентах
  tft.setCursor(1, 57);
  tft.print("Humidity:" + String(humidity, 0) + "%");
  tft.setCursor(1, 85);
  tft.setTextColor(BatteryColor);
  tft.print("Battery:" + String(batteryPercent) + "%");
  
   if(button.isClick()){
    Function++;
  }

  if(button.isHold()){
    SyncOn++;
  }

}

void odometer() {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.print("All traveled: ");
  tft.setTextSize(3);
  tft.print(distance); 
  tft.setCursor(1, 85);
  tft.print("   km");

  if(button.isClick()){
    Function++;
  }

  if (button.isHold()) {
    distance = 0.0;                              // Сброс одометра, удерживаем кнопку
    EEPROM.put(EEPROM_ADDRESS, distance);
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 40);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_GREEN);
  tft.print("  Reset all      travel");
  delay(2000);
  }
       
}

void LastRides() {
  if(i == 0) {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.print("          SaveTrips");
  tft.setTextSize(2);
  tft.setCursor(1, 38);
  tft.println("SaveTrip1");
  tft.setCursor(1, 61);
  tft.print("SaveTrip2");
  tft.setCursor(1, 85);
  tft.print("SaveTrip3");
  
  if(button.isClick()){
    Function = 1;
  }
  }
  if(button.isHold()) {
   i++;
   tripsButt++;
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextSize(3);
  tft.setTextColor(ST7735_WHITE);
  tft.print("Choose a trip");
  }

}

void SaveTrip1() { 
  tripsButt--;
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.print("          SaveTrip1");
  tft.setTextSize(2);
  tft.setCursor(1, 38);
  tft.println("Dist " + String(SaveRideDist) + "Km");
  tft.setCursor(1, 61);
  tft.print("MaxSPD" + String(SaveRideMaxSPD, 0) + "Km/h");
  tft.setCursor(1, 85);
  tft.print("MidSPD" + String(SaveRideMidSPD, 0) + "Km/h");

  if(button.isHold()) {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextSize(3);
  tft.setTextColor(ST7735_GREEN);
  SaveRideDist = Dist;
  SaveRideMaxSPD = MaxSpeed;
  SaveRideMidSPD = averageSpeed;
  EEPROM.put(15, SaveRideDist);
  EEPROM.put(20, SaveRideMaxSPD);
  EEPROM.put(25, SaveRideMidSPD);
  tft.print("   Trip1   saved");
  delay(2000);
  }

  if(button.isTriple()) {
    i = 0;
    SaveTripint1 = 0;
    SaveTripint2 = 0;
    SaveTripint3 = 0;
    tripsButt = 0;
    Function = 4;
  }
}

void SaveTrip2() {
  tripsButt--;
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.print("          SaveTrip2");
  tft.setTextSize(2);
  tft.setCursor(1, 38);
  tft.println("Dist " + String(SaveRideDist1) + "Km");
  tft.setCursor(1, 61);
  tft.print("MaxSPD" + String(SaveRideMaxSPD1, 0) + "Km/h");
  tft.setCursor(1, 85);
  tft.print("MidSPD" + String(SaveRideMidSPD1, 0) + "Km/h");
  if(button.isHold()) {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextSize(3);
  tft.setTextColor(ST7735_GREEN);
  SaveRideDist1 = Dist;
  SaveRideMaxSPD1 = MaxSpeed;
  SaveRideMidSPD1 = averageSpeed;
  EEPROM.put(30, SaveRideDist1);
  EEPROM.put(35, SaveRideMaxSPD1);
  EEPROM.put(40, SaveRideMidSPD1);
  tft.print("   Trip2   saved");
  delay(2000);
  }

  if(button.isTriple()) {
    i = 0;
    SaveTripint1 = 0;
    SaveTripint2 = 0;
    SaveTripint3 = 0;
    tripsButt = 0;
    Function = 4;
  }
}

void SaveTrip3() { 
  tripsButt--;
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.print("          SaveTrip3");
  tft.setTextSize(2);
  tft.setCursor(1, 38);
  tft.println("Dist " + String(SaveRideDist2) + "Km");
  tft.setCursor(1, 61);
  tft.print("MaxSPD" + String(SaveRideMaxSPD2, 0) + "Km/h");
  tft.setCursor(1, 85);
  tft.print("MidSPD" + String(SaveRideMidSPD2, 0) + "Km/h");

  if(button.isHold()) {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextSize(3);
  tft.setTextColor(ST7735_GREEN);
  SaveRideDist2 = Dist;
  SaveRideMaxSPD2 = MaxSpeed;
  SaveRideMidSPD2 = averageSpeed;
  EEPROM.put(45, SaveRideDist2);
  EEPROM.put(50, SaveRideMaxSPD2);
  EEPROM.put(55, SaveRideMidSPD2);
  tft.print("   Trip3   saved");
  delay(2000);
  }

  if(button.isTriple()) {
    i = 0;
    SaveTripint1 = 0;
    SaveTripint2 = 0;
    SaveTripint3 = 0;
    tripsButt = 0;
    Function = 4;
  }
}

void startAccelerationTime() {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextSize(3);
  tft.setTextColor(ST7735_GREEN);
  tft.print("press   button  to start");
  if(button.isClick()) {
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(1, 44);
    tft.setTextSize(6);
    tft.setTextColor(ST7735_BLUE);
    tft.print("  3");
    delay(950);
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(1, 44);
    tft.setTextSize(6);
    tft.setTextColor(ST7735_MAGENTA);
    tft.print("  2");
    delay(950);
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(1, 44);
    tft.setTextSize(6);
    tft.setTextColor(ST7735_GREEN);
    tft.print("  1");
    delay(950);
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(10, 44);
    tft.setTextSize(4);
    tft.print("Start!");
    delay(600);
    elapsedTime = 0.00;
    DistForAcc = Dist;
    measurementAcc = true;
    AccTime++;
  }
}

void RunAccTime() {
  if(measurementAcc == true) {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 44);
  tft.setTextSize(6);
  tft.setTextColor(ST7735_WHITE);
  tft.print(" ");
  tft.print(speedKPH, 0);
  DistAcc = Dist - DistForAcc;

  if(speedKPH >= 15 && speedKPH <= 25 && flag1 == true) {
    dist15 = elapsedTime;
    flag1 = false;
  }
  if(speedKPH >= 30 && speedKPH <= 35 && flag2 == true) {
    dist30 = elapsedTime;
    flag2 = false;
  }
  if(speedKPH >= 40) {
    dist40 = elapsedTime;
    measurementAcc = false;
    ResultAcc = true;
  }
}
  if(ResultAcc == true) {
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(1, 27);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.print("Time:");
    tft.print(elapsedTime);
    tft.setTextSize(1);
    tft.print("s");
    tft.setTextSize(2);
    tft.setCursor(1, 57);
    tft.print("TotDist:");
    tft.print(DistAcc);
    tft.setTextSize(1);
    tft.print("km");
    if(button.isClick()) {
      ResultAcc = false;
      results = true;
    }
  }
  if(results == true) {
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(1, 27);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.print("1-15:");
    tft.print(String(dist15) + "S");
    tft.setCursor(1, 57);
    tft.print("1-30:");
    tft.print(String(dist30) + "S");
    tft.setCursor(1, 85);
    tft.print("1-40:");
    tft.print(String(dist40) + "S");
    if(button.isClick()){
      elapsedTime = 0.0;
      results = false;
      AccTime = 0;
      dist15 = 0.0;
      dist30 = 0.0;
      dist40 = 0.0;
      startAccTime = 0;
      Function = LastFunction;
      flag1 = true;
      flag2 = true;
    }
  }
}

void SyncData () {
  Serial.setTimeout(25);
  while (!Serial) {
    ;  // для старых юсб 
  } 
  if(PrintText == 1) {
    Serial.println("Введите общее расстояние в киломерах.метрах ( Пример : 50.67 )");
    PrintText--;
    }
    if(Serial.available()) {
    SyncDist =  Serial.parseFloat();
    distance = SyncDist;
    EEPROM.put(EEPROM_ADDRESS, distance);
    delay(50);
    Serial.println("Общее расстояние  : " + String(distance) + " км");
    SyncOn = 0;
    PrintText++;
    }
  }

void LowCharge() {
   tft.fillScreen(ST7735_BLACK);
   tft.setCursor(1, 27);
   tft.setTextColor(ST7735_BLUE);
   tft.setTextSize(3);
   tft.print("    Low   Charge");
   tft.setTextSize(2);
   tft.setCursor(1, 27);
   tft.print(batteryPercent);
   tft.print("%");
   tft.setTextSize(1);
   tft.setCursor(30, 85);
    tft.setTextColor(ST7735_GREEN);
   tft.print("Charge the battery");
   delay(5000);
   tft.fillScreen(ST7735_BLACK);
    digitalWrite(3, LOW);
    speedKPH = 0;
    MaxSpeed = 0;
    Dist = 0;
    power.hardwareDisable(PWR_ALL);
    power.sleep(SLEEP_FOREVER); // спим
}

void Service() {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(1, 27);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.print("       Service mode");
  tft.setCursor(1, 35);
  tft.print("Batt");
  tft.setCursor(1, 45);
  tft.print(String(batteryPercent) + "%");
  tft.setCursor(1, 55);
  tft.print(String(batteryVoltage) + "V");
  if(batteryVoltage > 3.70) {
    tft.setCursor(1, 65);
    tft.print("charged");
  }
  else {
    tft.setCursor(1, 65);
    tft.print("discharged");
  }
  tft.setCursor(1, 75);
  tft.print("loop chrg:" + String(chargeCycles));
  tft.setCursor(1, 85);
  tft.print("wheelrev:" + String(wheelrevolutions));
  if(button.isDouble()) {
    servmode = 0;
  }
}
/*
// на это положите болт, тут я пытаюсь температуру взять из процессора и у меня получается кал
int16_t getInternalTemp(void) {
  ADMUX = (INTERNAL << 6) | 0b1000; // Опорное: INTERNAL, вход: THERM.
  ADCSRA = (1 << ADEN) | 0b111;     // АЦП включен, мин. скорость
  delayMicroseconds(100);           // Ждем стабилизации после смены опорного
  // 3 холостых прохода АЦП, прогнать мусор
  for (uint8_t i = 0; i < 3; i++) {
    ADCSRA |= (1 << ADSC);          // Запускаем
    while (ADCSRA & (1 << ADSC));   // Ждем
    volatile uint16_t a = ADC;      // Холостое чтение
  }

  uint32_t result = 0;
  // Основное чтение АЦП, 4 раза с усреднением
  for (uint8_t i = 0; i < 4; i++) {
    ADCSRA |= (1 << ADSC);          // Запускаем
    while (ADCSRA & (1 << ADSC));   // Ждем
    result += ADC;                  // Читаем, аккумулируем
  } result /= 4;                    // Делим на кол-во итераций

  return map(result, T1_ADC, T2_ADC, T1_TEMP, T2_TEMP);
}
*/
void wakeUp() {
  power.wakeUp();
  power.hardwareEnable(PWR_ALL);
}