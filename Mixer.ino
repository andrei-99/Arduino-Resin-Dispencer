#include <GyverHX711.h>
#include <GyverFilters.h>
#include <TimerMs.h>
#include <EncButton.h>
#include <LiquidCrystal_I2C_Hangul.h>

GyverHX711 sensor(6, 7, HX_GAIN128_A);
LiquidCrystal_I2C_Hangul lcd(0x27, 16, 2);  // LCD — адрес , столбцов, строк
//TimerMs wyt(dt, 0, 1); //инициализация таймера dt - уст время, 0 остановлен, 1 - режим "таймер" // (период, мсек ), (0 не запущен / 1 запущен), (режим: 0 период / 1 таймер)
EncButton eb(3, 2, 4); // энкодер на 2-3-4 пины
Button btn5(5); // кнопка "старт" на 5 пин
//пропорции А:Б в гр по умолч 100:40 гр
uint8_t PropA = 100, PropB = 40;
//масса на вых по умолч = 70 гр + компоненты А и Б
uint16_t MassA = 500, MassB = 200;
uint16_t MassTotal = MassA + MassB;
//масса с весов стартовая по умолчанию
int32_t MassC = 0;
int32_t MassTemp = 0;

//Псевдомасса сырая с АЦП датчика,
int32_t MassRaw = 0;
//Сдвиг сырой массы по таре к нулю
uint16_t MassOffset = 0;
//Коэффициент преобразования сырой массы в граммы
uint8_t MassMult = 1;
// таймер для опроса датчика
uint16_t tmr, tmr1;

//Флаг "Наливается?" 
bool isFilling = false;

// Быстрый фильтр
FastFilter filteredMass(29, 100);

void setup() {
//выводы для клапанов А и Б
pinMode(11, OUTPUT);
digitalWrite(11, LOW);
pinMode(12, OUTPUT);
digitalWrite(12, LOW);

  // init lcd and backlite    
lcd.init();
lcd.backlight();
  
  Serial.begin(115200);  // запуск порта на 115200
// Serial.println("MassRAW, MassF"); // вывод названия графиков
        LcdPP();
        if (sensor.available()) {    
    MassC = (sensor.read()); // чтение массы

}
// Исходные начения для фильтра
filteredMass.setK(29);
filteredMass.setRaw(1000);
filteredMass.setFil(0);

}

void loop() {
  GetMass();   
btn5.tick();
    if (btn5.press()) {
      Serial.println("Button press"); //вывод в отладку
      fill_timer(); //вызов подпрограммы наполнения "полуавтомат"
      }
eb.tick();
    if (eb.turn()) {
      switch (eb.pressing()) {
        case false:
// меняем значение общей массы если простой поворот
            MassTotal += 10 * eb.dir();
            Mass();
            LcdPP();
         break;
            
         case true:
// меняем значение пропорции если поворот с нажатием
            PropB += 10 * eb.dir();
            Mass();
            LcdPP();
          break;
      }
   }


   uint16_t ms = millis();
   if (ms - tmr > 1000) {
    tmr = ms;
      lcd.setCursor(12,1);
      lcd.print("     "); 
      lcd.setCursor(12,1);
      lcd.print(String(MassC)); 
     Serial.println(String(MassC));
    //  Serial.println();
   
}



}

void LcdPP() {
      lcd.clear();
      lcd.home();
      lcd.print("A:B " + String(PropA) + ":" + String(PropB));
      lcd.setCursor(11,0);
      lcd.print(String(MassTotal) + "g.");
      lcd.setCursor(0,1);
      lcd.print("A:" + String(MassA) + " B:" + String(MassB));
      lcd.setCursor(12,1);
      lcd.print("     "); 
      lcd.setCursor(12,1);
      lcd.print(String(MassC)); 
     
}
void Mass() {
            //считаем массу B
            MassB = (MassTotal * PropB) / (PropA + PropB);
            //считаем массу A
            MassA = (MassTotal * PropA) / (PropA + PropB);
            // Serial.println("MassTotal:" + String(MassTotal) + " A:" + String(MassA) + " B:" + String(MassB) + " A:" + String(PropA) + " B:" + String(PropB));
}

void GetMass() {
// считывание с тензодатчика   
    if (sensor.available()) {
       // чтение сырой массы с АЦП 
      MassRaw = (sensor.read());
    }
    //Serial.print(String(MassC));
    //Serial.println();    
    //Serial.print(String(MassC));
    //Serial.print(',');
// нормировка показаний к нулю
  MassC = (MassRaw - MassOffset);
// перевод показаний в граммы
  MassC = (MassC / MassMult);
}

void  fill_timer() {
  GetMass();
  if (MassC > 100) return;

while (MassC < MassA) {
  GetMass();
  if (MassC < 10) break;
  digitalWrite(11, HIGH);
  uint16_t ms = millis();
   if (ms - tmr > 250) {
    tmr = ms;
      LcdPP();
   }
}
digitalWrite(11, LOW);
while (MassC < MassTotal) {
 GetMass();
  if (MassC < 10) break;
  digitalWrite(12, HIGH);
  uint16_t ms = millis();
   if (ms - tmr > 250) {
    tmr = ms;
      LcdPP();
   }
digitalWrite(12, LOW);
}
}