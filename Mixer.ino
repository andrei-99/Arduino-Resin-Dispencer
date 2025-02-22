#include <GyverHX711.h>
#include <GyverFilters.h>
#include <EncButton.h>
#include <GyverTM1637.h>
#define CLK 6
#define DIO 7

GyverTM1637 disp(CLK, DIO);
EncButton eb(3, 2, 4); // энкодер на 2-3-4 пины
Button btn5(5); // кнопка "старт" на 5 пин
// Быстрый фильтр
FastFilter filteredMass(29, 100); // коэффициент 0-31, dt период фильтрации в миллисекундах
//пропорции А:Б в гр по умолч 100:40 гр
uint8_t PropA = 100, PropB = 40;
//масса компонентов по умолч = 280 гр + компоненты А и Б
uint16_t MassA = 200, MassB = 80;
//масса смеси в итоге
uint16_t MassTotal = MassA + MassB;
//масса текущая в гр. стартовая по умолчанию
int32_t MassC = 0;

//Масса сырая с АЦП датчика,
int32_t MassRaw = 0;
//Сдвиг сырой массы по таре к нулю начальное значение.
uint16_t MassOffset = 4000;
//Коэффициент преобразования сырой массы в граммы
uint8_t MassMult = 146;
// таймер для опроса датчика
uint16_t tmr, tmr1;

//Флаг "Наливается?" 
bool isFilling = false;

void setup() {
//Подготовка выводов для клапанов А и Б
pinMode(11, OUTPUT);
digitalWrite(11, LOW);
pinMode(12, OUTPUT);
digitalWrite(12, LOW);

//инициализация дисплея    
disp.clear();
disp.brightness(7);  // яркость, 0 - 7 (минимум - максимум)
for uint8_t i 
uint8_t ms = millis();
   if (ms - tmr > 1000) {
    tmr = ms;
    LcdPP();   
  }

if (sensor.available()) {
    MassRaw = (sensor.read()); //Первое чтение массы
  }
//Первые показания в фильтр 
filteredMass.setRaw(MassRaw);
}



void loop() {

btn5.tick();
    if (btn5.press()) {
      fill_timer(); //вызов подпрограммы наполнения "полуавтомат"
      }

eb.tick();
    if (eb.turn()) {
            MassTotal += 10 * eb.dir();
            Mass(); //вызов подрограммы расчета компонентов 
            LcdPP(); //обновление дисплея
         break;
      }
   }

//асинхронно обновляем дисплей каждую секунду на всякий случай
uint16_t ms = millis();
   if (ms - tmr > 1000) {
    tmr = ms;
    LcdPP();   
  }

//асинхронно читаем массу
uint16_t ms1 = millis();
   if (ms1 - tmr > 50) {
    tmr = ms1;
    GetMass();   
  }
}

//подпрограмма вывод на дисплей
void LcdPP() {
disp.displayInt(MassTotal)
}


//подпрограмма расчета массы компонентов А и Б по заданной общей
void Mass() {
            //считаем массу B
            MassB = (MassTotal * PropB) / (PropA + PropB);
            //считаем массу A
            MassA = (MassTotal * PropA) / (PropA + PropB);
}
//подпрограмма снятия массы с датчика и перевода в граммы, возвращает MassC в граммах

void GetMass() {
// считывание с тензодатчика   
    if (sensor.available()) {
       // чтение сырой массы с АЦП 
FilterdMass.setRaw(sensor.read());
FilterdMass.compute();
    }
// нормировка показаний к нулю
  MassC = (FilteredMass.getFil() - MassOffset);
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